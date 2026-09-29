"""实时指标曲线卡 —— 行为照 `litearm-studio` 的 `MetricsPanel`，版式照设计稿。

## 行为出处（`studio/src/components/MetricsPanel.tsx` + `lib/arm/useArmMetrics.ts`）

| 事 | studio 怎么做 |
| --- | --- |
| 数据集 | 固定四路：温度 `°C` / 速度 `rad/s` / 力矩 `Nm` / 跟踪误差 `rad`（`useArmMetrics.ts:49-54`） |
| 采样 | **定时器 100 ms（10 Hz）**，不是每帧推（`:56,147`）—— 注释明说故意不依赖帧引用 |
| 窗口 | **100 点 = 10 s**（`:57`），超出 `splice(0, len-100)` 滑窗 |
| 缓冲 | **一个共享缓冲**，每拍同时存四路通道（`SeriesSample = {t, temp[], dq[], tau[], err[]}`，`:9-15`） |
| 切指标 | **不切缓冲、不清空** —— 只是换读哪个字段（`metricDatasets.ts:5,30`） |
| 暂停 | 停止采样；**缓冲原样冻结**（`:119`），恢复后从当前时刻继续、不补采样 |
| 全选 | 勾选**当前全部轴**（不是写死 0..6）（`:210`） |
| 清空 | **只取消勾选**，不清缓冲（`:211`）—— 重新勾上能看到完整历史 |
| 断连 | 清空缓冲（`:109-116`）；重连时先清一次（`:123-127`） |
| 缺数据 | 补 `null` 形成**断线**，**不补 0**；读数显示 `—`（em dash）而不是 0 |
| 曲线 | 一关节一条，取色 `JOINT_COLORS[i % 7]`；`borderWidth 1.5`、`tension 0.3`、无数据点圆 |
| y 轴 | 自动范围；**仅温度**有 `suggestedMin: 0`；无 `beginAtZero` |
| 网格 | y 网格开 `rgba(128,138,150,.16)`；**x 网格关**；x 底线开 `rgba(128,138,150,.3)` |
| 刻度 | y 上限 5 条、x 上限 6 条且不旋转；色 `#9aa6b6`、10px |
| 无数据态 | **图框保留**，只叠加说明文字（`MetricsPanel.tsx:187-192` 测试钉住） |

## ⚠ 与 studio 的两处**有意**不同

1. **版式**：设计稿把图例做成**右栏竖排**、每行直接带当前读数；studio 是横排纯开关芯片、
   读数另起一行。按"布局照设计稿、行为照 studio"的裁决，这里取设计稿版式。
2. **`err`（跟踪误差）本机没有这个通道**：studio 在实机上也**故意留空、不伪造**
   （`useArmMetrics.ts:137` 写死 `err: []`，`noData: real && m.id === 'err'`）。
   本仓同理 —— `arm_worker.Snapshot` 里根本没收主臂 q，所以显示"无数据"而不是画零线。
   ⚠ 这是**如实呈现**，不是没做完：要让它有数据得先把主臂 q 接进 `Snapshot`。
"""
from __future__ import annotations

import math
import time
from collections import deque
from typing import Deque, List, Optional, Sequence, Tuple

from PyQt5 import QtCore, QtGui, QtWidgets

from .cards import Badge, Card, SegmentedControl
from .theme import C, FONT_MONO, FONT_SANS, RADIUS, joint_color, mono, sans
from ..wire import N_JOINTS

__all__ = [
    "METRIC_DEFS", "DECIMALS", "MetricSeries", "MetricsChart", "MetricsCard",
]

#: 四路数据集 —— 逐条抄 `useArmMetrics.ts:49-54`。
#: `(id, 中文名, 单位, y 轴标签)`。⚠ studio 里**没有**任何 per-dataset 的 min/max 常量，
#: 唯一的范围干预是温度 `suggestedMin: 0` —— 别去编"温度 0~80"这种表。
METRIC_DEFS: List[Tuple[str, str, str, str]] = [
    ("temp", "温度", "°C", "T (°C)"),
    ("dq", "速度", "rad/s", "dq (rad/s)"),
    ("tau", "力矩", "Nm", "tau (Nm)"),
    ("err", "跟踪误差", "rad", "e (rad)"),
]

#: 逐指标小数位（`useArmMetrics.ts:77`）。
DECIMALS = {"temp": 0, "dq": 2, "tau": 1, "err": 3}

#: 采样间隔与窗口 —— `useArmMetrics.ts:56-57`。
SERIES_INTERVAL_MS = 100
SERIES_MAX_LEN = 100

#: 实机不提供的通道（`useArmMetrics.ts:137,185`）。本仓同理，见模块 docstring。
NO_DATA_METRICS = {"err"}


def _fmt(v: Optional[float], key: str) -> str:
    """按指标的小数位格式化；缺失给 `—`（**不是 0**）。"""
    if v is None:
        return "—"
    d = DECIMALS.get(key, 2)
    return f"{v:.{d}f}"


def _nice_ticks(lo: float, hi: float, want: int = 5) -> List[float]:
    """把 `[lo, hi]` 摊成 `want` 条左右的"整"刻度（1/2/5×10^k）。

    照 chart.js 的 `maxTicksLimit: 5` 口径 —— 只是上限，所以取最接近的档。
    """
    if not math.isfinite(lo) or not math.isfinite(hi) or hi <= lo:
        return [lo, hi] if math.isfinite(lo) else []
    raw = (hi - lo) / max(1, want - 1)
    mag = 10 ** math.floor(math.log10(raw))
    for m in (1, 2, 2.5, 5, 10):
        if raw <= m * mag:
            step = m * mag
            break
    else:
        step = 10 * mag
    start = math.floor(lo / step) * step
    end = math.ceil(hi / step) * step
    n = int(round((end - start) / step)) + 1
    if n < 2 or n > 12:                      # 兜底：步长选得太小就别用这套刻度
        return [lo, hi]
    # 按步长的量级取整，免得出现 0.30000000000000004 这种刻度
    dec = max(0, -int(math.floor(math.log10(step))) + 1)
    return [round(start + k * step, dec) for k in range(n)]


class MetricSeries:
    """**一个共享的**滚动缓冲，每拍四路通道一起存（照 studio 的 `SeriesSample`）。

    ⚠ 切指标**不动**这个缓冲 —— 四路数据始终同帧积累，切换只换读哪一路。
    """

    def __init__(self, maxlen: int = SERIES_MAX_LEN):
        self._buf: Deque[dict] = deque(maxlen=maxlen)

    def __len__(self) -> int:
        return len(self._buf)

    def clear(self) -> None:
        self._buf.clear()

    def append(self, t: float, temp: Sequence[float], dq: Sequence[float],
               tau: Sequence[float], err: Sequence[float]) -> None:
        self._buf.append({"t": t, "temp": list(temp), "dq": list(dq),
                          "tau": list(tau), "err": list(err)})

    def samples(self) -> List[dict]:
        return list(self._buf)

    def channel(self, key: str) -> List[Sequence[float]]:
        return [s[key] for s in self._buf]

    def times(self) -> List[float]:
        return [s["t"] for s in self._buf]


class MetricsChart(QtWidgets.QWidget):
    """自绘折线图（QPainter，**不引新依赖**）。

    ⚠ 不补 0：通道缺值画成**断线**（照 `metricDatasets.ts:30` 的 `?? null`）。
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(210)
        self._times: List[float] = []
        self._series: List[List[Optional[float]]] = []    # 每轴一条
        self._colors: List[QtGui.QColor] = []
        self._unit = ""
        self._ylabel = ""
        self._notice: Optional[str] = None

    def set_data(self, times: Sequence[float], series: Sequence[Sequence[Optional[float]]],
                 colors: Sequence[str], unit: str, ylabel: str,
                 notice: Optional[str] = None) -> None:
        self._times = list(times)
        self._series = [list(s) for s in series]
        self._colors = [QtGui.QColor(c) for c in colors]
        self._unit = unit
        self._ylabel = ylabel
        self._notice = notice
        self.update()

    # ── 绘制 ──
    def paintEvent(self, _ev) -> None:
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing, True)
        r = self.rect().adjusted(0, 4, -1, -1)

        left, right, top, bottom = 46, 6, 8, 24
        plot = QtCore.QRectF(r.left() + left, r.top() + top,
                             max(10, r.width() - left - right),
                             max(10, r.height() - top - bottom))

        vals = [v for s in self._series for v in s if v is not None]
        if vals:
            lo, hi = min(vals), max(vals)
        else:
            lo, hi = 0.0, 1.0
        is_temp = self._ylabel.startswith("T ")
        if is_temp:
            lo = min(0.0, lo)                      # 温度 suggestedMin: 0
        if hi - lo < 1e-9:
            # 数据全等（臂静止时 dq/tau 就是恒 0）⇒ 照 **chart.js 的 linear scale** 那套
            # 给一个偏移：`|max| * 5%`，max 为 0 时取 1。⛔ 别自造 per-metric 的 min/max
            # 常量 —— studio 里没有那张表（`useArmMetrics.ts:49-54` 只有 id/名/单位）。
            off = 1.0 if hi == 0 else abs(hi) * 0.05
            lo, hi = lo - off, hi + off
        pad = (hi - lo) * 0.08
        lo, hi = lo - pad, hi + pad
        ticks = _nice_ticks(lo, hi, 5) or [lo, hi]
        if is_temp:
            # ⚠ 上面那圈 padding 会把「0 起」又拽到负数（温度 24~36 ⇒ 算出 -20 起）。
            #   温度按 studio 的 `suggestedMin: 0` 语义**钉在 0 以上**，并丢掉负刻度。
            ticks = [t for t in ticks if t >= 0.0] or [0.0]
        lo, hi = min(lo, ticks[0]), max(hi, ticks[-1])
        if is_temp:
            lo = max(lo, 0.0)

        def y_of(v: float) -> float:
            return plot.bottom() - (v - lo) / (hi - lo) * plot.height()

        # ── y 网格线 + 刻度（x 网格按 studio 是**关**的）──
        p.setFont(QtGui.QFont(FONT_MONO.split(",")[0].strip('"'), 7))
        for tv in ticks:
            y = y_of(tv)
            if not (plot.top() - 1 <= y <= plot.bottom() + 1):
                continue
            p.setPen(QtGui.QPen(QtGui.QColor(128, 138, 150, 41), 1))   # 0.16α
            p.drawLine(QtCore.QPointF(plot.left(), y), QtCore.QPointF(plot.right(), y))
            p.setPen(QtGui.QPen(QtGui.QColor(C["chart_tick"]), 1))
            # ⚠ 夹住：最下面那条刻度的数字不许压到 x 轴时间标签那一带
            ty = min(max(y, r.top() + 7), plot.bottom() - 7)
            p.drawText(QtCore.QRectF(r.left(), ty - 7, left - 6, 14),
                       QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter,
                       f"{tv:g}")

        # ── x 底线（studio 开）──
        p.setPen(QtGui.QPen(QtGui.QColor(128, 138, 150, 77), 1))       # 0.3α
        p.drawLine(QtCore.QPointF(plot.left(), plot.bottom()),
                   QtCore.QPointF(plot.right(), plot.bottom()))

        n = len(self._times)
        if n >= 2:
            def x_of(i: int) -> float:
                return plot.left() + i / (n - 1) * plot.width()

            # x 刻度：上限 6 条、不旋转（`MetricsPanel.tsx:112`）。
            # ⚠ 还要按**标签宽度**再收一次 —— 时间串 "HH:MM:SS" 等宽约 52px，
            #    只按 6 条算会在窄栏里挤成一团（离屏截图量的）。
            want = max(2, min(6, int(plot.width() // 62)))
            step = max(1, (n - 1) // max(1, want - 1))
            p.setPen(QtGui.QPen(QtGui.QColor(C["chart_tick"]), 1))
            for i in range(n - 1, -1, -step):
                ts = time.strftime("%H:%M:%S", time.localtime(self._times[i]))
                # ⚠ 也夹住：否则首尾两个标签会被画到图框外、只剩半截
                tx = min(max(x_of(i) - 26, plot.left()), plot.right() - 52)
                p.drawText(QtCore.QRectF(tx, plot.bottom() + 3, 52, 14),
                           QtCore.Qt.AlignHCenter | QtCore.Qt.AlignTop, ts)

            # ── 曲线 ──
            for si, s in enumerate(self._series):
                if si >= len(self._colors):
                    break
                pen = QtGui.QPen(self._colors[si], 1.5)
                pen.setCapStyle(QtCore.Qt.RoundCap)
                pen.setJoinStyle(QtCore.Qt.RoundJoin)
                p.setPen(pen)
                # 分段画：遇到 None 就断线（不补 0）
                seg: List[QtCore.QPointF] = []
                for i, v in enumerate(s):
                    if v is None:
                        self._stroke(p, seg)
                        seg = []
                    else:
                        seg.append(QtCore.QPointF(x_of(i), y_of(v)))
                self._stroke(p, seg)

        # ── 无数据/断连：**图框保留**，只叠加说明（studio `MetricsPanel.tsx:187-192`）
        if self._notice:
            p.setPen(QtGui.QPen(QtGui.QColor(C["ink_faint"]), 1))
            f = QtGui.QFont(FONT_SANS.split(",")[0].strip('"'), 10)
            p.setFont(f)
            p.drawText(plot, QtCore.Qt.AlignCenter, self._notice)

    @staticmethod
    def _stroke(p: QtGui.QPainter, pts: List[QtCore.QPointF]) -> None:
        """把一串点画成折线；≥3 点时用 tension≈0.3 的贝塞尔平滑（照 chart.js）。"""
        if len(pts) < 2:
            return
        if len(pts) == 2:
            p.drawLine(pts[0], pts[1])
            return
        path = QtGui.QPainterPath(pts[0])
        tension = 0.3
        for i in range(len(pts) - 1):
            p0 = pts[i - 1] if i > 0 else pts[i]
            p1, p2 = pts[i], pts[i + 1]
            p3 = pts[i + 2] if i + 2 < len(pts) else pts[i + 1]
            c1 = QtCore.QPointF(p1.x() + (p2.x() - p0.x()) * tension / 3.0,
                                p1.y() + (p2.y() - p0.y()) * tension / 3.0)
            c2 = QtCore.QPointF(p2.x() - (p3.x() - p1.x()) * tension / 3.0,
                                p2.y() - (p3.y() - p1.y()) * tension / 3.0)
            path.cubicTo(c1, c2, p2)
        p.drawPath(path)


class MetricsCard(Card):
    """曲线卡 = 标题 + 指标分段 + 图 + 右栏图例（每行带读数）+ 全选/清空 + 暂停。

    ⚠ 采样由**本卡自己的 100 ms 定时器**驱动，读的是调用方喂进来的**最新快照**
    （`set_snapshot`）—— 而不是每来一帧就画一次。studio 就是这么做并**特意**说明了理由
    （`useArmMetrics.ts:98-99`：每帧换引用会把定时器反复重建；200 Hz 重绘也没有意义）。
    """

    def __init__(self, metrics: Sequence[str] = ("temp", "dq", "tau", "err"),
                 joint_count: int = N_JOINTS, parent=None):
        super().__init__("", parent=parent, padding=12, spacing=8)
        self.metrics = [m for m in METRIC_DEFS if m[0] in metrics]
        self.joint_count = joint_count
        self.series = MetricSeries()
        self.paused = False
        self._shown: List[int] = list(range(joint_count))
        self._snap = None
        self._was_live = False

        # ── 卡头：指标分段（左） + 暂停（右）──
        self.title.hide()
        self.seg = SegmentedControl([m[1] for m in self.metrics], 0, size="sm")
        self.seg.changed.connect(self._on_metric)
        self.header.insertWidget(0, self.seg)
        self.pause_badge = Badge("暂停", "outline")
        self.pause_badge.setCursor(QtCore.Qt.PointingHandCursor)
        self.pause_badge.mousePressEvent = self._toggle_pause      # type: ignore[assignment]
        self.pause_badge.setToolTip("暂停：只冻结采样，缓冲原样保留；继续后从当前时刻接着采")
        self.header.addWidget(self.pause_badge)

        # ── 图 + 右栏图例 ──
        mid = QtWidgets.QHBoxLayout()
        mid.setSpacing(10)
        self.chart = MetricsChart()
        mid.addWidget(self.chart, 1)

        legend_box = QtWidgets.QWidget()
        legend_box.setFixedWidth(112)
        self._legend = QtWidgets.QVBoxLayout(legend_box)
        self._legend.setContentsMargins(0, 0, 0, 0)
        self._legend.setSpacing(2)
        self.rows: List[Tuple[QtWidgets.QFrame, QtWidgets.QLabel, QtWidgets.QLabel]] = []
        for i in range(joint_count):
            self._legend.addWidget(self._make_legend_row(i))
        self._legend.addStretch(1)
        btnrow = QtWidgets.QHBoxLayout()
        btnrow.setSpacing(6)
        self.btn_all = QtWidgets.QPushButton("全选")
        self.btn_none = QtWidgets.QPushButton("清空")
        for b in (self.btn_all, self.btn_none):
            b.setProperty("variant", "outline")
            b.setCursor(QtCore.Qt.PointingHandCursor)
            b.setStyleSheet(
                f"QPushButton {{ {sans(11.5)} color: {C['ink_muted']}; background: {C['card']};"
                f" border: 1px solid {C['line']}; border-radius: {RADIUS['md']}px;"
                f" padding: 3px 9px; }}"
                f"QPushButton:hover {{ background: {C['hover']}; }}")
            btnrow.addWidget(b)
        self.btn_all.clicked.connect(self.select_all)
        self.btn_none.clicked.connect(self.select_none)
        self._legend.addLayout(btnrow)
        mid.addWidget(legend_box)
        self.body.addLayout(mid)

        self._timer = QtCore.QTimer(self)
        self._timer.setInterval(SERIES_INTERVAL_MS)
        self._timer.timeout.connect(self._tick)
        self._timer.start()
        self._redraw()

    # ── 图例 ──
    def _make_legend_row(self, i: int):
        row = QtWidgets.QFrame()
        row.setCursor(QtCore.Qt.PointingHandCursor)
        row.setToolTip(f"J{i + 1} —— 点击可隐藏/显示这条曲线")
        lay = QtWidgets.QHBoxLayout(row)
        lay.setContentsMargins(2, 1, 2, 1)
        lay.setSpacing(5)
        bar = QtWidgets.QLabel()
        bar.setFixedSize(11, 3)
        name = QtWidgets.QLabel(f"J{i + 1}")
        name.setStyleSheet(f"QLabel {{ {mono(11, 600)} color: {C['ink_strong']}; }}")
        val = QtWidgets.QLabel("—")
        val.setStyleSheet(f"QLabel {{ {mono(11, 600)} color: {C['ink']}; }}")
        lay.addWidget(bar)
        lay.addWidget(name)
        lay.addStretch(1)
        lay.addWidget(val)
        row.mousePressEvent = lambda _e, idx=i: self.toggle_joint(idx)   # type: ignore[assignment]
        self.rows.append((row, bar, val))
        return row

    def _refresh_legend(self) -> None:
        key = self.metrics[self.seg.current()][0]
        unit = self.metrics[self.seg.current()][2]
        last = self.series.samples()[-1] if len(self.series) else None
        for i, (row, bar, val) in enumerate(self.rows):
            on = i in self._shown
            color = joint_color(i) if on else C["line_strong"]
            bar.setStyleSheet(f"QLabel {{ background: {color}; border-radius: 1px; }}")
            v = None
            if last is not None and i < len(last[key]):
                v = last[key][i]
            # 单位只挂在第一行，避免 7 行都重复
            txt = _fmt(v, key) + (f" {unit}" if i == 0 and v is not None else "")
            val.setText(txt)
            val.setStyleSheet(
                f"QLabel {{ {mono(11, 600)}"
                f" color: {C['ink'] if on else C['ink_ghost']}; }}")

    # ── 交互 ──
    def _on_metric(self, idx: int) -> None:
        # ⚠ 切换指标**不清缓冲**（studio `metricDatasets.ts:5,30`）
        self._redraw()

    def _toggle_pause(self, _ev) -> None:
        self.paused = not self.paused
        self.pause_badge.set_state("继续" if self.paused else "暂停", "warn" if self.paused else "outline")
        self.pause_badge.setText("继续" if self.paused else "暂停")
        self._redraw()

    def toggle_joint(self, i: int) -> None:
        if i in self._shown:
            self._shown.remove(i)
        else:
            self._shown.append(i)
            self._shown.sort()            # 照 studio：重新勾选后保持升序（`:203-204`）
        self._redraw()

    def select_all(self) -> None:
        """勾选**当前存在的全部轴**（`useArmMetrics.ts:210`）。"""
        self._shown = list(range(self.joint_count))
        self._redraw()

    def select_none(self) -> None:
        """**只取消勾选**，不清缓冲（`useArmMetrics.ts:211`）。"""
        self._shown = []
        self._redraw()

    # ── 采样 ──
    def set_snapshot(self, snap) -> None:
        """喂最新快照。**只存引用**，采样发生在定时器里。"""
        self._snap = snap

    def _tick(self) -> None:
        s = self._snap
        live = bool(s is not None and s.connected)
        if not live:
            # 断连即清空缓冲（`useArmMetrics.ts:109-116`）
            if len(self.series) or self._was_live:
                self.series.clear()
                self._was_live = False
                self._redraw()
            return
        if not self._was_live:
            self.series.clear()          # 重连先清一次（`:123-127`）
            self._was_live = True
        if self.paused:
            return                       # ⚠ 暂停：不写缓冲、已有数据冻结（`:119`）
        n = max(len(s.q), 0)
        if n:
            def col(x, k):
                return list(x) if x else []
            self.series.append(
                time.time(),
                col(s.t_mos, n), col(s.dq, n), col(s.tau, n),
                [] if "err" in NO_DATA_METRICS else col(s.err, n))
        self._redraw()

    # ── 重绘 ──
    def _redraw(self) -> None:
        self._refresh_legend()
        key, _name, unit, ylabel = self.metrics[self.seg.current()]
        times = self.series.times()
        chans = self.series.channel(key) if len(self.series) else []
        series: List[List[Optional[float]]] = []
        for i in range(self.joint_count):
            series.append([
                (ch[i] if i < len(ch) else None) for ch in chans
            ] if i in self._shown else [None] * len(chans))
        colors = [joint_color(i) for i in range(self.joint_count)]

        notice = None
        if key in NO_DATA_METRICS:
            notice = "本机不提供「跟踪误差」通道（不伪造数据）"
        elif self._snap is None or not getattr(self._snap, "connected", False):
            notice = "未连接 · 无实时数据" if not len(self.series) else "链路已断开（缓冲已清空）"
        elif not len(self.series):
            notice = "等待数据…"
        elif not self._shown:
            notice = "已全部隐藏 —— 点右侧 J1…Jn 或「全选」"
        self.chart.set_data(times, series, colors, unit, ylabel, notice)
