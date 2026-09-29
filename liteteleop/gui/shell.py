"""应用外壳：左侧导航栏 + 顶栏。

出处 `litearm-studio`：
- `src/layout/AppShell.tsx:11-18` —— `flex h-screen w-screen overflow-hidden`，
  底 `--app-bg`，左栏固定宽，右侧叠 TopBar + 内容区。
- `src/layout/RailNav.tsx` —— 见 `RailNav` 的 docstring。
- `src/layout/TopBar.tsx` —— 见 `TopBar` 的 docstring。

## 图标

studio 用 `lucide-react` 的 SVG 线性图标。本仓是 PyQt5、**不引新依赖** ⇒ 这里用
`QPainter` 自绘同样笔触的线性图标（`icon_pixmap`）。⛔ 别改成 emoji/Unicode 字符：
不同机器上的字形差异极大，而且与 studio 的线性图标根本不是一回事。
"""
from __future__ import annotations

import math
from typing import Dict, List, Sequence, Tuple

from PyQt5 import QtCore, QtGui, QtWidgets

from .cards import MetricTile, StatusPill
from .theme import C, mono, sans

__all__ = ["RailNav", "TopBar", "icon_pixmap", "RAIL_W", "TOPBAR_H"]

#: 左栏宽度 —— `RailNav.tsx:34`：`w-20 basis-20` = 5rem = 82.19px。
RAIL_W = 82
#: 顶栏高度 —— `TopBar.tsx:61`：`h-14 basis-14` = 3.5rem = 57.53px。
TOPBAR_H = 58


# ────────────────────────── 图标 ──────────────────────────
def icon_pixmap(kind: str, size: int = 21, color: str = "#8b97a6") -> QtGui.QPixmap:
    """自绘线性图标，笔触 1.6px（按 24×24 画布，与 lucide 同规格）。

    `kind`: `link`（CircleDot）/ `arm`（机械臂）/ `log`（Activity 折线）/ `gear`（Settings）。
    """
    pm = QtGui.QPixmap(size * 2, size * 2)          # ×2 超采样，缩放后不毛糙
    pm.fill(QtCore.Qt.transparent)
    p = QtGui.QPainter(pm)
    p.setRenderHint(QtGui.QPainter.Antialiasing, True)
    u = size * 2 / 24.0                              # lucide 的画布是 24×24
    pen = QtGui.QPen(QtGui.QColor(color), 1.6 * u / 1.0)
    pen.setCapStyle(QtCore.Qt.RoundCap)
    pen.setJoinStyle(QtCore.Qt.RoundJoin)
    p.setPen(pen)
    p.setBrush(QtCore.Qt.NoBrush)

    if kind == "link":                               # CircleDot（studio 的 control 图标）
        p.drawEllipse(QtCore.QPointF(12 * u, 12 * u), 8.5 * u, 8.5 * u)
        p.setBrush(QtGui.QColor(color))
        p.drawEllipse(QtCore.QPointF(12 * u, 12 * u), 2.6 * u, 2.6 * u)
    elif kind == "arm":                              # 底座 + 两节臂 + 三个关节
        p.drawLine(QtCore.QPointF(6 * u, 21 * u), QtCore.QPointF(6 * u, 15 * u))
        p.drawLine(QtCore.QPointF(4 * u, 21 * u), QtCore.QPointF(8 * u, 21 * u))
        p.drawLine(QtCore.QPointF(6 * u, 15 * u), QtCore.QPointF(13 * u, 9 * u))
        p.drawLine(QtCore.QPointF(13 * u, 9 * u), QtCore.QPointF(19 * u, 13 * u))
        p.setBrush(QtGui.QColor(color))
        for cx, cy in ((6.0, 15.0), (13.0, 9.0), (19.0, 13.0)):
            p.drawEllipse(QtCore.QPointF(cx * u, cy * u), 1.7 * u, 1.7 * u)
    elif kind == "log":                              # Activity（studio 的 log 图标）
        pts = [(3, 12), (7.5, 12), (10, 5), (14, 19), (16.5, 12), (21, 12)]
        for (x1, y1), (x2, y2) in zip(pts, pts[1:]):
            p.drawLine(QtCore.QPointF(x1 * u, y1 * u), QtCore.QPointF(x2 * u, y2 * u))
    elif kind == "gear":                             # Settings：内外圆 + 8 根径向齿
        p.drawEllipse(QtCore.QPointF(12 * u, 12 * u), 3.4 * u, 3.4 * u)
        p.drawEllipse(QtCore.QPointF(12 * u, 12 * u), 7.6 * u, 7.6 * u)
        for k in range(8):
            a = k * math.pi / 4
            p.drawLine(QtCore.QPointF(12 * u + 7.6 * u * math.cos(a),
                                      12 * u + 7.6 * u * math.sin(a)),
                       QtCore.QPointF(12 * u + 10.4 * u * math.cos(a),
                                      12 * u + 10.4 * u * math.sin(a)))
    p.end()
    return pm.scaled(size, size, QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation)


# ────────────────────────── 左导航栏 ──────────────────────────
class RailNav(QtWidgets.QFrame):
    """左侧导航栏。

    出处 `RailNav.tsx`：整栏 **82.19px**、底 **`#141b26`（硬编码、不跟主题）**、
    内边距 `pt-3.5 pb-3`(14.38/12.33)、子项间距 6.16px；导航项 61.64px 宽、
    圆角 11.30px、`py-[0.5625rem]`(9.25)、图标 20.55px、文字 10.79px/600；
    **未激活 `#8b97a6` / 激活 `#ffffff`**；hover `#222c3a`、选中 `#2a3444`；
    底部靠 `flex-1` 撑开、**没有分隔线**。

    ⚠ 设计稿底部还有一项「中文」——**本仓不做**：本工具没有多语言（没有 i18n 资源），
    放一个点了没反应的按钮比不放更糟。底部只留「设置」。
    ⚠ logo 也不搬 studio 的 `logo.svg`（那是它家的资产）⇒ 用同尺寸的纯白文字标记占位。
    """

    changed = QtCore.pyqtSignal(int)
    settings_clicked = QtCore.pyqtSignal()

    def __init__(self, items: Sequence[Tuple[str, str]], parent=None):
        """`items` = `[(图标名, 文字), …]`，与页面栈同序。"""
        super().__init__(parent)
        self.setFixedWidth(RAIL_W)
        self.setStyleSheet(f"RailNav {{ background: {C['rail_bg']}; }}")
        self._icons: List[str] = [i for i, _t in items]

        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(0, 14, 0, 12)
        lay.setSpacing(6)

        logo = QtWidgets.QLabel("LA")
        logo.setAlignment(QtCore.Qt.AlignCenter)
        logo.setFixedHeight(21)
        logo.setStyleSheet(
            f"QLabel {{ color: #ffffff; {sans(13, 700)} letter-spacing: 1px; }}")
        logo.setToolTip("LiteArm 同构遥操")
        lay.addWidget(logo, 0, QtCore.Qt.AlignHCenter)
        lay.addSpacing(6)

        self._buttons: List[QtWidgets.QToolButton] = []
        for idx, (_icon, text) in enumerate(items):
            b = self._tool_button(text)
            b.setCheckable(True)
            b.setAutoExclusive(True)
            b.clicked.connect(lambda _c, i=idx: self._select(i))
            lay.addWidget(b, 0, QtCore.Qt.AlignHCenter)
            self._buttons.append(b)

        lay.addStretch(1)

        self.btn_settings = self._tool_button("设置")
        self.btn_settings.clicked.connect(self.settings_clicked)  # type: ignore[arg-type]
        lay.addWidget(self.btn_settings, 0, QtCore.Qt.AlignHCenter)

        if self._buttons:
            self._buttons[0].setChecked(True)
        self._refresh()

    @staticmethod
    def _tool_button(text: str) -> QtWidgets.QToolButton:
        b = QtWidgets.QToolButton()
        b.setText(text)
        b.setCursor(QtCore.Qt.PointingHandCursor)
        b.setToolButtonStyle(QtCore.Qt.ToolButtonTextUnderIcon)
        b.setIconSize(QtCore.QSize(21, 21))
        b.setFixedWidth(62)
        b.setStyleSheet(RailNav._item_qss(False, "gear"))
        return b

    @staticmethod
    def _item_qss(active: bool, _icon: str) -> str:
        fg = C["rail_fg_active"] if active else C["rail_fg"]
        hover = C["rail_active"] if active else C["rail_hover"]
        return (f"QToolButton {{ background: transparent; color: {fg};"
                f" border: none; border-radius: 11px; padding: 9px 0;"
                f" {sans(10.8, 600)} }}"
                f"QToolButton:hover {{ background: {hover}; color: {fg}; }}")

    def _select(self, idx: int) -> None:
        self._refresh()
        self.changed.emit(idx)

    def _refresh(self) -> None:
        for i, b in enumerate(self._buttons):
            active = b.isChecked()
            b.setStyleSheet(self._item_qss(active, self._icons[i]))
            b.setIcon(QtGui.QIcon(icon_pixmap(
                self._icons[i], 21,
                C["rail_fg_active"] if active else C["rail_fg"])))
        # 底部「设置」恒为非激活色（studio 的底部按钮也不随激活变色，`:72-73`）
        self.btn_settings.setIcon(QtGui.QIcon(icon_pixmap("gear", 21, C["rail_fg"])))

    def current(self) -> int:
        for i, b in enumerate(self._buttons):
            if b.isChecked():
                return i
        return 0

    def set_current(self, idx: int) -> None:
        """程序化选中。⚠ **不发** `changed`（避免自激）。"""
        if 0 <= idx < len(self._buttons):
            self._buttons[idx].setChecked(True)
            self._refresh()


# ────────────────────────── 顶栏 ──────────────────────────
class TopBar(QtWidgets.QFrame):
    """顶栏：标题 + 连接状态胶囊 + 端点 + 右侧指标组。

    出处 `TopBar.tsx:61-126`：高 **57.53px**、底 `--card`（**不是** `--app-bg`）、
    下边框 1px `--border`、左右内边距 18.49px；左侧标题 16.44px/**700**；
    端点串**等宽** 12.33px `--muted-foreground`；右侧指标组 `gap-4`(16.44)。
    """

    def __init__(self, title: str = "遥操控制台", parent=None):
        super().__init__(parent)
        self.setFixedHeight(TOPBAR_H)
        self.setStyleSheet(
            f"TopBar {{ background: {C['card']}; border-bottom: 1px solid {C['border']}; }}")
        lay = QtWidgets.QHBoxLayout(self)
        lay.setContentsMargins(18, 0, 18, 0)
        lay.setSpacing(12)

        self.lab_title = QtWidgets.QLabel(title)
        self.lab_title.setStyleSheet(
            f"QLabel {{ {sans(16.44, 700)} color: {C['foreground']}; }}")
        lay.addWidget(self.lab_title)

        self.pill = StatusPill("未连接")
        lay.addWidget(self.pill)

        # 副标题：**CDC 口 · 固件**（等宽，照 studio 把"端口·固件串"放这儿的做法）。
        # ⚠ 别再放一次 peer:jport —— 那跟左边状态胶囊里的端点重复。
        self.lab_endpoint = QtWidgets.QLabel("—")
        self.lab_endpoint.setStyleSheet(
            f"QLabel {{ {mono(12.33)} color: {C['muted_foreground']}; }}")
        lay.addWidget(self.lab_endpoint)

        lay.addStretch(1)

        self.tiles: Dict[str, MetricTile] = {}
        for key, label, unit in (("hz", "控制频率", "Hz"), ("load", "负载", "kg"),
                                 ("temp", "最高关节温度", "°C"), ("fault", "故障", "")):
            t = MetricTile(label, unit)
            lay.addWidget(t)
            self.tiles[key] = t

    # ── 刷新 ──
    def set_connection(self, connected: bool, text: str) -> None:
        self.pill.set_state(text, "ok" if connected else "idle")

    def set_endpoint(self, text: str) -> None:
        self.lab_endpoint.setText(text or "—")

    def update_from(self, s) -> None:
        """从快照刷新右侧四个指标。⚠ 没数据一律 `—`，**不写 0**。"""
        self.tiles["hz"].set_value(f"{s.state_hz:g}" if s.state_hz else "—")
        self.tiles["load"].set_value(f"{s.payload_mass:g}" if s.payload_com else "—")
        # ⚠ 「最高关节温度」取 **MOS 温度** —— 与曲线卡的「温度」是同一路
        #   （studio 也只取 `mosTemp`，`useArmMetrics.ts:133`）。
        #   ⛔ 别改用 `t_coil`：那会让顶栏与曲线卡显示两个不同的"温度"。
        t = max(s.t_mos) if s.t_mos else None
        # 温度取整显示（`36.9972` 那种读数在顶栏没有意义）；⛔ 别写 `:g`，
        # 它会把 f32 往返的尾数原样吐出来。
        self.tiles["temp"].set_value(f"{t:.0f}" if t is not None else "—")

        faults: List[str] = []
        if s.faulted:
            faults.append("FAULT")
        if s.joint_fault:
            faults.append(f"0x{s.joint_fault:x}")
        if s.error:
            faults.append(s.error)
        faults.extend(s.flag_names or [])
        if faults:
            self.tiles["fault"].set_value(faults[0][:14], "bad")
            self.tiles["fault"].setToolTip("、".join(faults))
        else:
            self.tiles["fault"].set_value("无" if s.connected else "—",
                                          "ok" if s.connected else "none")
            self.tiles["fault"].setToolTip("")
