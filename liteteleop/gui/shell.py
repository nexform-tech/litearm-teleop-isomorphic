"""应用外壳 = **顶栏**。

出处 `litearm-studio/src/layout/TopBar.tsx`：高 57.53px、底 `--card`（**不是**
`--app-bg`）、下边框 1px `--border`、左右内边距 18.49px；左侧标题 16.44px/**700**；
副标题（端口·固件串）**等宽** 12.33px `--muted-foreground`；右侧指标组 `gap-4`。

⚠ **2026-09-29 用户裁决：左侧导航栏（`RailNav`）整条删掉** —— 「连接/遥操/日志
三个分块没有意义，只要遥操这一个页面」。连带的清理：

- `RailNav` 与只服务于它的 `icon_pixmap()`（那四个 QPainter 线性图标）一并删除，
  ⛔ 不留死代码；
- `main_window` 不再是「车架 + 页面栈」，就是「顶栏 + 唯一一页」；
- studio 那套深色竖条的视觉标识**有意放弃**（用户选择了"整条去掉"）。

`RAIL_W` / `TOPBAR_H` 里只剩后者还有用；前者已随 `RailNav` 删除。
"""
from __future__ import annotations

from typing import Dict, List

from PyQt5 import QtCore, QtWidgets

from .cards import MetricTile, StatusPill
from .theme import C, mono, sans

__all__ = ["TopBar", "TOPBAR_H"]

#: 顶栏高度 —— `TopBar.tsx:61`：`h-14 basis-14` = 3.5rem = 57.53px。
TOPBAR_H = 58


class TopBar(QtWidgets.QFrame):
    """顶栏：标题 + 连接状态胶囊 + CDC/固件 + 右侧四个指标 + 急停（由 main_window 加）。

    出处 `TopBar.tsx:61-126`。
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

        #: ⚠ **连接组（CDC 口 / 重新扫描 / 连接臂）插在这** ——
        #: 用户裁决 2026-09-29：「放在『遥操控制台』这几个字的**水平右侧**」。
        #: 由 `main_window` 调 `add_connect()` 把 `pages.TeleopPage.connect_bar` 放进来。
        self.slot = QtWidgets.QHBoxLayout()
        self.slot.setContentsMargins(0, 0, 0, 0)
        self.slot.setSpacing(8)
        lay.addLayout(self.slot)

        self.pill = StatusPill("未连接")
        lay.addWidget(self.pill)

        # 详情串（等宽）：固件 / 端点，出错时是错误文本。
        # ⚠ 它**可被压缩**（`setMinimumWidth(0)` + 省略号），是这一行里最先让步的那个；
        #   右边那四个指标和急停是固定宽度。
        self.lab_detail = QtWidgets.QLabel("—")
        self.lab_detail.setMinimumWidth(0)
        self.lab_detail.setStyleSheet(
            f"QLabel {{ {mono(12.33)} color: {C['muted_foreground']}; }}")
        lay.addWidget(self.lab_detail, 0)

        lay.addStretch(1)

        self.tiles: Dict[str, MetricTile] = {}
        for key, label, unit in (("hz", "控制频率", "Hz"), ("load", "负载", "kg"),
                                 ("temp", "最高关节温度", "°C"), ("fault", "故障", "")):
            t = MetricTile(label, unit)
            lay.addWidget(t)
            self.tiles[key] = t

    # ── 装配 ──
    def add_connect(self, w: QtWidgets.QWidget) -> None:
        """把「连接」组插到标题的**水平右侧**（用户裁决 2026-09-29）。"""
        self.slot.addWidget(w)

    # ── 刷新 ──
    def set_connection(self, connected: bool, text: str, kind: str = "") -> None:
        """`kind` 显式给 `bad` 时会显示成错误态（出错时用）。"""
        self.pill.set_state(text, kind or ("ok" if connected else "idle"))

    def set_detail(self, text: str, role: str = "hintSubtle") -> None:
        """详情串（固件/端点，或错误文本）。太长就省略 —— 见 `lab_detail` 的注释。"""
        fm = self.lab_detail.fontMetrics()
        self.lab_detail.setText(
            fm.elidedText(text or "—", QtCore.Qt.ElideRight, max(120, self.lab_detail.width())))
        self.lab_detail.setToolTip(text or "")
        color = {"danger": C["danger"], "ok": C["ok"]}.get(role, C["muted_foreground"])
        self.lab_detail.setStyleSheet(
            f"QLabel {{ {mono(12.33)} color: {color}; }}")

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
