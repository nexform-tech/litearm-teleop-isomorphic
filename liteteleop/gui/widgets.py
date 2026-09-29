"""常驻控件：关节表 + 状态条（spec §8）。"""
from __future__ import annotations

from typing import List, Optional

from PyQt5 import QtCore, QtGui, QtWidgets

from ..arm_worker import Snapshot
from ..wire import N_JOINTS

__all__ = ["JointTable", "StatusStrip", "OK_BRUSH", "BAD_BRUSH"]

OK_BRUSH = QtGui.QColor(0x20, 0x7a, 0x30)      # 正常
BAD_BRUSH = QtGui.QColor(0xb0, 0x20, 0x20)     # 异常（红底白字）

#: `err` 的**正确口径**（用户裁决，spec §8）：**1 = 使能**，0 = 失能，其余 = 故障。
#: ⛔ 判据是 `err != 1`，**不是** `err != 0` —— 写反了健康的使能臂会七轴全红。
ERR_ENABLED = 1


class JointTable(QtWidgets.QTableWidget):
    """`J | q | dq | tau | t_mos | t_coil | err`。

    温度**只显示数值**，不判阈值 —— 固件的温度锁存已反映在 `err`/`joint_fault` 上，
    界面另发明一套阈值就是制造第二份真相（spec §8）。
    """

    COLS = ["关节", "q (rad)", "dq (rad/s)", "tau (Nm)", "t_mos (°C)", "t_coil (°C)", "err"]

    def __init__(self, parent=None):
        super().__init__(N_JOINTS, len(self.COLS), parent)
        self.setHorizontalHeaderLabels(self.COLS)
        self.verticalHeader().setVisible(False)
        self.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.setSelectionMode(QtWidgets.QAbstractItemView.NoSelection)
        self.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        for r in range(N_JOINTS):
            self.setItem(r, 0, QtWidgets.QTableWidgetItem(f"J{r + 1}"))
            for c in range(1, len(self.COLS)):
                it = QtWidgets.QTableWidgetItem("—")
                it.setTextAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
                self.setItem(r, c, it)

    def update_from(self, s: Snapshot) -> None:
        if len(s.q) != N_JOINTS:
            return
        for r in range(N_JOINTS):
            vals = (s.q[r], s.dq[r], s.tau[r], s.t_mos[r], s.t_coil[r])
            for c, v in enumerate(vals, start=1):
                self.item(r, c).setText(f"{v:+.4f}" if c <= 3 else f"{v:.1f}")
            it = self.item(r, 6)
            it.setText(str(s.err[r]))
            # ⚠ `err != 1` 才是异常（1 = 使能）
            bad = s.err[r] != ERR_ENABLED
            it.setBackground(BAD_BRUSH if bad else OK_BRUSH)
            it.setForeground(QtGui.QColor("white"))


class StatusStrip(QtWidgets.QWidget):
    """常驻状态条：模式 | 使能 | 遥操 | 频率 | 帧龄 | 故障。"""

    def __init__(self, parent=None):
        super().__init__(parent)
        lay = QtWidgets.QHBoxLayout(self)
        lay.setContentsMargins(8, 2, 8, 2)
        self._labels = {}
        for key in ("模式", "使能", "遥操", "频率", "匹配/帧龄", "故障"):
            lab = QtWidgets.QLabel(f"{key}: —")
            lab.setFrameShape(QtWidgets.QFrame.StyledPanel)
            lay.addWidget(lab)
            self._labels[key] = lab

    def update_from(self, s: Snapshot) -> None:
        self._labels["模式"].setText(f"模式: {s.mode_name or '—'}")
        self._labels["使能"].setText(f"使能: {'是' if s.enabled else '否'}")
        self._labels["遥操"].setText(f"遥操: {'运行中' if s.teleop_active else '停止'}")
        if s.role == "master":
            self._labels["频率"].setText(f"已发: {s.frames_sent}")
            self._labels["匹配/帧龄"].setText(
                f"订阅: {'已匹配' if s.matching else '未匹配'}")
        else:
            self._labels["频率"].setText(f"已收: {s.frames_received}")
            age = "—" if s.frame_age is None else f"{s.frame_age * 1000:.0f} ms"
            self._labels["匹配/帧龄"].setText(f"帧龄: {age}")
        faults: List[str] = []
        if s.faulted:
            faults.append("FAULT")
        if s.joint_fault:
            faults.append(f"joint_fault=0x{s.joint_fault:x}")
        faults.extend(s.flag_names)
        if s.error:
            faults.append(s.error)
        self._labels["故障"].setText("故障: " + ("、".join(faults) if faults else "无"))
