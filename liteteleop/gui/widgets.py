"""常驻控件：7 轴关节表（spec §8）。

⚠ 原来这里还有一个 `StatusStrip`（底部常驻状态条）。**2026-09-29 删掉了** ——
设计稿里没有底部状态条，那几格被**顶栏的指标组**和**主从链路卡的指标块**取代了
（`shell.TopBar` / `pages.LinkCard`）。判据（主臂显示"已发"、从臂显示"已收"）
原样搬到了链路卡，没有丢。
"""
from __future__ import annotations

from typing import List, Optional

from PyQt5 import QtCore, QtGui, QtWidgets

from ..arm_worker import Snapshot
from ..wire import N_JOINTS

__all__ = ["JointTable", "OK_BRUSH", "BAD_BRUSH", "ERR_ENABLED"]

OK_BRUSH = QtGui.QColor(0x20, 0x7a, 0x30)      # 正常
BAD_BRUSH = QtGui.QColor(0xb0, 0x20, 0x20)     # 异常（红底白字）

#: `err` 的**正确口径**（用户裁决，spec §8）：**1 = 使能**，0 = 失能，其余 = 故障。
#: ⛔ 判据是 `err != 1`，**不是** `err != 0` —— 写反了健康的使能臂会七轴全红。
ERR_ENABLED = 1


class JointTable(QtWidgets.QTableWidget):
    """`J | q | dq | tau | t_mos | t_coil | err`。

    温度**只显示数值**，不判阈值 —— 固件的温度锁存已反映在 `err`/`joint_fault` 上，
    界面另发明一套阈值就是制造第二份真相（spec §8）。

    ⚠ `compact=True` 给「遥操页左栏」用（那里只有 ~430px 宽）。它**只换表头文案与字体**，
    **列数与判据一律不变** —— `err != 1 才高亮` 这条口径在任何模式下都成立。
    ⛔ 别在紧凑模式下砍掉 `t_coil` 或 `err` 列：那是拿"放不下"当借口改判据。
    """

    COLS = ["关节", "q (rad)", "dq (rad/s)", "tau (Nm)", "t_mos (°C)", "t_coil (°C)", "err"]
    #: 紧凑表头（窄栏用）。列序与 `COLS` **逐一对应**，不许增删列。
    COLS_COMPACT = ["J", "q", "dq", "tau", "t_mos", "t_coil", "err"]

    def __init__(self, parent=None, compact: bool = False):
        super().__init__(N_JOINTS, len(self.COLS), parent)
        self._compact = compact
        self.setHorizontalHeaderLabels(self.COLS_COMPACT if compact else self.COLS)
        self.verticalHeader().setVisible(False)
        self.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.setSelectionMode(QtWidgets.QAbstractItemView.NoSelection)
        self.setShowGrid(False)
        if compact:
            # ⚠⚠ 紧凑模式**不能靠 `QHeaderView.Stretch`**：它按表头构造时的长度算列宽，
            #   离屏实测出来是 `91×7 = 637px`，而左栏只有 ~420px
            #   ⇒ **最后一列（err）被裁掉**，而 err 恰恰是唯一的判据列。
            #   改成显式分配：`Interactive` 模式 + `resizeEvent` 里均分。
            for c in range(self.columnCount()):
                self.horizontalHeader().setSectionResizeMode(
                    c, QtWidgets.QHeaderView.Interactive)
            # ⚠ 字体必须用 `setFont` 直接设 —— 只写在 QSS 里改不动 Qt 算行高用的字体，
            #   实测行高仍是 25px（QSS 是绘制期生效），7 行就装不下被裁。
            f = QtGui.QFont(self.font())
            f.setPixelSize(11)
            self.setFont(f)
            self.horizontalHeader().setFont(f)
            self.verticalHeader().setDefaultSectionSize(21)
            self.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
            self.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
            self._lock_height()
        else:
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

    # ── 紧凑模式：显式均分列宽 + 高度贴合 ──
    def _lock_height(self) -> None:
        """把高度钉成「实测表头 + 7 行」。

        ⚠⚠ **必须在 `resizeEvent` 里反复重算，不能在 `__init__` 里算一次。**
        构造那一刻表头还没按最终字体量过高（实测差 3px），于是固定高度偏小、
        **最后一行 J7 被裁掉** —— 而这一列里恰好有 `err` 这个唯一判据。
        """
        h = (self.horizontalHeader().height()
             + sum(self.rowHeight(r) for r in range(N_JOINTS)) + 2)
        if h != self.height():
            self.setFixedHeight(h)

    def resizeEvent(self, ev) -> None:
        super().resizeEvent(ev)
        if not self._compact:
            return
        n = self.columnCount()
        avail = self.viewport().width()
        if avail > 0:
            each = max(28, avail // n)      # 夹一个下限，太窄就让文字自己省略
            for c in range(n):
                self.setColumnWidth(c, each)
        self._lock_height()
