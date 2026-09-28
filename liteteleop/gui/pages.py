"""三个页面：链路 / 关节 / 遥操（spec §8）。"""
from __future__ import annotations

from typing import Optional

from PyQt5 import QtCore, QtWidgets

from ..arm_worker import ROLE_MASTER, ROLE_SLAVE, Snapshot, PUB_HZ, SLAVE_HZ
from ..ports import list_arms
from ..settings import Settings
from ..wire import N_JOINTS
from .widgets import JointTable

__all__ = ["LinkPage", "JointsPage", "TeleopPage"]


class LinkPage(QtWidgets.QWidget):
    """角色单选、地址端口、动作按钮、固件/许可、连接灯。"""

    connect_clicked = QtCore.pyqtSignal()
    teleop_toggled = QtCore.pyqtSignal(bool)
    settings_changed = QtCore.pyqtSignal()

    def __init__(self, settings: Settings, parent=None):
        super().__init__(parent)
        self.s = settings
        form = QtWidgets.QFormLayout(self)

        row = QtWidgets.QHBoxLayout()
        self.rb_master = QtWidgets.QRadioButton("主臂（监听 IP:端口）")
        self.rb_slave = QtWidgets.QRadioButton("从臂（连接主臂）")
        (self.rb_master if settings.role == ROLE_MASTER else self.rb_slave).setChecked(True)
        for rb in (self.rb_master, self.rb_slave):
            row.addWidget(rb)
            rb.toggled.connect(self._role_changed)
        form.addRow("角色", self._wrap(row))

        row2 = QtWidgets.QHBoxLayout()
        self.ed_peer = QtWidgets.QLineEdit(settings.peer)
        self.sp_port = QtWidgets.QSpinBox()
        self.sp_port.setRange(1, 65535)
        self.sp_port.setValue(settings.jport)
        self.ed_arm_id = QtWidgets.QLineEdit(settings.arm_id)
        row2.addWidget(QtWidgets.QLabel("IP")); row2.addWidget(self.ed_peer)
        row2.addWidget(QtWidgets.QLabel("端口")); row2.addWidget(self.sp_port)
        row2.addWidget(QtWidgets.QLabel("arm_id")); row2.addWidget(self.ed_arm_id)
        form.addRow("地址", self._wrap(row2))
        for w in (self.ed_peer, self.ed_arm_id):
            w.editingFinished.connect(self._emit_changed)
        self.sp_port.valueChanged.connect(self._emit_changed)

        # ⚠ CDC 口：**两条以上同型号臂时必须显式选**（VID:PID 相同，自动挑会挑错且不报错）
        row_port = QtWidgets.QHBoxLayout()
        self.cb_port = QtWidgets.QComboBox()
        self.btn_rescan = QtWidgets.QPushButton("重新扫描")
        self.btn_rescan.clicked.connect(self.rescan_ports)
        self.lab_port_note = QtWidgets.QLabel()
        row_port.addWidget(QtWidgets.QLabel("CDC 口"))
        row_port.addWidget(self.cb_port, 1)
        row_port.addWidget(self.btn_rescan)
        form.addRow("臂", self._wrap(row_port))
        form.addRow("", self.lab_port_note)
        self.cb_port.currentIndexChanged.connect(self._emit_changed)

        row3 = QtWidgets.QHBoxLayout()
        self.btn_connect = QtWidgets.QPushButton("连接臂")
        self.btn_teleop = QtWidgets.QPushButton("启动遥操")
        self.btn_teleop.setEnabled(False)
        self.btn_teleop.setCheckable(True)          # ⚠ 必须在 connect 之前
        self.btn_connect.clicked.connect(self.connect_clicked.emit)
        # ⚠⚠ 不要写 `not isChecked()`：Qt 的 `clicked` 是在按钮状态**已经切换之后**
        #     才发出的 ⇒ 此刻 `isChecked()` **就是**用户想要的新值。
        #     加 `not` 会把"启动"发成"停止"（我这么错过一次，真机上表现为"点了没反应"）。
        self.btn_teleop.clicked.connect(
            lambda: self.teleop_toggled.emit(self.btn_teleop.isChecked()))
        row3.addWidget(self.btn_connect); row3.addWidget(self.btn_teleop)
        form.addRow("动作", self._wrap(row3))

        self.rescan_ports()
        self.lab_info = QtWidgets.QLabel("未连接")
        self.lab_info.setWordWrap(True)
        form.addRow("状态", self.lab_info)

        note = QtWidgets.QLabel(
            "⚠ 一个 CDC 口只允许一个进程（STM32 CDC 在 Linux 不独占，第二个进程会分吃同一字节流）")
        note.setWordWrap(True)
        form.addRow("", note)

    @staticmethod
    def _wrap(lay) -> QtWidgets.QWidget:
        w = QtWidgets.QWidget(); w.setLayout(lay); return w

    def rescan_ports(self) -> None:
        """重新枚举 CDC 口。**序列号才是唯一标识**，所以标签里带上它。"""
        cur = self.s.cdc_port
        self.cb_port.blockSignals(True)
        self.cb_port.clear()
        self.cb_port.addItem("自动（只有一条臂时）", "")
        for a in list_arms():
            self.cb_port.addItem(a.label, a.device)
        idx = self.cb_port.findData(cur)
        self.cb_port.setCurrentIndex(idx if idx >= 0 else 0)
        self.cb_port.blockSignals(False)
        n = self.cb_port.count() - 1
        if n >= 2:
            self.lab_port_note.setText(
                f"⚠ 检测到 <b>{n}</b> 条臂。同型号 VID:PID 相同，"
                "<b>必须选一个</b> —— 自动挑会挑错而且不报错。")
            self.lab_port_note.setStyleSheet("color:#b02020;")
        elif n == 0:
            self.lab_port_note.setText("未检测到 STM32 CDC 口（检查 USB 与 dialout 权限）")
            self.lab_port_note.setStyleSheet("color:#b02020;")
        else:
            self.lab_port_note.setText(f"检测到 1 条臂：{self.cb_port.itemText(1)}")
            self.lab_port_note.setStyleSheet("")

    def role(self) -> str:
        return ROLE_MASTER if self.rb_master.isChecked() else ROLE_SLAVE

    def _role_changed(self) -> None:
        self.ed_peer.setEnabled(self.role() == ROLE_SLAVE)
        self._emit_changed()

    def _emit_changed(self) -> None:
        self.s.role = self.role()
        self.s.peer = self.ed_peer.text().strip() or "127.0.0.1"
        self.s.jport = int(self.sp_port.value())
        self.s.arm_id = self.ed_arm_id.text().strip() or "armA"
        self.s.cdc_port = self.cb_port.currentData() or ""
        self.settings_changed.emit()

    def apply(self, s: Snapshot) -> None:
        if s.error:
            self.lab_info.setText(f"⛔ {s.error}")
        elif s.connected:
            side = "监听" if s.role == ROLE_MASTER else f"连接 {self.s.peer}"
            self.lab_info.setText(f"✓ 已连接 · 固件 {s.firmware} · {side}:{self.s.jport}")
        else:
            self.lab_info.setText("未连接")
        self.btn_teleop.setEnabled(s.connected)
        # ⚠ 灯是**按真实状态**刷新，不是按点击
        self.btn_teleop.setText("停止遥操" if s.teleop_active else "启动遥操")
        self.btn_teleop.setChecked(bool(s.teleop_active))


class JointsPage(QtWidgets.QWidget):
    """7 轴表。"""

    def __init__(self, parent=None):
        super().__init__(parent)
        lay = QtWidgets.QVBoxLayout(self)
        self.table = JointTable()
        lay.addWidget(self.table)

    def apply(self, s: Snapshot) -> None:
        self.table.update_from(s)


class TeleopPage(QtWidgets.QWidget):
    """遥操页：主臂 vs 从臂对照 + 参数。"""

    def __init__(self, parent=None):
        super().__init__(parent)
        lay = QtWidgets.QVBoxLayout(self)
        self.lab = QtWidgets.QLabel("未启动")
        self.lab.setStyleSheet("font-size: 15px;")
        lay.addWidget(self.lab)

        self.cmp = QtWidgets.QTableWidget(N_JOINTS, 2)
        self.cmp.setHorizontalHeaderLabels(["主臂 q（收到的帧）", "本臂 q（实测）"])
        self.cmp.verticalHeader().setVisible(False)
        self.cmp.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        for r in range(N_JOINTS):
            self.cmp.setItem(r, 0, QtWidgets.QTableWidgetItem("—"))
            self.cmp.setItem(r, 1, QtWidgets.QTableWidgetItem("—"))
        lay.addWidget(self.cmp)

        self.lab_params = QtWidgets.QLabel()
        self.lab_params.setWordWrap(True)
        lay.addWidget(self.lab_params)

    def apply(self, s: Snapshot) -> None:
        # 状态行：**没有状态机** —— litearm-server 用 `is_running` 派生 active
        if s.role == ROLE_MASTER:
            self.lab.setText(
                f"主臂：零重力拖动中 · 发布 {PUB_HZ:.0f} Hz · 已发 {s.frames_sent} 帧 · "
                f"{'已匹配订阅者' if s.matching else '⚠ 未匹配（发了没人在收）'}")
        else:
            age = "—" if s.frame_age is None else f"{s.frame_age * 1000:.0f} ms"
            self.lab.setText(
                f"从臂：跟随中 · 环频 {SLAVE_HZ:.0f} Hz · 已收 {s.frames_received} 帧 · "
                f"帧龄 {age} · watchdog 超时 {s.watchdog_trips}")
        for r in range(N_JOINTS):
            if len(s.q) == N_JOINTS:
                self.cmp.item(r, 1).setText(f"{s.q[r]:+.4f}")
        self.lab_params.setText(
            "参数（逐值照抄 litearm-server 的 litearm_balanced.yaml · 执行器 move_js）\n"
            "  K = [25.0]×7   B = [0.5]×7   engage_sec = 0.3\n"
            "  speed_limit = [2.8, 3.4, 5.0, 5.0, 10.0, 8.0, 13.0]\n"
            "  accel_limit = [14.0, 22.0, 24.0, 24.0, 45.0, 40.0, 60.0]\n"
            "⚠ K/B 由 apply_joint_gains 写进固件参数（**只写 RAM**），退出时还原；\n"
            "  同时**清零 kd_extra** —— 否则有效阻尼 = mit_kd + kd_extra，J1~J4 会变成 13 倍")
