"""三个页面：链路 / 关节 / 遥操（spec §8）。"""
from __future__ import annotations

from typing import Optional

from PyQt5 import QtCore, QtWidgets

from .. import servo
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
    """遥操页：主臂 vs 从臂对照 + 参数 + **末端载荷（夹爪）**。"""

    #: (mass_kg, [x, y, z]) —— 由 main_window 接到 `worker.set_payload`
    payload_applied = QtCore.pyqtSignal(float, object)

    def __init__(self, parent=None):
        super().__init__(parent)
        # ⚠ 套**滚动区**：本页内容（对照表 + 参数 + 载荷组）比默认窗口高，
        #    不套的话底部会被裁掉（离屏量过：内容到 y≈496 而页面只有 476）。
        outer = QtWidgets.QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        sa = QtWidgets.QScrollArea()
        sa.setWidgetResizable(True)
        sa.setFrameShape(QtWidgets.QFrame.NoFrame)
        inner = QtWidgets.QWidget()
        lay = QtWidgets.QVBoxLayout(inner)
        sa.setWidget(inner)
        outer.addWidget(sa)

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
        self.cmp.setMinimumHeight(190)
        lay.addWidget(self.cmp)

        # ⚠ 文本**全静态** ⇒ 在 __init__ 里就设好。
        #    放在 apply() 里设的后果：布局按"空标签"算好了高度，之后不重算 ⇒ **文字被裁**
        #    （离屏量出来只有 20px 高，而这里有 4 行）。
        self.lab_params = QtWidgets.QLabel(
            "参数（限速/限加速照抄 litearm-server 的配置口径）\n"
            f"  speed_limit = {servo.DEFAULT_SPEED_LIMIT}\n"
            f"  accel_limit = {servo.DEFAULT_ACCEL_LIMIT}"
            f"    engage_sec = {servo.DEFAULT_ENGAGE_SEC}\n"
            f"刚度/阻尼**随帧下发**（非固件出厂值）："
            f"K={servo.SETUP_K}  B={servo.SETUP_B}")
        self.lab_params.setWordWrap(True)
        self.lab_params.setMinimumHeight(self.lab_params.sizeHint().height())
        lay.addWidget(self.lab_params)

        # ── 末端载荷（夹爪）──────────────────────────────────────────────
        # ── 末端载荷（夹爪）──────────────────────────────────────────────
        # ⚠ 用**嵌套 QHBoxLayout**，不用 QGridLayout —— 网格是**共享列**的，
        #    上一行的"质心 x"标签会和下一行按钮占同一列 ⇒ 宽度互相挤，标签重叠。
        box = QtWidgets.QGroupBox("末端载荷（夹爪）—— 影响重力前馈")
        bl = QtWidgets.QVBoxLayout(box)

        row1 = QtWidgets.QHBoxLayout()
        row1.addWidget(QtWidgets.QLabel("质量"))
        self.sp_mass = QtWidgets.QDoubleSpinBox()
        self.sp_mass.setRange(0.0, 20.0); self.sp_mass.setDecimals(3)
        self.sp_mass.setSingleStep(0.05); self.sp_mass.setSuffix(" kg")
        self.sp_mass.setMinimumWidth(110)
        self.sp_mass.setValue(servo.DEFAULT_PAYLOAD_MASS)
        row1.addWidget(self.sp_mass)
        self.sp_com = []
        for k, name in enumerate(("质心 X", "质心 Y", "质心 Z")):
            row1.addSpacing(12)
            row1.addWidget(QtWidgets.QLabel(name))
            sp = QtWidgets.QDoubleSpinBox()
            sp.setRange(-1.0, 1.0); sp.setDecimals(4)
            sp.setSingleStep(0.005); sp.setSuffix(" m")
            sp.setMinimumWidth(105)
            sp.setValue(servo.DEFAULT_PAYLOAD_COM[k])
            self.sp_com.append(sp)
            row1.addWidget(sp)
        row1.addStretch(1)
        bl.addLayout(row1)

        lab_hint = QtWidgets.QLabel(
            "⚠ 质心在 <b>ee_link 系</b>、单位<b>米</b>；本项目夹爪的 3 cm 在 <b>Z</b> 轴")
        lab_hint.setWordWrap(True)
        bl.addWidget(lab_hint)

        row2 = QtWidgets.QHBoxLayout()
        self.btn_payload = QtWidgets.QPushButton("应用载荷")
        self.btn_gripper = QtWidgets.QPushButton("预设：夹爪 600 g / Z 3 cm")
        self.btn_gripper.clicked.connect(self._preset_gripper)
        self.btn_payload.clicked.connect(self._apply_payload)
        row2.addWidget(self.btn_payload)
        row2.addWidget(self.btn_gripper)
        row2.addStretch(1)
        bl.addLayout(row2)

        self.lab_payload = QtWidgets.QLabel("当前生效：—")
        self.lab_payload.setWordWrap(True)
        bl.addWidget(self.lab_payload)

        note = QtWidgets.QLabel(
            "⚠ 固件会**静默钳制**（质量 [0,20]、质心 [-1,1]）且照样回 ACK ⇒ "
            "上面显示的是**读回值**，不是输入值。⛔ 只写 RAM，断电即还原。")
        note.setWordWrap(True)
        bl.addWidget(note)
        lay.addWidget(box)

    def _preset_gripper(self) -> None:
        self.sp_mass.setValue(servo.DEFAULT_PAYLOAD_MASS)
        for k, v in enumerate(servo.DEFAULT_PAYLOAD_COM):
            self.sp_com[k].setValue(v)

    def _apply_payload(self) -> None:
        self.payload_applied.emit(
            float(self.sp_mass.value()),
            [float(sp.value()) for sp in self.sp_com])

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
        if s.payload_com:
            self.lab_payload.setText(
                f"当前生效：<b>{s.payload_mass:.3f} kg</b>，质心 "
                f"[{', '.join(f'{v:+.4f}' for v in s.payload_com)}] m"
                + ("　⚠ 全是 0 ⇒ 可能没设上" if s.payload_mass == 0.0 else ""))
        else:
            self.lab_payload.setText("当前生效：—（未连接或读不到）")
