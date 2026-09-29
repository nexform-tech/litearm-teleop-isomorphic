"""四个页面：连接 / 遥操 / 日志 / 设置。

版式照用户给的设计稿（`微信图片_20260928133319_156_1227.png`），
视觉语言照 `litearm-studio`（见 `theme.py` / `cards.py` / `shell.py` 的出处注释）。

## ⚠⚠ 这一层的一条硬纪律：**不许画"看起来能改、其实没接线"的输入框**

设计稿里有几格在**本版并没有接到任何东西上**（臂的 K/B 增益、对齐速度、watchdog、
夹爪的频率/kp/kd）。这些一律用 `ro_field()` 渲染成**只读的"当前生效值"**，
并在 tooltip 里写明出处。⛔ 别图省事把它们画成可编辑的 `QLineEdit`：
那会让界面在撒谎 —— 用户改了、数字变了，而臂的行为一点没变。

（要真做成可编辑是**另一个单子**：得把它们接进 `Settings` → 传给
`servo.follow(K=,B=)` / `GripWorker(kp=,kd=,rate_hz=)`，而且臂的 K/B 属于调参，
按仓库规矩要在 `PARAM_STATE.md` §B 登记。见最终报告里的待办。）

## ⚠ 设计稿有、本仓**不做**的

- 左侧导航的「中文」项：本工具没有 i18n 资源，放一个点了没反应的按钮更糟。
- 「抖动 (ms)」指标块：`Snapshot` 里没有抖动这个量 ⇒ 换成**帧龄 (ms)**（真有的量），
  ⛔ 而不是把「抖动」永远显示成 `—`。
"""
from __future__ import annotations

import os
from typing import List, Optional

from PyQt5 import QtCore, QtWidgets

from .. import servo
from ..arm_worker import (ROLE_MASTER, ROLE_SLAVE, PUB_HZ, SLAVE_HZ, WATCHDOG_MS,
                          Snapshot)
from ..ports import list_arms
from ..settings import Settings
from ..wire import N_JOINTS
from .cards import (Badge, Card, MetricTile, ScrollColumn, SegmentedControl,
                    StopButton, field_row, hint, separator)
from .chart import MetricsCard
from .theme import C, RADIUS, mono, sans
from .widgets import JointTable

__all__ = ["ConnectPage", "TeleopPage", "LogsPage", "SettingsPage", "can_channels"]


def can_channels() -> List[str]:
    """本机的 `can*` 网络接口名（spec §9.3）。

    ⚠ 只是个**提示**，不是扫描按钮：`ports.py` 枚举的是 CDC（按 STM32 的 VID:PID
    过滤），本仓**没有**枚举 SocketCAN 的代码；而 `litegrip` 的 `channel` 就是
    一个字符串 ⇒ 加枚举＝新增一条没验过的代码路径。
    读 `/sys/class/net` 是零依赖的，且能挡住"通道名打错"这个常见失败。
    """
    try:
        return sorted(n for n in os.listdir("/sys/class/net") if n.startswith("can"))
    except OSError:
        return []


def ro_field(text: str, width: int = 96, tip: str = "") -> QtWidgets.QLineEdit:
    """**只读**的"当前生效值"格子。

    ⚠ 用只读 `QLineEdit`（而不是 `QLabel`）是为了保住设计稿那种"格子"的外观，
    但它是灰底只读的，不会被误当成可输入 —— 见模块 docstring 的硬纪律。
    """
    w = QtWidgets.QLineEdit(text)
    w.setReadOnly(True)
    w.setFixedWidth(width)
    w.setFocusPolicy(QtCore.Qt.NoFocus)
    w.setStyleSheet(
        f"QLineEdit {{ {mono(12)} color: {C['ink_soft']}; background: {C['line_soft']};"
        f" border: 1px solid {C['line']}; border-radius: {RADIUS['md']}px;"
        f" padding: 3px 7px; }}")
    if tip:
        w.setToolTip(tip)
    return w


def _spin(val: int, lo: int, hi: int, width: int = 84) -> QtWidgets.QSpinBox:
    w = QtWidgets.QSpinBox()
    w.setRange(lo, hi)
    w.setValue(val)
    w.setFixedWidth(width)
    return w


def rc_label(text: str = "只读") -> QtWidgets.QLabel:
    """给只读格子配的小标注。"""
    w = QtWidgets.QLabel(text)
    w.setStyleSheet(f"QLabel {{ {sans(10.5)} color: {C['ink_faint']}; }}")
    return w


# ══════════════════════════ 连接页 ══════════════════════════
class ConnectPage(QtWidgets.QWidget):
    """连接前的设置：角色 / 地址 / CDC 口 / 连接。

    ⚠ **角色**只有这里能改 —— `ArmWorker` 在构造时就吃掉 `role`/`port`/`arm_id`/`peer`，
    连上之后改这些没有意义（所以 `MainWindow._connect` 会拒绝二次连接）。
    遥操页卡片头那个「本机角色」因此是**静态徽标**，不是第二个控件。
    """

    connect_clicked = QtCore.pyqtSignal()
    settings_changed = QtCore.pyqtSignal()

    def __init__(self, settings: Settings, parent=None):
        super().__init__(parent)
        self.s = settings
        outer = QtWidgets.QVBoxLayout(self)
        outer.setContentsMargins(14, 14, 14, 14)
        outer.setSpacing(12)

        card = Card("连接")
        self.card = card

        # ── 角色 ──
        self.seg_role = SegmentedControl(["主臂（发布）", "从臂（跟随）"],
                                         0 if settings.role == ROLE_MASTER else 1)
        self.seg_role.changed.connect(self._role_changed)
        card.add_row("角色", self.seg_role)

        # ── 地址 ──
        self.ed_peer = QtWidgets.QLineEdit(settings.peer)
        self.ed_peer.setFixedWidth(150)
        self.ed_peer.setPlaceholderText("主臂 IP")
        self.sp_port = _spin(settings.jport, 1, 65535, 96)
        self.ed_arm_id = QtWidgets.QLineEdit(settings.arm_id)
        self.ed_arm_id.setFixedWidth(96)
        card.add_row("地址", "IP", self.ed_peer, "端口", self.sp_port, "arm_id",
                     self.ed_arm_id)
        for w in (self.ed_peer, self.ed_arm_id):
            w.editingFinished.connect(self._emit_changed)
        self.sp_port.valueChanged.connect(self._emit_changed)

        # ── CDC 口 ──
        # ⚠ **两条以上同型号臂时必须显式选**（VID:PID 相同，自动挑会挑错且不报错）
        self.cb_port = QtWidgets.QComboBox()
        self.cb_port.setMinimumWidth(300)
        self.btn_rescan = QtWidgets.QPushButton("重新扫描")
        self.btn_rescan.setProperty("variant", "outline")
        self.btn_rescan.clicked.connect(self.rescan_ports)
        card.add_row("CDC 口", self.cb_port, self.btn_rescan)
        self.cb_port.currentIndexChanged.connect(self._emit_changed)
        self.lab_port_note = hint("")
        card.add(self.lab_port_note)

        card.add(separator())

        self.btn_connect = QtWidgets.QPushButton("连接臂")
        self.btn_connect.setProperty("variant", "primaryAction")
        self.btn_connect.setFixedWidth(132)
        self.btn_connect.clicked.connect(self.connect_clicked.emit)  # type: ignore[arg-type]
        self.lab_info = hint("未连接", "hint")
        card.add_row(self.btn_connect, self.lab_info)

        note = hint("⚠ 一个 CDC 口只允许一个进程（STM32 CDC 在 Linux 不独占，"
                    "第二个进程会分吃同一字节流）", "hintSubtle")
        card.add(note)

        # ⚠ 表单类页面**别让卡片拉满全宽** —— 1560px 宽的一行只有三个输入框，
        #   看上去像没做完。夹一个左对齐的定宽列。
        holder = QtWidgets.QHBoxLayout()
        card.setMaximumWidth(780)
        holder.addWidget(card, 0, QtCore.Qt.AlignTop)
        holder.addStretch(1)
        outer.addLayout(holder)
        outer.addStretch(1)

        self.rescan_ports()
        self._role_changed()

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
            self.lab_port_note.setProperty("role", "danger")
        elif n == 0:
            self.lab_port_note.setText("未检测到 STM32 CDC 口（检查 USB 与 dialout 权限）")
            self.lab_port_note.setProperty("role", "danger")
        else:
            self.lab_port_note.setText(f"检测到 1 条臂：{self.cb_port.itemText(1)}")
            self.lab_port_note.setProperty("role", "hintSubtle")
        self.lab_port_note.style().unpolish(self.lab_port_note)
        self.lab_port_note.style().polish(self.lab_port_note)

    def role(self) -> str:
        return ROLE_MASTER if self.seg_role.current() == 0 else ROLE_SLAVE

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
            self.lab_info.setProperty("role", "danger")
        elif s.connected:
            side = "监听" if s.role == ROLE_MASTER else f"连接 {self.s.peer}"
            self.lab_info.setText(f"✓ 已连接 · 固件 {s.firmware} · {side}:{self.s.jport}")
            self.lab_info.setProperty("role", "ok")
        else:
            self.lab_info.setText("未连接")
            self.lab_info.setProperty("role", "hint")
        self.lab_info.style().unpolish(self.lab_info)
        self.lab_info.style().polish(self.lab_info)
        # ⚠ 连上之后角色不可再改（`ArmWorker` 构造时吃掉了 role）
        self.seg_role.setEnabled(not s.connected)
        self.btn_connect.setEnabled(not s.connected)


# ══════════════════════════ 遥操页：左栏卡片 ══════════════════════════
class LinkCard(Card):
    """「主从链路」卡。

    出处：设计稿左栏第一张卡。承载两件事：
      1. 两张 peer 卡（主臂 / 从臂）+ 中间 `>>>` 箭头 + 各自的外形说明；
      2. 四个指标块。

    ⚠ 设计稿那四格是「发布/接收 Hz · 帧数 · **抖动 ms** · 看门狗」。本仓
    `Snapshot` 里**没有抖动**这个量 ⇒ 第三格换成**帧龄 (ms)**（真有的量），
    ⛔ 而不是把「抖动」永远显示成 `—`（那是个死格子）。
    """

    def __init__(self, parent=None):
        super().__init__("主从链路", parent=parent)
        self.badge = Badge("未连接", "outline")
        self.header.addWidget(self.badge)

        peers = QtWidgets.QHBoxLayout()
        peers.setSpacing(6)
        self.peer_master = self._peer_box("主臂", "tcp/{ip}:{port}", "零重力拖动 · 发布关节流")
        self.arrow = QtWidgets.QLabel(">>>")
        self.arrow.setStyleSheet(
            f"QLabel {{ {mono(13, 700)} color: {C['ink_faint']}; }}")
        self.peer_slave = self._peer_box("从臂", "tcp/{ip}:{port}", "joint_follow 跟随 · watchdog 保护")
        peers.addWidget(self.peer_master, 1)
        peers.addWidget(self.arrow, 0)
        peers.addWidget(self.peer_slave, 1)
        self.body.addLayout(peers)

        tiles = QtWidgets.QHBoxLayout()
        tiles.setSpacing(10)
        self.t_hz = MetricTile("发布/接收", "Hz")
        self.t_frames = MetricTile("帧数")
        self.t_age = MetricTile("帧龄", "ms")
        self.t_wd = MetricTile("看门狗")
        for t in (self.t_hz, self.t_frames, self.t_age, self.t_wd):
            tiles.addWidget(t)
        self.body.addLayout(tiles)

    def _peer_box(self, name: str, endpoint: str, desc: str) -> QtWidgets.QFrame:
        f = QtWidgets.QFrame()
        f.setStyleSheet(
            f"QFrame {{ background: {C['line_soft']}; border: 1px solid {C['line']};"
            f" border-radius: {RADIUS['md']}px; }}")
        lay = QtWidgets.QVBoxLayout(f)
        lay.setContentsMargins(8, 6, 8, 6)
        lay.setSpacing(2)
        top = QtWidgets.QHBoxLayout()
        top.setSpacing(5)
        lab = QtWidgets.QLabel(name)
        lab.setStyleSheet(f"QLabel {{ {sans(12.5, 600)} color: {C['ink']}; }}")
        # ⚠ 徽标**必须无条件建好并加进布局** —— 只在主臂那一侧加的话，
        #   从臂的 `f._local` 会是个从没进过布局的孤儿控件，
        #   而 `update_from` 还会去 `setVisible()` 它（看不见、也说不清为什么）。
        #   谁带「本机」由 `update_from` 按当前角色决定。
        local = Badge("", "info")
        top.addWidget(lab)
        top.addWidget(local)
        top.addStretch(1)
        lay.addLayout(top)
        ep = QtWidgets.QLabel(endpoint)
        ep.setStyleSheet(f"QLabel {{ {mono(10.5)} color: {C['ink_muted']}; }}")
        d = QtWidgets.QLabel(desc)
        d.setWordWrap(True)
        d.setStyleSheet(f"QLabel {{ {sans(10.5)} color: {C['ink_faint']}; }}")
        lay.addWidget(ep)
        lay.addWidget(d)
        f._ep = ep          # type: ignore[attr-defined]
        f._local = local    # type: ignore[attr-defined]
        return f

    def update_from(self, s: Snapshot, peer: str, jport: int) -> None:
        self.badge.set_state("未连接" if not s.connected else "已连接",
                             "ok" if s.connected else "outline")
        local_role = s.role or ROLE_MASTER
        # "本机" 徽标挂在**本机那一侧**上（设计稿里 主臂 带 本机）
        for f, name in ((self.peer_master, "主臂"), (self.peer_slave, "从臂")):
            is_local = (name == "主臂") == (local_role == ROLE_MASTER)
            f._local.setText("本机" if is_local else "")
            f._local.setVisible(bool(is_local))
        if local_role == ROLE_MASTER:
            self.peer_master._ep.setText(f"tcp/0.0.0.0:{jport}")
            self.peer_slave._ep.setText(f"{peer}:{jport}")
        else:
            self.peer_master._ep.setText(f"{peer}:{jport}")
            self.peer_slave._ep.setText(f"tcp/0.0.0.0:{jport}")
        self.arrow.setText("<<<" if (s.role == ROLE_SLAVE) else ">>>")

        master = (s.role or ROLE_MASTER) == ROLE_MASTER
        self.t_hz.lab.setText("发布" if master else "接收")
        self.t_hz.set_value(f"{s.state_hz:g}" if s.state_hz else "—")
        self.t_frames.lab.setText("已发帧数" if master else "已收帧数")
        self.t_frames.set_value(str(s.frames_sent if master else s.frames_received))
        self.t_age.lab.setText("匹配" if master else "帧龄")
        if master:
            self.t_age.set_value("已匹配" if s.matching else "未匹配",
                                 "ok" if s.matching else "bad")
        else:
            self.t_age.set_value(
                "—" if s.frame_age is None else f"{s.frame_age * 1000:.0f}")
        self.t_wd.set_value(str(s.watchdog_trips),
                            "bad" if s.watchdog_trips else "none")
        self.t_wd.setToolTip("watchdog 超时次数（超时即停止跟随，不自动重连）")


class ConnectStatusCard(Card):
    """「连接状态」卡（设计稿左栏最后一张）—— 只读的本机端点与主臂 ID。"""

    def __init__(self, parent=None):
        super().__init__("连接状态", parent=parent)
        self.badge = Badge("未连接", "outline")
        self.header.addWidget(self.badge)
        self.f_ep = ro_field("—", 150, "本机 zenoh 监听端点")
        self.f_arm = ro_field("—", 96, "arm_id —— 决定 topic litearm/v4/{arm_id}/teleop")
        self.body.addWidget(field_row("本机端点", self.f_ep))
        self.body.addWidget(field_row("主臂 ID", self.f_arm))

    def update_from(self, s: Snapshot, peer: str, jport: int, arm_id: str) -> None:
        self.badge.set_state("已连接" if s.connected else "未连接",
                             "ok" if s.connected else "outline")
        host = peer if (s.role or ROLE_MASTER) == ROLE_SLAVE else "0.0.0.0"
        self.f_ep.setText(f"tcp/{host}:{jport}")
        self.f_arm.setText(arm_id)
        self.f_arm.setToolTip(f"topic = litearm/v4/{arm_id}/teleop")


# ══════════════════════════ 遥操页 ══════════════════════════
class TeleopPage(QtWidgets.QWidget):
    """遥操台：**三列**（照设计稿）。

    | 列 | 内容 |
    | --- | --- |
    | 左（窄，可滚动） | 主从链路卡 · 曲线卡 · 关节表卡 · 连接状态卡 |
    | 中（吃掉余量） | 机械臂遥操卡 · 夹爪遥操卡 |
    | 右（窄） | 大 STOP · 节点日志 |

    ⚠ 三列的 flex 语义照 `studio/src/lib/responsive.ts:11-14`：
    **左右固定、中间 flex:1 吃掉全部余量**（不是等分）。
    """

    teleop_toggled = QtCore.pyqtSignal(bool)
    grip_toggled = QtCore.pyqtSignal(bool)
    grip_settings_changed = QtCore.pyqtSignal()

    def __init__(self, settings: Optional[Settings] = None, parent=None):
        super().__init__(parent)
        self.gs = settings if settings is not None else Settings()
        outer = QtWidgets.QVBoxLayout(self)
        outer.setContentsMargins(14, 14, 14, 14)
        outer.setSpacing(14)

        cols = QtWidgets.QHBoxLayout()
        cols.setSpacing(14)
        outer.addLayout(cols, 1)

        # ── 左栏 ──
        self.left = ScrollColumn()
        self.left.setMinimumWidth(400)
        self.left.setMaximumWidth(470)
        self.link_card = LinkCard()
        self.metrics = MetricsCard()
        self.joints_card = Card("关节（q / dq / tau / 温度 / err）")
        self.joints = JointTable(compact=True)
        self.joints_card.add(self.joints)
        self.joints_card.add(hint(
            "⚠ `err != 1` 才高亮（1 = 使能，其余 = 失能/故障）—— 温度只报数值，"
            "不另判阈值（固件的温度锁存已反映在 err 上）", "hintSubtle"))
        self.status_card = ConnectStatusCard()
        for w in (self.link_card, self.metrics, self.joints_card, self.status_card):
            self.left.add(w)
        self.left.add_stretch()
        cols.addWidget(self.left, 0)

        # ── 中栏（吃余量）──
        mid = QtWidgets.QWidget()
        mid.setMinimumWidth(430)
        ml = QtWidgets.QVBoxLayout(mid)
        ml.setContentsMargins(0, 0, 0, 0)
        ml.setSpacing(12)
        ml.addWidget(self._build_arm_card())
        ml.addWidget(self._build_grip_card())
        ml.addStretch(1)
        cols.addWidget(mid, 1)

        # ── 右栏 ──
        right = QtWidgets.QWidget()
        right.setMinimumWidth(300)
        right.setMaximumWidth(380)
        rl = QtWidgets.QVBoxLayout(right)
        rl.setContentsMargins(0, 0, 0, 0)
        rl.setSpacing(12)

        self.stop = StopButton("STOP")
        self.stop.setToolTip(
            "停止遥操 —— **受控接管**：臂保持使能、由 movej 慢慢停住并持位。\n"
            "⚠ 这与「⛔ 急停」是两件事：急停是全失能 ⇒ 臂**自由落体**。")
        self.stop.clicked.connect(lambda: self.teleop_toggled.emit(False))
        rl.addWidget(self.stop)

        self.log_card = Card("节点日志")
        self.log_badge = Badge("—", "outline")
        self.log_card.header.addWidget(self.log_badge)
        self.log = QtWidgets.QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(500)
        self.log_card.add(self.log)
        rl.addWidget(self.log_card, 1)
        cols.addWidget(right, 0)

    # ── 机械臂遥操卡 ──
    def _build_arm_card(self) -> Card:
        card = Card("机械臂遥操")
        self.role_badge = Badge("本机角色 主臂（发布）", "outline")
        card.header.addWidget(self.role_badge)

        card.add(hint("本机订阅主臂关节流并跟随（slave）；需填写主臂 IP 与主臂 ID。",
                      "hint"))

        self.ro_peer = ro_field(self.gs.peer, 150, "来自「连接」页 —— 连上后不可改")
        self.ro_port = ro_field(str(self.gs.jport), 96, "来自「连接」页 —— 连上后不可改")
        self.ro_arm = ro_field(self.gs.arm_id, 96, "来自「连接」页 —— 连上后不可改")
        card.add_row("主臂 IP", self.ro_peer, "端口", self.ro_port, "主臂 ID", self.ro_arm)
        card.add_row("", rc_label("↑ 只读：这三项在「连接」页设置，`ArmWorker` 构造时定死"),
                     stretch_at_end=False)

        # 对齐 / watchdog（只读生效值）
        self.ro_align_speed = ro_field(f"{servo.ALIGN_SPEED:g}", 72,
                                       "`servo.ALIGN_SPEED`（常量，改它属于调参）")
        self.ro_watchdog = ro_field(f"{WATCHDOG_MS:g}", 72,
                                    "`arm_worker.WATCHDOG_MS`（常量，改它属于调参）")
        card.add_row("启动时对齐", self.ro_align_speed, "rad/s（固定开启）",
                     "watchdog", self.ro_watchdog, "ms")

        # 跟随调参（只读生效值）
        k_big = f"{servo.SETUP_K[0]:g}"
        k_wrist = f"{servo.SETUP_K[4]:g}"
        b_big = f"{servo.SETUP_B[1]:g}"
        b_wrist = f"{servo.SETUP_B[4]:g}"
        tip = ("逐帧随 0x08 下发的刚度/阻尼（**不是**固件出厂值）。"
               "真值在 `servo.SETUP_K` / `servo.SETUP_B`，改它属于调参")
        self.ro_k1 = ro_field(k_big, 72, tip)
        self.ro_k2 = ro_field(k_wrist, 72, tip)
        self.ro_b1 = ro_field(b_big, 72, tip)
        self.ro_b2 = ro_field(b_wrist, 72, tip)
        card.add_row("跟随调参", "K 大", self.ro_k1, "K 腕", self.ro_k2,
                     "B 大", self.ro_b1, "B 腕", self.ro_b2)
        card.add(hint(f"见回摆→加大 B；追不上→加大 K；啸叫→降 K（大关节 J1~J4 / "
                      f"腕部 J5~J7 两档，改完需重新启动生效）　⛔ 本版只读",
                      "hintSubtle"))

        card.add(separator())

        # 安全确认 + 启动
        self.chk_ok = QtWidgets.QCheckBox("☑ 我已确认机械臂周围无障碍、急停可及")
        self.chk_ok.setChecked(False)
        card.add(self.chk_ok)

        self.btn_teleop = QtWidgets.QPushButton("▶ 启动遥操")
        self.btn_teleop.setEnabled(False)
        self.btn_teleop.setCheckable(True)          # ⚠ 必须在 connect 之前
        self.btn_teleop.setProperty("variant", "primaryAction")
        # ⚠⚠ 不要写 `not isChecked()`：Qt 的 `clicked` 是在按钮状态**已经切换之后**
        #     才发出的 ⇒ 此刻 `isChecked()` **就是**用户想要的新值。
        #     加 `not` 会把"启动"发成"停止"（我这么错过一次，真机上表现为"点了没反应"）。
        self.btn_teleop.clicked.connect(
            lambda: self.teleop_toggled.emit(self.btn_teleop.isChecked()))
        self.lab_arm = hint("未连接", "hint")
        card.add_row(self.lab_arm)
        row = QtWidgets.QHBoxLayout()
        row.addStretch(1)
        row.addWidget(self.btn_teleop)
        card.body.addLayout(row)

        card.add(hint("⚠ 从臂启动前会先低速对齐到主臂位置；watchdog 超时自动停止跟随。",
                      "hintSubtle"))
        return card

    # ── 夹爪遥操卡 ──
    def _build_grip_card(self) -> Card:
        card = Card("夹爪遥操")
        self.grip_badge = Badge("未启动", "outline")
        card.header.addWidget(self.grip_badge)
        card.add(hint("从夹爪订阅主夹爪开合度并跟随；需填写主夹爪 IP 与主夹爪 ID。",
                      "hint"))

        self.ed_gcan = QtWidgets.QLineEdit(self.gs.gcan)
        # ⚠ `can0` 只是**占位提示**，不是默认值 —— 未填通道 ⇒ 不启用夹爪遥操。
        #    在显示时把 `can0` 写进 `gcan` 会把「默认不启用」静默变成「默认启用」。
        self.ed_gcan.setPlaceholderText("can0")
        self.ed_gcan.setFixedWidth(96)
        self.sp_gport = _spin(self.gs.gport, 1, 65535, 96)
        self.ed_grip_id = QtWidgets.QLineEdit(self.gs.grip_id)
        self.ed_grip_id.setFixedWidth(96)
        self.ed_gpeer = QtWidgets.QLineEdit(self.gs.gpeer)
        self.ed_gpeer.setFixedWidth(140)
        card.add_row("本夹爪通道", self.ed_gcan, "端口", self.sp_gport,
                     "grip_id", self.ed_grip_id)
        card.add_row("主夹爪 IP", self.ed_gpeer)

        found = can_channels()
        card.add(hint(
            "⚠ 通道留空 = <b>不启用</b>夹爪遥操。夹爪的 CAN 口与臂的 CDC 口是两回事。"
            + (f"　本机可用：{', '.join(found)}" if found
               else "　本机未发现 <code>can*</code> 接口（先 "
                    "<code>ip link set can0 up</code>）"), "hintSubtle"))

        # 只读的 SDK 侧参数（本版没接线）
        gtip = "`GripWorker` 的构造默认值 —— 本版没接到界面上，故只读"
        self.ro_rate = ro_field("—", 72, gtip)
        self.ro_kp = ro_field("—", 72, gtip)
        self.ro_kd = ro_field("—", 72, gtip)
        card.add_row("频率", self.ro_rate, "Hz", "kp", self.ro_kp, "kd", self.ro_kd)

        self.chk_align = QtWidgets.QCheckBox("启动时对齐")
        self.chk_align.setChecked(True)
        # ⚠ 这条按钮**有意不接**安全确认那个勾选：那个勾选是臂安全的闸门
        #   （臂会自由落体），而夹爪不会。夹爪自己的保护是行程钳位 + 主从标定一致性
        #   告警；用户裁决夹爪与臂**代码上完全分开**。**这是有意的，不是漏了。**
        self.btn_grip = QtWidgets.QPushButton("▶ 启动夹爪遥操")
        self.btn_grip.setCheckable(True)
        self.btn_grip.setProperty("variant", "primaryAction")
        # ⚠⚠ **不要在这里 `setEnabled(False)`。** 与臂的 `btn_teleop` 不同：
        #    臂的 worker 在「连接臂」时就建好了；而夹爪的 `GripWorker`
        #    **只能由点这个按钮来创建**（`main_window._ensure_grip_worker`）
        #    ⇒ 若初始禁用、且 `apply_grip` 又只按 `connected` 启停，
        #    就**永远点不了**（死锁，只能重启应用）。
        self.btn_grip.clicked.connect(
            lambda: self.grip_toggled.emit(self.btn_grip.isChecked()))
        self.lab_grip = hint("未启动", "hint")
        self.lab_grip_mismatch = hint("", "danger")
        card.add_row(self.chk_align, self.btn_grip)
        card.add(self.lab_grip)
        card.add(self.lab_grip_mismatch)

        for w in (self.ed_gcan, self.ed_grip_id, self.ed_gpeer):
            w.editingFinished.connect(self.grip_settings_changed.emit)
        self.sp_gport.valueChanged.connect(self.grip_settings_changed.emit)
        self.chk_align.toggled.connect(self.grip_settings_changed.emit)
        return card

    # ── 值 ──
    def gripper_values(self) -> dict:
        """把夹爪分区的当前值读出来。

        ⚠ `gcan` **不做默认值回填** —— 空串就是「不启用夹爪遥操」这个安全默认
        （spec §9.1）。`can0` 只在输入框里当占位提示，不进设置。
        """
        return {
            "gcan": self.ed_gcan.text().strip(),
            "gpeer": self.ed_gpeer.text().strip() or "127.0.0.1",
            "gport": int(self.sp_gport.value()),
            "grip_id": self.ed_grip_id.text().strip() or "gripA",
            "align": bool(self.chk_align.isChecked()),
        }

    def set_grip_defaults(self, rate_hz, kp, kd) -> None:
        """把夹爪 SDK 侧的**只读**默认值显示出来（本版没接线，只是别显示 `—`）。"""
        self.ro_rate.setText(f"{rate_hz:g}")
        self.ro_kp.setText("默认" if kp is None else f"{kp:g}")
        self.ro_kd.setText("默认" if kd is None else f"{kd:g}")

    def apply(self, s: Snapshot, peer: str, jport: int, arm_id: str) -> None:
        master = (s.role or ROLE_MASTER) == ROLE_MASTER
        self.role_badge.setText(f"本机角色 {'主臂（发布）' if master else '从臂（跟随）'}")
        self.link_card.update_from(s, peer, jport)
        self.status_card.update_from(s, peer, jport, arm_id)
        self.metrics.set_snapshot(s)

        # ⚠⚠ **必须走 `JointTable.update_from`**，不要在这里手写各列 ——
        #    我第一版就是手写的，漏了第 7 列（err）与它的红/绿高亮，
        #    于是「`err != 1` 才高亮」这条**安全判据在界面上整个消失了**
        #    （表里永远显示 `—`，失能/故障的轴不会变红）。
        #    那份判据只有一份真源，就在 `widgets.JointTable.update_from` 里。
        self.joints.update_from(s)

        if s.error:
            self.lab_arm.setText(f"⛔ {s.error}")
            self.lab_arm.setProperty("role", "danger")
        elif s.teleop_active:
            if master:
                self.lab_arm.setText(
                    f"主臂：零重力拖动中 · 发布 {PUB_HZ:.0f} Hz · 已发 {s.frames_sent} 帧 · "
                    f"{'已匹配订阅者' if s.matching else '⚠ 未匹配（发了没人在收）'}")
            else:
                age = "—" if s.frame_age is None else f"{s.frame_age * 1000:.0f} ms"
                self.lab_arm.setText(
                    f"从臂：跟随中 · 环频 {SLAVE_HZ:.0f} Hz · 已收 {s.frames_received} 帧 · "
                    f"帧龄 {age} · watchdog 超时 {s.watchdog_trips}")
        else:
            self.lab_arm.setText("未启动遥操")
        self.lab_arm.setProperty("role", "danger" if s.error else "hint")
        self.lab_arm.style().unpolish(self.lab_arm)
        self.lab_arm.style().polish(self.lab_arm)

        # ⚠ 按钮文本/勾选按**真实状态**刷新，不是按点击
        self.btn_teleop.setText("⏸ 停止遥操" if s.teleop_active else "▶ 启动遥操")
        self.btn_teleop.setChecked(bool(s.teleop_active))

    def apply_grip(self, g) -> None:
        """夹爪快照 → 界面。

        ⚠ 与 `apply()` **分开**：夹爪状态由**另一条**信号
        （`WorkerBridge.grip_state`）送来，两条链路不共享任何对象（spec §2）。
        """
        if g.error:
            self.lab_grip.setText(f"⛔ {g.error}")
            self.grip_badge.set_state("出错", "danger")
        elif not g.connected:
            self.lab_grip.setText("未启动")
            self.grip_badge.set_state("未启动", "outline")
        elif g.role == ROLE_MASTER:
            self.lab_grip.setText(
                f"主端夹爪：零重力 · {g.topic} · 发 {g.frames_sent} 帧 · "
                f"开合 {g.openness:.2f} · {g.position_mm:.1f} mm · "
                f"力 {g.force_n:.1f} N · "
                + ("已匹配订阅者" if g.matching else "⚠ 未匹配（发了没人在收）"))
            self.grip_badge.set_state(
                "已匹配" if g.matching else "未匹配", "ok" if g.matching else "warn")
        else:
            age = "—" if g.frame_age is None else f"{g.frame_age * 1000:.0f} ms"
            self.lab_grip.setText(
                f"从端夹爪：{'持位（watchdog 超时）' if g.stale else '跟随中'} · "
                f"环频 {g.loop_hz:.0f} Hz · 收 {g.frames_received} 帧 · 帧龄 {age} · "
                f"开合 {g.openness:.2f} · {g.position_mm:.1f} mm · "
                f"力 {g.force_n:.1f} N")
            self.grip_badge.set_state("持位" if g.stale else "跟随中",
                                      "warn" if g.stale else "ok")
        # ⚠ 三类"静默失败"都要**看得见**：丢弃的坏帧、发不出去的帧、夹爪自己的故障码
        self.lab_grip_mismatch.setText("　".join(x for x in (
            (f"⚠ 已丢弃 {g.rejected} 条非有限值帧（NaN/Inf）—— 保持不动"
             if g.rejected else ""),
            (f"⚠ {g.send_failed} 次 send_mit_frame 返回 False —— 夹爪可能没在动"
             if g.send_failed else ""),
            (f"⛔ {g.fault}" if g.fault else ""),
            g.mismatch) if x))
        # ⚠ 按钮文本/勾选按**真实状态**刷新，不是按点击 —— 与 `apply()` 同款。
        # ⚠⚠ 但**不碰 `setEnabled`**：见 `_build_grip_card` 里的死锁说明。
        self.btn_grip.setText("⏸ 停止夹爪遥操" if g.teleop_active else "▶ 启动夹爪遥操")
        self.btn_grip.setChecked(bool(g.teleop_active))


# ══════════════════════════ 日志页 ══════════════════════════
class LogsPage(QtWidgets.QWidget):
    """全屏日志（左侧导航「日志」）。

    ⚠ 与遥操页右栏那个「节点日志」是**两份控件、同一份来源** ——
    `MainWindow._log()` 往两个 `QPlainTextEdit` 各写一次（保留上限不同）。
    Qt 里一个 `QWidget` 不能同时挂在两个布局上，所以只能这样。
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        outer = QtWidgets.QVBoxLayout(self)
        outer.setContentsMargins(14, 14, 14, 14)
        outer.setSpacing(12)
        card = Card("运行日志")
        card.add(hint(
            "⚠ 一个 CDC 口只允许一个进程。急停 = 全失能 ⇒ 臂**自由落体**；"
            "要「稳住」请用停止遥操（受控接管 movej）。", "hintSubtle"))
        self.view = QtWidgets.QPlainTextEdit()
        self.view.setReadOnly(True)
        self.view.setMaximumBlockCount(5000)
        card.add(self.view)
        outer.addWidget(card)


# ══════════════════════════ 设置页 ══════════════════════════
class SettingsPage(QtWidgets.QWidget):
    """设置页：末端载荷（夹爪）。

    为什么载荷放这儿而不是遥操台：它是**设一次、长期生效**的装置参数
    （改的是固件里的重力前馈），不是每次遥操都要碰的操作项。
    studio 的 `SettingsPage` 也是同一个分法（夹爪那类装置参数进设置页）。
    """

    #: (mass_kg, [x, y, z]) —— 由 main_window 接到 `worker.set_payload`
    payload_applied = QtCore.pyqtSignal(float, object)

    def __init__(self, parent=None):
        super().__init__(parent)
        outer = QtWidgets.QVBoxLayout(self)
        outer.setContentsMargins(14, 14, 14, 14)
        outer.setSpacing(12)

        box = Card("末端载荷（夹爪）—— 影响重力前馈")
        # ⚠ 用**嵌套 QHBoxLayout**，不用 QGridLayout —— 网格是**共享列**的，
        #    上一行的"质心 x"标签会和下一行按钮占同一列 ⇒ 宽度互相挤，标签重叠。
        self.sp_mass = QtWidgets.QDoubleSpinBox()
        self.sp_mass.setRange(0.0, 20.0)
        self.sp_mass.setDecimals(3)
        self.sp_mass.setSingleStep(0.05)
        self.sp_mass.setSuffix(" kg")
        self.sp_mass.setFixedWidth(110)
        self.sp_mass.setValue(servo.DEFAULT_PAYLOAD_MASS)
        self.sp_com = []
        row = QtWidgets.QHBoxLayout()
        row.setSpacing(8)
        row.addWidget(QtWidgets.QLabel("质量"))
        row.addWidget(self.sp_mass)
        for k, name in enumerate(("质心 X", "质心 Y", "质心 Z")):
            row.addSpacing(10)
            lab = QtWidgets.QLabel(name)
            lab.setProperty("role", "fieldLabel")
            row.addWidget(lab)
            sp = QtWidgets.QDoubleSpinBox()
            sp.setRange(-1.0, 1.0)
            sp.setDecimals(4)
            sp.setSingleStep(0.005)
            sp.setSuffix(" m")
            sp.setFixedWidth(105)
            sp.setValue(servo.DEFAULT_PAYLOAD_COM[k])
            self.sp_com.append(sp)
            row.addWidget(sp)
        row.addStretch(1)
        box.body.addLayout(row)
        box.add(hint("⚠ 质心在 <b>ee_link 系</b>、单位<b>米</b>；"
                     "本项目夹爪的 3 cm 在 <b>Z</b> 轴", "hint"))

        self.btn_payload = QtWidgets.QPushButton("应用载荷")
        self.btn_payload.setProperty("variant", "primaryAction")
        self.btn_gripper = QtWidgets.QPushButton("预设：夹爪 600 g / Z 3 cm")
        self.btn_gripper.setProperty("variant", "outline")
        self.btn_gripper.clicked.connect(self._preset_gripper)
        self.btn_payload.clicked.connect(self._apply_payload)
        box.add_row(self.btn_payload, self.btn_gripper)

        self.lab_payload = hint("当前生效：—")
        box.add(self.lab_payload)
        box.add(hint("⚠ 固件会**静默钳制**（质量 [0,20]、质心 [-1,1]）且照样回 ACK ⇒ "
                     "上面显示的是**读回值**，不是输入值。⛔ 只写 RAM，断电即还原。",
                     "hintSubtle"))
        holder = QtWidgets.QHBoxLayout()
        box.setMaximumWidth(780)
        holder.addWidget(box, 0, QtCore.Qt.AlignTop)
        holder.addStretch(1)
        outer.addLayout(holder)
        outer.addStretch(1)

    def _preset_gripper(self) -> None:
        self.sp_mass.setValue(servo.DEFAULT_PAYLOAD_MASS)
        for k, v in enumerate(servo.DEFAULT_PAYLOAD_COM):
            self.sp_com[k].setValue(v)

    def _apply_payload(self) -> None:
        self.payload_applied.emit(
            float(self.sp_mass.value()),
            [float(sp.value()) for sp in self.sp_com])

    def apply(self, s: Snapshot) -> None:
        if s.payload_com:
            self.lab_payload.setText(
                f"当前生效：<b>{s.payload_mass:.3f} kg</b>，质心 "
                f"[{', '.join(f'{v:+.4f}' for v in s.payload_com)}] m"
                + ("　⚠ 全是 0 ⇒ 可能没设上" if s.payload_mass == 0.0 else ""))
        else:
            self.lab_payload.setText("当前生效：—（未连接或读不到）")
