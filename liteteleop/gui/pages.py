"""**单页**控制台 —— 版式照设计稿，视觉语言照 `litearm-studio`。

⚠ 2026-09-29 用户裁决：**去掉左侧导航栏**（连接/遥操/日志三项"没有意义"，
只要遥操这一个页面）。于是：

- 「连接」并进中栏的**机械臂遥操卡**（而且那几个字段从此**真正可编辑** ——
  原先它们在独立页上，这里只读是权宜之计）；
- 「日志」本来就在右栏（节点日志卡），独立日志页删掉；
- 「末端载荷」原本挂在左栏的"设置"里，现在下移到中栏底部；
- `shell.RailNav` 整条删掉，`main_window` 不再有 `QStackedWidget`。

三列（照设计稿）：

| 列 | 内容 |
| --- | --- |
| 左（窄，可滚动） | 主从链路 · 曲线卡 · 关节表 · 连接状态 |
| 中（吃掉余量，可滚动） | 机械臂遥操（含连接）· 夹爪遥操 · 末端载荷 |
| 右（窄） | 大 STOP · 节点日志 |

## ⚠⚠ 一条硬纪律：**不许画"看起来能改、其实没接线"的输入框**

臂的 K/B 增益、对齐速度、watchdog、夹爪的频率/kp/kd —— 这些**本版没接到任何东西上**，
一律用 `ro_field()` 渲染成**只读的"当前生效值"**，tooltip 里写明出处。
⛔ 别图省事画成可编辑的 `QLineEdit`：那会让界面在撒谎。

（**连接**那几项是例外 —— 它们是真接线的，所以是真输入框。见 `_build_arm_card` 的注释。）

## ⚠ 设计稿有、本仓不做的

- 左侧导航栏（用户裁决去掉）。
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

__all__ = ["TeleopPage", "can_channels"]


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


def rc_label(text: str = "只读") -> QtWidgets.QLabel:
    w = QtWidgets.QLabel(text)
    w.setStyleSheet(f"QLabel {{ {sans(10.5)} color: {C['ink_faint']}; }}")
    return w


def _spin(val: int, lo: int, hi: int, width: int = 88) -> QtWidgets.QSpinBox:
    w = QtWidgets.QSpinBox()
    w.setRange(lo, hi)
    w.setValue(val)
    w.setFixedWidth(width)
    return w


# ══════════════════════════ 左栏卡片 ══════════════════════════
class LinkCard(Card):
    """「主从链路」卡（设计稿左栏第一张）。

    两张 peer 卡（主臂 / 从臂）+ 中间箭头 + 各自的外形说明；下面四个指标块。

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
        self.peer_master = self._peer_box("主臂", "零重力拖动 · 发布关节流")
        self.arrow = QtWidgets.QLabel(">>>")
        self.arrow.setStyleSheet(f"QLabel {{ {mono(13, 700)} color: {C['ink_faint']}; }}")
        self.peer_slave = self._peer_box("从臂", "joint_follow 跟随 · watchdog 保护")
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

    def _peer_box(self, name: str, desc: str) -> QtWidgets.QFrame:
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
        # ⚠ 徽标**必须无条件建好并加进布局** —— 只在主臂那侧加的话，从臂的
        #   `f._local` 会是个从没进过布局的孤儿控件，而 `update_from` 还会去
        #   `setVisible()` 它（看不见、也说不清为什么）。谁带「本机」由 `update_from` 定。
        local = Badge("", "info")
        top.addWidget(lab)
        top.addWidget(local)
        top.addStretch(1)
        lay.addLayout(top)
        ep = QtWidgets.QLabel("—")
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
        self.arrow.setText("<<<" if local_role == ROLE_SLAVE else ">>>")

        master = local_role == ROLE_MASTER
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
        self.t_wd.set_value(str(s.watchdog_trips), "bad" if s.watchdog_trips else "none")
        self.t_wd.setToolTip("watchdog 超时次数（超时即停止跟随，不自动重连）")


class ConnectStatusCard(Card):
    """「连接状态」卡（设计稿左栏最后一张）—— 只读的本机端点与主臂 ID。"""

    def __init__(self, parent=None):
        super().__init__("连接状态", parent=parent)
        self.badge = Badge("未连接", "outline")
        self.header.addWidget(self.badge)
        self.f_ep = ro_field("—", 150, "本机 zenoh 监听端点（从臂时为主臂端点）")
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


# ══════════════════════════ 唯一的一页 ══════════════════════════
class TeleopPage(QtWidgets.QWidget):
    """控制台（**唯一页面**）：顶栏以下全归它。

    ⚠ 中栏现在是**连接 + 臂遥操 + 夹爪遥操 + 末端载荷**四张卡，比一屏高 ⇒
    中栏也是滚动列（照 studio `responsive.ts:18` 的 `SCROLL_COLUMN`：
    三列都是"自己滚自己的"）。
    """

    teleop_toggled = QtCore.pyqtSignal(bool)
    grip_toggled = QtCore.pyqtSignal(bool)
    grip_settings_changed = QtCore.pyqtSignal()
    connect_clicked = QtCore.pyqtSignal()
    settings_changed = QtCore.pyqtSignal()
    #: (mass_kg, [x, y, z]) —— 由 main_window 接到 `worker.set_payload`
    payload_applied = QtCore.pyqtSignal(float, object)

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

        # ── 中栏（吃余量，可滚动）──
        self.mid = ScrollColumn()
        self.mid.setMinimumWidth(470)
        self.mid.add(self._build_arm_card())
        self.mid.add(self._build_grip_card())
        self.mid.add(self._build_payload_card())
        self.mid.add_stretch()
        cols.addWidget(self.mid, 1)

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
            "⚠ 这与顶栏的「⛔ 急停」是两件事：急停是全失能 ⇒ 臂**自由落体**。")
        self.stop.clicked.connect(lambda: self.teleop_toggled.emit(False))
        rl.addWidget(self.stop)

        self.log_card = Card("节点日志")
        self.log_badge = Badge("—", "outline")
        self.log_card.header.addWidget(self.log_badge)
        self.log = QtWidgets.QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(2000)
        self.log_card.add(self.log)
        rl.addWidget(self.log_card, 1)
        cols.addWidget(right, 0)

    # ── 机械臂遥操卡（**含连接**）──
    def _build_arm_card(self) -> Card:
        """臂这一张卡：**连接参数 + 遥操参数 + 两个按钮**。

        ⚠⚠ 为什么把「连接」并进来：用户裁决去掉独立页之后，这些参数总得有个家；
        而它们本来就只服务于臂遥操这一件事。
        ⚠⚠ 于是这几格从"只读生效值"变回**真输入框** —— 它们是真接线的：
        `_connect()` 会读 `role()/ed_peer/sp_port/ed_arm_id/cb_port` 去构造 `ArmWorker`。
        ⛔ 别再改回只读：那会让"改了没反应"这件事重新出现。
        ⚠ 但**连上之后必须锁死** —— `ArmWorker` 在构造时就把这几项吃掉了。
        """
        card = Card("机械臂遥操")
        self.role_badge = Badge("本机角色 主臂（发布）", "outline")
        card.header.addWidget(self.role_badge)

        card.add(hint("主臂零重力拖动、发布关节流；从臂订阅并跟随。"
                      "两端的「主臂 ID」与端口必须一致。", "hint"))

        # ── 角色（连上后不可改）──
        self.seg_role = SegmentedControl(
            ["主臂（发布）", "从臂（跟随）"],
            0 if self.gs.role == ROLE_MASTER else 1)
        self.seg_role.changed.connect(self._role_changed)
        card.add_row("本机角色", self.seg_role)

        # ── 地址（真输入框）──
        self.ed_peer = QtWidgets.QLineEdit(self.gs.peer)
        self.ed_peer.setFixedWidth(140)
        self.ed_peer.setPlaceholderText("主臂 IP（从臂填）")
        self.sp_port = _spin(self.gs.jport, 1, 65535, 96)
        self.ed_arm_id = QtWidgets.QLineEdit(self.gs.arm_id)
        self.ed_arm_id.setFixedWidth(92)
        card.add_row("主臂 IP", self.ed_peer, "端口", self.sp_port,
                     "主臂 ID", self.ed_arm_id)
        for w in (self.ed_peer, self.ed_arm_id):
            w.editingFinished.connect(self._emit_changed)
        self.sp_port.valueChanged.connect(self._emit_changed)

        # ── CDC 口（真输入框 + 重扫）──
        # ⚠ **两条以上同型号臂时必须显式选**（VID:PID 相同，自动挑会挑错且不报错）
        self.cb_port = QtWidgets.QComboBox()
        self.cb_port.setMinimumWidth(300)
        self.btn_rescan = QtWidgets.QPushButton("重新扫描")
        self.btn_rescan.setProperty("variant", "outline")
        self.btn_rescan.clicked.connect(self.rescan_ports)
        self.cb_port.currentIndexChanged.connect(self._emit_changed)
        card.add_row("CDC 口", self.cb_port, self.btn_rescan)
        self.lab_port_note = hint("")
        card.add(self.lab_port_note)

        # ── 连接 ──
        self.btn_connect = QtWidgets.QPushButton("连接臂")
        self.btn_connect.setProperty("variant", "primaryAction")
        self.btn_connect.setFixedWidth(120)
        self.btn_connect.clicked.connect(self.connect_clicked.emit)  # type: ignore[arg-type]
        self.lab_info = hint("未连接", "hint")
        card.add_row(self.btn_connect, self.lab_info)

        card.add(separator())

        # ── 只读的生效值（本版没接线）──
        self.ro_align_speed = ro_field(f"{servo.ALIGN_SPEED:g}", 72,
                                       "`servo.ALIGN_SPEED`（常量，改它属于调参）")
        self.ro_watchdog = ro_field(f"{WATCHDOG_MS:g}", 72,
                                    "`arm_worker.WATCHDOG_MS`（常量，改它属于调参）")
        card.add_row("启动时对齐", self.ro_align_speed, "rad/s（固定开启）",
                     "watchdog", self.ro_watchdog, "ms")

        tip = ("逐帧随 0x08 下发的刚度/阻尼（**不是**固件出厂值）。"
               "真值在 `servo.SETUP_K` / `servo.SETUP_B`，改它属于调参")
        self.ro_k1 = ro_field(f"{servo.SETUP_K[0]:g}", 72, tip)
        self.ro_k2 = ro_field(f"{servo.SETUP_K[4]:g}", 72, tip)
        self.ro_b1 = ro_field(f"{servo.SETUP_B[1]:g}", 72, tip)
        self.ro_b2 = ro_field(f"{servo.SETUP_B[4]:g}", 72, tip)
        card.add_row("跟随调参", "K 大", self.ro_k1, "K 腕", self.ro_k2,
                     "B 大", self.ro_b1, "B 腕", self.ro_b2)
        card.add(hint("见回摆→加大 B；追不上→加大 K；啸叫→降 K"
                      "（大关节 J1~J4 / 腕部 J5~J7 两档）　⛔ 本版只读", "hintSubtle"))

        card.add(separator())

        # ── 安全确认 + 启动 ──
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
        card.add(self.lab_arm)
        row = QtWidgets.QHBoxLayout()
        row.addStretch(1)
        row.addWidget(self.btn_teleop)
        card.body.addLayout(row)
        card.add(hint("⚠ 从臂启动前会先低速对齐到主臂位置；watchdog 超时自动停止跟随。",
                      "hintSubtle"))
        self.rescan_ports()
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
        self.ed_gcan.setFixedWidth(92)
        self.sp_gport = _spin(self.gs.gport, 1, 65535, 96)
        self.ed_grip_id = QtWidgets.QLineEdit(self.gs.grip_id)
        self.ed_grip_id.setFixedWidth(92)
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

        gtip = "`GripWorker` 的构造默认值 —— 本版没接到界面上，故只读"
        self.ro_rate = ro_field("—", 72, gtip)
        self.ro_kp = ro_field("—", 72, gtip)
        self.ro_kd = ro_field("—", 72, gtip)
        card.add_row("频率", self.ro_rate, "Hz", "kp", self.ro_kp, "kd", self.ro_kd)

        self.chk_align = QtWidgets.QCheckBox("启动时对齐")
        self.chk_align.setChecked(True)
        self.btn_grip = QtWidgets.QPushButton("▶ 启动夹爪遥操")
        self.btn_grip.setCheckable(True)
        self.btn_grip.setProperty("variant", "primaryAction")
        # ⚠⚠ **不要在这里 `setEnabled(False)`。** 与臂的 `btn_teleop` 不同：
        #    臂的 worker 在「连接臂」时就建好了；而夹爪的 `GripWorker`
        #    **只能由点这个按钮来创建**（`main_window._ensure_grip_worker`）
        #    ⇒ 若初始禁用、且 `apply_grip` 又只按 `connected` 启停，
        #    就**永远点不了**（死锁，只能重启应用）。
        #
        # ⚠ 另外这条按钮**有意不接**顶栏那个安全确认勾选：那个勾选是臂安全的闸门
        #   （臂会自由落体），而夹爪不会。夹爪自己的保护是行程钳位 + 主从标定一致性
        #   告警；用户裁决夹爪与臂**代码上完全分开**。**这是有意的，不是漏了。**
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

    # ── 末端载荷卡（原「设置」页）──
    def _build_payload_card(self) -> Card:
        """末端载荷 —— **设一次、长期生效**的装置参数（改的是固件里的重力前馈），
        不是每次遥操都要碰的操作项。原来独立成页，去掉导航后收到这里。"""
        card = Card("末端载荷（夹爪）—— 影响重力前馈")
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
        card.body.addLayout(row)
        card.add(hint("⚠ 质心在 <b>ee_link 系</b>、单位<b>米</b>；"
                      "本项目夹爪的 3 cm 在 <b>Z</b> 轴", "hint"))

        self.btn_payload = QtWidgets.QPushButton("应用载荷")
        self.btn_payload.setProperty("variant", "primaryAction")
        self.btn_gripper = QtWidgets.QPushButton("预设：夹爪 600 g / Z 3 cm")
        self.btn_gripper.setProperty("variant", "outline")
        self.btn_gripper.clicked.connect(self._preset_gripper)
        self.btn_payload.clicked.connect(self._apply_payload)
        card.add_row(self.btn_payload, self.btn_gripper)

        self.lab_payload = hint("当前生效：—")
        card.add(self.lab_payload)
        card.add(hint("⚠ 固件会**静默钳制**（质量 [0,20]、质心 [-1,1]）且照样回 ACK ⇒ "
                      "上面显示的是**读回值**，不是输入值。⛔ 只写 RAM，断电即还原。",
                      "hintSubtle"))
        return card

    # ── 连接参数 ──
    def rescan_ports(self) -> None:
        """重新枚举 CDC 口。**序列号才是唯一标识**，所以标签里带上它。"""
        cur = self.gs.cdc_port
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
            role = "danger"
        elif n == 0:
            self.lab_port_note.setText("未检测到 STM32 CDC 口（检查 USB 与 dialout 权限）")
            role = "danger"
        else:
            self.lab_port_note.setText(f"检测到 1 条臂：{self.cb_port.itemText(1)}")
            role = "hintSubtle"
        self.lab_port_note.setProperty("role", role)
        self.lab_port_note.style().unpolish(self.lab_port_note)
        self.lab_port_note.style().polish(self.lab_port_note)

    def role(self) -> str:
        return ROLE_MASTER if self.seg_role.current() == 0 else ROLE_SLAVE

    def _role_changed(self) -> None:
        self.ed_peer.setEnabled(self.role() == ROLE_SLAVE)
        self._emit_changed()

    def _emit_changed(self) -> None:
        self.gs.role = self.role()
        self.gs.peer = self.ed_peer.text().strip() or "127.0.0.1"
        self.gs.jport = int(self.sp_port.value())
        self.gs.arm_id = self.ed_arm_id.text().strip() or "armA"
        self.gs.cdc_port = self.cb_port.currentData() or ""
        self.settings_changed.emit()

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

    def _preset_gripper(self) -> None:
        self.sp_mass.setValue(servo.DEFAULT_PAYLOAD_MASS)
        for k, v in enumerate(servo.DEFAULT_PAYLOAD_COM):
            self.sp_com[k].setValue(v)

    def _apply_payload(self) -> None:
        self.payload_applied.emit(
            float(self.sp_mass.value()),
            [float(sp.value()) for sp in self.sp_com])

    # ── 刷新 ──
    def apply(self, s: Snapshot, peer: str, jport: int, arm_id: str) -> None:
        master = (s.role or ROLE_MASTER) == ROLE_MASTER
        self.role_badge.setText(f"本机角色 {'主臂（发布）' if master else '从臂（跟随）'}")
        self.link_card.update_from(s, peer, jport)
        self.status_card.update_from(s, peer, jport, arm_id)
        self.metrics.set_snapshot(s)
        # ⚠⚠ 必须走 `JointTable.update_from`，不要在这里手写各列 ——
        #    我第一版就是手写的，漏了第 7 列（err）与它的红/绿高亮，
        #    于是「`err != 1` 才高亮」这条**安全判据在界面上整个消失了**。
        #    那份判据只有一份真源，就在 `widgets.JointTable.update_from` 里。
        self.joints.update_from(s)

        # 连接状态
        if s.error:
            self.lab_info.setText(f"⛔ {s.error}")
            info_role = "danger"
        elif s.connected:
            side = "监听" if master else f"连接 {peer}"
            self.lab_info.setText(f"✓ 已连接 · 固件 {s.firmware} · {side}:{jport}")
            info_role = "ok"
        else:
            self.lab_info.setText("未连接")
            info_role = "hint"
        self.lab_info.setProperty("role", info_role)
        self.lab_info.style().unpolish(self.lab_info)
        self.lab_info.style().polish(self.lab_info)
        # ⚠ 连上之后角色/地址/CDC 全部锁死 —— `ArmWorker` 构造时就把它们吃掉了
        for w in (self.seg_role, self.ed_peer, self.sp_port, self.ed_arm_id,
                  self.cb_port, self.btn_rescan):
            w.setEnabled(not s.connected)
        self.btn_connect.setEnabled(not s.connected)

        # 遥操状态文本
        if s.error:
            self.lab_arm.setText(f"⛔ {s.error}")
            arm_role = "danger"
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
            arm_role = "hint"
        else:
            self.lab_arm.setText("未启动遥操")
            arm_role = "hint"
        self.lab_arm.setProperty("role", arm_role)
        self.lab_arm.style().unpolish(self.lab_arm)
        self.lab_arm.style().polish(self.lab_arm)

        # ⚠ 按钮文本/勾选按**真实状态**刷新，不是按点击
        self.btn_teleop.setText("⏸ 停止遥操" if s.teleop_active else "▶ 启动遥操")
        self.btn_teleop.setChecked(bool(s.teleop_active))

        # 末端载荷（显示**读回值**，不是输入值 —— 固件会静默钳制）
        if s.payload_com:
            self.lab_payload.setText(
                f"当前生效：<b>{s.payload_mass:.3f} kg</b>，质心 "
                f"[{', '.join(f'{v:+.4f}' for v in s.payload_com)}] m"
                + ("　⚠ 全是 0 ⇒ 可能没设上" if s.payload_mass == 0.0 else ""))
        else:
            self.lab_payload.setText("当前生效：—（未连接或读不到）")

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
