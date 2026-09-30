"""主窗口：**顶栏 + 唯一一页**（单页控制台）。

⚠ 2026-09-29 用户裁决：去掉左侧导航栏、去掉页面栈 —— 「连接/遥操/日志三个分块
没有意义，只要遥操这一个页面」。连接并进中栏的「机械臂遥操」卡，日志本来就在
右栏，末端载荷从中栏底部那张卡进（原「设置」页）。

## ⚠⚠ 重做界面时**不许弄丢**的九条（都是真机上踩出来的）

1. **安全确认闸门**（`_relock`）：没勾「我已确认…」时运动按钮一律锁定；
   断线 / 急停 / FAULT 自动重新锁定。判据取**界面正在显示的那一份快照**。
2. **急停不走 worker 队列**（§7.4）：worker 可能正卡在收尾 `movej` 里。
3. **急停 ≠ 停止遥操**：急停是全失能 ⇒ 臂**自由落体**。右栏那个大 STOP 接的是
   **停止遥操**（用户裁决 2026-09-29），急停另放顶栏、小小一个。
4. **`clicked` 在按钮状态切换之后才发出** ⇒ 必须直接发 `isChecked()`，不许加 `not`。
5. **夹爪按钮不许初始禁用**：`GripWorker` 只能由点它创建 ⇒ 禁用即死锁。
6. **夹爪与臂是两条独立链路**：独立信号、独立 worker、不共享对象。
7. **`BaseException` 兜底**：夹爪 SDK 不可用时异常逃出 Qt 槽会**直接 abort 进程**。
8. **CDC 口两条以上必须显式选**（同型号 VID:PID 相同，自动挑会挑错且不报错）。
9. **收尾必须显式 `close()`**：zenoh 不关 ⇒ 解释器退不出去。
"""
from __future__ import annotations

import inspect
import sys
import time

from PyQt5 import QtCore, QtWidgets

from ..arm_worker import ROLE_MASTER, ROLE_SLAVE, ArmWorker, Snapshot, TeleopParams
from ..grip_worker import GripSnapshot, GripWorker
from ..settings import Settings, load_settings, save_settings
from . import theme
from .bridge import WorkerBridge
from .pages import TeleopPage
from .shell import TopBar

__all__ = ["MainWindow", "run"]


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, settings: Settings, parent=None):
        super().__init__(parent)
        self.s = settings
        self.setWindowTitle("LiteArm 同构遥操 · 主/从 · zenoh 点对点")
        self.resize(1560, 980)
        self.setStyleSheet(theme.qss())

        self.worker: ArmWorker | None = None
        #: ⚠ 夹爪是**另一条链路**（独立 CAN / session / 端口 / 开关，spec §2）——
        #: 与 `self.worker` 没有任何共享对象。
        self.grip: GripWorker | None = None
        #: ⚠ **界面显示的那一份**快照 —— `_relock` 必须读它，不能另去问 worker，
        #: 否则"显示的是这帧、判定用的是另一帧"，而且没连接时判定拿不到东西。
        self._last: Snapshot | None = None
        self.bridge = WorkerBridge(self)
        self.bridge.state.connect(self._on_state)
        self.bridge.log.connect(self._log)
        self.bridge.grip_state.connect(self._on_grip_state)

        root = QtWidgets.QWidget()
        root.setObjectName("AppBg")
        rl = QtWidgets.QVBoxLayout(root)
        rl.setContentsMargins(0, 0, 0, 0)
        rl.setSpacing(0)

        # ⚠ 顺序要紧：**页面先造**，因为顶栏要插它的「连接」组（见下）。
        self.page = TeleopPage(settings)

        # ── 顶栏 ──
        # ⚠ 顶栏**没有标题**（用户裁决 2026-09-29）—— 见 `shell.TopBar`
        self.top = TopBar()
        self.btn_estop = QtWidgets.QPushButton("⛔ 急停")
        self.btn_estop.setStyleSheet(
            f"QPushButton {{ {theme.sans(12.5, 600)} color: {theme.C['danger']};"
            f" background: {theme.C['danger_soft']};"
            f" border: 1px solid {theme.C['danger_line']};"
            f" border-radius: {theme.RADIUS['md']}px; padding: 5px 12px; }}"
            f"QPushButton:hover {{ background: {theme.C['danger_line']}; }}"
            f"QPushButton:disabled {{ color: {theme.C['ink_ghost']};"
            f" background: {theme.C['line_soft']}; border-color: {theme.C['line']}; }}")
        self.btn_estop.setToolTip(
            "⚠ 急停 = 全失能 ⇒ 臂**自由落体**。\n"
            "要「稳住」请用右侧的大 STOP（停止遥操 → 受控接管 movej）—— 那是两件事。")
        self.btn_estop.clicked.connect(self._estop)
        self.top.layout().addWidget(self.btn_estop)

        # ⚠ 「连接」组插到标题的**水平右侧**（用户裁决 2026-09-29）——
        #   控件与信号都在 `TeleopPage` 那边（`connect_clicked` 等），这里只是
        #   **把它挂到顶栏上**：一个控制台只有一条命令入口，没必要为它另起一层。
        self.top.add_connect(self.page.connect_bar)
        rl.addWidget(self.top)

        rl.addWidget(self.page, 1)
        self.setCentralWidget(root)

        #: 关节表就在左栏里。单独挂一个名字出来，是因为
        #: 「`err != 1` 才高亮」有专门的回归测试钉着。
        self.joints_table = self.page.joints
        self.chk_ok = self.page.chk_ok
        #: 右栏那个「节点日志」。`self.log` 保持这个名字（回归测试都对着它写）。
        self.log = self.page.log

        # 夹爪 SDK 侧的只读默认值（本版没接线，但别显示 `—`）
        self._show_grip_defaults()

        # ── 接线 ──
        self.page.connect_clicked.connect(self._connect)
        self.page.disconnect_clicked.connect(self._disconnect)
        self.page.settings_changed.connect(self._save)
        self.page.teleop_toggled.connect(self._toggle_teleop)
        self.page.grip_toggled.connect(self._toggle_grip)
        self.page.grip_settings_changed.connect(self._save_grip)
        self.page.payload_applied.connect(self._apply_payload)
        self.page.arm_action.connect(self._arm_action)

        self._relock()
        self._log("就绪。⚠ 一个 CDC 口只允许一个进程。")

    def _show_grip_defaults(self) -> None:
        """把 `GripWorker` 的构造默认值显示到夹爪卡的只读格子里（**只是显示**）。"""
        from ..grip_worker import GRIP_RATE_HZ, GRIP_WATCHDOG_MS
        sig = inspect.signature(GripWorker.__init__)
        self.page.set_grip_defaults(
            GRIP_RATE_HZ, sig.parameters["kp"].default, sig.parameters["kd"].default)
        self.page.ro_watchdog.setToolTip(
            f"`arm_worker.WATCHDOG_MS`（常量）　夹爪侧的 watchdog 默认 "
            f"{GRIP_WATCHDOG_MS:g} ms —— 本版没接到界面上")

    # ────────────────────────── 安全闸门（§7.2）──────────────────────────
    def _relock(self, *_a) -> None:
        """勾选前运动类按钮一律锁定；断线 / 急停 / FAULT 自动重新锁定。

        ⚠ 判据取 **`self._last`（界面正在显示的那一帧）**，不另去问 worker ——
        否则就是两份真相，而且没连接时判定会拿不到东西（FAULT 就锁不住了）。
        """
        snap = self._last
        bad = bool(snap and (snap.faulted or snap.joint_fault or snap.error))
        armed = self.chk_ok.isChecked() and not bad
        connected = bool(snap and snap.connected)
        live = bool(snap and snap.teleop_active)
        self.page.btn_teleop.setEnabled(armed and connected)
        # ⚠ 大 STOP **不锁**：链路挂了也必须留一条"停"的退路
        #   （夹爪按钮那条死锁是同一个道理）。断开时点它是个无害的 no-op。
        self.btn_estop.setEnabled(self.worker is not None)

        # ── 臂维护四键，**门控分两档**（见 `pages._build_arm_card`）──
        # ⚠⚠ 运动类（使能/回零）要勾安全确认：它们会让臂动。
        # ⚠⚠ 状态类（清错/复位）**绝不要**那个勾选 —— 臂出故障时 `_relock` 会把
        #   确认框自动摘掉，而那一刻**正是**要按"清错→复位"的时候。
        #   绑上勾选就等于"最需要它的时候它不可用"。
        # 遥操跑着的时候四键一律禁用：它们会和伺服环抢臂。
        # ⚠ 「连接臂 / 断开」按 **worker 是否还在** 驱动（不是按快照的 connected）：
        #   断开之后快照要等我们自己复位，中间会有一瞬"快照说已连接、worker 已没了"。
        self.page.btn_connect.setEnabled(self.worker is None)
        self.page.btn_disconnect.setEnabled(self.worker is not None)
        self.page.btn_enable.setEnabled(armed and connected and not live)
        self.page.btn_home.setEnabled(armed and connected and not live)
        self.page.btn_clear.setEnabled(connected and not live)
        self.page.btn_reset.setEnabled(connected and not live)
        if bad:
            self.chk_ok.setChecked(False)

    # ────────────────────────── worker 生命周期 ──────────────────────────
    def _save(self) -> None:
        save_settings(self.s)

    def _connect(self) -> None:
        if self.worker is not None:
            self._log("已经连接（先点顶栏的「断开」）")
            return
        self._save()
        # ⚠⚠ 连接**只吃连接参数**（CDC 口）。角色 / arm_id / peer / 端口都是
        #     **遥操**参数，在点「启动遥操」时才读 —— 见 `TeleopParams`。
        #     与 litearm-server 同形（它的 transport 启动时就建好、**不按 mode 分叉**）。
        self.worker = ArmWorker(
            port=self.s.cdc_port or None,
            on_state=self.bridge.on_state, on_log=self.bridge.on_log)
        self.worker.start()
        self._log("正在连接臂（角色在点「启动遥操」时才定）…")

    def _disconnect(self) -> None:
        """断开：停遥操 → 受控接管 movej → 关 zenoh → 关臂（**不失能**）。

        ⚠ 收尾顺序与关窗**完全同路**（都走 `ArmWorker.shutdown` → `_teardown`）⇒
        断开后臂保持**使能并停在当前位置**，不是自由落体。
        ⚠⚠ **收尾没跑完就不许解除连接**：超时那次 join 只是"没等到"，线程仍占着
        **CDC 口与 zenoh 端口**，这时再连一个新的会让两者抢同一份资源。
        所以这里跟夹爪那条错误恢复同一个套路：报告 + 让用户再点一次。
        """
        w = self.worker
        if w is None:
            return
        self._log("正在断开：停遥操 → 受控接管 movej → 关 zenoh → 关臂（**不失能**）…")
        # ⚠ 用短超时：收尾是 `movej(实测位姿)`，到位即返回，正常是毫秒级；
        #   别让 Qt 主线程为了一个理论上限卡 8 s（与关窗那条同款取舍）。
        w.shutdown(timeout=3.0)
        if w.is_alive():
            self._log("⚠ 收尾尚未完成（线程还占着 CDC 口与 zenoh 端口）—— "
                      "暂不解除连接；请稍后再点一次「断开」")
            return
        self.worker = None
        # ⚠ 必须把界面那份快照也复位：`_teardown` 只改 worker 自己那份、不会推给界面，
        #   留着旧的"已连接"会让 `_relock` 继续按已连接判定、字段也不解锁。
        self._last = Snapshot()          # ⚠ 角色未定（还没启遥操）
        self.page.apply(self._last, self.s.peer, self.s.jport, self.s.arm_id)
        self.top.update_from(self._last)
        self.top.set_connection(False, "未连接")
        self.top.set_detail("—")
        self.page.log_badge.set_state("未连接", "outline")
        self._relock()
        self._log("✓ 已断开（臂保持使能与当前位置）")

    def _toggle_teleop(self, on: bool) -> None:
        if self.worker is None:
            return
        if not on:
            self._log("停止遥操（受控接管 movej）")
            self.worker.set_teleop(False)
            return
        # ⚠⚠ **角色/arm_id/peer/端口在这一刻读**，不是连接时 —— 见 `TeleopParams`。
        #    所以连上之后这四个控件仍然可以改，改了**下次启动生效**
        #    （界面按 `teleop_active` 锁，不是按 `connected`）。
        params = TeleopParams(role=self.page.role(), arm_id=self.s.arm_id,
                              peer=self.s.peer, jport=self.s.jport)
        self._log(f"启动遥操（角色={'主臂' if params.role == ROLE_MASTER else '从臂'}"
                  f" · {params.key} · 端口 {params.jport}）")
        self.worker.set_teleop(True, params)

    def _arm_action(self, what: str) -> None:
        """臂维护动作（使能/清错/复位/回零）—— 排到 worker 线程上执行。

        ⚠ 这里**不做 SDK 调用**，只投递（spec §3.1）；日志与出错处理在 worker 里。
        """
        if self.worker is None:
            self._log("⚠ 未连接，臂操作未执行")
            return
        table = {
            "enable": (self.worker.enable_arm, "使能"),
            "clear": (self.worker.clear_faults, "清错"),
            "reset": (self.worker.reset_arm, "复位"),
            "home": (self.worker.go_home, "回零"),
        }
        hit = table.get(what)
        if hit is None:                                  # 理论不可达（信号值写死）
            self._log(f"⚠ 未知的臂操作 {what!r}")
            return
        fn, name = hit
        if what == "home":
            # ⚠ 回零是**会动臂**的，且 worker 会在里面阻塞到到位/超时 ⇒ 必须让用户看见
            self._log("⚠ 回零：臂将移动到 URDF 零位（固件低安全速度 0.10）…")
        fn()

    def _apply_payload(self, mass, com) -> None:
        if self.worker is None:
            self._log("⚠ 未连接，载荷未设置")
            return
        self._log(f"设置末端载荷：{mass:.3f} kg，质心 {[round(v, 4) for v in com]} m …")
        self.worker.set_payload(mass, com)

    # ────────────────────────── 夹爪遥操（独立链路，§2）──────────────────────────
    def _save_grip(self) -> None:
        v = self.page.gripper_values()
        self.s.gcan = v["gcan"]
        self.s.gpeer = v["gpeer"]
        self.s.gport = v["gport"]
        self.s.grip_id = v["grip_id"]
        self.s.grip_torque_limit_nm = v["torque_limit_nm"]
        self._save()

    def _ensure_grip_worker(self) -> "GripWorker | None":
        """按需建 `GripWorker`。⚠ 与 `ArmWorker` **完全独立**（spec §2）。

        没有通道就**不建** —— 那是「不启用夹爪遥操」的安全默认，不是错误。
        """
        if self.grip is not None:
            if self.grip.is_alive():
                return self.grip
            # ⚠ 线程已经收尾完了 ⇒ 丢掉重建（否则会一直把一个死 worker 当活的用）
            self.grip = None
        v = self.page.gripper_values()
        if not v["gcan"]:
            self._log("⚠ 未填夹爪 CAN 通道 —— 夹爪遥操未启用")
            return None
        self._save_grip()
        # ⚠⚠ 钉死必须排在 `GripWorker.start()` **之前** —— 那条线程里的工厂会
        #    `import litegrip`，本机那份 editable 的 gitee 克隆会静默抢先（实测）。
        try:
            from ..grip_worker import pin_grip_sdk
            self._log(f"夹爪 SDK: {pin_grip_sdk()}")
        except BaseException as e:                   # noqa: BLE001
            # ⚠⚠ 必须兜住 **BaseException**，不能只兜 `SystemExit`：
            #    · `assert_sdk_pinned` 抛的是 `SystemExit`（**不是** `Exception` 的子类）；
            #    · 一台**没装 litegrip** 的机器上 `import litegrip` 会抛
            #      `ModuleNotFoundError`；
            #    · 而 PyQt5 对**逃出槽的异常会直接 abort 进程**（本机实测：
            #      `ModuleNotFoundError` 逃出槽 ⇒ exit 134、core dumped）。
            #    ⇒ 只兜 SystemExit 的结果是：点一下「启动夹爪遥操」把**整个应用**
            #      带走，连正在跑的臂遥操一起（没有收尾 movej）——
            #      直接违反 spec §2「与臂完全解耦」的故障隔离承诺。
            self._log(f"⛔ 夹爪 SDK 不可用：{e}")
            self._log("   （夹爪遥操已停用；臂遥操不受影响）")
            return None
        self.grip = GripWorker(
            gcan=v["gcan"], grip_id=v["grip_id"],
            gpeer=v["gpeer"], gport=v["gport"], align=v["align"],
            torque_limit_nm=v["torque_limit_nm"],
            on_state=self.bridge.grip_state.emit, on_log=self.bridge.on_log)
        self.grip.start()
        return self.grip

    def _toggle_grip(self, on: bool) -> None:
        if not on:
            if self.grip is not None:
                self._log("停止夹爪遥操（夹爪保持当前位置，**不失能**）")
                self.grip.set_teleop(False)
            return
        w = self._ensure_grip_worker()
        if w is None:
            self.page.btn_grip.setChecked(False)
            return
        # ⚠ 夹爪的角色**跟着臂走** ⇒ 也在这一刻读（不是连接时、也不是建 worker 时）
        role = self.page.role()
        self._log(f"启动夹爪遥操（角色={'主臂' if role == ROLE_MASTER else '从臂'}，跟随臂）")
        w.set_teleop(True, role)

    def _on_grip_state(self, g: GripSnapshot) -> None:
        # ⚠ 本槽在 **Qt 主线程**跑（信号跨线程排队），碰控件是安全的
        self.page.apply_grip(g)
        # ⚠⚠ 夹爪出错过 ⇒ 丢掉这个 worker，让下次点击能**重建**。
        #    不复位的话 `GripWorker` 已经死了、按钮又永远停用 ⇒ 只能重启应用。
        #    （`stop()` 是幂等的；此时那条线程早已在 `_teardown` 里收完尾。）
        if g.error and self.grip is not None:
            try:
                # ⚠ 这是个**会阻塞 Qt 主线程**的 join。用短超时把最坏冻结界在 2 s
                #    （CAN 卡死时 `_handoff`/`disconnect` 可能慢）；超时不会丢线程
                #    ——`stop()` 保留引用，下面用 `is_alive()` 判断能不能重建。
                self.grip.stop(timeout=2.0)
            except Exception:                        # noqa: BLE001
                pass
            self.page.btn_grip.setChecked(False)
            if self.grip.is_alive():
                # ⚠⚠ 线程还活着 ⇒ 它仍占着 **zenoh 端口**与 **CAN**。
                #    这时候再建一个 worker 会让两条线程抢同一份资源
                #    （旧的要等 `_teardown` 才放端口 ⇒ 新的报 Address already in use）。
                self._log("⚠ 夹爪 worker 尚未退出 —— 暂不重建；"
                          "请再点一次「启动夹爪遥操」重试收尾")
            else:
                self.grip = None
                self._log("夹爪遥操出错并已停止 —— 可再次点「启动夹爪遥操」重试")

    def _estop(self) -> None:
        """⛔ **不经 worker 队列**（§7.4）—— worker 可能正卡在收尾 `movej` 里。"""
        if self.worker is None:
            return
        self.chk_ok.setChecked(False)
        self.btn_estop.setEnabled(False)
        self.btn_estop.setText("急停已发出…")
        self._log("⛔ 急停已发出 —— 全失能 ⇒ 臂**自由落体**")
        self.worker.emergency_stop()
        QtCore.QTimer.singleShot(1500, self._estop_check)

    def _estop_check(self) -> None:
        out = self.worker.emergency_outcome() if self.worker else None
        if out is None:
            self._log("⚠ 急停尚无结果（最坏等 1.2 s 应答超时）")
            QtCore.QTimer.singleShot(1500, self._estop_check)
            return
        ok, err = out
        self._log("⛔ 急停已执行" if ok else f"⛔ 急停失败: {err}")
        self.btn_estop.setText("⛔ 急停")
        self.btn_estop.setEnabled(True)

    # ────────────────────────── 刷新 ──────────────────────────
    def _on_state(self, snap: Snapshot) -> None:
        # ⚠ 本槽在 **Qt 主线程**跑（信号跨线程排队），碰控件是安全的
        self._last = snap
        self.page.apply(snap, self.s.peer, self.s.jport, self.s.arm_id)
        self.top.update_from(snap)
        # ⚠ 连接状态全部归顶栏：**胶囊**（状态）+ **详情串**（固件/端点，或错误文本）。
        #   出错时胶囊走 `bad`、详情串转红 —— 错误不许只躺在日志里。
        if snap.error:
            self.top.set_connection(False, "出错", kind="bad")
            self.top.set_detail(f"⛔ {snap.error}", role="danger")
        else:
            master = snap.role == ROLE_MASTER
            self.top.set_connection(snap.connected,
                                    "已连接" if snap.connected else "未连接")
            # ⚠ 角色**未定**时（连上但还没启遥操）不写"监听/连接" —— 那是猜的。
            # ⚠⚠ 而且**端点只在 side 非空时才拼**：从前那句 `if snap.connected` 挂错了
            #     对象 ⇒ 角色未定时会吐出 `"… ·  127.0.0.1:17447"`（**双空格**），
            #     未连接时会吐出 `"… · "`（**尾部悬挂一个分隔符**）。
            side = "监听" if master else ("连接" if snap.role == ROLE_SLAVE else "")
            parts = [snap.firmware or ""]
            if side and snap.connected:
                parts.append(f"{side} {self.s.peer}:{self.s.jport}")
            self.top.set_detail(" · ".join(p for p in parts if p) or "—")
        self.page.log_badge.set_state(
            "已连接" if snap.connected else "未连接",
            "ok" if snap.connected else "outline")
        self._relock()

    def _log(self, text: str) -> None:
        self.log.appendPlainText(f"[{time.strftime('%H:%M:%S')}] {text}")

    def closeEvent(self, ev) -> None:
        if self.worker is not None:
            self._log("正在退出：停遥操 → 关 zenoh → 关臂 …")
            self.worker.shutdown()
        if self.grip is not None:
            self._log("正在退出夹爪遥操：交接持位 → 关 zenoh → 断 CAN（**不失能**）…")
            # 同上：别用 5 s 默认值，退出路径上冻结窗口没必要
            self.grip.stop(timeout=2.0)
        self._save()
        super().closeEvent(ev)


def run(argv=None, settings=None) -> int:
    argv = list(sys.argv if argv is None else argv)
    app = QtWidgets.QApplication(argv)
    w = MainWindow(settings or load_settings())
    w.show()
    return app.exec_()
