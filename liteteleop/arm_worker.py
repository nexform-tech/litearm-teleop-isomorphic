"""`ArmWorker` —— **唯一持有 `Arm` 的线程**（spec §3.1）。

GUI 只做两件事：**投命令**、**读快照**。任何 SDK 调用都发生在本线程里。

## 为什么要把 SDK 调用收进一条线程

`litearm-python` 的 `_Ack` 按 `(上行 id, 回显码)` 分队列 —— 两条读线程**大体**能各取各的，
但"取帧"这个动作本身会在链路上互相抢（SDK 自己的注释就记着"放在锁外实测会把在跑那条的
`ACK{0x3A}` 抢走 ⇒ 受害者报无应答而命令其实已生效"）。串行化让这一整类问题不存在。

**唯二例外**（SDK 明文允许，见 §7.4）：

- **急停**走专用短线程直调 `emergency_stop()` —— worker 可能正卡在收尾 `movej` 里
  （上界 `move_timeout`）；排在它后面最长要等 3 s 才发得出去。
- `get_state(refresh=False)` 只读缓存、不取帧，**不排队**。

## 状态帧怎么到 GUI

`movej` / `follow` 期间 worker 是忙的，若界面只能经 worker 取状态那几秒就死了 ——
而「对齐中」正是最需要看状态的时刻。做法照抄 litetool：把 `arm._a._on_status` 包一层，
**在 SDK 自己的读线程上**把每帧解码结果推给 GUI。挂不上时**抛 `StateHookMissing`**，
宁可启动即报，也不要"连上了但界面不动"。
"""
from __future__ import annotations

import logging
import queue
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, List, Optional

from . import link, servo, wire
from .safety import clamp_to_limits, read_safe_limits
from .wire import N_JOINTS

log = logging.getLogger("liteteleop.worker")

__all__ = ["StateHookMissing", "Snapshot", "ArmWorker", "ROLE_MASTER", "ROLE_SLAVE"]

ROLE_MASTER = "master"
ROLE_SLAVE = "slave"

#: 主臂发布频率 = litearm-server 的 `pub_hz` 默认值（`__main__.py:121` 的 `--teleop-pub-hz`）。
PUB_HZ = 200.0

#: 从臂伺服环 = litearm-server 的 `control_loop_hz`（`litearm_balanced.yaml:32`）。
SLAVE_HZ = servo.DEFAULT_HZ

#: 「同步/对齐」允许的最长时间。**用户裁决 2026-09-28：3 s → 30 s。**
#: `movej` **一旦到位就立刻返回**（`_arrive` 是"到位/超时/故障"三者先到为准）
#: ⇒ 这只是上限，不是固定等待。
#: ⚠ 它是 `Arm(move_timeout=)`，而 SDK 明说**连接后不许改**（改会让"等待窗口"与
#: `_CartPending` 的额度有效期分叉，且两个方向都不安全）⇒ **只能在构造时定**。
#: ⚠ 代价：链路真断时，收尾那句 `movej` 也会阻塞到 30 s（不是 3 s）。
#:    界面不会冻（状态帧走钩子）、急停走旁路 ⇒ 可接受。
ALIGN_MOVE_TIMEOUT = 30.0

#: 从臂 watchdog（照搬 `TeleopManager.watchdog_ms` 默认值）。**收到首帧之后才生效。**
WATCHDOG_MS = 200.0


class StateHookMissing(RuntimeError):
    """`Arm` 上找不到状态钩子缝隙（SDK 内部结构变了）。

    照抄 litetool：**抛错而不是静默降级** —— 后者会变成"连上了但界面不动"这种要查半天的现场。
    """


def attach_state_hook(arm, on_state: Callable[[object], None]) -> None:
    """把 `_Ack._on_status` 包一层，让每帧解码结果也流向 GUI。

    这是本仓对 SDK 的**唯一私有耦合**。`on_state` 在 **SDK 自己的读线程**上被调用
    ⇒ **只许写一个槽**（不许做阻塞操作、不许碰 GUI 控件；经 Qt 信号转投即可）。
    """
    ack = getattr(arm, "_a", None)
    if ack is None or not callable(getattr(ack, "_on_status", None)):
        raise StateHookMissing(
            "litearm.Arm 上找不到 _a._on_status 状态钩子缝隙 —— SDK 内部结构变了。"
            "本工具依赖它把运动期间的 100Hz 状态推给界面；"
            "请更新 liteteleop/arm_worker.py::attach_state_hook 以适配新版 SDK。")
    orig = ack._on_status

    def hooked(payload: bytes) -> None:
        orig(payload)
        st = ack.state
        if st is not None:
            on_state(st)

    ack._on_status = hooked


@dataclass
class Snapshot:
    """**只读**快照 —— GUI 只看这个，不碰 `Arm`。"""

    connected: bool = False
    role: str = ""
    firmware: str = ""
    # 关节
    q: List[float] = field(default_factory=list)
    dq: List[float] = field(default_factory=list)
    tau: List[float] = field(default_factory=list)
    t_mos: List[float] = field(default_factory=list)
    t_coil: List[float] = field(default_factory=list)
    err: List[int] = field(default_factory=list)
    # 整体
    mode_name: str = ""
    enabled: bool = False
    faulted: bool = False
    joint_fault: int = 0
    flags: int = 0
    flag_names: List[str] = field(default_factory=list)
    seq: int = 0
    state_hz: float = 0.0
    # 遥操
    teleop_active: bool = False
    frames_sent: int = 0
    frames_received: int = 0
    matching: bool = False          # 主臂：有无订阅者（**是布尔不是计数**）
    frame_age: Optional[float] = None   # 从臂：最新一帧有多老（s）
    watchdog_trips: int = 0
    rejects: int = 0
    # 末端载荷（夹爪）—— 单位为 kg / m（m 在 ee_link 系）
    payload_mass: float = 0.0
    payload_com: List[float] = field(default_factory=list)
    error: str = ""


class ArmWorker:
    """独占 `Arm` 的线程。生命周期：`start()` → … → `shutdown()`。"""

    def __init__(self, role: str, port: Optional[str] = None,
                 arm_id: str = link.DEFAULT_ARM_ID,
                 peer: Optional[str] = None, jport: int = 0,
                 on_state: Optional[Callable[[Snapshot], None]] = None,
                 on_log: Optional[Callable[[str], None]] = None,
                 move_timeout: float = ALIGN_MOVE_TIMEOUT):
        if role not in (ROLE_MASTER, ROLE_SLAVE):
            raise ValueError(f"role 必须是 {ROLE_MASTER}/{ROLE_SLAVE}，收到 {role!r}")
        self.role = role
        self.port = port
        self.arm_id = arm_id
        self.key = wire.teleop_topic(arm_id)
        self.peer = peer
        self.jport = int(jport or 0)
        self.move_timeout = float(move_timeout)
        self._on_state = on_state
        self._on_log = on_log

        self._arm = None
        self._pub = None            # master: link.Listener
        self._sub = None            # slave: link.Connector
        self._slot = link.LatestSlot()
        self._limits = None
        self._gains = None

        self._lock = threading.Lock()
        self._snap = Snapshot(role=role)
        self._cmds: "queue.Queue[Callable[[], None]]" = queue.Queue()
        self._stop = threading.Event()
        self._teleop_want = False
        self._thread: Optional[threading.Thread] = None
        self._frames_sent = 0
        self._frames_received = 0
        self._watchdog_trips = 0
        self._estop_outcome: Optional[tuple] = None
        self._payload_t = 0.0

    # ────────────────────────── 生命周期 ──────────────────────────
    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name="ArmWorker", daemon=True)
        self._thread.start()

    def shutdown(self, timeout: float = 8.0) -> None:
        """停遥操 → 关 zenoh → 关臂。**幂等**。"""
        self._stop.set()
        t = self._thread
        if t is not None:
            t.join(timeout=timeout)
            if t.is_alive():
                self._log("⚠ ArmWorker 未在超时内退出")

    # ────────────────────────── GUI 用的投递口 ──────────────────────────
    def post(self, fn: Callable[[], None]) -> None:
        """把一件事排到 worker 线程上执行。"""
        self._cmds.put(fn)

    def set_teleop(self, on: bool) -> None:
        """请求启动/停止遥操（异步）。"""
        def _do() -> None:
            self._teleop_want = bool(on)
        self.post(_do)

    def set_payload(self, mass: float, com) -> None:
        """设末端载荷（夹爪）—— 排到 worker 线程上执行。

        ⚠ 固件会**静默钳制**（mass→[0,20]、com→[-1,1]）且照样回 ACK ⇒
        本方法**回读**并把真值记进快照，界面显示的是**读回值**而不是输入值。
        """
        def _do() -> None:
            if self._arm is None:
                self._log("⚠ 未连接，载荷未设置")
                return
            try:
                m, c = servo.apply_payload(self._arm, mass, com)
            except Exception as e:                       # noqa: BLE001
                self._log(f"⛔ 设载荷失败: {e}")
                return
            with self._lock:
                self._snap.payload_mass = m
                self._snap.payload_com = list(c)
            clamped = (abs(m - float(mass)) > 1e-9
                       or any(abs(a - float(b)) > 1e-9 for a, b in zip(c, com)))
            self._log(f"载荷已设：{m:.3f} kg，质心 {[round(v, 4) for v in c]} m"
                      + ("   ⚠ **被固件钳过**（给的值超出 [0,20]/[-1,1]）" if clamped else ""))
        self.post(_do)

    def refresh_payload(self) -> None:
        """读回载荷到快照。**只许在非跟随路径上调** —— 它是串口往返，会拖累伺服环。"""
        if self._arm is None:
            return
        try:
            m, c = servo.read_payload(self._arm)
        except Exception:                                # noqa: BLE001
            return
        with self._lock:
            self._snap.payload_mass = m
            self._snap.payload_com = list(c)

    def snapshot(self) -> Snapshot:
        with self._lock:
            s = self._snap
            s.teleop_active = self._teleop_want
            s.frames_sent = self._frames_sent
            s.frames_received = self._frames_received
            s.watchdog_trips = self._watchdog_trips
            if self._pub is not None:
                s.matching = self._pub.matching
            if self.role == ROLE_SLAVE:
                s.frame_age = self._slot.peek_age(time.monotonic())
            return s

    # ────────────────────────── 急停旁路（§7.4）──────────────────────────
    def emergency_stop(self) -> None:
        """⛔ **不经 worker 队列** —— worker 可能正卡在 `movej` 里。

        依据是 SDK 自己写的（`arm.py` 的锁序注释）：
        「`emergency_stop`/`disable`/`zero_g*`/`get_tcp` 自己不许获取本锁 ——
        降能量方向的动作与只读查询必须永远可达（持锁者可能阻塞到 `move_timeout`）」。

        ⚠ **急停 = 全失能 ⇒ 臂自由落体**，与"停止遥操"是**两件事**。
        """
        arm = self._arm
        if arm is None:
            self._estop_outcome = (False, "未连接")
            return
        # 立刻置位，让遥操循环下一拍就退（不等这一次调用返回）
        self._teleop_want = False
        self._estop_outcome = None

        def _worker() -> None:
            try:
                arm.emergency_stop()
                self._estop_outcome = (True, "")
            except Exception as e:                       # noqa: BLE001
                self._estop_outcome = (False, str(e))

        threading.Thread(target=_worker, name="EmergencyStop", daemon=True).start()

    def emergency_outcome(self):
        return self._estop_outcome

    # ────────────────────────── worker 线程 ──────────────────────────
    def _log(self, msg: str) -> None:
        log.info(msg)
        if self._on_log is not None:
            try:
                self._on_log(msg)
            except Exception:                            # noqa: BLE001
                log.exception("on_log 回调炸了")

    def _push(self, st) -> None:
        """在 **SDK 读线程**上被调用 ⇒ 只写槽，不做别的。"""
        with self._lock:
            s = self._snap
            s.q = list(st.q)
            s.dq = list(st.dq)
            s.tau = list(st.tau)
            s.t_mos = [j.t_mos for j in st.joints]
            s.t_coil = [j.t_coil for j in st.joints]
            s.err = [j.err for j in st.joints]
            s.mode_name = st.mode_name
            s.enabled = st.enabled
            s.faulted = st.faulted
            s.joint_fault = st.joint_fault
            s.flags = st.flags
            s.flag_names = list(st.flag_names)
            s.seq = st.seq
        if self._on_state is not None:
            try:
                self._on_state(self.snapshot())
            except Exception:                            # noqa: BLE001
                log.exception("on_state 回调炸了")

    def _drain(self, timeout: float) -> None:
        try:
            fn = self._cmds.get(timeout=timeout)
        except queue.Empty:
            return
        try:
            fn()
        except Exception as e:                           # noqa: BLE001
            self._log(f"命令执行失败: {e}")

    def _run(self) -> None:
        import litearm as pa
        from .ports import resolve_port
        try:
            # ⚠ **不能用 SDK 的 find_cdc_port() 自动选**：同型号两条臂 VID:PID 相同，
            #    它取第一个匹配 ⇒ 主/从两个进程会抢到同一个口而且不报错（见 ports.py）。
            port = resolve_port(self.port)
            self._log(f"使用 CDC 口 {port}")
            self._arm = pa.Arm(port=port, move_timeout=self.move_timeout)
            self._arm.connect()
            attach_state_hook(self._arm, self._push)
            with self._lock:
                self._snap.connected = True
                self._snap.firmware = self._arm.firmware
            self._log(f"已连接 {self._arm.firmware!r} n={self._arm.n}")
            if self._arm.n != N_JOINTS:
                raise RuntimeError(f"只支持 {N_JOINTS} 关节整臂，当前 n={self._arm.n}")
            self._arm.enable()
            st = self._arm.get_state(refresh=True).value
            if st is None or not st.enabled or st.faulted or st.joint_fault:
                raise RuntimeError(f"起始状态不干净: {st and st.fault_detail}")
            self._log("已使能")
            self.refresh_payload()                       # 读回当前载荷（含出厂默认）

            while not self._stop.is_set():
                if self._teleop_want:
                    self._run_teleop()
                else:
                    self._drain(0.1)
                    # ⚠ 只在空闲时刷：读载荷是串口往返，**绝不能**塞进伺服环
                    now = time.monotonic()
                    if now - self._payload_t > 2.0:
                        self._payload_t = now
                        self.refresh_payload()
        except Exception as e:                           # noqa: BLE001
            with self._lock:
                self._snap.error = str(e)
            self._log(f"⛔ {e}")
        finally:
            self._teardown()

    def _teardown(self) -> None:
        """收尾顺序（§7.3）：停遥操 → 关 zenoh → 关臂。"""
        try:
            if self._arm is not None and self._teleop_want:
                self._teleop_want = False
        finally:
            self._teleop_want = False
        try:
            self._restore_gains()                        # ⚠ 必须排在下面那句 movej 之前
        except Exception:                                # noqa: BLE001
            log.exception("还原增益失败")
        if self._arm is not None:
            try:
                servo.hold_at_current(self._arm)         # ⛔ 绝不 disable
            except Exception as e:                       # noqa: BLE001
                self._log(f"⚠ 收尾 movej 失败: {e}")
        for ep in (self._pub, self._sub):
            if ep is not None:
                try:
                    ep.close()                           # ⛔ 不 close ⇒ 进程永久挂死
                except Exception:                        # noqa: BLE001
                    log.exception("close zenoh 端点失败")
        self._pub = self._sub = None
        if self._arm is not None:
            try:
                self._arm.close()                        # ⛔ 同上
            except Exception:                            # noqa: BLE001
                log.exception("close arm 失败")
        with self._lock:
            self._snap.connected = False

    # ────────────────────────── 遥操 ──────────────────────────
    def _run_teleop(self) -> None:
        try:
            if self.role == ROLE_MASTER:
                self._run_master()
            else:
                self._run_slave()
        except Exception as e:                           # noqa: BLE001
            self._log(f"⛔ 遥操异常退出: {e}")
            with self._lock:
                self._snap.error = str(e)
        finally:
            self._teleop_want = False
            # ⚠⚠ **先还原增益再 movej**：跟随期间 `mit_kp` 是 25，`movej` 的到位环撑不住。
            self._restore_gains()
            try:
                servo.hold_at_current(self._arm)         # 受控接管
            except Exception as e:                       # noqa: BLE001
                self._log(f"⚠ 收尾 movej 失败: {e}")


    def _restore_gains(self) -> None:
        """还原出厂增益。**幂等**；**任何 `movej` 之前都必须先调它**。"""
        if self._gains is not None and self._arm is not None:
            servo.restore_joint_gains(self._arm, self._gains)
            self._gains = None
            self._log("已还原出厂增益（movej 要用它）")

    def _run_master(self) -> None:
        """主臂：零重力拖动 → 定频采样 → 发布（spec §6）。**主臂不做任何钳位。**"""
        arm = self._arm
        self._pub = link.Listener(self.jport, self.key)
        self._log(f"主臂监听 {self.key} @ 端口 {self.jport}")
        arm.zero_g_start()
        time.sleep(0.5)                                  # 等过 engage 段
        dt = 1.0 / PUB_HZ
        nxt = time.monotonic() + dt
        while self._teleop_want and not self._stop.is_set():
            st = arm.get_state(refresh=False).value
            if st is not None:
                self._pub.put(wire.encode_teleop(st.q, st.dq, time.monotonic()))
                self._frames_sent += 1
            self._drain(0.0)                             # 顺手处理 GUI 投来的命令
            r = nxt - time.monotonic()
            if r > 0:
                time.sleep(r)
            nxt += dt
            if nxt < time.monotonic():
                nxt = time.monotonic() + dt
        try:
            arm.zero_g_stop()
        except Exception as e:                           # noqa: BLE001
            self._log(f"⚠ zero_g_stop 失败: {e}")

    def _run_slave(self) -> None:
        """从臂：订阅 → 钳位 → `slew_target` → `move_js`（spec §5）。"""
        arm = self._arm
        self._limits = read_safe_limits(arm)             # 读不到会抛 ⇒ 拒启动
        self._log(f"软限位 {list(zip(self._limits.lo, self._limits.hi))}")
        self._sub = link.Connector(self._peer_host(), self.jport, self.key,
                                   on_frame=self._on_wire)
        # ── 对齐（照搬 `_do_align`）：等首帧 → 钳位 → **低速 movej** ──
        # ⚠ 少了这一步，从臂会由 `slew_target` 直接拉过去，速度上限是 `speed_limit`
        #    （J1 到 2.8 rad/s），比 `align_speed=0.15` 快近 20 倍 —— 那是**大幅甩动**。
        #
        self._log("等待主臂首帧并对齐 …")
        aligned = servo.align_to_master(arm, self._slot.take, self._limits)

        # ⚠ **不改任何固件参数**（用户裁决）：K/B 用固件出厂的 `mit_kp`/`mit_kd`。
        #    写 `mit_kp` 会连带改坏 `movej`（它用的就是 `mit_kp`），真机踩过两次。
        #
        # ⚠⚠ 但**速度上限必须按 kd 预算收紧**：`move_js` 的 `dq` 进电机速度前馈
        #      （`τ += kd_eff·dq`，kd_eff 出厂 J1~J4 = 11），照抄 server 那份配 B=0.5 的
        #      `speed_limit` 会让 J4 的 `kd·dq` = 55 Nm 顶满 tau_max=21 ⇒ **抖**。
        if aligned is None:
            self._log("⚠ 未对齐（没收到帧 或 movej 失败）—— 跟随会逐步修正")

        # ⚠⚠ 对齐**之后**才写增益：写进去 `mit_kp` 变 25，`movej` 会撑不住（见 servo.SETUP_K）。
        self._gains = servo.apply_joint_gains(arm)
        self._log(f"已写跟随增益 mit_kp={servo.SETUP_K} mit_kd={servo.SETUP_B}、清零 kd_extra"
                  f"（⛔ 此后任何 movej 之前都必须先还原）")

        # 速度上限：跟随期间 kd_eff 就是 B（很小）⇒ 预算**不会**成为瓶颈，
        # 但仍算出来打日志 —— 哪天有人改了 SETUP_B，这条会立刻显形。
        kd_pre, tau_max = servo.effective_kd(arm)
        kd_post = [servo.SETUP_B] * N_JOINTS
        sl_budget = servo.speed_limit_from_kd(kd_post, tau_max)
        sl = [min(a, b) for a, b in zip(servo.DEFAULT_SPEED_LIMIT, sl_budget)]
        self._log(f"出厂 kd_eff={[round(x, 1) for x in kd_pre]} ⇒ 写后 kd_eff="
                  f"{[round(x, 1) for x in kd_post]}")
        self._log(f"speed_limit 预算={[round(x, 2) for x in sl_budget]}"
                  f" ⇒ 实取(=配置值)={[round(x, 2) for x in sl]}")

        def provider():
            payload, _ts = self._slot.take()
            if payload is None:
                return None
            try:
                frame = wire.decode_teleop(payload)
            except Exception:                            # noqa: BLE001
                return None
            clamped, _sat = clamp_to_limits(frame["q"], self._limits)
            return clamped

        def should_stop() -> bool:
            if not self._teleop_want or self._stop.is_set():
                return True
            # watchdog：收到首帧之后才生效（照搬 teleop_manager）
            age = self._slot.peek_age(time.monotonic())
            if age is not None and age * 1000.0 > WATCHDOG_MS:
                self._watchdog_trips += 1
                self._log(f"⚠ watchdog 超时 {WATCHDOG_MS:.0f}ms，停止跟随（不自动重连）")
                return True
            self._drain(0.0)
            return False

        servo.follow(arm, provider, should_stop=should_stop, hz=SLAVE_HZ,
                     speed_limit=sl)

    def _peer_host(self) -> str:
        peer = (self.peer or "").strip()
        return peer or "127.0.0.1"

    def _on_wire(self, payload: bytes) -> None:
        """**在 zenoh 线程上**被调用 ⇒ 只写槽。"""
        self._slot.put(payload, time.monotonic())
        self._frames_received += 1
