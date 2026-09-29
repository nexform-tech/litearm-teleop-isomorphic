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
from .wall import JointLimitWall
from .safety import clamp_to_limits, read_safe_limits
from .wire import N_JOINTS

log = logging.getLogger("liteteleop.worker")

__all__ = ["StateHookMissing", "Snapshot", "TeleopParams", "ArmWorker",
           "ROLE_MASTER", "ROLE_SLAVE"]

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


@dataclass(frozen=True)
class TeleopParams:
    """**遥操参数** —— 在点「启动遥操」的那一刻读取，**不是**连接时。

    ⚠⚠ 为什么不是连接时：一个 CDC 口连上之后，"这台机是主臂还是从臂"、
    "话题用哪个 arm_id"、"监听/连哪个端口"、"主臂在哪台机" 全都还没定 ——
    它们是**遥操**的属性，不是**连接**的属性。连接只做「打开串口 + 使能 + 持位」。

    与 `litearm-server` 同形：它的 transport 在启动时建好、**不按 mode 分叉**
    （`__main__.py:175` 的注释「始终 router/listen，不再按 teleop_mode 分叉建
    peer session」），角色是 `TeleopController.enter(mode, ...)` 的**参数**。

    ⚠ 四个字段里没有一个属于"连接" ⇒ 界面**不该**在连上后把它们锁死
    （那是旧设计「构造时定死」逼出来的补丁）。改成按 `teleop_active` 锁。
    """

    #: ⚠⚠ **必填、没有默认值** —— "不给就默认主臂"正是那种会造成事故的静默默认值
    #: （把不该软的臂送进零重力）。调用方必须显式说明这台机是主还是从。
    role: str
    arm_id: str = link.DEFAULT_ARM_ID
    peer: str = ""
    jport: int = 0

    def __post_init__(self) -> None:
        if self.role not in (ROLE_MASTER, ROLE_SLAVE):
            raise ValueError(f"role 必须是 {ROLE_MASTER}/{ROLE_SLAVE}，收到 {self.role!r}")

    @property
    def key(self) -> str:
        """遥操 zenoh topic —— 由 `arm_id` 派生。"""
        return wire.teleop_topic(self.arm_id)

    @property
    def topic(self) -> str:                                  # noqa: D401 - 见上
        return self.key

    def peer_host(self) -> str:
        return (self.peer or "").strip() or "127.0.0.1"


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
    #: 电机**状态帧**速率（Hz）—— 顶栏「控制频率」。⚠ 1 s 窗口的实测值，无数据为 0
    state_hz: float = 0.0
    #: **遥操链路**帧率（Hz）—— 主臂=发布、从臂=接收。主从链路卡那格。
    #: ⚠ 与 `state_hz` **不是一回事**：那个量的是 CDC 上的电机状态帧，
    #: 这个量的是 zenoh 上我们自己的遥操帧。停遥操后归 0（界面显示 `—`）。
    link_hz: float = 0.0
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

    def __init__(self, port: Optional[str] = None,
                 on_state: Optional[Callable[[Snapshot], None]] = None,
                 on_log: Optional[Callable[[str], None]] = None,
                 move_timeout: float = ALIGN_MOVE_TIMEOUT):
        """⚠ 构造只收**连接**参数（CDC 口）。

        角色 / arm_id / peer / jport 都是**遥操**参数，走
        `set_teleop(True, TeleopParams(...))` —— 见 `TeleopParams` 的说明。
        """
        self.port = port
        self.move_timeout = float(move_timeout)
        self._on_state = on_state
        self._on_log = on_log

        self._arm = None
        self._pub = None            # master: link.Listener（**常驻**，见 _ensure_pub）
        self._pub_meta = None       # 那个常驻端点是用哪组遥操参数建的
        self._sub = None            # slave: link.Connector（**每会话**建/关）
        self._slot = link.LatestSlot()
        self._limits = None
        self._params: Optional[TeleopParams] = None

        self._lock = threading.Lock()
        self._snap = Snapshot()
        self._cmds: "queue.Queue[Callable[[], None]]" = queue.Queue()
        self._stop = threading.Event()
        self._teleop_want = False
        self._thread: Optional[threading.Thread] = None
        self._frames_sent = 0
        self._frames_received = 0
        self._watchdog_trips = 0
        self._estop_outcome: Optional[tuple] = None
        self._payload_t = 0.0
        # 速率统计（1 s 窗口）：状态帧 / 遥操链路各一组。见 _state_tick / _link_tick
        self._state_n = 0
        self._state_t0 = 0.0
        self._state_hz = 0.0
        self._link_n = 0
        self._link_t0 = 0.0
        self._link_hz = 0.0

    # ────────────────────────── 生命周期 ──────────────────────────
    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name="ArmWorker", daemon=True)
        self._thread.start()

    def is_alive(self) -> bool:
        """worker 线程还在跑吗。

        ⚠ 收尾路径必须问它 —— `shutdown()` 只是 join 到超时，**超时不代表线程没了**：
        它还占着 **CDC 口**与 **zenoh 端口**，这时再建一个 worker 会抢同一份资源
        （新连接报 Address already in use / 串口打不开）。与 `GripWorker.is_alive`
        同款判据。
        """
        t = self._thread
        return bool(t is not None and t.is_alive())

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

    def set_teleop(self, on: bool, params: Optional[TeleopParams] = None) -> None:
        """请求启动/停止遥操（异步）。

        ⚠⚠ **角色（以及 arm_id / peer / 端口）在这里定**，不是构造时 ——
        见 `TeleopParams`。启动时必须给 `params`（**同步**抛，别让错误留到线程里）；
        停止时可以不给，沿用上一次那组。
        """
        if on and params is None:
            raise ValueError(
                "启动遥操必须给 params —— 角色/arm_id/peer/端口都是**遥操**参数，"
                "连接时还不知道（见 TeleopParams）")

        def _do() -> None:
            if on and params is not None:
                self._params = params
            self._teleop_want = bool(on)

        # ⚠ `_do` 在 **worker 线程**上执行（`_drain`），而 `_run_teleop` 也在那条
        #    线程上读 `_params` ⇒ 没有竞态。且这里**先赋值 params 再置 want**，
        #    顺序上也不会出现"want 已真、params 还是旧的"。
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
            # ⚠ 容差按 **float32 往返**定：读回值是从固件的 f32 解出来的，而
            #   `mass`/`com` 是 Python double —— 0.6 在 f32 里是 0.6000000238…，
            #   差约 2.4e-8。原先用 1e-9 会把它**误判成"被固件钳过"**（实测踩到：
            #   0.6 kg / 质心 0.03 m 本来就在 [0,20]/[-1,1] 内，却报了钳位告警）。
            clamped = (abs(m - float(mass)) > 1e-5
                       or any(abs(a - float(b)) > 1e-5 for a, b in zip(c, com)))
            self._log(f"载荷已设：{m:.3f} kg，质心 {[round(v, 4) for v in c]} m"
                      + ("   ⚠ **被固件钳过**（给的值超出 [0,20]/[-1,1]）" if clamped else ""))
        self.post(_do)

    # ────────────────────── 臂维护动作（使能 / 清错 / 复位 / 回零）──────────────────────
    #
    # ⚠⚠ 这些**都是排到 worker 线程上执行的**（与 `set_payload` 同款）——
    #   GUI 只投命令，任何 SDK 调用都在 worker 线程里（spec §3.1）。
    # ⚠ `go_home` 是**阻塞**的：SDK 的 `Arm.home()` 内部 `_arrive()` 会一直等到
    #   到位/超时（超时 = 构造时的 `move_timeout`，本仓默认 30 s）。
    #   期间 worker 线程被占着 ⇒ 起不了遥操、处理不了别的投递命令；
    #   ⛔ 但**急停仍然可达**（`emergency_stop` 绕过队列，见 §7.4 与它那条回归测试）。

    def _arm_action(self, name: str, fn) -> None:
        """把一次臂操作排到 worker 线程上执行，并统一记日志/错误。"""
        def _do() -> None:
            if self._arm is None:
                self._log(f"⚠ 未连接，{name}未执行")
                return
            try:
                out = fn(self._arm)
            except Exception as e:                       # noqa: BLE001
                self._log(f"⛔ {name}失败: {e}")
                with self._lock:
                    self._snap.error = f"{name}失败: {e}"
                return
            with self._lock:
                # 一次成功的手动操作 ⇒ 清掉上一次的错误（否则界面会一直挂着旧错）
                if self._snap.error:
                    self._snap.error = ""
            if out is not None:
                self._log(f"✓ {name}完成：{out}")
            else:
                self._log(f"✓ {name}已执行")
        self.post(_do)

    def enable_arm(self) -> None:
        """使能全关节。

        ⚠ **是一条运动类动作**：臂若正被自重压着（例如刚失能过），使能瞬间会
        "弹"到保持位 —— 所以界面把它与「回零」一起按运动类门控（要勾安全确认）。
        重试策略在 SDK 里（只对固件明说可重试的码重试）。
        """
        self._arm_action("使能", lambda a: a.enable())

    def clear_faults(self) -> None:
        """清故障码（逐轴；**健康轴零触碰**）。

        ⚠⚠ **它不是 `reset()` 的替代**：`clear_faults` 能把 `joint_fault` 清成 0，
        但臂可能还在 **EMERGENCY 锁存**态 —— 那时 `enable` 会恒拒
        `ERR[10,6]「锁存, 须先 RESET」`。
        真机验证过的恢复顺序：**`clear_faults()` → `reset_arm()` → `enable_arm()`**。
        """
        self._arm_action("清错", lambda a: a.clear_faults())

    def reset_arm(self) -> None:
        """清 EMERGENCY 锁存（固件 `CMD_RESET 0x00`）。见 `clear_faults` 里的恢复顺序。"""
        self._arm_action("复位", lambda a: a.reset())

    def go_home(self) -> None:
        """**回零**：各轴回 URDF 零位舒展姿（固件 `CMD_HOME 0x2A`）。

        ⚠ 固件侧速度写死 **0.10**（低安全速度），本方法**不接受 speed** ——
        与 `movej([0]*n)` 的差别是固件**允许它从越软限/贴端发起**（软限位 clamp 作用在
        **目标**上，零位在限内，起点不影响），所以它才是失能漂出限位之后的回家动作。
        ⚠ **须先使能**：未使能时先喊一声，别让用户对着"点了没反应"发呆。
        """
        def _fn(a):
            st = a.get_state(refresh=True).value
            if st is not None and not st.enabled:
                raise RuntimeError("臂未使能 —— 先点「使能」（固件会拒未使能的 home）")
            st2 = a.home()
            q = getattr(st2, "q", None)
            return None if q is None else [round(v, 3) for v in q]
        self._arm_action("回零", _fn)

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
            # ⚠ 角色是**遥操参数** ⇒ 没启过遥操时快照里没有角色
            #   （界面据此显示"未定"，而不是替用户猜一个）
            s.role = self._params.role if self._params is not None else ""
            s.state_hz = self._state_hz
            s.link_hz = self._link_hz
            if self._params is not None and self._params.role == ROLE_SLAVE:
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

    def _state_tick(self) -> None:
        """统计**电机状态帧**速率（1 s 窗口）。只被 SDK 读线程调用。

        ⚠⚠ `Snapshot.state_hz` 从前**只声明、全仓无人赋值** ⇒ 恒为 0 ⇒
        顶栏「控制频率」与主从链路卡那个 Hz 格子**永远显示 `—`** ——
        用户 2026-09-29 报的就是这个。
        """
        self._state_n += 1
        now = time.monotonic()
        if self._state_t0 == 0.0:
            self._state_t0 = now
        elif now - self._state_t0 >= 1.0:
            self._state_hz = self._state_n / (now - self._state_t0)
            self._state_n, self._state_t0 = 0, now

    def _link_tick(self) -> None:
        """统计**遥操链路**速率（1 s 窗口）：主臂=发布、从臂=接收。

        ⚠ 与 `_state_tick` **不是同一个量**：那个量 CDC 上的电机状态帧，
        这个量 zenoh 上我们自己的遥操帧。主从链路卡那格标签是「发布/接收」，
        用的该是这一个。
        """
        self._link_n += 1
        now = time.monotonic()
        if self._link_t0 == 0.0:
            self._link_t0 = now
        elif now - self._link_t0 >= 1.0:
            self._link_hz = self._link_n / (now - self._link_t0)
            self._link_n, self._link_t0 = 0, now

    def _link_reset(self) -> None:
        """清链路速率（会话开始/结束时调）—— 停了就该显示 `—`，不留上一个会话的数。"""
        self._link_n = 0
        self._link_t0 = 0.0
        self._link_hz = 0.0

    def _push(self, st) -> None:
        """在 **SDK 读线程**上被调用 ⇒ 只写槽，不做别的。"""
        self._state_tick()
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
        if self._arm is not None:
            try:
                servo.hold_at_current(self._arm)         # ⛔ 绝不 disable
            except Exception as e:                       # noqa: BLE001
                self._log(f"⚠ 收尾 movej 失败: {e}")
        self._close_sub()
        self._close_pub()                                # ⛔ 不 close ⇒ 进程永久挂死
        if self._arm is not None:
            try:
                self._arm.close()                        # ⛔ 同上
            except Exception:                            # noqa: BLE001
                log.exception("close arm 失败")
        with self._lock:
            self._snap.connected = False

    # ────────────────────────── zenoh 端点 ──────────────────────────

    def _ensure_pub(self, p: TeleopParams):
        """主臂的发布端：**建一次、活一个进程**（跨遥操会话不重建）。

        ⚠⚠ 实测过两个坑，都出在"每轮遥操拆了重建"上：
        ① 遥操停下时没人关 ⇒ TCP 端口一直被这个对象引用着 ⇒ 第二次「启动」抛
           `Can not create a new TCP listener bound to ...: Address already in use`，
           **而且端口要等进程退出才回来**（表现：**只能启动一次**）；
        ② 就算补上关闭，**同一端口上拆了重建**会让订阅↔发布的匹配**间歇性**
           建立不起来（实测 12 轮里 5 轮 `matching=False`）。
        参考实现也是这个形态：主臂发在**常驻** transport 上。

        ⚠ 但 `arm_id` / 端口是**遥操参数**（用户随时可改）⇒ 参数变了必须重建。
        所以比 `_pub_meta` 决定复用还是重建。
        """
        meta = (p.key, int(p.jport))
        if self._pub is not None and self._pub_meta == meta:
            return self._pub
        self._close_pub()
        self._pub = link.Listener(p.jport, p.key)
        self._pub_meta = meta
        return self._pub

    def _close_pub(self) -> None:
        pub, self._pub, self._pub_meta = self._pub, None, None
        if pub is not None:
            try:
                pub.close()                              # ⛔ 同上，必须关
            except Exception:                            # noqa: BLE001
                log.exception("close zenoh 发布端点失败")

    def _open_sub(self, p: TeleopParams):
        """从臂的订阅端：**每会话建/关**。

        ⚠ 与主臂**故意不一样**（照 `litearm-server` 的形态）：**只有主臂 bind
        端口**（`Listener` 是 listen），从臂是连出去、不 bind ⇒ 从臂建/关很廉价，
        而主臂一关一开就要重新抢端口。别把两端做成同一种。
        """
        self._close_sub()
        self._sub = link.Connector(p.peer_host(), p.jport, p.key,
                                   on_frame=self._on_wire)
        return self._sub

    def _close_sub(self) -> None:
        sub, self._sub = self._sub, None
        if sub is not None:
            try:
                sub.close()                              # ⛔ 不 close ⇒ 进程永久挂死
            except Exception:                            # noqa: BLE001
                log.exception("close zenoh 订阅端点失败")

    # ────────────────────────── 遥操 ──────────────────────────
    def _run_teleop(self) -> None:
        p = self._params
        if p is None:                                    # 不该发生：set_teleop 已同步拦过
            self._log("⛔ 没有遥操参数 —— 不启动")
            self._teleop_want = False
            return
        try:
            if p.role == ROLE_MASTER:
                self._run_master(p)
            else:
                self._run_slave(p)
        except Exception as e:                           # noqa: BLE001
            self._log(f"⛔ 遥操异常退出: {e}")
            with self._lock:
                self._snap.error = str(e)
        finally:
            self._teleop_want = False
            self._link_reset()                           # 停了就该显示 `—`，不留旧数
            # ⚠ 从臂的订阅是**每会话**建的 ⇒ 停遥操就得关（否则连接/端口不释放）
            self._close_sub()
            try:
                servo.hold_at_current(self._arm)         # 受控接管
            except Exception as e:                       # noqa: BLE001
                self._log(f"⚠ 收尾 movej 失败: {e}")


    def _run_master(self, p: TeleopParams) -> None:
        """主臂：零重力拖动 → 定频采样 → 发布（spec §6）。**主臂不做任何钳位。**"""
        arm = self._arm
        pub = self._ensure_pub(p)                        # ⚠ 常驻，不每轮重建（见 docstring）
        self._link_reset()                               # 新会话 ⇒ 速率从 0 起算
        self._log(f"主臂监听 {p.key} @ 端口 {p.jport}")
        arm.zero_g_start()
        time.sleep(0.5)                                  # 等过 engage 段
        dt = 1.0 / PUB_HZ
        nxt = time.monotonic() + dt
        while self._teleop_want and not self._stop.is_set():
            st = arm.get_state(refresh=False).value
            if st is not None:
                pub.put(wire.encode_teleop(st.q, st.dq, time.monotonic()))
                self._frames_sent += 1
                self._link_tick()                        # 统计**发布**速率
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

    def _run_slave(self, p: TeleopParams) -> None:
        """从臂：订阅 → 钳位 → `slew_target` → `joint_follow`（0x08）（spec §5）。"""
        arm = self._arm
        self._limits = read_safe_limits(arm)             # 读不到会抛 ⇒ 拒启动
        self._log(f"软限位 {list(zip(self._limits.lo, self._limits.hi))}")
        self._link_reset()                               # 新会话 ⇒ 速率从 0 起算
        self._open_sub(p)
        # ── 对齐（照搬 `_do_align`）：等首帧 → 钳位 → **低速 movej** ──
        # ⚠ 少了这一步，从臂会由 `slew_target` 直接拉过去，速度上限是 `speed_limit`
        #    （J1 到 2.8 rad/s），比 `align_speed=0.15` 快近 20 倍 —— 那是**大幅甩动**。
        #
        self._log("等待主臂首帧并对齐 …")
        aligned = servo.align_to_master(arm, self._slot.take, self._limits)

        # ⚠ **不改任何固件参数** —— K/B 走 `CMD_JOINT_FOLLOW(0x08)` **随帧下发**，
        #    而不是写固件的全局 `mit_kp`/`mit_kd`（后者会连带改坏 `movej`，真机踩过两次）。
        #    随帧下发**只对这一帧生效** ⇒ `movej` 完全不受影响。
        # ⚠ 对齐成败**必须报出来**（这行曾被误删 ⇒ 出了故障却看不出对齐成没成、差多少）
        if aligned is None:
            self._log("⚠ 未对齐（5 s 内没收到主臂帧，或 movej 失败）—— 跟随会逐步修正")
        else:
            self._log(f"✓ 已对齐 → {[round(v, 3) for v in aligned]}")

        # ⚠⚠ 速度上限**照抄 litearm-server 的配置值**（用户裁决 2026-09-28，真机实测）。
        #    曾按 kd 预算收紧（`speed_limit_i = 0.30·tau_max_i/kd_i`），把 J3/J4 压到 **11%**
        #    （5.0 → 0.63/0.573）、腕部 J5~J7 压到 **9~15%**（10/8/13 → 1.2）
        #    ⇒ 用户实测「跟随太慢，有明显的延迟」⇒ 撤掉，回到 S0 验过的 server 原值。
        # ⚠ 这份配置**本来就是配 `B=0.5` 的** —— K/B 随帧下发之后，它才第一次名副其实。
        sl = list(servo.DEFAULT_SPEED_LIMIT)
        # ── 限位墙：位置护栏的**第二道**（第一道是 `clamp_to_limits` 把目标钳进限位）──
        # ⚠ 用**固件原始**软限位 —— 墙自己按 `margin_rad` 内缩。⛔ 别用 `self._limits`：
        #   那份已经内缩过 margin 了，再用会**双重内缩**。
        # ⚠ 真机实证（2026-09-28）：只靠目标钳位**不够** —— 从臂带着柔性去追一个
        #   **恰好贴在边界上**的目标，实测位置会冲过去 ⇒ 越界锁存 `joint_fault`
        #   ⇒ 该轴掉力并连带 `FB_STALE`。断轴先是 J4、换增益后又变成 **J2+J3**
        #   ⇒ **轴会变** ⇒ 是"撞软限位"，不是某个电机坏了。
        _jp = arm.params.all_joint_params()
        wall = JointLimitWall.from_limits(
            [float(x.q_min) for x in _jp], [float(x.q_max) for x in _jp])
        self._log(f"限位墙已接线：margin={wall.margin} rad  "
                  f"stiffness={wall.stiffness}  damping={wall.damping}")
        self._log(f"从臂跟随：K={servo.SETUP_K} B={servo.SETUP_B}"
                  f"（⚠ 不是 yaml 默认档 —— 2026-09-29 起有意整体放大以压过冲，"
                  f"见 servo.SETUP_K 注释；随 0x08 帧每拍下发）")
        self._log(f"speed_limit={sl}（照抄 litearm-server 配置）  hz={SLAVE_HZ:.0f}")
        self._log(f"每拍 1 次下发（CMD_JOINT_FOLLOW；G + 限位墙由**固件**算）"
                  f"⇒ 节拍 {SLAVE_HZ:.0f} Hz（收尾会打印实测值）")

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

        servo.follow(arm, provider, should_stop=should_stop, hz=SLAVE_HZ, wall=wall,
                     speed_limit=sl)

    def _peer_host(self) -> str:
        # 保留给测试/调试：真源已挪到 `TeleopParams.peer_host()`
        p = self._params
        return p.peer_host() if p is not None else "127.0.0.1"

    def _on_wire(self, payload: bytes) -> None:
        """**在 zenoh 线程上**被调用 ⇒ 只写槽。"""
        self._slot.put(payload, time.monotonic())
        self._frames_received += 1
        # ⚠ 在**到达**处计数（不是在被消费处）—— LatestSlot 是 latest-wins，
        #   被覆盖掉的帧也是真到了的，在消费处数会低估速率。
        self._link_tick()
