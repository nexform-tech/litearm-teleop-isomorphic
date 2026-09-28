"""`GripWorker` —— **唯一持有夹爪的线程**。

⚠⚠ **与 `ArmWorker` 代码上完全解耦**（用户裁决 2026-09-28）：独立线程、独立对象、
独立 zenoh session、独立端口、独立生命周期、零共享状态。两者之间**没有任何直接引用**。
界面把两者放在一起显示，仅此而已。

拓扑（与 litearm-device 的 `_master_loop`/`_slave_loop` 同构）::

    主端: 夹爪零重力（自己每拍发零力矩帧，人可掰动）+ 50 Hz pub 32 B 帧
    从端: 连主端 IP:port → 订阅 → 首帧 goto_rad 对齐 → send_mit_frame 位置跟随
          watchdog 超时 **持位**（继续发帧，不掉力、不松开、不自动重连）

⚠ 收尾**绝不失能**（spec §8 rule 4/6）：构造夹爪时必须传
   `disable_on_disconnect=False` —— 它的**默认值是 True**，
   默认走 `disconnect()` 会把电机关掉、当场松掉正夹着的物体。
"""
from __future__ import annotations

import logging
import math
import sys
import threading
import time
from dataclasses import dataclass
from typing import Callable, Optional

from . import link
from .grip_wire import (GRIP_FRAME_BYTES, decode_gripper_teleop,
                        encode_gripper_teleop, gripper_teleop_topic)

log = logging.getLogger("liteteleop.grip")

__all__ = ["ROLE_MASTER", "ROLE_SLAVE", "GRIP_RATE_HZ", "GRIP_WATCHDOG_MS",
           "GRIP_SDK_SRC", "GripNotReady", "GripSnapshot", "GripWorker",
           "assert_sdk_pinned", "pin_grip_sdk",
           "clamp_to_calibrated", "mm_to_openness", "openness_to_rad",
           "travel_mm_of"]

ROLE_MASTER = "master"
ROLE_SLAVE = "slave"

#: 与 litearm-device 的 `rate_hz=50.0`（`gripper_teleop.py:106`）同值。
GRIP_RATE_HZ = 50.0

#: 与臂的 `WATCHDOG_MS`（`arm_worker.py:61`）和参考实现（`:110`）同值。
GRIP_WATCHDOG_MS = 200.0

#: 主从 `travel_mm` 偏差超过这个比例就告警（spec §7.3）。
_MISMATCH_TOL = 0.15

#: 只在 `openness` **没有被 `clamp01` 饱和**的窗口里做标定一致性比对。
#: `openness→1` 时比值会报出主端**原始** `position_mm`（大于其真实 travel）⇒ 会误报。
_MISMATCH_WINDOW = (0.15, 0.85)

#: ⚠⚠ 夹爪 SDK 必须钉死。**本机有两份包名都叫 `litegrip` 的仓**：
#:   `/home/llx/litegrip-python`（github nexform-tech，本设计的目标，src 布局）
#:   `/home/llx/moduangongju/lite-grip`（gitee，被 editable 装成 `litegrip` 2.2.0）
#: 裸 `import litegrip` **落到后者**（实测）。两者的 API 面并不相同
#: （`close_sign`/`calibrated`/`load_template` 只在目标那份有）⇒ 走错仓从
#: `AttributeError` 到**静默语义漂移**都可能。
#: 用户裁决 2026-09-28：SDK 采用 `litegrip-python`。
GRIP_SDK_SRC = "/home/llx/litegrip-python/src"


class GripNotReady(RuntimeError):
    """夹爪没做好遥操准备（未标定 / 零行程 / `rad_to_mm` 为 0）。"""


# ────────────────────────────── SDK 钉死（spec §9.4）──────────────────────────────

def assert_sdk_pinned(mod_file: str, src: str = GRIP_SDK_SRC) -> None:
    """判据：导入后 `litegrip.__file__` 必须落在 `src` 下，否则**抛 `SystemExit`**。

    ⚠ **断言不能省，只 `sys.path.insert` 不算数。**
    本机那份 editable 安装是用 `sys.meta_path.append(_EditableFinder)` 注册的
    （`__editable___litegrip_2_2_0_finder.py` 的 `install()`），排在 `PathFinder`
    **之后** ⇒ `sys.path.insert(0, ...)` 目前**压得住**（实测）。
    但这是**实现细节**：一旦改成 `sys.meta_path.insert` 式注册，
    `sys.path` 插入就会**静默失效** —— 只有这条断言能抓住它。
    与 `liteteleop/__main__.py:11-21` 的 `_pin_sdk()` 同款纪律。
    """
    if not str(mod_file).startswith(str(src)):
        raise SystemExit(
            f"⛔ litegrip 导入自 {mod_file}，不是 {src} —— 环境里有另一份抢先了")


def pin_grip_sdk(src: str = GRIP_SDK_SRC) -> str:
    """把夹爪 SDK 钉到 `src`，断言后返回实际导入路径（供启动日志打印）。"""
    if src not in sys.path:
        sys.path.insert(0, src)
    import litegrip                          # noqa: PLC0415 - 故意延迟到 pin 之后
    assert_sdk_pinned(litegrip.__file__, src)
    return str(litegrip.__file__)


# ────────────────────────────── 换算（spec §6）──────────────────────────────

def travel_mm_of(cfg) -> float:
    """满行程 mm。与 SDK 的 `CalibrationData.travel_mm`（`models.py:163`）同义，
    但我们从 `cfg` 的**两个限位**自己算 —— 跑起来之后手头只有 `cfg`，
    且这样 `travel_mm` 与 `close_sign` 出自**同一组数**，不会分叉。"""
    return (abs(float(cfg.pos_open_rad) - float(cfg.pos_closed_rad))
            * float(cfg.rad_to_mm))


def _clamp01(x: float) -> float:
    return 0.0 if x < 0.0 else (1.0 if x > 1.0 else x)


def mm_to_openness(cfg, position_mm: float) -> float:
    """读侧：`position_mm`（SDK 算的，**已含 `close_sign`**）→ `openness[0,1]`。"""
    t = travel_mm_of(cfg)
    return _clamp01(float(position_mm) / t) if t > 0 else 0.0


def openness_to_rad(cfg, openness: float) -> float:
    """`openness[0,1]` → 电机弧度（用**本机**标定）。

    复刻 SDK 的 `goto` 公式（`litegrip/src/litegrip/gripper.py:1183-1185`）::

        position_rad = pos_closed_rad - close_sign * position_mm / rad_to_mm
        position_mm  = openness * travel_mm

    ⚠⚠ `close_sign` **不能省**。参考实现 `gripper_teleop.py:155-163` 漏了它
    ⇒ 反装夹爪会**镜像**（`openness=1` 算出 -3.096 rad，远出标定限位）。
    正装时 `close_sign == +1` ⇒ 与参考实现**逐位相同**，不影响互通。
    """
    t = travel_mm_of(cfg)
    return (float(cfg.pos_closed_rad)
            - float(cfg.close_sign) * (_clamp01(openness) * t) / float(cfg.rad_to_mm))


def clamp_to_calibrated(cfg, q: float) -> float:
    """把 `q` 钳进**本机**标定区间（spec §8.2）。

    两个限位谁大谁小都可能（反装），所以取 min/max 而不是假定顺序 ——
    与 SDK 自己的 `goto_rad` 钳位同源（`gripper.py:1208-1210`）。
    """
    lo = min(float(cfg.pos_closed_rad), float(cfg.pos_open_rad))
    hi = max(float(cfg.pos_closed_rad), float(cfg.pos_open_rad))
    return max(lo, min(hi, float(q)))


def check_ready(cfg) -> None:
    """spec §8.1 的三条前置。⚠ **必须在 `enable()` 之前调。**

    `send_mit_frame` 与 `goto_rad` **都不检查 `calibrated`** —— SDK 的
    `_check_calibrated` 只保护 `open`/`close`/`grasp`
    （`litegrip/src/litegrip/actions.py:200-215`）。不拦就会拿**占位默认值**
    `pos_closed_rad=1.14` / `pos_open_rad=0.0`（`models.py:90-91`）当真实限位用。

    ⚠ 零行程判据写在 **rad 空间**（`<= 1e-6`），与 SDK 同款 ——
    写成 mm 空间的 `travel_mm == 0` 是**更弱**的条件。
    """
    if not bool(getattr(cfg, "calibrated", False)):
        raise GripNotReady(
            "夹爪未标定：pos_closed_rad / pos_open_rad 还是占位默认值，方向是猜的。"
            "先 load_calibration() / load_template()，或跑一次 zero()。")
    if abs(float(cfg.pos_closed_rad) - float(cfg.pos_open_rad)) <= 1e-6:
        raise GripNotReady("行程为零：pos_closed_rad 与 pos_open_rad 相同 —— 重新标定。")
    if not float(cfg.rad_to_mm):
        raise GripNotReady("rad_to_mm 为 0：openness↔rad 换算会除零。")


def _default_gripper_factory(channel: str):
    """默认夹爪工厂。⚠ `disable_on_disconnect=False` **不能省**（spec §8 rule 6）。

    另注 SDK 的一个怪癖（`gripper.py:209-212`）：`channel` 只有在**与默认值不同**
    或 `config is None` 时才生效。我们这里不传 `config` ⇒ `channel` 生效。
    """
    import litegrip                          # noqa: PLC0415
    return litegrip.LiteGrip(channel=channel, disable_on_disconnect=False)


# ────────────────────────────── 快照 ──────────────────────────────

@dataclass
class GripSnapshot:
    """**只读**快照 —— 界面只看这个，不碰夹爪。"""

    connected: bool = False
    role: str = ""
    topic: str = ""
    openness: float = 0.0
    position_mm: float = 0.0
    force_n: float = 0.0
    travel_mm: float = 0.0
    frames_sent: int = 0
    frames_received: int = 0
    loop_hz: float = 0.0
    matching: bool = False              # 主端：有无订阅者（**是布尔不是计数**）
    frame_age: Optional[float] = None   # 从端：最新一帧有多老（s）；None = 从未收到
    stale: bool = False                 # 从端：watchdog 超时、正在持位
    watchdog_trips: int = 0
    teleop_active: bool = False         # 由 snapshot() 从 _want 填（供界面按钮刷新）
    rejected: int = 0                   # 被**协议边界**丢弃的帧（非有限值，见 §8 rule 9）
    mismatch: str = ""                  # 主从标定不一致告警（§7.3），空 = 无
    error: str = ""


# ────────────────────────────── worker ──────────────────────────────

class GripWorker:
    """独占夹爪的线程。生命周期：`start()` → … → `stop()`。"""

    def __init__(self, role: str, gcan: str, grip_id: str = "gripA",
                 gpeer: str = "127.0.0.1", gport: int = 17448,
                 rate_hz: float = GRIP_RATE_HZ,
                 kp: Optional[float] = None, kd: Optional[float] = None,
                 align: bool = True, watchdog_ms: float = GRIP_WATCHDOG_MS,
                 gripper_factory: Optional[Callable[[str], object]] = None,
                 on_state: Optional[Callable[[GripSnapshot], None]] = None,
                 on_log: Optional[Callable[[str], None]] = None):
        if role not in (ROLE_MASTER, ROLE_SLAVE):
            raise ValueError(f"role 必须是 {ROLE_MASTER}/{ROLE_SLAVE}，收到 {role!r}")
        self.role = role
        self.gcan = gcan
        self.grip_id = grip_id
        self.gpeer = gpeer
        self.gport = int(gport)
        self.rate_hz = float(rate_hz)
        self.kp = kp
        self.kd = kd
        self.align = bool(align)
        self.watchdog_ms = float(watchdog_ms)
        self._topic = gripper_teleop_topic(grip_id)
        self._factory = gripper_factory or _default_gripper_factory
        self._on_state = on_state
        self._on_log = on_log

        self._grip = None
        self._cfg = None
        self._pub = None            # 主端：link.Listener
        self._sub = None            # 从端：link.Connector
        self._lock = threading.Lock()
        self._snap = GripSnapshot(role=role, topic=self._topic)
        self._stop = threading.Event()
        self._want = False
        self._thread: Optional[threading.Thread] = None
        self._frames_sent = 0
        self._frames_received = 0
        self._watchdog_trips = 0
        self._mismatch = ""
        self._mismatch_checked = False
        self._rejected = 0                  # 边界丢弃的帧数（非有限值）
        self._reject_warned = False
        self._loops = 0
        self._loop_hz = 0.0
        self._hz_t0 = 0.0
        self._hz_n0 = 0

    # ────────────────────── 生命周期 ──────────────────────

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name="GripWorker", daemon=True)
        self._thread.start()

    def stop(self, timeout: float = 5.0) -> None:
        """停止并收尾。**幂等**。⚠ 收尾**不失能**（spec §8 rule 4/6）。"""
        self._stop.set()
        self._want = False
        t = self._thread
        if t is not None:
            t.join(timeout=timeout)
            if t.is_alive():
                self._log("⚠ GripWorker 未在超时内退出")
        self._thread = None

    def set_teleop(self, on: bool) -> None:
        """请求启动/停止夹爪遥操。**与臂的开关是两个独立控件**（spec §2）。"""
        self._want = bool(on)

    def snapshot(self) -> GripSnapshot:
        with self._lock:
            s = self._snap
            s.frames_sent = self._frames_sent
            s.frames_received = self._frames_received
            s.watchdog_trips = self._watchdog_trips
            s.loop_hz = self._loop_hz
            s.mismatch = self._mismatch
            s.rejected = self._rejected
            s.teleop_active = bool(self._want)
            if self._pub is not None:
                s.matching = self._pub.matching
            return s

    def _log(self, msg: str) -> None:
        log.info(msg)
        if self._on_log is not None:
            try:
                self._on_log(msg)
            except Exception:                    # noqa: BLE001
                log.exception("on_log 回调炸了")

    def _publish_state(self) -> None:
        if self._on_state is not None:
            try:
                self._on_state(self.snapshot())
            except Exception:                    # noqa: BLE001
                log.exception("on_state 回调炸了")

    # ────────────────────── 主循环 ──────────────────────

    def _run(self) -> None:
        try:
            self._grip = self._factory(self.gcan)
            self._grip.connect()
            self._grip.load_calibration()
            self._cfg = self._grip.config
            # ⚠⚠ **必须在 enable() 之前** —— 否则未标定的夹爪会先被使能、再被拒
            check_ready(self._cfg)
            self._grip.enable()
            with self._lock:
                self._snap.connected = True
                self._snap.travel_mm = travel_mm_of(self._cfg)
            self._log(f"夹爪已就绪：{self.gcan}  travel={travel_mm_of(self._cfg):.2f} mm  "
                      f"close_sign={float(self._cfg.close_sign):+.0f} rad_to_mm="
                      f"{float(self._cfg.rad_to_mm):.2f}")
            if self.role == ROLE_MASTER:
                # ⚠⚠ **主端的 Listener 建一次、活一个进程** —— 不能每轮遥操拆了重建。
                #
                # 实测（2026-09-28）：若在 `_master_loop` 里每轮新建，
                #   ① 遥操停下后端口不释放（`_run_teleop` 停时没人关）⇒ 第二次启动必定
                #      `Can not create a new TCP listener bound to ...: Address already in use`；
                #   ② 就算补上关闭，**同一端口上拆了重建**会让订阅↔发布的匹配**间歇性**
                #      建立不起来：12 轮重启里 5 轮 `matching=False`（从端一帧收不到，
                #      而主端照发 ⇒ 界面上显示「⚠ 未匹配（发了没人在收）」）。
                #      改成常驻后同样 12 轮：**0 轮失败**。
                # 这也与参考实现同形 —— litearm-device 的主端就是发在**常驻** transport 上。
                self._pub = link.Listener(self.gport, self._topic)
                self._log(f"主端夹爪：已监听 {self._topic} @ 端口 {self.gport}"
                          "（常驻，跨遥操会话不重建）")
            while not self._stop.is_set():
                if self._want:
                    self._run_teleop()
                else:
                    time.sleep(0.1)
                    self._publish_state()
        except Exception as e:                   # noqa: BLE001
            with self._lock:
                self._snap.error = str(e)
            self._log(f"⛔ 夹爪 {e}")
        finally:
            self._teardown()

    def _run_teleop(self) -> None:
        try:
            if self.role == ROLE_MASTER:
                self._master_loop()
            else:
                self._slave_loop()
        except Exception as e:                   # noqa: BLE001
            self._log(f"⛔ 夹爪遥操异常退出: {e}")
            with self._lock:
                self._snap.error = str(e)
        finally:
            self._want = False
            self._handoff()
            # ⚠⚠ **这一步不能省，也不能只放在 `_teardown` 里。**
            #    从端的订阅是每会话建的，不关就会占着连接/端口。
            self._close_sub()

    def _close_sub(self) -> None:
        """关掉**本次遥操**的订阅端点。

        ⚠ `_sub` 是**每次遥操会话**建的（从端才有），所以每次停下都必须关 ——
        否则它一直被引用着，端口/连接不释放。
        参考实现的 slave 也是每次 `enter()` 建、`exit()` 关
        （`litearm_device` 的 `GripTeleopController._close_sub_tp`）。

        ⚠⚠ **`_pub` 不在这里关** —— 主端的 Listener 是**常驻**的（见 `_run`），
        由 `_teardown` 在进程退出时统一关。这是实测结论，不是随手写的：
        每轮拆了重建会让匹配间歇性失败。
        """
        ep, self._sub = self._sub, None
        if ep is not None:
            try:
                ep.close()                       # ⛔ 不 close ⇒ 进程永久挂死（link.py:6-7）
            except Exception:                    # noqa: BLE001
                log.exception("close zenoh 订阅端点失败")

    def _handoff(self) -> None:
        """遥操停下时的交接：**发一帧持位帧**，绝不失能。"""
        g = self._grip
        if g is None:
            return
        try:
            if self.role == ROLE_MASTER:
                # 内部就是「按当前位、kp/kd 取 cfg」的一帧（gripper.py:520-526）
                g.exit_zero_gravity()
            else:
                st = g.get_state(wait=False)
                q = clamp_to_calibrated(self._cfg, st.position_rad)
                g.send_mit_frame(q=q, kp=self._kp(), kd=self._kd(), dq=0.0)
            self._log("已交接：夹爪保持当前位置，**不失能**")
        except Exception as e:                   # noqa: BLE001
            self._log(f"⚠ 交接持位帧失败: {e}")
        self._publish_state()

    # ────────────────────── 主端 ──────────────────────

    def _master_loop(self) -> None:
        """零重力（每拍自己发零力矩帧）+ 定频发布当前位置。

        ⚠ **不调 `enter_zero_gravity()`**：它只发**一帧** bootstrap，
        docstring 明说「caller must poll/sustain to sustain the mode」
        （`gripper.py:496-518`）。自己每拍发才是持续维持（参考实现 `:220-224` 同）。
        ⚠ 零力矩帧的 `q` 传 `0.0`：`kp=kd=0` 时 `q` 不参与力矩计算。
        """
        g = self._grip
        # ⚠ `self._pub` 已在 `_run` 里建好（常驻，跨遥操会话不重建）—— 这里只用，不建。
        self._log(f"主端夹爪：零重力拖动 · 发布 {self._topic} @ 端口 {self.gport} "
                  f"· {self.rate_hz:.0f} Hz")
        self._hz_reset()
        dt = 1.0 / self.rate_hz
        nxt = time.monotonic() + dt
        while self._want and not self._stop.is_set():
            t0 = time.monotonic()
            g.send_mit_frame(q=0.0, kp=0.0, kd=0.0)
            g.poll(timeout_s=0.0)
            st = g.get_state(wait=False)
            openness = mm_to_openness(self._cfg, st.position_mm)
            if not (math.isfinite(openness) and math.isfinite(st.position_mm)):
                # ⚠⚠ 主端也守在边界上（§8 rule 9）：读数坏掉时**不发帧**。
                #    发出去的话从端会照单全收（见从端那段的说明）；
                #    不发 ⇒ 从端的 watchdog 超时 ⇒ **持位**，是安全那一侧。
                self._rejected += 1
                if not self._reject_warned:
                    self._reject_warned = True
                    self._log(f"⚠ 读数非有限值（openness={openness!r}）—— 停止发帧，"
                              "由从端的 watchdog 转持位")
            else:
                self._pub.put(encode_gripper_teleop(openness, st.position_mm,
                                                    st.force_n, time.monotonic()))
                self._frames_sent += 1
                with self._lock:
                    self._snap.openness = openness
                    self._snap.position_mm = float(st.position_mm)
                    self._snap.force_n = float(st.force_n)
            self._publish_state()
            nxt = self._pace(t0, nxt, dt)

    # ────────────────────── 从端 ──────────────────────

    def _slave_loop(self) -> None:
        g = self._grip
        slot = link.LatestSlot()

        def _on_wire(payload: bytes) -> None:
            # ⚠ **必须单参包装**：`LatestSlot.put(payload, now)` 收两个参数，
            #   而 zenoh 回调只给一个（`link.py:118`）⇒ 直接 `on_frame=slot.put` 会 TypeError。
            #   且在 zenoh 自己的线程上 ⇒ **只许写槽**。
            slot.put(payload, time.monotonic())

        self._sub = link.Connector(self.gpeer, self.gport, self._topic,
                                  on_frame=_on_wire)
        self._log(f"从端夹爪：订阅 {self._topic} @ {self.gpeer}:{self.gport} "
                  f"· align={self.align} · watchdog={self.watchdog_ms:.0f} ms")

        # ⚠⚠ 循环入口先把 q_cmd 定成**本机实测位置**。
        #    首帧到达前 q_cmd 若是 0.0，那是一个**真实位置指令** —— 按装法不同
        #    可能直冲机械限位（正装 0.0 = 全开，反装 0.0 = 全闭）。
        q_cmd = clamp_to_calibrated(self._cfg, g.get_state(wait=False).position_rad)
        stale = False

        if self.align:
            first = self._wait_first_frame(slot, timeout_s=5.0)
            if first is not None:
                o0 = _clamp01(first[0])
                q_cmd = openness_to_rad(self._cfg, o0)
                self._log(f"对齐首帧 → openness={o0:.3f} → {q_cmd:+.4f} rad")
                try:
                    g.goto_rad(q_cmd, kp=self.kp, kd=self.kd, duration=1.0)
                except Exception as e:           # noqa: BLE001
                    self._log(f"⚠ 对齐 goto_rad 失败: {e}")
            else:
                self._log("⚠ 5 s 内没收到主端帧 —— 保持当前位置继续发帧")

        self._hz_reset()
        dt = 1.0 / self.rate_hz
        nxt = time.monotonic() + dt
        while self._want and not self._stop.is_set():
            t0 = time.monotonic()
            payload, _ts = slot.take()
            if payload is not None and len(payload) == GRIP_FRAME_BYTES:
                openness, position_mm, force_n, _t = decode_gripper_teleop(payload)
                if not (math.isfinite(openness) and math.isfinite(position_mm)):
                    # ⚠⚠ **协议边界：非有限值一律拒收**（§8 rule 9），与臂侧
                    #    `safety.clamp_to_limits` 的 `NonFiniteTarget`（`safety.py:142-144`）
                    #    同款纪律。
                    #    不拒的话实测是这样：`_clamp01(NaN)` 返回 NaN、
                    #    `clamp_to_calibrated` 里 `min(hi, NaN)` **返回 hi**
                    #    ⇒ 一条 NaN 帧把从端命令到**全闭限位**，而且 `error` 是空的。
                    #    `±inf` 同样被折成端点。⇒ **错在危险一侧且静默**，必须拦。
                    self._rejected += 1
                    if not self._reject_warned:
                        self._reject_warned = True
                        self._log(f"⚠ 丢弃非有限值帧（openness={openness!r}）—— "
                                  "保持当前位置不动，**不是**折成某个端点")
                else:
                    q_cmd = openness_to_rad(self._cfg, _clamp01(openness))
                    stale = False
                    self._frames_received += 1
                    self._check_mismatch(_clamp01(openness), float(position_mm))
                    with self._lock:
                        self._snap.openness = _clamp01(openness)
                        self._snap.position_mm = float(position_mm)
                        self._snap.force_n = float(force_n)
            else:
                # `peek_age` 对「从未收到」返回 None（`link.py:170-183`）
                # ⇒ `is not None` 正好等于「收到过首帧之后 watchdog 才生效」
                age = slot.peek_age(time.monotonic())
                if age is not None and age * 1000.0 > self.watchdog_ms:
                    if not stale:
                        self._watchdog_trips += 1
                        self._log(f"⚠ watchdog 超时 {self.watchdog_ms:.0f} ms "
                                  "—— **持位**（继续发帧：不掉力、不松开、不重连）")
                    stale = True
            # §8.2：即使 stale 也照发，只是 q_cmd 不变
            q_cmd = clamp_to_calibrated(self._cfg, q_cmd)
            g.send_mit_frame(q=q_cmd, kp=self._kp(), kd=self._kd(), dq=0.0)
            g.poll(timeout_s=0.0)
            with self._lock:
                self._snap.stale = stale
                self._snap.frame_age = slot.peek_age(time.monotonic())
            self._publish_state()
            nxt = self._pace(t0, nxt, dt)

    def _wait_first_frame(self, slot, timeout_s: float) -> Optional[tuple]:
        deadline = time.monotonic() + timeout_s
        while (self._want and not self._stop.is_set()
               and time.monotonic() < deadline):
            payload, _ts = slot.take()
            if payload is not None and len(payload) == GRIP_FRAME_BYTES:
                return decode_gripper_teleop(payload)
            time.sleep(0.01)
        return None

    def _check_mismatch(self, openness: float, position_mm: float) -> None:
        """spec §7.3：从端能同时拿到 `openness` 与 `position_mm`，
        于是 `position_mm / openness ≈ 主端的 travel_mm`。差太多就告警。

        挡的是「主从装的是不同型号夹爪」—— 不报错、只是动作幅度悄悄不对。
        ⚠ 只在**没被 `clamp01` 饱和**的窗口里比（`openness→1` 会把主端**原始**
        `position_mm` 报出来 ⇒ 误报）；⚠ **只告警一次**。
        """
        if self._mismatch_checked:
            return
        lo, hi = _MISMATCH_WINDOW
        if not (lo <= openness <= hi):
            return
        implied = float(position_mm) / openness
        mine = travel_mm_of(self._cfg)
        if implied <= 0 or mine <= 0:
            return
        dev = abs(implied - mine) / mine
        if dev > _MISMATCH_TOL:
            self._mismatch_checked = True
            self._mismatch = (f"主从夹爪标定可能不一致：主端 travel≈{implied:.1f} mm，"
                              f"本端 {mine:.1f} mm（偏差 {dev * 100:.0f}%）"
                              " —— 运动幅度会被按比例缩放")
            self._log(f"⚠ {self._mismatch}")

    # ────────────────────── 辅助 ──────────────────────

    def _kp(self) -> float:
        """增益：显式给的优先，否则用 SDK 装好标定后的 `cfg.kp`。

        ⚠ 参考实现的兜底是硬编码 `100.0`（`:319`），恰好等于 SDK 默认 `kp`
        （`models.py:83`）⇒ 正常情况等价；用 `cfg.kp` 还能带上标定文件里的值。
        """
        return float(self.kp) if self.kp is not None else float(self._cfg.kp)

    def _kd(self) -> float:
        return float(self.kd) if self.kd is not None else float(self._cfg.kd)

    def _hz_reset(self) -> None:
        self._loops = 0
        self._loop_hz = 0.0
        self._hz_t0 = 0.0
        self._hz_n0 = 0

    def _pace(self, t0: float, nxt: float, dt: float) -> float:
        """实测环频（每 1 s 更新一次）+ 定拍。返回下一个应醒时刻。"""
        self._loops += 1
        now = time.monotonic()
        if self._hz_t0 == 0.0:
            self._hz_t0, self._hz_n0 = now, self._loops
        elif now - self._hz_t0 >= 1.0:
            self._loop_hz = (self._loops - self._hz_n0) / (now - self._hz_t0)
            self._hz_t0, self._hz_n0 = now, self._loops
        r = nxt - now
        if r > 0:
            time.sleep(r)
        nxt += dt
        if nxt < time.monotonic():
            nxt = time.monotonic() + dt
        return nxt

    def _teardown(self) -> None:
        """收尾顺序（spec §8 rule 5）：交接持位 → 关 zenoh → 断 CAN（**不失能**）。"""
        self._want = False
        self._close_sub()
        pub, self._pub = self._pub, None      # 常驻的发布端在这里统一关
        if pub is not None:
            try:
                pub.close()                   # ⛔ 不 close ⇒ 进程永久挂死（link.py:6-7）
            except Exception:                  # noqa: BLE001
                log.exception("close zenoh 发布端点失败")
        if self._grip is not None:
            try:
                # ⚠ 构造时传了 disable_on_disconnect=False ⇒ 这一步**不失能**
                self._grip.disconnect()
            except Exception:                    # noqa: BLE001
                log.exception("close 夹爪失败")
        with self._lock:
            self._snap.connected = False
        self._publish_state()
