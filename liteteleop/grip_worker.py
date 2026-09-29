"""`GripWorker` —— **唯一持有夹爪的线程**。

⚠⚠ **与 `ArmWorker` 代码上完全解耦**（用户裁决 2026-09-28）：独立线程、独立对象、
独立 zenoh session、独立端口、独立生命周期、零共享状态。两者之间**没有任何直接引用**。
界面把两者放在一起显示，仅此而已。

遥操本身**不再由本仓实现**：协议、环、watchdog、对齐、交接全部下沉到 litegrip SDK 的
`LiteGrip.teleop_start/teleop_stop/teleop_status`（`litegrip/src/litegrip/teleop.py`）。
本类只做三件事：

  1. 按需把 SDK 拉起来（连接 → 标定 → 使能 → `teleop_start`），并**独占**它；
  2. 以 `poll_s` 的周期把 `teleop_status()` 翻成界面要的 `GripSnapshot`；
  3. 收尾**绝不失能**（spec §8 rule 4/6）：`teleop_stop()` 让夹爪保持当前位置，
     `disconnect()` 也带 `disable_on_disconnect=False`。

⚠ 线上帧格式由 SDK 负责，与本仓再无副本 —— 本仓的 `grip_wire.py` 已删除。
"""
from __future__ import annotations

import logging
import sys
import threading
from dataclasses import dataclass
from typing import Callable, Optional

log = logging.getLogger("liteteleop.grip")

__all__ = ["ROLE_MASTER", "ROLE_SLAVE", "GRIP_RATE_HZ", "GRIP_WATCHDOG_MS",
           "POLL_S", "GRIP_SDK_SRC", "GripNotReady", "GripSnapshot", "GripWorker",
           "assert_sdk_pinned", "pin_grip_sdk", "check_ready", "travel_mm_of"]

ROLE_MASTER = "master"
ROLE_SLAVE = "slave"

#: 与 SDK 的环频（`teleop.py` 的 `rate_hz=50.0`）同值。
GRIP_RATE_HZ = 50.0

#: 与臂的 `WATCHDOG_MS`（`arm_worker.py:61`）同值。
GRIP_WATCHDOG_MS = 200.0

#: 界面快照的刷新周期（秒）。SDK 自己按 `rate_hz` 跑环，这里只是**读**它的状态。
POLL_S = 0.1

#: 主从 `travel_mm` 偏差超过这个比例就告警（spec §7.3）。
_MISMATCH_TOL = 0.15

#: 只在 `openness` **没有被 `clamp01` 饱和**的窗口里做标定一致性比对。
#: `openness→1` 时比值会报出主端**原始** `position_mm`（大于其真实 travel）⇒ 会误报。
_MISMATCH_WINDOW = (0.15, 0.85)

#: ⚠⚠ 夹爪 SDK 必须钉死。**本机有两份包名都叫 `litegrip` 的仓**：
#:   `/home/llx/litegrip-python`（github nexform-tech，本设计的目标，src 布局）
#:   `/home/llx/moduangongju/lite-grip`（gitee，被 editable 装成 `litegrip` 2.2.0）
#: 裸 `import litegrip` **落到后者**（实测）。两者的 API 面并不相同
#: （`teleop_start`/`close_sign`/`calibrated` 只在目标那份有）⇒ 走错仓从
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


# ────────────────────────────── 前置 / 换算 ──────────────────────────────

def travel_mm_of(cfg) -> float:
    """满行程 mm。与 SDK 的 `CalibrationData.travel_mm`（`models.py:163`）同义，
    但我们从 `cfg` 的**两个限位**自己算 —— 跑起来之后手头只有 `cfg`，
    且这样 `travel_mm` 与 `close_sign` 出自**同一组数**，不会分叉。"""
    return (abs(float(cfg.pos_open_rad) - float(cfg.pos_closed_rad))
            * float(cfg.rad_to_mm))


def check_ready(cfg) -> None:
    """spec §8.1 的三条前置。⚠ **必须在 `enable()` 之前调。**

    ⚠ 判据与 SDK 的 `teleop.check_ready` 同义（SDK 也会在 `teleop_start` 里查一遍），
    这里**再留一份**是因为**顺序**：SDK 的检查发生在 `teleop_start` 里，而那时
    调用方已经 `enable()` 过了。未标定的夹爪先用占位默认值
    （`pos_closed_rad=1.14` / `pos_open_rad=0.0`，`models.py:90-91`）被使能、
    再被拒 —— 我们要的是**先拒、后使能**。

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
    teleop_active: bool = False         # 供界面按钮刷新
    rejected: int = 0                   # 被**协议边界**丢弃的帧（非有限值，见 §8 rule 9）
    send_failed: int = 0                # send_mit_frame 返回 False 的次数（§8 rule 10）
    fault: str = ""                     # 夹爪自己报的 error_code != 1（§8 rule 10）
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
                 poll_s: float = POLL_S,
                 gripper_factory: Optional[Callable[[str], object]] = None,
                 on_state: Optional[Callable[[GripSnapshot], None]] = None,
                 on_log: Optional[Callable[[str], None]] = None):
        if role not in (ROLE_MASTER, ROLE_SLAVE):
            raise ValueError(f"role 必须是 {ROLE_MASTER}/{ROLE_SLAVE}，收到 {role!r}")
        # ⚠ 退化的构造参数在**这里**就拒掉，不要留到线程里才崩：
        #    `rate_hz <= 0` / `watchdog_ms <= 0` 会被 SDK 的 `GripperTeleop` 拒掉
        #    （`teleop.py` 的 `__init__`）—— 但从线程里抛出来的表现是"点了按钮没反应"，
        #    而且 `watchdog_ms <= 0` 的语义是"从端永远判 stale ⇒ 一动不动"，
        #    是个"能启动、但什么都不做"的静默无用配置。构造时就拒更清楚。
        if not float(rate_hz) > 0.0:
            raise ValueError(f"rate_hz 必须 > 0，收到 {rate_hz!r}")
        if not float(watchdog_ms) > 0.0:
            raise ValueError(f"watchdog_ms 必须 > 0，收到 {watchdog_ms!r}")
        if not float(poll_s) > 0.0:
            raise ValueError(f"poll_s 必须 > 0，收到 {poll_s!r}")
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
        self.poll_s = float(poll_s)
        self._topic = f"litearm/v4/{grip_id}/gripper_teleop"
        self._factory = gripper_factory or _default_gripper_factory
        self._on_state = on_state
        self._on_log = on_log

        self._grip = None
        self._cfg = None
        self._lock = threading.Lock()
        self._snap = GripSnapshot(role=role, topic=self._topic)
        self._stop = threading.Event()
        self._want = False
        self._thread: Optional[threading.Thread] = None
        self._session = False               # 本次会话是否已经 teleop_start 过
        self._watchdog_trips = 0
        self._prev_stale = False
        self._mismatch = ""
        self._mismatch_checked = False

    # ────────────────────── 生命周期 ──────────────────────

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name="GripWorker", daemon=True)
        self._thread.start()

    def is_alive(self) -> bool:
        """worker 线程是否仍在跑（收尾超时后调用方要据此决定能不能重建）。"""
        t = self._thread
        return bool(t is not None and t.is_alive())

    def stop(self, timeout: float = 5.0) -> None:
        """停止并收尾。**幂等**。⚠ 收尾**不失能**（spec §8 rule 4/6）。"""
        self._stop.set()
        self._want = False
        t = self._thread
        if t is not None:
            t.join(timeout=timeout)
            if t.is_alive():
                # ⚠⚠ **超时时不要把 `self._thread` 清掉。** 清了就没人知道那条线程
                #    还活着 ⇒ 调用方（`_on_grip_state` / `_ensure_grip_worker`）
                #    会再建一个 worker，两个线程同时抢**同一个 CAN** 与
                #    **同一个 zenoh 端口**（旧的那个要等收尾才放端口）。
                #    保留引用 ⇒ 再点一次「停止」能重试 join，`is_alive()` 也能被问到。
                self._log(f"⚠ GripWorker 未在 {timeout:.0f}s 内退出 —— 它仍在运行；"
                          "暂不重建（再点一次「停止夹爪遥操」可重试收尾）")
                return
            self._thread = None

    def set_teleop(self, on: bool) -> None:
        """请求启动/停止夹爪遥操。**与臂的开关是两个独立控件**（spec §2）。"""
        self._want = bool(on)

    def snapshot(self) -> GripSnapshot:
        with self._lock:
            return GripSnapshot(**vars(self._snap))

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
            # ⚠⚠ **必须在 enable() 之前** —— 见 `check_ready` 的说明。
            check_ready(self._cfg)
            # ⚠⚠ `enable()` **不抛异常** —— 失败时返回一个 falsy 的 `EnableResult`
            #    （`EnableResult.__bool__` 就是 `self.ok`；SDK `actions.py:401-443`）。
            #    不查的实测后果：电机没使能，此后 SDK 的 `send_mit_frame` 会**永远
            #    静默返回 False** ⇒ 界面照显示"已发 N 帧 · 跟随中"，可夹爪其实是软的。
            if not self._grip.enable():
                raise RuntimeError(
                    "夹爪使能失败（enable() 返回假值）—— 电机没有使能；"
                    "此时 send_mit_frame 会静默返回 False、夹爪是软的。"
                    "检查电机供电 / CAN 接线 / 故障码。")
            with self._lock:
                self._snap.connected = True
                self._snap.travel_mm = travel_mm_of(self._cfg)
            self._log(f"夹爪已就绪：{self.gcan}  travel={travel_mm_of(self._cfg):.2f} mm  "
                      f"close_sign={float(self._cfg.close_sign):+.0f} rad_to_mm="
                      f"{float(self._cfg.rad_to_mm):.2f}")
            self._poll_loop()
        except Exception as e:                   # noqa: BLE001
            with self._lock:
                self._snap.error = str(e)
            self._log(f"⛔ 夹爪 {e}")
        finally:
            self._teardown()

    def _poll_loop(self) -> None:
        """按 `poll_s` 读 SDK 的状态；请求变了就启停 SDK 的遥操会话。"""
        while not self._stop.is_set():
            st = self._grip.teleop_status()
            running = bool(st.get("active"))
            # ⚠⚠ 这条必须排在"想启动"**之前**：会话自己结束（SDK 的环退出 / CAN 出错）
            #    时 `_want` 还是 True，若先判 `_want and not running` 就会**每一拍都
            #    重启一次**（重启风暴）。这里明确收手、清 `_want`，让界面弹回
            #    「启动」，由用户决定要不要再来一次。
            if self._session and not running:
                self._session = False
                if self._want:
                    self._want = False
                    self._log("⚠ 夹爪遥操会话已自行结束 —— 请重新点「启动夹爪遥操」")
            elif self._want and not running:
                st = self._start_session()
                running = bool(st.get("active"))
            elif not self._want and running:
                self._grip.teleop_stop()
                self._session = False
                running = False
                self._log("夹爪遥操已停止：夹爪保持当前位置，**不失能**")
                st = self._grip.teleop_status()
            self._update(st, running)
            self._publish_state()
            self._stop.wait(self.poll_s)

    def _start_session(self) -> dict:
        st = self._grip.teleop_start(
            self.role, link="zenoh",
            host=None if self.role == ROLE_MASTER else self.gpeer,
            port=self.gport, grip_id=self.grip_id,
            kp=self.kp, kd=self.kd, align=self.align,
            watchdog_s=self.watchdog_ms / 1000.0, rate_hz=self.rate_hz)
        self._session = True
        if self.role == ROLE_MASTER:
            self._log(f"主端夹爪：零重力拖动 · 发布 {self._topic} @ 端口 {self.gport}"
                      f" · {self.rate_hz:.0f} Hz")
        else:
            self._log(f"从端夹爪：订阅 {self._topic} @ {self.gpeer}:{self.gport} "
                      f"· align={self.align} · watchdog={self.watchdog_ms:.0f} ms")
        return st

    # ────────────────────── 快照 ──────────────────────

    def _update(self, st: dict, running: bool) -> None:
        """把 SDK 的状态字典翻成界面要的 `GripSnapshot`。"""
        master = self.role == ROLE_MASTER
        # `teleop_status()` 在没有会话时只返回 `{"active": False, "mode": None}` ——
        # 那不是一份状态，别拿它把上一帧的读数清成 0。
        has_session = "topic" in st
        with self._lock:
            s = self._snap
            s.teleop_active = bool(self._want or running)
            if not has_session:
                return
            frames = int(st.get("frames", 0) or 0)
            age_ms = st.get("last_frame_age_ms")
            s.openness = float(st.get("openness", 0.0) or 0.0)
            s.position_mm = float(st.get("position_mm", 0.0) or 0.0)
            s.force_n = float(st.get("force_n", 0.0) or 0.0)
            s.loop_hz = float(st.get("loop_hz", 0.0) or 0.0)
            s.rejected = int(st.get("rejected", 0) or 0)
            s.send_failed = int(st.get("send_failed", 0) or 0)
            s.fault = str(st.get("fault", "") or "")
            s.stale = bool(st.get("stale", False))
            s.frame_age = None if age_ms is None else float(age_ms) / 1000.0
            s.matching = bool(st.get("matching")) if master else False
            s.frames_sent = frames if master else 0
            s.frames_received = 0 if master else frames
            s.mismatch = self._mismatch
        if s.stale and not self._prev_stale:
            self._watchdog_trips += 1
            self._log(f"⚠ watchdog 超时 {self.watchdog_ms:.0f} ms "
                      "—— **持位**（继续发帧：不掉力、不松开、不重连）")
        self._prev_stale = s.stale
        with self._lock:
            s.watchdog_trips = self._watchdog_trips
        if not master:
            self._check_mismatch(s.openness, s.position_mm)

    def _check_mismatch(self, openness: float, position_mm: float) -> None:
        """spec §7.3：从端能同时拿到 `openness` 与 `position_mm`，
        于是 `position_mm / openness ≈ 主端的 travel_mm`。差太多就告警。

        挡的是「主从装的是不同型号夹爪」—— 不报错、只是动作幅度悄悄不对。
        ⚠ 只在**没被 `clamp01` 饱和**的窗口里比（`openness→1` 会把主端**原始**
        `position_mm` 报出来 ⇒ 误报）；⚠ **只告警一次**。
        """
        if self._mismatch_checked or self._cfg is None:
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

    def _teardown(self) -> None:
        """收尾顺序（spec §8 rule 5）：交接持位 → 关 zenoh → 断 CAN（**不失能**）。

        `teleop_stop()` 内部就是交接持位（主端退出零重力、从端补发一帧当前位），
        SDK 的 `disconnect()` 也**不失能**（构造时 `disable_on_disconnect=False`）。
        """
        self._want = False
        g = self._grip
        if g is not None:
            try:
                g.teleop_stop()
            except Exception:                        # noqa: BLE001
                log.exception("stop 夹爪遥操失败")
            try:
                # ⚠ 构造时传了 disable_on_disconnect=False ⇒ 这一步**不失能**
                g.disconnect()
            except Exception:                        # noqa: BLE001
                log.exception("close 夹爪失败")
        with self._lock:
            self._snap.connected = False
            self._snap.teleop_active = False
        self._publish_state()
