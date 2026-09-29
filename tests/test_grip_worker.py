"""`GripWorker` —— 构造护栏 / 前置 / 收尾 / **SDK 状态 → 界面快照** 的映射。

⚠⚠ 遥操本身（协议、环、watchdog、对齐、交接、非有限值拒收）已**下沉到 litegrip
SDK**（`LiteGrip.teleop_start/teleop_stop/teleop_status`）⇒ 本文件**只**测这层包装：
它把 SDK 的状态字典翻成界面要的 `GripSnapshot`。那些语义的用例在
`litegrip-python/tests/test_teleop.py`（本仓不再重复，也不再维护第二份实现）。

⚠⚠ CI **不装 litegrip**（`.github/workflows/ci.yml` 只装 `pytest` + `zenoh`）
⇒ 这里一律走 `FakeGrip` 替身，绝不 `import litegrip`。
"""
import sys
import time
import types

import pytest

from liteteleop import grip_worker as gw
from liteteleop.grip_worker import (ROLE_MASTER, ROLE_SLAVE, GripNotReady,
                                    GripWorker, check_ready, travel_mm_of)
from tests.fake_grip import NORMAL, REVERSE, FakeCfg, FakeGrip


# ════════════════════ 换算（本仓只留 travel_mm 一处）════════════════════

def test_travel_mm_matches_sdk_template():
    """正装模板 `1.605 rad × 74.8` ≈ 120.05 mm，与 `max_stroke_mm=120` 吻合。

    ⚠ 只留这一条换算：`openness↔rad` 的换算现在**只在 SDK 里**，本仓不该有副本。
    """
    assert travel_mm_of(FakeCfg(**NORMAL)) == pytest.approx(120.054, abs=0.01)


# ════════════════════ 构造期就拒退化参数（别留到线程里才崩）════════════════════

@pytest.mark.parametrize("kw,match", [
    (dict(rate_hz=0.0), "rate_hz"),
    (dict(rate_hz=-1.0), "rate_hz"),
    (dict(watchdog_ms=0.0), "watchdog_ms"),
    (dict(watchdog_ms=-5.0), "watchdog_ms"),
    (dict(poll_s=0.0), "poll_s"),
    (dict(poll_s=-1.0), "poll_s"),
])
def test_degenerate_construction_params_are_rejected(kw, match):
    """⚠ `rate_hz <= 0` / `watchdog_ms <= 0` 会被 SDK 的 `GripperTeleop` 拒掉；
    `watchdog_ms <= 0` 的语义是"从端**永远**判 stale ⇒ 一动不动"（能启动、但什么都
    不做的静默无用配置）。从线程里抛出来的表现只是"点了按钮没反应" ⇒ 构造时就拒。

    判别力：去掉 `__init__` 里那三条 `ValueError` 时本用例必红。
    """
    with pytest.raises(ValueError, match=match):
        GripWorker(ROLE_MASTER, "can0", **kw)


def test_bad_role_is_rejected():
    with pytest.raises(ValueError, match="role"):
        GripWorker("bogus", "can0")


# ════════════════════ §8.1 前置：拒启动 ════════════════════

def test_check_ready_rejects_uncalibrated():
    with pytest.raises(GripNotReady, match="未标定"):
        check_ready(FakeCfg(calibrated=False))


def test_check_ready_rejects_zero_travel():
    with pytest.raises(GripNotReady, match="行程为零"):
        check_ready(FakeCfg(pos_closed_rad=0.5, pos_open_rad=0.5))


def test_check_ready_rejects_zero_rad_to_mm():
    with pytest.raises(GripNotReady, match="rad_to_mm"):
        check_ready(FakeCfg(rad_to_mm=0.0))


def test_worker_refuses_before_enabling():
    """⚠ 断言 `enable()` **一次都没被调用** —— 未标定的夹爪不该先上一次电再被拒。

    这正是本仓**留着** `check_ready` 的唯一理由：SDK 的检查发生在 `teleop_start`
    里，而那时调用方已经 `enable()` 过了（`enable` 后 `teleop_start` 才拒）。
    我们要的是**先拒、后使能**。
    """
    g = FakeGrip(calibrated=False)
    w = GripWorker(ROLE_MASTER, "can0", gripper_factory=lambda _c: g)
    w.start()
    w.stop(timeout=5.0)
    assert g.enable_calls == 0, "拒绝启动必须发生在 enable() 之前"
    assert "未标定" in w.snapshot().error


# ════════════════════ §8 rule 6：收尾不失能 ════════════════════

def test_factory_keeps_motor_enabled_on_disconnect(monkeypatch):
    """⚠⚠ `_default_gripper_factory` 必须显式传 `disable_on_disconnect=False`。

    `LiteGrip.__init__` 的默认值是 **True**（`gripper.py:225`）—— 不传的话
    `disconnect()` 会走 `self._can.disconnect(disable=True)` → `self.disable()`
    （`protocols/can_bus.py`）⇒ **掉力、松开**，让收尾那帧持位帧白做。
    **删掉这个 kwarg 时本用例必红。**

    ⚠ 断言打在**构造参数**上：本设计里 worker 是直接 `g.disconnect()` 的，
    行为在**构造时**就已决定。
    """
    seen = {}

    class _Recorder:
        def __init__(self, **kw):
            seen.update(kw)

    mod = types.ModuleType("litegrip")
    mod.LiteGrip = _Recorder
    monkeypatch.setitem(sys.modules, "litegrip", mod)
    gw._default_gripper_factory("can0")
    assert seen.get("channel") == "can0"
    assert seen.get("disable_on_disconnect") is False


def test_teardown_stops_session_and_disconnects():
    """收尾顺序（spec §8 rule 5）：`teleop_stop()` **然后** `disconnect()`。

    `teleop_stop()` 让夹爪交接持位（主端退零重力 / 从端补一帧当前位），
    `disconnect()` 也**不失能**。⚠ `FakeGrip` **故意没有 `disable()`** ⇒
    任何"顺手失能"的路径会当场 `AttributeError`。
    """
    g = FakeGrip()
    w = GripWorker(ROLE_MASTER, "can0", poll_s=0.02, gripper_factory=lambda _c: g)
    w.start()
    time.sleep(0.15)
    w.set_teleop(True)
    time.sleep(0.2)
    w.stop(timeout=5.0)
    assert g.teleop_stops >= 1, "收尾必须 teleop_stop（交接持位）"
    assert g.disconnects >= 1, "收尾必须 disconnect（否则进程可能挂死）"
    assert w.snapshot().connected is False
    assert w.snapshot().teleop_active is False


# ════════════════════ §9.4 SDK 钉死断言 ════════════════════

def test_sdk_pin_accepts_the_target_and_rejects_the_other_copy():
    """⚠ 断言必须有 —— 只 `sys.path.insert` 压不住未来改成 insert 式注册的
    editable MetaPathFinder，那时 import 会**静默**落到 gitee 那份。

    本用例不需要真的装 litegrip（CI 上也没有）⇒ 直接喂路径判据。
    """
    ok = "/home/llx/litegrip-python/src/litegrip/__init__.py"
    bad = "/home/llx/moduangongju/lite-grip/litegrip/__init__.py"
    gw.assert_sdk_pinned(ok)                     # 不抛
    with pytest.raises(SystemExit, match="另一份抢先了"):
        gw.assert_sdk_pinned(bad)


def test_sdk_src_is_overridable_by_env(monkeypatch):
    """⚠ `LITEGRIP_SRC` 覆盖 SDK **位置** —— 与 `__main__.py` 的 `LITEARM_SRC` 同款。

    硬编码的是 `/home/llx/litegrip-python/src`，别的机器上那份不存在，而
    `assert_sdk_pinned` 又要求落点必须与它相符 ⇒ 不改代码就根本起不来。
    覆盖的只是位置，判据一字未变。

    判别力：把 `GRIP_SDK_SRC` 改回裸字符串常量时本用例必红。
    """
    import importlib
    monkeypatch.setenv("LITEGRIP_SRC", "/opt/litegrip/src")
    try:
        assert importlib.reload(gw).GRIP_SDK_SRC == "/opt/litegrip/src"
    finally:
        monkeypatch.delenv("LITEGRIP_SRC", raising=False)
        importlib.reload(gw)


# ════════════════════ §8 rule 10：失败必须可见（不能静默死掉）════════════════════

def test_enable_failure_is_not_silent():
    """⚠⚠ `enable()` 失败**必须**报出来。

    真 SDK **不抛** —— 只返回一个 falsy 的 `EnableResult`，此后 `send_mit_frame`
    恒返回 `False`。不查的实测后果：`error=''`、`connected=True`、
    `frames_sent` 一直涨，而**电机根本没使能（夹爪是软的）**。

    判别力：去掉 `_run` 里那句 `if not self._grip.enable(): raise` 时本用例必红。
    """
    g = FakeGrip(enable_ok=False)
    w = GripWorker(ROLE_MASTER, "can0", poll_s=0.02, gripper_factory=lambda _c: g)
    try:
        w.start()
        time.sleep(0.25)
        snap = w.snapshot()
        assert "使能失败" in snap.error, f"enable 失败必须记进 error，实际 {snap.error!r}"
        assert g.enable_calls == 1
        assert g.teleop_starts == [], "使能失败就不该去 teleop_start"
    finally:
        w.stop(timeout=5.0)


def test_teleop_start_failure_is_recorded():
    """`teleop_start` 抛（`TeleopNotReady` / `TeleopBusyError` / …）⇒ 记 error。

    `error` 也是**错误恢复**的触发条件（`main_window._on_grip_state` 靠它丢掉
    worker 让用户能重试）⇒ 不能吞。
    """
    g = FakeGrip()
    g.raise_on_teleop_start = RuntimeError("teleop is already running")
    w = GripWorker(ROLE_MASTER, "can0", poll_s=0.02, gripper_factory=lambda _c: g)
    try:
        w.start()
        time.sleep(0.15)
        w.set_teleop(True)
        time.sleep(0.25)
        assert "already running" in w.snapshot().error, \
            f"teleop_start 的异常必须被记下来，实际 {w.snapshot().error!r}"
    finally:
        w.stop(timeout=5.0)


# ════════════════════ SDK 状态 → GripSnapshot 的映射 ════════════════════

def test_master_status_maps_frames_to_sent_and_matching():
    """主端：`frames` → `frames_sent`（不是 received），`matching` 透传。"""
    g = FakeGrip()
    g.teleop_status_extra = {"openness": 0.4, "position_mm": 42.0, "force_n": 3.5,
                             "matching": True, "loop_hz": 48.0}
    w = GripWorker(ROLE_MASTER, "can0", poll_s=0.02, gripper_factory=lambda _c: g)
    try:
        w.start()
        time.sleep(0.15)
        w.set_teleop(True)
        time.sleep(0.25)
        s = w.snapshot()
        assert s.teleop_active is True
        assert s.frames_sent > 0, "主端要把 frames 记成 sent"
        assert s.frames_received == 0
        assert s.matching is True
        assert s.openness == pytest.approx(0.4)
        assert s.position_mm == pytest.approx(42.0)
        assert s.force_n == pytest.approx(3.5)
        assert s.loop_hz == pytest.approx(48.0)
    finally:
        w.stop(timeout=5.0)


def test_slave_status_maps_frames_to_received_and_watchdog():
    """从端：`frames` → `frames_received`；`stale` 的**假→真跳变**要计一次
    `watchdog_trips`（不是每拍都加）；`frame_age` 由 `last_frame_age_ms` 换算。
    """
    g = FakeGrip()
    g.teleop_status_extra = {"openness": 0.6, "position_mm": 60.0, "force_n": 1.0,
                             "stale": True, "fault": "fault 11: MOS 过温",
                             "rejected": 3, "send_failed": 2,
                             "last_frame_age_ms": 250.0}
    w = GripWorker(ROLE_SLAVE, "can1", gpeer="127.0.0.1", poll_s=0.02,
                   gripper_factory=lambda _c: g)
    try:
        w.start()
        time.sleep(0.15)
        w.set_teleop(True)
        time.sleep(0.35)
        s = w.snapshot()
        assert s.frames_received > 0, "从端要把 frames 记成 received"
        assert s.frames_sent == 0
        assert s.stale is True
        assert s.watchdog_trips == 1, "stale 一直为真也只算一次跳变"
        assert s.frame_age == pytest.approx(0.25)
        assert s.rejected == 3
        assert s.send_failed == 2
        assert "过温" in s.fault
        assert s.matching is False, "matching 是主端字段"
    finally:
        w.stop(timeout=5.0)


def test_slave_mismatch_warning_from_status():
    """§7.3：从端从 `position_mm / openness` 反推主端 travel ≈ 60 mm，
    与本端 120 mm 差太多 ⇒ 告警一次（挡"主从装了不同型号夹爪"）。
    """
    g = FakeGrip()
    g.teleop_status_extra = {"openness": 0.5, "position_mm": 30.0}
    w = GripWorker(ROLE_SLAVE, "can1", gpeer="127.0.0.1", poll_s=0.02,
                   gripper_factory=lambda _c: g)
    try:
        w.start()
        time.sleep(0.15)
        w.set_teleop(True)
        time.sleep(0.25)
        assert "不一致" in w.snapshot().mismatch
    finally:
        w.stop(timeout=5.0)


def test_status_without_session_does_not_zero_the_readout():
    """⚠ 会话没起来时 `teleop_status()` 只有 `{"active": False, "mode": None}`
    （**没有 `topic`**）—— 那不是一份状态，别拿它把上一帧的读数清成 0。

    判别力：把 `_update` 里的 `has_session` 判据去掉（无条件覆盖）时，
    本用例会因为读数被清零而红。
    """
    g = FakeGrip()
    g.teleop_status_extra = {"openness": 0.7, "position_mm": 70.0}
    w = GripWorker(ROLE_SLAVE, "can1", gpeer="127.0.0.1", poll_s=0.02,
                   gripper_factory=lambda _c: g)
    try:
        w.start()
        time.sleep(0.15)
        w.set_teleop(True)
        time.sleep(0.25)
        assert w.snapshot().openness == pytest.approx(0.7)
        w.set_teleop(False)                       # 停会话 ⇒ 状态回落到无会话形态
        time.sleep(0.25)
        assert w.snapshot().openness == pytest.approx(0.7), \
            "无会话的状态不是一份读数，不该把上一帧清成 0"
    finally:
        w.stop(timeout=5.0)


# ════════════════════ 会话自行结束时**不重启风暴** ════════════════════

def test_self_ended_session_does_not_restart_storm():
    """⚠⚠ SDK 的会话自己结束（CAN 出错 / 环退出）时 `_want` 还是 True。

    若判序是"想启动 且 没在跑 ⇒ 启动"，就**每一拍都重启一次**（重启风暴）。
    本仓明确收手：清 `_want`、让界面弹回「启动」，由用户决定要不要再来一次。

    判别力：把 `_poll_loop` 里"会话自行结束"那条移到 `_want and not running`
    **之后**（即原顺序）时，本用例会因为 `teleop_starts` 一路涨而红。
    """
    g = FakeGrip()
    w = GripWorker(ROLE_MASTER, "can0", poll_s=0.02, gripper_factory=lambda _c: g)
    try:
        w.start()
        time.sleep(0.15)
        w.set_teleop(True)
        time.sleep(0.2)
        assert len(g.teleop_starts) == 1
        g.teleop_running = False                  # 会话"自己"结束
        time.sleep(0.3)
        assert len(g.teleop_starts) == 1, "会话自行结束不许被反复重启"
        assert w._want is False, "要收手（清 _want），否则下一拍又重启"
        assert w.snapshot().teleop_active is False
    finally:
        w.stop(timeout=5.0)


# ════════════════════ 传给 SDK 的实参 ════════════════════

@pytest.mark.parametrize("role,gcan,gpeer", [
    (ROLE_MASTER, "can0", "127.0.0.1"),
    (ROLE_SLAVE, "can1", "10.0.0.5"),
])
def test_teleop_start_receives_the_right_arguments(role, gcan, gpeer):
    """⚠ 模式、`link`、`host`、端口、`grip_id`、`watchdog_s`、`rate_hz` 都要对。

    `host` 的规矩：**主端只监听 ⇒ 传 None**；从端必须传对端地址。
    `watchdog_s` 是**秒**，本仓的 `watchdog_ms` 要除以 1000（传错就是量纲 bug）。
    """
    g = FakeGrip()
    w = GripWorker(role, gcan, grip_id="gB", gpeer=gpeer, gport=17449,
                   rate_hz=100.0, watchdog_ms=250.0, align=False,
                   poll_s=0.02, gripper_factory=lambda _c: g)
    try:
        w.start()
        time.sleep(0.15)
        w.set_teleop(True)
        time.sleep(0.2)
        assert g.teleop_starts, "应当已经 teleop_start"
        kw = g.teleop_starts[0]
        assert kw["mode"] == role
        assert kw["link"] == "zenoh"
        assert kw["host"] == (None if role == ROLE_MASTER else gpeer)
        assert kw["port"] == 17449
        assert kw["grip_id"] == "gB"
        assert kw["align"] is False
        assert kw["watchdog_s"] == pytest.approx(0.25)
        assert kw["rate_hz"] == pytest.approx(100.0)
    finally:
        w.stop(timeout=5.0)


# ════════════════════ 收尾超时：不许清掉线程引用 ════════════════════

def test_stop_timeout_keeps_the_thread_reference():
    """⚠ 收尾超时时**不能**清掉 `self._thread`。

    清了就没人知道那条线程还活着 ⇒ 调用方会再建一个 worker，
    两条线程同时抢**同一个 CAN** 与**同一个 zenoh 端口**。
    保留引用 ⇒ 再点一次「停止」能重试 join，`is_alive()` 也能被问到。

    判别力：把 `stop()` 里那句超时 `return` 去掉（即无论超时都清 `_thread`）时必红。
    """
    class _SlowDisconnect(FakeGrip):
        def disconnect(self):
            time.sleep(1.5)
            super().disconnect()

    g = _SlowDisconnect()
    w = GripWorker(ROLE_MASTER, "can0", poll_s=0.02, gripper_factory=lambda _c: g)
    try:
        w.start()
        time.sleep(0.2)
        w.stop(timeout=0.2)                   # 故意短于 disconnect 的 1.5 s
        assert w.is_alive() is True, "超时后线程仍在跑，必须能被问到"
        assert w._thread is not None, "超时时不许清掉线程引用"
        w.stop(timeout=5.0)                   # 再点一次能重试收尾
        assert w.is_alive() is False
    finally:
        w.stop(timeout=5.0)
