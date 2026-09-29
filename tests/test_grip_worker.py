"""`GripWorker` —— 换算 / 前置 / 收尾 / 两端环（真 zenoh 回环，非 mock）。"""
import socket
import sys
import threading
import time
import types

import pytest

from liteteleop import grip_worker as gw
from liteteleop import link
from liteteleop.grip_worker import (ROLE_MASTER, ROLE_SLAVE, GripNotReady,
                                    GripWorker, check_ready,
                                    clamp_to_calibrated, mm_to_openness,
                                    openness_to_rad, travel_mm_of)
from tests.fake_grip import NORMAL, REVERSE, FakeCfg, FakeGrip


def _free_port() -> int:
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


# ════════════════════ §6.1 换算：close_sign 回归网 ════════════════════

@pytest.mark.parametrize("mount,cfg", [
    ("normal", FakeCfg(**NORMAL)),
    ("reverse", FakeCfg(**REVERSE)),
])
def test_openness_maps_onto_both_calibrated_limits(mount, cfg):
    """`openness=0` → 闭合限位；`openness=1` → 张开限位。**两种装法都必须成立**。

    ⚠ 反装那组在**漏掉 `close_sign`** 时必红 —— 那正是参考实现
    （`gripper_teleop.py:155-163`）的真实缺陷，这条就是它的回归网。
    """
    assert openness_to_rad(cfg, 0.0) == pytest.approx(cfg.pos_closed_rad)
    assert openness_to_rad(cfg, 1.0) == pytest.approx(cfg.pos_open_rad)


def test_travel_mm_matches_sdk_template():
    """正装模板 `1.605 rad × 74.8` ≈ 120.05 mm，与 `max_stroke_mm=120` 吻合。"""
    assert travel_mm_of(FakeCfg(**NORMAL)) == pytest.approx(120.054, abs=0.01)


def test_mm_to_openness_inverts_openness_to_rad():
    """读侧（SDK 的 mm）与写侧（我们的 rad）必须互逆。"""
    cfg = FakeCfg(**NORMAL)
    for o in (0.0, 0.2, 0.5, 0.8, 1.0):
        q = openness_to_rad(cfg, o)
        mm = (cfg.pos_closed_rad - q) * cfg.close_sign * cfg.rad_to_mm
        assert mm_to_openness(cfg, mm) == pytest.approx(o)


def test_clamp_to_calibrated_handles_both_mounts():
    n, r = FakeCfg(**NORMAL), FakeCfg(**REVERSE)
    assert clamp_to_calibrated(n, 99.0) == pytest.approx(n.pos_closed_rad)
    assert clamp_to_calibrated(n, -99.0) == pytest.approx(n.pos_open_rad)
    assert clamp_to_calibrated(r, 99.0) == pytest.approx(r.pos_open_rad)
    assert clamp_to_calibrated(r, -99.0) == pytest.approx(r.pos_closed_rad)


# ════════════════════ 构造期就拒退化参数（别留到线程里才崩）════════════════════

@pytest.mark.parametrize("kw,match", [
    (dict(rate_hz=0.0), "rate_hz"),
    (dict(rate_hz=-1.0), "rate_hz"),
    (dict(watchdog_ms=0.0), "watchdog_ms"),
    (dict(watchdog_ms=-5.0), "watchdog_ms"),
])
def test_degenerate_construction_params_are_rejected(kw, match):
    """⚠ `rate_hz <= 0` 会在环里 `1.0 / rate_hz` 抛 ZeroDivisionError；
    `watchdog_ms <= 0` 不崩但会让从端**永远**判 stale ⇒ 一动不动（静默无用）。

    判别力：去掉 `__init__` 里那两条 `ValueError` 时本用例必红。
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
    """⚠ 断言 `enable()` **一次都没被调用** —— 未标定的夹爪不该先上一次电再被拒。"""
    g = FakeGrip(calibrated=False)
    w = GripWorker(ROLE_MASTER, "can0", gripper_factory=lambda _c: g)
    w.start()
    w.stop(timeout=5.0)
    assert g.enable_calls == 0, "拒绝启动必须发生在 enable() 之前"
    assert "未标定" in w.snapshot().error


# ════════════════════ §8 rule 6：收尾不失能 ════════════════════

def test_factory_keeps_motor_enabled_on_disconnect(monkeypatch):
    """⚠⚠ `_default_gripper_factory` 必须显式传 `disable_on_disconnect=False`。

    `LiteGrip.__init__` 的默认值是 **True**（`gripper.py:205`）—— 不传的话
    `disconnect()` 会走 `self._can.disconnect(disable=True)` → `self.disable()`
    （`protocols/can_bus.py:92`）⇒ **掉力、松开**，让收尾那帧持位帧白做。
    **删掉这个 kwarg 时本用例必红。**

    ⚠ 断言打在**构造参数**上：本设计里 worker 是直接 `g.disconnect()` 的，
    行为在**构造时**就已决定（不是 litearm-device 那种 adapter 加锁层）。
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


def test_master_handoff_exits_zero_gravity():
    """主端停下时必须退零重力（那一步本身就是一帧按当前位的持位帧）。"""
    g = FakeGrip()
    w = GripWorker(ROLE_MASTER, "can0", gripper_factory=lambda _c: g)
    try:
        w.start()
        time.sleep(0.3)
        w.set_teleop(True)
        time.sleep(0.4)
    finally:
        w.stop(timeout=5.0)
    assert g.zero_gravity_exits >= 1
    assert g.disconnects >= 1, "收尾必须 disconnect（否则进程可能挂死）"


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


# ════════════════════ §7.3 主从标定一致性诊断 ════════════════════

def test_mismatch_warning_fires_on_different_travel():
    w = GripWorker(ROLE_SLAVE, "can1")
    w._cfg = FakeCfg(**NORMAL)                   # travel ≈ 120 mm
    w._check_mismatch(0.5, 30.0)                 # 主端 travel ≈ 60 mm
    assert "不一致" in w._mismatch


def test_mismatch_warns_once_only():
    w = GripWorker(ROLE_SLAVE, "can1")
    w._cfg = FakeCfg(**NORMAL)
    w._check_mismatch(0.5, 30.0)
    first = w._mismatch
    w._check_mismatch(0.5, 3.0)                  # 后来一致了也不撤回、不再叠加
    assert w._mismatch == first


def test_mismatch_window_excludes_clamped_openness():
    """⚠ `openness→1` 时比值会报出主端**原始** `position_mm`（大于其真实 travel）
    ⇒ 会误报「不一致」。饱和区必须排除。"""
    w = GripWorker(ROLE_SLAVE, "can1")
    w._cfg = FakeCfg(**NORMAL)
    w._check_mismatch(1.0, 400.0)
    assert not w._mismatch
    w._check_mismatch(0.05, 3.0)
    assert not w._mismatch


# ════════════════════ 失败必须可见（不能静默死掉）════════════════════

def test_exception_inside_the_loop_is_recorded():
    """⚠⚠ 环里抛异常时，`snapshot().error` **必须**有内容。

    否则界面只会显示「未启动」，用户看不到任何原因 —— 正是本仓哲学反对的
    「连上了但界面不动」。`error` 也是**错误恢复**的触发条件
    （`main_window._on_grip_state` 靠它丢掉 worker 让用户能重试）。

    判别力：把 `_run_teleop` 的 `except` 里那句 `self._snap.error = str(e)`
    删掉时本用例必红。
    """
    class _BadSend(FakeGrip):
        def send_mit_frame(self, *a, **k):
            raise RuntimeError("CAN 发送失败")

    g = _BadSend()
    w = GripWorker(ROLE_MASTER, "can0", gport=_free_port(), rate_hz=100.0,
                   gripper_factory=lambda _c: g)
    try:
        w.start()
        time.sleep(0.35)
        w.set_teleop(True)
        time.sleep(0.4)
        err = w.snapshot().error
        assert "CAN 发送失败" in err, f"环里的异常没被记下来，error={err!r}"
    finally:
        w.stop(timeout=5.0)


def test_uncalibrated_error_is_recorded_and_loop_never_started():
    """未标定 ⇒ 记 error，且**主端 Listener 都不该建**（`_run` 在建之前就抛了）。"""
    g = FakeGrip(calibrated=False)
    w = GripWorker(ROLE_MASTER, "can0", gport=_free_port(), rate_hz=100.0,
                   gripper_factory=lambda _c: g)
    try:
        w.start()
        time.sleep(0.35)
        assert "未标定" in w.snapshot().error
        assert w._pub is None, "拒绝启动时不该已经占住端口"
    finally:
        w.stop(timeout=5.0)


# ════════════════════ §8 rule 9：协议边界必须拒非有限值 ════════════════════

def _listen_and_publish(port, key):
    """起一个裸发布端（模拟"坏主端"）。"""
    return link.Listener(port, key)


def test_slave_rejects_non_finite_frames_and_holds():
    """⚠⚠ 一条 `openness=NaN` 的帧**必须**被拒，且**保持不动**。

    不拒的话实测后果（危险一侧且静默）：

        _clamp01(NaN)            = nan
        openness_to_rad(cfg,NaN) = nan
        clamp_to_calibrated(NaN) = pos_closed_rad   ← min(hi, NaN) 返回 hi

    ⇒ 一条 NaN 帧把从端命令到**全闭限位**，而 `error` 是空的。
    `±inf` 同样会被折成端点。

    判别力：去掉 `_slave_loop` 里那个 `math.isfinite` 分支时，本用例会因为
    `q` 变成全闭限位而红。
    """
    from liteteleop import link as _link
    from liteteleop import grip_wire as _gw

    port = _free_port()
    sg = FakeGrip(position_rad=-0.7)
    s = GripWorker(ROLE_SLAVE, "can1", grip_id="gA", gpeer="127.0.0.1", gport=port,
                   rate_hz=100.0, align=False, gripper_factory=lambda _c: sg)
    pub = _link.Listener(port, _gw.gripper_teleop_topic("gA"))
    try:
        s.start()
        time.sleep(0.3)
        s.set_teleop(True)
        time.sleep(0.3)
        sg.clear_sent()
        for bad in (float("nan"), float("inf"), float("-inf")):
            for _ in range(5):
                pub.put(_gw.encode_gripper_teleop(bad, bad, 0.0, time.monotonic()))
                time.sleep(0.01)
        time.sleep(0.3)
        qs = [f["q"] for f in sg.sent]
        assert qs, "从端必须在持续发帧（持位）"
        assert all(abs(q - (-0.7)) < 1e-9 for q in qs), \
            f"收到非有限值帧后**必须保持不动**（-0.7），实际 q={sorted(set(round(q, 4) for q in qs))}"
        assert s.snapshot().rejected >= 1, "被拒的帧要计数（不静默）"
        assert s.snapshot().frames_received == 0, "非有限值帧不算收到有效帧"
    finally:
        s.stop(timeout=5.0)
        pub.close()


def test_align_rejects_non_finite_first_frame_and_holds():
    """⚠⚠ **`align=True`（默认值！）时，非有限值的首帧也必须被跳过。**

    这是我前面几轮全漏掉的一条路径：对齐用的 `_wait_first_frame` 曾经直接把
    解出来的 tuple 返回给 `openness_to_rad` → `goto_rad`，而
    `_clamp01(NaN)` 是 NaN、SDK 的 `goto_rad` 会把 NaN **折成端点**
    ⇒ 夹爪被"吸"到全闭（正常装法 `hi == pos_closed_rad`），
    而且 `rejected` 同时在涨、`error` 为空 —— 环里的守卫拦不住这条路。

    实测证据（修前）：`goto_rad q=[nan]` 且此后 MIT 指令全是 `0.114`。

    判别力：去掉 `_wait_first_frame` 里的 `math.isfinite` 检查时本用例必红。
    ⚠ 我这里**故意用 `align=True`** —— 旧用例全用 `align=False`，所以从没走到这一格。
    """
    port = _free_port()
    sg = FakeGrip(position_rad=-0.7)
    s = GripWorker(ROLE_SLAVE, "can1", grip_id="gA", gpeer="127.0.0.1", gport=port,
                   rate_hz=100.0, align=True, gripper_factory=lambda _c: sg)
    pub = link.Listener(port, gw.gripper_teleop_topic("gA"))
    stop = threading.Event()

    def flood():
        while not stop.is_set():
            pub.put(gw.encode_gripper_teleop(float("nan"), float("nan"),
                                             0.0, time.monotonic()))
            time.sleep(0.005)

    th = threading.Thread(target=flood, daemon=True)
    try:
        s.start()
        time.sleep(0.3)
        th.start()
        s.set_teleop(True)
        time.sleep(1.2)
        assert sg.gotos == [], f"非有限值的首帧**不许**进 goto_rad，实际 {sg.gotos}"
        qs = [f["q"] for f in sg.sent]
        assert qs, "对齐等待期间也必须继续发持位帧（停发会掉力）"
        assert all(abs(q - (-0.7)) < 1e-9 for q in qs), \
            ("必须保持本机实测位置 -0.7，实际 "
             f"{sorted(set(round(q, 4) for q in qs))}")
        assert s.snapshot().rejected >= 1, "被跳过的帧要计数"
        assert s.snapshot().frames_received == 0, "非有限值帧不算收到有效帧"
    finally:
        stop.set()
        th.join(timeout=1.0)
        s.stop(timeout=5.0)
        pub.close()


def test_non_finite_force_or_position_field_is_also_rejected():
    """⚠ **四个字段都要判** —— `force_n` 现在也会显示在界面上。"""
    from liteteleop import grip_wire as _gw

    port = _free_port()
    sg = FakeGrip(position_rad=-0.7)
    s = GripWorker(ROLE_SLAVE, "can1", grip_id="gA", gpeer="127.0.0.1", gport=port,
                   rate_hz=100.0, align=False, gripper_factory=lambda _c: sg)
    pub = link.Listener(port, _gw.gripper_teleop_topic("gA"))
    try:
        s.start()
        time.sleep(0.3)
        s.set_teleop(True)
        time.sleep(0.3)
        sg.clear_sent()
        for _ in range(5):                    # openness 有限，但 force_n 是 NaN
            pub.put(_gw.encode_gripper_teleop(0.5, 60.0, float("nan"), time.monotonic()))
            time.sleep(0.01)
        time.sleep(0.25)
        assert s.snapshot().frames_received == 0, "force_n 非有限值的帧也要拒"
        assert s.snapshot().rejected >= 1
        assert all(abs(f["q"] - (-0.7)) < 1e-9 for f in sg.sent), "必须保持不动"
    finally:
        s.stop(timeout=5.0)
        pub.close()


# ════════════════════ §8 rule 10：SDK 的失败信号必须被消费 ════════════════════

def test_enable_failure_is_not_silent():
    """⚠⚠ `enable()` 失败**必须**报出来。

    真 SDK **不抛** —— 只返回一个 falsy 的 `EnableResult`
    （`EnableResult.__bool__` 就是 `self.ok`），此后 `send_mit_frame` 恒返回 `False`。
    不查的实测后果：`error=''`、`connected=True`、`frames_sent` 一直涨，
    而**电机根本没使能（夹爪是软的）** —— 界面还在显示"已发 N 帧 · 跟随中"。

    判别力：去掉 `_run` 里那句 `if not self._grip.enable(): raise` 时本用例必红。
    """
    g = FakeGrip(enable_ok=False)
    w = GripWorker(ROLE_MASTER, "can0", gport=_free_port(), rate_hz=100.0,
                   gripper_factory=lambda _c: g)
    try:
        w.start()
        time.sleep(0.4)
        snap = w.snapshot()
        assert "使能失败" in snap.error, f"enable 失败必须记进 error，实际 {snap.error!r}"
        assert g.enable_calls == 1
        assert snap.frames_sent == 0, "使能失败就不该进环发帧"
    finally:
        w.stop(timeout=5.0)


def test_send_failure_is_counted():
    """⚠ `send_mit_frame` 返回 `False` 时**要计数**（真 SDK 未使能时不抛）。"""
    g = FakeGrip(enable_ok=False)
    # 让 enable 成功但发送失败（模拟"使能了、CAN 却发不出去"）
    g.enable_ok = True
    orig = g.send_mit_frame
    g.send_mit_frame = lambda *a, **k: (orig(*a, **k), False)[1]
    w = GripWorker(ROLE_MASTER, "can0", gport=_free_port(), rate_hz=100.0,
                   gripper_factory=lambda _c: g)
    try:
        w.start()
        time.sleep(0.35)
        w.set_teleop(True)
        time.sleep(0.4)
        assert w.snapshot().send_failed >= 1, \
            f"send_mit_frame 返回 False 必须计数，实际 {w.snapshot().send_failed}"
    finally:
        w.stop(timeout=5.0)


def test_gripper_own_fault_code_is_surfaced():
    """⚠ 夹爪**自己**报的 `error_code` 要消费掉。

    spec §7.4 偏离 #2 说「不 poll 就永远看不到夹爪自己的故障」——
    可**只 poll 不看 `error_code` 等于没做**。SDK 语义：`1`=已使能、`0`=已失能、
    其它=真实故障（`litegrip/constants.py:73-95`）。

    判别力：去掉 `_note_grip_fault` 调用时本用例必红。
    """
    g = FakeGrip(fault_code=0xB)              # 0xB = MOS 过温
    w = GripWorker(ROLE_MASTER, "can0", gport=_free_port(), rate_hz=100.0,
                   gripper_factory=lambda _c: g)
    try:
        w.start()
        time.sleep(0.35)
        w.set_teleop(True)
        time.sleep(0.4)
        fault = w.snapshot().fault
        assert "11" in fault, f"error_code 要报出来，实际 {fault!r}"
        assert "过温" in fault, "要说明是什么故障"
    finally:
        w.stop(timeout=5.0)


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
    w = GripWorker(ROLE_MASTER, "can0", gport=_free_port(), rate_hz=100.0,
                   gripper_factory=lambda _c: g)
    try:
        w.start()
        time.sleep(0.35)
        w.stop(timeout=0.2)                   # 故意短于 disconnect 的 1.5 s
        assert w.is_alive() is True, "超时后线程仍在跑，必须能被问到"
        assert w._thread is not None, "超时时不许清掉线程引用"
        w.stop(timeout=5.0)                   # 再点一次能重试收尾
        assert w.is_alive() is False
    finally:
        w.stop(timeout=5.0)


def test_master_refuses_to_publish_non_finite():
    """主端读数坏掉时**不发帧** ⇒ 从端 watchdog 超时转持位（安全那一侧）。"""
    class _BadRead(FakeGrip):
        def get_state(self, wait: bool = True):
            from tests.fake_grip import FakeState
            return FakeState(position_rad=float("nan"), position_mm=float("nan"),
                             force_n=0.0)

    g = _BadRead()
    w = GripWorker(ROLE_MASTER, "can0", grip_id="gA", gport=_free_port(),
                   rate_hz=100.0, gripper_factory=lambda _c: g)
    try:
        w.start()
        time.sleep(0.35)
        w.set_teleop(True)
        time.sleep(0.4)
        snap = w.snapshot()
        assert snap.frames_sent == 0, f"读数非有限值就不该发帧，实际发了 {snap.frames_sent}"
        assert snap.rejected >= 1, "被拒的帧要计数（不静默）"
    finally:
        w.stop(timeout=5.0)


# ════════════════════ 两端环（真 zenoh 回环）════════════════════

def _pair(port: int, rate_hz: float = 100.0):
    mg, sg = FakeGrip(), FakeGrip()
    m = GripWorker(ROLE_MASTER, "can0", grip_id="gA", gport=port,
                   rate_hz=rate_hz, gripper_factory=lambda _c: mg)
    s = GripWorker(ROLE_SLAVE, "can1", grip_id="gA", gpeer="127.0.0.1", gport=port,
                   rate_hz=rate_hz, align=True, gripper_factory=lambda _c: sg)
    return m, s, mg, sg


def test_master_to_slave_follows():
    """主端手掰 → 从端跟到位（端到端，真 zenoh）。"""
    port = _free_port()
    m, s, mg, sg = _pair(port)
    try:
        m.start()
        time.sleep(0.3)
        m.set_teleop(True)
        time.sleep(0.5)
        s.start()
        time.sleep(0.3)
        s.set_teleop(True)
        time.sleep(1.5)
        assert s.snapshot().frames_received > 0, "从端一帧都没收到"

        mg.drag_to(openness_to_rad(mg.config, 0.5))
        deadline = time.monotonic() + 3.0
        want = openness_to_rad(sg.config, 0.5)
        while time.monotonic() < deadline:
            if sg.last_sent_q() is not None and abs(sg.last_sent_q() - want) < 0.05:
                break
            time.sleep(0.05)
        assert sg.last_sent_q() == pytest.approx(want, abs=0.05)
    finally:
        s.stop(timeout=5.0)
        m.stop(timeout=5.0)


def test_watchdog_holds_position_when_master_stops():
    """⚠ 主端停发 ⇒ 从端 **持位**：q 一动不动，但**继续发帧**（否则掉力）。"""
    port = _free_port()
    m, s, mg, sg = _pair(port)
    try:
        m.start()
        time.sleep(0.3)
        m.set_teleop(True)
        time.sleep(0.5)
        s.start()
        time.sleep(0.3)
        s.set_teleop(True)
        time.sleep(1.0)

        mg.drag_to(openness_to_rad(mg.config, 0.7))
        deadline = time.monotonic() + 3.0
        want = openness_to_rad(sg.config, 0.7)
        while time.monotonic() < deadline:
            if sg.last_sent_q() is not None and abs(sg.last_sent_q() - want) < 0.05:
                break
            time.sleep(0.05)
        held = sg.last_sent_q()
        assert held == pytest.approx(want, abs=0.05)

        m.set_teleop(False)                      # 主端停发
        time.sleep(0.4)                          # 等过 200 ms watchdog
        sg.clear_sent()
        time.sleep(0.5)

        assert s.snapshot().stale is True, "watchdog 应该已判 stale"
        assert s.snapshot().watchdog_trips >= 1
        assert len(sg.sent) > 0, "stale 之后仍必须继续发帧（MIT 模式停发会掉力）"
        assert all(f["q"] == pytest.approx(held, abs=1e-9) for f in sg.sent), \
            "持位：q 必须在整段 stale 期间一动不动"
        assert all(f["kp"] > 0 for f in sg.sent), "持位帧必须带刚度，不能是零力矩帧"
    finally:
        s.stop(timeout=5.0)
        m.stop(timeout=5.0)


def test_slave_without_first_frame_sends_measured_position_not_zero():
    """⚠⚠ 首帧没到时若把 `q` 初始化成 `0.0`，那是一个**真实位置指令**
    （正装 `0.0 rad` = 全开）⇒ 可能直冲机械限位。
    入口必须取**本机实测位置** —— 这条就是那个兜底的回归网。"""
    port = _free_port()                          # ⚠ 故意没有主端在监听
    sg = FakeGrip(position_rad=-0.7)
    s = GripWorker(ROLE_SLAVE, "can1", grip_id="gA", gpeer="127.0.0.1", gport=port,
                   rate_hz=100.0, align=False, gripper_factory=lambda _c: sg)
    try:
        s.start()
        time.sleep(0.3)
        s.set_teleop(True)
        time.sleep(0.6)
        assert len(sg.sent) > 0, "没收到帧也必须持续发帧"
        assert sg.sent[0]["q"] == pytest.approx(-0.7), \
            "入口必须取本机实测位置（-0.7），不能是 0.0"
    finally:
        s.stop(timeout=5.0)


def test_teleop_can_be_restarted_repeatedly():
    """⚠⚠ 「停止 → 再启动」必须还能用，而且要**每轮都通**。

    这里钉住两个实测过的真 bug（2026-09-28，都是本仓自己踩的）：

    1. 主端 `Listener` 若每轮遥操拆了重建：遥操停下后端口不释放 ⇒ 第二次启动
       必定 `Can not create a new TCP listener bound to ...: Address already in use`，
       而且端口要等**进程退出**才回来（表现：**只能启动一次**）。
    2. 就算补上关闭，**同一端口上拆了重建**会让订阅↔发布匹配**间歇性**建立不起来
       —— 实测 12 轮里 5 轮 `matching=False`（主端照发、从端一帧收不到）。

    判别力：把 `_run` 里那句常驻 `link.Listener` 挪回 `_master_loop` 时，
    本用例会在第 2 轮就红（报错或 `matching` 为假）。
    """
    port = _free_port()
    m, s, mg, sg = _pair(port, rate_hz=100.0)
    try:
        m.start()
        time.sleep(0.3)
        s.start()
        time.sleep(0.3)
        prev_m, prev_s = 0, 0
        for i in range(4):
            m.set_teleop(True)
            s.set_teleop(True)
            time.sleep(0.5)
            snap_m, snap_s = m.snapshot(), s.snapshot()
            assert not snap_m.error, f"第{i + 1}轮主端报错: {snap_m.error}"
            assert not snap_s.error, f"第{i + 1}轮从端报错: {snap_s.error}"
            assert snap_m.matching is True, f"第{i + 1}轮主端没有订阅者（匹配失败）"
            assert snap_m.frames_sent > prev_m, f"第{i + 1}轮主端没发出新帧"
            assert snap_s.frames_received > prev_s, f"第{i + 1}轮从端没收到新帧"
            prev_m, prev_s = snap_m.frames_sent, snap_s.frames_received
            m.set_teleop(False)
            s.set_teleop(False)
            time.sleep(0.3)
    finally:
        s.stop(timeout=5.0)
        m.stop(timeout=5.0)


def test_align_moves_slave_toward_master_first_frame():
    """对齐：先 `goto_rad` 到首帧位置，再进高频跟随。"""
    port = _free_port()
    m, s, mg, sg = _pair(port)
    try:
        m.start()
        time.sleep(0.3)
        mg.drag_to(openness_to_rad(mg.config, 0.8))
        m.set_teleop(True)
        time.sleep(0.6)
        s.start()
        time.sleep(0.3)
        s.set_teleop(True)
        s._want = True                            # 已经在 _run 里
        deadline = time.monotonic() + 4.0
        while time.monotonic() < deadline and not sg.gotos:
            time.sleep(0.05)
        assert sg.gotos, "对齐应当调用 goto_rad"
        assert sg.gotos[0]["q"] == pytest.approx(openness_to_rad(sg.config, 0.8), abs=0.05)
    finally:
        s.stop(timeout=5.0)
        m.stop(timeout=5.0)
