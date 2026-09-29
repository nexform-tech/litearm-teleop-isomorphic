"""`ArmWorker` 的离线测试（假臂，不需要硬件）。

最重要的一条是 §7.4 的硬要求：**worker 卡在 `movej` 期间，旁路急停仍能发出**。
那条判据的价值在于：如果哪天有人把急停改成"投进 worker 队列"，它会**红**。
"""
from __future__ import annotations

import threading
import time

import pytest

from liteteleop import arm_worker, servo
from liteteleop.arm_worker import (
    ROLE_MASTER,
    ROLE_SLAVE,
    ArmWorker,
    Snapshot,
    StateHookMissing,
    attach_state_hook,
)


# ────────────────────────── 状态钩子（唯一私有耦合）──────────────────────────

class _FakeAck:
    def __init__(self):
        self.state = None

    def _on_status(self, payload: bytes) -> None:      # SDK 的钩子缝隙
        pass

    def feed(self, st) -> None:
        self.state = st
        self._on_status(b"<frame>")


class _FakeState:
    def __init__(self, q):
        self.q = list(q)
        self.dq = [0.0] * len(q)
        self.tau = [0.0] * len(q)
        self.joints = [type("J", (), {"t_mos": 30.0, "t_coil": 28.0, "err": 1})()
                       for _ in q]
        self.mode_name = "MOVE_J"
        self.enabled = True
        self.faulted = False
        self.joint_fault = 0
        self.flags = 0
        self.flag_names = []
        self.seq = 0


def test_state_hook_missing_raises():
    """SDK 若改了内部结构，**启动即报** —— 不要"连上了但界面不动"。"""
    class _NoSeam:
        pass

    with pytest.raises(StateHookMissing):
        attach_state_hook(_NoSeam(), lambda s: None)


def test_state_hook_forwards_every_frame():
    arm = type("A", (), {})()
    arm._a = _FakeAck()
    got = []
    attach_state_hook(arm, got.append)
    arm._a.feed(_FakeState([0.1] * 7))
    arm._a.feed(_FakeState([0.2] * 7))
    assert len(got) == 2 and got[-1].q[0] == 0.2


# ────────────────────────── 急停旁路（§7.4 硬要求）──────────────────────────

class _StuckArm:
    """`movej` 会长时间阻塞 —— 模拟 worker 卡在收尾 `movej` 里。"""

    def __init__(self):
        self.in_movej = threading.Event()
        self.estop_called = threading.Event()
        self.movej_block = 2.0

    def movej(self, q, speed=1.0):
        self.in_movej.set()
        time.sleep(self.movej_block)

    def emergency_stop(self):
        self.estop_called.set()


def test_estop_bypasses_a_worker_stuck_in_movej():
    """⛔ **急停不许排在 worker 队列后面**（那最长要等 `move_timeout`）。

    判别力：若有人把 `emergency_stop` 改成投进 `_cmds` 队列，本用例会**超时失败**。
    """
    arm = _StuckArm()
    w = ArmWorker(role=ROLE_MASTER)
    w._arm = arm
    threading.Thread(target=lambda: arm.movej([0.0] * 7), daemon=True).start()
    assert arm.in_movej.wait(1.0), "假臂没进 movej"

    t0 = time.monotonic()
    w.emergency_stop()                                  # ⛔ 旁路，不排队
    assert arm.estop_called.wait(0.5), "急停被卡在 movej 后面了 —— 旁路失效"
    assert time.monotonic() - t0 < 0.5


def test_estop_without_arm_reports_failure():
    w = ArmWorker(role=ROLE_MASTER)
    w.emergency_stop()
    assert w.emergency_outcome() == (False, "未连接")


def test_estop_outcome_captures_exception():
    class _Bad:
        def emergency_stop(self):
            raise RuntimeError("链路断了")

    w = ArmWorker(role=ROLE_MASTER)
    w._arm = _Bad()
    w.emergency_stop()
    for _ in range(50):
        if w.emergency_outcome() is not None:
            break
        time.sleep(0.01)
    ok, err = w.emergency_outcome()
    assert ok is False and "链路断了" in err


# ────────────────────────── 快照 ──────────────────────────

def test_snapshot_defaults_are_safe():
    """未连接时快照必须是一份**安全的空值**，不是 None。"""
    s = ArmWorker(role=ROLE_MASTER).snapshot()
    assert isinstance(s, Snapshot)
    assert s.connected is False and s.teleop_active is False
    assert s.q == [] and s.err == []


def test_snapshot_reflects_pushed_state():
    w = ArmWorker(role=ROLE_MASTER)
    w._push(_FakeState([0.5] * 7))
    s = w.snapshot()
    assert s.q == [0.5] * 7
    assert s.err == [1] * 7
    assert s.enabled is True


def test_push_does_not_require_on_state_callback():
    """`on_state=None` 也要能推（回调炸了不该影响 SDK 读线程）。"""
    w = ArmWorker(role=ROLE_MASTER, on_state=None)
    w._push(_FakeState([0.0] * 7))                      # 不该抛
    assert w.snapshot().mode_name == "MOVE_J"


def test_on_state_callback_exception_is_swallowed():
    """⚠ 回调在 **SDK 读线程**上跑 —— 它抛异常会污染 SDK 的读路径，必须吞掉。"""
    def boom(_s):
        raise ValueError("GUI 炸了")

    w = ArmWorker(role=ROLE_MASTER, on_state=boom)
    w._push(_FakeState([0.0] * 7))                      # 不该抛出去


# ────────────────────────── 角色校验 ──────────────────────────

def test_bad_role_rejected():
    with pytest.raises(ValueError, match="role"):
        ArmWorker(role="observer")


def test_teleop_topic_matches_server_convention():
    w = ArmWorker(role=ROLE_MASTER, arm_id="armB")
    assert w.key == "litearm/v4/armB/teleop"


# ────────────── 调用顺序（真机踩过：movej 用了被改软的 mit_kp）──────────────

class _FakeEndpoint:
    def close(self):
        pass


class _FakeLimits:
    lo = [-1.0] * 7
    hi = [1.0] * 7


class _ArmedForSlave:
    move_timeout = 3.0

    def get_state(self, refresh=False):
        return type("M", (), {"value": type("S", (), {"q": [0.0] * 7})()})()

    def movej(self, q, speed=1.0):
        pass


def test_slave_aligns_then_follows_and_writes_no_firmware_parameters(monkeypatch):
    """⚠⚠ 从臂的顺序是 **对齐 → 跟随**，而且**不写任何固件参数**（用户裁决 2026-09-28）。

    写 `mit_kp` 那条路已经**证伪**：静态托得住（0.0004 rad），但**动态跟随会位置越界
    断轴 J2/J4**（真机 24 s 后 `FAULT FB_STALE POS_VIOL`）。
    ⇒ 用固件出厂刚度，抖动靠 kd 预算收紧限速去压。

    判别力：谁要是让从臂去**写固件参数**（`set_joint_param` / `set_ff_vec`），本用例会红
    —— `_NoWriteArm` 只放行**读**，一写就抛（`save_params` 更是直接 AssertionError）。
    """
    order = []
    seen = {}
    monkeypatch.setattr(servo, "align_to_master",
                        lambda *a, **k: (order.append("align"), [0.0] * 7)[1])

    def _fake_follow(*a, **k):
        order.append("follow")
        seen.update(k)
        return True

    monkeypatch.setattr(servo, "follow", _fake_follow)
    monkeypatch.setattr(arm_worker.link, "Connector",
                        lambda *a, **k: _FakeEndpoint())
    monkeypatch.setattr(arm_worker, "read_safe_limits", lambda arm, **k: _FakeLimits())

    class _NoWriteArm(_ArmedForSlave):
        """**读**参数是允许的（要算 kd 预算），**写**不许。"""

        def __init__(self):
            self.writes = []
            self.jp = [type("P", (), {"kp": 400.0, "kd": 5.0, "tau_max": 78.0,
                                       "q_min": -1.5, "q_max": 1.5})()
                   for _ in range(7)]

        @property
        def params(self):
            return self

        def all_joint_params(self):
            return list(self.jp)

        def get_ff_vec(self, item):
            return type("M", (), {"value": [6.0] * 7})()

        def set_joint_param(self, idx, kp, kd, tau_max):
            self.writes.append(("kp/kd", kp, kd))

        def set_ff_vec(self, item, values):
            assert item == 15, "kd_extra 在 0x26 向量表 item 15"
            self.writes.append(("kd_extra", list(values)))

        def save_params(self):
            raise AssertionError("⛔ 绝不许调 save_params（整扇区擦写、不可逆）")

        def __getattr__(self, name):
            raise AttributeError(name)

    w = ArmWorker(role=ROLE_SLAVE)
    w._arm = _NoWriteArm()
    w._run_slave()
    assert order == ["align", "follow"], f"顺序必须是 对齐 → 跟随，实际 {order}"
    # ⛔ 速度上限必须**逐字是 litearm-server 的配置值**（用户裁决 2026-09-28，真机实测）。
    #   曾按 kd 预算收紧（`0.30·tau_max/kd_eff`）⇒ J3/J4 只剩 **11%**、腕部只剩 **9~15%**
    #   ⇒ 用户实测「跟随太慢，有明显的延迟」。
    #   判别力：谁把那个预算加回来，本用例会红。
    assert seen["speed_limit"] == list(servo.DEFAULT_SPEED_LIMIT), (
        f"speed_limit 必须是 server 原值 {servo.DEFAULT_SPEED_LIMIT}，"
        f"实际 {seen['speed_limit']}")



def test_align_move_timeout_is_thirty_seconds():
    """用户裁决 2026-09-28：**同步时间 3 s → 30 s**。

    ⚠ 它是 `Arm(move_timeout=)`，SDK 明说**连接后不许改** ⇒ 只能在构造时定。
    ⚠ 但 `movej` **到位就立刻返回**，所以这只是**上限**、不是固定等待 —— 到点就跟。
    """
    assert arm_worker.ALIGN_MOVE_TIMEOUT == 30.0
    assert ArmWorker(role=ROLE_SLAVE).move_timeout == 30.0


# ────────────── 臂维护动作：使能 / 清错 / 复位 / 回零 ──────────────


class _MaintArm:
    """记下 SDK 调用的假臂。`enabled` 决定 `go_home` 的预检走哪条。"""

    def __init__(self, enabled=True, boom=None):
        self.enabled = enabled
        self.boom = boom                # 非 None ⇒ 那个方法抛异常
        self.calls = []

    def _hit(self, name):
        self.calls.append(name)
        if self.boom == name:
            raise RuntimeError(f"{name} 炸了")

    def get_state(self, refresh=False):
        return type("M", (), {"value": type("S", (), {"enabled": self.enabled})()})()

    def enable(self):
        self._hit("enable")

    def clear_faults(self):
        self._hit("clear_faults")

    def reset(self):
        self._hit("reset")

    def home(self):
        self._hit("home")
        return type("S", (), {"q": [0.0] * N_JOINTS})()


def _drain_once(w):
    """跑掉队列里排着的那一条命令。"""
    w._drain(0.0)


def test_maintenance_commands_reach_the_arm():
    """四个维护动作都要**真的落到 SDK 上**（名字别接错）。"""
    for method, sdk in (("enable_arm", "enable"), ("clear_faults", "clear_faults"),
                        ("reset_arm", "reset"), ("go_home", "home")):
        w = ArmWorker(role=ROLE_MASTER)
        w._arm = _MaintArm()
        getattr(w, method)()
        _drain_once(w)
        assert w._arm.calls == [sdk], f"{method} 应当调用 arm.{sdk}，实际 {w._arm.calls}"


def test_go_home_refuses_when_the_arm_is_not_enabled():
    """⚠⚠ **未使能时不许闷头发 home** —— 固件会拒，而用户只会看到"点了没反应"。

    判别力：把那个预检删掉，本用例会红（`home()` 会被调用）。
    """
    w = ArmWorker(role=ROLE_MASTER)
    w._arm = _MaintArm(enabled=False)
    w.go_home()
    _drain_once(w)
    assert w._arm.calls == [], f"未使能就不该调 arm.home()，实际 {w._arm.calls}"
    assert "未使能" in w._snap.error, f"要把原因说出来，实际 {w._snap.error!r}"


def test_maintenance_failure_is_visible_not_swallowed():
    """失败要**记进快照**（界面看得见），不能只写一行日志。"""
    w = ArmWorker(role=ROLE_MASTER)
    w._arm = _MaintArm(boom="clear_faults")
    w.clear_faults()
    _drain_once(w)
    assert "清错失败" in w._snap.error and "炸了" in w._snap.error, w._snap.error


def test_a_successful_maintenance_clears_the_previous_error():
    """一次成功的手动操作 ⇒ 清掉旧错误，否则界面会一直挂着上一次的错。"""
    w = ArmWorker(role=ROLE_MASTER)
    w._arm = _MaintArm()
    with w._lock:
        w._snap.error = "上一次的旧错"
    w.reset_arm()
    _drain_once(w)
    assert w._snap.error == "", f"成功之后该清空，实际 {w._snap.error!r}"


def test_maintenance_on_a_missing_arm_says_so():
    """没连接时要说一声，不是静默 no-op。

    ⚠ 这里只**记日志**、**不写 `snap.error`** —— 与 `set_payload` 同一惯例：
    "没连接"是用户操作问题，不是臂的故障；写进快照会让顶栏的"故障"格亮红。
    （界面上这条路径其实到不了：`_relock` 没连接时就把四键禁掉了，这里是兜底。）
    """
    logs = []
    w = ArmWorker(role=ROLE_MASTER, on_log=logs.append)
    w._arm = None
    w.enable_arm()
    _drain_once(w)
    assert any("未连接" in x for x in logs), logs
    assert w._snap.error == "", "不是臂的故障，别污染快照里的 error"
