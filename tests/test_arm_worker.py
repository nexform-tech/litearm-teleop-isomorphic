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
    """⚠⚠ 从臂的顺序必须是 **对齐 → 写增益 → 跟随**。

    写增益会把 `mit_kp` 从出厂的 400 改成 25，而 **`movej` 用的就是 `mit_kp`** ——
    对齐那句 `movej` 若排在写增益**之后**，位置环软 16 倍、撑不住、到不了位，
    会撞 `move_timeout` 报「未到位, 超时 3.0s」（**真机踩过**）。

    （2026-09-28 真机验证：写完之后 kp=25 的臂**稳稳托住**，最大偏移 0.0004 rad，
      ⇒ "软而稳"就是 litearm-server 的手感，不是故障。之前我误判成"垂下去"。）

    判别力：把前两句对调，本用例会红。
    """
    order = []
    monkeypatch.setattr(servo, "align_to_master",
                        lambda *a, **k: (order.append("align"), [0.0] * 7)[1])
    monkeypatch.setattr(servo, "apply_joint_gains",
                        lambda *a, **k: (order.append("gains"), servo.JointGains())[1])
    monkeypatch.setattr(servo, "follow",
                        lambda *a, **k: (order.append("follow"), True)[1])
    monkeypatch.setattr(arm_worker.link, "Connector",
                        lambda *a, **k: _FakeEndpoint())
    monkeypatch.setattr(arm_worker, "read_safe_limits", lambda arm, **k: _FakeLimits())

    class _NoWriteArm(_ArmedForSlave):
        """**读**参数是允许的（要算 kd 预算），**写**不许。"""

        def __init__(self):
            self.writes = []
            self.jp = [type("P", (), {"kp": 400.0, "kd": 5.0, "tau_max": 78.0})() for _ in range(7)]

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
    assert order == ["align", "gains", "follow"], (
        f"顺序必须是 对齐 → 写增益 → 跟随，实际 {order}\n"
        "⛔ 写增益会改 `mit_kp`，而 `movej` 用的就是它 —— 对齐必须排在写增益之前。")



def test_align_move_timeout_is_thirty_seconds():
    """用户裁决 2026-09-28：**同步时间 3 s → 30 s**。

    ⚠ 它是 `Arm(move_timeout=)`，SDK 明说**连接后不许改** ⇒ 只能在构造时定。
    ⚠ 但 `movej` **到位就立刻返回**，所以这只是**上限**、不是固定等待 —— 到点就跟。
    """
    assert arm_worker.ALIGN_MOVE_TIMEOUT == 30.0
    assert ArmWorker(role=ROLE_SLAVE).move_timeout == 30.0
