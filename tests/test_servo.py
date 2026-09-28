"""从臂伺服环的离线测试（用假臂，不需要硬件）。

覆盖三件真机上不好反复验的事：
  1. 参数写入/还原**逐字**正确（`K`/`B`/`kd_extra`），且异常路径也会还原
  2. 参考生成确实经过 `slew_target`（速度/加速度受限、且**永不发 NaN**）
  3. 固件连续拒帧会**升级**（不是无限吞掉 —— 拒帧 = 没 kick 看门狗 = 会下垂）
"""
from __future__ import annotations

import math

import pytest

from liteteleop import servo
from liteteleop.wire import N_JOINTS


class _JP:
    """`JointParam` 的最小替身。"""

    def __init__(self, kp, kd, tau_max, q_min=-2.8, q_max=2.8):
        self.kp, self.kd, self.tau_max, self.q_min, self.q_max = kp, kd, tau_max, q_min, q_max


class _Msg:
    def __init__(self, value):
        self.value = value


class _State:
    def __init__(self, q):
        self.q = list(q)


class FakeArm:
    """够 `servo` 用的假臂。记录每一次 `move_js` 的入参。"""

    def __init__(self, reject_at=(), q=None):
        """`reject_at`：**第几次 `move_js` 调用**被拒（1 基）。空 = 全接受。"""
        self.q = list(q or [0.0] * N_JOINTS)
        # 出厂值刻意与 servo 的默认 K/B 不同，才能测出"真的改了"
        self.jp = [_JP(kp=400.0, kd=5.0, tau_max=78.0) for _ in range(N_JOINTS)]
        self.kd_extra = [6.0, 6.0, 6.0, 6.0, 0.0, 0.0, 0.0]
        self.move_js_calls: list = []
        self.move_timeout = 3.0
        self.movej_calls: list = []
        self.reject_at = set(reject_at)
        self._sent = 0

    # ---- servo 用到的接口 ----
    def get_state(self, refresh=False):
        return _Msg(_State(self.q))

    def move_js(self, q, dq=None):
        self._sent += 1
        if self._sent in self.reject_at:
            raise RuntimeError("被固件拒绝: ERR [03,2]")
        self.move_js_calls.append((list(q), list(dq)))

    def movej(self, q, speed=1.0):
        self.movej_calls.append(list(q))
        self.q = list(q)

    @property
    def params(self):
        return self

    def all_joint_params(self):
        return list(self.jp)

    def set_joint_param(self, idx, kp, kd, tau_max):
        self.jp[idx] = _JP(kp, kd, tau_max)

    def get_ff_vec(self, item):
        assert item == 15, "kd_extra 在 0x26 向量表的 item 15"
        return _Msg(list(self.kd_extra))

    def set_ff_vec(self, item, values):
        assert item == 15
        self.kd_extra = list(values)


# ────────────────────────── 参数写入 / 还原 ──────────────────────────

def test_apply_writes_kb_and_zeroes_kd_extra():
    arm = FakeArm()
    servo.apply_joint_gains(arm, servo.DEFAULT_K, servo.DEFAULT_B)
    assert [p.kp for p in arm.jp] == servo.DEFAULT_K
    assert [p.kd for p in arm.jp] == servo.DEFAULT_B
    assert arm.kd_extra == [0.0] * N_JOINTS, (
        "⛔ 必须清零 kd_extra：move_js 的有效阻尼 = mit_kd + kd_extra，"
        "J1~J4 不清零就是 0.5+6.0=6.5（13 倍），kd·dq 会吃掉整个力矩预算")


def test_apply_keeps_tau_max_untouched():
    arm = FakeArm()
    before = [p.tau_max for p in arm.jp]
    servo.apply_joint_gains(arm, servo.DEFAULT_K, servo.DEFAULT_B)
    assert [p.tau_max for p in arm.jp] == before


def test_restore_is_verbatim():
    arm = FakeArm()
    kp0 = [p.kp for p in arm.jp]
    kd0 = [p.kd for p in arm.jp]
    extra0 = list(arm.kd_extra)

    saved = servo.apply_joint_gains(arm, servo.DEFAULT_K, servo.DEFAULT_B)
    assert [p.kp for p in arm.jp] != kp0, "改之前得**真的**改了"

    servo.restore_joint_gains(arm, saved)
    assert [p.kp for p in arm.jp] == kp0
    assert [p.kd for p in arm.jp] == kd0
    assert arm.kd_extra == extra0


def test_restore_is_idempotent():
    arm = FakeArm()
    kp0 = [p.kp for p in arm.jp]
    saved = servo.apply_joint_gains(arm, servo.DEFAULT_K, servo.DEFAULT_B)
    servo.restore_joint_gains(arm, saved)
    servo.restore_joint_gains(arm, saved)          # 再来一次不该炸、也不该改坏
    assert [p.kp for p in arm.jp] == kp0


# ────────────────────────── 参考生成 ──────────────────────────

def test_follow_slew_respects_speed_limit():
    """逐拍位移不得超过 `speed_limit·dt` —— 这是 `slew_target` 存在的理由。"""
    arm = FakeArm()
    far = [1.0] * N_JOINTS                      # 一步之遥的目标
    calls = []

    def provider():
        return far

    # 只跑一小段就停，够看前几拍
    ticks = {"n": 0}

    def stop():
        ticks["n"] += 1
        return ticks["n"] > 30

    servo.follow(arm, provider, should_stop=stop, engage_sec=0.0,
                 hz=100.0, gains=servo.JointGains())

    dt = 1.0 / 100.0
    prev = None
    for q, dq in arm.move_js_calls:
        if prev is not None:
            step = max(abs(a - b) for a, b in zip(q, prev))
            assert step <= max(servo.DEFAULT_SPEED_LIMIT) * dt * 1.5, (
                f"单拍位移 {step} 过大 —— 参考没经过 slew_target？")
        prev = q


def test_follow_never_sends_non_finite():
    arm = FakeArm()

    def stop():
        return len(arm.move_js_calls) > 20

    servo.follow(arm, lambda: [0.5] * N_JOINTS, should_stop=stop,
                 engage_sec=0.0, hz=100.0, gains=servo.JointGains())
    for q, dq in arm.move_js_calls:
        assert all(math.isfinite(v) for v in q)
        assert all(math.isfinite(v) for v in dq)


def test_follow_holds_when_provider_returns_none():
    """`target_provider()` 返回 None ⇒ **保持上一拍**（照搬 joint_follow 的那一支）。"""
    arm = FakeArm()
    calls = {"n": 0}

    def stop():
        calls["n"] += 1
        return calls["n"] > 15

    servo.follow(arm, lambda: None, should_stop=stop,
                 engage_sec=0.0, hz=100.0, gains=servo.JointGains())
    # 目标恒为初始实测位姿 ⇒ 不该走动
    for q, _dq in arm.move_js_calls[1:]:
        assert max(abs(v) for v in q) < 1e-6, "返回 None 时不该往别处走"


# ────────────────────────── 拒帧升级 ──────────────────────────

def test_follow_escalates_after_consecutive_rejects():
    """连续被拒必须**炸出来**：拒帧 = 没 kick 看门狗 = 0.1s 后 fail-soft 下垂。

    ⚠ 判据要有**判别力**：不仅"抛了异常"，还要断言**走了受控接管那条路**
    （`movej_calls` 非空）—— 否则 prime 阶段随便抛个异常也能让本条变绿。
    """
    # prime 用掉第 1 次调用；从第 2 次起全拒 ⇒ 必然走到 _REJECT_ESCALATE
    arm = FakeArm(reject_at=range(2, 10_000))
    with pytest.raises(RuntimeError, match="拒绝"):
        servo.follow(arm, lambda: [0.1] * N_JOINTS, should_stop=lambda: False,
                     engage_sec=0.0, hz=100.0, gains=servo.JointGains())
    assert arm.movej_calls, "升级前必须**受控接管**（movej），不能只抛异常"
    assert len(arm.move_js_calls) == 1, \
        "只有 prime 那一次该成功（第 1 次调用），主循环里的全部被拒"


def test_consecutive_reject_count_is_what_triggers_it():
    """把连续拒帧控制在阈值**以下** ⇒ 不该升级（证明阈值真的在起作用）。"""
    n = servo._REJECT_ESCALATE - 1
    arm = FakeArm(reject_at=range(2, 2 + n))
    calls = {"i": 0}

    def stop():
        calls["i"] += 1
        return calls["i"] > n + 5

    ok = servo.follow(arm, lambda: [0.1] * N_JOINTS, should_stop=stop,
                      engage_sec=0.0, hz=100.0, gains=servo.JointGains())
    assert ok is True, f"只拒了 {n} 次（阈值 {servo._REJECT_ESCALATE}）不该升级"


def test_follow_tolerates_occasional_reject():
    """零星被拒**不该**中断跟随（下一拍就恢复）。"""
    arm = FakeArm(reject_at=(2, 3, 4))            # prime 之后的头三拍被拒
    calls = {"n": 0}

    def stop():
        calls["n"] += 1
        return calls["n"] > 25

    ok = servo.follow(arm, lambda: [0.2] * N_JOINTS, should_stop=stop,
                      engage_sec=0.0, hz=100.0, gains=servo.JointGains())
    assert ok is True
    assert arm.move_js_calls, "被拒几次之后应该恢复正常下发"


# ────────────────────────── 收尾 ──────────────────────────

def test_follow_applies_and_restores_gains_when_it_owns_them():
    """`gains=None` ⇒ 自己改、自己还原 —— **两头都要检**（中途必须是改过的）。"""
    arm = FakeArm()
    kp0 = [p.kp for p in arm.jp]
    extra0 = list(arm.kd_extra)
    seen = {}
    n = {"i": 0}

    def stop():
        n["i"] += 1
        if n["i"] == 5:
            seen["kp"] = [p.kp for p in arm.jp]
            seen["extra"] = list(arm.kd_extra)
        return n["i"] > 20

    servo.follow(arm, lambda: None, should_stop=stop, engage_sec=0.0,
                 hz=100.0, gains=None)
    assert seen["kp"] == servo.DEFAULT_K, "跟随期间必须已写入 K"
    assert seen["extra"] == [0.0] * N_JOINTS, "跟随期间 kd_extra 必须已清零"
    assert [p.kp for p in arm.jp] == kp0, "退出必须还原 kp"
    assert arm.kd_extra == extra0, "退出必须还原 kd_extra"


def test_follow_restores_gains_even_when_provider_provokes_hold():
    """`target_provider` 抛异常 ⇒ 受控接管 + 返回 False，**且参数仍要还原**。"""
    arm = FakeArm()
    kp0 = [p.kp for p in arm.jp]

    def boom():
        raise ValueError("模拟上游炸了")

    ok = servo.follow(arm, boom, should_stop=lambda: False, engage_sec=0.0,
                      hz=100.0, gains=None)
    assert ok is False
    assert arm.movej_calls, "必须先受控接管（movej），绝不 disable"
    assert [p.kp for p in arm.jp] == kp0, "异常路径也必须还原参数"


def test_hold_at_current_uses_measured_pose():
    arm = FakeArm(q=[0.3] * N_JOINTS)
    servo.hold_at_current(arm)
    assert arm.movej_calls[-1] == [0.3] * N_JOINTS


# ────────────────────────── 对齐（照搬 _do_align）──────────────────────────

def _limits():
    from liteteleop.safety import read_limits_ok
    return read_limits_ok([-1.0] * N_JOINTS, [1.0] * N_JOINTS, N_JOINTS)


def test_align_skips_when_no_frame_arrives():
    """等不到主臂帧 ⇒ **跳过对齐**（不是卡死、也不是拿垃圾去 movej）。"""
    arm = FakeArm()
    got = servo.align_to_master(arm, lambda: (None, None), _limits(), timeout=0.05)
    assert got is None
    assert arm.movej_calls == [], "没帧就不该动臂"


def test_align_clips_to_limits_and_uses_align_speed():
    """超限的帧要**钳位**后再 movej，而且速度必须是 `align_speed`（慢）。"""
    arm = FakeArm(q=[0.0] * N_JOINTS)
    payload = __import__("liteteleop.wire", fromlist=["x"]).encode_teleop(
        [5.0] + [0.2] * (N_JOINTS - 1), [0.0] * N_JOINTS, 0.0)
    got = servo.align_to_master(arm, lambda: (payload, 0.0), _limits(), timeout=0.5)
    assert got is not None
    assert got[0] == 1.0, "超出上界的轴必须被钳到上界"
    assert arm.movej_calls, "应该调了 movej"
    assert abs(got[1] - 0.2) < 1e-9, "没超限的轴一个数都不许动"


def test_align_failure_is_not_fatal():
    """`movej` 失败**不致命** —— 原版就是"跟随会逐步修正"。"""
    class _MovejFails(FakeArm):
        def movej(self, q, speed=1.0):
            raise RuntimeError("movej 超时")

    arm = _MovejFails()
    payload = __import__("liteteleop.wire", fromlist=["x"]).encode_teleop(
        [0.1] * N_JOINTS, [0.0] * N_JOINTS, 0.0)
    assert servo.align_to_master(arm, lambda: (payload, 0.0), _limits(),
                                 timeout=0.5) is None


def test_align_warns_loudly_on_a_large_slew(caplog):
    """⚠ 大位移必须**大声预警**：`movej(speed=0.15)` 走大位移会撞上 move_timeout。"""
    import logging
    arm = FakeArm(q=[0.0] * N_JOINTS)
    payload = __import__("liteteleop.wire", fromlist=["x"]).encode_teleop(
        [0.9] * N_JOINTS, [0.0] * N_JOINTS, 0.0)          # 位移 0.9 > 阈值 0.30
    with caplog.at_level(logging.WARNING, logger="liteteleop.servo"):
        servo.align_to_master(arm, lambda: (payload, 0.0), _limits(), timeout=0.5)
    assert any("过大" in r.message for r in caplog.records), "大位移没预警"


# ────────────── 固件「目标≠实测 + dq=0 就拒」的退路（真机踩过）──────────────

class RuleArm(FakeArm):
    """模拟固件那条规则：**目标 ≠ 实测位姿 且 dq 全 0 ⇒ 拒帧**。

    ⚠ 实测残差用 `movej` 的到位判据 `q_tol=0.03` 那个量级 —— 真机就是死在这上面：
    从臂到位后 `slew_target` 给 `dq=0`，而实测与目标差 0.03 ⇒ **每拍都被拒**
    ⇒ 不 kick 看门狗 ⇒ 0.1 s 后 fail-soft ⇒ 臂会垂。
    """

    TOL = 0.005

    def move_js(self, q, dq=None):
        dq = list(dq) if dq is not None else [0.0] * N_JOINTS
        gap = max(abs(a - b) for a, b in zip(q, self.q))
        if gap > self.TOL and max(abs(v) for v in dq) == 0.0:
            self._sent += 1
            raise RuntimeError("被固件拒绝: ERR [03,2]")
        super().move_js(q, dq)


def test_follow_falls_back_to_holding_when_target_is_unreachable():
    """目标差 0.03（`q_tol` 量级）且 dq 收敛到 0 ⇒ **退路必须接管，不许升级退出**。

    判别力：去掉那条退路，本用例会因为「连续 20 次被拒」而抛 RuntimeError。
    """
    arm = RuleArm(q=[0.0] * N_JOINTS)
    n = {"i": 0}

    def stop():
        n["i"] += 1
        return n["i"] > 120                      # 够 slew_target 收敛

    ok = servo.follow(arm, lambda: [0.03] * N_JOINTS, should_stop=stop,
                      engage_sec=0.0, hz=200.0, gains=servo.JointGains())
    assert ok is True, "应该靠退路走完，而不是升级退出"
    assert arm.move_js_calls, "退路那一帧必须真的发出去"


def test_follow_resumes_normal_tracking_after_a_fallback():
    """退路之后，目标一动就要**恢复**（不是永远托在原地）。"""
    arm = RuleArm(q=[0.0] * N_JOINTS)
    targets = [[0.03] * N_JOINTS] * 60 + [[0.5] * N_JOINTS] * 40
    n = {"i": 0}

    def provider():
        i = min(n["i"], len(targets) - 1)
        return targets[i]

    def stop():
        n["i"] += 1
        return n["i"] > len(targets)

    servo.follow(arm, provider, should_stop=stop, engage_sec=0.0, hz=200.0,
                 gains=servo.JointGains())
    moved = max(max(abs(v) for v in q) for q, _dq in arm.move_js_calls)
    assert moved > 0.3, f"目标改到 0.5 之后应当恢复跟随，实际最大只到 {moved}"
