"""从臂伺服环的离线测试（用假臂，不需要硬件）。

覆盖三件真机上不好反复验的事：
  1. **K/B 逐帧是 server 的值**（`joint_follow` 的 25 / 0.5），且 `G(q)` 被钳到 `tau_max`
  2. 参考生成确实经过 `slew_target`（速度/加速度受限、且**永不发 NaN**）
  3. 实测 **23 ms** 量级的 `all_joint_params()` **只在启动时读一次**，绝不进循环
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
    """够 `servo` 用的假臂。记录每一次 `send_mit_all` 的五元组。

    ⚠ MIT 透传**没有** `move_js` 那条「目标 ≠ 实测位姿且 `dq == 0` 就拒帧」的限制
    （那属于固件的 `MOVE_JS` 分支）⇒ 本假臂**不需要** `reject_at`。
    """

    def __init__(self, q=None, tau_max=78.0, gravity=None):
        self.q = list(q or [0.0] * N_JOINTS)
        # 出厂值刻意与 servo 的 K/B 不同，才能测出"真的随帧下发了 server 的值"
        self.jp = [_JP(kp=400.0, kd=5.0, tau_max=tau_max) for _ in range(N_JOINTS)]
        self.kd_extra = [6.0, 6.0, 6.0, 6.0, 0.0, 0.0, 0.0]
        self.mit_calls: list = []          # (q, dq, kp, kd, tau)
        self.movej_calls: list = []
        self.move_timeout = 3.0
        self._gravity = gravity
        self.joint_param_reads = 0

    # ---- servo 用到的接口 ----
    def get_state(self, refresh=False):
        return _Msg(_State(self.q))

    def send_mit_all(self, q, dq, kp, kd, tau):
        self.mit_calls.append((list(q), list(dq), list(kp), list(kd), list(tau)))

    def movej(self, q, speed=1.0):
        self.movej_calls.append(list(q))
        self.q = list(q)

    @property
    def model(self):
        return self

    def get_gravity(self, q):
        """假重力项。⚠ 默认刻意取**远超 tau_max** 的数，好测钳位。"""
        if self._gravity is not None:
            return _Msg(list(self._gravity))
        return _Msg([100.0] * N_JOINTS)

    @property
    def params(self):
        return self

    def all_joint_params(self):
        self.joint_param_reads += 1
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
                 hz=100.0, )

    dt = 1.0 / 100.0
    prev = None
    for q, dq, *_ in arm.mit_calls:
        if prev is not None:
            step = max(abs(a - b) for a, b in zip(q, prev))
            assert step <= max(servo.DEFAULT_SPEED_LIMIT) * dt * 1.5, (
                f"单拍位移 {step} 过大 —— 参考没经过 slew_target？")
        prev = q


def test_follow_never_sends_non_finite():
    arm = FakeArm()

    def stop():
        return len(arm.mit_calls) > 20

    servo.follow(arm, lambda: [0.5] * N_JOINTS, should_stop=stop,
                 engage_sec=0.0, hz=100.0, )
    for q, dq, *_ in arm.mit_calls:
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
                 engage_sec=0.0, hz=100.0, )
    # 目标恒为初始实测位姿 ⇒ 不该走动
    for q, _dq, *_ in arm.mit_calls[1:]:
        assert max(abs(v) for v in q) < 1e-6, "返回 None 时不该往别处走"


# ────────────────────── MIT 透传：照搬 server 的 K/B ──────────────────────

def _run(arm, target, ticks=20, hz=100.0):
    """跑 `ticks` 拍（`engage_sec=0` ⇒ 只有 prime + 主循环）。"""
    n = {"i": 0}

    def stop():
        n["i"] += 1
        return n["i"] > ticks

    return servo.follow(arm, lambda: target, should_stop=stop, engage_sec=0.0, hz=hz)


def test_follow_sends_the_server_gains_on_every_frame():
    """⛔ K/B 必须**逐帧是 server 的值**（25 / 0.5）—— 这是「丝滑」的全部来源。

    旧 `move_js` 路线用的是固件出厂值 **400 / 11**（刚度 16×、阻尼 22×），
    真机判据：「跟随太慢，有明显的延迟」「没有 litearm-server 丝滑」。

    判别力：把 `follow` 里的 `kp/kd` 换回出厂值，本用例立刻红。
    """
    arm = FakeArm()
    _run(arm, [0.1] * N_JOINTS)
    assert arm.mit_calls, "一帧都没发出去"
    for _q, _dq, kp, kd, _tau in arm.mit_calls:
        assert kp == [servo.SETUP_K] * N_JOINTS, f"K 不是 server 的 {servo.SETUP_K}"
        assert kd == [servo.SETUP_B] * N_JOINTS, f"B 不是 server 的 {servo.SETUP_B}"


def test_follow_clamps_gravity_to_tau_max():
    """`τ_ff = clamp(G(q), ±tau_max)` —— 照搬 `joint_follow.compute_tau_ff`。

    ⚠ 固件对 MIT 透传**不会**自己加 G（`control_loop.c`：「`move_mit` 永不叠加内置」）
    ⇒ **这一项不发就是没有重力补偿，臂会垂。**
    """
    arm = FakeArm(tau_max=21.0)          # 假 G 恒给 100 ⇒ 必须被钳到 21
    _run(arm, [0.0] * N_JOINTS)
    assert arm.mit_calls, "一帧都没发出去"
    for _q, _dq, _kp, _kd, tau in arm.mit_calls:
        assert tau == [21.0] * N_JOINTS, f"G 没被钳到 tau_max：{tau}"


def test_follow_reads_joint_params_once_not_per_tick():
    """⚠ `all_joint_params()` 实测 **23 ms**（7 轴逐个读）⇒ 进循环就把环频钉死在 43 Hz。

    判别力：把 `_read_tau_max` 挪进主循环，本用例会红。
    """
    arm = FakeArm()
    _run(arm, [0.1] * N_JOINTS, ticks=30)
    assert arm.joint_param_reads == 1, \
        f"`all_joint_params` 被调了 {arm.joint_param_reads} 次，应当只有启动那一次"


# ────────────────────────── 收尾 ──────────────────────────
# ────────────────────────── 收尾 ──────────────────────────



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


def test_align_clips_to_limits():
    """超限的帧要**钳位**后再 movej。

    ⚠ 臂起始位姿刻意取 **0.9**（贴近上界）：这样"钳到 1.0"的位移只有 0.1，
    不会撞上 `AlignTooFar` 那条。用 0.0 起的话位移 1.0 会被**拒绝启动**，
    测的就变成另一件事了。
    """
    arm = FakeArm(q=[0.9] * N_JOINTS)
    payload = __import__("liteteleop.wire", fromlist=["x"]).encode_teleop(
        [5.0] + [0.95] * (N_JOINTS - 1), [0.0] * N_JOINTS, 0.0)
    got = servo.align_to_master(arm, lambda: (payload, 0.0), _limits(), timeout=0.5)
    assert got is not None
    assert got[0] == 1.0, "超出上界的轴必须被钳到上界"
    assert arm.movej_calls, "应该调了 movej"
    assert abs(got[1] - 0.95) < 1e-9, "没超限的轴一个数都不许动"


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


def test_align_warns_but_still_moves_on_a_large_slew(caplog):
    """⚠ 大位移**只预警、照样对齐**（用户裁决 2026-09-28：超时 3 s → 30 s）。

    30 s 在 J1 上约能走 9 rad ⇒ **限位内任何位移都够**，再"拒绝启动"就没意义了。
    但仍要**大声预警** —— 大位移意味着从臂会大幅摆动。

    判别力：`assert arm.movej_calls` 断言"还是动了" —— 改回"拒绝就不动"这条会红。
    """
    import logging
    arm = FakeArm(q=[0.0] * N_JOINTS)
    payload = __import__("liteteleop.wire", fromlist=["x"]).encode_teleop(
        [0.9] * N_JOINTS, [0.0] * N_JOINTS, 0.0)          # 位移 0.9 > 阈值 0.30
    with caplog.at_level(logging.WARNING, logger="liteteleop.servo"):
        got = servo.align_to_master(arm, lambda: (payload, 0.0), _limits(), timeout=0.5)
    assert got is not None, "不该拒绝"
    assert arm.movej_calls, "应当**照样**执行 movej（只是先预警）"
    assert any("偏大" in r.message for r in caplog.records), "大位移没预警"


# ────────────────────────── 末端载荷（夹爪）──────────────────────────

class PayloadArm(FakeArm):
    """模拟固件的**静默钳制**：mass→[0,20]、com 逐轴→[-1,1]，而且**照样回 ACK**。"""

    def __init__(self, **kw):
        super().__init__(**kw)
        self.p_mass = 0.0
        self.p_com = [0.0, 0.0, 0.0]

    def set_payload(self, mass, com=(0.0, 0.0, 0.0)):
        self.p_mass = min(max(float(mass), 0.0), 20.0)          # ⚠ 静默钳，不报错
        self.p_com = [min(max(float(v), -1.0), 1.0) for v in com]

    def get_ff_scalar(self, item, sub=0):
        if item == 4:
            return _Msg(self.p_mass)
        if item == 5:
            return _Msg(self.p_com[sub])
        raise AssertionError(f"意外的 item {item}")


def test_apply_payload_reads_back_what_actually_landed():
    """⚠ 固件**静默钳制**且照样回 ACK ⇒ 必须**读回**才知道写进去了什么。

    判别力：`apply_payload` 若返回输入值而不是读回值，本用例会红。
    """
    arm = PayloadArm()
    m, c = servo.apply_payload(arm, 0.6, (0.0, 0.0, 0.03))
    assert (m, c) == (0.6, [0.0, 0.0, 0.03])

    m2, c2 = servo.apply_payload(arm, -5.0, (2.0, 0.0, 0.0))     # 全越界
    assert m2 == 0.0, "负质量被固件钳成 0，读回必须反映这一点"
    assert c2[0] == 1.0, "质心被钳到 [-1,1]"


def test_default_payload_is_the_gripper_the_user_gave():
    assert servo.DEFAULT_PAYLOAD_MASS == 0.6            # 600 g
    assert servo.DEFAULT_PAYLOAD_COM == (0.0, 0.0, 0.03)  # 质心 3 cm 在 Z 轴


# ────────────────────────── 跟随增益的写入 / 还原 ──────────────────────────

def test_apply_writes_kb_and_zeroes_kd_extra():
    """⚠ **必须清零 `kd_extra`**：`move_js` 的有效阻尼 = `mit_kd + kd_extra`。

    不清的话 J1~J4 是 0.5+6.0 = 6.5（13 倍），`kd·dq` 在 5 rad/s 时 32.5 Nm，
    而 J4 的 `tau_max` 只有 21 ⇒ 力矩预算被吃光 ⇒ 抖。
    """
    arm = FakeArm()
    servo.apply_joint_gains(arm)
    assert [p.kp for p in arm.jp] == [25.0] * N_JOINTS
    assert [p.kd for p in arm.jp] == [0.5] * N_JOINTS
    assert arm.kd_extra == [0.0] * N_JOINTS


def test_apply_keeps_tau_max():
    arm = FakeArm()
    before = [p.tau_max for p in arm.jp]
    servo.apply_joint_gains(arm)
    assert [p.tau_max for p in arm.jp] == before


def test_restore_is_verbatim_and_idempotent():
    arm = FakeArm()
    kp0, kd0, ex0 = ([p.kp for p in arm.jp], [p.kd for p in arm.jp], list(arm.kd_extra))
    saved = servo.apply_joint_gains(arm)
    assert [p.kp for p in arm.jp] != kp0, "改之前得真的改了"
    servo.restore_joint_gains(arm, saved)
    assert [p.kp for p in arm.jp] == kp0
    assert [p.kd for p in arm.jp] == kd0
    assert arm.kd_extra == ex0
    servo.restore_joint_gains(arm, saved)          # 幂等：再来一次不该炸/不该改坏
    assert [p.kp for p in arm.jp] == kp0


def test_verified_setup_values():
    """真机验证过的那一组（2026-09-28：150 拍 / 2 s 最大偏移 0.0004 rad）。"""
    assert servo.SETUP_K == 25.0
    assert servo.SETUP_B == 0.5
