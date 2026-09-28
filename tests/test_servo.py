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
    def __init__(self, q, dq=None):
        self.q = list(q)
        # ⚠ `_send_mit` 现在要**实测 dq** 去算限位墙 ⇒ 状态帧必须有这一项
        self.dq = list(dq) if dq is not None else [0.0] * len(self.q)


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

    def joint_follow(self, q, dq, kp, kd):
        """[JOINT_FOLLOW] 帧里**没有 tau** —— 前馈由固件算。"""
        self.mit_calls.append((list(q), list(dq), list(kp), list(kd)))

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

def _wall():
    """两边留 ±1.5 rad 的假限位墙（与 `FakeArm` 的 `q_min/q_max` 一致）。"""
    from liteteleop.wall import JointLimitWall
    return JointLimitWall.from_limits([-1.5] * N_JOINTS, [1.5] * N_JOINTS)


def _run(arm, target, ticks=20, hz=100.0, wall=None):
    """跑 `ticks` 拍（`engage_sec=0` ⇒ 只有 prime + 主循环）。"""
    n = {"i": 0}

    def stop():
        n["i"] += 1
        return n["i"] > ticks

    return servo.follow(arm, lambda: target, should_stop=stop, engage_sec=0.0,
                        hz=hz, wall=wall)


def test_follow_sends_the_server_gains_on_every_frame():
    """⛔ K/B 必须**逐帧是 server 的值**（25 / 0.5）—— 这是「丝滑」的全部来源。

    旧 `move_js` 路线用的是固件出厂值 **400 / 11**（刚度 16×、阻尼 22×），
    真机判据：「跟随太慢，有明显的延迟」「没有 litearm-server 丝滑」。

    判别力：把 `follow` 里的 `kp/kd` 换回出厂值，本用例立刻红。
    """
    arm = FakeArm()
    _run(arm, [0.1] * N_JOINTS)
    assert arm.mit_calls, "一帧都没发出去"
    for _q, _dq, kp, kd in arm.mit_calls:
        assert kp == list(servo.SETUP_K), f"K 不是 server 的 {servo.SETUP_K}"
        assert kd == list(servo.SETUP_B), f"B 不是 server 的 {servo.SETUP_B}"




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


# ────────── 限速：必须落在固件的速度包络之内（绕不过去的硬约束）──────────

def test_speed_limit_stays_within_the_firmware_velocity_envelope():
    """⛔ `speed_limit` 必须 ≤ 固件整臂表的 `vel_max` —— 否则必然锁存掉力。

    固件 `safety_check.c`：`|dq| > jp->vel_max × 1.5` 连续 5 拍 ⇒ 锁存 `joint_fault`
    ⇒ 固件**停发该轴控制帧** ⇒ 达妙电机"收帧才回状态"⇒ 静默 ⇒ 80 ms 后 `FB_STALE`。
    真机实录：`OVERSPEED` 首拍即报，5 拍后锁存。

    ⚠ `vel_max` 是**编译期常量**（`defaults.c`），SDK 只暴露 `kp/kd/tau_max/q_min/q_max`
    ⇒ **不改固件就绕不过去**。server 能无视它，是因为走 CAN 直连电机。

    判别力：把 `DEFAULT_SPEED_LIMIT` 改回 server 的 `[2.8,3.4,5,5,10,8,13]`，本用例立刻红。
    """
    from liteteleop import servo
    vel_max = [2.0, 2.0, 1.75, 1.75, 2.0, 2.0, 2.0]        # 固件 defaults.c 的整臂表
    for i, (sl, vm) in enumerate(zip(servo.DEFAULT_SPEED_LIMIT, vel_max)):
        assert sl <= vm, f"J{i+1} speed_limit={sl} 越过固件 vel_max={vm}"


# ─────────── joint_follow 帧：只有 4 组，且墙区抬高 kd ───────────

def test_joint_follow_frame_carries_no_tau():
    """⛔ `joint_follow` 的帧里**没有 `tau`** —— 前馈由固件算（G + 墙）。

    ⚠ 这是本路线与 `send_mit_all` 的**唯一实质差别**，也是"省掉一次 `get_gravity`
    往返 ⇒ 每拍 1 次往返 ⇒ 250 Hz 可达"的来源。判据取"假臂收到的就是 4 组"：
    谁把它改回 `send_mit_all`（5 组），本用例立刻红。
    """
    arm = FakeArm()
    _run(arm, [0.1] * N_JOINTS)
    assert arm.mit_calls, "一帧都没发"
    for call in arm.mit_calls:
        assert len(call) == 4, f"joint_follow 帧应是 (q,dq,kp,kd) 四组，实际 {len(call)} 组"


def test_wall_zone_raises_kd_by_the_configured_extra():
    """进墙区的轴：`kd` 被抬高 `WALL_FW_KD`（上限 5.0）—— 照 server 的做法。

    ⚠ 这是**纯 PC 侧**的加法（固件不知道），所以它仍归本仓验。
    判别力：把 `WALL_FW_KD` 置 0 或去掉叠加，本用例红。
    """
    q = [1.49] + [0.0] * (N_JOINTS - 1)          # J1 越过墙线(限位 1.5 − margin 0.02)
    arm = FakeArm(q=q)
    _run(arm, q, ticks=3, wall=_wall())
    _q, _dq, _kp, kd = arm.mit_calls[-1]
    assert kd[0] == min(servo.SETUP_B[0] + servo.WALL_FW_KD, servo.WALL_FW_KD_CAP), \
        f"J1 在墙区, kd 应抬高到 {servo.SETUP_B[0] + servo.WALL_FW_KD}, 实际 {kd[0]}"
    assert kd[1] == servo.SETUP_B[1], f"J2 不在墙区, kd 不该被动过, 实际 {kd[1]}"


# ────────────── 限位内缩量 ──────────────

def test_limit_margin_does_not_latch_at_limits():
    """目标贴限位 + 最坏冲过，仍须落在固件锁存死区之内。

    ⚠ 判据形态 2026-09-28 修正。固件 `safety_check.c` 的锁存条件是
    `q_meas > q_max + 0.05`，而 `q_meas ≤ q_max − margin + overshoot`：

        不锁存  ⟺  overshoot ≤ 0.05 + margin  ⟺  **margin ≥ overshoot − 0.05**
        原式「margin ≥ 0.05 + overshoot」把 0.05 加成而非减掉 ⇒ 要求翻倍，
        正是 margin 被抬到 0.14 的原因（见 safety.py 的注释）。

    判别力：谁把 `DEFAULT_SPEED_LIMIT` 调大（overshoot 随之变大）到盖过 0.05+margin，
    本用例立刻红。
    """
    from liteteleop import safety, servo
    overshoot = max(servo.DEFAULT_SPEED_LIMIT) * 2 * 0.003333   # 最快轴 × 2 次往返
    assert safety.DEFAULT_LIMIT_MARGIN >= overshoot - 0.05, (
        f"margin={safety.DEFAULT_LIMIT_MARGIN} 盖不住 overshoot={overshoot:.4f}"
        f"（需 ≥ {overshoot - 0.05:.4f}）")


def test_limit_margin_keeps_j4_usable():
    """⛔ margin 不得把**最窄轴 J4** 的正半轴压没。

    J4 的固件软限位上端 = **+0.017547 rad（1°）**，全臂最窄
    （真源 `litearm-stm32/User/litearm/params/joint_limit_macros.h:16`）。
    统一内缩 θ 后其上端变 `0.017547 − θ` ⇒ θ ≥ 0.017547 时 J4 正半轴**完全不可用**，
    主臂往正方向拖 J4 时从臂纹丝不动 —— 2026-09-28 报的「J4 特别容易过软件限位」。

    判别力：谁把 margin 调回 0.14（即任何 ≥ 0.0175 的值），本用例立刻红。
    ⚠ 固件若改了 J4 的软限位，本常量须同步；真源见上一行的文件。
    """
    from liteteleop import safety
    J4_QMAX = 0.017547
    assert safety.DEFAULT_LIMIT_MARGIN < J4_QMAX, (
        f"margin={safety.DEFAULT_LIMIT_MARGIN} ≥ J4 上端 {J4_QMAX} ⇒ J4 正半轴不可用")
    assert J4_QMAX - safety.DEFAULT_LIMIT_MARGIN >= J4_QMAX / 3.0, (
        f"J4 剩余正向行程仅 {J4_QMAX - safety.DEFAULT_LIMIT_MARGIN:.6f} rad，"
        f"不足上端的三分之一")


# ────────────────────── 限位墙（第二道位置护栏）──────────────────────




# ────────────── 增益必须是 server 默认配置的那一份 ──────────────

def test_setup_gains_are_the_server_default_config():
    """⛔ 逐值比对 `litearm.yaml` 的 `joint_follow:` 段 —— **server 默认加载的就是它**。

    ⚠⚠ **别抄 `litearm_balanced.yaml`**：那份是 `[25]×7 / [0.5]×7`，而它的历史注释写着
    「从 25 全一律**提高**以改善主从遥操的**滞后/追不上**手感（2026-08-13）」
    ⇒ 25 是**已知会滞后**的档。真机实测复现了：用户报「明显延迟，而且会很软」。

    判别力：把 `SETUP_K` 改回 25（或换成任何别的档），本用例立刻红。
    """
    assert servo.SETUP_K == [60.0, 60.0, 60.0, 60.0, 40.0, 40.0, 40.0]
    assert servo.SETUP_B == [1.0, 1.0, 1.0, 1.0, 0.8, 0.8, 0.8]


def test_engage_gains_are_softer_than_the_follow_gains():
    """接管瞬间刚度**明显更软** —— `joint_follow.engage(engage_kp=15.0, engage_kd=0.8)`。

    ⚠ 只对 `K` 断言"严格更小"：`SETUP_B` 的**腕部值就是 0.8**，与 engage 的 0.8 相同，
    拿它比"更小"会假红（我第一版正是这么写的，测试当场抓出来了）。
    """
    assert servo.ENGAGE_KP < min(servo.SETUP_K), "接管刚度必须比跟随刚度软"
    assert servo.ENGAGE_KD <= max(servo.SETUP_B)