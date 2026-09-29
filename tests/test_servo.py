"""从臂伺服环的离线测试（用假臂，不需要硬件）。

覆盖三件真机上不好反复验的事：
  1. **K/B 逐帧下发的是本仓的增益**（基线取自 `litearm.yaml` 的 `joint_follow:` 段，
     但 2026-09-29 起**有意整体放大**以压制快速拖动下的过冲 —— 依据见 `servo.SETUP_K`）
  2. 参考生成确实经过 `slew_target`（速度/加速度受限、且**永不发 NaN**）
  3. 实测 **23 ms** 量级的 `all_joint_params()` **只在启动时读一次**，绝不进循环
  4. `joint_follow` 的帧**只有四组**（q/dq/kp/kd）—— **没有 `tau`**，`G` 由固件算
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
    """够 `servo` 用的假臂。记录每一次 `joint_follow` 的四元组（q/dq/kp/kd）。

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
        """假重力项。

        ⛔ **已无调用点** —— S4 起 `G` 由固件算（`CMD_JOINT_FOLLOW`），`follow()` 不再
        调 `get_gravity`。保留作测试替身的一部分，免得将来要在 PC 侧验 G 相关逻辑时重造。
        """
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
    """⛔ K/B 必须**逐帧是 server 的值**（`litearm.yaml` 的 `joint_follow:` 段）。

    ⚠ **别抄 `litearm_balanced.yaml`** 的 25 / 0.5 —— 那一档自己的历史注释就写着
    "从 25 一律提高以改善滞后/追不上"，真机复现过同样的「明显延迟、很软」。

    旧 `move_js` 路线用的是固件出厂值 **400 / 11**（刚度 6.7×、阻尼 11×），
    真机判据：「跟随太慢，有明显的延迟」。

    判别力：把 `follow` 里的 `kp/kd` 换回出厂值，本用例立刻红。
    """
    arm = FakeArm()
    _run(arm, [0.1] * N_JOINTS)
    assert arm.mit_calls, "一帧都没发出去"
    for _q, _dq, kp, kd in arm.mit_calls:
        assert kp == list(servo.SETUP_K), f"K 不是 `servo.SETUP_K` 的 {servo.SETUP_K}"
        assert kd == list(servo.SETUP_B), f"B 不是 `servo.SETUP_B` 的 {servo.SETUP_B}"




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
    """⛔ `speed_limit` 必须 ≤ **固件 joint_follow 专用表** `s_jf_vel_max`。

    ⚠ 对标物 2026-09-29 换过。原先比的是 `jp->vel_max`（固件整臂表
    `[2.0, 2.0, 1.75, 1.75, 2.0, 2.0, 2.0]`）—— 那张表是 **`movej` 的"满速语义"**，
    固件却拿它当所有模式的 slew 上限，于是把 joint_follow 也锁在 2.0。真机表现：
    「从臂永远在追」（实测滞后 0.35 rad / 20°）。现已给 joint_follow 加专用表
    （`control_loop.c` 的 `s_jf_vel_max`，逐值照抄 server）。

    为什么仍要"≤"：两级 slew **串联**（PC `slew_target` → 固件 `slew_linear`），
    **谁小谁说了算**。PC 若大于固件那张表，多出来的部分不会变成速度，
    只会让 `q_cmd` 空超前、跟踪误差读数虚高。

    判别力：改任一端的表而不同步另一端，本用例立刻红。
    """
    from liteteleop import servo
    # 固件 control_loop.c 的 s_jf_vel_max（= litearm.yaml 的 joint_follow.speed_limit）
    jf_vel_max = [2.8, 3.4, 5.0, 5.0, 10.0, 8.0, 13.0]
    for i, (sl, vm) in enumerate(zip(servo.DEFAULT_SPEED_LIMIT, jf_vel_max)):
        assert sl <= vm, f"J{i+1} speed_limit={sl} 越过固件 s_jf_vel_max={vm}"


# ─────────── joint_follow 帧：只有 4 组，且墙区抬高 kd ───────────

def test_joint_follow_frame_carries_no_tau():
    """⛔ `joint_follow` 的帧里**没有 `tau`** —— 前馈由固件算（G + 墙）。

    ⚠ 这是本执行器与 `send_mit_all` 的**实质差别之一**：省掉一次 `get_gravity` 往返
    ⇒ 每拍 1 次往返（节拍由 `DEFAULT_HZ` 定，当前 150 Hz；硬件上限 ~300 Hz）。
    判据取"假臂收到的就是 4 组"：谁改回 `send_mit_all`（5 组），本用例立刻红。
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

def test_limit_margin_stays_in_a_sane_band():
    """`DEFAULT_LIMIT_MARGIN` 取 server 值，落在 [0.005, J4 上端) 之间。

    ⚠ 判据形态 2026-09-29 换掉了。旧判据是 `margin ≥ overshoot − 0.05`
    （依据：固件锁存条件 `q_meas > q_max + 0.05`）。**那个前提已不成立**：
    joint_follow 会话里位置判据**已豁免**（S2）⇒ 贴限位**不会**锁存掉力，
    「盖住死区」这个诉求随之消失。

    ⚠ 为什么不再按 `overshoot` 反推：`overshoot = 13 × 6.7ms ≈ 0.087 rad`，
    反推会要求 margin ≈ 0.037 —— 那会**白吃行程**（J4 上端总共才 2°），
    而 server 用同样的 0.01 + 13 rad/s 跑得很好。真正兜底的是固件 `law_wall`
    （距限位 `wall_margin` 就给排斥力矩）与已豁免的锁存判据。

    ⚠ 代价如实记：13 rad/s 下实测仍可能冲过软限位零点几度、**贴近物理硬限位**
    （软限位距它只有 1°）。这是 server 同样接受的取舍，不是本仓引入的。

    判别力：下界拦「把防撞余量砍光」，上界拦「吃没 J4 行程」
    （后者另有 `test_limit_margin_keeps_j4_usable` 专门守）。
    """
    from liteteleop import safety
    assert 0.005 <= safety.DEFAULT_LIMIT_MARGIN < 0.0175, (
        f"margin={safety.DEFAULT_LIMIT_MARGIN} 超出合理区间 [0.005, J4 上端 0.0175)")


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

def test_setup_gains_are_deliberately_stiffer_than_server():
    """K/B **有意偏离 server**（2026-09-29）—— 本用例锁的是**偏离的方向**，不是数值本身。

    背景（完整依据见 `servo.py` 里 `SETUP_K` / `SETUP_B` 的注释）：
      server 的 `K=60 / B=1.0` 是**为慢速遥操定的**；快速拖动下它必然过冲，而且
      **server 自己也有**（用户复述「150Hz 的也有」）。真机实测滞后 **0.2919 rad**、
      J2 过冲 **0.2516 rad**，两者**比值 0.86** —— 这是欠阻尼二阶系统的标志。
      机理：`q_cmd` 一停，从臂身后还欠着滞后量没还，它带着速度扑上去 ⇒ 冲过头再被拉回
      （用户原话「有点过冲然后回拉的感觉」）。

    绝对过冲 = **滞后 × 过冲率**，两个因子各压一路：
      滞后 ∝ 1/K ⇒ 提 K；   过冲率 = f(ζ)、ζ = B/(2√(K·J)) ⇒ **同步提 B**。

    ⇒ 钉四条不变量（比"逐值等于某个数"更有判别力）：
      ① **K 整体严格高于 server 基线** —— 否则退回"必然过冲"的那一档；
      ② **每轴 ζ 不许下降**：`(B/B_server) / √(K/K_server) ≥ 1.2`。
         ⚠⚠ 这条抓的是**"只提 K 不提 B"**这个真实错误 —— ζ 会随 √K 下降，
            把提 K 的收益吃掉一部分。**它是本用例存在的主要理由。**
      ③ **B 不许超过固件上限** —— 超了会被 `MIT_KD_MAX` **静默钳住**，表上"看起来"
         满足等比、实际值与意图不符，属静默失败（这一条是"判据的判据"）。
      ④ **J2 仍是 B 最大的那一轴**（肩部托着整条臂 ⇒ 惯量最大 ⇒ ζ 最小 ⇒ 过冲最大）。

    ⚠ 判别力已逐条实跑验证：`SETUP_B` 改回 server 原值 ⇒ ① + ② 红（min ζ 比 1.00×）；
      "只提 K 不提 B" ⇒ ② 红（0.55×）；`SETUP_K` 改回 60 ⇒ ① 红；
      J2 的 B 写 6.0 ⇒ ③ 红；把 J1 的 B 抬到超过 J2 ⇒ ④ 红。
    ⚠⚠ **④ 的已知盲区（如实记账，别假装它抓得住）**：把 J2 的 B 降到与其余轴同倍（3.0）
      时本用例**仍然全绿** —— 那种改法 ζ 仍达标（1.64×）、J2 也仍最大。
      它的病灶是 **J2 的绝对 ζ 仍最低**，而判它需要各轴惯量 `J`（`ζ = B/(2√(K·J))`），
      本仓没有 J 的实测值 ⇒ **这一格判不了**，只能靠真机统计 + `servo.py` 的注释守着。
    """
    server_k = [60.0, 60.0, 60.0, 60.0, 40.0, 40.0, 40.0]
    server_b = [1.0, 1.0, 1.0, 1.0, 0.8, 0.8, 0.8]

    # ① 整体更硬 —— 本仓的刻意选择
    assert all(k > s for k, s in zip(servo.SETUP_K, server_k)), (
        f"K 必须整体高于 server 基线 {server_k} —— 退回那一档就等于接受"
        f"「快速拖动必然过冲」；实际 {servo.SETUP_K}")

    # ② 提 K 必须配提 B，否则 ζ 会随 √K 下降
    for i, (k, b, sk, sb) in enumerate(zip(servo.SETUP_K, servo.SETUP_B, server_k, server_b)):
        ratio = (b / sb) / math.sqrt(k / sk)
        assert ratio >= 1.2, (
            f"J{i + 1} 的 ζ 只到 server 的 {ratio:.2f}× —— K 提上去了但 B 没跟上，"
            f"ζ 会被 √K 拉低 ⇒ 过冲率变差，吃掉提 K 的收益。"
            f"K={k}（server {sk}）、B={b}（server {sb}）")

    # ③ 别写超上限的值（固件 control_loop.c:45）
    _MIT_KD_MAX = 5.0
    assert all(b <= _MIT_KD_MAX for b in servo.SETUP_B), (
        f"B 超过固件 `MIT_KD_MAX={_MIT_KD_MAX}` 会被**静默钳住** —— 表上看着对、"
        f"实际值不符，属静默失败。要更高得先改固件。实际 {servo.SETUP_B}")

    # ④ 病灶轴不能掉队
    assert servo.SETUP_B[1] == max(servo.SETUP_B), (
        f"J2 是肩部关节、托着整条臂 ⇒ 负载惯量最大 ⇒ ζ 最小、过冲最大，"
        f"必须是 B 最大的那一轴；实际 {servo.SETUP_B}")


def test_engage_gains_are_softer_than_the_follow_gains():
    """接管瞬间刚度**明显更软** —— `joint_follow.engage(engage_kp=15.0, engage_kd=0.8)`。

    ⚠ 只对 `K` 断言"严格更小"：`SETUP_B` 的**腕部值就是 0.8**，与 engage 的 0.8 相同，
    拿它比"更小"会假红（我第一版正是这么写的，测试当场抓出来了）。
    """
    assert servo.ENGAGE_KP < min(servo.SETUP_K), "接管刚度必须比跟随刚度软"
    assert servo.ENGAGE_KD <= max(servo.SETUP_B)