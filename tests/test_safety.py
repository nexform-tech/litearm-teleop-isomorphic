"""safety.py —— 纯逻辑层（不 import litearm，不碰硬件）。

⭐ 核心判据：移植的 `slew_target` 与 `pylitearm` 原版**同一组输入逐拍全等**。
这是"照抄而非重写"的唯一硬证据；没有它，"我照抄了"只是自述。
"""
import math

import pytest

from liteteleop import safety

# ── 参照实现：从 pylitearm/src/pylitearm/control/joint_follow.py:45-88 抄成独立副本 ──────────
# ⚠ 这份副本是**测试的对照组**，刻意与实现分开写。两边都错才可能同时通过。


def _ref_slew(raw_target, q_cmd, dq_cmd, speed_limit, accel_limit, dt):
    dt = max(dt, 1e-4)
    for i in range(len(raw_target)):
        v_limit = max(1e-4, speed_limit[i])
        a_limit = max(1e-4, accel_limit[i])
        dv_max = a_limit * dt
        diff = raw_target[i] - q_cmd[i]
        v = dq_cmd[i]
        if abs(diff) < 1e-5 and abs(v) < dv_max:
            q_cmd[i] = raw_target[i]
            dq_cmd[i] = 0.0
            continue
        stopping_dist = (v * v) / (2.0 * a_limit) if a_limit > 0.0 else 0.0
        moving_toward = diff * v > 0.0
        if moving_toward and abs(diff) <= stopping_dist:
            desired_v = 0.0
        else:
            desired_v = math.copysign(v_limit, diff)
        v += max(-dv_max, min(dv_max, desired_v - v))
        v = max(-v_limit, min(v_limit, v))
        step = v * dt
        if diff * step > 0.0 and abs(step) >= abs(diff):
            q_cmd[i] = raw_target[i]
            dq_cmd[i] = 0.0
        else:
            q_cmd[i] += step
            dq_cmd[i] = v
    return q_cmd, dq_cmd


def _traj(t, q0):
    """确定的合成轨迹 —— 固定输入，对拍才可复现。"""
    tgt = list(q0)
    tgt[1] += 0.30 * math.sin(2 * math.pi * 0.5 * t)
    tgt[2] += 0.20 * math.sin(2 * math.pi * 0.3 * t + 0.7)
    return tgt


def test_slew_matches_pylitearm_reference():
    """⭐ 移植版与原版**逐拍全等**（60 拍 × 3 组参数 = 180 次比对）。"""
    n, dt = 7, 0.01
    for sp, ac in [(0.3, 14.0), (2.0, 24.0), (1.2, 45.0)]:
        q0 = [0.1 * i for i in range(n)]
        a_cmd, a_dq = list(q0), [0.0] * n
        b_cmd, b_dq = list(q0), [0.0] * n
        for k in range(60):
            tgt = _traj(k * dt, q0)
            a_cmd, a_dq = safety.slew_target(tgt, a_cmd, a_dq, [sp] * n, [ac] * n, dt)
            b_cmd, b_dq = _ref_slew(tgt, b_cmd, b_dq, [sp] * n, [ac] * n, dt)
            assert a_cmd == pytest.approx(b_cmd, abs=1e-15), f"sp={sp} ac={ac} 第 {k} 拍 q_cmd 分叉"
            assert a_dq == pytest.approx(b_dq, abs=1e-15), f"sp={sp} ac={ac} 第 {k} 拍 dq_cmd 分叉"


def test_slew_speed_limited():
    """|dq_cmd| 恒 ≤ speed_limit —— spec §5.1 那条「accel_limit 不抬高天花板」的依据。"""
    n, dt, sp, ac = 7, 0.01, 0.5, 20.0
    q_cmd, dq_cmd = [0.0] * n, [0.0] * n
    for k in range(200):
        q_cmd, dq_cmd = safety.slew_target([3.0] * n, q_cmd, dq_cmd, [sp] * n, [ac] * n, dt)
        assert all(abs(v) <= sp + 1e-12 for v in dq_cmd), f"第 {k} 拍越过 speed_limit"


def test_slew_converges_and_stops():
    """收敛且到目标即停（不超冲、不振荡）。"""
    n, dt = 7, 0.01
    q_cmd, dq_cmd = [0.0] * n, [0.0] * n
    for _ in range(1000):
        q_cmd, dq_cmd = safety.slew_target([1.0] * n, q_cmd, dq_cmd, [2.0] * n, [14.0] * n, dt)
    assert q_cmd == pytest.approx([1.0] * n, abs=1e-6)
    assert dq_cmd == pytest.approx([0.0] * n, abs=1e-9)


def test_slew_brakes_before_target():
    """⭐ 制动距离减速**真的生效** —— 判据是「最长减速连跑」拍数。

    为什么不用"有没有过冲"当判据：**过冲为 0 是被别的东西挡住的**（见下方模块注释与
    Task 5 Step 5 的实测），去掉制动距离后过冲**依然是 0** ⇒ 用过冲当判据是**没有判别力**的。

    实测（本计划作者量化过）：`sp=2.0, ac=14.0` 下 ——
    原版最长减速连跑 **14 拍**；去掉制动距离 **0 拍**；去掉吸附 **14 拍**。
    """
    n, dt, tgt = 7, 0.01, 0.5
    q_cmd, dq_cmd = [0.0] * n, [0.0] * n
    seq, worst = [], 0.0
    for _ in range(400):
        q_cmd, dq_cmd = safety.slew_target([tgt] * n, q_cmd, dq_cmd, [2.0] * n, [14.0] * n, dt)
        seq.append(abs(dq_cmd[0]))
        worst = max(worst, max(q_cmd) - tgt)
    run = best = 0
    prev = None
    for v in seq:
        run = run + 1 if (prev is not None and 0.0 < v < prev) else 0
        best = max(best, run)
        prev = v
    assert best >= 5, f"没有减速段（最长连跑 {best} 拍）—— 制动距离减速没生效"
    # 顺带：不过冲。⚠ 这条**单独没有判别力**（制动与吸附都去掉才为假），
    # 只作为上面那条强判据的附带检查，别当成独立覆盖。
    assert worst <= 1e-9, f"过冲 {worst}"


# ── 软限位闸门 ──────────────────────────────────────────────────────────────


def test_limits_ok():
    lim = safety.read_limits_ok([-1.0] * 7, [1.0] * 7, 7)
    assert lim.n == 7

@pytest.mark.parametrize("lo,hi,n,match", [
    ([-1.0] * 6, [1.0] * 7, 7, "长度不符"),
    ([-1.0] * 7, [1.0] * 7, 8, "长度不符"),
    ([float("nan")] * 7, [1.0] * 7, 7, "非有限值"),
    ([1.0] * 7, [-1.0] * 7, 7, "上下界反了"),
    ([0.0] * 7, [0.0] * 7, 7, "上下界反了"),
])


def test_limits_reject(lo, hi, n, match):
    """限位任何一处不合法都**拒启动** —— 不静默退化（spec §7.1）。"""
    with pytest.raises(safety.LimitsError, match=match):
        safety.read_limits_ok(lo, hi, n)


def test_clamp_flags_saturated_axes():
    """被钳住的轴要被标出来（抗饱和的输入）。"""
    lim = safety.read_limits_ok([-1.0] * 7, [1.0] * 7, 7)
    q, sat = safety.clamp_to_limits([0.5, 2.0, -0.3, -5.0, 0.0, 1.0, 0.9], lim)
    assert q == pytest.approx([0.5, 1.0, -0.3, -1.0, 0.0, 1.0, 0.9])
    assert sat == [False, True, False, True, False, False, False]


def test_saturate_dq_zeroes_clamped_axes():
    """spec §5.1：被钳位的轴 `dq` 必为 0（这条不变量的具名载体）。"""
    dq = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
    sat = [False, True, False, True, False, False, False]
    assert safety.saturate_dq(dq, sat) == [1.0, 0.0, 1.0, 0.0, 1.0, 1.0, 1.0]
    assert safety.saturate_dq([2.0] * 7, [False] * 7) == [2.0] * 7, "没被钳的不许动"


def test_saturate_dq_in_j4_upper_limit_case():
    """⚠ J4 上端只有 +0.0175 rad —— 主臂一拖过零就吃到，不是边角情况。"""
    # ⚠ 用 J4 的**真实**软限位（`e2e_movejs_real.py` 的 REAL_MIN/REAL_MAX）：
    # 下 -3.0715、上只有 +0.0175 —— 是**非对称**的紧上端，不是一个零宽度的轴。
    # （早先这里把 lo/hi 都写成 0.0175，等于构造了一个非法的 `Limits`；
    #   零宽度现在被类型本身拒绝，因为这测试原先把"工厂是唯一守卫"当成了前提。）
    lo = (-3.0, -3.0, -3.0, -3.0715, -3.0, -3.0, -3.0)
    hi = (3.0, 3.0, 3.0, 0.0175, 3.0, 3.0, 3.0)
    lim = safety.read_limits_ok(lo, hi, 7)
    q, sat = safety.clamp_to_limits([0.0, 0.0, 0.0, 0.20, 0.0, 0.0, 0.0], lim)
    assert sat[3] is True, "J4 超出 0.0175 应被判被钳"
    dq = safety.saturate_dq([1.0] * 7, sat)
    assert dq[3] == 0.0, "被钳的 J4 必须把 dq 置 0（否则 kd·dq 持续顶限位）"


def test_clamp_uses_range_not_float_equality():
    """⚠ 判据用区间比较，不用浮点 `!=`（spec §5.1 的 `saturated` 定义）。"""
    lim = safety.read_limits_ok([0.0] * 7, [1.0] * 7, 7)
    _, sat = safety.clamp_to_limits([0.0, 1.0, 0.5, 0.5, 0.5, 0.5, 0.5], lim)
    assert sat == [False, False, False, False, False, False, False], "边界值不应算被钳"

# ── kd 预算闸门 ─────────────────────────────────────────────────────────────


def test_speed_limit_from_kd_matches_spec_table():
    """spec §5.1 那张表：本机 7 轴 @30% 预算。"""
    kd = [11.0, 11.0, 10.0, 11.0, 2.5, 2.5, 2.5]          # mit_kd + kd_extra
    tau = [78.0, 78.0, 21.0, 21.0, 10.0, 10.0, 10.0]
    got = safety.speed_limit_from_kd(kd, tau, kd_budget=0.30)
    assert got == pytest.approx([2.127, 2.127, 0.63, 0.573, 1.2, 1.2, 1.2], abs=0.01)


def test_speed_limit_j4_is_the_binding_case():
    """⛔ J4 是全局最紧的轴 —— 它决定了「高速跟随」到底多快。"""
    kd = [11.0, 11.0, 10.0, 11.0, 2.5, 2.5, 2.5]
    tau = [78.0, 78.0, 21.0, 21.0, 10.0, 10.0, 10.0]
    got = safety.speed_limit_from_kd(kd, tau)
    assert min(got) == pytest.approx(got[3]), "最紧的应是 J4"
    assert min(got) < 0.7, "J4 的上限必须明显低于它的 speed_limit=1.75"

@pytest.mark.parametrize("budget", [0.0, -0.1, 1.5])
def test_speed_limit_rejects_bad_budget(budget):
    with pytest.raises(safety.LimitsError, match="kd_budget"):
        safety.speed_limit_from_kd([1.0] * 7, [1.0] * 7, kd_budget=budget)


def test_speed_limit_rejects_bad_kd():
    with pytest.raises(safety.LimitsError, match="非法"):
        safety.speed_limit_from_kd([0.0] * 7, [1.0] * 7)


# ── 状态机与 watchdog ───────────────────────────────────────────────────────


def _sm(**kw):
    defaults = dict(n=7, watchdog_ms=200.0, recover_frames=5, hold_escalate_s=2.0)
    defaults.update(kw)
    return safety.TeleopState(**defaults)


def test_starts_idle():
    assert _sm().state == safety.IDLE


def test_start_goes_to_align_fast():
    sm = _sm()
    sm.start(now=0.0)
    assert sm.state == safety.ALIGN_FAST


def test_align_done_goes_to_following():
    sm = _sm()
    sm.start(now=0.0)
    sm.align_done(now=1.0)
    assert sm.state == safety.FOLLOWING


def test_watchdog_trips_to_holding():
    sm = _sm()
    sm.start(now=0.0)
    sm.align_done(now=1.0)
    assert sm.tick(now=1.0 + 0.1, frame_age=0.1) is safety.FOLLOWING
    # 200 ms 无新帧 ⇒ HOLDING
    assert sm.tick(now=1.0 + 0.35, frame_age=0.35) is safety.HOLDING


def test_hold_escalates_once_after_2s():
    """HOLDING ≥ 2 s ⇒ 升级 movej(q_now) **一次**，且**仍留在 HOLDING**。"""
    sm = _sm()
    sm.start(now=0.0)
    sm.align_done(now=1.0)
    sm.tick(now=1.5, frame_age=0.5)
    assert sm.state == safety.HOLDING
    assert sm.wants_stop_command() is False, "刚进 HOLDING 不该升级"
    sm.tick(now=2.6, frame_age=1.6)                 # 进入 HOLDING 已 1.1 s
    assert sm.wants_stop_command() is False
    sm.tick(now=3.6, frame_age=2.6)                 # 进入 HOLDING 已 2.1 s
    assert sm.wants_stop_command() is True
    assert sm.state == safety.HOLDING, "升级后仍留在 HOLDING"
    sm.consume_stop_command()
    sm.tick(now=4.0, frame_age=2.0)
    assert sm.wants_stop_command() is False, "升级只做一次"


def test_hold_recovers_to_align_fast_not_following():
    """恢复必须重走 ALIGN_FAST —— 主臂在断链期间可能已经动了（spec §5.2）。"""
    sm = _sm()
    sm.start(now=0.0)
    sm.align_done(now=1.0)
    sm.tick(now=1.5, frame_age=0.5)
    assert sm.state == safety.HOLDING
    for k in range(5):                              # 连续 5 拍收到帧
        sm.tick(now=2.0 + 0.01 * k, frame_age=0.01)
    assert sm.state == safety.ALIGN_FAST


def test_hold_recovery_needs_consecutive_frames():
    """不足 5 拍不许恢复（防抖动来回切）。"""
    sm = _sm()
    sm.start(now=0.0)
    sm.align_done(now=1.0)
    sm.tick(now=1.5, frame_age=0.5)
    for k in range(4):
        sm.tick(now=2.0 + 0.01 * k, frame_age=0.01)
    assert sm.state == safety.HOLDING
    sm.tick(now=2.10, frame_age=0.5)                # 又断了 ⇒ 计数清零
    sm.tick(now=2.20, frame_age=0.01)
    assert sm.state == safety.HOLDING


def test_align_failed_stays_put():
    """对齐失败必须停在 ALIGN_FAST —— 且 tick 永远不把它推进 FOLLOWING（spec §5.1）。"""
    sm = _sm()
    sm.start(now=0.0)
    sm.align_failed(now=1.0)
    assert sm.state == safety.ALIGN_FAST
    for k in range(20):
        sm.tick(now=1.0 + 0.01 * k, frame_age=0.01)
        assert sm.state == safety.ALIGN_FAST, "只有 align_done() 才能进 FOLLOWING"


def test_user_stop_from_any_state():
    for prep in (None, "align", "following", "holding"):
        sm = _sm()
        sm.start(now=0.0)
        if prep == "align":
            pass
        else:
            sm.align_done(now=1.0)
        if prep in ("following", "holding"):
            if prep == "holding":
                sm.tick(now=2.0, frame_age=0.5)
        sm.user_stop(now=9.0)
        assert sm.state == safety.IDLE, f"prep={prep}"
        assert sm.wants_stop_command() is True


def test_state_missing_q_blocks_dispatch():
    """拿不到实测 q 就不许下发（spec §5.1 第 4 点）—— 不许拿 0 去猜 err。"""
    sm = _sm()
    sm.start(now=0.0)
    sm.align_done(now=1.0)
    assert sm.may_dispatch(now=1.0, have_slave_q=True) is True
    assert sm.may_dispatch(now=1.0, have_slave_q=False) is False


# ── 评审订正（2026-09-28）：四条会造成具体臂行为错误的缺陷 ──────────────────

def test_align_failed_does_not_swallow_stop_command():
    """⛔ 真缺陷：`align_failed()` 早先会把已排队的收尾命令清掉。

    路径：「用户点停止（已排队收尾 `movej`）→ 恰好那条 `movej` 失败」。
    早先版本的结果：收尾命令被吞、状态留在 `ALIGN_FAST` 而不是 `IDLE`。
    """
    sm = _sm()
    sm.start(now=0.0)
    sm.user_stop(now=1.0)
    assert sm.state == safety.IDLE
    assert sm.wants_stop_command() is True
    sm.align_failed(now=1.1)                       # 从 IDLE 调（非 ALIGN_FAST）
    assert sm.state == safety.IDLE, "有状态守卫 ⇒ 不该被拽去 ALIGN_FAST"
    assert sm.wants_stop_command() is True, "⛔ 收尾命令绝不能被吞掉"


def test_align_failed_guarded_to_align_fast():
    """`align_failed()` 只在 `ALIGN_FAST` 生效（与 `align_done()` 对称）。"""
    sm = _sm()
    sm.start(now=0.0)
    sm.align_done(now=1.0)
    assert sm.state == safety.FOLLOWING
    sm.align_failed(now=2.0)
    assert sm.state == safety.FOLLOWING, "从 FOLLOWING 调不许把状态拽回对齐"


def test_align_failed_blocks_retry():
    """失败后不许重试 —— 否则 phase 2 的 `while wants_align_movej()` 会变成
    每 3 秒一次阻塞 `movej` 的忙重试环。"""
    sm = _sm()
    sm.start(now=0.0)
    assert sm.wants_align_movej() is True, "还没失败 ⇒ 该发对齐命令"
    sm.align_failed(now=1.0)
    assert sm.state == safety.ALIGN_FAST
    assert sm.wants_align_movej() is False, "失败后不许重试"
    sm.start(now=2.0)                              # 用户重新启动 ⇒ 允许再来
    assert sm.wants_align_movej() is True


def test_recover_needs_distinct_frames():
    """⛔ 真缺陷：恢复必须数**不同的帧**，不是 tick 数。

    watchdog(200ms) 比 tick(10ms) 长 20 倍 ⇒ 同一帧能被连续 20 拍判为"新鲜"，
    早先版本下 `recover_frames=5` 只要一帧 50ms 就满足，"防抖"根本不存在：
    链路每 250ms 一帧时会以约 4 次/秒的节奏反复触发**阻塞式**重对齐。
    """
    sm = _sm()
    sm.start(now=0.0)
    sm.align_done(now=1.0)
    sm.tick(now=1.5, frame_age=0.5)                # 断链 ⇒ HOLDING
    assert sm.state == safety.HOLDING
    for k in range(20):                            # 20 拍都是**同一帧** seq=7
        sm.tick(now=2.0 + 0.01 * k, frame_age=0.01, frame_id=7)
    assert sm.state == safety.HOLDING, "同一帧重复 20 拍不许触发恢复"
    for k in range(5):                             # 5 个**不同帧**
        sm.tick(now=3.0 + 0.01 * k, frame_age=0.01, frame_id=100 + k)
    assert sm.state == safety.ALIGN_FAST, "5 个不同帧才恢复"


def test_tick_none_age_means_never_received():
    """`frame_age=None`（从未收到）必须判成不新鲜 —— 与 `LatestSlot.peek_age` 的契约一致。"""
    sm = _sm()
    sm.start(now=0.0)
    sm.align_done(now=1.0)
    assert sm.tick(now=1.1, frame_age=None) is safety.HOLDING


def test_wants_align_and_stop_movej_are_distinct():
    """对齐 `movej` 与收尾 `movej` 是**两条不同的命令**（目标与速度都不同），
    早先合成一个 `wants_movej()` 不提供安全性，反而招来"发错目标"。"""
    sm = _sm()
    sm.start(now=0.0)
    assert (sm.wants_align_movej(), sm.wants_stop_movej()) == (True, False)
    sm.align_done(now=1.0)
    assert (sm.wants_align_movej(), sm.wants_stop_movej()) == (False, False)
    sm.user_stop(now=2.0)
    assert (sm.wants_align_movej(), sm.wants_stop_movej()) == (False, True)


def test_non_finite_target_has_its_own_type():
    """⚠ 「本拍该跳过」与「你的接线坏了」必须能被 phase 2 分开 —— 否则一个真实的
    接线错误会被当成一次例行的"跳过这一帧"吞掉。"""
    lim = safety.read_limits_ok([-1.0] * 7, [1.0] * 7, 7)
    with pytest.raises(safety.NonFiniteTarget):
        safety.clamp_to_limits([0.0, float("nan")] + [0.0] * 5, lim)
    with pytest.raises(safety.NonFiniteTarget):
        safety.clamp_to_limits([0.0, float("inf")] + [0.0] * 5, lim)
    assert issubclass(safety.NonFiniteTarget, safety.LimitsError), "仍是它的子类，便于粗粒度捕获"
    # 长度不符是**配置/接线**错误，不是"本拍跳过"
    with pytest.raises(safety.LimitsError) as e:
        safety.clamp_to_limits([0.0] * 6, lim)
    assert not isinstance(e.value, safety.NonFiniteTarget)


def test_limits_reject_zero_width_in_the_type_itself():
    """零宽度不变量放在**类型**里，不只是工厂里（phase 2 会长期持有 `Limits` 对象）。"""
    with pytest.raises(safety.LimitsError, match="零宽度或反了"):
        safety.Limits(lo=(0.0, 1.0), hi=(0.0, 2.0))


def test_slew_clamps_out_of_range_initial_speed():
    """⛔ 真缺陷：`v = clamp(v, ±v_limit)` 这行**是承重的**，而它原先**没有判别力**。

    删掉它，全套 30 条仍绿 —— 因为当 `dq_cmd` 起点本来就合法时，加速限幅足以把 `v`
    留在范围内。只有**起点就越界**才露出来 —— 而这正是运行期把 `speed_limit_j`
    或 `kd_budget` 调小（spec §8 把它们作为界面字段）时的情形。

    实测（评审量化）：起点 `dq=3.0`、`speed_limit=0.57` 时，原版夹到 0.57；
    删掉该行后冲到 **2.76** ⇒ `kd·|dq| = 30.4 Nm` vs `tau_max = 21 Nm`（**145%**，预算是 30%）。
    """
    sp, ac, dt = 0.57, 14.0, 0.01
    q_cmd, dq_cmd = [0.0], [3.0]                     # 起点速度就越界
    for k in range(3):
        q_cmd, dq_cmd = safety.slew_target([10.0], q_cmd, dq_cmd, [sp], [ac], dt)
        assert abs(dq_cmd[0]) <= sp + 1e-12, (
            f"第 {k} 拍 |dq_cmd|={abs(dq_cmd[0])} 越过 speed_limit={sp} —— 限速钳位失效")
