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



def test_clamp_uses_range_not_float_equality():
    """⚠ 判据用区间比较，不用浮点 `!=`（spec §5.1 的 `saturated` 定义）。"""
    lim = safety.read_limits_ok([0.0] * 7, [1.0] * 7, 7)
    _, sat = safety.clamp_to_limits([0.0, 1.0, 0.5, 0.5, 0.5, 0.5, 0.5], lim)
    assert sat == [False, False, False, False, False, False, False], "边界值不应算被钳"



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
