"""安全层 —— 纯逻辑：软限位钳位 + 跟随平滑（`slew_target`）。

设计依据 spec §5.1 / §7.1，**逻辑照 litearm-server**（用户裁决「必须严格按照它来写」）。

本模块**不 import litearm**（`read_safe_limits` 只吃一个 duck-typed `arm`），
所以纯函数部分能在无臂机器上单测。

## 本版删掉了什么（以及为什么）

- `speed_limit_from_kd` / `DEFAULT_KD_BUDGET` —— 那是**为 `move_js` 造出来的**。
  `move_js` 的刚度/阻尼由固件定死，所以只能反推一个"安全速度"；而 litearm-server
  **不用 `move_js`**：它用 `joint_follow` → `send_mit`，**K/B 随帧下发**。
  真实的速度限幅是 `joint_follow.speed_limit`（真机验证过的
  `[2.8, 3.4, 5.0, 5.0, 10.0, 8.0, 13.0]`），不是推导出来的。
- `saturate_dq` —— 同上：它是"`dq` 是速度前馈"那个语义下的产物。
- `IDLE/ALIGN_FAST/FOLLOWING/HOLDING` + `TeleopState` 状态机 —— **litearm-server 没有
  状态机**：它的 `active` 由 `TeleopManager.is_running` **派生**，watchdog 超时调
  `request_stop()` 让控制环退出，`active` 自动变假。自造一台状态机是"乱来"。

## 保留的那一条是硬要求

⚠ `slew_target` **逐字移植自** `pylitearm/src/pylitearm/control/joint_follow.py:45-100`
（用户在 spec 里裁决过：跟随手感与制动距离**只有**这一个来源，"照抄"才有既有真机
行为的可预期性；任何"改进"都必须先报用户裁决）。
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

__all__ = [
    "LimitsError", "NonFiniteTarget", "Limits",
    "read_limits_ok", "clamp_to_limits", "read_safe_limits", "slew_target",
    "DEFAULT_LIMIT_MARGIN",
]

#: 软限位内缩量。取自 litearm-server 的 `safety.joint_limit_margin_rad` 默认值。
DEFAULT_LIMIT_MARGIN = 0.01


class LimitsError(ValueError):
    """限位配置不合法 —— **一律拒启动**，绝不静默退化（spec §7.1）。"""


class NonFiniteTarget(LimitsError):
    """本拍的目标值里出现 NaN/Inf ⇒ 本拍不许下发（spec §7.1）。"""


@dataclass(frozen=True)
class Limits:
    """一组关节软限位（rad）。`lo`/`hi` 逐轴成对，长度 = 轴数。"""

    lo: Tuple[float, ...]
    hi: Tuple[float, ...]
    n: Optional[int] = None

    def __post_init__(self) -> None:
        object.__setattr__(self, "lo", tuple(float(v) for v in self.lo))
        object.__setattr__(self, "hi", tuple(float(v) for v in self.hi))
        if self.n is None:
            object.__setattr__(self, "n", len(self.lo))
        if len(self.lo) != len(self.hi) or len(self.lo) != self.n:
            raise LimitsError(
                f"长度不符: lo={len(self.lo)} hi={len(self.hi)} n={self.n}"
            )
        for i, (a, b) in enumerate(zip(self.lo, self.hi)):
            if not a < b:
                raise LimitsError(f"第 {i} 轴软限位不合法（零宽度或反了）: lo={a} hi={b}")


def _check_finite(vals: Sequence[float], what: str) -> None:
    for i, v in enumerate(vals):
        if not math.isfinite(v):
            raise LimitsError(f"非有限值: {what}[{i}] = {v}")


def read_limits_ok(lo: Sequence[float], hi: Sequence[float], n: int) -> Limits:
    """校验并固化一对软限位；任何一处不合法都**拒启动**（spec §7.1）。"""
    if n <= 0:
        raise LimitsError(f"轴数非法: n={n}")
    if len(lo) != n or len(hi) != n:
        raise LimitsError(f"长度不符: lo={len(lo)} hi={len(hi)} n={n}")
    _check_finite(lo, "lo")
    _check_finite(hi, "hi")
    for i, (a, b) in enumerate(zip(lo, hi)):
        if not a < b:
            raise LimitsError(f"第 {i} 轴上下界反了: lo={a} hi={b}")
    return Limits(lo=tuple(lo), hi=tuple(hi), n=n)


def clamp_to_limits(
    q: Sequence[float], limits: Limits
) -> Tuple[List[float], List[bool]]:
    """把 `q` 逐轴钳进软限位，返回 `(q_clamped, saturated)`。

    对照 litearm-server：它用 `np.clip(np.asarray(master_q), safe_lo, safe_hi)`。
    本条**语义等价**，另外把「哪些轴被钳了」显式返回（server 那边是用
    `np.where(master_q != clamped)` 现算）。
    """
    if len(q) != limits.n:
        raise LimitsError(f"长度不符: q={len(q)} 限位轴数={limits.n}")
    for i, v in enumerate(q):
        if not math.isfinite(v):
            raise NonFiniteTarget(f"q[{i}] 非有限值: {v}")
    out: List[float] = []
    sat: List[bool] = []
    for v, a, b in zip(q, limits.lo, limits.hi):
        if v < a:
            out.append(a)
            sat.append(True)
        elif v > b:
            out.append(b)
            sat.append(True)
        else:
            out.append(float(v))
            sat.append(False)
    return out, sat


def read_safe_limits(arm, margin: float = DEFAULT_LIMIT_MARGIN) -> Limits:
    """读**固件里的**软限位并内缩 `margin`。

    对应 litearm-server 的 `teleop_manager._read_safe_limits()` —— 它是从
    `arm.kin.q_min/q_max` 或配置 yaml 读的；本 SDK **没有 `arm.kin`**，
    改读 `arm.params.all_joint_params()` 的 `q_min`/`q_max`。

    ⚠ 那是**固件真正用来钳 `move_js`/`movej` 的那一对值**，比配置文件更权威。
    ⚠ 拿不到就**抛**，绝不退回 ±9 那种兜底哨兵 —— server 那边命中兜底会大声告警，
      本仓直接拒启动（spec §7.1：限位不可信 ⇒ 不启动）。
    """
    jp = arm.params.all_joint_params()
    if not jp:
        raise LimitsError("读不到关节参数（all_joint_params 为空）")
    lo = [float(p.q_min) + margin for p in jp]
    hi = [float(p.q_max) - margin for p in jp]
    return read_limits_ok(lo, hi, len(jp))


def slew_target(raw_target, q_cmd, dq_cmd, speed_limit, accel_limit, dt):
    """速度/加速度限制的目标位置平滑（梯形速度曲线）。

    ⚠ **逐字移植自 `pylitearm/src/pylitearm/control/joint_follow.py:45-100`**
    （唯一偏离：原版循环上界是模块常量 `N = 7`，这里取 `len(raw_target)`；
    7 轴输入下两者**完全等价**）。对每个关节：

    - 限制最大速度为 `speed_limit[i]`
    - 限制最大加速度为 `accel_limit[i]`
    - 当接近目标时自动减速（基于制动距离 `v²/(2a)`）
    - 落在死区（`|diff| < 1e-5` 且 `|v| < dv_max`）时吸附到目标并停住

    ⚠ **移植已知边界（照抄，不修）**：`diff` 恰为 0 或落在 `±1e-5` 死区内、
    而 `|v| ≥ dv_max` 时，`math.copysign(v_limit, diff)` 在 `diff == 0` 会返回
    `+v_limit` ⇒ 该轴可能继续正向加速而非停住（原版同样如此）。实践中 `diff`
    极少恰为 0，且 `abs(diff) < 1e-5 and abs(v) < dv_max` 已挡掉绝大多数情形。
    """
    dt = max(dt, 1e-4)
    for i in range(len(raw_target)):
        v_limit = max(1e-4, speed_limit[i])
        a_limit = max(1e-4, accel_limit[i])
        dv_max = a_limit * dt

        diff = raw_target[i] - q_cmd[i]
        v = dq_cmd[i]

        # 已到达目标
        if abs(diff) < 1e-5 and abs(v) < dv_max:
            q_cmd[i] = raw_target[i]
            dq_cmd[i] = 0.0
            continue

        # 期望速度（考虑制动距离）
        stopping_dist = (v * v) / (2.0 * a_limit) if a_limit > 0.0 else 0.0
        moving_toward = diff * v > 0.0
        if moving_toward and abs(diff) <= stopping_dist:
            desired_v = 0.0  # 开始减速
        else:
            desired_v = math.copysign(v_limit, diff)

        # 限加速度
        v += max(-dv_max, min(dv_max, desired_v - v))
        v = max(-v_limit, min(v_limit, v))

        # 更新位置
        step = v * dt
        if diff * step > 0.0 and abs(step) >= abs(diff):
            q_cmd[i] = raw_target[i]
            dq_cmd[i] = 0.0
        else:
            q_cmd[i] += step
            dq_cmd[i] = v

    return q_cmd, dq_cmd
