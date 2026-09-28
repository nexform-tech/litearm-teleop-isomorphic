"""关节限位虚拟墙 —— **逐字移植自 `pylitearm/control/joint_limit_wall.py`**。

⛔ 不是新设计。litearm-server 的遥操（经真机验证）在手动模式下开这道墙，
用户裁决「必须严格按照 litearm-server 来写」。算法、符号约定、防抖设计**全部照搬**，
只改两处**接口适配**（下面标了 `[适配]`）：

- `[适配]` `N`（pylitearm 的全局常量）→ 本仓的 `N_JOINTS`（= 7，见 `wire.py`）
- `[适配]` `from_params(cfg)`（读 pylitearm 的 yaml）→ `from_limits(q_min, q_max, w)`
  —— 本仓的软限位来自**固件**（`arm.params.all_joint_params()` 的 `q_min/q_max`），
  那比配置文件更权威（`teleop_manager._read_safe_limits` 的注释也要求"来自权威来源"）。

力矩方向约定（照搬）：排斥力矩始终把关节往【限位内】推。
接近 `q_max` ⇒ `τ` 为负；接近 `q_min` ⇒ `τ` 为正。
幅值 = `min(stiffness·d + damping·|dq|, tau_max[i])`，`d` = 进入墙区的深度。
"""
from __future__ import annotations

import time
from typing import List, Optional, Sequence

from .wire import N_JOINTS

__all__ = ["JointLimitWall"]


class JointLimitWall:
    """关节限位虚拟墙。

    `[适配]` 原版 `from_params` 从 `motors.*.limits` 读软限位；本仓改为
    `from_limits(...)`，软限位由调用方从**固件**读出来传进来。
    """

    def __init__(self, q_min, q_max, margin_rad, stiffness, damping, tau_max,
                 slew_rate=None):
        self.q_min = [float(v) for v in q_min]
        self.q_max = [float(v) for v in q_max]
        self.margin = float(margin_rad)
        # stiffness/damping 支持标量或 7 元素列表（腕部惯量小，需更低刚度才稳）
        self.stiffness = self._to_list(stiffness, "stiffness")
        self.damping = self._to_list(damping, "damping")
        self.tau_max = [float(v) for v in tau_max]
        # 力矩变化率限幅（Nm/s）。None=关闭（直接构造的默认；from_limits 默认 20）
        self.slew_rate = None if slew_rate is None else float(slew_rate)
        self._prev_tau = [0.0] * N_JOINTS
        self._last_t: Optional[float] = None
        if (len(self.q_min) != N_JOINTS or len(self.q_max) != N_JOINTS
                or len(self.tau_max) != N_JOINTS):
            raise ValueError(f"q_min/q_max/tau_max 必须是 {N_JOINTS} 元素列表")

    @staticmethod
    def _to_list(value, name):
        if isinstance(value, (int, float)):
            return [float(value)] * N_JOINTS
        values = [float(v) for v in value]
        if len(values) != N_JOINTS:
            raise ValueError(f"{name} 必须是标量或 {N_JOINTS} 元素列表")
        return values

    @classmethod
    def from_limits(cls, q_min: Sequence[float], q_max: Sequence[float],
                    w: Optional[dict] = None,
                    tau_max: Optional[Sequence[float]] = None) -> "JointLimitWall":
        """`[适配]` 用**固件读回的软限位** + 墙参数段构造。

        `w` 的默认值取自 litearm-server 验证过的配置段
        （`pylitearm/config/litearm_balanced.yaml` 的 `joint_limit_wall:`）——
        **逐值照抄**，没有就地调参。
        """
        w = w or {}
        margin = float(w.get("margin_rad", 0.02))
        stiffness = w.get("stiffness", [200.0, 200.0, 150.0, 150.0, 100.0, 60.0, 20.0])
        damping = w.get("damping", [2.0, 2.0, 2.0, 2.0, 2.0, 3.0, 1.0])
        slew = w.get("slew_rate", 0.0)          # 0 = 关闭（balanced.yaml 就是 0）
        slew_rate = None if slew in (None, 0, 0.0) else float(slew)
        tm = tau_max if tau_max is not None else w.get(
            "tau_max", [19.5, 19.5, 5.25, 5.25, 1.0, 1.0, 1.0])
        return cls(q_min, q_max, margin, stiffness, damping, tm, slew_rate)

    def wall_zone_mask(self, q) -> List[bool]:
        """逐关节墙区判定（含 margin 提前区），返回 bool 列表（照搬）。"""
        mask = [False] * N_JOINTS
        for i in range(N_JOINTS):
            if q[i] > self.q_max[i] - self.margin or q[i] < self.q_min[i] + self.margin:
                mask[i] = True
        return mask

    def in_wall_zone(self, q) -> bool:
        """任一关节进入墙区（含 margin 提前区）时返回 True（照搬）。"""
        return any(self.wall_zone_mask(q))

    def tau(self, q, dq=None):
        """限位墙排斥力矩 [N]，含力矩变化率限幅（照搬）。"""
        dq = dq if dq is not None else [0.0] * N_JOINTS
        out = [0.0] * N_JOINTS
        for i in range(N_JOINTS):
            d = 0.0
            sign = 0.0
            if q[i] > self.q_max[i] - self.margin:
                d = q[i] - (self.q_max[i] - self.margin)
                sign = -1.0
            elif q[i] < self.q_min[i] + self.margin:
                d = (self.q_min[i] + self.margin) - q[i]
                sign = +1.0
            if sign == 0.0:
                continue
            spring = self.stiffness[i] * d
            # 阻尼永远阻碍运动：深入墙内时吸收冲击，回退时削减出墙力。
            # 出墙速度过快时墙力不为负（墙绝不把关节往墙内拉），钳到 0。
            damper = -self.damping[i] * dq[i] * sign
            out[i] = sign * min(max(spring + damper, 0.0), self.tau_max[i])

        # 力矩变化率限幅：平滑墙力突跳（进入/退出墙区的冲击激励是抖动主因）
        if self.slew_rate is not None:
            now = time.monotonic()
            dt = 0.0 if self._last_t is None else min(max(now - self._last_t, 1e-4), 0.1)
            if self._last_t is not None:
                max_step = self.slew_rate * dt
                for i in range(N_JOINTS):
                    out[i] = max(min(out[i], self._prev_tau[i] + max_step),
                                 self._prev_tau[i] - max_step)
            self._last_t = now
        self._prev_tau = list(out)
        return out
