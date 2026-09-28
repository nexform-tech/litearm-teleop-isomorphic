"""从臂伺服环 —— **逐字移植 litearm-server 的 `joint_follow`**。

⛔ 不是新设计。litearm-server 的遥操是**经过真机验证**的实现，用户裁决
「必须必须严格按照它的逻辑来」。本模块把下面三份源码的**算法与调用序**照搬过来：

- 控制律与循环：`pylitearm/control/joint_follow.py`（`JointFollowController`）
- 伺服环本体：`pylitearm/sdk/arm.py:2006+` 的 `joint_follow()`
- 参数真值：`pylitearm/config/litearm_balanced.yaml` 的 `joint_follow:` 段

控制律（照搬）::

    τ = K·(q_cmd − q) + B·(dq_cmd − dq) + G(q)

`q_cmd/dq_cmd` 是 `slew_target` 平滑后的指令；弹簧-阻尼项由**电机固件的 MIT 环**
执行，Python 侧只算重力前馈并下发。

## 接口适配（只有这些地方与 pylitearm 不同，逐一列出）

| pylitearm | 本仓 | 说明 |
| --- | --- | --- |
| `hw.send_mit(K, B, q, dq, tau)` | `arm.send_mit_all(q, dq, K, B, tau)` | ⚠ **参数顺序不同**（K/B 在前面 vs 在后面） |
| `self.dyn.gravity(q)` | `arm.model.get_gravity(q).value` | 本地 Pinocchio → **固件往返**，见下 ⚠ |
| `hw.read_q_dq()` | `st.q` / `st.dq`（`get_state`） | SDK 已有独立读线程，无需 `update_states()` |
| `hw.faulted()` | `st.faulted` / `st.joint_fault` | |
| `hw.assert_operational(factor=inf, skip_position=True)` | **丢弃** | 它那两个实参的意思就是「把位置与超速护栏都关掉」，**丢弃即等价** |
| `hw.emergency_hold_healthy()` | `hold_via_movej(arm)` | 本 SDK 无 MIT 层"持位健康轴"原语，用 `movej(实测位姿)` 受控接管代替 |

⚠⚠ **`G(q)` 的来源不同，这是唯一有性能后果的偏离**：pylitearm 在**本地**用 Pinocchio
算（亚毫秒）；本仓走 `model.get_gravity` 的**串口往返**。伺服环是定频的，所以
**环频上限由它决定**，不能照抄 200 Hz 就以为没事 —— 必须实测（见 `measure_gravity_cost`）。
"""
from __future__ import annotations

import logging
import time
from typing import Callable, List, Optional, Sequence, Tuple

from .safety import slew_target
from .wall import JointLimitWall
from .wire import N_JOINTS

log = logging.getLogger("liteteleop.servo")

__all__ = ["DEFAULT_K", "DEFAULT_B", "DEFAULT_SPEED_LIMIT", "DEFAULT_ACCEL_LIMIT",
           "DEFAULT_ENGAGE_SEC", "JointFollowController", "follow", "hold_via_movej",
           "measure_gravity_cost"]

# ── 参数真值 ────────────────────────────────────────────────────────────────
# ⛔ 逐个照抄 `pylitearm/config/litearm_balanced.yaml` 的 `joint_follow:` 段
#    （那是 litearm-server 真机验证过的那一套），**没有就地调参**。
DEFAULT_K = [25.0] * N_JOINTS
DEFAULT_B = [0.5] * N_JOINTS
DEFAULT_SPEED_LIMIT = [2.8, 3.4, 5.0, 5.0, 10.0, 8.0, 13.0]
DEFAULT_ACCEL_LIMIT = [14.0, 22.0, 24.0, 24.0, 45.0, 40.0, 60.0]
DEFAULT_ENGAGE_SEC = 0.3

#: **从臂伺服环**频率 = pylitearm 的 `arm._hz` = `cfg["transport"]["control_loop_hz"]`
#: （`sdk/arm.py:652` 读它，`litearm_balanced.yaml:32` = **250**）。
#: ⚠⚠ **与主臂的 `pub_hz = 200` 是两个不同的数** —— 那个是 `TeleopManager.__init__`
#: 的发布率，管的是"主臂多久发一帧"。别混（我混过一次）。
#: ⚠ 但见 §9.3：250 Hz 的周期只有 4 ms，而本仓的 `G(q)` 走串口往返 **3.31 ms**
#: （pylitearm 是本地 Pinocchio，亚毫秒）⇒ **250 Hz 在这套移植上跑不动**。
#: 这条是"照抄不来"的地方，**环频必须按实测 `G(q)` 代价定**，见 `measure_gravity_cost`。
DEFAULT_HZ = 250.0

#: MIT 帧的电机硬性范围（达妙）：超范围会被电调饱和钳掉。
MIT_KP_MAX = 500.0
MIT_KD_MAX = 5.0


def _clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else (hi if v > hi else v)


class JointFollowController:
    """`τ = K·(q_cmd−q) + B·(dq_cmd−dq) + G(q)`（照搬 `joint_follow.py`）。

    调用序（照搬）：`prime` → `engage` → `start` → 每拍 `set_target` + `step`。
    """

    def __init__(self, K, B, speed_limit, accel_limit, tau_max, hz,
                 wall: Optional[JointLimitWall] = None, wall_fw_kd: float = 0.0):
        self.K = list(K)
        self.B = list(B)
        self.speed_limit = list(speed_limit)
        self.accel_limit = list(accel_limit)
        self.tau_max = list(tau_max)
        self.hz = float(hz)
        self.wall = wall
        self.wall_fw_kd = float(wall_fw_kd)
        # 当前指令位置和速度（经限幅平滑后）
        self.q_cmd = [0.0] * N_JOINTS
        self.dq_cmd = [0.0] * N_JOINTS
        # 外部目标（原始，未经限幅）
        self.q_target = [0.0] * N_JOINTS
        self.dq_target = [0.0] * N_JOINTS
        # MIT 范围告警（照搬 `from_params` 的那两条 log.warning）
        for i in range(N_JOINTS):
            if not 0.0 <= self.K[i] <= MIT_KP_MAX:
                log.warning("K[J%d]=%s 超出 MIT kp 范围[0,%.0f]，电机会钳到边界",
                            i + 1, self.K[i], MIT_KP_MAX)
            if not 0.0 <= self.B[i] <= MIT_KD_MAX:
                log.warning("B[J%d]=%s 超出 MIT kd 范围[0,%.0f]，电机会钳到边界",
                            i + 1, self.B[i], MIT_KD_MAX)

    # ── 反馈与模型 ────────────────────────────────────────────────────────
    @staticmethod
    def _read_q_dq(arm, refresh: bool = True) -> Tuple[List[float], List[float]]:
        st = arm.get_state(refresh=refresh).value
        if st is None:
            raise RuntimeError("状态帧取不到（链路静默）")
        return list(st.q), list(st.dq)

    @staticmethod
    def _gravity(arm, q: Sequence[float]) -> List[float]:
        """`self.dyn.gravity(q)` 的对应物 —— 固件算，一次串口往返。"""
        msg = arm.model.get_gravity(list(q))
        g = msg.value
        if g is None:
            raise RuntimeError("get_gravity 无应答")
        return [float(v) for v in g]

    # ── 生命周期（照搬）──────────────────────────────────────────────────
    def prime(self, arm) -> None:
        """进入瞬间叠加重力补偿 + 当前位置刚度，防止松手前下坠（照搬）。"""
        q0, dq0 = self._read_q_dq(arm, refresh=True)
        self.q_target = list(q0)
        self.dq_target = list(dq0)
        self.q_cmd = list(q0)
        self.dq_cmd = list(dq0)
        tau_g = self._gravity(arm, q0)
        arm.send_mit_all(self.q_target, self.dq_target, self.K, self.B, tau_g)

    def engage(self, arm, engage_sec: float = DEFAULT_ENGAGE_SEC,
               engage_kp: float = 15.0, engage_kd: float = 0.8,
               should_stop: Callable[[], bool] = lambda: False) -> None:
        """先以位置刚度托住当前姿态 `engage_sec` 秒，减少接管冲击（照搬）。"""
        if engage_sec <= 1e-6:
            return
        q_ref, _ = self._read_q_dq(arm, refresh=True)
        q_ref = list(q_ref)
        kp_vec = [engage_kp] * N_JOINTS
        kd_vec = [engage_kd] * N_JOINTS
        dt = 1.0 / max(self.hz, 1.0)
        t_end = time.monotonic() + engage_sec
        while time.monotonic() < t_end and not should_stop():
            q_meas, _ = self._read_q_dq(arm, refresh=True)
            tau_g = self._gravity(arm, q_meas)
            tau = [_clamp(tau_g[i], -self.tau_max[i], self.tau_max[i])
                   for i in range(N_JOINTS)]
            arm.send_mit_all(q_ref, [0.0] * N_JOINTS, kp_vec, kd_vec, tau)
            time.sleep(dt)

    def start(self, arm) -> None:
        """`engage` 结束后，初始化目标为当前位置（照搬）。"""
        q, dq = self._read_q_dq(arm, refresh=True)
        self.q_cmd = list(q)
        self.dq_cmd = list(dq)
        self.q_target = list(q)
        self.dq_target = list(dq)

    def set_target(self, q_target, dq_target=None) -> None:
        """运行时更新外部目标（照搬；`dq_target` 默认全零）。"""
        if len(q_target) != N_JOINTS:
            raise ValueError(f"q_target 必须是 {N_JOINTS} 元素列表")
        if dq_target is None:
            dq_target = [0.0] * N_JOINTS
        if len(dq_target) != N_JOINTS:
            raise ValueError(f"dq_target 必须是 {N_JOINTS} 元素列表")
        self.q_target = list(q_target)
        self.dq_target = list(dq_target)

    def compute_tau_ff(self, q, dq=None) -> List[float]:
        """重力前馈 `τ_ff = G(q) + wall(q)`，钳到 `tau_max`（照搬）。"""
        raise NotImplementedError("需传入 arm 以取 G(q)，见 compute_tau_ff_arm()")

    def compute_tau_ff_arm(self, arm, q, dq=None) -> List[float]:
        tau_g = self._gravity(arm, q)
        tau_wall = (self.wall.tau(q, dq) if self.wall is not None
                    else [0.0] * N_JOINTS)
        return [_clamp(tau_g[i] + tau_wall[i], -self.tau_max[i], self.tau_max[i])
                for i in range(N_JOINTS)]

    def step(self, arm, dt: Optional[float] = None):
        """单步：读反馈 → 检故障 → 限幅平滑 → 重力前馈 → 下发 MIT（照搬）。

        返回 `(q, dq, tau_est, bad)`；`bad` 非空表示故障（此时 `tau_est is None`，
        **未下发**）。
        """
        if dt is None:
            dt = 1.0 / max(self.hz, 1.0)

        st = arm.get_state(refresh=False).value
        if st is None:
            return None, None, None, ["状态帧取不到"]
        q, dq = list(st.q), list(st.dq)
        bad = []
        if st.faulted:
            bad.append("faulted")
        if st.joint_fault:
            bad.append(f"joint_fault={st.joint_fault:#x}")
        if bad:
            return q, dq, None, bad

        # 限幅平滑：将外部目标平滑到可执行的指令（照搬）
        self.q_cmd, self.dq_cmd = slew_target(
            self.q_target, self.q_cmd, self.dq_cmd,
            self.speed_limit, self.accel_limit, dt)

        tau_ff = self.compute_tau_ff_arm(arm, q, dq)

        # 估算总力矩（供日志/回调；照搬）
        tau_est = [self.K[i] * (self.q_cmd[i] - q[i])
                   + self.B[i] * (self.dq_cmd[i] - dq[i])
                   + tau_ff[i]
                   for i in range(N_JOINTS)]

        # 下发 MIT 帧：固件 PD 完成弹簧-阻尼，tau_ff 做重力前馈（照搬）
        # 墙区叠加固件阻尼（kHz 环无控制延迟，治墙区振荡）
        kd_vec = list(self.B)
        if self.wall is not None and self.wall_fw_kd > 0.0:
            zone = self.wall.wall_zone_mask(q)
            if any(zone):
                for i in range(N_JOINTS):
                    if zone[i]:
                        kd_vec[i] = min(kd_vec[i] + self.wall_fw_kd, MIT_KD_MAX)
        arm.send_mit_all(self.q_cmd, self.dq_cmd, self.K, kd_vec, tau_ff)

        return q, dq, tau_est, None


def hold_via_movej(arm) -> None:
    """受控接管（`hw.emergency_hold_healthy()` 的对应物）。

    本 SDK 没有 MIT 层"只持健康轴"的原语；用 `movej(实测位姿)` 让固件的
    S 曲线 + `ht_on` 接管 —— 这是 spec §5.3 定的收尾方式。
    ⛔ **绝不 `disable()`**：失能会让臂在自重下自由落体。
    """
    st = arm.get_state(refresh=True).value
    if st is None:
        raise RuntimeError("收尾取不到状态帧")
    arm.movej(list(st.q), speed=0.3)


def measure_gravity_cost(arm, n: int = 50) -> float:
    """实测 `G(q)` 一次往返的耗时（ms）—— 环频上限由它决定（纯读，不动臂）。"""
    q = list(arm.get_state(refresh=True).value.q)
    t0 = time.monotonic()
    for _ in range(n):
        arm.model.get_gravity(q)
    return (time.monotonic() - t0) / n * 1000.0


def follow(arm, target_provider: Callable[[], Optional[Tuple[Sequence[float],
                                                              Optional[Sequence[float]]]]],
           K=None, B=None, speed_limit=None, accel_limit=None,
           engage_sec: float = DEFAULT_ENGAGE_SEC, hz: float = DEFAULT_HZ,
           tau_max=None, wall: Optional[JointLimitWall] = None,
           wall_fw_kd: float = 0.0,
           should_stop: Callable[[], bool] = lambda: False,
           duration_s: Optional[float] = None) -> bool:
    """从臂跟随环 —— **照搬 `pylitearm/sdk/arm.py:joint_follow()` 的循环体**。

    `target_provider()` 返回 `(q_target, dq_target)`；返回 `None` 时**保持
    上一拍的 `q_cmd/dq_cmd` 不动**（照搬那支 `q_target, dq_target = jfc.q_cmd,
    jfc.dq_cmd`）—— 这是"首帧到达前原地不动"的落点。

    `should_stop()` 为真 / `duration_s` 到了 ⇒ 返回。故障时**先受控接管再抛**。
    """
    jfc = JointFollowController(
        K or DEFAULT_K, B or DEFAULT_B,
        speed_limit or DEFAULT_SPEED_LIMIT, accel_limit or DEFAULT_ACCEL_LIMIT,
        tau_max if tau_max is not None else _tau_max_from(arm),
        hz, wall=wall, wall_fw_kd=wall_fw_kd)

    jfc.prime(arm)
    jfc.engage(arm, engage_sec=engage_sec, should_stop=should_stop)
    jfc.start(arm)

    dt_nom = 1.0 / max(hz, 1.0)
    base = time.monotonic()
    next_tick = base + dt_nom

    while not should_stop():
        if duration_s is not None and time.monotonic() - base >= duration_s:
            break
        try:
            result = target_provider()
            if result is None:
                q_target, dq_target = jfc.q_cmd, jfc.dq_cmd
            elif isinstance(result, tuple) and len(result) == 2:
                q_target, dq_target = result
                if dq_target is None:
                    dq_target = [0.0] * N_JOINTS
            else:
                q_target = result
                dq_target = [0.0] * N_JOINTS
        except Exception as e:                       # noqa: BLE001 - 照搬：先持位再上抛
            log.error("target_provider 异常: %s", e)
            hold_via_movej(arm)
            return False

        jfc.set_target(q_target, dq_target)
        try:
            _q, _dq, _tau, bad = jfc.step(arm, dt=dt_nom)
        except Exception:
            log.exception("jfc.step 故障，受控接管")
            hold_via_movej(arm)
            raise
        if bad:
            log.error("电机故障 %s，退出跟随；受控接管", bad)
            hold_via_movej(arm)
            raise RuntimeError(f"关节跟随电机故障: {bad}")

        r = next_tick - time.monotonic()
        if r > 0:
            time.sleep(r)
        next_tick += dt_nom
        if next_tick < time.monotonic():
            next_tick = time.monotonic() + dt_nom
    return True


def _tau_max_from(arm) -> List[float]:
    """`tau_max` 来自**固件**（`all_joint_params()` 的 `tau_max`）。

    pylitearm 从 `cfg.motion.tau_max` 读；本仓从固件读 —— 更权威（同一个值
    也是固件真正用来钳 `τ` 的那个）。
    """
    return [float(p.tau_max) for p in arm.params.all_joint_params()]
