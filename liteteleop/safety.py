"""安全层（一/二）—— 纯逻辑：跟随平滑 + 软限位闸门 + kd 预算闸门。

设计依据 spec §5.1（跟随律与限幅）与 §7.1（限位不合法就**拒启动**，不静默退化）。

本模块**不 import litearm / pylitearm，不碰硬件**：它是可以在无臂机器上单测的纯函数层。
状态机与 watchdog 是第二部分（见 spec §5.2），本文件只放第一步的纯逻辑。

⚠ `slew_target` 是**从既有验证实现逐字移植**的（`pylitearm/src/pylitearm/control/joint_follow.py:45-88`），
⛔ **不是**重新设计。理由：跟随手感与制动距离是**整条从臂跟随行为**的唯一来源，
"照抄"才有既有真机行为的可预期性；任何"改进"都必须先报用户裁决（spec §11 S3）。
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

__all__ = [
    "DEFAULT_KD_BUDGET", "LimitsError", "Limits",
    "read_limits_ok", "clamp_to_limits", "saturate_dq", "speed_limit_from_kd",
    "slew_target",
    "IDLE", "ALIGN_FAST", "FOLLOWING", "HOLDING", "TeleopState",
]

#: 速度上限的默认 kd 预算：`speed_limit_i = kd_budget * tau_max_i / kd_i`（spec §5.1）。
#: 含义是「按 kd 折算出来的指令速度所对应的力矩不超过 tau_max 的 30%」。
DEFAULT_KD_BUDGET = 0.30


class LimitsError(ValueError):
    """限位/预算配置不合法 —— **一律拒启动**，绝不静默退化（spec §7.1）。"""


class NonFiniteTarget(LimitsError):
    """**本拍的目标值**里出现 NaN/Inf ⇒ 本拍不许下发（spec §7.1）。

    与 `LimitsError` 分开是刻意的：两者在 phase 2 的处置**完全不同** ——
    `LimitsError` 是"你的接线/配置坏了"（该停、该报），而本异常是"本拍跳过、
    计数、并在连续若干拍后升级 `HOLDING`"（该继续跑）。合成一个类型的话，
    一个真实的接线错误会被当成一次例行的"跳过这一帧"吞掉。
    """


@dataclass(frozen=True)
class Limits:
    """一组关节软限位（rad）。`lo`/`hi` 逐轴成对，长度 = 轴数。

    `n` 可以省略（默认 = `len(lo)`）；显式给出时必须与 `lo`/`hi` 自洽。
    """

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
            # ⚠ 不变量放在**类型**里，不只是工厂里：`Limits` 是 phase 2 会长期持有的对象，
            # 零宽度轴（`lo == hi`）意味着该关节被锁死，与 spec §7.1 的"拒启动"同性质。
            if not a < b:
                raise LimitsError(f"第 {i} 轴软限位不合法（零宽度或反了）: lo={a} hi={b}")


def _check_finite(vals: Sequence[float], what: str) -> None:
    for i, v in enumerate(vals):
        if not math.isfinite(v):
            raise LimitsError(f"非有限值: {what}[{i}] = {v}")


def read_limits_ok(lo: Sequence[float], hi: Sequence[float], n: int) -> Limits:
    """校验并固化一对软限位；任何一处不合法都**拒启动**（spec §7.1）。

    依次判：①长度（两数组各自 = `n`）②有限 ③`lo[i] < hi[i]`（**相等也算反了**：
    零宽度的轴意味着该关节被锁死，是配置错误而不是"恰好不动"）。
    """
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

    ⚠ 是否"被钳"的判据是**区间比较**（`q < lo` 或 `q > hi`），**不是**浮点 `!=`
    （spec §5.1）：恰在边界上的值**不算**被钳 —— 它数值上并未越界，
    把它判成饱和会平白把那个轴的 `dq` 打成 0（见 `saturate_dq`），
    即"边界姿态被误判为顶住限位"。

    ⚠ 非有限值**拒算**（`NonFiniteTarget`）而不是放行：NaN 与任何边界比较都是 False，
    照原样传下去会**悄悄**把一个 NaN 目标送到从臂 —— 这是静默失败，不是限位。
    ⚠ 但"拒算 ⇒ 本拍不下发"**不是完整的契约**：跳过下发也跳过了 `watchdog_kick`，
    固件 100 ms 后 fail-soft（0.6× 刚度 + τ=0）⇒ 臂**缓慢下垂**。phase 2 必须
    连续若干拍后升级 `HOLDING`（见 spec §7.1 那一行）。
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


def saturate_dq(dq: Sequence[float], saturated: Sequence[bool]) -> List[float]:
    """把被钳轴的 `dq` 置 0 —— spec §5.1 那条不变量的**具名载体**。

    被钳的轴必须 `dq = 0`（否则 `kd·dq` 会持续把限位顶住：位置钳住了，
    速度前馈还在推，力矩一直压在限位上）。没被钳的轴**一个数都不许动**。
    """
    if len(dq) != len(saturated):
        raise LimitsError(f"长度不符: dq={len(dq)} saturated={len(saturated)}")
    return [0.0 if s else float(v) for v, s in zip(dq, saturated)]


def speed_limit_from_kd(
    kd: Sequence[float],
    tau_max: Sequence[float],
    kd_budget: float = DEFAULT_KD_BUDGET,
) -> List[float]:
    """按 kd 预算把力矩上限折算成速度上限（spec §5.1）::

        speed_limit_i = kd_budget * tau_max_i / kd_i     (rad/s)

    含义：速度跟随里 `tau ≈ kd·dq`，让 `kd·speed_limit ≤ kd_budget·tau_max`
    ⇒ 纯阻尼项就把力矩预算吃掉 `kd_budget` 那份，剩下的留给刚度与重力前馈。

    ⛔ J4 通常是**全局最紧的轴**（`tau_max/kd` 比最小）⇒ 它决定了"高速跟随"
    到底能有多快；这条闸门就是 spec 里"高速跟随时 J4 会不会顶住"的落点。

    任何不合法（长度/有限/kd≤0/tau_max≤0/预算越界）都 `LimitsError` 拒启动。
    """
    n = len(kd)
    if len(tau_max) != n:
        raise LimitsError(f"长度不符: kd={n} tau_max={len(tau_max)}")
    if n == 0:
        raise LimitsError("长度不符: 轴数 0")
    if not 0.0 < kd_budget <= 1.0:
        raise LimitsError(f"kd_budget 越界: {kd_budget} 不在 (0, 1] 内")
    _check_finite(kd, "kd")
    _check_finite(tau_max, "tau_max")
    out: List[float] = []
    for i in range(n):
        if kd[i] <= 0.0 or tau_max[i] <= 0.0:
            raise LimitsError(
                f"第 {i} 轴 kd/tau_max 非法: kd={kd[i]} tau_max={tau_max[i]}（须 > 0）"
            )
        out.append(kd_budget * tau_max[i] / kd[i])
    return out


def slew_target(raw_target, q_cmd, dq_cmd, speed_limit, accel_limit, dt):
    """速度/加速度限制的目标位置平滑（梯形速度曲线）。

    ⚠ **逐字移植自 `pylitearm/src/pylitearm/control/joint_follow.py:45-88`**（唯一偏离：原版循环
    上界是模块常量 `N = 7`，这里取 `len(raw_target)`；7 轴输入下两者**完全等价**）。
    对每个关节：

    - 限制最大速度为 `speed_limit[i]`
    - 限制最大加速度为 `accel_limit[i]`
    - 当接近目标时自动减速（基于制动距离 `v²/(2a)`）
    - 落在死区（`|diff| < 1e-5` 且 `|v| < dv_max`）时吸附到目标并停住

    Args:
        raw_target: 原始目标位置 [N] (rad)
        q_cmd: 当前指令位置 [N] (rad)，**会被就地修改**
        dq_cmd: 当前指令速度 [N] (rad/s)，**会被就地修改**
        speed_limit: 每关节最大速度 [N] (rad/s)
        accel_limit: 每关节最大加速度 [N] (rad/s²)
        dt: 控制周期 (s)

    Returns:
        (q_cmd, dq_cmd): 平滑后的位置和速度

    ⚠ **移植已知边界（照抄，不在本计划修）**：`diff` 恰为 0 或落在 `±1e-5` 死区内、
    而 `|v| ≥ dv_max` 时，`math.copysign(v_limit, diff)` 在 `diff == 0` 会返回 `+v_limit`
    ⇒ 该轴可能继续正向加速而非停住（原版同样如此）。实践中 `diff` 极少恰为 0，
    且 `abs(diff) < 1e-5 and abs(v) < dv_max` 已挡掉绝大多数情形。
    **忠实移植优先**；若真机 S3 观察到自激，这是嫌疑点之一（见 spec §11 S3）。
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


# ────────────────────────── 4. 从臂状态机与 watchdog ──────────────────────────

#: 状态常量（字符串，便于日志与界面直接显示）
IDLE = "IDLE"
ALIGN_FAST = "ALIGN_FAST"
FOLLOWING = "FOLLOWING"
HOLDING = "HOLDING"


class TeleopState:
    """从臂状态机 + watchdog（spec §5）。**纯逻辑，不碰硬件**。

    问询式（而不是回调式）驱动：真正的 `movej`/`move_js`/`park` 由 phase 2 的 `ArmWorker`
    按 `wants_*` 取，于是"这个状态该不该发命令"变成可断言的事实。

    ⚠ **每一拍必须"先 `tick()` 再 poll `wants_*`/`may_dispatch`"** —— 那几个 poll 口
    只读 `self.state`，不自带陈旧性检查（`may_dispatch` 收的 `now` 因此是**没有用的**，
    留着只为签名对称；别以为它在查新鲜度）。

    状态迁移（spec §5.1）::

        IDLE --start--> ALIGN_FAST --align_done--> FOLLOWING
          ^                ^                          |
          |         连续 recover_frames 个             | frame_age 超 watchdog
          |         **不同帧**                        v
          |                +----------------------  HOLDING
          |                                       | 停留 >= hold_escalate_s
          +--user_stop--> (movej 收尾)             ⇒ wants_stop_command() 一次
    """

    def __init__(self, n: int, watchdog_ms: float = 200.0,
                 recover_frames: int = 5, hold_escalate_s: float = 2.0):
        if n < 1:
            raise ValueError(f"关节数需 >= 1（给的是 {n}）")
        if watchdog_ms <= 0.0:
            raise ValueError(f"watchdog_ms 需 > 0（给的是 {watchdog_ms}）")
        if recover_frames < 1:
            raise ValueError(f"recover_frames 需 >= 1（给的是 {recover_frames}）")
        self.n = int(n)
        self.watchdog_s = float(watchdog_ms) / 1000.0
        self.recover_frames = int(recover_frames)
        self.hold_escalate_s = float(hold_escalate_s)

        self.state = IDLE
        self._hold_since: Optional[float] = None
        self._recover_seen = 0                  # 收到过的**不同帧**计数
        self._last_frame_id: Optional[int] = None
        self._stop_pending = False
        self._escalated = False
        self._align_failed = False              # 「对齐失败、不许重试」的具名载体

    # ── 迁移入口 ──────────────────────────────────────────────────────────

    def start(self, now: float) -> None:
        """用户点「启动遥操」。"""
        self.state = ALIGN_FAST
        self._hold_since = None
        self._recover_seen = 0
        self._last_frame_id = None
        self._stop_pending = False
        self._escalated = False
        self._align_failed = False

    def align_done(self, now: float) -> None:
        """`ALIGN_FAST` 的 `movej` 已到位（含 spec §5.1 的交接补丁）。"""
        if self.state == ALIGN_FAST:
            self.state = FOLLOWING

    def align_failed(self, now: float) -> None:
        """`ALIGN_FAST` 的 `movej` 失败/超时 ⇒ 保持原地，**不自动进 `FOLLOWING`**（spec §5.1）。

        刻意比 litearm-server 严格：它那边对齐失败只 `log.warning` 一句就继续跟随。
        本仓停住 —— 因为那一步失败意味着 `q_cmd` 没有同步到对齐位，
        进 `FOLLOWING` 就是带着一个未知的大误差起步。

        ⚠ **两处必须与 `align_done()` 对称**（评审订正，两处都是真缺陷）：

        1. **有状态守卫**：只在 `ALIGN_FAST` 生效。早先版本无条件置位 ⇒ 从
           `FOLLOWING`/`HOLDING` 调它会把状态**拽回** `ALIGN_FAST`。
        2. **不许动 `_stop_pending`**：早先版本把它清成 `False`，于是
           「刚点停止（已排队收尾 `movej`）→ 恰好一条的 `movej` 失败」这条路径会
           **把停止命令吞掉**，臂留在 `ALIGN_FAST` 而不是回到 `IDLE`。

        另记 `_align_failed`：它让"失败后不许重试"**可表达**。没有它，
        `wants_align_movej()` 在失败前后都为真 ⇒ phase 2 的
        `while wants_align_movej(): movej(...)` 会变成一个每 3 秒一次阻塞调用的忙重试环。
        """
        if self.state != ALIGN_FAST:
            return
        self._align_failed = True
        # ⚠ 刻意**不**动 self._stop_pending（见上）

    def user_stop(self, now: float) -> None:
        """用户点「停止」⇒ 收尾 `movej(q_now)`，回 IDLE（spec §5.3）。"""
        self.state = IDLE
        self._hold_since = None
        self._recover_seen = 0
        self._last_frame_id = None
        self._escalated = False
        self._align_failed = False
        self._stop_pending = True

    # ── 每拍 ──────────────────────────────────────────────────────────────

    def tick(self, now: float, frame_age: Optional[float],
             frame_id: Optional[int] = None) -> str:
        """每拍调一次。返回当前状态（与 `self.state` 相同）。

        Args:
            now: 本机单调时刻。
            frame_age: 距最近一帧的**本地**秒数。**`None` = 从未收到过**
                （与 `LatestSlot.peek_age` 的契约一致）；负值不会被传进来（那边已夹到 ≥ 0）。
            frame_id: 该帧的标识（用线协议里的 `Frame.seq`）。**恢复判据数的是
                "不同的帧"，不是 tick 数** —— 见下方 `recover_frames` 的说明。
        """
        fresh = frame_age is not None and frame_age <= self.watchdog_s
        if self.state == FOLLOWING:
            if not fresh:
                self.state = HOLDING
                self._hold_since = now
                self._recover_seen = 0
                self._last_frame_id = None
                self._escalated = False
        elif self.state == HOLDING:
            if fresh:
                # ⚠ 数**不同的帧**：早先版本数 tick，而 watchdog(200ms) 比 tick(10ms) 长 20 倍
                # ⇒ 同一帧能被连续 20 拍都判为"新鲜"，`recover_frames=5` 其实只要一帧 50ms 就满足。
                # 于是"防抖"根本不存在：链路每 250ms 来一帧时，会以约 4 次/秒的节奏
                # 反复触发**阻塞式** `movej` 重对齐（spec §5.1 明确说那段期间从臂完全不跟随）。
                new_frame = frame_id is None or frame_id != self._last_frame_id
                if new_frame:
                    self._recover_seen += 1
                    self._last_frame_id = frame_id
                if self._recover_seen >= self.recover_frames:
                    # ⚠ 回 ALIGN_FAST，**不是** FOLLOWING：主臂在断链期间可能已经动了，
                    # 直接跟会阶跃（spec §5.2）
                    self.state = ALIGN_FAST
                    self._hold_since = None
                    self._recover_seen = 0
                    self._last_frame_id = None
                    self._escalated = False
            else:
                self._recover_seen = 0
                self._last_frame_id = None
                if (not self._escalated and self._hold_since is not None
                        and now - self._hold_since >= self.hold_escalate_s):
                    # 长时间持位：`park()` 只把刚度钉回 1.0×、tau 仍为 0 ⇒ 会缓慢下垂。
                    # 补一次 `movej(q_now)` 进 `ht_on`（2× 刚度 + 重力前馈）才真稳。
                    self._stop_pending = True
                    self._escalated = True
        return self.state

    # ── 问询式出口（由 phase 2 的 ArmWorker 消费） ────────────────────────

    def wants_stop_command(self) -> bool:
        """是否该发一条收尾 `movej(q_now, speed=0.3)`。读后须 `consume_stop_command()`。"""
        return self._stop_pending

    def consume_stop_command(self) -> None:
        self._stop_pending = False

    def wants_align_movej(self) -> bool:
        """是否该发**对齐**的 `movej(主臂快照, speed=align_speed)`。

        与 `wants_stop_movej()` 是**两条不同的命令**（目标与速度都不同），
        早先合在一个 `wants_movej()` 里 ⇒ 调用方还得自己看 `self.state` 才知道该发哪个目标，
        那个谓词不提供任何安全性，反而招来"发错目标"。
        这里还门控了 `_align_failed`：**失败之后不再重试**（否则 phase 2 会忙重试）。
        """
        return self.state == ALIGN_FAST and not self._align_failed

    def wants_stop_movej(self) -> bool:
        """是否该发**收尾**的 `movej(q_now 实测, speed=0.3)`（用户停止 / HOLDING 升级）。"""
        return self._stop_pending

    def may_dispatch(self, now: float, have_slave_q: bool) -> bool:
        """本拍能否下发 `move_js`。

        ⚠ **`now` 不参与判断**（签名留着只为调用点对称）：陈旧性完全由 `tick()` 负责 ——
        **每拍必须先 `tick()` 再调本方法**，只 poll 不 tick 就会拿陈旧数据下发。
        ⚠ 拿不到实测 `q` 就**不许下发**：公式要 `q_slave实测`，`get_state().value`
        可能为 `None`；**绝不用 0 或上一次的值去猜**（猜出来的 `dq` 会被当速度前馈发出去）。
        """
        return self.state == FOLLOWING and have_slave_q
