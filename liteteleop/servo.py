"""从臂伺服环 —— 逻辑照 litearm-server，**执行器用 `move_js`**。

## 为什么是 `move_js`（用户裁决 2026-09-28）

litearm-server 的从臂用 `joint_follow` → `send_mit`：**K/B 随帧下发、重力在 PC 本地算**。
本 SDK 没有 `joint_follow`，而把它移植到 PC 上会**每拍要两次往返**
（`send_mit_all` + 问 `get_gravity`），实测各 3.3 ms ⇒ 6.7 ms ⇒ 只到 ~150 Hz。

`move_js` **本身就是固件里的伺服环**（`ARM_MODE_MOVE_JS` 分支，300 Hz 里跑）：

    q_ref = slew_linear(target_q, q_ref, |dq|·dt)     ← 限速跟随
    τ = mit_kp·(q_ref−q) + (mit_kd+kd_extra)·(dq_ref−dq) + G(q_d) + 摩擦

⇒ **每拍只要一次往返（3.3 ms）** ⇒ **250 Hz 装得下**，而且**不用改固件**。

## 与 `joint_follow` 的三处已知差异（用户已知悉并接受）

| # | `joint_follow` | 本实现（`move_js`） | 处置 |
| --- | --- | --- | --- |
| ① | K/B **随帧下发** | K/B **预先写进固件参数** | `apply_joint_gains()`，退出时 `restore_joint_gains()` |
| ② | 有力矩通道（限位墙叠在 `tau_ff`） | **没有** | 位置护栏靠 `np.clip`-等价的 `clamp_to_limits`（server 的主护栏也是它） |
| ③ | `dq` **只做**速度前馈 | `dq` **双角色**（还限 `q_ref` 速率） | ⚠ `dq=0` 且目标 ≠ 实测时**固件会拒帧** —— 见 `_REJECT_ESCALATE` |

## ⚠⚠ 参数写入的安全性质

- `set_joint_param` / `set_ff_vec` **只写 RAM**，**不调 `save_params()`** ⇒ **断电即还原**，
  不会把臂的出厂配置改坏（⛔ 绝不调 `save_params()` —— 那是整扇区擦写，不可逆）
- 但**进程崩溃时参数会留在改过的值上**（直到断电）⇒ 必须提供显式恢复路径，
  并把"退出恢复"放进 `finally`

控制律的**参考生成**（`slew_target`）、**参数真值**、**watchdog**、**对齐**、**钳位**
仍然逐条照 litearm-server，见 spec §5、§9.1。
"""
from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field
from typing import Callable, List, Optional, Sequence, Tuple

from .safety import clamp_to_limits, slew_target
from .wire import N_JOINTS
from .wire import decode_teleop as wire_decode

log = logging.getLogger("liteteleop.servo")

__all__ = [
    "ALIGN_SPEED", "ALIGN_TIMEOUT", "ALIGN_WARN_DELTA",
    "align_to_master", "DEFAULT_K", "DEFAULT_B", "DEFAULT_SPEED_LIMIT", "DEFAULT_ACCEL_LIMIT",
    "DEFAULT_ENGAGE_SEC", "DEFAULT_HZ", "JointGains", "apply_joint_gains",
    "restore_joint_gains", "hold_at_current", "follow", "measure_move_js_cost",
]

# ── 参数真值 ────────────────────────────────────────────────────────────────
# ⛔ 逐个照抄 `pylitearm/config/litearm_balanced.yaml` 的 `joint_follow:` 段
#    （litearm-server 真机验证过的那一套），**没有就地调参**。
DEFAULT_K = [25.0] * N_JOINTS
DEFAULT_B = [0.5] * N_JOINTS
DEFAULT_SPEED_LIMIT = [2.8, 3.4, 5.0, 5.0, 10.0, 8.0, 13.0]
DEFAULT_ACCEL_LIMIT = [14.0, 22.0, 24.0, 24.0, 45.0, 40.0, 60.0]
DEFAULT_ENGAGE_SEC = 0.3

#: **从臂伺服环**频率 = pylitearm 的 `arm._hz` = `control_loop_hz`
#: （`sdk/arm.py:652` 读它；`litearm_balanced.yaml:32` = 250）。
#: ⚠ **与主臂的 `pub_hz = 200` 是两个不同的数**，别混。
#: ⚠ `move_js` 一次往返实测 ≈3.3 ms ⇒ 周期 4 ms 占 83%，**250 装得下但没有余量**。
DEFAULT_HZ = 250.0

#: `emit_reason` 的 item：`kd_extra`（τ 域软件微分阻尼）。见 spec §10 陷阱 #8：
#: 它在 **0x26 向量表**里，要用 `get_ff_vec(15)` 读，**不是** `get_ff_scalar(15,·)`。
_FF_VEC_KD_EXTRA = 15

#: **对齐**：照搬 `teleop_manager` 的三个默认值。
#: `align_speed` 是"多快挪到主臂位姿"，不是跟随速度 —— 它必须慢。
ALIGN_SPEED = 0.15
#: 等首帧的上限（`_do_align` 里写死 5.0 s）。
ALIGN_TIMEOUT = 5.0
#: 对齐位移超过这个值就**大声预警**。理由：`movej(speed=0.15)` 走大位移要很久，
#: 而本工具 `move_timeout=3 s` ⇒ 会在半路超时；更糟的是位移大意味着**从臂会大幅甩过去**。
#: ⚠ **等距遥操的正确用法是【先用手把两条臂摆到相近姿态再启动】** —— 对齐只兜小差。
ALIGN_WARN_DELTA = 0.30

#: 连续被固件拒帧到这个次数 ⇒ 视作"链路/状态坏了"，抛出而不是继续硬撑。
#: 理由：`move_js` 被拒 ⇒ **没有 kick 看门狗** ⇒ 0.1 s 后固件 fail-soft ⇒ **臂会垂**。
#: 零星一两次没关系（下一拍就恢复），连续不停就是真问题，必须让人知道。
_REJECT_ESCALATE = 20


@dataclass
class JointGains:
    """改参数前**存下来的原值**，用于退出时逐字还原。"""

    kp: List[float] = field(default_factory=list)
    kd: List[float] = field(default_factory=list)
    tau_max: List[float] = field(default_factory=list)
    kd_extra: List[float] = field(default_factory=list)
    applied: bool = False


def apply_joint_gains(arm, K: Sequence[float], B: Sequence[float]) -> JointGains:
    """把 `K`/`B` 写进固件的关节参数 —— `joint_follow` 的"K/B 随帧下发"的替代。

    ⚠ 为什么必须连 `kd_extra` 一起改：`move_js` 的有效阻尼是
    **`mit_kd + kd_extra`**，本机 `kd_extra = [6,6,6,6,0,0,0]`。
    只设 `kd=0.5` 的话，J1~J4 的实际阻尼是 **6.5（13 倍）**，`kd·dq` 在 5 rad/s 时
    是 32.5 Nm —— 而 J4 的 `tau_max` 只有 21 ⇒ **力矩预算被吃光**。

    ⚠ **只写 RAM**（不调 `save_params()`）⇒ 断电即还原。
    """
    jp = arm.params.all_joint_params()
    saved = JointGains(
        kp=[float(p.kp) for p in jp],
        kd=[float(p.kd) for p in jp],
        tau_max=[float(p.tau_max) for p in jp],
        kd_extra=[float(x) for x in arm.get_ff_vec(_FF_VEC_KD_EXTRA).value],
    )
    for i in range(N_JOINTS):
        arm.params.set_joint_param(i, float(K[i]), float(B[i]), saved.tau_max[i])
    arm.set_ff_vec(_FF_VEC_KD_EXTRA, [0.0] * N_JOINTS)
    saved.applied = True
    log.info("已写入关节增益：K=%s B=%s，并清零 kd_extra", list(K), list(B))
    return saved


def restore_joint_gains(arm, saved: Optional[JointGains]) -> None:
    """把 `apply_joint_gains` 存下的原值逐字写回。幂等，可重复调。"""
    if saved is None or not saved.applied:
        return
    try:
        for i in range(N_JOINTS):
            arm.params.set_joint_param(i, saved.kp[i], saved.kd[i], saved.tau_max[i])
        arm.set_ff_vec(_FF_VEC_KD_EXTRA, saved.kd_extra)
        saved.applied = False
        log.info("已还原关节增益 kp/kd/tau_max 与 kd_extra")
    except Exception:
        log.exception("⚠ 还原关节增益失败 —— 参数会留在改过的值上，"
                      "直到断电或手工还原！")


def hold_at_current(arm) -> None:
    """受控接管 —— `request_stop()` / `emergency_hold_healthy()` 的对应物。

    用 `movej(实测位姿)` 让固件的 S 曲线 + `ht_on` 接管。
    ⛔ **绝不 `disable()`**：失能会让臂在自重下自由落体。
    """
    st = arm.get_state(refresh=True).value
    if st is None:
        raise RuntimeError("收尾取不到状态帧")
    arm.movej(list(st.q), speed=0.3)


def _q_meas(arm) -> List[float]:
    st = arm.get_state(refresh=False).value
    if st is None:
        raise RuntimeError("状态帧取不到（链路静默）")
    return list(st.q)


def measure_move_js_cost(arm, n: int = 100) -> float:
    """实测 `move_js` 一次往返的耗时（ms）—— 环频上限由它决定。

    ⚠ 用 `move_js(q_now, dq=0)` 量：目标 == 实测位姿、`dq=0` ⇒ **臂不动**
    （真机验证过这种组合会被接受）。
    """
    q = _q_meas(arm)
    t0 = time.monotonic()
    for _ in range(n):
        arm.move_js(q, [0.0] * N_JOINTS)
    return (time.monotonic() - t0) / n * 1000.0


def follow(arm, target_provider: Callable[[], Optional[Sequence[float]]],
           K=None, B=None, speed_limit=None, accel_limit=None,
           engage_sec: float = DEFAULT_ENGAGE_SEC, hz: float = DEFAULT_HZ,
           should_stop: Callable[[], bool] = lambda: False,
           duration_s: Optional[float] = None,
           gains: Optional[JointGains] = None) -> bool:
    """从臂跟随环。**参考生成与调用序照抄 `joint_follow`，执行器换 `move_js`。**

    `target_provider()` 返回**目标关节角序列**；返回 `None` 时**保持上一拍的
    `q_cmd/dq_cmd` 不动**（照搬 `joint_follow` 那一支 —— "首帧到达前原地不动"）。

    ⚠ `gains`：传入 `apply_joint_gains()` 的返回值，则**不**在这里改参数
    （调用方负责生命周期）。传 `None` 时本函数自己改、自己在 `finally` 里还原。
    """
    own_gains = gains is None
    saved = gains
    try:
        if own_gains:
            saved = apply_joint_gains(arm, K or DEFAULT_K, B or DEFAULT_B)

        sp = list(speed_limit or DEFAULT_SPEED_LIMIT)
        ac = list(accel_limit or DEFAULT_ACCEL_LIMIT)
        dt_nom = 1.0 / max(hz, 1.0)

        # ── prime：目标 == 实测位姿、dq=0 ⇒ 固件接受，且用内置刚度与重力【托住】──
        q_cmd = _q_meas(arm)
        arm.move_js(q_cmd, [0.0] * N_JOINTS)
        dq_cmd = [0.0] * N_JOINTS
        q_target = list(q_cmd)

        # ── engage：engage_sec 内持续"托在原地"（对应 joint_follow 的低刚度托举段）──
        t_end = time.monotonic() + max(engage_sec, 0.0)
        while time.monotonic() < t_end and not should_stop():
            arm.move_js(_q_meas(arm), [0.0] * N_JOINTS)
            time.sleep(dt_nom)

        # ── start：指令/目标都初始化成【当前实测】──
        q_cmd = _q_meas(arm)
        dq_cmd = [0.0] * N_JOINTS
        q_target = list(q_cmd)

        base = time.monotonic()
        next_tick = base + dt_nom
        rejects = 0
        holding = False          # 是否正处在「托住实测位姿」的退路里

        while not should_stop():
            if duration_s is not None and time.monotonic() - base >= duration_s:
                break

            try:
                got = target_provider()
            except Exception:                        # noqa: BLE001 - 照搬：先持位再上抛
                log.exception("target_provider 异常，受控接管")
                hold_at_current(arm)
                return False
            if got is not None:
                if len(got) != N_JOINTS:
                    raise ValueError(f"目标必须是 {N_JOINTS} 元素，收到 {len(got)}")
                q_target = [float(v) for v in got]

            # 参考生成：**逐字照抄 joint_follow**（梯形速度曲线 + 制动距离）
            q_cmd, dq_cmd = slew_target(q_target, q_cmd, dq_cmd, sp, ac, dt_nom)

            try:
                arm.move_js(q_cmd, dq_cmd)
                rejects = 0
                if holding:
                    holding = False
                    log.info("已恢复正常跟随")
            except Exception as e:                   # noqa: BLE001
                # ⚠⚠ `move_js` 在「目标 ≠ 实测位姿」且「`dq == 0`」时**会被固件拒**
                # （真机实证）。而从臂到位后 `slew_target` 给的就是 `dq=0`，并且
                # **实测与目标总有 settle 残差** —— `movej` 的到位判据是 `q_tol=0.03`，
                # 允许差 0.03 rad。于是**每一拍都被拒** ⇒ 拒帧就**不 kick 看门狗**
                # ⇒ 0.1 s 后 fail-soft ⇒ **臂会垂**。真机实测：连续 20 次后升级退出。
                #
                # 这不是 `send_mit` 的问题（MIT 帧不受这条限制），所以 litearm-server
                # 碰不到 —— 是 `move_js` 路线的固有短板。
                #
                # 退路：**托住实测位姿**。那一帧目标==实测 ⇒ 一定被接受 ⇒ 看门狗照 kick，
                # 再把参考同步过去，下一拍 `slew_target` 从实测位姿重新起步
                # ⇒ **主臂一动，目标一变，就立刻恢复正常跟随**。
                try:
                    q_hold = _q_meas(arm)
                    arm.move_js(q_hold, [0.0] * N_JOINTS)
                except Exception as e2:              # noqa: BLE001
                    rejects += 1
                    log.warning("move_js 被拒（第 %d 次连续）：%s", rejects, e)
                    if rejects >= _REJECT_ESCALATE:
                        log.error("连续 %d 次被拒 —— 受控接管并上抛", rejects)
                        hold_at_current(arm)
                        raise
                else:
                    rejects = 0
                    q_cmd[:] = q_hold
                    dq_cmd[:] = [0.0] * N_JOINTS
                    q_target = list(q_hold)
                    if not holding:
                        holding = True
                        log.info("目标不可达（残差在 movej 的 q_tol=0.03 之内）"
                                 "⇒ 改为托住实测位姿；主臂一动即恢复。原错误: %s", e)

            r = next_tick - time.monotonic()
            if r > 0:
                time.sleep(r)
            next_tick += dt_nom
            if next_tick < time.monotonic():
                next_tick = time.monotonic() + dt_nom
        return True
    finally:
        if own_gains:
            restore_joint_gains(arm, saved)


# ────────────────────────── 对齐（照搬 `_do_align`）──────────────────────────

def align_to_master(arm, take_frame, limits, *, speed: float = ALIGN_SPEED,
                    timeout: float = ALIGN_TIMEOUT):
    """等首帧（带超时）→ 钳位 → **低速 `movej` 对齐** → 返回对齐到的位姿。

    逐条照搬 `teleop_manager._do_align()`：

    - 等首帧上限 `timeout`（原版写死 5 s）；等不到就**跳过对齐**并返回 `None`
      （原版：`log.warning("teleop 对齐：5s 内未收到 master 帧，跳过对齐")`）
    - `clip` 到软限位；被钳的轴要报出来
    - `movej(clamped, speed=align_speed)`（原版还有 `settle_s=0.5`，本 SDK 的 `movej`
      本身阻塞到位，`settle` 是隐含的）
    - `movej` 失败**不致命**（原版：`对齐 movej 失败（跟随会逐步修正）`）⇒ 返回 `None`

    ⚠ 本仓**多加一条**：位移超过 `ALIGN_WARN_DELTA` 时**大声预警**（原版没有）。
    理由见那个常量的注释 —— 大位移会让从臂在大幅摆动中撞上 `move_timeout`。
    """
    deadline = time.monotonic() + float(timeout)
    master_q = None
    while time.monotonic() < deadline:
        payload, _ts = take_frame()
        if payload is not None:
            try:
                master_q = wire_decode(payload)["q"]
                break
            except Exception as e:                        # noqa: BLE001
                log.warning("对齐：帧解不开（%s），继续等", e)
        time.sleep(0.01)
    if master_q is None:
        log.warning("对齐：%.0f s 内未收到主臂帧，跳过对齐（跟随会逐步修正）", timeout)
        return None

    clamped, sat = clamp_to_limits(master_q, limits)
    if any(sat):
        log.warning("对齐：关节 %s 超限已钳位",
                    [i + 1 for i, s_ in enumerate(sat) if s_])
    q_now = _q_meas(arm)
    delta = max(abs(a - b) for a, b in zip(clamped, q_now))
    if delta > ALIGN_WARN_DELTA:
        log.warning(
            "⚠⚠ 对齐位移 %.3f rad 过大（阈值 %.2f）—— 从臂会**大幅摆动**。"
            "`movej(speed=%.2f)` 很可能撞上 move_timeout=%.1fs 而在半路超时；"
            "**正确做法是先用手把两条臂摆到相近姿态再启动遥操**。",
            delta, ALIGN_WARN_DELTA, speed, arm.move_timeout)
    try:
        arm.movej(clamped, speed=speed)
    except Exception as e:                                # noqa: BLE001
        log.warning("对齐 movej 失败（跟随会逐步修正）: %s", e)
        return None
    log.info("对齐完成（位移 %.3f rad）", delta)
    return list(clamped)
