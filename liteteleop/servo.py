"""从臂伺服环 —— 逻辑照 litearm-server，**执行器用 `move_js`**。

## 为什么是 `move_js`（用户裁决 2026-09-28）

litearm-server 的从臂用 `joint_follow` → `send_mit`：**K/B 随帧下发、重力在 PC 本地算**。
本 SDK 没有 `joint_follow`，而把它移植到 PC 上会**每拍要两次往返**
（`send_mit_all` + 问 `get_gravity`），实测各 3.3 ms ⇒ 6.7 ms ⇒ 只到 ~150 Hz。

`move_js` **本身就是固件里的伺服环**（`ARM_MODE_MOVE_JS` 分支，300 Hz 里跑）：

    q_ref = slew_linear(target_q, q_ref, |dq|·dt)     ← 限速跟随
    τ = mit_kp·(q_ref−q) + (mit_kd+kd_extra)·(dq_ref−dq) + G(q_d) + 摩擦

⇒ **每拍只要一次往返（3.3 ms）** ⇒ **250 Hz 装得下**，而且**不用改固件**。

## 为什么没有 K/B（用户裁决 2026-09-28）

litearm-server 的 `K=25` / `B=0.5` 走 `send_mit` **随帧下发**。`move_js` **没有那条通道**，
要改只能写固件的 `mit_kp`/`mit_kd`。**但那是碰不得的**：

- `movej` 用的就是 `mit_kp`。把它从出厂的 **400 降到 25**（软 16 倍），位置环**撑不住、
  到不了位** ⇒ `movej` 撞 `move_timeout` 报「未到位, 超时 3.0s」（**真机踩过**）
- 臂会**瞬间变软**（真机也确实如此）

⇒ 写进去还得在**每一句 `movej` 之前还原**，一整套耦合。用户裁决：**不改刚度，用出厂值**。

**代价（已接受）**：刚度/阻尼不再是 litearm-server 的 25/0.5，而是固件的
`mit_kp`（J1/J2 400、J3/J4 300、腕部 50）与 `mit_kd + kd_extra`（J1~J4 5+6=11、腕部 2.5）。
**比 litearm-server 硬得多**，跟踪更紧、手感更"僵"，但**行为可预期**。

## 与 `joint_follow` 的已知差异（用户已知悉并接受）

| # | `joint_follow` | 本实现（`move_js`） | 处置 |
| --- | --- | --- | --- |
| ① | K/B **随帧下发**（25 / 0.5） | **用固件出厂刚度**（400/300/50，阻尼 11/2.5） | 见上：改固件参数会连带改坏 `movej` |
| ② | 有力矩通道（限位墙叠在 `tau_ff`） | **没有** | 位置护栏靠 `clamp_to_limits`（server 的主护栏也是它） |
| ③ | `dq` **只做**速度前馈 | `dq` **双角色**（还限 `q_ref` 速率） | ⚠ `dq=0` 且目标 ≠ 实测时**固件会拒帧** ⇒ 退路 + `_REJECT_ESCALATE` |

## ⚠ 本模块**不写任何固件参数**

不碰 `set_joint_param` / `set_ff_vec` / `save_params`。⇒ 没有"崩了之后参数留在改过的值上"
这类问题，也没有"哪句 `movej` 必须先还原"的顺序耦合。

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
    "align_to_master", "DEFAULT_PAYLOAD_MASS", "DEFAULT_PAYLOAD_COM",
    "read_payload", "apply_payload", "speed_limit_from_kd",
    "DEFAULT_KD_BUDGET", "DEFAULT_SPEED_LIMIT", "DEFAULT_ACCEL_LIMIT",
    "DEFAULT_ENGAGE_SEC", "DEFAULT_HZ", "hold_at_current", "follow",
    "measure_move_js_cost",
]

# ── 参数真值 ────────────────────────────────────────────────────────────────
# ⛔ 限速/限加速逐个照抄 `pylitearm/config/litearm_balanced.yaml` 的 `joint_follow:` 段。
#
# ⚠⚠ **K/B 不在这里** —— 见模块 docstring「为什么没有 K/B」。
#    litearm-server 的 K=25 / B=0.5 走 `send_mit` **随帧下发**；`move_js` **没有那条通道**，
#    要改只能写固件的 `mit_kp`/`mit_kd`，而那会**连带改坏 `movej`**（`movej` 用的就是
#    `mit_kp`，软 16 倍的位置环撑不住、到不了位）。用户裁决：**不改刚度，用出厂值**。
DEFAULT_SPEED_LIMIT = [2.8, 3.4, 5.0, 5.0, 10.0, 8.0, 13.0]
DEFAULT_ACCEL_LIMIT = [14.0, 22.0, 24.0, 24.0, 45.0, 40.0, 60.0]
DEFAULT_ENGAGE_SEC = 0.3

#: **从臂伺服环**频率 = pylitearm 的 `arm._hz` = `control_loop_hz`
#: （`sdk/arm.py:652` 读它；`litearm_balanced.yaml:32` = 250）。
#: ⚠ **与主臂的 `pub_hz = 200` 是两个不同的数**，别混。
#: ⚠ `move_js` 一次往返实测 ≈3.3 ms ⇒ 周期 4 ms 占 83%，**250 装得下但没有余量**。
DEFAULT_HZ = 250.0

#: **对齐**：照搬 `teleop_manager` 的三个默认值。
#: `align_speed` 是"多快挪到主臂位姿"，不是跟随速度 —— 它必须慢。
ALIGN_SPEED = 0.15
#: 等首帧的上限（`_do_align` 里写死 5.0 s）。
ALIGN_TIMEOUT = 5.0
#: 对齐位移超过这个值就**大声预警**（不拒绝 —— 30 s 足以覆盖限位内的任何移动）。理由：`movej(speed=0.15)` 走大位移要很久，
#: 而本工具 `move_timeout=3 s` ⇒ 会在半路超时；更糟的是位移大意味着**从臂会大幅甩过去**。
#: ⚠ **等距遥操的正确用法是【先用手把两条臂摆到相近姿态再启动】** —— 对齐只兜小差。
ALIGN_WARN_DELTA = 0.30

# ── 末端载荷（夹爪）────────────────────────────────────────────────────────
#: 默认载荷：**夹爪 600 g、质心 3 cm 在 Z 轴**（用户给的值，2026-09-28）。
#: ⚠ 质心在 **`ee_link` 系**、单位 **米**（固件 `params.c:195` 逐轴钳 [-1, 1]）。
#: ⚠⚠ **别弄错轴**：填错分量会让重力补偿朝错误方向偏，**比不补更糟**。
#: 用户确认：3 cm 在 **Z** 上（不是 x —— 我第一版按 SDK 示例填了 x，是错的）。
#: ⚠ 夹爪是装在**主臂**上的，所以主臂进程要设；从臂若也装了就得各设各的。
DEFAULT_PAYLOAD_MASS = 0.6
DEFAULT_PAYLOAD_COM = (0.0, 0.0, 0.03)

#: 载荷质量/质心的 ff item（`FF_SCALAR_ITEMS`）。
_FF_ITEM_PAYLOAD_MASS = 4
_FF_ITEM_PAYLOAD_COM = 5

#: 速度前馈能吃掉多少力矩预算。`speed_limit_i ≤ budget · tau_max_i / kd_eff_i`。
DEFAULT_KD_BUDGET = 0.30

#: `kd_extra` 在 0x26 **向量表**（`FF_VEC_ITEMS[15]`）—— ⚠ **不是** 0x28 标量表，
#: `get_ff_scalar(15, ·)` 取到的是 `zg_engage_kp`（见 spec §10 陷阱 #8）。
_FF_VEC_KD_EXTRA = 15

#: 连续被固件拒帧到这个次数 ⇒ 视作"链路/状态坏了"，抛出而不是继续硬撑。
#: 理由：`move_js` 被拒 ⇒ **没有 kick 看门狗** ⇒ 0.1 s 后固件 fail-soft ⇒ **臂会垂**。
#: 零星一两次没关系（下一拍就恢复），连续不停就是真问题，必须让人知道。
_REJECT_ESCALATE = 20



def effective_kd(arm):
    """读**有效阻尼** `kd_eff = mit_kd + kd_extra`（逐轴）。

    ⚠ 这是 `move_js` 路线**最要紧的一个数**：`move_js` 的 `dq` 会进电机速度前馈
    （`τ += kd_eff·dq`），而 `kd_eff` 是**固件出厂值**（本机 J1~J4 = 5+6 = **11**、
    腕部 = 2.5+0 = 2.5）。litearm-server 的 B=0.5 是随帧下发的，**不是这个数**。
    """
    jp = arm.params.all_joint_params()
    kd = [float(p.kd) for p in jp]
    extra = [float(x) for x in arm.get_ff_vec(_FF_VEC_KD_EXTRA).value]
    return [a + b for a, b in zip(kd, extra)], [float(p.tau_max) for p in jp]


def speed_limit_from_kd(kd, tau_max, kd_budget: float = DEFAULT_KD_BUDGET):
    """按 kd 预算把力矩上限折算成**速度上限**：`speed_limit_i = budget·tau_max_i/kd_i`。

    ## 为什么 `move_js` 路线**必须**做这一步

    `move_js` 的 `dq` **是双角色**：既限 `q_ref` 的走位速率，**又直接进电机速度前馈**
    （`τ += kd_eff·dq`）。而 `kd_eff` 是固件出厂值（J1~J4 = **11**）。

    拿 litearm-server 的配置值 `speed_limit = [2.8, 3.4, 5, 5, 10, 8, 13]` 直接用的后果：

        J4: kd_eff·dq = 11 × 5.0 = **55 Nm**，而 J4 的 tau_max 只有 **21**

    ⇒ 前馈一项就把力矩预算**顶满** ⇒ `τ` 被钳 ⇒ **非线性** ⇒ **抖**（真机现象）。

    litearm-server 不会遇到：它的 `B = 0.5` 走 `send_mit` **随帧下发**，
    `B·dq = 0.5 × 5 = 2.5 Nm`，微不足道 —— **那份配置是配 `B=0.5` 的**。

    ⇒ 逐轴取 `min(配置值, 预算值)`。@30%：J1/J2 2.13、**J3 0.63、J4 0.57**、J5~J7 1.20。
    """
    if len(kd) != len(tau_max) or not kd:
        raise ValueError("kd/tau_max 长度不符或为空")
    out = []
    for i, (k, tm) in enumerate(zip(kd, tau_max)):
        if k <= 0.0 or tm <= 0.0:
            raise ValueError(f"第 {i} 轴 kd/tau_max 非法: kd={k} tau_max={tm}")
        out.append(kd_budget * tm / k)
    return out


def read_payload(arm):
    """读回**末端载荷** `(mass_kg, [x, y, z])`。

    ⚠ 读回的是**固件钳后的真值** —— 这是唯一能确认"到底写进去了什么"的办法。
    """
    mass = float(arm.get_ff_scalar(_FF_ITEM_PAYLOAD_MASS, 0).value)
    com = [float(arm.get_ff_scalar(_FF_ITEM_PAYLOAD_COM, k).value) for k in range(3)]
    return mass, com


def apply_payload(arm, mass: float, com=(0.0, 0.0, 0.0)):
    """设末端载荷，并**读回**实际生效值。

    ⛔ **只写 RAM**（不调 `save_params()` —— 那是整扇区擦写、不可逆）。

    ⚠⚠ SDK 的 `set_payload` docstring 明说：**固件会静默钳制**
    （`mass` → `[0, 20]`、`com` 逐轴 → `[-1, 1]`，`params.c:190/195`），
    并且**照样回 ACK**。所以"调用了没报错"**不等于**"值生效了" ——
    必须读回。本函数把读回值返回给调用方去显示。
    """
    arm.set_payload(float(mass), [float(v) for v in com])
    return read_payload(arm)


def hold_at_current(arm) -> None:
    """受控接管 —— `request_stop()` / `emergency_hold_healthy()` 的对应物。

    用 `movej(实测位姿)` 让固件的 S 曲线 + `ht_on` 接管。
    ⛔ **绝不 `disable()`**：失能会让臂在自重下自由落体。
    """
    st = arm.get_state(refresh=True).value
    if st is None:
        raise RuntimeError("收尾取不到状态帧")
    arm.movej(list(st.q), speed=0.3)


def _send_hold(arm, tries: int = 5, gap: float = 0.02) -> bool:
    """发一帧「**托住实测位姿**」。**每次重读实测**，因为臂可能还在动。

    ⚠⚠ 为什么要重读 + 重试：`move_js` 在「目标 ≠ 实测位姿」且「`dq == 0`」时**被固件拒**。
    而 `movej` 的到位判据是 `q_tol=0.03`、`dq_tol=0.10` —— 它返回时**臂可能还在动**。
    于是"读一次实测 → 发一帧"之间位姿就过期了 ⇒ 被拒。

    ⚠ `prime` / `engage` 曾经**没有**这层保护，真机上就是这么崩的：
    对齐的 `movej` 超时（臂还在走）⇒ `prime` 拿缓存位姿发 `move_js` ⇒ 被拒 ⇒
    异常直接穿出 `follow` ⇒ `⛔ 遥操异常退出`。
    """
    q = _q_meas(arm)
    for _ in range(max(1, tries)):
        try:
            arm.move_js(q, [0.0] * N_JOINTS)
            return True
        except Exception:                                # noqa: BLE001
            time.sleep(gap)
            try:
                q = _q_meas(arm)                         # 重读：臂可能又动了
            except Exception:                            # noqa: BLE001
                pass
    return False


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
           duration_s: Optional[float] = None) -> bool:
    """从臂跟随环。**参考生成与调用序照抄 `joint_follow`，执行器换 `move_js`。**

    `target_provider()` 返回**目标关节角序列**；返回 `None` 时**保持上一拍的
    `q_cmd/dq_cmd` 不动**（照搬 `joint_follow` 那一支 —— "首帧到达前原地不动"）。

    ⚠ **不改任何固件参数** —— K/B 用固件出厂的 `mit_kp`/`mit_kd`(+`kd_extra`)。
    """
    try:
        sp = list(speed_limit or DEFAULT_SPEED_LIMIT)
        ac = list(accel_limit or DEFAULT_ACCEL_LIMIT)
        dt_nom = 1.0 / max(hz, 1.0)

        # ── prime：**托住实测位姿**（目标==实测 ⇒ 固件接受，内置刚度+重力托住）──
        if not _send_hold(arm):
            raise RuntimeError(
                "prime 连发 5 次「托住实测位姿」都被固件拒 —— 这不是稳态残差，"
                "是链路或臂状态有问题（臂可能一直在动）。受控接管后上抛。")
        q_cmd = _q_meas(arm)
        dq_cmd = [0.0] * N_JOINTS
        q_target = list(q_cmd)

        # ── engage：engage_sec 内持续"托在原地"（对应 joint_follow 的低刚度托举段）──
        t_end = time.monotonic() + max(engage_sec, 0.0)
        while time.monotonic() < t_end and not should_stop():
            _send_hold(arm, tries=2)
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
                q_hold = _q_meas(arm)
                if not _send_hold(arm, tries=3):
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
        pass


# ────────────────────────── 对齐（照搬 `_do_align`）──────────────────────────

def align_to_master(arm, take_frame, limits, *, speed: float = ALIGN_SPEED,
                    timeout: float = ALIGN_TIMEOUT,
                    max_delta: float = ALIGN_WARN_DELTA):
    """等首帧（带超时）→ 钳位 → **低速 `movej` 对齐** → 返回对齐到的位姿。

    逐条照搬 `teleop_manager._do_align()`：

    - 等首帧上限 `timeout`（原版写死 5 s）；等不到就**跳过对齐**并返回 `None`
      （原版：`log.warning("teleop 对齐：5s 内未收到 master 帧，跳过对齐")`）
    - `clip` 到软限位；被钳的轴要报出来
    - `movej(clamped, speed=align_speed)`（原版还有 `settle_s=0.5`，本 SDK 的 `movej`
      本身阻塞到位，`settle` 是隐含的）
    - `movej` 失败**不致命**（原版：`对齐 movej 失败（跟随会逐步修正）`）⇒ 返回 `None`

    ⚠ **本函数会阻塞到 `arm.move_timeout`** —— 用户裁决把它设成 **30 s**，
    但 `movej` **一旦到位就立刻返回**（`_arrive` 是"到位/超时/故障"三者先到为准）
    ⇒ 这只是上限，不是固定等待。
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
        log.warning("对齐：**%.0f s 内没收到主臂帧**，跳过对齐（跟随会逐步修正）。"
                    "检查主臂是否已启动遥操、arm_id/端口是否一致", timeout)
        return None

    clamped, sat = clamp_to_limits(master_q, limits)
    if any(sat):
        log.warning("对齐：关节 %s 超限已钳位",
                    [i + 1 for i, s_ in enumerate(sat) if s_])
    q_now = _q_meas(arm)
    delta = max(abs(a - b) for a, b in zip(clamped, q_now))
    if delta > max_delta:
        # ⚠ 只**预警**、不拒绝：`move_timeout` 已放到 30 s（用户裁决 2026-09-28），
        #   而 `movej(speed=0.15)` 在 J1 上约 0.3 rad/s ⇒ 30 s 能走 ~9 rad，
        #   **限位内的任何位移都够**。但仍要说一声 —— 大位移意味着从臂会大幅摆动。
        est = delta / max(speed * 2.0, 1e-6)
        log.warning(
            "⚠ 对齐位移 %.3f rad 偏大（>%.2f）—— 从臂会**大幅摆动**，"
            "按 speed=%.2f 估计约需 %.1f s（上限 %.0f s，到位即提前返回）。"
            "⚠ 确认从臂周围清空、且不会撞到主臂。",
            delta, max_delta, speed, est, arm.move_timeout)
    try:
        arm.movej(clamped, speed=speed)
    except Exception as e:                                # noqa: BLE001
        log.warning("对齐：**收到帧了，但 movej 失败**（位移 %.3f rad）: %s —— "
                    "⚠ 臂可能还在移动中，随后的 prime 会重读实测位姿再托住",
                    delta, e)
        return None
    log.info("对齐完成（位移 %.3f rad）", delta)
    return list(clamped)
