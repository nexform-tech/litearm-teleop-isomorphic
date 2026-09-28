"""从臂伺服环 —— **逻辑与执行器都照 litearm-server 的 `joint_follow`**。

litearm-server 的从臂是 `joint_follow` → `send_mit(K, B, q_cmd, dq_cmd, G(q))`：
**K/B 随帧下发、重力在 PC 本地算**。本 SDK 没有 `joint_follow`，但这两件事都直接可做：

    τ_ff = clamp(G(q), ±tau_max)                      # get_gravity，一次往返
    send_mit_all(q_cmd, dq_cmd, K=25, B=0.5, τ_ff)    # 一次往返

⚠ **别用 `move_js` 代替**（那是已弃的旧路线）：`move_js` 的 K/B 是**固件出厂参数**
（`mit_kp`=400、`mit_kd`+`kd_extra`=11），**没有随帧通道** —— 刚度差 16 倍、阻尼差 22 倍，
手感**不可能**一样。真机判据：「跟随太慢，有明显的延迟」「没有 litearm-server 丝滑」。
唯一能改它的办法是写固件参数，而**那条路已证伪**（见 `SETUP_K` 上面那段）。

**执行器是 `send_mit_all`（MIT 透传），K/B 随帧下发** —— 与 litearm-server 的
`joint_follow` **逐值一致**（`litearm.yaml` 的 60/40 与 1.0/0.8，见 `SETUP_K` 那段）。

## 与 `joint_follow` 的已知差异（用户已知悉并接受）

| # | `joint_follow` | 本实现 | 处置 |
| --- | --- | --- | --- |
| ① | K/B **随帧下发**（25 / 0.5） | **同**（走 `send_mit_all` 的 `kp`/`kd`） | ✅ 一致 |
| ② | 有力矩通道（限位墙叠在 `tau_ff`） | **没有**（`tau_ff` 让给重力项） | 位置护栏靠 `clamp_to_limits`（server 的主护栏也是它） |
| ③ | 每拍 **1 次**往返 | 每拍 **2 次**（`get_gravity` + `send_mit_all`，各实测 3.333 ms） | ⇒ **~150 Hz**（旧 `move_js` 是 1 次 ⇒ 250 Hz）。用户裁决：先试纯粹版 |

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
    "read_payload", "apply_payload",
    "SETUP_K", "SETUP_B", "ENGAGE_KP", "ENGAGE_KD",
    "DEFAULT_SPEED_LIMIT", "DEFAULT_ACCEL_LIMIT",
    "DEFAULT_ENGAGE_SEC", "DEFAULT_HZ", "hold_at_current", "follow",
]

# ── 参数真值 ────────────────────────────────────────────────────────────────
# ⛔ 限速/限加速逐个照抄 `pylitearm/config/litearm_balanced.yaml` 的 `joint_follow:` 段。
#
# ⚠⚠ **K/B 不在这里** —— 见模块 docstring「为什么没有 K/B」。
#    litearm-server 的 K=25 / B=0.5 走 `send_mit` **随帧下发**；`move_js` **没有那条通道**，
#    要改只能写固件的 `mit_kp`/`mit_kd`，而那会**连带改坏 `movej`**（`movej` 用的就是
#    `mit_kp`，软 16 倍的位置环撑不住、到不了位）。用户裁决：**不改刚度，用出厂值**。
#: ⛔⛔ **不是 litearm-server 的值** —— 有实测依据（2026-09-28）。
#:
#: server 的那份 `[2.8, 3.4, 5.0, 5.0, 10.0, 8.0, 13.0]` 来自一个**没有固件安全层**的
#: 系统：它走 CAN **直连电机**，且 `joint_follow` 每步都主动豁免了检查 ——
#:
#:     hw.assert_operational(measured_overspeed_factor=float('inf'),   # 超速检查关掉
#:                           skip_position=True)                        # 位置检查跳过
#:
#: 我们经过 STM32 固件，而固件的超速判据**永久开着、豁免不了**：
#:
#:     safety_check.c:  |dq| > jp->vel_max × 1.5   连续 5 拍 ⇒ 锁存 joint_fault
#:     固件整臂表:      vel_max = [2.0, 2.0, 1.75, 1.75, 2.0, 2.0, 2.0]  ⇒ 阈值 2.6~3.0
#:
#: ⇒ 拿 server 的 5/13 rad/s 会**持续越线**：从臂不会更快，只会锁存掉力
#:   （锁存 ⇒ 固件停发该轴控制帧 ⇒ 达妙电机"收帧才回状态"⇒ 静默 ⇒ 80 ms 后 `FB_STALE`）。
#:   真机实录（`21:04:17`）：`OVERSPEED` 首拍即报，5 拍后 J3 锁存，J4 随后跟进。
#:
#: ⚠ `vel_max` 是**编译期常量**（`defaults.c`）—— SDK 只暴露 `kp/kd/tau_max/q_min/q_max`，
#:   **没有任何命令能改它**（`set_joint_limit` 只能改限位）⇒ **不改固件就绕不过去**。
#:
#: ⇒ 取固件整臂表的 `speed_limit`（与 `vel_max` 同值）。**这不是"变慢"**：`move_js` 路线下
#:   固件用的就是这张表 ⇒ **这才是从臂本来就有的速度**，只是 MIT 路线的位置命令由 PC 给，
#:   所以这张表要由我们在 PC 侧执行。
DEFAULT_SPEED_LIMIT = [2.0, 2.0, 1.75, 1.75, 2.0, 2.0, 2.0]
DEFAULT_ACCEL_LIMIT = [14.0, 22.0, 24.0, 24.0, 45.0, 40.0, 60.0]
DEFAULT_ENGAGE_SEC = 0.3

#: **从臂伺服环**频率。⚠ **这是"打算跑到多少"，不是"能跑到多少"** ——
#: MIT 路线每拍 **2 次往返**（`get_gravity` + `send_mit_all`，各实测 **3.333 ms**）
#: ⇒ **实际上限 ~150 Hz**。`dt_nom = 1/hz` 会**直接进 `slew_target`**，
#: 所以这个数**必须贴着实际周期填**，否则限速参考本身就算错了。
#: ⚠ 旧 `move_js` 路线每拍 1 次往返，那时这里填 250（`litearm_balanced.yaml:32`）。
#: ⚠ 与主臂的 `pub_hz = 200` 是两个不同的数，别混。
DEFAULT_HZ = 150.0

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

#: ⛔⛔ **未接线**（2026-09-28 用户裁决：先回到能跑的状态）。
#:
#: ## 为什么停用 —— 我只验证了一半就上了
#:
#: **静态托得住 ≠ 动态跟得住。**
#:
#:    静态（臂不动、只托住自己）  150 拍 / 2 s 最大偏移 **0.0004 rad** ✓
#:    动态（主臂在动、从臂要跟）  24 s 后 **FAULT FB_STALE POS_VIOL，断轴 J2/J4** ✗
#:
#: `move_js` 的参考是固件按 `|dq|` **二次 slew** 的，`mit_kp=25` 太软**跟不上**
#: ⇒ 位置越界 ⇒ 自保护断轴。而 **J4 的上限只有 +0.018 rad**，一点正偏就爆。
#:
#: ⚠ **`FB_STALE`（反馈失效）也同时出现了** —— 若它是**先**发生的，这次故障就与增益无关。
#:   日志看不出先后 ⇒ **根因未定**，不能断定"就是软增益的锅"。
#:   要复现并查清（固件 `log.capture` 能看时序）之后再谈。
#:
#: ## 原来的设想（留着备查）
#:
#:
#: ⚠⚠ **真机验证过（2026-09-28）**：写进去后臂**稳稳托住** ——
#:     150 拍 / 2.00 s，七轴**最大偏移 0.0004 rad**，`dq` 峰值 0.037 rad/s。
#:     ⇒ 因为 `move_js` 的 `builtin_mode`（`ff_mask` 含 `FF_MASTER`）**会加 `G(q)`**。
#:     ⇒ **"软而稳"就是 litearm-server 的手感**，不是故障。
#:     ⚠ 我一度把它误判成"臂垂下去了"并回退 —— **那是错的**，量过才知道。
#:
#: ## 为什么这样能得到「250 Hz + 丝滑」
#:
#:     `move_js` + 固件写 K/B      每拍 **1 次往返** ⇒ **250 Hz** ✓
#:     `send_mit(K,B)` + PC 算 G   每拍 2 次往返   ⇒ 只到 ~150 Hz
#:
#: 两条在**力矩上算出来是同一件事**（`builtin_mode` 的 G 顶上 PC 送的那份），
#: 所以前者用更少的往返拿到同样的手感。
#:
#: ## 代价（必须管住）
#:
#: ⚠ 写的是**固件的 `mit_kp`/`mit_kd`** ⇒ **`movej` 也用这两个数**。
#:   `mit_kp` 从 400 降到 25 后，`movej` 的到位环**撑不住**（真机踩过：报
#:   「未到位, 超时 3.0s」）。⇒ **凡是要调 `movej` 的地方，必须先把增益还原。**
#: ⚠ 只写 RAM（不调 `save_params()`）⇒ 断电即还原；但**进程崩溃会留在改过的值上**，
#:   所以恢复点必须放进 `finally`。
#: **从臂跟随增益 —— 照抄 `joint_follow` 的 `K=25.0 / B=0.5`，随 `send_mit_all` 每帧下发。**
#:
#: ⚠⚠ 与**上面那段"写进固件"的路线完全不同**（那条已证伪），两者是**不同通道**：
#:
#:     写 `mit_kp`/`mit_kd`      → **固件全局参数** ⇒ `movej` 也用 ⇒ 改坏 `movej`（真机踩过）
#:     `send_mit_all(kp=,kd=)`   → **只对这一帧生效** ⇒ `movej` **完全不受影响** ✓
#:
#: ## 为什么必须换执行器，才拿得到 server 的手感
#:
#: 两条路控制律**形式相同、参数差一个数量级**：
#:
#:     joint_follow（server）   τ =  25·(q_cmd−q) + 0.5·(dq_cmd−dq) + G(q)
#:     move_js      （旧路线）   τ = 400·(q_cmd−q) +  11·(dq_cmd−dq) + G(q)
#:                                     ↑ 刚度 16×       ↑ 阻尼 22×
#:
#: `move_js` **没有 K/B 通道**（那两个是固件出厂参数）⇒ 手感**不可能**一样。
#: 真机判据：旧路线用户报「跟随太慢，有明显的延迟」「没有 litearm-server 丝滑」。
#:
#: ## 代价
#:
#:     `send_mit_all` + `get_gravity`   每拍 **2 次往返**（各实测 3.333 ms）⇒ **~150 Hz**
#:     `move_js`                        每拍 1 次往返 ⇒ 250 Hz
#:
#: ⚠ 这一趟**省不掉**：固件对 MIT 透传**不会**自己加 G（`control_loop.c`：
#:   「`move_mit` / `move_js+tau_ff` 永不叠加内置」）⇒ **G 必须 PC 送**。
#: **从臂跟随增益 —— 照抄 litearm-server 默认配置 `litearm.yaml` 的 `joint_follow:` 段。**
#:
#: ⚠⚠ **别抄 `litearm_balanced.yaml`**：那份是 `[25]×7 / [0.5]×7`，而它自己的历史注释
#:   写着「从 25 全一律**提高**以改善主从遥操的**滞后/追不上**手感（2026-08-13）」
#:   ⇒ **25 就是已知会"滞后、追不上"的那一档**。真机实测（2026-09-28）复现了同一现象：
#:   用户报「跟随性比较差，明显延迟，而且会很软」。
#:
#: ⚠ server 的取值链：`TeleopManager(K=None)` → `arm.joint_follow(K=None)` →
#:   `from_params` → `cfg["joint_follow"]["K"]`；而 server **没有传 config** ⇒
#:   走 SDK 默认（`pylitearm.default_config_path()` = `config/litearm.yaml`）⇒ **就是这份**。
#:   ⚠ 当初我只比对了 `speed_limit`/`accel_limit`（所有 yaml 都一样）就**想当然**把 K/B
#:   也记成了 balanced 的值。**教训：跨仓取"真值"必须把那份文件打开看到数字本身。**
#:
#: 与**写固件参数**那条已证伪的路**完全不同的通道**：
#:     写 `mit_kp`/`mit_kd`      → **固件全局参数** ⇒ `movej` 也用 ⇒ 改坏 `movej`（真机踩过）
#:     `send_mit_all(kp=,kd=)`   → **只对这一帧生效** ⇒ `movej` **完全不受影响** ✓
#:
#: 分配理由（配置原文）：大关节 J1~J4 承重多、刚度给大；腕部 J5~J7 适中防啸叫。
SETUP_K = [60.0, 60.0, 60.0, 60.0, 40.0, 40.0, 40.0]
SETUP_B = [1.0, 1.0, 1.0, 1.0, 0.8, 0.8, 0.8]

#: `joint_follow.engage()` 的托举增益（`engage_kp=15.0, engage_kd=0.8`）——
#: ⚠ **比跟随增益软得多**（跟随是 60/40）：接管瞬间"轻轻接住"，而不是猛拉过去。
ENGAGE_KP = 15.0
ENGAGE_KD = 0.8


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

    ⚠ **用 `movej`（会把固件切回 `MOVE_J` 模式）** —— 这正是收尾想要的：交给固件的
    S 曲线 + `ht_on` 接管。⛔ **绝不 `disable()`**：失能会让臂在自重下自由落体。
    """
    st = arm.get_state(refresh=True).value
    if st is None:
        raise RuntimeError("收尾取不到状态帧")
    arm.movej(list(st.q), speed=0.3)


def _q_meas(arm) -> List[float]:
    """读**实测**关节角。⚠ `refresh=False` 走 SDK 缓存（实测 **0.001 ms**）——
    `refresh=True` 实测 **10 ms**，进了循环就等于把环频钉死在 100 Hz。"""
    return _q_dq_meas(arm)[0]


def _q_dq_meas(arm):
    """读**实测**位置与速度 `(q, dq)` —— **同一帧缓存，不多花一次往返**。

    ⚠ `G(q)` 与限位墙 `wall.tau(q, dq)` 都必须吃**实测**（照搬 `joint_follow.step()`：
    它先 `read_q_dq()`、再 `compute_tau_ff(q, dq)`），**不能拿指令值代替**。
    """
    st = arm.get_state(refresh=False).value
    if st is None:
        raise RuntimeError("状态帧取不到（链路静默）")
    return list(st.q), list(st.dq)



#: `joint_limit_wall.firmware_kd_extra`（`litearm.yaml:266` = 0.8）——
#: 进墙区的那几轴把 `kd` 加上它再下发（照 litearm-server 的 `joint_follow.py:306-312`）。
#: ⚠ 这是**纯 PC 侧**的加法，固件不需要知道。
WALL_FW_KD = 0.8
#: 叠加后 kd 的上限（达妙 MIT kd 硬上限，与 server 同值）。
WALL_FW_KD_CAP = 5.0


def _send_joint_follow(arm, q_cmd, dq_cmd, kp, kd, wall=None) -> None:
    """**照搬 `joint_follow.step()` 的最后那一步**：

        τ_ff = clamp(G(q_meas) + wall, ±tau_max)   # ← **固件算**（本函数不发）
        joint_follow(q_cmd, dq_cmd, K, B)          # 帧里只有这 4 组

    ⚠⚠ **`G(q)` 必须由 PC 侧送**：固件对 MIT 透传**永不叠加内置前馈**
    （`control_loop.c` 的 gating 注释原话：「`move_mit` / `move_js+tau_ff` 永不叠加内置」）
    ⇒ **不送 G 就是没有重力补偿，臂会垂。**

    ⚠⚠ **`wall` 是位置护栏的第二道**（第一道是 `clamp_to_limits` 把**目标**钳进限位）。
    只有目标钳位不够：从臂带着柔性去追一个**恰好贴在边界上**的目标，实测位置会**冲过去**
    —— 越界 >0.10 rad（或已使能时 >0.05）就**锁存 `joint_fault`**，该轴掉力并连带报
    `FB_STALE`。真机实证（2026-09-28）：断轴先是 J4、换增益后变成 **J2+J3** ——
    **轴会变** ⇒ 是"撞软限位"而不是某个电机坏了。墙在距限位 `margin` 处就给**排斥力矩**，
    让它**减速**而不是撞上去（`joint_limit_wall`，逐字移植，先前因 `move_js` 无力矩通道而未接线）。

    ⚠ 指令与实测**是两组不同的值**：`q_cmd/dq_cmd` 下发给电机；`G` 与墙吃 `q_meas/dq_meas`
    （照搬 server）。实测读缓存 ⇒ **不额外花往返**。

    ⚠ 代价：`get_gravity` 是一次往返（实测 **3.333 ms** = 一个固件 tick），
    `send_mit_all` 又一次 ⇒ **每拍 2 次往返 ⇒ ~150 Hz**。这是 MIT 路线的固有开销。
    """
    kd_sent = [float(x) for x in kd]
    if wall is not None:
        # 墙区叠加固件阻尼（照 server）：**只为在墙区的那几轴**抬高 kd。
        # ⚠ 用**实测** q 判墙区 —— 与固件算墙力用的是同一个量。
        q_meas, dq_meas = _q_dq_meas(arm)
        zone = wall.wall_zone_mask(q_meas)
        if any(zone):
            for i in range(N_JOINTS):
                if zone[i]:
                    kd_sent[i] = min(kd_sent[i] + WALL_FW_KD, WALL_FW_KD_CAP)
    # ⚠ 只发 q/dq/K/B —— **`τ_ff` 由固件算**（`G(q_meas) + wall(q_meas, dq_meas)`，
     #   照 server 的 `compute_tau_ff`）。⇒ 每拍 **1 次往返**（原来是 2 次）。
    arm.joint_follow(list(q_cmd), list(dq_cmd), list(kp), kd_sent)


def follow(arm, target_provider: Callable[[], Optional[Sequence[float]]],
           K=None, B=None, speed_limit=None, accel_limit=None,
           engage_sec: float = DEFAULT_ENGAGE_SEC, hz: float = DEFAULT_HZ,
           should_stop: Callable[[], bool] = lambda: False,
           duration_s: Optional[float] = None, wall=None) -> bool:
    """从臂跟随环 —— **逐条照搬 `joint_follow`**（`pylitearm/control/joint_follow.py`）。

    `target_provider()` 返回**目标关节角序列**；返回 `None` 时**保持上一拍的
    `q_cmd/dq_cmd` 不动**（照搬 `joint_follow` 那一支 —— "首帧到达前原地不动"）。

    ## ⚠⚠ 执行器是 `send_mit_all`（MIT 透传），**不是** `move_js`

    两条路的控制律**形式相同、参数差一个数量级** —— 这就是手感的全部差别：

    | | `joint_follow`（本函数照搬的） | `move_js`（旧路线，已弃） |
    | --- | --- | --- |
    | `K` | **25**（随帧下发） | **400**（固件 `mit_kp`，改不了） |
    | `B` | **0.5**（随帧下发） | **11**（固件 `mit_kd`+`kd_extra`，改不了） |
    | 重力 `G` | **PC 送** | 固件内置 |
    | 每拍往返 | 2 次 ⇒ **~150 Hz** | 1 次 ⇒ 250 Hz |

    `move_js` 的 K/B 是**固件出厂参数，没有随帧通道**；唯一能改的办法是写固件参数，
    而那条路已证伪（动态跟随断轴 + 连带改坏 `movej`）。⇒ **要 server 的手感就得换执行器。**

    ⚠ 用户裁决 2026-09-28：**先试纯粹的 150 Hz**，`G` 不降频（降频能回到 250 Hz，
    但那是下一步的事，先把"对不对"验了再谈"快不快"）。
    """
    try:
        sp = list(speed_limit or DEFAULT_SPEED_LIMIT)
        ac = list(accel_limit or DEFAULT_ACCEL_LIMIT)
        dt_nom = 1.0 / max(hz, 1.0)
        kp = [float(x) for x in (K if K is not None else SETUP_K)]
        kd = [float(x) for x in (B if B is not None else SETUP_B)]

        # ── prime：托住实测位姿（`joint_follow.prime`）──
        q_cmd = _q_meas(arm)
        dq_cmd = [0.0] * N_JOINTS
        q_target = list(q_cmd)
        _send_joint_follow(arm, q_cmd, dq_cmd, kp, kd, wall)

        # ── engage：`engage_sec` 内用**低刚度**托住（`joint_follow.engage`）──
        if engage_sec > 1e-6:
            q_ref = _q_meas(arm)
            kp_e = [ENGAGE_KP] * N_JOINTS
            kd_e = [ENGAGE_KD] * N_JOINTS
            t_end = time.monotonic() + engage_sec
            while time.monotonic() < t_end and not should_stop():
                _send_joint_follow(arm, q_ref, [0.0] * N_JOINTS, kp_e, kd_e, wall)
                time.sleep(dt_nom)

        # ── start：指令/目标都初始化成【当前实测】──
        q_cmd = _q_meas(arm)
        dq_cmd = [0.0] * N_JOINTS
        q_target = list(q_cmd)

        base = time.monotonic()
        next_tick = base + dt_nom
        # ⚠ 固件状态监视的上一拍快照（见循环里那段）
        _prev_fw = None

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
                _send_joint_follow(arm, q_cmd, dq_cmd, kp, kd, wall)
            except Exception:                        # noqa: BLE001
                # ⚠ MIT 透传**没有** `move_js` 那条「目标 ≠ 实测位姿且 `dq == 0` 就拒帧」
                #   的限制（`move_js` 路线因此才需要「托住实测位姿」的退路）。
                #   走到这里就是链路/帧本身出问题 ⇒ **直接受控接管并上抛**，不再硬撑。
                log.exception("send_mit_all 失败，受控接管")
                hold_at_current(arm)
                raise

            # ── 固件状态监视：**一变就记一行**（读缓存，实测 0.001 ms，不进循环开销）──
            # ⚠⚠ 故障的**先后顺序**只能从这里看出来。前几次排查我都是**杀完进程**才查状态，
            #    看到的是"结果"，还混进了我自己 kill 造成的 `WD_TRIPPED` —— **永远抓不到现场**。
            #    这一段让故障**发生的那一拍**就留下 `flags`/`joint_fault`/`err` 与跟踪误差。
            _snap = arm.get_state(refresh=False).value
            if _snap is not None and hasattr(_snap, "joints"):
                _cur = (tuple(getattr(_snap, "flag_names", ()) or ()),
                        int(getattr(_snap, "joint_fault", 0) or 0),
                        tuple(int(j.err) for j in _snap.joints))
                if _cur != _prev_fw:
                    _e = max(abs(a - b) for a, b in zip(q_cmd, _snap.q))
                    _dq = [round(float(v), 3) for v in getattr(_snap, "dq", [])]
                    _vmax = [2.0, 2.0, 1.75, 1.75, 2.0, 2.0, 2.0]
                    _over = [f"J{i+1}:{abs(_dq[i]):.2f}>{_vmax[i] * 1.5:.2f}"
                             for i in range(min(len(_dq), len(_vmax)))
                             if abs(_dq[i]) > _vmax[i] * 1.5]
                    log.warning(
                        "⚠ 固件状态变化 flags=%s joint_fault=0x%X err=%s  "
                        "最大跟踪误差=%.4f rad  dq=%s%s  q=%s",
                        list(_cur[0]), _cur[1], list(_cur[2]), _e, _dq,
                        ("  ⚠ 超速轴: " + "、".join(_over)) if _over else "",
                        [round(v, 3) for v in _snap.q])
                    _prev_fw = _cur

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
