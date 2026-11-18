"""从臂伺服环 —— **逻辑照 litearm-server 的 `joint_follow`，但环本身落在固件里**。

litearm-server 的从臂是 `joint_follow` → `send_mit(K, B, q_cmd, dq_cmd, G(q))`：
K/B **随帧下发**、重力 **PC 本地算**。两条路的分歧**只在"这一环落在谁身上"**：

    server :  PC ──SocketCAN──▶ 电机               （中间无固件；K/B 与 G 都由 PC 送）
    本实现 :  PC ──USB CDC──▶ STM32 ──CAN──▶ 电机   （伺服环在固件的 300 Hz 里）

⇒ 本仓发的是 `arm.joint_follow(q, dq, K, B)`（`CMD_JOINT_FOLLOW 0x08`）：**帧里没有
`tau`** —— 前馈 `τ_ff = clamp(G(q_meas) + wall, ±tau_max)` 由固件每拍自己算（照
`joint_follow.compute_tau_ff`），**省掉 PC 的 `get_gravity` 往返**。

## 为什么必须把环搬进固件

CDC 每拍有 ~6.7 ms 往返延迟，而 `kp` 弹簧的**控制层瞬态**会让实测 `dq` 冲过
`vel_max × 1.5` ⇒ 固件锁存 `joint_fault` ⇒ **停发该轴控制帧** ⇒ 达妙电机"收帧才回
状态" ⇒ 该轴静默 ⇒ 80 ms 后报 `FB_STALE`（次生现象）。**根因是这个"判了并且锁存"的
动作**，而 server 靠 `assert_operational(measured_overspeed_factor=inf,
skip_position=True)` **主动豁免**那两条判据 —— 固件没有这个开关，故给它加了一条专用
通道（豁免范围逐条限定，见 spec §3.4；通用 `move_mit_all` 路径**一条都不放宽**）。

## 与 `joint_follow` 的差异

| # | `joint_follow` | 本实现 | 处置 |
| --- | --- | --- | --- |
| ① | K/B **随帧下发** | **同**（`CMD_JOINT_FOLLOW` 的 `kp`/`kd` 字段） | ✅ 一致 |
| ② | 力矩通道（限位墙叠在 `tau_ff`） | **同**（固件把 `law_wall` 叠进 `τ_ff`） | ✅ 一致 |
| ③ | 位置护栏 | PC `clamp_to_limits` **+** 固件 `law_wall` | ✅ 更严 |
| ④ | 超速/位置判据**不判** | 该通道上**豁免那两条**（其余判据一条不动） | ✅ 一致 |

⚠ **别用 `move_js` 代替**（那是已弃的旧路线）：它的 K/B 是**固件出厂参数**
（`mit_kp`=400、`mit_kd`+`kd_extra`=11），**没有随帧通道** —— 刚度差 16 倍、阻尼差
22 倍，手感**不可能**一样。真机判据：「跟随太慢，有明显的延迟」。唯一能改它的办法是
写固件参数，而**那条路已证伪**（见 `SETUP_K` 上面那段）。

**参数真值**、**watchdog**、**对齐**、**钳位** 仍逐条照 litearm-server，见 spec §5、§9.1。

## 参考生成：直通 + 每拍硬限速（2026-11-18 起）

从臂把主臂目标**逐拍直通**下发，只保留一条**硬限速**：单拍位移不超过
`speed_limit·dt`（与固件 `s_jf_vel_max` 逐值相同）。**不再经过** `safety.slew_target`
——它里面的加速度限幅与「制动距离」分支会在匀速拖动时让参考恒定落后主臂
`v²/(2a)`（J1 在 2 rad/s 下实测 0.137 rad），而正常手拖速度远低于 `speed_limit`
⇒ 直通后这一段滞后为 0。

⚠ 速度上限仍有**两层**：本条 PC 侧硬限速 + 固件 `slew_linear`。正常情况下两层都是
**原样通过**（主臂速度低于上限）；超过上限时谁小谁说了算，不会被任何一层"平滑"。
⚠ 位置护栏（`clamp_to_limits` + 固件 `law_wall` + 固件 `q_meas ± 0.20` 窗口）一条没动。

## 指令重建滤波（默认开启）

主臂有效帧率 ≈98.5 Hz，从臂 250 Hz 消费 ⇒ 保持后的指令是阶梯，30–120 Hz 的能量
会激起末端抖动。默认 `hybrid`：J1–J4 二阶 20 Hz 低通，J5–J7 One-Euro 自适应低通
（抑制腕部 6–10 Hz，1–5 Hz 意图动作的 30–40 ms 链路延迟不变）。`LITEARM_CMD_FILTER`
可切 `hybrid|lp20|lp25|off`；腕部参数见 `_oe_params()` / README。
"""
from __future__ import annotations

import logging
import math
import os
import time
from dataclasses import dataclass, field
from typing import Callable, List, Optional, Sequence, Tuple

from .safety import clamp_to_limits
from .wire import N_JOINTS
from .wire import decode_teleop as wire_decode

log = logging.getLogger("liteteleop.servo")

__all__ = [
    "ALIGN_SPEED", "ALIGN_TIMEOUT", "ALIGN_WARN_DELTA",
    "align_to_master", "DEFAULT_PAYLOAD_MASS", "DEFAULT_PAYLOAD_COM",
    "read_payload", "apply_payload",
    "SETUP_K", "SETUP_B", "ENGAGE_KP", "ENGAGE_KD",
    "DEFAULT_SPEED_LIMIT",
    "DEFAULT_ENGAGE_SEC", "DEFAULT_HZ", "hold_at_current", "follow",
]

# ── 参数真值 ────────────────────────────────────────────────────────────────
#: **照抄 litearm-server 的 `litearm.yaml` 的 `joint_follow.speed_limit`**（2026-09-29）。
#:
#: ## 为什么现在能照抄了（这天之前不能）
#:
#: 这份值来自一个**没有固件安全层**的系统：server 走 CAN **直连电机**，且
#: `joint_follow` 每步都主动豁免了检查（`measured_overspeed_factor=inf` +
#: `skip_position=True`）。而我们的固件**有**独立的超速判据
#: （`|dq| > jp->vel_max × 1.5` 连续 5 拍 ⇒ 锁存 `joint_fault` ⇒ 停发该轴控制帧
#: ⇒ 电机静默 ⇒ 80 ms 后 `FB_STALE`），而且固件的 `slew_linear` 原先按
#: `jp->vel_max` 推进参考 —— 那张表是 **`movej` 的"满速语义"**，不是"跟随该跑多快"。
#:
#: ⇒ 那天之前照抄拿不到速度，只会得到「**从臂永远在追**」：真机实测滞后
#:   **0.35 rad（20°）**、J2 的指令-实测差值一度 0.31 rad。表现就是用户报的
#:   「快速拖动时到末端过冲、再回拉」—— 那不是阻尼不够，是**追不上**。
#:
#: ## 两处必须同时改才生效
#:
#:     PC 侧（本表）              决定 `follow()` 每拍最多推进多少（硬限速）
#:     固件 `s_jf_vel_max`        决定 `slew_linear` 放行多快（**只对 joint_follow 会话**）
#:
#: ⚠ **固件那张表是逐值照抄本表的**（`control_loop.c` 的 `s_jf_vel_max`）——
#:   改这里必须同步改那里：两级串联，**谁小谁说了算**。正常手拖（1–2 rad/s）低于
#:   两者 ⇒ **逐拍原样通过，不产生滞后**；超过上限时才被限住。
#: ⚠ 通用路径（`move_j` / `move_js` / `move_mit_all`）仍受 `jp->vel_max` 约束，
#:   一点没变 —— 这是刻意的，见 spec §5 第 4 条（通用路径不得被削弱）。
#: ⚠ 位置护栏（`clamp_to_limits` + 固件 `law_wall`）**仍然保留**：server 也保留它，
#:   关掉会让从臂撞机械限位（J4 上端只有 2°）。
DEFAULT_SPEED_LIMIT = [2.8, 3.4, 5.0, 5.0, 10.0, 8.0, 13.0]
DEFAULT_ENGAGE_SEC = 0.3

#: **从臂伺服环**的节拍频率（`follow()` 用 `sleep` 主动对齐到点，不是"能跑多快"）。
#: `dt_nom = 1/hz` 会**直接进每拍硬限速** ⇒ 这个数**必须等于实际循环周期**，
#: 否则限速按错周期算（每拍上限 = `speed_limit × dt_nom`）。
#:
#: ⚠ **S4 之后每拍只 1 次往返**（`CMD_JOINT_FOLLOW` 一次下发；`get_gravity` 已由固件
#:   内算取代）⇒ 实测单往返 3.333 ms ⇒ 硬件上限 ~300 Hz。取 **250**，与 litearm-server
#:   的 `control_loop_hz` 同值。
#: ⚠ 曾经用的 150 是 MIT 路线（每拍 2 次往返）时代的**上限**；省掉一次往返之后它只是
#:   一个主动的保守值 —— 2026-09-29 提到 250。
#: ⚠⚠ **别凭上面的算术再往上加**：Python 那圈还有 slew / 钳位 / 状态监视的开销。
#:   `follow()` 收尾会打印**实测**节拍与单拍净耗时，实际跟不上设定时会告警 —— 以那个为准。
#: ⚠ 与主臂的 `pub_hz = 200` 是两个不同的数，别混。
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

#: ⛔⛔ **历史：已证伪的「写固件参数」路线**（2026-09-28 用户裁决停用）。
#:
#: ⚠ **以下是过程记录，不是当前实现** —— 当前实现见本段末尾的 `SETUP_K` / `SETUP_B`
#:   （K/B 随 `0x08` 帧下发）。留这段只为**别再走回去**。
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
#: **从臂跟随增益 —— 照抄 litearm-server 默认配置 `litearm.yaml` 的 `joint_follow:` 段。**
#:
#: ⚠⚠ 与**上面那段"写进固件"的路线完全不同**（那条已证伪），两者是**不同通道**：
#:
#:     写 `mit_kp`/`mit_kd`                    → **固件全局参数** ⇒ `movej` 也用 ⇒ 改坏 `movej`（真机踩过）
#:     `CMD_JOINT_FOLLOW(0x08)` 的 `kp`/`kd`   → **只对这一帧生效** ⇒ `movej` **完全不受影响** ✓
#:
#: ## 为什么必须换执行器，才拿得到 server 的手感
#:
#: 三条路控制律**形式相同、参数差一个量级**：
#:
#:     joint_follow（server）   τ =  60·(q_cmd−q) + 1.0·(dq_cmd−dq) + G(q)   ← J1~J4
#:     move_js      （旧路线）   τ = 400·(q_cmd−q) +  11·(dq_cmd−dq) + G(q)
#:                                     ↑ 刚度 6.7×      ↑ 阻尼 11×
#:     本仓（2026-09-29 起）     τ = 200·(q_cmd−q) + 3.0·(dq_cmd−dq) + G(q)   ← 见下方 K/B
#:
#: `move_js` **没有 K/B 通道**（那两个是固件出厂参数）⇒ 手感**不可能**一样。
#: 真机判据：旧路线用户报「跟随太慢，有明显的延迟」。
#:
#: ## 代价（S4 起已不再是代价）
#:
#:     旧 MIT 路线（`send_mit_all` + PC 算 G）  每拍 **2 次往返** ⇒ ~150 Hz
#:     现在（`CMD_JOINT_FOLLOW`，G 由固件算）    每拍 **1 次往返** ⇒ 与 server 同级
#:
#: ⇒ 固件替 PC 算掉了 `G(q)`（`dyn_gravity` + `law_wall`），那一次往返**省掉了**。
#: ⚠ 但**通用透传路径**（`move_mit` / `move_js+tau_ff` / `move_mit_all`）**仍然是"永不
#:   叠加内置"**（`control_loop.c` 的原话）—— 它们不享受这个待遇，G 仍须 PC 送。
#:   只有 `0x08` 这条**专用通道**由固件补 G。
#: **从臂跟随增益 —— 基线取自 litearm-server 默认配置 `litearm.yaml` 的 `joint_follow:` 段，
#: 但 2026-09-29 起**整体放大**，不再是逐字照抄（依据见下）。**
#:
#: ## 为什么必须偏离 server（本仓第二处有意偏离；第一处是固件侧速度表 `s_jf_vel_max`）
#:
#: server 的 `K=60 / B=1.0` 是**为慢速遥操定的**。真机把它推到快速拖动下**必然过冲** ——
#: 而且 **server 自己也有**：用户 2026-09-29 复述「150Hz 的也有」。
#: ⇒ 「严格照抄 server」与「丝滑不过冲」这两个目标，在快速拖动下**互斥**，只能二选一。
#:   用户的目标是后者（原话：「从臂跟随得非常丝滑，并且不过冲不抖动」）。
#:
#: ## 定量依据（2026-09-29 真机，逐轴统计）
#:
#:     最大滞后 0.2919 rad       J2 过冲 −0.2516 rad       比值 0.86
#:
#: 滞后与过冲**几乎一比一** —— 这是欠阻尼二阶系统的标志（ζ≈0.15 ⇒ 过冲率 ~0.8）。
#: 机理：`q_cmd` 一停，从臂身后还欠着 0.29 rad 没还，它带着速度扑上去、冲过头再被拉回。
#: 用户原话「有点过冲然后回拉的感觉」就是那个回弹。
#:
#: ⇒ 绝对过冲 = **滞后 × 过冲率**，两个因子各压一路：
#:
#:     滞后   ∝ 1/K（驱动力矩不够 ⇒ 同样的力矩需要更大的位置误差）  ⇒ **提 K**
#:     过冲率 = f(ζ)，  ζ = B / (2√(K·J))                          ⇒ **同步提 B 保住 ζ**
#:
#: ⚠ **实测印证**：只把 J2 的 B 由 1.0 翻到 2.0（ζ 0.075→0.15），用户报
#:   「比以前似乎好一点点」—— 正是预测的那 ~20%（过冲率 0.79→0.62）。模型成立。
#:
#: ## ⚠ `B` 这条路走不远：撞固件上限
#:
#: 固件 `control_loop.c:45` 的 `MIT_KD_MAX = 5.0f` 是**线上 kd 的硬上限**，
#: `0x08` 帧里的 `kd` 会被它**静默钳住**。要让 J2 的 ζ 到 0.7 需要 B≈6~8 ⇒ **装不下**。
#: ⇒ 只能靠 `K` —— 它的对应上限是 `MIT_KP_MAX=500`，而 200 离它还很远。
#:
#: ## 取值（与 server 的对比）
#:
#:     J1~J4:  60 → 200（×3.33）        J5~J7:  40 → 80（只 ×2，追得更少）
#:
#: ⚠⚠ **J5~J7 追得更少 —— 参照系是【固件出厂调优值】，不是 server 的比例**（2026-09-29 第 2 轮订正）：
#:   第 1 轮我把腕部按 server 的同比例放大到 133（`=40×200/60`），**参照系选错了**。真正的锚点
#:   是 `defaults.c` 的出厂值（注释写明「非中空版真机重调」）。两组并排：
#:
#:     轴        电机        出厂 mit_kp   我们(第1轮)    K / 出厂
#:     J1~J2     DM6248P     400          200            0.50×
#:     J3~J4     DM4340      300          200            0.67×
#:     J5~J7     DM4310       50          133           **2.66×**  ← 反向偏离
#:
#: ⚠ **腕部不能按同比例放大，三条独立证据指向同一件事**：
#:   ① `defaults.c` 给腕部三轴的 `kd_extra` **专门设为 0**（J1~J4 是 6.0），台账 #37 原话：
#:      「腕部必须 0：**`kd_extra/J` 过快会反向激发**」—— 分母是惯量 J，而腕部 J 小；
#:   ② 腕部 `tau_max` 只有 **10 Nm**（J1/J2 是 78）⇒ 同样的 kp 对应高得多的带宽；
#:   ③ 腕部电机 DM4310 是三种里**最小**的。
#: ⚠ 第 1 轮腕部放大后 J5 的过冲**没降反升**（0.0533 → 0.0594），与 J1~J4 的普遍下降相反
#:   ⇒ 本轮把腕部往回拉，**实验判据见 `SETUP_B` 注释末尾**（双向判据）。
#: ⚠ 与 server 的 `move_js` 旧路线（`K=400`）相比，200 仍是**较保守**的一档；
#:   若本轮 K 有效但不够，下一步往 300~400 走（那段是真机跑过的）。
#:
#: ⚠⚠ **别抄 `litearm_balanced.yaml`**：那份是 `[25]×7 / [0.5]×7`，而它自己的历史注释
#:   写着「从 25 全一律**提高**以改善主从遥操的**滞后/追不上**手感（2026-08-13）」
#:   ⇒ **25 就是已知会"滞后、追不上"的那一档**。真机实测（2026-09-28）复现了同一现象：
#:   用户报「跟随性比较差，明显延迟，而且会很软」。
#:
#: ⚠ server 的取值链（留档备核）：`TeleopManager(K=None)` → `arm.joint_follow(K=None)` →
#:   `from_params` → `cfg["joint_follow"]["K"]`；而 server **没有传 config** ⇒
#:   走 SDK 默认（`pylitearm.default_config_path()` = `config/litearm.yaml`）。
#:   ⚠ 当初我只比对了 `speed_limit`/`accel_limit`（所有 yaml 都一样）就**想当然**把 K/B
#:   也记成了 balanced 的值。**教训：跨仓取"真值"必须把那份文件打开看到数字本身。**
SETUP_K = [200.0, 200.0, 200.0, 200.0, 80.0, 80.0, 80.0]

#: 阻尼 —— 与 `SETUP_K` **同步放大**，理由是 ζ 的分母带 √K：
#:
#:     ζ = B / (2√(K·J))     ⇒  B 放大 3×、√K 只放大 1.83× ⇒ **ζ 净升 ~1.64×**
#:
#: ⚠ 若只提 K 不提 B，ζ 会随 √K **下降** ⇒ 过冲率变差，把提 K 的收益吃掉一部分。
#:   两路必须**同步**动。
#:
#:     J1、J3、J4   1.0 → 3.0     （第 1 轮起，未动）
#:     J2           2.0 → 5.0     ⚠ **顶到固件上限**（`MIT_KD_MAX`）
#:     J5~J7        0.8 → 1.5     （第 1 轮曾是 2.4，第 2 轮往回拉 —— 理由见下）
#:
#: ⚠ **腕部（J5~J7）的 B 为什么往回拉**：第 1 轮按与 J1~J4 的同一逻辑放大到 2.4，但
#:   参照系错了（详见 `SETUP_K` 上方那段：出厂 `mit_kp` 腕部只有 50，不该按 server 比例追）。
#:   ⚠ 注意这在 ζ 上**是退让**：K 133→80 配 B 2.4→1.5 ⇒ ζ 比从 1.64× 降到 1.326×。
#:   **这是有意的取舍** —— 若 J5 的病根是"腕部被增益激发弹性模态"，那 ζ 这条标尺本就不适用，
#:   继续按它加阻尼只会加重症状。故本轮**宁可在 ζ 上退一步，也要把腕部增益拉回来**，
#:   用真机数据判定是哪一种（判据见本块末尾）。
#:
#: ⚠ **J2 为什么直接写 5.0 而不是等比的 6.0**：
#:   ① 6.0 会被固件**静默钳成** 5.0 —— 表上"看起来"满足等比、实际值与意图不符，
#:      属静默失败；宁可写真实值让这张表可读；
#:   ② J2 是唯一仍显著过冲的轴（肩部托着整条臂 ⇒ 负载惯量最大 ⇒ ζ 最小），
#:      按 2.0→5.0 实算它本轮 ζ 只提升 1.37×，**已低于其余轴的 1.64×** ——
#:      再往下给就是在让病灶轴掉队。
#:
#: ⚠ 为什么只加 B、不减 K: 减 K 会变软变拖沓（用户明确抱怨过「很软」）。
#: ⚠ **不需要烧固件** —— `B` 随 `0x08` 帧每拍下发，重启从臂即生效。
#:
#: ⚠ **第 1 轮的验收判据已成立**（滞后 0.2919 → 0.1325，↓55%；J2 过冲 ↓86%）。
#: ⚠ 别拿绝对过冲值与**改口径前**那一轮直接相减：统计口径在那一轮改过
#:   （`follow()` 只在"参考已停住"时才计过冲），跨口径的数字不可比。
#:
#: ⚠⚠ **第 2 轮（2026-09-29）的实验判据 —— 双向判据，两个方向都有信息**：
#:   第 1 轮后 **J5 成了全表最大的过冲轴**（−0.0594，其余 ≤0.042）。而它的三轮轨迹
#:   **对增益不敏感、甚至反向**：
#:
#:     J5 过冲   0.0761   →   0.0533   →   0.0594
#:               150 Hz       250 Hz       250 Hz + K/B 放大（40→133、0.8→2.4）
#:                            ↑ 提采样率 −30%     ↑ 提增益 **+11%**
#:
#:   ⇒ 与 J2（对增益 −86%）**不是一类**。腕部更像被**增益激发了弹性模态**，而非欠阻尼。
#:   本轮把 J5~J7 的 K 133→80、B 2.4→1.5 往回拉（**J1~J4 一动不动**）：
#:
#:     · **J5 降** ⇒ "腕部不该给高增益"成立 ⇒ 沿这条继续收（甚至照出厂的 50/2.5）；
#:     · **J5 升** ⇒ 它是欠阻尼那一类（降 B 就是降 ζ） ⇒ 回到高 B 路线，但**上限被
#:       固件 `MIT_KD_MAX=5.0` 封顶** ⇒ 届时要动的是固件（`kd_extra` 走 tau 域）。
#:     · **J5 纹丝不动**（两种都解释不了） ⇒ 是**腕部机械弹性**，轨迹层够不着 ——
#:       与 §C 已有的「J4-J7 减速末小振铃 = 弹性模态」同一条，**不必再追**。
#:   ⚠ 无论哪个方向，本轮 **J1~J4 不动** ⇒ 它们的数字应与上一轮基本一致，可作对照。
#: ⚠⚠ **腕部 B 保持 1.5 —— 2026-11-18 傍晚的 1.5→3.0 试验已回退**：同一轮真机数据里，
#: 从臂**实测/自身指令**的同源比值（4–6 rad/s、6–20 Hz）由 J5 1.09× / J6 1.19× / J7 0.94×
#: 变成 1.37× / 1.40× / 1.28×，20–45 Hz 放大到 1.4–2.8× ⇒ 腕部加阻尼激发了弹性/高频模态
#: （`defaults.c` 里腕部 `kd_extra=0` 的注释即此意）。高速抖动的病根在**指令**里：
#: 高速段从臂实测对**自身指令**的 6–10 Hz 相干性 0.97–0.99、增益 ≈1.0 ⇒ 压它要压指令
#: 高频（`LITEARM_OE_MAX_HZ`），不是加 `B`。详见 `litearm-teleop-command-filter-tuning`。
SETUP_B = [3.0, 5.0, 3.0, 3.0, 1.5, 1.5, 1.5]

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

    ⚠ 吃这个实测值的消费者有两个：**本模块**的 `wall_zone_mask`（判墙区以抬高 `kd`），
    以及**下面的 `kd_sent` 计算**。都必须是**实测**（照搬 `joint_follow.step()`：
    它先 `read_q_dq()`、再 `compute_tau_ff(q, dq)`），**不能拿指令值代替**。
    ⚠ 固件那份实测由它**自己**在 300 Hz 里采（`g_arm.joint[].q`），不经本函数。
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

    ⚠⚠ **`G(q)` 由固件算，不从 PC 送** —— 这正是 `0x08` 这条接口存在的理由：
    `control_loop.c` 在 `s_jf_ff` 会话里每拍自己跑 `dyn_gravity(q_meas)` + `law_wall`，
    再钳到 `tau_max`（照 server 的 `compute_tau_ff`）⇒ **省掉 `get_gravity` 那次往返**。

    ⚠ **别把这条专用通道的待遇记到通用路径上**：`move_mit` / `move_mit_all` /
    `move_js+tau_ff` 那几条**永不叠加内置前馈**（`control_loop.c` 的 gating 原话），
    它们的 `tau_ff` **仍必须由 PC 送**。

    ⚠⚠ **限位护栏共三层**：① `clamp_to_limits` 把**目标**钳进限位；
    ② 固件 `law_wall` 在距限位 `margin` 处给**排斥力矩**（叠进 `τ_ff`）；
    ③ 固件 `slew_linear` 用 `vel_max` 限速。

    ⚠ 本通道上固件**豁免**「位置越界」与「超速」两条**锁存**判据（照 server 的
    `skip_position=True` + `measured_overspeed_factor=inf`）⇒ 越界不再锁存掉力。
    但豁免的是「发现越界就锁存掉力」这个**动作**，不是「越界」本身 —— 所以①②③必须都留着。

    ⚠ 指令与实测**是两组不同的值**：`q_cmd/dq_cmd` 下发给电机；固件算 G 与墙吃
    `q_meas/dq_meas`（照搬 server）。实测读缓存 ⇒ **不额外花往返**。
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


#: 从臂指令重建低通 —— 把主臂 ~98.5 Hz 的零阶保持阶梯平滑掉（2026-11-18 实测）。
#: `LITEARM_CMD_FILTER=hybrid|lp20|lp25|off`；默认 `hybrid`（J1–J4 lp20，
#: J5–J7 One-Euro）。`lp20`/`lp25` 为全轴二阶；`off` 只用于 A/B 对照。
DEFAULT_FILTER_MODE = "hybrid"

#: 对齐后的起步追赶：主臂在对齐期间移动过时，先用这个速度追齐，避免按正常硬限速
#: （J1 2.8 … J7 13 rad/s）突然甩过去。误差进入 `CATCHUP_TOL` 后切回正常限速。
CATCHUP_SPEED = 0.5
CATCHUP_TOL = 0.02
CATCHUP_WARN_SECONDS = 5.0


class _AdaptiveLP:
    """One-Euro 式自适应一阶低通：截止频率 = min + beta·|dq|（逐轴）。

    用**主臂实测 dq**（不是微分保持后的 q）估计速度，避免把零阶保持的台阶
    误当成高速运动。速度估计本身再走 1 Hz 低通，抑制 `dq` 噪声。
    """

    def __init__(self, min_cut, beta, max_cut, knee, dt, d_cut=1.0):
        self.min_cut = float(min_cut)
        self.beta = float(beta)
        self.max_cut = float(max_cut)
        self.knee = max(0.0, float(knee))
        self.dt = float(dt)
        self.d_cut = float(d_cut)
        self.y = None
        self.speed_hat = 0.0

    def process(self, x, speed):
        x = float(x)
        if self.y is None:
            self.y = x
            self.speed_hat = abs(float(speed))
            return x
        a_d = 1.0 - math.exp(-2.0 * math.pi * self.d_cut * self.dt)
        self.speed_hat += a_d * (abs(float(speed)) - self.speed_hat)
        cut = self.min_cut + self.beta * max(0.0, self.speed_hat - self.knee)
        cut = min(self.max_cut, max(self.min_cut, cut))
        a = 1.0 - math.exp(-2.0 * math.pi * cut * self.dt)
        self.y += a * (x - self.y)
        return self.y


class _LowPass2:
    """二阶 Butterworth 低通（RBJ 双二次），逐轴状态；不依赖 scipy。"""

    def __init__(self, fc: float, fs: float):
        q = 1.0 / math.sqrt(2.0)
        w0 = 2.0 * math.pi * float(fc) / float(fs)
        alpha = math.sin(w0) / (2.0 * q)
        cw = math.cos(w0)
        a0 = 1.0 + alpha
        self.b0 = (1.0 - cw) / 2.0 / a0
        self.b1 = (1.0 - cw) / a0
        self.b2 = self.b0
        self.a1 = (-2.0 * cw) / a0
        self.a2 = (1.0 - alpha) / a0
        self._x1 = self._x2 = self._y1 = self._y2 = None

    def process(self, x, speed=None):
        x = [float(v) for v in x]
        if self._x1 is None:
            # 初值取第一拍输入 ⇒ 无启动瞬态
            self._x1 = list(x)
            self._x2 = list(x)
            self._y1 = list(x)
            self._y2 = list(x)
            return x
        y = [
            self.b0 * x[i] + self.b1 * self._x1[i] + self.b2 * self._x2[i]
            - self.a1 * self._y1[i] - self.a2 * self._y2[i]
            for i in range(len(x))
        ]
        self._x2, self._x1 = self._x1, x
        self._y2, self._y1 = self._y1, y
        return y


class _HybridFilter:
    """J1–J4 二阶 20 Hz 低通；J5–J7 用 One-Euro 自适应滤波。

    `j5_lp_hz`（>0 时）在 J5 的 One-Euro **之后**再串一级二阶低通 —— 见
    `_j5_lp_hz()` 的注释：腕部高速段的 10–45 Hz 残余靠 OE 压不下去。

    `j5_lp_only`（`LITEARM_J5_LP_ONLY`）把 J5 整个退回 **J1–J4 那条路**：只走
    固定二阶低通、不经过 One-Euro。用途是 A/B 用户反馈的那个观感 ——
    最早全轴 lp20 时 J5 只有偶发单帧抖动，换成 OE 后低速也开始抖。
    """

    def __init__(self, fc, fs, oe_params, j5_lp_hz=None, j5_lp_only=False):
        self.lp = _LowPass2(fc, fs)
        dt = 1.0 / float(fs)
        self.oe = [_AdaptiveLP(*oe_params, dt=dt) for _ in range(3)]
        self.j5_lp = _LowPass2(j5_lp_hz, fs) if j5_lp_hz else None
        self.j5_lp_only = bool(j5_lp_only)

    def process(self, x, speed=None):
        y = self.lp.process(x)
        if speed is None:
            speed = [0.0] * len(x)
        for j, i in enumerate(range(4, 7)):
            if i < len(x) and not (i == 4 and self.j5_lp_only):
                y[i] = self.oe[j].process(x[i], speed[i])
        if self.j5_lp is not None and len(x) > 4 and not self.j5_lp_only:
            y[4] = self.j5_lp.process([y[4]])[0]
        return y


def _oe_params():
    """`LITEARM_OE_MIN_HZ / BETA / MAX_HZ / KNEE`（带默认值）。

    `KNEE`（rad/s）是高速段也继续滤波的拐点：`|dq| ≤ KNEE` 时保持 `MIN_HZ`
    重平滑，超过后才按 `BETA` 放开。默认 6 rad/s。
    """
    def num(name, default):
        try:
            return float(os.environ.get(name, default))
        except ValueError:
            log.warning("环境变量 %s 不是数字 —— 用默认值 %s", name, default)
            return float(default)

    min_cut = max(0.2, num("LITEARM_OE_MIN_HZ", 2.0))
    beta = max(0.0, num("LITEARM_OE_BETA", 5.0))
    max_cut = max(min_cut, num("LITEARM_OE_MAX_HZ", 25.0))
    knee = max(0.0, num("LITEARM_OE_KNEE", 6.0))
    return min_cut, beta, max_cut, knee


def _j5_lp_hz() -> float:
    """`LITEARM_J5_LP_HZ`：J5 在 One-Euro 之后追加的二阶低通截止（Hz，0=关闭）。

    为什么只给 J5：类人臂的 J5 是腕部第一个轴，惯量小、又直接暴露在末端。
    真机（`rec-20261118-185235` / `rec-20261118-190315` 三条轨迹离线复算）里，
    同一段高速运动下 `指令 10–20 Hz` 用 OE 是 1.69 mrad，串一级 12 Hz 二阶降到
    1.04、`20–45 Hz` 从 1.23 降到 0.23；低速段不劣化（6–10 Hz 1.53→1.46）。
    ⚠ 别改成"按速度在 OE 与二阶之间切换/淡入"：两者相位不同，混合处会产生调制
    （实测 6–10 Hz 从 3.04 涨到 8.16 mrad），比不切还差。
    代价是意图滞后 +20 ms（12→32 ms）。
    """
    try:
        v = float(os.environ.get("LITEARM_J5_LP_HZ", 0.0))
    except ValueError:
        log.warning("环境变量 LITEARM_J5_LP_HZ 不是数字 —— 视为关闭")
        return 0.0
    return max(0.0, v)


def _j5_lp_only() -> bool:
    """`LITEARM_J5_LP_ONLY`：J5 只用二阶低通（与 J1–J4 同一条路），不走 One-Euro。

    用户反馈（2026-11-18）：最早全轴 lp20 时 J5 只有**偶发单帧**抖动；换成 hybrid 的
    One-Euro 之后低速也开始抖。机理上说得通 —— OE 低速截止只有 2 Hz（≈80 ms 滞后），
    手一起停就欠着滞后再追，1–3 Hz 上形成摆动；而 lp20 相位固定、不随速度变，
    起停时没有"追"的过程。代价是主臂 6–10 Hz 的内容会 1:1 通过（同源实测
    0.7–1.2 mrad），这正是当初引入 OE 要压的那一段。A/B 用，不是默认。
    """
    raw = (os.environ.get("LITEARM_J5_LP_ONLY", "") or "").strip().lower()
    return raw in ("1", "true", "yes", "on")


def _filter_spec(mode=None):
    """解析 `off|lpNN|hybrid`；返回 `(kind, fc, label)`。"""
    raw = os.environ.get("LITEARM_CMD_FILTER", DEFAULT_FILTER_MODE) \
        if mode is None else mode
    raw = (raw or "").strip().lower()
    if raw in ("", "off", "none", "0"):
        return "off", None, "off"
    if raw in ("hybrid", "oe"):
        return "hybrid", 20.0, "hybrid"
    if raw.startswith("lp"):
        try:
            fc = float(raw[2:])
        except ValueError:
            fc = -1.0
        if fc > 0.0:
            return "lp", fc, raw
    log.warning("未知的 LITEARM_CMD_FILTER=%r —— 按 off 处理", raw)
    return "off", None, "off"


def filter_mode_label(mode=None) -> str:
    """给日志用的低通描述：`off`、`lp20 (20 Hz)` 或 `hybrid (...)`。"""
    kind, fc, label = _filter_spec(mode)
    if kind == "off":
        return "off"
    if kind == "hybrid":
        mn, beta, mx, knee = _oe_params()
        j5 = _j5_lp_hz()
        if _j5_lp_only():
            extra = " + J5 只走二阶低通"
        else:
            extra = f" + J5 lp{j5:g} Hz" if j5 else ""
        return (f"hybrid (J1-J4 lp{fc:g} Hz + J5-J7 OE "
                f"{mn:g}/{beta:g}/{mx:g} Hz, knee {knee:g}{extra})")
    return f"{label} ({fc:g} Hz)"


def _split_target(got):
    """把 `target_provider` 的返回值拆成 `(q, dq 或 None)`。

    支持两种返回：7 个关节角；或 `(q, dq)` 二元组（`dq` 作为速度前馈）。
    非有限值一律拒（位置由调用方保证、速度坏值丢弃后回落到差分值）。
    """
    if (len(got) == 2
            and isinstance(got[0], (list, tuple))
            and isinstance(got[1], (list, tuple))):
        q_raw, dq_raw = got
    else:
        q_raw, dq_raw = got, None
    if len(q_raw) != N_JOINTS:
        raise ValueError(f"目标必须是 {N_JOINTS} 元素，收到 {len(q_raw)}")
    q = [float(v) for v in q_raw]
    if not all(math.isfinite(v) for v in q):
        raise ValueError("目标关节角含非有限值")
    dq = None
    if dq_raw is not None:
        if len(dq_raw) != N_JOINTS:
            raise ValueError(f"目标速度必须是 {N_JOINTS} 元素，收到 {len(dq_raw)}")
        dq = [float(v) for v in dq_raw]
        if not all(math.isfinite(v) for v in dq):
            dq = None                       # 坏速度丢弃：位置仍安全，前馈回落到差分
    return q, dq


def follow(arm, target_provider: Callable[[], Optional[Sequence[float]]],
           K=None, B=None, speed_limit=None,
           engage_sec: float = DEFAULT_ENGAGE_SEC, hz: float = DEFAULT_HZ,
           should_stop: Callable[[], bool] = lambda: False,
           duration_s: Optional[float] = None, wall=None,
           filter_mode: Optional[str] = None,
           catchup_speed: Optional[float] = CATCHUP_SPEED) -> bool:
    """从臂跟随环 —— 执行器与增益照搬 `joint_follow`，**参考生成改为直通 + 硬限速**。

    `target_provider()` 返回 7 个关节角，或 `(q, dq)` 二元组（`dq` 作为速度前馈）；

    返回 `None` 时**保持上一拍的位置目标**（"首帧到达前原地不动"），速度前馈在
    50 ms 内保持（容忍主臂 100 Hz 状态帧与 250 Hz 环之间的空隙），之后归零。

    ⚠ **不再经过 `safety.slew_target`**（2026-11-18）：它的加速度限幅 + 制动距离
    分支会在匀速拖动时让参考恒定落后主臂 `v²/(2a)`（J1 在 2 rad/s 下 0.137 rad）。
    现在只保留**每拍硬限速** `speed_limit·dt`（与固件 `s_jf_vel_max` 同值）：正常手拖
    速度低于该上限 ⇒ 参考**逐拍原样通过**；超过上限时才被限住（固件 `slew_linear`
    还会再限一次，两级串联、谁小谁说了算）。

    ## ⚠⚠ 执行器是 `arm.joint_follow`（`CMD_JOINT_FOLLOW 0x08`，**伺服环在固件里**）

    **不是** `move_js`（旧路线，已弃），也不是 `send_mit_all`（S4 之前的过渡执行器）。

    | | `joint_follow`（server，本函数照搬的） | 本实现 | `move_js`（旧路线，已弃） |
    | --- | --- | --- | --- |
    | `K` | 随帧下发 | **随帧下发**（`0x08` 的 `kp`） | **400**（固件 `mit_kp`，改不了） |
    | `B` | 随帧下发 | **随帧下发**（`0x08` 的 `kd`） | **11**（`mit_kd`+`kd_extra`，改不了） |
    | 重力 `G` | PC 算 | **固件算**（`dyn_gravity` + `law_wall`） | 固件内置 |
    | 每拍往返 | 1 次 | **1 次**（`0x08` 一次下发） | 1 次 |

    `move_js` 的 K/B 是**固件出厂参数，没有随帧通道**；唯一能改的办法是写固件参数，
    而那条路已证伪（动态跟随断轴 + 连带改坏 `movej`）。⇒ **要 server 的手感就得换执行器。**

    ⚠ 节拍当前是 **250 Hz**（`DEFAULT_HZ`）。S4 起每拍只 1 次往返 ⇒ **硬件上能到 ~300 Hz**；
    250 是**主动选的保守值**，不是上限 —— 见 `DEFAULT_HZ` 的注释。

    ## 指令重建低通（2026-11-18 实测后加入）

    主臂有效帧率 ≈98.5 Hz，从臂 250 Hz 消费 ⇒ 保持后的指令是阶梯（J4/J6/J7 单拍
    可达 50–64 mrad），实测在 30–150 Hz 产生的能量会激起从臂末端抖动。`filter_mode`
    在**每拍硬限速之前**对保持后的目标做二阶 Butterworth 低通（`dq` 同截止频率）：

        lp20     默认：20 Hz 全轴二阶；2 Hz 幅值 1.000、延迟约 11 ms，98.5 Hz 衰减 0.8%
        lp25     25 Hz 全轴二阶：延迟约 9 ms，98.5 Hz 衰减 1.3%
        hybrid   J1–J4 用 lp20；J5–J7 用 One-Euro 自适应低通（可选实验）
        off      关闭（只用于 A/B 对照）

    可用环境变量 `LITEARM_CMD_FILTER=lp20|lp25|hybrid|off` 选择，不必改代码。

    `catchup_speed`：对齐后的起步追赶限速（默认 `CATCHUP_SPEED`=0.5 rad/s）。
    主臂在对齐期间移动过、导致从臂落后时，先用这个速度追到 `CATCHUP_TOL`（0.02 rad），
    再切回正常 `speed_limit`；`None` 关闭。
    """
    try:
        sp = list(speed_limit or DEFAULT_SPEED_LIMIT)
        dt_nom = 1.0 / max(hz, 1.0)
        kind, fc, _fname = _filter_spec(filter_mode)
        if kind != "off" and fc >= 0.45 * max(hz, 1.0):
            log.warning("指令低通 %.1f Hz 对 %.0f Hz 环路太高 —— 按 off 处理", fc, hz)
            kind = "off"
        lp_q = lp_dq = None
        if kind == "lp":
            lp_q = _LowPass2(fc, max(hz, 1.0))
            lp_dq = _LowPass2(fc, max(hz, 1.0))
        elif kind == "hybrid":
            oe = _oe_params()
            j5_lp = _j5_lp_hz()
            if j5_lp and j5_lp >= 0.45 * max(hz, 1.0):
                log.warning("J5 追加低通 %.1f Hz 对 %.0f Hz 环路太高 —— 忽略",
                            j5_lp, hz)
                j5_lp = 0.0
            j5_only = _j5_lp_only()
            lp_q = _HybridFilter(fc, max(hz, 1.0), oe, j5_lp or None, j5_only)
            lp_dq = _HybridFilter(fc, max(hz, 1.0), oe, j5_lp or None, j5_only)
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
        q_raw_target = list(q_cmd)                    # 保持后的主臂目标（未滤波）
        dq_raw_target = None                          # 主臂速度前馈（有新帧才更新）
        q_target = list(q_raw_target)
        dq_target = None
        last_target_t = time.monotonic()
        # 起步追赶（对齐期间主臂可能已移动 ⇒ 先低速追齐，别按正常限速甩过去）
        catching_up = catchup_speed is not None and catchup_speed > 0.0
        catchup_t0 = time.monotonic()
        catchup_warned = False
        q_meas_last = list(q_cmd)
        # ⚠⚠ 追赶判据必须等**真正收到过一帧主臂目标**才允许退出。
        #   否则首拍 `provider()` 返回 `None` ⇒ `q_raw_target` 还是从臂自己的位姿
        #   ⇒ 误差 0 ⇒ 保护在首拍就放掉。真机踩过（`rec-20261118-200745`）：对齐因
        #   「5 s 没收到主臂帧」被跳过，首帧晚到 1 s，J2 按 3.4 rad/s 甩了 0.907 rad。
        have_target = False

        base = time.monotonic()
        next_tick = base + dt_nom
        # ⚠ 固件状态监视的上一拍快照（见循环里那段）
        _prev_fw = None
        # 节拍统计 —— **纯观测，不进控制律**。用来回答"这个 hz 填得住吗"：
        # `dt_nom` 必须贴近真实周期，而真实周期只有跑起来才知道（Python 那圈还有
        # 钳位 / 状态监视的开销，光按 CDC 单次往返算会高估）。
        _ticks = 0
        _work_s = 0.0
        # 逐轴跟踪误差极值：`q_cmd − q_meas` 的最大值（滞后）与最小值（**过冲**）。
        # ⚠ 为什么必须逐轴：「到末端看到过冲」是**多关节的复合**，末端一个可见偏差
        #   可能来自某个轴的大过冲，也可能是几个轴的小过冲叠加 —— 只有逐轴才分得清，
        #   而要压过冲只能按轴调 `B`（腕部惯量小，通常要加得更多）。
        _err_hi = [0.0] * N_JOINTS
        _err_lo = [0.0] * N_JOINTS

        while not should_stop():
            _t_work0 = time.monotonic()
            if duration_s is not None and time.monotonic() - base >= duration_s:
                break

            try:
                got = target_provider()
                if got is not None:
                    q_raw_target, dq_raw_target = _split_target(got)
                    last_target_t = time.monotonic()
                    have_target = True
            except Exception:                        # noqa: BLE001 - 照搬：先持位再上抛
                log.exception("target_provider 异常，受控接管")
                hold_at_current(arm)
                return False
            # 主臂帧的固有间隔（100 Hz 状态 / 200 Hz 发布）远小于 50 ms ⇒ 速度前馈
            # 在帧间保持；真正断流时归零并交由 watchdog 结束遥操。
            if time.monotonic() - last_target_t > 0.05:
                dq_raw_target = None

            # 指令重建：滤波的是**保持后的原始目标**（每拍都滤），不是上一拍滤波输出。
            q_target = list(q_raw_target)
            dq_target = None if dq_raw_target is None else list(dq_raw_target)
            speed = ([abs(v) for v in dq_raw_target]
                     if dq_raw_target is not None else [0.0] * N_JOINTS)
            if lp_q is not None:
                q_target = lp_q.process(q_target, speed)
                if dq_target is not None:
                    dq_target = lp_dq.process(dq_target, speed)

            # 起步追赶：用**未滤波**的目标与实测比较，进入容差后立刻切回正常限速。
            # 没收到过主臂目标之前一律不判 —— 那时 `q_raw_target` 只是从臂自己的位姿，
            # 拿它当"误差 0"会把保护提前放掉（见上面 `have_target` 的注释）。
            if catching_up and have_target:
                err = max(abs(a - b) for a, b in zip(q_raw_target, q_meas_last))
                if err <= CATCHUP_TOL:
                    if time.monotonic() - catchup_t0 > 0.2:
                        log.info("✓ 起步追赶完成：最大误差 %.3f rad（耗时 %.1f s）",
                                 err, time.monotonic() - catchup_t0)
                    catching_up = False
                elif (not catchup_warned
                      and time.monotonic() - catchup_t0 > CATCHUP_WARN_SECONDS):
                    catchup_warned = True
                    log.warning(
                        "⚠ 起步追赶超过 %.0f s 仍未追齐（当前最大误差 %.3f rad）—— "
                        "请把主臂带回或停下；从臂仍按 %.2f rad/s 追赶",
                        CATCHUP_WARN_SECONDS, err, float(catchup_speed))

            # 参考生成：**直通（或滤波后直通）+ 每拍硬限速**（无加速度限幅 / 制动距离）
            prev_q_cmd = q_cmd
            q_cmd = list(q_target)
            dq_cmd = [0.0] * N_JOINTS
            for i in range(N_JOINTS):
                v_limit = max(1e-4, sp[i])
                if catching_up:
                    v_limit = min(v_limit, float(catchup_speed))
                step = q_target[i] - prev_q_cmd[i]
                max_step = v_limit * dt_nom
                if abs(step) > max_step:
                    # 超过速度上限：本拍按上限推进，速度前馈取上限本身
                    q_cmd[i] = prev_q_cmd[i] + math.copysign(max_step, step)
                    dq_cmd[i] = math.copysign(v_limit, step)
                elif dq_target is not None:
                    dq_cmd[i] = max(-v_limit, min(v_limit, dq_target[i]))
                else:
                    # 没有主臂速度：用本拍实际推进量反推（参考生成器自己的输出）
                    dq_cmd[i] = step / dt_nom

            try:
                _send_joint_follow(arm, q_cmd, dq_cmd, kp, kd, wall)
            except Exception:                        # noqa: BLE001
                # ⚠ 本通道**没有** `move_js` 那条「目标 ≠ 实测位姿且 `dq == 0` 就拒帧」
                #   的限制（`move_js` 路线因此才需要「托住实测位姿」的退路）。
                #   走到这里就是链路/帧本身出问题 ⇒ **直接受控接管并上抛**，不再硬撑。
                log.exception("joint_follow 下发失败，受控接管")
                hold_at_current(arm)
                raise

            # ── 固件状态监视：**一变就记一行**（读缓存，实测 0.001 ms，不进循环开销）──
            # ⚠⚠ 故障的**先后顺序**只能从这里看出来。前几次排查我都是**杀完进程**才查状态，
            #    看到的是"结果"，还混进了我自己 kill 造成的 `WD_TRIPPED` —— **永远抓不到现场**。
            #    这一段让故障**发生的那一拍**就留下 `flags`/`joint_fault`/`err` 与跟踪误差。
            _snap = arm.get_state(refresh=False).value
            if _snap is not None and hasattr(_snap, "joints"):
                if len(_snap.q) == N_JOINTS:
                    q_meas_last = list(_snap.q)
                for _i in range(min(N_JOINTS, len(_snap.q))):
                    _d = q_cmd[_i] - float(_snap.q[_i])
                    if _d > _err_hi[_i]:
                        _err_hi[_i] = _d
                    # ⚠ **过冲只在"参考已停住"时计**（`|dq_cmd|` 小 ⇒ 参考认为已经到位），
                    #   此刻实测还超前才是真的冲过头。否则会把「快速**反向**拖动时指令已
                    #   掉头、而从臂还在原方向走」那个瞬间大差值也算成过冲 —— 实测它能让
                    #   读数虚高一倍以上，且**肩部（J2）受影响最大**（拖动幅度最大）。
                    if abs(dq_cmd[_i]) < 0.02 and _d < _err_lo[_i]:
                        _err_lo[_i] = _d
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

            # 单拍**净**耗时（算到 sleep 之前 ⇒ 不含主动等待的那段）
            _work_s += time.monotonic() - _t_work0
            _ticks += 1

            r = next_tick - time.monotonic()
            if r > 0:
                time.sleep(r)
            next_tick += dt_nom
            if next_tick < time.monotonic():
                next_tick = time.monotonic() + dt_nom

        if _ticks:
            _el = time.monotonic() - base
            _hz = _ticks / _el if _el > 0 else 0.0
            _w_ms = _work_s / _ticks * 1000.0
            log.info("从臂节拍：设定 %.0f Hz / 实测 %.1f Hz（%d 拍 / %.1f s）；"
                     "单拍净耗时 %.2f ms ⇒ 理论上限 ≈ %.0f Hz",
                     hz, _hz, _ticks, _el, _w_ms,
                     (1000.0 / _w_ms) if _w_ms > 0 else float("inf"))
            if _hz < hz * 0.9:
                log.warning(
                    "⚠ 实测节拍 %.1f Hz 明显低于设定 %.0f Hz —— 说明 `dt_nom` 与真实周期"
                    "不符，每拍硬限速的上限 `speed_limit·dt_nom` 会偏小（主臂高速时跟不动）。"
                    "把 `DEFAULT_HZ` 调到 ≈ %.0f 或更低。",
                    _hz, hz, _hz)
            # 逐轴过冲/滞后（过冲阈值 2 mrad —— 低于它算正常跟踪噪声，不报）
            _over = "、".join(f"J{i + 1}={_err_lo[i]:+.4f}"
                             for i in range(N_JOINTS) if _err_lo[i] < -0.002)
            log.info("从臂跟踪误差峰值：最大滞后 %.4f rad；过冲（实测超前指令）：%s",
                     max(_err_hi), _over or "无（各轴均 < 2 mrad）")
            if _over:
                log.warning(
                    "⚠ 上述轴在到位时**冲过了指令值** —— 这就是「过冲」。"
                    "压它靠加大**该轴**的 `B`（阻尼）：`servo.SETUP_B`（当前 %s）。"
                    "⚠ 别减 `K` —— 减 K 会变软变拖沓（那是另一个不希望的观感）。",
                    SETUP_B)
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
