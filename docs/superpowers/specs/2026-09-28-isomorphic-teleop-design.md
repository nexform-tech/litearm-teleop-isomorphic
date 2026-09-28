# 同构遥操（主从，Zenoh 点对点）设计

> 日期：2026-09-28
> 状态：待评审
> 目标仓：`litearm-teleop-isomorphic`
> 依赖：`litearm-python`（直连 CDC，**零 SDK 改动**）、`litearm-stm32` 固件（`Litearm1.5.x`）、`eclipse-zenoh` 1.6.2（**按 `python3 -m pip` 量**）、PyQt5
> 参考实现：litearm-server 遥操（`docs/superpowers/specs/2026-08-13-teleop-relay-design.md`）
> —— **只参考结构，执行路径必须换**，见 §9

---

## 1. 概述

### 1.1 要做什么

一个 PyQt5 上位机工具，同一份代码既能当**主臂端**（监听 IP:端口）也能当**从臂端**（连接主臂地址），经 **Zenoh 纯点对点**链路做关节空间同构遥操：

- 主臂进零重力，人拖动；**定频采样**关节角发布出去（`pub_hz = 200 Hz`，照搬 server 默认）。
⚠ 受限的是**从臂伺服环**（250 Hz → 实务 100 Hz），见 §9.3。
- 从臂订阅，**先慢速对齐**，对齐完成后**高速跟随**。
- 界面实时显示本机臂状态（各关节 q / dq / tau / 温度 / 故障码）。
- 部署形态两种都支持：**本机双臂**（两个进程走 `127.0.0.1`）与**跨机单臂**（各一条臂，填真实 IP）。**代码里不区分这两种**，差别只在地址栏。

### 1.2 已确认的决策（用户裁决）

| 维度 | 决策 |
| --- | --- |
| 主臂输入 | **零重力拖动**（`zero_g_start()`），人手拖动，读实测 q |
| 从臂执行 | **移植 litearm-server 的 `joint_follow`**（`send_mit_all` + `get_gravity` + `slew_target`），K/B 随帧下发。⛔ 早先选的「`move_js` 位置直通」**已废弃** |
| 部署 | 本机双臂 **与** 跨机单臂**同一份代码** |
| 对齐 | `movej` 低速粗对齐（`speed=0.15`）→ 伺服环跟随；**照搬 `_do_align` + `joint_follow`**。⛔ 早先那套 `IDLE/ALIGN_FAST/FOLLOWING/HOLDING` 状态机是自创的，已删 |
| 传输 | Zenoh **点对点**，关 multicast/gossip，**不用广播** |

### 1.3 不做（YAGNI）

力反馈 / 双向力矩 · 笛卡尔空间遥操 · 多从臂 · 录像回放 · 自动重连 · 鉴权加密（局域网内）· 遥操与其它运动控制并发 · 末端夹爪遥操 · **修改 `litearm-python`**。

---

## 2. 关键事实与依据

> 本节是全篇的地基。**每条都标了来源等级**：`[实测]` = 本机真跑过；`[源码]` = 读固件/SDK 源码得出，未经真机；`[文档]` = 只见过文字描述。§11 的 spike 任务负责把 `[源码]` 升级为 `[实测]`。

### 2.1 Zenoh 点对点 `[实测]`

在 `zenoh 1.6.2` 上实测（本机回环；⚠ **量的是 `python3 -m pip` 那个解释器** —— 本机裸 `pip` 指向另一个
python3.10 环境、装的是 1.7.2，用裸 `pip` 量版本会得到错的那个）：

| 项 | 结果 |
| --- | --- |
| 关 `scouting/multicast/enabled` + `scouting/gossip/enabled`，两端 `mode="peer"`，主臂 `listen/endpoints`、从臂 `connect/endpoints` | **通** |
| 100 Hz 持续 3 s（当时用 70 B 帧测） | 发 300 / 收 300，**丢 0**。⚠ 现协议已换成 120 B（§4.2），此条是 zenoh 能力记录，不代表当前帧格式 |
| 端到端延迟（**回环、单时钟** —— 跨机的量法不同，见 §4.2） | **p50 0.091 ms / p95 0.194 ms / max 0.397 ms** |
| **不调 `close()` 就退出** | **进程永久挂死**；显式 `close()` ⇒ `exit=0` |

最后一条是必须守的纪律（§10 陷阱 9）。

### 2.2 SDK 侧约束 `[源码]`

`litearm-python` **没有** `joint_follow` / `zero_gravity(on_sample=)` / `request_stop` —— 那是 `pylitearm` 的桶 B 接口。可用的是：

| 能力 | 入口 |
| --- | --- |
| 零重力 + 保活线程 | `arm.zero_g_start(period) / zero_g_stop()`，`period ∈ [0.005, 0.10)`，默认 0.04 |
| 读状态（100 Hz 被动流缓存） | `arm.get_state(refresh=False)` → `Msg[RobotState]`，`.value` 可能为 `None` |
| 连续伺服 | `arm.move_js(q, dq, tau_ff)`（`0x03`），**每次调用自己 kick 看门狗** |
| 关节软限位 | `arm.params.all_joint_params()` → 每轴 `q_min / q_max`（**无** `speed_limit`） |
| 受控接管 / 就地稳住 | `arm.movej(q, speed)`（`0x01`） |
| 就地高刚度声明 | `arm.park()`（`0x20` J=0） |
| 急停 / 失能 | `arm.emergency_stop()` / `arm.disable()` |
| 状态字段 | `JointState: q, dq, tau, t_mos, t_coil, err` |

### 2.3 固件语义 `[源码]` —— `move_js` 路线（**本设计已不用，留作参考**）

> ⛔ **本节描述的是 `move_js`；本设计现在走 `joint_follow` → `send_mit_all`**（用户裁决「严格按 litearm-server」）。
> 保留的理由：这些语义是**真机/源码核过的硬事实**，换路线不改变它们。
> ⚠⚠ 尤其第 (a) 条的前提在本固件上**已被真机证伪**：见 §10 陷阱 #1。


#### (a) `move_js` 的 `dq` 决定 `q_ref` 的走位速率

```c
/* control_loop.c:2105-2107  ARM_MODE_MOVE_JS */
v_lim = clampf(fabsf_(target_dq[i]), 0.0f, jp->speed_limit * gov_ratio);
q_s[i] = target_q[i];
...
/* control_loop.c:2227, 2230-2232 */
cmd->q_ref = slew_linear(q_s[i], cmd->q_ref, v_lim * LITEARM_CTRL_DT);
```

```c
/* math_utils.h:30-35 */  delta = val - prev;
                          if (delta >  max_step) return prev + max_step;
                          if (delta < -max_step) return prev - max_step;
                          return val;
```

- `v_lim` 只取 `|dq|`（**符号被 `fabsf` 丢掉**），方向由 `target_q − q_ref` 决定。
- ⇒ **`dq = 0` ⇒ `v_lim = 0` ⇒ `q_ref` 冻结 ⇒ 该轴一步都不动。**
- `speed_limit` 默认值（**7 关节整臂表**）：`J1=2.0, J2=2.0, J3=1.75, J4=1.75, J5=2.0, J6=2.0, J7=2.0` rad/s（`params/defaults.c` 的
  `g_defaults.joint[]`）。
  > ⚠ **别读成 3.5**：`defaults.c:34` 那张 `speed_limit=3.5f` 的表是 **`LITEARM_BENCH_1J` 单电机台架**的，
  > 两张表由条件编译互斥。**这类常量一律按符号定位、不按行号**（§10 陷阱 12）。
- ⚠⚠ **但 `dq` 有第二个角色，别只算第一个**：它**原样**进电机 MIT 帧的速度项 ——
  `dq_s[i] = clampf(target_dq[i], -jp->vel_max, jp->vel_max)`（`control_loop.c:2109`），
  而 MIT 控制律是 `τ = kp·(q_ref − q) + kd·(dq_ref − dq) + τ_ff`。
  ⇒ **给一个拍脑袋的定值 `dq`，等于向电机注入一个编造的速度前馈。**
  例：J1 的 `kd` 合计 `mit_kd 5.0 + kd_extra 6.0 = 11.0`，若 `dq` 填 6.0（会被钳到
  `vel_max = 2.0`）⇒ **凭空 22 Nm**。**这条是本设计最早的一版里被漏掉的，见 §5.1 的现写法。**

#### (a′) `dq` 的两个角色与真机实证 `[实测]`

`[实测]` 来源：`litearm-server/scripts/e2e_movejs_real.py` 的负控 / 正控对拍（2026-09-20 真机）：

| 组 | 条件 | 结果 |
| --- | --- | --- |
| **负控** | 发 `q_now + 0.06` 偏移、`dq = 0` | `max\|Δq\| = 0.0000`（**纹丝不动**） |
| **正控** | `dq` 取 `q(t)` 的**解析导数** | 正常跟随，跟踪误差 rms **0.0043 rad** |

⇒ **结论：`dq` 必须是真实的路径速度，逐拍算对 —— 不是"给个上限"。**
它同时满足两个角色：`|dq|` 给出走位速率，`dq` 本身给出正确的速度前馈。
固件的 `slew_linear` 就是现成的限速器 ⇒ 这条**便宜仍在**，只是 `dq` 的**值**必须算对，
能拧的只是它的**幅值上限**（即 §5.1 的 `speed_limit`）。

> ⛔ **既有实证的覆盖边界（本设计必须知道）**：那个脚本**刻意把速度压在 0.5 rad/s 以下**
> （脚本内写死 `if |dq|.max() > 0.5: raise SystemExit`），实际峰值只有 **0.032 rad/s**。
> 而遥操跟随会跑到 **1~2 rad/s** —— 该量级下速度前馈的行为**没有任何实证**。
> **这才是 §11 真正要补的 spike**（S3），不是重做已经验过的 `dq=0` 与低速跟随。
>
> ⚠ **SDK 文档在此处不完整**：`DEVELOPER_GUIDE.md:259` 只写 "`dq` is a velocity reference,
> not a limit"，**漏掉了"`|dq|` 同时是 `q_ref` 的斜率上限、`dq=0` 冻结该轴"这半句**。
> 它不算错（`dq` 确实是参考值，且 `vel_max` 另有一道钳位），但按它字面推理会写出"臂不动"的代码。
> 登记为陷阱 1（§10），**不改 SDK**。

#### (b) 看门狗 fail-soft = 0.6× 刚度 + `τ=0` ⇒ **缓慢下垂**

```c
/* control_loop.c:1813 */
const bool hold = watchdog_tripped() || g_arm.drop_hold;
/* control_loop.c:2004-2007 */
kp_s[i]  = jp->mit_kp * (park_requested ? 1.0f : g_params.hold_kp_scale);
tau_u[i] = 0.0f;                        /* 内置重力前馈被丢 */
```

`hold_kp_scale = 0.6`（`params/defaults.c:49`）。

⇒ 停发 `move_js` 后 100 ms 进入 fail-soft：**既不是自由落体（那是 disable），也不是稳住** —— 是降刚度 + 丢重力前馈 ⇒ 缓慢下垂。SDK 文档自己写着 "the arm sags slowly"。

**停发之后发生什么 —— 主臂与从臂走的是两条不同路径。**

关键：`g_arm.mode` **只被 `ctrl_accept_*` 改，停发命令不会改它**。所以主臂退出零重力后是 `INIT`，
而从臂停发 `move_js` 后**仍是 `MOVE_JS`** —— 落在 `switch (mode)` 的**不同分支**上。

| 路径 | 停发后 ~100 ms | 100 ms 后（看门狗触发） | `movej(q_now)` 到位后 |
| --- | --- | --- | --- |
| **主臂**：`zero_g_stop()` ⇒ mode=`INIT` | `default:`「未定义模式: 保持持位」（`control_loop.c:2216-2222`）：**1.0×** + **`τ=0`** ⇒ 垂到 `G/mit_kp` | fail-soft（`:2004-2008`）：**0.6×** + `τ=0` ⇒ 再垂到 `G/(0.6·mit_kp)` | `ht_on`（`:2292-2313`）：**2×** + `G(ht_q)` ⇒ **稳住** |
| **从臂**：停发 `move_js`，mode **仍是 `MOVE_JS`** | `case ARM_MODE_MOVE_JS`（`:2105-2112`）：**1.0×** + **`builtin_mode` 给重力前馈**（gating 原文："move_j 全；**move_js 仅无用户 tau_ff**"，`:2327-2333`）⇒ 基本不垂 | 同上（fail-soft）：**0.6×** + `τ=0` ⇒ 垂到 `G/(0.6·mit_kp)` | 同上 |

⇒ 三条推论：

1. **看门狗触发是让刚度从 1.0× 掉到 0.6×**（下垂变大），**不是**"从失力变成持位"。`ARM_MODE_INIT` 落 `default:` 分支 ⇒ 它**不**等于零力矩。
2. **从臂那条反而比主臂好**（它有重力前馈）—— 反直觉，但源码如此。别把主臂的路径当成通用路径。
3. **`park()` 只阻止第二次降刚度**（fail-soft 从 0.6× 拉回 1.0×）；**它不提供重力前馈** ⇒ 仍垂
   `G/mit_kp`。只有 `movej(q_now)` 进 `ht_on` 才是终解（**2× 刚度 + 重力前馈**）。

#### (c) 只有 `movej` 到位后的 `ht_on` 态才**真稳**

```c
/* control_loop.c:2292-2313  到位增刚过渡 */
uint16_t steps = 0.2f / LITEARM_CTRL_DT;               /* 0.2s smoothstep */
float kp_hold = jp->mit_kp * g_params.dyn.hold_kp_gain;  /* 2.0×, 钳到 500 */
tau_u[i] = ht_tau0[i] + alpha * (ht_tauf[i] - ht_tau0[i]);  /* 终点 = dyn_gravity(ht_q) */
```

⇒ **2× 刚度 + 重力前馈**。这是唯一"稳住不下垂"的持位态。

⇒ **收尾必须 `movej(q_now, speed≈0.3)` 受控接管**，不能只停发。

> `park()` **不是** `request_stop`（就地高刚度持位）的等价物：它只把 fail-soft 的刚度从 0.6× 拉回 1.0×，**`tau` 仍是 0 ⇒ 仍会垂 `G/mit_kp`**。它的价值在**瞬态** —— 把刚度钉在
> 1.0×、不掉到 0.6×，且**瞬时、无模式切换、零运动**，适合 HOLDING（§5.2）。终态收尾仍必须 `movej(q_now)`。
>
> ⚠ **1J 台架固件上这套结论不成立**：台架的 `.dyn` 初始化器**没有设 `hold_kp_gain`**
> （默认值表 `params/defaults.c` 的台架 `.dyn` 块）⇒ 它取 **0**，于是 `ht_on` 会下发
> kp = mit_kp × 0 = 0、`tau = 0` ⇒ **`movej` 收尾在台架板上会让臂逐渐完全失力**，
> 比不收尾更差。**§11 的 S5 与真机验证必须在 7 关节整臂固件上做**（§10 陷阱 11）。

#### (d) 急停 = EMERGENCY = **全失能自由落体**

`急停 / 包络越限 → EMERGENCY(全 DISABLE, 只可 RESET 恢复)`（`ARCHITECTURE-USB-CMD.md:262`）。

⇒ 与"停止遥操"是**两件不同的事**，界面必须分开且写明（§6）。

#### (e) 另外五条读源码得到的

1. **固件自己也钳位**：`ctrl_accept_move_js` 里 `target_q[i] = clampf(q[i], q_min, q_max)`（`control_loop.c:716`）⇒ PC 侧钳位是**纵深**，不是"唯一护栏"。
2. **`move_js` 自己 kick 看门狗**：`ctrl_accept_move_js` 末尾 `watchdog_kick()`（`control_loop.c:725`）⇒ 100 Hz 重发即保活。注意 `0x20`/`0x21` **刻意不
   kick**。
3. **`set_speed()` 会整体缩小走位速率**：`v_lim` 里乘了 `gov_ratio`（`0x21` 的全局调速器）⇒ **本工具绝不调用 `set_speed()`**。
4. **零重力退出时的越限轴由固件自动低速带回**（`ctrl_zero_g_leave` 的 A3 fix，`control_loop.c:794-805`）⇒ PC 侧不需要处理，但界面要提示。
5. **零重力中 `move_js` 被拒**（`0x04`，`control_loop.c:696`）⇒ 主臂进程不会误发伺服命令。

---

## 3. 架构

```text
主臂进程                                          从臂进程
┌────────────────────────────────┐               ┌────────────────────────────────┐
│ ArmWorker 线程（独占 Arm）      │               │ ArmWorker 线程（独占 Arm）      │
│   zero_g_start() 人手拖动       │               │   get_state() 缓存 100Hz        │
│   get_state().q 100Hz 采样      │  zenoh 点对点 │   ▲                            │
│        │                        │  TCP + 端口   │   │                            │
│        ▼                        │◀─────────────▶│   │ 100Hz 伺服环               │
│   pub.put(帧) ──────────────────┼───────────────┼──▶│ sub 回调 → latest 槽        │
│                                 │               │   ▼ move_js(q, dq)             │
└────────────────────────────────┘               └────────────────────────────────┘
         ▲                                                  ▲
         │ 只投命令 / 只读快照                                │
    ┌────┴─────┐                                      ┌────┴─────┐
    │ PyQt5 GUI│                                      │ PyQt5 GUI│
    └──────────┘                                      └──────────┘
```

### 3.1 线程模型

| 线程 | 职责 |
| --- | --- |
| **`ArmWorker`**（唯一持有 `Arm` 的线程） | 连接 / 使能 / 遥操循环 / 收尾。**任何 SDK 调用只在此线程** |
| Zenoh 回调线程（从臂） | 只写 `latest` 槽（latest-wins），**做别的什么都不做** |
| Qt 主线程 | 只投命令、只读快照（30 Hz 刷新） |

**为什么把所有 SDK 调用收进一个线程**：`litearm-python` 的 `_Ack` 按 `(上行 id, 回显码)` 分队列，两条读线程**大体**能各取各的，但"取帧"这个动作本身会在链路上互相抢（SDK
 自己的注释就记着"放在锁外实测会把在跑那条的 `ACK{0x3A}` 抢走 ⇒ 受害者报无应答而命令其实已生效"）。串行化让这一整类问题不存在，代价只是一条线程。

**决策**：动作类命令（`enable`/`movej`/`move_js`/`zero_g_*`/收尾）**只在 `ArmWorker` 上**；`get_state(refresh=False)` 只读缓存不取帧，伺服环读它安全。**唯二例外**是
 §7.4 的急停旁路与只读查询 —— SDK 明文允许它们不排队。

**Zenoh 回调只写槽**:`move_js` 是阻塞调用（每帧等 ACK），不能放进回调线程；回调只做「赋值给 latest 槽」，避免 Zenoh 线程被伺服循环拖住。

**状态帧另走一个钩子推给 GUI**：`ALIGN_FAST` 与收尾 `movej` 都会**阻塞 `ArmWorker`**（最长
`move_timeout`），若界面只能经 worker 取状态，那几秒界面就是死的 —— 而「对齐中」正是操作者最需要
看从臂状态的时刻。做法**照抄 litetool**（`litearm-tool-stm32/litetool/sdk_worker.py::_attach_state_hook`）：
把 `arm._a._on_status` 包一层，**在 SDK 自己的读线程上**把每帧解码结果推给 GUI。

- 这是本仓对 SDK 的**唯一私有耦合**；挂不上时**抛错而不是静默降级**
  （照抄 litetool 的 `StateHookMissing` 语义：宁可启动即报，也不要「连上了但界面不动」），
  并由 `test_arm_worker.py` 钉住这个缝隙存在。
- `get_state(refresh=False)` 读的就是这个缓存 ⇒ 推给 GUI 的值与伺服环用的是**同一帧**，不会出现两份真相。

### 3.2 数据流

```text
主臂:  Arm → (SDK 读线程) → get_state 缓存 → ArmWorker 100Hz 读 → pub.put(帧)
从臂:  sub 回调 → latest 槽 → ArmWorker 读槽 → 钳位 → slew_target → send_mit_all
```

采样与发布在从臂侧**不排队**：只用 `latest` 槽，迟到帧直接覆盖。理由与 litearm-server 的 `drain_latest()` 一致 —— 遥操只要最新姿态，积压帧会让从臂追一条过期的轨迹。

---

## 4. 线协议

> ⛔ **本节是 litearm-server 的格式，不是本仓的设计。** 用户裁决
> 「必须严格按照 litearm-server 来写 —— 它是经过验证的」。
> 逐字对应 `litearm_server/teleop_manager.py:19-45`，函数名也保持一致，
> 以便逐行对账。本仓实现见 [liteteleop/wire.py](../../../liteteleop/wire.py)。

### 4.1 Key

`teleop_topic(arm_id)` ⇒ **`litearm/v4/{arm_id}/teleop`**（与 `teleop_manager.teleop_topic` 同串）。
默认 `arm_id = "armA"`（对齐 `TeleopController(arm_id="armA")` 的默认值），界面可改。

> ⚠ 与早先的版本相反：**现在刻意与 litearm-server 共用同一个 topic 串**。
> 早先另起 `litearm/teleop/isomorphic` 的理由是"帧格式不同、共用会静默解错"——
> 帧格式既然已经对齐，那条理由就消失了，而共用 topic 让两端**可以**互通。

### 4.2 帧格式（**`>15d`，大端，定长 120 B**）

```text
struct ">15d"  =  120 字节
    q[0..6]      f64 BE    主臂实测关节角 rad
    dq[0..6]     f64 BE    主臂实测关节速度 rad/s
    timestamp    f64 BE    主臂时基（`time.monotonic()`）
```

对应 `encode_teleop(q, dq, timestamp)` / `decode_teleop(payload) -> {"q","dq","timestamp"}`。

#### 与早先那一版的差别（**为什么整条换掉**）

| | 早先（**已废弃**） | 现在（= litearm-server） |
| --- | --- | --- |
| 字节序 | 小端 `<` | **大端 `>`** |
| 数值 | f32 | **f64** |
| 关节数 | 变长 `n` + 版本字节 | **写死 7** |
| 序号 | 有 `seq` | **无** |
| 帧长 | 70 B | **120 B** |

⛔ 早先那版是**本仓自创**的。自创格式意味着"已验证"这句话不再适用 —— 这正是换掉它的理由。

#### 设计说明

- **`timestamp` 只能算"同一主臂相邻两帧的间隔"**（帧间隔抖动），**不能算端到端延迟**
  （跨机两端时钟不同源）。litearm-server 的 `frame_jitter_ms` 就是这么用的 ——
  收到帧时取相邻两帧 **master timestamp** 之差，再做指数滑动平均。
- **没有版本字节了**：定长 + 大端本身就是判据 —— 长度不对 `struct.unpack` 直接抛。
  litearm-server 就是这么做的，**不做"尽量解"的兜底**。
- ⚠ 判据仍要**钉字节**而不是只做往返测试：往返测试对字节序**没有判别力**（两端同错也全绿）。
  `tests/test_wire.py` 因此额外与 `struct.pack(">15d", ...)` 对拍一次。

---

## 5. 从臂伺服环

> ⛔ **本节是 litearm-server 的 `joint_follow` 的忠实移植**，不是本仓的设计。
> 三份来源：`pylitearm/control/joint_follow.py`（控制律与控制器）、
> `pylitearm/sdk/arm.py:2006+`（伺服环循环）、
> `pylitearm/config/litearm_balanced.yaml`（参数真值）。
> 本仓实现见 [liteteleop/servo.py](../../../liteteleop/servo.py)。

**从臂执行不再走 `move_js`。** 早先那版选 `move_js` 位置直通，由此推出"`K`/`B` 透不过
`move_js` ⇒ 只能按 kd 预算收紧 `speed_limit`"—— **那是在解自己造出来的问题**。
litearm-server 用 `joint_follow` → **`send_mit`，K/B 随帧下发**；本 SDK 有
`send_mit_all`，所以可以**逐行照搬**。

### 5.1 控制律（照搬）

```text
τ = K·(q_cmd − q) + B·(dq_cmd − dq) + G(q)
```

- `q_cmd`/`dq_cmd` = `slew_target` 对**外部目标**做速度/加速度限幅后的指令
- 弹簧-阻尼项由**电机固件的 MIT 环**执行；PC 侧只算重力前馈并下发
- `G(q)` 取 `arm.model.get_gravity(q)`；`τ_ff` 钳到 `tau_max`（同为固件读回值）
- 限位墙：`JointLimitWall`（`wall.py`，逐字移植）叠加在 `τ_ff` 上

### 5.2 每拍顺序（照搬 `joint_follow` 循环体）

```text
prime()    叠加重力补偿 + 当前位置刚度，防松手前下坠（一次性）
engage()   以 kp=15 / kd=0.8 托住当前姿态 engage_sec=0.3 s，减少接管冲击
start()    把 q_cmd/dq_cmd/q_target/dq_target 全部初始化成【当前实测】
loop:
    (q_target, dq_target) = target_provider()      # 返回 None ⇒ 保持上一拍的 q_cmd/dq_cmd
    set_target(q_target, dq_target)
    step():  读反馈 → 检故障 → slew_target → τ_ff = G(q) + wall → send_mit_all
```

⚠ `target_provider()` 返回 `None` 时**保持上一拍的 `q_cmd/dq_cmd` 不动** ——
这是"首帧到达前原地不动"的落点（`teleop_manager._target_provider` 把共享目标初始化成
**从臂自身当前 q**，所以首帧之前它自己不动，绝不返回垃圾值）。

### 5.3 参数真值（**逐个照抄**，不得就地调参）

`pylitearm/config/litearm_balanced.yaml` 的 `joint_follow:` 段：

| 参数 | 值 |
| --- | --- |
| `K` | `[25.0]×7` |
| `B` | `[0.5]×7` |
| `speed_limit` | `[2.8, 3.4, 5.0, 5.0, 10.0, 8.0, 13.0]` |
| `accel_limit` | `[14.0, 22.0, 24.0, 24.0, 45.0, 40.0, 60.0]` |
| `engage_sec` | `0.3` |

`joint_limit_wall:` 段：`margin_rad 0.02`、
`stiffness [200,200,150,150,100,60,20]`、`damping [2,2,2,2,2,3,1]`、
`firmware_kd_extra 0.8`、`slew_rate 0`（关闭）、`tau_max [19.5,19.5,5.25,5.25,1,1,1]`。

⇒ **J4 的 `speed_limit` 是 5.0 rad/s。** 早先按 kd 预算推出来的 0.573 rad/s 作废 ——
那套推导的立足点（`move_js` 刚度由固件定死）在 `joint_follow` 下不成立。

### 5.4 软限位钳位

对应 `teleop_manager._read_safe_limits()` + 收帧时的 `np.clip`：

1. 读**固件**软限位 —— `arm.params.all_joint_params()` 的 `q_min`/`q_max`
   （⚠ 本 SDK **没有 `arm.kin`**，server 是从 `kin` 或配置读的）
2. 两侧各内缩 `margin`（默认 `0.01`，取自 server 的 `safety.joint_limit_margin_rad`）
3. 收到帧后 `clip(q, lo, hi)` 再交给 `set_target`

⚠ 限位是 `joint_follow` **唯一的位置安全护栏**（SDK 内部不校验关节限位）。
拿不到就**抛**，绝不退回兜底哨兵 —— server 命中兜底会大声告警，本仓直接拒启动。

### 5.5 watchdog 与退出（照搬）

- `watchdog_ms = 200`（server 的默认值）。**收到首帧之后**才生效
- 超时 ⇒ 计数 + **结束伺服环**，**不自动重连**
- ⚠ 永远收不到首帧时 watchdog 不会触发（它是从"上一次收到"算的），
  server 另有一条 `max(5, 3×watchdog)` 的**首帧诊断告警**，只告警不停 —— 照搬

### 5.6 收尾（`request_stop()` 的对应物）

本 SDK **没有 `arm.request_stop()`**。收尾 = 用 `movej(实测位姿)` 让固件的 S 曲线 +
`ht_on` 接管 —— 即 `servo.hold_via_movej()`。

⛔ **绝不 `disable()`**：失能会让臂在自重下自由落体（spec §7.3）。

### 5.7 与 pylitearm 的接口适配（**只有这几处**）

| pylitearm | 本仓 | 说明 |
| --- | --- | --- |
| `hw.send_mit(K, B, q, dq, tau)` | `arm.send_mit_all(q, dq, K, B, tau)` | ⚠ **参数顺序不同** |
| `self.dyn.gravity(q)` | `arm.model.get_gravity(q).value` | 见下 ⚠⚠ |
| `hw.read_q_dq()` | `st.q` / `st.dq`（`get_state`） | SDK 已有独立读线程 |
| `hw.faulted()` | `st.faulted` / `st.joint_fault` | |
| `hw.assert_operational(measured_overspeed_factor=inf, skip_position=True)` | **丢弃** | 那两个实参的意思就是"把位置与超速护栏都关掉"，**丢弃即等价** |
| `hw.emergency_hold_healthy()` | `movej(实测位姿)` | 见 §5.6 |
| `arm.request_stop()` | 同上 | 本 SDK 无此原语 |
| `arm.zero_gravity(on_sample=)` | `arm.zero_g_start()` + 轮询 | 见 §6 |

⚠⚠ **`G(q)` 是唯一有性能后果的偏离。** pylitearm 在**本地**用 Pinocchio 算（亚毫秒）；
本仓走 `model.get_gravity` 的**串口往返**。**实测 3.31 ms/次**（`servo.measure_gravity_cost`）
⇒ 理论上限 ~302 Hz，**200 Hz（5 ms 周期）太紧**（光重力就吃掉 66%）。
⇒ **实务环频取 100 Hz**，并把 `G(q)` 的耗时作为可观测项持续盯住。
这条是"照抄不来"的地方，必须显式记着。

---

## 6. 主臂循环

1. 连接 + `enable()` + 断言无 `faulted` / `joint_fault`。
2. `zero_g_start()` ⇒ 固件有 **0.35 s engage 段**（低刚度托举 `τ=G+墙弹簧`）。等待 ≥0.5 s 再开始发布（engage 期间臂在受控托举，采样值无意义但不危险）。
3. 独立 pub 线程定频 **`pub_hz = 200 Hz`**（照搬 server：采样与发布**解耦**，
   回调/轮询只写共享槽）：
   `st = get_state(refresh=False).value`；`st is None` ⇒ 计数并跳过；否则 `pub.put(帧)`。
   ⚠ **主臂这一环不调 `G(q)`**，所以 200 Hz 不受 §9.3 的限制；
   受限的是**从臂伺服环**（250 Hz → 实务 100 Hz）。
4. 停止：见 §5.3。

**主臂不做任何钳位**：零重力下臂由人拖动，固件**刻意跳过位置/超速包络锁存**（§2.3(e)，"用户可任意拖拽"）。PC 侧插手钳位反而会限制拖动范围。发布的是**实测值**，钳位是从臂的事。

---

## 7. 安全

### 7.1 闸门清单

| 场景 | 行为 |
| --- | --- |
| 软限位读不到（`all_joint_params()` 抛错 / 值非法） | **拒绝启动跟随**，停在 `IDLE` + 响亮报错。**不静默退化**（不学 server 的 ±9 兜底告警） |
| 每帧目标 | `clamp(q_master, q_min, q_max)` —— **纵深**（固件自己也钳，§2.3(e)1），同时让界面能显示"被钳了" |
| 单帧 `send_mit_all` 抛异常 | 捕获 + 计数 + 循环继续；**不终止跟随**（一帧异常不该让臂停） |
| ⚠ **目标值非有限（NaN/Inf）** | `clamp_to_limits` 抛 **`NonFiniteTarget`**（`LimitsError` 的子类，专为"本拍跳过"而设）⇒ 本拍**不下发** + 计数。⛔ **但"跳过下发"不是完整契约**：它也跳过了 `watchdog_kick` ⇒ 固件 100 ms 后 fail-soft（0.6× 刚度 + `τ=0`）⇒ **臂缓慢下垂**，正是安全层要防的那件事。⇒ **连续 N 拍（默认 5）之后必须主动升级 `HOLDING`**，不能只计数 |
| ⚠ **陈旧数据不许下发** | `may_dispatch` 只读状态、**不自带新鲜度检查** ⇒ phase 2 **每拍必须先 `tick(now, frame_age, frame_id)` 再 poll**；`frame_id` 取线协议的 `Frame.seq`，恢复判据数的是**不同的帧**而不是 tick 数 |
| 首帧诊断 | 「启动后 N 秒未收首帧」与稳态 watchdog 是**两件事**（后者只在收到首帧后才生效）。载体 = `LatestSlot.ever_received` / `peek_age() is None` |
| ⚠ `send_mit_all` **单次** ACK 超时 | **当拍即报警**，不是「计数到 5 次再说」。理由：`_cmd` 超时是 **1.2 s**，而固件看门狗是 **0.1 s** ⇒ **一次超时就够让固件 fail-soft 下垂**。连续 2 次即转 `HOLDING` |
| watchdog | §5.2 |
| 启动遥操前 | 断言 `enabled`、`!faulted`、`joint_fault == 0` |
| **急停** | `emergency_stop()` 常驻按钮，走**旁路线程**（§7.4）。**界面写明"急停 = 全失能 ⇒ 臂自由落体"**，并把「停止遥操」（受控接管、稳住）作为并列的常用按钮 |
| **绝不调用** | `set_speed()`（§2.3(e)3）、`disable()`（收尾路径）、`save_params()`（不可逆，见仓外记忆） |
| 串口独占 | STM32 CDC 在 Linux **不独占**，第二个进程会分吃同一字节流 ⇒ 文档 + 界面写明"一个 CDC 口只允许一个进程" |

### 7.2 界面闸门（照抄 litetool 既有做法）

`☑ 我已确认机械臂周围无障碍、急停可及` —— 勾选前运动类按钮一律锁定；**断线 / 急停 / FAULT / 复位自动重新锁定**。

### 7.3 退出纪律

- Zenoh `session.close()` **必须在进程退出前显式调用**，否则**进程永久挂死**（§2.1 `[实测]`）。
- 收尾顺序：停遥操（§5.3）→ `close()` zenoh session → `arm.close()`。
- SDK 侧同样有 `close()` 纪律（仓外既有教训：忘了它进程退出永久挂死）。

### 7.4 急停必须走旁路，不能排 `ArmWorker` 的队

`ArmWorker` 会在收尾 `movej` 里阻塞（上界 `move_timeout`，本工具设 3 s，§5.3）。**如果急停是投进 worker 队列的一个任务，它就会被排在这次阻塞后面 —— 最长 3 s 发不出去。**

做法：**急停从一个专用短线程直接调 `arm.emergency_stop()`，不经 worker 队列。**

依据是 SDK 自己写的（`arm.py` `_cart_serial` 的锁序注释）：

> ⚠ **`emergency_stop`/`disable`/`zero_g*`/`get_tcp` 自己不许获取本锁** —— 降能量方向的动作与只读查询必须永远可达（持锁者可能阻塞到 `move_timeout`）。

⇒ 这条旁路是 SDK **明文设计的用法**，不是绕过安全设计；也正对齐 litearm-server 的"急停走专门旁路、不经 RPC 分发，任何时刻畅通"。

- 急停线程只做一件事：调 `emergency_stop()`，把成败经信号回 GUI。最坏阻塞 1.2 s（它自己的 ACK 超时），**不影响 GUI 与 worker**。
- GUI 在**发起时立即**把按钮置灰 + 显示"急停已发出"，不等返回。
- `test_arm_worker.py` 必须有一条：**worker 卡在 `movej` 期间，旁路 `emergency_stop()` 仍能成功**（§12）。

---

## 8. 界面

仿 `litearm-tool-stm32/litetool` 的形状：顶栏 + 常驻状态条 + 左侧导航 + 页面栈 + 底部常驻日志面板。

```text
┌ 顶栏: 角色[●主臂 ○从臂] 地址:端口 [连接臂][启动遥操][停止] ●在线 固件:xxx ☑确认 [⛔急停] ┐
├ 状态条: 模式 | 使能 | 遥操状态 | 频率 | 延迟 | 故障 ──────────────────────────────────┤
├ 导航: 链路 | 关节 | 遥操 ─┬─ 页面栈 ──────────────────────────────────────────────────┤
├ 日志面板（常驻）────────────────────────────────────────────────────────────────────────┤
```

| 页面 | 内容 |
| --- | --- |
| **链路** | 角色单选、IP/端口、[连接臂]/[启动遥操]/[停止]、固件版本、许可状态、连接灯；**主臂端显示 Zenoh 的 `matching` 布尔**（「发了没人在收」是现场第一类排查） |
| **关节** | 7 轴表 `J \| q \| dq \| tau \| t_mos \| t_coil \| err`；**`err != 1` 的轴高亮**（见下方 ⚠） |
| **遥操** | 状态机状态；**`ALIGN_FAST` 期间显著显示「对齐中，请勿移动主臂」+ 已用时**；主臂 q vs 从臂 q 对照表 + 逐轴跟踪误差；收/发频率、本机延迟、丢帧数、ACK 超时计数；**交班漂移告警**（§5.1）；参数（`align_speed` / 逐轴 `speed_limit_j` / `accel_limit_j` / `kd_budget` / `watchdog_ms`） |

**温度只显示数值，界面不实现阈值判据**：固件的温度锁存已经反映在 `t_mos/t_coil` + `err` + `joint_fault` 上，界面另发明一套阈值就是在制造第二份真相。

> ⚠⚠ **`err` 的口径（用户裁决）：`1` = 使能，`0` = 失能。判据是 `err != 1`，不是 `err != 0`。**
> 固件原注释（`litearm-stm32/User/litearm/litearm.h:303`）：
> 「【电机自报故障】(**`err != 1`**，自保护失能)」—— 即"不等于 1"就是故障。
> 本设计早先写的是「`err != 0` 高亮」，那是**写反了** —— 照它实现，一条健康的使能臂
> **七轴会永远全红**。
> **真机实证**（2026-09-28，主臂 `Litearm1.8.0-7J`，`scripts/verify_hw_readonly.py`）：
> `enabled=True`、`joint_fault=0x0` 时七轴 `err` **全为 `1`** ⇒ 与"`1`=使能"一致。
> 事故类型同 §2.3(a′) 那条：**断言了一个判据却没核那个值的语义**。
> ⚠ 另注意 `DEVELOPER_GUIDE.zh-CN.md:285` 的 `err=1/2/3` 是 **IK 的结果码**（无解/共线/超容量），
> **同名不同物**，别混。

**从臂端**额外显示"主臂 vs 本臂"对照与跟踪误差（数据免费，已在手上）；**主臂端**显示发布频率与已发帧数。

> ⚠ **`matching` 是布尔，不是计数** —— `zenoh.MatchingStatus` 只有 `.matching` 一个属性
> （在**跑测试的那个解释器**上核过：`dir(zenoh.MatchingStatus)` == `['matching']`）。要做「几个订阅者」得自己数
> `declare_matching_listener` 的回调边沿，**本设计不做**（YAGNI）。界面就显示「已匹配 / 未匹配」。

---

## 9. 与 litearm-server 遥操的对应关系

> 用户裁决：**「必须严格按照 litearm-server 的逻辑来，因为它是经过验证的。」**
> 本节是全篇的**验收口径** —— 逐条能指到出处，才算"照搬"；指不到出处的，就是自创，要报用户裁决。

### 9.1 逐条照搬（**每一条都能指到出处**）

| 功能 | 出处 | 本仓 |
| --- | --- | --- |
| 帧格式 `>15d` / 120 B / 大端 | `teleop_manager.py:19-45` | `wire.py` |
| topic `litearm/v4/{arm_id}/teleop` | `teleop_manager.teleop_topic` | `wire.teleop_topic` |
| 采样与发布**解耦**（回调只写共享槽，独立线程定频发） | `teleop_manager.py:163-206` | §6 |
| 主臂 `pub_hz` | `teleop_manager.__init__` 默认 `200.0` | §6 |
| 从臂**首帧前把自身当前 q 灌进目标** | `teleop_manager.py:258-264`（再同步在 `:309-312`） | §5.2 |
| 每帧**钳位到软限位**（含 margin） | `_read_safe_limits` + `np.clip` | §5.4 |
| 控制律 `τ = K(q_cmd−q) + B(dq_cmd−dq) + G(q)` | `joint_follow.py:5-12` | §5.1 |
| 参考生成 `slew_target`（梯形速度曲线、按制动距离减速） | `joint_follow.py:45-100` | `safety.slew_target`（**逐字**） |
| 参数 `K/B/speed_limit/accel_limit/engage_sec` | `litearm_balanced.yaml` 的 `joint_follow:` | §5.3（**逐值**） |
| 限位墙 `JointLimitWall` | `joint_limit_wall.py` | `wall.py`（**逐字**） |
| 伺服环调用序 `prime → engage → start → step` | `sdk/arm.py:2006+` | `servo.follow`（**照搬结构**） |
| `target_provider()` 返回 `None` ⇒ 保持上一拍 `q_cmd/dq_cmd` | `sdk/arm.py` 循环体 | `servo.follow` |
| watchdog `200 ms` + **不自动重连** | `teleop_manager.py:349-358` | §5.5 |
| **"N 秒未收首帧"单独诊断**（与稳态 watchdog 是两件事） | `teleop_manager.py:317-322` | §5.5 |
| 遥操状态**派生自控制环真实死活**（无状态机、无"停了但锁没解"） | `TeleopController.active` | 界面层 |
| 对齐：`movej` 低速粗对齐（`speed=0.15`）→ 交接时同步共享目标 | `_do_align` + `:307-313` | §5.2 |

### 9.2 必须偏离（**本 SDK 没有那个接口**）

逐条见 §5.7。汇总：

| litearm-server（pylitearm） | 本仓（litearm-python） | 偏离性质 |
| --- | --- | --- |
| `arm.joint_follow(...)` | 移植（`servo.py`） | **算法等价**，见 §5.7 |
| `arm.zero_gravity(on_sample=)` | `zero_g_start()` + 轮询 `get_state` | 机制不同、数据等价 |
| `arm.request_stop()` | `movej(实测位姿)` 受控接管 | 接口缺失 |
| `hw.emergency_hold_healthy()` | 同上 | 接口缺失 |
| `arm.kin.q_min/q_max` | `params.all_joint_params()` 的 `q_min/q_max` | **更权威**（与固件钳位同源） |
| `hw.send_mit(K,B,q,dq,tau)` | `send_mit_all(q,dq,K,B,tau)` | ⚠ 参数顺序不同 |
| `dyn.gravity(q)`（本地 Pinocchio） | `model.get_gravity(q)`（**串口往返**） | ⚠⚠ **唯一有性能后果的偏离** |

### 9.3 唯一有性能后果的偏离：`G(q)`

**实测 3.29–3.32 ms/次**（`servo.measure_gravity_cost`，50 与 200 次两轮一致，
2026-09-28，`Litearm1.8.0-7J`）。对照：`get_state(refresh=False)` 走缓存只要 **0.001 ms**。

⚠⚠ **先分清两个环频，它们是两个不同的数**（我混过一次）：

| | 值 | 出处 | 每拍调 `G(q)`？ |
| --- | --- | --- | --- |
| **主臂发布** `pub_hz` | **200 Hz** | `TeleopManager.__init__` 默认 | ❌ 不调 |
| **从臂伺服环** `control_loop_hz` | **250 Hz** | `litearm_balanced.yaml:32` → `arm._hz`（`sdk/arm.py:652`） | ✅ **每拍调** |

⇒ 卡住的是**从臂伺服环**：250 Hz 的周期只有 **4 ms**，而 `G(q)` 就要 **3.3 ms（83%）**，
还没算 `send_mit_all` 自己的往返。**250 Hz 在这套移植上跑不动。**

⇒ **实务环频取 100 Hz**（周期 10 ms，`G(q)` 占 33%，留得下 `send_mit_all`）。
   这是**照抄不来的偏离**，必须写进任何报告，不能假装不存在。

⇒ 要真正达到 250 Hz，唯一忠实的路是**把重力项挪到本地算**（pylitearm 用本地 Pinocchio，
   本 SDK 有 `model.get_body` / `model.get_jm` 可以把模型拉到 PC 侧）——
   那是**同一机制**的还原，不是发明。**在没做这件事之前，环频就是 100 Hz。**

---

## 10. 陷阱登记（写进 README，供后来者）

| # | 陷阱 | 等级 |
| --- | --- | --- |
| 1 | **`move_js` 在「目标 ≠ 实测位姿」且「`dq == 0`」时直接拒帧**，回 `ERR{0x03,0x02}`。真机实证：同一串 56 B 载荷写三次 ⇒ ACK / ACK / ERR。⇒ 「`dq=0` ⇒ 参考冻结」在本固件上**不成立**（是被**拒**，不是被冻）。⚠ 本设计**已不用 `move_js`**，留此条供后来者 | `[实测]` |
| 1b | ⚠ SDK 的 `errors.py` 把 `(0x03,0x02)` 译成「含非有限值」—— 在 1.8.0-7J 上**是误导**（我们是拿**有限**数据被拒的）。**别拿 SDK 的错误文案当根因** | `[实测]` |
| 2 | `G(q)` 走**串口往返**（3.31 ms），而 pylitearm 是本地 Pinocchio（亚毫秒）⇒ 环频上限由它决定 | `[实测]` |
| 3 | watchdog fail-soft = **0.6× 刚度 + `τ=0` ⇒ 缓慢下垂**（既非自由落体，也非稳住） | `[源码]` |
| 4 | 只有 `movej` 到位后的 `ht_on` 态才真稳（2× 刚度 + 重力前馈）⇒ **收尾必须 `movej(q_now)`** | `[源码]` |
| 5 | `park()` 只把 fail-soft 刚度**拉回** 1.0×，**`tau` 仍为 0 ⇒ 仍会垂**。不是 `request_stop` 的等价物 | `[源码]` |
| 6 | **急停 = EMERGENCY = 全失能 ⇒ 自由落体**，与"停止遥操"是两件事 | `[源码]` |
| 7 | **SDK 入口必须钉死 `/home/llx/litearm-python/src`**：本机 `sys.path` 上挂着另一份 `litearm`（gitee 克隆，停在 `chore/sync-repo-standards`），会**静默**抢先 import。判据是**导入后断言 + 打印 `pa.__file__`**，不是"我设了 PYTHONPATH" | `[实测]` |
| 8 | `kd_extra` 住在 **0x26 向量表**（`FF_VEC_ITEMS[15]`），不在 0x28 标量表：`get_ff_scalar(15,i)` 取到的是 `zg_engage_kp`，且 `sub` 只容许 0..2 ⇒ J4~J7 抛异常。正确读法是 `get_ff_vec(15)` | `[实测]` |
| 9 | Zenoh session **不 `close()` ⇒ 进程退出永久挂死** | `[实测]` |
| 10 | STM32 CDC 在 Linux **不独占** ⇒ 一个口只允许一个进程 | 仓外实测 |
| 11 | **1J 台架 vs 7J 整臂是两张互斥的默认值表**：台架 `hold_kp_gain` **未设 ⇒ 0** ⇒ 台架板上 `ht_on` 会下发 `kp=0`（`movej` 收尾反而失力）。**任何涉及 `speed_limit`/`hold_kp_gain`/`can_dyn` 的结论必须写明板卡** | `[源码]` |
| 12 | 引固件/SDK 事实**一律按符号定位、不按行号** —— 行号会漂，且条件编译表会让人读错表 | `[实测]` |
| 13 | 本仓 key 与 server 的 topic **现在刻意共用**（帧格式已对齐）。⚠ 但**跨版本仍不许混用**：格式一旦分叉，共用 key 会让两端静默解错 | 设计约定 |

---

## 11. 真机验证

> ⛔ **一律在 7 关节整臂固件上做，报告必须写明板卡与固件版本串。**
> 本机主臂：`Litearm1.8.0-7J`。

**主臂与从臂是两件事，分开验。** 当前机器上接的是**主臂**。

### 11.1 主臂（可离线/只读）

| # | 验什么 | 判据 | 状态 |
| --- | --- | --- | --- |
| M1 | SDK 链路 + 整臂判定 + 授权 | `n == 7`、使能、七轴 `err == 1`、无 `joint_fault` | ✅ **已验**（`scripts/verify_hw_readonly.py`） |
| M2 | 数据通路：采样 → 编码 → zenoh → 收到 → 解码 | 100 Hz、状态帧丢 0、zenoh 无丢失、解码 n=7 | ✅ **已验**（`scripts/verify_master.py`，600/600） |
| M3 | `G(q)` 往返耗时 | 实测，用于定环频 | ✅ **3.31 ms** |
| M4 | **零重力拖动**：`zero_g_start()` → 人手拖动 → 100 Hz 采样 | 拖动中 `dq` 明显非零、松手后不漂、通路不掉帧 | ⏳ **待做**（需人扶臂 + 授权） |

⚠ M4 会让臂**变软**（重力补偿托举、人手可拖），**必须人在场扶着**。臂上装夹爪（不在动力学模型里），
**不能无人值守跑**。

### 11.2 从臂（需要另一条臂）

| # | 验什么 | 判据 |
| --- | --- | --- |
| S1 | `joint_follow` 移植的静态正确性 | 离线：`slew_target` 与 pylitearm 参考实现对拍（已随 `test_safety.py` 保留） |
| S2 | 跟随跟踪误差 | `FOLLOWING` 下 rms ≤ 0.05 rad |
| S3 | **无自激/抖动** | `dq` 谱无新增高频峰 |
| S4 | `send_mit_all` 连续 200 Hz 的 ACK 返回率 | ≥ 99.9%，无 1.2 s 级卡顿 |
| S5 | 收尾：`movej(q_now)` vs 只停发 | 录 30 s 漂移，**A/B 两组都做**（单工况结论骗过人） |

⚠ 从臂的验证**必须等另一条臂接上**。在此之前，§5 的正确性只有**离线对拍**（`slew_target`
与 pylitearm 参考实现）撑着 —— 这条要如实写在任何报告里。

---

## 12. 文件结构与测试

```text
litearm-teleop-isomorphic/
  liteteleop/
    wire.py       帧编解码（照搬 server：">15d" 120 B 大端 + teleop_topic）
    link.py       Zenoh 点对点封装（listen/connect 配置、close 纪律）
    wall.py       关节限位虚拟墙（逐字移植 joint_limit_wall.py）
    servo.py      从臂伺服环（移植 joint_follow：控制器 + 循环 + 参数真值）
    safety.py     软限位校验/钳位 + slew_target（逐字移植）
    arm_worker.py 独占 Arm 的线程：连接/使能/遥操循环/收尾序列   ← 未写
    app.py  settings.py  ports.py  __main__.py                   ← 未写
    gui/  bridge.py  main_window.py  widgets.py
          pages/  link_page.py  joints_page.py  teleop_page.py   ← 未写
  scripts/
    verify_hw_readonly.py  主臂只读核对（不使能、不动作、不写参数）
    verify_master.py       主臂数据通路核对（采样→编码→zenoh→解码）
  tests/
    test_wire.py  test_link.py  test_safety.py     ← 已交付
    test_servo.py  test_wall.py                    ← 未写
    test_arm_worker.py  test_gui_smoke.py          ← 未写
```

---

## 13. 里程碑

1. **阶段一（已交付）** 线协议 + 链接层 + 安全层纯逻辑，离线可测
2. **阶段一补（本轮）** 按 litearm-server 重写：`wire` 换 `>15d`、`servo`/`wall` 移植、
   `safety` 去掉自创件。⚠ **`test_servo.py` / `test_wall.py` 还没写**
3. **阶段二** `arm_worker` + 主臂零重力拖动（M4）+ GUI
4. **阶段三** 从臂跟随（S1~S5），需另一条臂
