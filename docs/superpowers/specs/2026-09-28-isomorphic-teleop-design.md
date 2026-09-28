# 同构遥操（主从，Zenoh 点对点）设计

> 日期：2026-09-28
> 状态：待评审
> 目标仓：`litearm-teleop-isomorphic`
> 依赖：`litearm-python`（直连 CDC，**零 SDK 改动**）、`litearm-stm32` 固件（`Litearm1.5.x`）、`eclipse-zenoh` 1.7.2、PyQt5
> 参考实现：litearm-server 遥操（`docs/superpowers/specs/2026-08-13-teleop-relay-design.md`）
> —— **只参考结构，执行路径必须换**，见 §9

---

## 1. 概述

### 1.1 要做什么

一个 PyQt5 上位机工具，同一份代码既能当**主臂端**（监听 IP:端口）也能当**从臂端**（连接主臂地址），经 **Zenoh 纯点对点**链路做关节空间同构遥操：

- 主臂进零重力，人拖动；100 Hz 采样关节角发布出去。
- 从臂订阅，**先慢速对齐**，对齐完成后**高速跟随**。
- 界面实时显示本机臂状态（各关节 q / dq / tau / 温度 / 故障码）。
- 部署形态两种都支持：**本机双臂**（两个进程走 `127.0.0.1`）与**跨机单臂**（各一条臂，填真实 IP）。**代码里不区分这两种**，差别只在地址栏。

### 1.2 已确认的决策（用户裁决）

| 维度 | 决策 |
| --- | --- |
| 主臂输入 | **零重力拖动**（`zero_g_start()`），人手拖动，读实测 q |
| 从臂执行 | **`move_js` 位置直通**（固件 `0x03`），不走 MIT 阻抗 |
| 部署 | 本机双臂 **与** 跨机单臂**同一份代码** |
| 对齐 | 两态：`ALIGNING`（慢速）→ `FOLLOWING`（高速） |
| 传输 | Zenoh **点对点**，关 multicast/gossip，**不用广播** |

### 1.3 不做（YAGNI）

力反馈 / 双向力矩 · 笛卡尔空间遥操 · 多从臂 · 录像回放 · 自动重连 · 鉴权加密（局域网内）· 遥操与其它运动控制并发 · 末端夹爪遥操 · **修改 `litearm-python`**。

---

## 2. 关键事实与依据

> 本节是全篇的地基。**每条都标了来源等级**：`[实测]` = 本机真跑过；`[源码]` = 读固件/SDK 源码得出，未经真机；`[文档]` = 只见过文字描述。§11 的 spike 任务负责把 `[源码]` 升级为 `[实测]`。

### 2.1 Zenoh 点对点 `[实测]`

在 `zenoh 1.7.2` 上实测（本机回环）：

| 项 | 结果 |
| --- | --- |
| 关 `scouting/multicast/enabled` + `scouting/gossip/enabled`，两端 `mode="peer"`，主臂 `listen/endpoints`、从臂 `connect/endpoints` | **通** |
| 100 Hz 持续 3 s（70 B 帧） | 发 300 / 收 300，**丢 0** |
| 端到端延迟 | **p50 0.091 ms / p95 0.194 ms / max 0.397 ms** |
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

### 2.3 固件语义 `[源码]` —— 本轮最关键的几条

#### (a) `move_js` 的 `dq` 是 **q_ref 走位速率旋钮**，不是"速度前馈"

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
- **这条给了设计一个大便宜**：固件的 `slew_linear` 就是现成的限速器，`dq` 就是旋钮 ⇒ "慢速对齐 / 高速跟随"= **同一个 `move_js` 调用只换 `dq`**，PC 侧不需要手写 ramp。

> ⚠ **SDK 文档在此处不完整**：`DEVELOPER_GUIDE.md:259` 只写 "`dq` is a velocity reference, not a limit"，**漏掉了"`|dq|` 同时是 `q_ref` 的斜率上限、`dq=0`
> 冻结该轴"这半句**。它不算错（`dq` 确实是参考值，且 `vel_max` 另有一道钳位），但按它字面推理会写出"臂不动"的代码。登记为陷阱 1（§10），**不改 SDK**。

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

**停发之后的完整三段**（非 S 曲线模式下 `hold=false` 那一段落的是 `default:` 分支）：

| 时刻 | 分支 | `kp_s` | `tau_u` | 稳态 |
| --- | --- | --- | --- | --- |
| 停发瞬间 ~100 ms | `default:`「未定义模式: 保持持位」（`control_loop.c:2216-2222`） | `mit_kp`（**1.0×**） | 0 | 垂到 `G(q)/mit_kp` |
| 100 ms 后（看门狗触发） | fail-soft（`control_loop.c:2004-2008`） | `mit_kp × (park_requested ? 1.0 : 0.6)` | 0 | 再垂到 `G(q)/(0.6·mit_kp)` |
| `movej(q_now)` 到位后 | `ht_on` 增刚（`control_loop.c:2292-2313`） | `mit_kp × 2.0`（钳 500） | `dyn_gravity(ht_q)` | **真正稳住** |

⇒ **注意方向**：看门狗触发是让刚度从 **1.0× 掉到 0.6×**（下垂变大），**不是**"从失力变成持位"。`ARM_MODE_INIT` 落 `default:` 分支 ⇒ 它**不**等于零力矩。所以：

- 收尾只停发 ⇒ 会垂**两次**（先到 `G/mit_kp`，再扩到 `G/(0.6·mit_kp)`）；
- `park()` 的真实作用是**阻止第二次降刚度**（把 fail-soft 拉回 1.0×），**它不提供重力前馈** ⇒ 仍垂 `G/mit_kp`；
- 只有 `movej(q_now)` 进 `ht_on` 才是终解（**2× 刚度 + 重力前馈**）。

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
> 比不收尾更差。**§11 的 S4 与真机验证必须在 7 关节整臂固件上做**（§10 陷阱 11）。

#### (d) 急停 = EMERGENCY = **全失能自由落体**

`急停 / 包络越限 → EMERGENCY(全 DISABLE, 只可 RESET 恢复)`（`ARCHITECTURE-USB-CMD.md:261`）。

⇒ 与"停止遥操"是**两件不同的事**，界面必须分开且写明（§6）。

#### (e) 另外五条读源码得到的

1. **固件自己也钳位**：`ctrl_accept_move_js` 里 `target_q[i] = clampf(q[i], q_min, q_max)`（`control_loop.c:722`）⇒ PC 侧钳位是**纵深**，不是"唯一护栏"。
2. **`move_js` 自己 kick 看门狗**：`ctrl_accept_move_js` 末尾 `watchdog_kick()`（`control_loop.c:733`）⇒ 100 Hz 重发即保活。注意 `0x20`/`0x21` **刻意不
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

**Zenoh 回调只写槽**:`move_js` 是阻塞调用（每帧等 ACK），不能放进回调线程；回调只做"赋值给 latest 槽"，避免 Zenoh 线程被伺服循环拖住。

### 3.2 数据流

```text
主臂:  Arm → (SDK 读线程) → get_state 缓存 → ArmWorker 100Hz 读 → pub.put(帧)
从臂:  sub 回调 → latest 槽 → ArmWorker 100Hz 读槽 → 钳位 → move_js
```

采样与发布在从臂侧**不排队**：只用 `latest` 槽，迟到帧直接覆盖。理由与 litearm-server 的 `drain_latest()` 一致 —— 遥操只要最新姿态，积压帧会让从臂追一条过期的轨迹。

---

## 4. 线协议

### 4.1 Key

默认 `litearm/teleop/isomorphic`，界面可改，**两端必须一致**。

> ⚠ **刻意不用** litearm-server 的 `litearm/v4/{arm_id}/teleop`：server 的帧是 `>15d` 120 B，本文的是 70 B。共用 key 会让两端把对方的帧**静默解错**。key 不同 ⇒
> 不可能误配。

### 4.2 帧格式（v1，小端，变长关节数）

```text
offset  size   field                                  说明
0       1      version = 1                            协议版本；不符 ⇒ 拒绝启动跟随
1       1      n                                      关节数（1..255）
2       4*n    q[n]    f32 LE                         主臂实测关节角 rad
2+4n    4*n    dq[n]   f32 LE                         主臂实测关节速度 rad/s
2+8n    8      ts      f64 LE                         主臂 time.monotonic()
2+8n+8  4      seq     u32 LE                         主臂帧序号
```

n = 7 时共 **70 B**。

#### 设计说明

- **`version` 字节是防"静默解错"的那道判据**。两端版本不符时**拒绝启动跟随并响亮报错**，而不是照 n 去解一帧垃圾。
- **`dq` 只用于显示与遥测，绝不直接喂给从臂的 `move_js`**（见 §5.2 陷阱 2）。
- **`ts` 取主臂 `time.monotonic()`**：跨机时两端时钟不同源，所以它**只能算"同一主臂相邻两帧的间隔"**（帧间隔抖动），**不能算端到端延迟**。本工具显示的"端到端延迟"是**本机**从"收到帧"到"下发
  move_js"的耗时，与 `ts` 无关。
- 小端（`<`）**是本仓的约定**，与 server 的 `>`（大端）不同。§8 的 `test_wire.py` 用**黄金字节**钉死它，不用往返测试 —— **往返测试对字节序没有判别力**（两端同错也全绿）。

---

## 5. 从臂状态机

```text
                启动遥操
   IDLE ─────────────────────▶ ALIGNING ──收敛──▶ FOLLOWING
     ▲                            ▲                    │
     │                            │                    │
     │                     连续收帧 5 拍          watchdog 超时
     │                            │                    ▼
     │                            └──────────────  HOLDING
     │                                                 │ 持续 ≥ 2 s
     └────────── 用户点「停止」 ──── movej(q_now) ◀────┘ 自动升级一次
```

### 5.1 三个活跃态

| 态 | 下发 | 退出条件 |
| --- | --- | --- |
| `ALIGNING` | `move_js(clamp(q_master), dq=[align_rate]*n)` | `max_j \|q_master_j − q_slave实测_j\| ≤ 0.02 rad` 持续 5 拍 |
| `FOLLOWING` | `move_js(clamp(q_master), dq=[follow_rate]*n)` | watchdog 超时 / 用户停止 |
| `HOLDING` | 起手 `park()` + **不发** `move_js`；持续 ≥ 2 s 后补一次 `movej(q_now)` | 连续收帧 5 拍 ⇒ `ALIGNING`（**升级后仍是 `HOLDING`**，只是持位机制换成 `ht_on`，回 `ALIGNING` 的条件不变） |

- **`align_rate` 默认 0.3 rad/s**（≈17°/s，界面可调 0.1~1.0）。这就是"慢速对齐"——不需要 PC 侧算 ramp，直接交给固件的 `slew_linear`。
- **`follow_rate` 默认 6.0 rad/s**，**故意高于各轴 `speed_limit`(1.75~3.5)** ⇒ 固件的 `speed_limit` 成为事实上限，语义就是"跟着主臂，能多快多快"。
- **收敛判据用实测 q**（`get_state().q`），不是指令 q —— 只有实测到位才算对齐。
- **`ALIGNING → FOLLOWING` 不会造成阶跃**：切换的条件是误差已 ≤ 0.02 rad，此时换多大的 `dq` 都不会跳。
- **`ALIGNING` 卡住检测**：连续 >20 s 未收敛 ⇒ 停在 `ALIGNING` + 告警（**不自动进 `FOLLOWING`**）。常见原因：主臂正在被人拖动。界面须显示"对齐中，请勿移动主臂"+ 剩余最大误差 + 已用时。

> **为什么不是 litearm-server 的 `movej` 粗对齐 + `joint_follow`**：`joint_follow` 在 `litearm-python` 上不存在；而 §2.3(a) 一旦成立，`movej` 那一段就完全冗余
> —— 它是同一个 `move_js` 换 `dq` 就能表达的东西，却要平白引入一次模式切换和一次**阻塞式等到位**（不可中断）。详见 §9。

### 5.2 watchdog

- 判据：`now − last_frame_local_ts > watchdog_ms`（默认 **200 ms**，10× 于固件 100 ms 看门狗）。
- 首帧前的"永远收不到帧"是**另一件事**（稳态 watchdog 只在收到首帧后才生效）⇒ 单列诊断："启动后 N 秒未收首帧"，默认 5 s，`[源码]` 沿用 server 的做法。
- 触发动作：**`park()` + 停发 → `HOLDING`**。
  - `park()` 让 fail-soft 走 `kp = 1.0×` 分支（§2.3(b)），**瞬时、无模式切换、零运动**。HOLDING 预期是短抖动，不该引起任何臂的运动。
  - **不 `disable()`、不 `emergency_stop()`** —— 那是失能自由落体。
- **HOLDING ≥ 2 s 自动升级**：`movej(q_now, speed=0.3)` 受控接管 ⇒ 进 `ht_on`（2× 刚度 + 重力前馈）真正稳住。升级是**一次性**的，且**升级后仍停在 `HOLDING`**（不是
  `IDLE`）—— 只是持位机制从"`park()` + fail-soft"换成 `ht_on`，回 `ALIGNING` 的条件（连续收帧 5 拍）不变。
  理由：`park()` 只把刚度钉回 1.0×、`tau` 仍是 0 ⇒ 长时间持位会以 `G(q)/mit_kp` 的稳态误差缓慢下垂（§2.3(b) 三段表）。
- **恢复不自动进 `FOLLOWING`**：主臂在断链期间可能已经动了，直接跟会阶跃。必须重走 `ALIGNING`。

### 5.3 停止 / 收尾序列（**顺序与时限是硬要求**）

```text
从臂「停止」:  1) 停发 move_js
              2) 立即 movej(q_now_实测, speed=0.3)      ← 必须在上一条 move_js 后 < 100 ms 内
              3) 状态 → IDLE
主臂「停止」:  1) 停止采样与发布
              2) zero_g_stop()
              3) 立即 movej(q_now_实测, speed=0.3)      ← 同上 < 100 ms
```

- **`< 100 ms` 的理由**：固件命令看门狗 100 ms。在 100 ms 内发出下一条会 `watchdog_kick()` 的命令（`movej` 在 `MOVE_J` 模式下每周期自踢），就**根本不进
  fail-soft**，没有下垂窗口。
- 主臂那条尤其不能省：`ctrl_zero_g_leave()` **只在有轴越限时**才自动发回位 `movej`（§2.3(e)4），不越限时 mode 直接落 `INIT`。而 `INIT` 落 `default:`「保持持位」分支（§2.3(b)
  三段表）⇒ **1.0× `mit_kp` 但 `tau = 0`**，垂到 `G(q)/mit_kp`；100 ms 后看门狗把刚度**降到 0.6×**，再垂到 `G(q)/(0.6·mit_kp)`。`movej(q_now)`
  把这两次下垂一次性换成 `ht_on` 的 **2× 刚度 + `G(ht_q)` 前馈**。
- `movej(q_now)` 的目标由固件钳到软限位（§2.3(e)1）⇒ 即使 `q_now` 在限外也安全（会被带回限内），与 `ctrl_zero_g_leave` 的 A3 fix 同向。
- ⚠ **`movej()` 会阻塞**：`movej` 在 ACK 之后进 `_arrive()` 等到位，上界 = `arm.move_timeout`（**默认 15.0 s**）。收尾路径取的是"目标 = 当前实测
  q"，实际几十毫秒就回；但这**不是上界**（链路死掉或臂被挡住时会耗满）。故：
  - 本工具**构造时显式传 `move_timeout=3.0`**，并在 `connect()` 之后**断言它仍是 3.0**（`connect()` 会用当前值重建内部 `_cart`；将来 SDK 若改成在 `connect()`
    里重置，这个断言先红，而不是静默退回 15 s —— 见 §12）。
  - 由此产生的**已知限制**：`ArmWorker` 在收尾 `movej` 期间最长阻塞 3 s。此期间「启动/停止」按钮排队；**急停不排队**（§7.4 旁路线程）。

---

## 6. 主臂循环

1. 连接 + `enable()` + 断言无 `faulted` / `joint_fault`。
2. `zero_g_start()` ⇒ 固件有 **0.35 s engage 段**（低刚度托举 `τ=G+墙弹簧`）。等待 ≥0.5 s 再开始发布（engage 期间臂在受控托举，采样值无意义但不危险）。
3. 100 Hz：`st = get_state(refresh=False).value`；`st is None` ⇒ 计数并跳过；否则 `pub.put(帧)`。
4. 停止：见 §5.3。

**主臂不做任何钳位**：零重力下臂由人拖动，固件**刻意跳过位置/超速包络锁存**（§2.3(e)，"用户可任意拖拽"）。PC 侧插手钳位反而会限制拖动范围。发布的是**实测值**，钳位是从臂的事。

---

## 7. 安全

### 7.1 闸门清单

| 场景 | 行为 |
| --- | --- |
| 软限位读不到（`all_joint_params()` 抛错 / 值非法） | **拒绝启动跟随**，停在 `IDLE` + 响亮报错。**不静默退化**（不学 server 的 ±9 兜底告警） |
| 每帧目标 | `clamp(q_master, q_min, q_max)` —— **纵深**（固件自己也钳，§2.3(e)1），同时让界面能显示"被钳了" |
| `move_js` 单帧抛异常 | 捕获 + 计数 + 循环继续；**不终止跟随**（一帧异常不该让臂停） |
| `move_js` 连续 N 次 ACK 超时（默认 5） | 自动转 `HOLDING` + 告警。理由：`_cmd` 每次等 ACK、超时 1.2 s ⇒ 连续超时意味着链路已不可用 |
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
| **链路** | 角色单选、IP/端口、[连接臂]/[启动遥操]/[停止]、固件版本、许可状态、连接灯 |
| **关节** | 7 轴表 `J \| q \| dq \| tau \| t_mos \| t_coil \| err`；`err != 0` 的轴高亮 |
| **遥操** | 状态机状态；对齐进度条 + 剩余最大误差 + 已用时；**主臂 q vs 从臂 q 对照表 + 逐轴跟踪误差**；收/发频率、本机延迟、丢帧数、ACK 超时计数；参数（`align_rate` / `follow_rate` / `watchdog_ms` / 收敛阈值） |

**温度只显示数值，界面不实现阈值判据**：固件的温度锁存已经反映在 `t_mos/t_coil` + `err` + `joint_fault` 上，界面另发明一套阈值就是在制造第二份真相。唯一的颜色判据是 `err != 0`。

**从臂端**额外显示"主臂 vs 本臂"对照与跟踪误差（数据免费，已在手上）；**主臂端**显示发布频率与已发帧数。

---

## 9. 与 litearm-server 遥操的对应关系

### 9.1 直接沿用（结构）

| 沿用项 | 出处 |
| --- | --- |
| 采样与发布**解耦**（回调只写共享槽，独立线程定频发） | `teleop_manager.py:163-206` |
| 从臂**首帧前先把自身当前 q 灌进目标**（"绝不返回垃圾值"） | `teleop_manager.py:258-264`（对齐时的再同步在 `:309-312`） |
| 每帧**钳位到软限位** | `_read_safe_limits` |
| watchdog 200 ms + **不自动重连** | `teleop_manager.py:349-358` |
| **"N 秒未收首帧"单独诊断**（与稳态 watchdog 是两件事） | `teleop_manager.py:317-322` |
| 遥操状态**派生自控制环真实死活**，不是独立 bool ⇒ 无"停了但锁没解"的半退出 | relay spec §3.1 |
| 对齐**分两段语义**：慢速趋近 → 高速跟随 | `_do_align` + `joint_follow(speed_limit=)` |

### 9.2 必须偏离（接口不存在）

| litearm-server（pylitearm） | 本仓（litearm-python） |
| --- | --- |
| `arm.zero_gravity(on_sample=)` | `zero_g_start()` + 读 `get_state()` 缓存 |
| `arm.joint_follow(target_provider, K, B, speed_limit, accel_limit)` | **自己写 100 Hz `move_js` 环**，限速交给固件 `slew_linear`（§2.3(a)） |
| `arm.request_stop()`（就地高刚度持位） | `movej(q_now, speed=0.3)` 受控接管（`park()` 只作瞬态兜底，§2.3(c)） |
| `arm.kin.q_min/q_max` | `params.all_joint_params()` → `q_min/q_max`（**与固件钳位同源 ⇒ 更权威**） |
| `pylitearm` 的 `joint_limit_margin_rad` | 不需要：固件软限位本身已含 1° 余量，再减 margin 是双重收紧 |

### 9.3 本设计**结构性消除**的两处 server 已知缺陷

1. **`movej → joint_follow` 交接竞态**：server 在 main 上靠"对齐后把共享目标同步到对齐位置"打补丁（`teleop_manager.py:307-313`）。本设计把交接变成状态机里一个有明确判据（误差 ≤ 0.02
   rad 持续 5 拍）的**必然阶段**，且 `ALIGNING`/`FOLLOWING` 是**同一个 `move_js` 调用只换 `dq`** ⇒ 不存在模式切换的交接面。
2. **对齐期间不可中断**：server 的 `_do_align` 阻塞单 worker 最长 5 s，期间只能靠急停中止（relay spec §3.1
   自列为已知限制）。本设计的**伺服环（`ALIGNING`/`FOLLOWING`）完全没有阻塞式等到位** —— 它只是 100 Hz 发 `move_js`，每 10 ms 回一次头，停止随时生效。
   > ⚠ **但收尾路径不是无阻塞的**：两条收尾都要 `movej(q_now)`（§5.3），而 `movej()` 会经 `_arrive()` 阻塞到 `arm.move_timeout`。本工具把它显式设成 3
   > s，并让**急停走旁路**（§7.4）—— 所以"随时可停"成立于伺服环与急停，**不**成立于收尾 `movej` 自身。

---

## 10. 陷阱登记（写进 README，供后来者）

| # | 陷阱 | 等级 |
| --- | --- | --- |
| 1 | `move_js` 的 `dq`：**`\|dq\|` 是 `q_ref` 走位速率上限，`dq=0` 冻结该轴**；符号被 `fabsf` 丢弃。SDK `DEVELOPER_GUIDE.md:259` 只说了"不是 limit"，漏了这半句 | `[源码]` |
| 2 | **别把主臂 `dq` 直接当从臂 `move_js` 的 `dq`** —— 主臂一停（`dq≈0`），从臂立刻冻住 | `[源码]`，由陷阱 1 推出 |
| 3 | watchdog fail-soft = **0.6× 刚度 + `τ=0` ⇒ 缓慢下垂**（既非自由落体，也非稳住） | `[源码]` |
| 4 | 只有 `movej` 到位后的 `ht_on` 态才真稳（2× 刚度 + 重力前馈）⇒ **收尾必须 `movej(q_now)`** | `[源码]` |
| 5 | `park()` 只把 fail-soft 刚度**拉回** 1.0×（阻止 0.6× 那次降刚度），**`tau` 仍为 0 ⇒ 仍会垂 `G/mit_kp`**。不是 `request_stop` 的等价物 | `[源码]` |
| 6 | **急停 = EMERGENCY = 全失能 ⇒ 自由落体**，与"停止遥操"是两件事 | `[源码]` |
| 7 | `set_speed()` 经 `gov_ratio` **整体缩小走位速率** ⇒ 本工具绝不调用 | `[源码]` |
| 8 | `litearm-python` **没有** `joint_follow`/`request_stop`/`zero_gravity(on_sample=)` ⇒ **别照抄 litearm-server 的遥操实现** | `[源码]` |
| 9 | Zenoh session **不 `close()` ⇒ 进程退出永久挂死** | `[实测]` |
| 10 | STM32 CDC 在 Linux **不独占** ⇒ 一个口只允许一个进程 | 仓外实测 |
| 11 | **1J 台架 vs 7J 整臂是两张互斥的默认值表**：台架 `speed_limit=3.5`、`hold_kp_gain` **未设 ⇒ 0** ⇒ 台架板上 `ht_on` 会下发 `kp=0`（`movej` 收尾反而失力）。**任何涉及 `speed_limit`/`hold_kp_gain`/`can_dyn` 的结论必须写明板卡** | `[源码]` |
| 12 | 本项目引固件/SDK 事实**一律按符号定位、不按行号** —— 实测行号会漂（`clampf` 716 vs 722、`watchdog_kick` 725 vs 733、`ARCHITECTURE` 262 vs 261），且条件编译表会让人读错表（我自己就把台架表的 3.5 当成 J1 的） | `[实测]` |
| 13 | 本仓 key 与 server 的 topic **不可混用**（帧格式不同，会静默解错） | 设计约定 |

---

## 11. 真机 spike（**先于一切实现**）

> **为什么先做**：§2.3(a) 的 `dq` 语义是整个伺服环的地基，而它目前只是 `[源码]`。离线部分（状态机、界面）全都写在这个语义之上 —— 语义若错要返工，而 spike 只接一条臂、几十行、跑几十秒。
> **唯一例外**：臂此刻接不上时，先做 §12 里**与 `dq` 语义无关**的纯离线三件（`wire.py` / `link.py` / `test_link.py`），它们不会白做。
>
> ⛔ **S1~S4 必须在 7 关节整臂固件上做，不许在 `LITEARM_BENCH_1J` 单电机台架上做。**
> 台架固件的 `.dyn` 初始化器**未设 `hold_kp_gain` ⇒ 取 0**，于是 `ht_on` 下发
> kp = 0、`tau = 0`；`can_dyn=false` 也让 `ht_tauf = 0`。后果：**台架板上 `movej` 收尾会
> 逐渐完全失力**，S4 会得出"`movej` 比只停发更差"的结论 —— 那是台架构型造成的，
> 与本设计无关。spike 报告必须写明**板卡与固件版本串**。

| # | 验什么 | 通过判据 |
| --- | --- | --- |
| **S1** | **`dq=0` 是否真的冻结该轴** | 发 `move_js(q_now + 0.1, dq=0)` 持续 2 s，臂**不动**（`\|Δq\| < 0.005 rad`） |
| **S2** | `dq` 与实测走位速率的关系 | 发 `move_js(q_now + 0.2, dq=0.3)`，实测速率 ≈ 0.3 rad/s（±20%）；`dq=6.0` 时实测速率 ≈ 该轴 `speed_limit` |
| **S3** | 100 Hz `move_js` 连续 30 s 的 **ACK 返回率** | ACK 成功率 ≥ 99.9%，实测下发频率 ≥ 95 Hz，且**无 1.2 s 级卡顿**（`_cmd` 超时上界） |
| **S4** | 收尾：`movej(q_now)` **vs** 只停发的 **A/B 对照** | 录 30 s 的 `q` 漂移。预期：只停发 ⇒ 下垂后稳定于 `G(q)/kp` 量级的偏移；`movej(q_now)` ⇒ 漂移显著更小。**必须两组都做** |

**S4 是 A/B 对照，不许只做单工况** —— 本仓外有两次被单工况结论骗过的记录。若 S4 的两组差异不显著，则 §5.3 的 `movej` 收尾**证据不足**，须重新评估而不是照抄本 spec。

**S1~S3 任一不通过时的退路**：S1/S2 不通过 ⇒ `dq` 语义读错，整个伺服环需重设计；S3 不通过 ⇒ 改用 `arm._raw_write(0x03, payload)` 不等 ACK（代价：失去 ACK
 级的错误上报，须自行补状态帧活性判据）。

---

## 12. 文件结构与测试

```text
litearm-teleop-isomorphic/
  liteteleop/
    wire.py          帧编解码（纯函数、零依赖；版本校验；变长 n）
    link.py          Zenoh 点对点封装（listen/connect 配置、close 纪律）
    safety.py        软限位校验、钳位、watchdog、收敛判据、状态机（纯逻辑，可测）
    arm_worker.py    独占 Arm 的线程：连接/使能/遥操循环/收尾序列
    settings.py  ports.py  app.py  __main__.py
    gui/  bridge.py  main_window.py  widgets.py
          pages/  link_page.py  joints_page.py  teleop_page.py
  tests/
    test_wire.py  test_link.py  test_safety.py  test_arm_worker.py  test_gui_smoke.py
```

| 测试 | 判据要点 |
| --- | --- |
| `test_wire.py` | **黄金字节**钉死小端布局（**不用往返测试 —— 往返对字节序无判别力**）；`version` 不符 ⇒ 拒收；`n` 不符 / 短帧 ⇒ 拒收；n = 1 与 n = 7 都覆盖 |
| `test_link.py` | **真起两个 zenoh session 走回环**（非 mock）：100 帧逐字节全等；100 Hz 持续 2 s 零丢包；**子进程**验证"显式 `close()` ⇒ 能退出 / 不 close ⇒ 挂死"（后者用超时断言，标记为慢测） |
| `test_safety.py` | 钳位；限位读不到 ⇒ **拒启动**；watchdog 判定；状态机**全部迁移**（含 `HOLDING` 2 s 升级、回 `ALIGNING` 的 5 拍条件、`ALIGNING` 20 s 卡住）；收敛判据用**实测 q** |
| `test_arm_worker.py` | `FakeArm` 驱动：100 Hz 节拍；ACK 连续超时 ⇒ 转 `HOLDING` 且**不终止循环**；**停止序列顺序 + < 100 ms 时限**（假时钟，可判别）；**`connect()` 后 `arm.move_timeout == 3.0`**（防将来 SDK 在 `connect()` 里重置）；**worker 被卡在 `movej` 期间，急停旁路仍能成功发出**（§7.4） |
| `test_gui_smoke.py` | `QT_QPA_PLATFORM=offscreen` 起窗口；角色切换；闸门锁定/解锁 |

**离线的两道闸门（编译 / 对拍）在本仓全部可跑**：`test_link.py` 起的是真 zenoh，不是打桩。

**真机才能验的（`[源码]`→`[实测]` 的升级清单）**：§11 的 S1~S4，外加：主臂零重力下 100 Hz `get_state()` 的稳定性；`all_joint_params()` 读回的软限位与固件实际软限位一致。

---

## 13. 里程碑

1. **真机 spike S1~S4**（§11，**7 关节整臂固件**）—— 证伪优先。产出：`move_js` 的 `dq` 语义实测结论 + 收尾 A/B 结论，回写本 spec 的 §2.3。
2. **纯离线三件**：`wire.py` + `link.py` + `test_link.py` + `test_wire.py`。
3. **`safety.py` + `test_safety.py`**：状态机与判据（纯逻辑，不碰硬件）。
4. **`arm_worker.py` + `test_arm_worker.py`**：接 `FakeArm`，跑通节拍与收尾序列。
5. **GUI + `test_gui_smoke.py`**：四页 + 闸门。
6. **本机双臂端到端**：两个进程走 `127.0.0.1`，跑通"对齐 → 跟随 → 停止"。
7. **跨机单臂**：借部署节点验证真实网络下的延迟与丢帧表现。

每一步的验收都必须**真跑过并贴出输出**，不以"看起来对"结案。

**计划拆分建议**：7 个里程碑对单个计划偏大。天然断点在 **2~3**（协议层 + `safety`：纯离线、无硬件、可整段 TDD）与 **4~6**（`arm_worker` + GUI +
 端到端：需硬件）。建议**至少拆成两个计划**，第一个到里程碑 3 为止（它能在 spike 之前或之后独立完成，且不被 `dq` 语义之外的任何未知挡住）。
