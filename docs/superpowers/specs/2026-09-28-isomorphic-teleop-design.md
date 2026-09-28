# 同构遥操（主从，Zenoh 点对点）设计

> 日期：2026-09-28
> 状态：待评审
> 目标仓：`litearm-teleop-isomorphic`
> 依赖：`litearm-python`（直连 CDC，**零 SDK 改动**）、`litearm-stm32` 固件（`Litearm1.5.x`）、`eclipse-zenoh` 1.7.2、PyQt5
> 参考实现：[litearm-server 遥操](file:///home/llx/litearm-server/docs/superpowers/specs/2026-08-13-teleop-relay-design.md)（**只参考结构，执行路径必须换**，见 §9）

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

### 2.3 固件语义 `[源码]` —— 本轮最关键的四条

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
- `speed_limit` 默认值：J1 = 3.5 rad/s，J2~J7 = 1.75~2.0 rad/s（`params/defaults.c:34-70`）。
- **这条给了设计一个大便宜**：固件的 `slew_linear` 就是现成的限速器，`dq` 就是旋钮 ⇒ "慢速对齐 / 高速跟随"= **同一个 `move_js` 调用只换 `dq`**，PC 侧不需要手写 ramp。

> ⚠ **SDK 文档在此处不完整**：`DEVELOPER_GUIDE.md:259` 只写 "`dq` is a velocity reference, not a limit"，**漏掉了"`|dq|` 同时是 `q_ref` 的斜率上限、`dq=0` 冻结该轴"这半句**。它不算错（`dq` 确实是参考值，且 `vel_max` 另有一道钳位），但按它字面推理会写出"臂不动"的代码。登记为陷阱 1（§10），**不改 SDK**。

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

#### (c) 只有 `movej` 到位后的 `ht_on` 态才**真稳**

```c
/* control_loop.c:2292-2313  到位增刚过渡 */
uint16_t steps = 0.2f / LITEARM_CTRL_DT;               /* 0.2s smoothstep */
float kp_hold = jp->mit_kp * g_params.dyn.hold_kp_gain;  /* 2.0×, 钳到 500 */
tau_u[i] = ht_tau0[i] + alpha * (ht_tauf[i] - ht_tau0[i]);  /* 终点 = dyn_gravity(ht_q) */
```

⇒ **2× 刚度 + 重力前馈**。这是唯一"稳住不下垂"的持位态。

⇒ **收尾必须 `movej(q_now, speed≈0.3)` 受控接管**，不能只停发。

> `park()` 我原先以为等价于 `request_stop`（就地高刚度持位）—— 查源码后降级：它只把 fail-soft 的刚度从 0.6× 提到 1.0×，**`τ` 仍是 0 ⇒ 仍会垂**。仅用作瞬态兜底（§5.3）。

#### (d) 急停 = EMERGENCY = **全失能自由落体**

`急停 / 包络越限 → EMERGENCY(全 DISABLE, 只可 RESET 恢复)`（`ARCHITECTURE-USB-CMD.md:261`）。

⇒ 与"停止遥操"是**两件不同的事**，界面必须分开且写明（§6）。

#### (e) 另外三条读源码得到的

1. **固件自己也钳位**：`ctrl_accept_move_js` 里 `target_q[i] = clampf(q[i], q_min, q_max)`（`control_loop.c:722`）⇒ PC 侧钳位是**纵深**，不是"唯一护栏"。
2. **`move_js` 自己 kick 看门狗**：`ctrl_accept_move_js` 末尾 `watchdog_kick()`（`control_loop.c:733`）⇒ 100 Hz 重发即保活。注意 `0x20`/`0x21` **刻意不 kick**。
3. **`set_speed()` 会整体缩小走位速率**：`v_lim` 里乘了 `gov_ratio`（`0x21` 的全局调速器）⇒ **本工具绝不调用 `set_speed()`**。
4. **零重力退出时的越限轴由固件自动低速带回**（`ctrl_zero_g_leave` 的 A3 fix，`control_loop.c:794-805`）⇒ PC 侧不需要处理，但界面要提示。
5. **零重力中 `move_js` 被拒**（`0x04`，`control_loop.c:696`）⇒ 主臂进程不会误发伺服命令。

---

## 3. 架构

```
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

**为什么必须单线程独占 `Arm`**：`litearm-python` 的 `_Ack` 是**单消费者**设计 —— 两条读线程会互相抢 ACK，表现为随机的"无应答(超时)"。`get_state(refresh=False)` 只读缓存、不取帧，所以伺服环读状态是安全的；但**所有下行动作命令必须串行**。

**Zenoh 回调只写槽**:`move_js` 是阻塞调用（每帧等 ACK），不能放进回调线程；回调只做"赋值给 latest 槽"，避免 Zenoh 线程被伺服循环拖住。

### 3.2 数据流

```
主臂:  Arm → (SDK 读线程) → get_state 缓存 → ArmWorker 100Hz 读 → pub.put(帧)
从臂:  sub 回调 → latest 槽 → ArmWorker 100Hz 读槽 → 钳位 → move_js
```

采样与发布在从臂侧**不排队**：只用 `latest` 槽，迟到帧直接覆盖。理由与 litearm-server 的 `drain_latest()` 一致 —— 遥操只要最新姿态，积压帧会让从臂追一条过期的轨迹。

---

## 4. 线协议

### 4.1 Key

默认 `litearm/teleop/isomorphic`，界面可改，**两端必须一致**。

> ⚠ **刻意不用** litearm-server 的 `litearm/v4/{arm_id}/teleop`：server 的帧是 `>15d` 120 B，本文的是 70 B。共用 key 会让两端把对方的帧**静默解错**。key 不同 ⇒ 不可能误配。

### 4.2 帧格式（v1，小端，变长关节数）

```
offset  size   field                                  说明
0       1      version = 1                            协议版本；不符 ⇒ 拒绝启动跟随
1       1      n                                      关节数（1..255）
2       4*n    q[n]    f32 LE                         主臂实测关节角 rad
2+4n    4*n    dq[n]   f32 LE                         主臂实测关节速度 rad/s
2+8n    8      ts      f64 LE                         主臂 time.monotonic()
2+8n+8  4      seq     u32 LE                         主臂帧序号
```

n = 7 时共 **70 B**。

**设计说明**

- **`version` 字节是防"静默解错"的那道判据**。两端版本不符时**拒绝启动跟随并响亮报错**，而不是照 n 去解一帧垃圾。
- **`dq` 只用于显示与遥测，绝不直接喂给从臂的 `move_js`**（见 §5.2 陷阱 2）。
- **`ts` 取主臂 `time.monotonic()`**：跨机时两端时钟不同源，所以它**只能算"同一主臂相邻两帧的间隔"**（帧间隔抖动），**不能算端到端延迟**。本工具显示的"端到端延迟"是**本机**从"收到帧"到"下发 move_js"的耗时，与 `ts` 无关。
- 小端（`<`）**是本仓的约定**，与 server 的 `>`（大端）不同。§8 的 `test_wire.py` 用**黄金字节**钉死它，不用往返测试 —— **往返测试对字节序没有判别力**（两端同错也全绿）。

---

## 5. 从臂状态机

```
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
| `HOLDING` | **不发** `move_js` | 连续收帧 5 拍 ⇒ `ALIGNING`；持续 ≥ 2 s ⇒ 升级 `movej(q_now)` |

- **`align_rate` 默认 0.3 rad/s**（≈17°/s，界面可调 0.1~1.0）。这就是"慢速对齐"——不需要 PC 侧算 ramp，直接交给固件的 `slew_linear`。
- **`follow_rate` 默认 6.0 rad/s**，**故意高于各轴 `speed_limit`(1.75~3.5)** ⇒ 固件的 `speed_limit` 成为事实上限，语义就是"跟着主臂，能多快多快"。
- **收敛判据用实测 q**（`get_state().q`），不是指令 q —— 只有实测到位才算对齐。
- **`ALIGNING → FOLLOWING` 不会造成阶跃**：切换的条件是误差已 ≤ 0.02 rad，此时换多大的 `dq` 都不会跳。
- **`ALIGNING` 卡住检测**：连续 >20 s 未收敛 ⇒ 停在 `ALIGNING` + 告警（**不自动进 `FOLLOWING`**）。常见原因：主臂正在被人拖动。界面须显示"对齐中，请勿移动主臂"+ 剩余最大误差 + 已用时。

> **为什么不是 litearm-server 的 `movej` 粗对齐 + `joint_follow`**：`joint_follow` 在 `litearm-python` 上不存在；而 §2.3(a) 一旦成立，`movej` 那一段就完全冗余 —— 它是同一个 `move_js` 换 `dq` 就能表达的东西，却要平白引入一次模式切换（`MOVE_JS → MOVE_J → MOVE_JS`）和一次**阻塞式等到位**（不可中断）。详见 §9。

### 5.2 watchdog

- 判据：`now − last_frame_local_ts > watchdog_ms`（默认 **200 ms**，10× 于固件 100 ms 看门狗）。
- 首帧前的"永远收不到帧"是**另一件事**（稳态 watchdog 只在收到首帧后才生效）⇒ 单列诊断："启动后 N 秒未收首帧"，默认 5 s，`[源码]` 沿用 server 的做法。
- 触发动作：**`park()` + 停发 → `HOLDING`**。
  - `park()` 让 fail-soft 走 `kp = 1.0×` 分支（§2.3(b)），**瞬时、无模式切换、零运动**。HOLDING 预期是短抖动，不该引起任何臂的运动。
  - **不 `disable()`、不 `emergency_stop()`** —— 那是失能自由落体。
- **HOLDING ≥ 2 s 自动升级**：`movej(q_now, speed=0.3)` 受控接管 ⇒ 进 `ht_on`（2× 刚度 + 重力前馈）真正稳住。理由：`park()` 只解决刚度、`τ` 仍为 0，长时间持位会以 `G(q)/kp` 的稳态误差缓慢下垂（见 §10 陷阱 3）。
- **恢复不自动进 `FOLLOWING`**：主臂在断链期间可能已经动了，直接跟会阶跃。必须重走 `ALIGNING`。

### 5.3 停止 / 收尾序列（**顺序与时限是硬要求**）

```
从臂「停止」:  1) 停发 move_js
              2) 立即 movej(q_now_实测, speed=0.3)      ← 必须在上一条 move_js 后 < 100 ms 内
              3) 状态 → IDLE
主臂「停止」:  1) 停止采样与发布
              2) zero_g_stop()
              3) 立即 movej(q_now_实测, speed=0.3)      ← 同上 < 100 ms
```

- **`< 100 ms` 的理由**：固件命令看门狗 100 ms。在 100 ms 内发出下一条会 `watchdog_kick()` 的命令（`movej` 在 `MOVE_J` 模式下每周期自踢），就**根本不进 fail-soft**，没有下垂窗口。
- 主臂那条尤其不能省：`ctrl_zero_g_leave()` **只在有轴越限时**才自动发回位 `movej`（§2.3(e)4），不越限时 mode 直接落 `INIT`，而 `INIT` 不在控制环的 `switch (mode)` 分支里 ⇒ `kp=kd=τ=0` ⇒ **约 100 ms 完全失力**，之后才被 fail-soft 以 0.6× 接住。
- `movej(q_now)` 的目标由固件钳到软限位（§2.3(e)1）⇒ 即使 `q_now` 在限外也安全（会被带回限内），与 `ctrl_zero_g_leave` 的 A3 fix 同向。

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
| **急停** | `emergency_stop()` 常驻按钮。**界面写明"急停 = 全失能 ⇒ 臂自由落体"**，并把「停止遥操」（受控接管、稳住）作为并列的常用按钮 |
| **绝不调用** | `set_speed()`（§2.3(e)3）、`disable()`（收尾路径）、`save_params()`（不可逆，见仓外记忆） |
| 串口独占 | STM32 CDC 在 Linux **不独占**，第二个进程会分吃同一字节流 ⇒ 文档 + 界面写明"一个 CDC 口只允许一个进程" |

### 7.2 界面闸门（照抄 litetool 既有做法）

`☑ 我已确认机械臂周围无障碍、急停可及` —— 勾选前运动类按钮一律锁定；**断线 / 急停 / FAULT / 复位自动重新锁定**。

### 7.3 退出纪律

- Zenoh `session.close()` **必须在进程退出前显式调用**，否则**进程永久挂死**（§2.1 `[实测]`）。
- 收尾顺序：停遥操（§5.3）→ `close()` zenoh session → `arm.close()`。
- SDK 侧同样有 `close()` 纪律（仓外既有教训：忘了它进程退出永久挂死）。

---

## 8. 界面

仿 `litearm-tool-stm32/litetool` 的形状：顶栏 + 常驻状态条 + 左侧导航 + 页面栈 + 底部常驻日志面板。

```
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
| 从臂**首帧前先把自身当前 q 灌进目标**（"绝不返回垃圾值"） | `teleop_manager.py:277-283` |
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

1. **`movej → joint_follow` 交接竞态**：server 在 main 上靠"对齐后把共享目标同步到对齐位置"打补丁（`teleop_manager.py:307-313`）。本设计把交接变成状态机里一个有明确判据（误差 ≤ 0.02 rad 持续 5 拍）的**必然阶段**，且 `ALIGNING`/`FOLLOWING` 是**同一个 `move_js` 调用只换 `dq`** ⇒ 不存在模式切换的交接面。
2. **对齐期间不可中断**：server 的 `_do_align` 阻塞单 worker 最长 5 s，期间只能靠急停中止（relay spec §3.1 自列为已知限制）。本设计**无任何阻塞式等到位**，`ArmWorker` 的循环每 10 ms 回一次头 ⇒ 停止/急停随时生效。

---

## 10. 陷阱登记（写进 README，供后来者）

| # | 陷阱 | 等级 |
| --- | --- | --- |
| 1 | `move_js` 的 `dq`：**`\|dq\|` 是 `q_ref` 走位速率上限，`dq=0` 冻结该轴**；符号被 `fabsf` 丢弃。SDK `DEVELOPER_GUIDE.md:259` 只说了"不是 limit"，漏了这半句 | `[源码]` |
| 2 | **别把主臂 `dq` 直接当从臂 `move_js` 的 `dq`** —— 主臂一停（`dq≈0`），从臂立刻冻住 | `[源码]`，由陷阱 1 推出 |
| 3 | watchdog fail-soft = **0.6× 刚度 + `τ=0` ⇒ 缓慢下垂**（既非自由落体，也非稳住） | `[源码]` |
| 4 | 只有 `movej` 到位后的 `ht_on` 态才真稳（2× 刚度 + 重力前馈）⇒ **收尾必须 `movej(q_now)`** | `[源码]` |
| 5 | `park()` 只把 fail-soft 刚度提到 1.0×，**`τ` 仍为 0 ⇒ 仍会垂**。不是 `request_stop` 的等价物 | `[源码]` |
| 6 | **急停 = EMERGENCY = 全失能 ⇒ 自由落体**，与"停止遥操"是两件事 | `[源码]` |
| 7 | `set_speed()` 经 `gov_ratio` **整体缩小走位速率** ⇒ 本工具绝不调用 | `[源码]` |
| 8 | `litearm-python` **没有** `joint_follow`/`request_stop`/`zero_gravity(on_sample=)` ⇒ **别照抄 litearm-server 的遥操实现** | `[源码]` |
| 9 | Zenoh session **不 `close()` ⇒ 进程退出永久挂死** | `[实测]` |
| 10 | STM32 CDC 在 Linux **不独占** ⇒ 一个口只允许一个进程 | 仓外实测 |
| 11 | 本仓 key 与 server 的 topic **不可混用**（帧格式不同，会静默解错） | 设计约定 |

---

## 11. 真机 spike（**先于一切实现**）

> **为什么先做**：§2.3(a) 的 `dq` 语义是整个伺服环的地基，而它目前只是 `[源码]`。离线部分（状态机、界面）全都写在这个语义之上 —— 语义若错要返工，而 spike 只接一条臂、几十行、跑几十秒。
> **唯一例外**：臂此刻接不上时，先做 §12 里**与 `dq` 语义无关**的纯离线三件（`wire.py` / `link.py` / `test_link.py`），它们不会白做。

| # | 验什么 | 通过判据 |
| --- | --- | --- |
| **S1** | **`dq=0` 是否真的冻结该轴** | 发 `move_js(q_now + 0.1, dq=0)` 持续 2 s，臂**不动**（`\|Δq\| < 0.005 rad`） |
| **S2** | `dq` 与实测走位速率的关系 | 发 `move_js(q_now + 0.2, dq=0.3)`，实测速率 ≈ 0.3 rad/s（±20%）；`dq=6.0` 时实测速率 ≈ 该轴 `speed_limit` |
| **S3** | 100 Hz `move_js` 连续 30 s 的 **ACK 返回率** | ACK 成功率 ≥ 99.9%，实测下发频率 ≥ 95 Hz，且**无 1.2 s 级卡顿**（`_cmd` 超时上界） |
| **S4** | 收尾：`movej(q_now)` **vs** 只停发的 **A/B 对照** | 录 30 s 的 `q` 漂移。预期：只停发 ⇒ 下垂后稳定于 `G(q)/kp` 量级的偏移；`movej(q_now)` ⇒ 漂移显著更小。**必须两组都做** |

**S4 是 A/B 对照，不许只做单工况** —— 本仓外有两次被单工况结论骗过的记录。若 S4 的两组差异不显著，则 §5.3 的 `movej` 收尾**证据不足**，须重新评估而不是照抄本 spec。

**S1~S3 任一不通过时的退路**：S1/S2 不通过 ⇒ `dq` 语义读错，整个伺服环需重设计；S3 不通过 ⇒ 改用 `arm._raw_write(0x03, payload)` 不等 ACK（代价：失去 ACK 级的错误上报，须自行补状态帧活性判据）。

---

## 12. 文件结构与测试

```
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
| `test_arm_worker.py` | `FakeArm` 驱动：100 Hz 节拍；ACK 连续超时 ⇒ 转 `HOLDING` 且**不终止循环**；**停止序列顺序 + < 100 ms 时限**（假时钟，可判别） |
| `test_gui_smoke.py` | `QT_QPA_PLATFORM=offscreen` 起窗口；角色切换；闸门锁定/解锁 |

**离线的两道闸门（编译 / 对拍）在本仓全部可跑**：`test_link.py` 起的是真 zenoh，不是打桩。

**真机才能验的（`[源码]`→`[实测]` 的升级清单）**：§11 的 S1~S4，外加：主臂零重力下 100 Hz `get_state()` 的稳定性；`all_joint_params()` 读回的软限位与固件实际软限位一致。

---

## 13. 里程碑

1. **真机 spike S1~S4**（§11）—— 证伪优先。产出：`move_js` 的 `dq` 语义实测结论 + 收尾 A/B 结论，回写本 spec 的 §2.3。
2. **纯离线三件**：`wire.py` + `link.py` + `test_link.py` + `test_wire.py`。
3. **`safety.py` + `test_safety.py`**：状态机与判据（纯逻辑，不碰硬件）。
4. **`arm_worker.py` + `test_arm_worker.py`**：接 `FakeArm`，跑通节拍与收尾序列。
5. **GUI + `test_gui_smoke.py`**：四页 + 闸门。
6. **本机双臂端到端**：两个进程走 `127.0.0.1`，跑通"对齐 → 跟随 → 停止"。
7. **跨机单臂**：借部署节点验证真实网络下的延迟与丢帧表现。

每一步的验收都必须**真跑过并贴出输出**，不以"看起来对"结案。
