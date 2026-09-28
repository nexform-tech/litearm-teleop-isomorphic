# `JOINT_FOLLOW`：把伺服环搬进固件 —— 设计

> **状态**：**已实施（S1–S4），待真机验收**（2026-09-28）。
> 固件 + 宿主台：已提交、全绿（26 用例 261 判据）；SDK：契约测试已过。
> ⛔ **§5 的六条真机判据一条都还没跑** —— 那需要烧录固件。
>
> ⚠ 本设计改了三个仓：`litearm-stm32`（固件）、`litearm-python`（SDK）、本仓。
> 前两个在用户「不准动 `litearm-teleop-isomorphic` 以外任何项目」的禁令内 ⇒
> **已逐仓取得明确授权**（两仓各在 `feat/joint-follow` 分支上）。

## 1. 为什么要做

### 1.1 现象与已排除的原因

从臂跟随反复锁存掉力（`joint_fault` 非 0 ⇒ 停发该轴控制帧 ⇒ 达妙电机"收帧才回状态"
⇒ 该轴静默 ⇒ 80 ms 后报 `FB_STALE`）。**`FB_STALE` 始终是次生现象**，今晚逐条排除了：

| 假设 | 排除依据 |
| --- | --- |
| 软增益不稳 | 换 `send_mit_all` + server 真增益后**照样复发** |
| 撞软限位 | 内缩到 0.14 后**仍复发**，且 `flags` 里可以没有 `POS_VIOL` |
| `q_ref` 陈旧 | 判据只在 `MOVE_J`/`MOVE_JS` 跑，`MOVE_MIT_ALL` **不进那段** |
| 位置命令无 slew | **错**：`MOVE_MIT_ALL` 未置 `direct_ref` ⇒ 走 `slew_linear`（`control_loop.c:2225`） |
| `speed_limit` 太高 | **无效**：固件本就把它钳到 `vel_max`，PC 侧再钳一层没有意义 |

**真凶**：`OVERSPEED`。判据 `|dq| > vel_max × 1.5` 连续 5 拍 ⇒ 锁存
（`safety_check.c:197`）。它判的是**实测速度**，而实测速度**会瞬间超过参考推进速度**
—— `τ = kp·(q_ref−q) + kd·(dq_ref−dq) + τ_ff`，kp=60 时弹簧推力足以让瞬态冲过阈值。
**这是控制层瞬态，不是命令层超速** ⇒ 调命令层的任何参数都无效。

真机日志（`21:14:55`）实录该链：`OVERSPEED` 首拍即报 → 5 拍后 J4 锁存 → J6 跟进 → `FB_STALE`。

### 1.2 为什么 server 不会遇到

litearm-server 的从臂**不在固件里**：

```
litearm-server :  PC ──SocketCAN──▶ 电机          （pc 直连，中间无固件）
我们           :  PC ──USB CDC──▶ STM32 ──CAN──▶ 电机
```

而 `joint_follow.step()` 每步都**主动豁免**那两条判据：

```python
hw.assert_operational(measured_overspeed_factor=float('inf'),   # 超速检查关掉
                      skip_position=True)                        # 位置检查跳过
```

⇒ **server 与我们的差别不是"超不超"，而是"判不判"。** 而它敢不判，是因为它没有那
6.7 ms 的 USB 往返延迟 —— 延迟正是瞬态的产地。

### 1.3 结论

**用「PC 每拍算好、发位置」的架构，去模仿一个「固件内常驻伺服」的系统，中间那
6.7 ms 就是所有瞬态的产地。** 本设计把这个环搬到延迟不存在的地方。

## 2. 目标与非目标

**目标**
- 从臂在**大幅度快速**拖动下**不再锁存**（`joint_fault` 保持 0）
- 从臂的**跟踪手感**与 server 一致（K/B 逐值同源，逐帧下发）
- 保留温度、电机 `err`、总线级 `FB_STALE` 的保护

**非目标**
- 不改主臂（零重力拖动那条路已验证可用）
- 不改 `move_j` / `move_js` / `move_mit` 的既有行为（**尤其不能削弱通用路径**）

## 3. 关键设计决策

### 3.1 不新增模式号 —— 复用 `ARM_MODE_MOVE_MIT_ALL`

`litearm.h:122-129` 的模式枚举是 **0..7**，注释写明「状态帧 mode 是 **3 bit**, 7 正好不扩位宽」。
新增第 9 个模式会**改状态帧布局** ⇒ 四端同步（固件/SDK/工具/上位机）。

⇒ **复用 `ARM_MODE_MOVE_MIT_ALL` 的执行路径**，用一个**固件内部子状态**
`s_jf_active` 区分「PD 透传」与「joint_follow」。状态帧的 `mode` 字段**一个比特都不动**。

**进入/退出**：

| 动作 | 效果 |
| --- | --- |
| `CMD_JOINT_FOLLOW(0x08)` | 置 `s_jf_active = true`，模式切到 `MOVE_MIT_ALL`，写入目标与 K/B |
| 任何其它模式命令（`move_j`/`move_js`/`move_mit*`/`zero_g`/`home`…） | **清 `s_jf_active`**（退出 joint_follow） |
| `reset` / `emergency_stop` / `clear_faults` | 同上 |
| 看门狗 fail-soft / 掉线 | 同上 |

⚠ 这条「任何其它命令都清标志」是**安全关键**：否则一个残留的 `s_jf_active`
会让**别的模式**也带着豁免跑，那就等于削弱了通用路径。

### 3.2 命令码：`CMD_JOINT_FOLLOW = 0x08`

`0x08–0x0F` 空闲（`0x01–0x07` 是运动命令、`0x10+` 是使能族）⇒ 取 `0x08` 紧邻运动区。

**帧格式**（大端，与既有运动命令同风格）：

```
CMD_JOINT_FOLLOW (0x08)
  q_target[7]   f32   目标关节角 rad
  dq_ref[7]     f32   目标角速度 rad/s   ← 速度前馈，照 server 的 send_mit
  K[7]          f32   刚度 Nm/rad   （每帧下发）
  B[7]          f32   阻尼 Nm·s/rad （每帧下发）
  = 1 + 112 = 113 B
```

⚠ `dq_ref` **必须带**（定案依据）：server 的最后一次下发是
`hw.send_mit(self.K, kd_vec, self.q_cmd, **self.dq_cmd**, tau_ff)`（`joint_follow.py:313`）
—— `dq_cmd` 作速度前馈进电机 MIT 环。

**应答**：`RSP_ACK{0x08}`（照既有约定）；被拒时 `RSP_ERR{0x08, code}`。

⚠ **`K`/`B` 逐帧下发**（用户裁决 2026-09-28）：与 server 的 `send_mit(K, B, q_cmd, dq_cmd, τ)` 同语义。
调参**不需要重烧固件**。固件把 `K/B` 写入 `cmd->kp`/`cmd->kd` 后走既有的 MIT 收口，**不碰参数表**。

### 3.3 固件内部：300 Hz 常驻环（零件全部复用）

每个 `control_loop_step()` tick（`LITEARM_CTRL_HZ` = 300 Hz）：

```
1. q_ref = slew_linear(q_target, q_ref, vel_max·dt)     ← 复用 control_loop.c:2225 那条
2. τ_ff  = clamp( dyn_gravity(q) + law_wall(q, dq), ±tau_max )   ← 复用，与 ZERO_G 同源
3. τ     = kp·(q_ref − q) + kd·(dq_ref − dq) + τ_ff      ← 电机 MIT 环执行弹簧-阻尼
4. 发 MIT
```

**⇒ PC 只需要把目标「喂」进来**，不必再每拍做两次 USB 往返（`get_gravity` 由固件算掉）。

**限速**：由 `vel_max`（固件整臂表 `[2.0,2.0,1.75,1.75,2.0,2.0,2.0]`）在固件内部保证 ——
**这才是唯一有效的限速点**（PC 侧的 `speed_limit` 早已被它覆盖）。

**墙区固件阻尼（`wall_fw_kd`）在 PC 侧做，固件不用管** —— 照 server 的
`joint_follow.py:306-312`，它只是在发帧前把进墙区那几轴的 `kd` 加上
`firmware_kd_extra`（`litearm.yaml:266` = **0.8**，上限 5.0）：

```python
kd_vec = list(B)
zone = wall.wall_zone_mask(q_meas)
if any(zone):
    for i in range(N):
        if zone[i]: kd_vec[i] = min(kd_vec[i] + 0.8, 5.0)
```

⚠ 这是**纯 PC 侧**的加法 ⇒ **不增加固件改动面**。

### 3.4 豁免范围（**必须逐条明确**）

仅当 `s_jf_active == true` 时豁免，**且只豁免 server 也豁免的那两条**：

| 判据 | 处置 | 依据 |
| --- | --- | --- |
| `OVERSPEED`（`dq` 绝对值 > `vel_max×1.5`） | **豁免** | server：`measured_overspeed_factor=float('inf')` |
| `POSITION_VIOLATION`（越界 >0.05） | **豁免** | server：`skip_position=True` |
| `TEMP_WARNING` | **保留** | server 也不关；且电机保护不可让 |
| 电机 `err` 码判读 | **保留** | `bad = hw.faulted()` 在 server 里仍然生效 |
| 总线级 `FB_STALE`（全轴 50 ms 无帧） | **保留** | 那是**真掉线**，与单轴锁存级联是两件事 |

⚠ **位置豁免不等于位置不管**：PC 侧仍有 `clamp_to_limits`（已内缩 0.14）+ 固件侧
`law_wall` 的排斥力矩。**速度豁免不等于速度不管**：固件 `slew_linear` 用 `vel_max` 限速。

⇒ 豁免的是「**发现越界就锁存掉力**」这个动作，不是「越界」本身。

### 3.5 谁负责什么

| 层 | 职责 |
| --- | --- |
| PC (`liteteleop`) | 解码主臂帧 → 软限位钳位 → **发目标 + K/B**（不算法向量的、不限速） |
| 固件 | **限速（`vel_max`）→ 重力 `G(q)` → 限位墙 → 弹簧-阻尼** |
| 电机 | MIT 环执行 |

**PC 侧因此变薄**：`servo.follow` 不再调 `get_gravity`（省掉一次往返）。

⚠ **但 `slew_target` 保留了**（与本文档原稿不同）：固件的 `slew_linear` 用 `vel_max`
兜底，而 PC 侧的 `speed_limit`/`accel_limit` 更保守 —— 两层不冲突（同值时自然退化为一层），
且 PC 侧这一层交给固件的是**平滑目标**而不是阶跃。由 `test_safety.py` 与 pylitearm
原版逐拍对拍锁定。

## 4. 接口与协议同步面

⚠ 本仓记忆里有明确教训：**跨仓线契约有多份拷贝**，必须机械核对。

| 文件 | 内容 |
| --- | --- |
| `litearm-stm32/User/litearm/hal/usb_cmd.h` | `CMD_JOINT_FOLLOW 0x08` |
| `litearm-stm32/User/litearm/control/control_loop.c` | `case CMD_JOINT_FOLLOW` → 置标志 + 存目标/K/B |
| `litearm-stm32/User/litearm/safety/safety_check.c` | 两条判据加 `&& !s_jf_active` |
| `litearm-python/src/litearm/_protocol.py` | `CMD_JOINT_FOLLOW` 常量 + 打包 |
| `litearm-python/src/litearm/arm.py` | `joint_follow(q, kp, kd)` 方法 |
| 本仓 `liteteleop/servo.py` | `follow()` 改为发目标 + K/B |

## 5. 验收判据（真机，缺一不可）

1. **不锁存**：大幅度快速拖动 ≥ 3 分钟，`joint_fault` 始终 `0x0`、七轴 `err=1`
2. **不产生 `OVERSPEED` 锁存**：即使 `flags` 里出现过 `OVERSPEED`（首拍预警），也**不得累积到锁存**
3. **温保仍在**：人为验证温度判据未被误伤（可离线断言 `s_temp` 路径不受 `s_jf_active` 影响）
4. **通用路径未被削弱**：🔴 **关键回归** —— 用 `move_mit_all`（**不带** `CMD_JOINT_FOLLOW`）
   直驱时，超速/位置判据**必须仍然生效**（造一个越界的 `move_mit_all`，断言它**照样锁存**）
5. **退出干净**：joint_follow 期间发 `move_j`，`s_jf_active` 被清（豁免立即失效）
6. **收尾**：停止后 `movej` 受控接管，末态使能持位、`flags` 干净

⚠ **第 4 条是安全底线**：它保证这次改动**只新增一条专用通道**，没有把通用路径变弱。

## 6. 风险与代价

| 风险 | 说明 | 处置 |
| --- | --- | --- |
| **拆掉两层保护** | 从臂跟随期间位置/速度只剩 PC 钳位 + 墙 | 已限定在 `s_jf_active` 期间，且退出即恢复 |
| **改固件需重烧** | 编译 + 烧录（`pyocd` + ATK CMSIS-DAP） | ⚠ **烧录由用户执行**，我不擅自动烧录工具 |
| **跨三仓协调** | 任一仓漏改 ⇒ 静默失败或帧被拒 | 按 §4 清单机械核对；协议同步测试须覆盖 `0x08` |
| **闪回** | 固件改坏了需要回退 | 保留当前固件的烧录镜像；「先证明能救再弄坏」 |

## 7. 分阶段实施（每阶段单独可验）

| 阶段 | 内容 | 验证 |
| --- | --- | --- |
| **S1** | 固件：加 `CMD_JOINT_FOLLOW` + `s_jf_active`，**先不豁免任何判据** | 真机：能进能出，行为与 `move_mit_all` 一致（**应当照样锁存**） |
| **S2** | 固件：加两条豁免 | 真机：不再锁存（判据 1、2） |
| **S3** | 固件：接入 `slew_linear` + `dyn_gravity` + `law_wall` 的常驻环 | 真机：跟踪误差、手感（判据 1–3） |
| **S4** | SDK + 本仓：接口与 `servo.follow` 改接 | 端到端（判据 1–6，含第 4 条回归） |

⚠ **S1 单独成阶段是刻意的**：先证明"新通道能通、且**行为与旧路一致**（照样锁存）"，
再加豁免。否则一旦 S2 之后不锁存，**分不清是豁免生效了还是新通道根本没在控制电机**。

### 7.1 落地状态（2026-09-28）

| 阶段 | 代码 | 离线验证（宿主台 `tools/ctl_loop_check_host.c`）|
| --- | --- | --- |
| **S1** | ✅ 已提交 | ✅ `t25` A1/A2（进得来；模式落 `MOVE_MIT_ALL`，未新增模式号）|
| **S2** | ✅ 已提交 | ✅ `t25` D1（PC 在线时不锁存）＋ **D3 回归**（通用路径照样锁存）|
| **S3** | ✅ **被 S1 覆盖** | ✅ `t25` C1/C2（前馈非零；对照 `move_mit_all` 为 0 ⇒ 确系固件所算）|
| **S4** | ✅ 已提交 | ✅ 本仓 `test_servo.py` + `litearm-python` 契约测试 |

⚠ **S3 与 S1 合并的原因**：`slew_linear`（`control_loop.c:2225`）本就跑在
`MOVE_MIT_ALL` 分支上，而 `dyn_gravity` 与 `law_wall` 随 `s_jf_ff` 分支一并接上
⇒ **没有独立的 S3 改动**。原稿以为要另做一阶段，实际被 S1 的接线覆盖。

⚠ **对抗审查后追加的两条修复**（原稿没有，均已提交）：

- `ctrl_joint_follow_active()` **必须带 `!hold`** —— 否则 PC 失联（看门狗跳闸 /
  `drop_hold`）后豁免**无限期**继续。实测：J1 被推到 `q_max+0.20` 持续 40 拍，
  既不锁存、**连 `POS_VIOL` 告警位都不上报**；同条件走通用 `move_mit_all` 第 5 拍就锁存。
  判据 **`t25` D1b** 锁定它（判别力已反向验证：去掉 `!hold` 则双红）。
- **前馈来源判据收成一处**：原先前馈算在 `s_jf_ff && enabled && !hold` 下、而 MIT_ALL
  分支里选前馈来源却用裸 `s_jf_ff` ⇒ `hold` 期间二者不一致。

⛔ **§5 的六条真机判据一条都未跑**（需烧录固件，由用户执行）。

## 8. 四项定案（用户裁决 2026-09-28：**一律以 litearm-server 的实际做法为准**）

| # | 项 | 定案 | 依据（已逐项开文件核实） |
| --- | --- | --- | --- |
| 1 | 目标帧频率 | **250 Hz**，每拍一发 | `litearm.yaml:55  control_loop_hz: 250` |
| 2 | `dq_ref` | **要带**（速度前馈） | `joint_follow.py:313  hw.send_mit(K, kd_vec, q_cmd, **dq_cmd**, tau_ff)` |
| 3 | `engage` | **固件内做，`engage_kp=15.0 / engage_kd=0.8`**（**不是** `zg_engage_*`） | `joint_follow.py:212  def engage(hw, engage_sec=0.3, engage_kp=15.0, engage_kd=0.8)` |
| 4 | `wall_fw_kd` | **启用，0.8**，**在 PC 侧叠加**（不增固件改动面） | `litearm.yaml:266  firmware_kd_extra: 0.8` + `joint_follow.py:306-312` |

⚠ 第 4 项**我先前的判断是错的**（曾据一次只取了 12 行的 `diff` 断言"`litearm.yaml` 没有
`firmware_kd_extra`"）—— 该行在 266 行、落在窗口外。**教训重申：跨仓取"真值"必须按内容
grep 到那一行本身，`diff` 的窗口会撒谎。**
