# 夹爪遥操设计（litearm-teleop-isomorphic）

- 日期：2026-09-28
- 状态：待评审
- 分支：`feat/gripper-teleop`
- 参考实现：`/home/llx/litearm-device/src/litearm_device/gripper_teleop.py`（已真机跑通）
- 夹爪 SDK：`/home/llx/litegrip-python`（包名 `litegrip`，SocketCAN + 达妙 MIT 协议）

## §1 目标

给同构遥操上位机 `liteteleop` 加上**夹爪遥操**：主端夹爪进零重力被人手掰动，
从端夹爪跟着开合。链路与机械臂同构 —— zenoh **纯点对点**，从端主动连主端的 topic。

## §2 铁律：与机械臂遥操**代码上完全解耦**

**用户裁决（2026-09-28）**：夹爪遥操和机械臂遥操在代码上完全分开，
**只是界面上把它们放在一起显示**。

推论（本设计的每一处都从这条推出）：

1. `GripWorker` 是**独立线程、独立对象、独立生命周期**，与 `ArmWorker` 零共享状态。
   - 两者不共享 zenoh session、不共享锁、不共享 CAN、不共享任何可变对象。
   - 臂遥操在跑时夹爪可以不起；夹爪在跑时臂可以不起；两个都不跑也行。
2. **因此端口方案只能是「独立 session + 独立端口」**（见 §5.2）。
   共享 session 需要把 session 所有权提到两个 worker 之上，那本身就是耦合，
   且必须改动 `arm_worker.py` 里真机跑通的代码。
3. 夹爪有**自己的一套配置**（`gcan` / `gpeer` / `gport` / `grip_id`），不读臂的字段。
4. 故障隔离是这条铁律的**可观测结果**：夹爪链路死掉不得扰动臂链路，反之亦然。

**验收这条铁律的判据**：`ArmWorker` 与 `GripWorker` 之间不存在任何直接引用，
除了 GUI 层的展示代码。见 §11.2.3。

## §3 范围

**做**：

- 主端：夹爪零重力 + 定频发布开合度
- 从端：订阅 + 首帧对齐 + 高频位置跟随 + watchdog 持位
- 未标定/零行程拒绝启动、从端钳位、收尾持位（**不失能**）
- GUI 夹爪面板（独立开关、独立状态显示）
- 线协议与 `litearm-device` 逐字节对齐（跨仓可互通）

**不做**（YAGNI）：

- 夹爪力控遥操 / 触觉回传
- 跨仓真机联调（只做对拍，见 §11.2）
- 多夹爪（一台机器一个夹爪）
- 夹爪录制 / 回放
- CAN 通道自动枚举（见 §9.3）
- 改 `link.py` / `arm_worker.py` / `servo.py` / `wire.py` / `ports.py` —— **这几个文件一行不动**

## §4 架构

```text
        主端机                                    从端机
┌──────────────────────┐                 ┌──────────────────────┐
│  ArmWorker (既有)    │                 │  ArmWorker (既有)    │
│   :17447 litearm/v4/ │◄─── TCP ────────│   /{arm_id}/teleop   │
│   {arm_id}/teleop    │                 │                      │
└──────────────────────┘                 └──────────────────────┘
┌──────────────────────┐                 ┌──────────────────────┐
│  GripWorker (新增)   │                 │  GripWorker (新增)   │
│   :17448 litearm/v4/ │◄─── TCP ────────│   /{grip_id}/        │
│   {grip_id}/         │                 │    gripper_teleop    │
│    gripper_teleop    │                 │                      │
└──────────────────────┘                 └──────────────────────┘
    两条链路、两个 session、两个端口、互不可见
```

两条链路**完全平行**：各自的 zenoh session 关掉全部广播发现
（`scouting/multicast/enabled=false`、`scouting/gossip.enabled=false`、`mode=peer`），
主端 `listen`、从端 `connect` —— 与既有 `link._base_config()`（`liteteleop/link.py:29-35`）
逐字相同。

### §4.1 为什么不复用臂的 session

`link.Listener` / `link.Connector` 是**单 key 单 port** 的
（`liteteleop/link.py:63-93`、`:96-124`），而臂的 session 是在
`_run_master` / `_run_slave` **内部**创建的（`liteteleop/arm_worker.py:406`、`:434`），
遥操一停 session 就没了。要让一个 session 同时承载两个 topic，必须：

1. 把 `Listener`/`Connector` 扩成多 key；
2. 把 session 的所有权提到两个 worker 之上；
3. 因此必须改 `arm_worker.py` 里已经真机跑通的那段。

这既违反 §2 铁律，又是对已验证代码的无谓风险。**故各自开 session。**

## §5 线契约

新增 `liteteleop/grip_wire.py`，与
`litearm-device/src/litearm_device/gripper_teleop.py:38-75` 的帧编解码**逐字对齐**。

### §5.1 topic

```python
def gripper_teleop_topic(grip_id: str) -> str:
    return f"litearm/v4/{grip_id}/gripper_teleop"
```

主端用自己的 `grip_id` 发布，从端用主端的 `grip_id` 订阅。
与参考实现 `gripper_teleop.py:73-75` 同构（那边参数名是 `device_id`）。

### §5.2 帧

```python
_FRAME = struct.Struct(">4d")   # 32 字节，大端
# [0] openness      归一化开合度 [0,1]，0=闭合 1=全开  ← 主传输量
# [1] position_mm   开合宽度 mm（单侧），仅诊断/显示
# [2] force_n       夹持力 N，仅诊断
# [3] timestamp     发帧本地时间（秒），从端 watchdog 判活
```

**为什么传 `openness` 而不是弧度** —— 参考实现第 44-49 行记着真机结论：

> 每台夹爪的电机零点/方向/标定各不相同（如 A 张开=-1.42rad, B 张开=+1.14rad），
> 直接传弧度无法跨夹爪通用。openness 由各自标定的 position_mm/travel_mm 归一化得到，
> 方向统一、量纲无关，slave 再按自己标定映射回弧度。

⚠ **照抄陷阱（已核实）**：`gripper_teleop.py` 的**模块 docstring（第 14-18 行）与代码不一致** ——
docstring 仍写着 `[0] position_rad, [1] velocity_rad_s`，而第 51 行的真代码是
`struct.Struct(">4d")  # openness[0..1], position_mm, force_n, timestamp`。
代码是新增的（第 44-49 行的注释解释了为何改），docstring 是陈旧的。
**以代码为准**，并在我们的模块注释里记下这条漂移。

⚠ `timestamp` 是**发帧方的本地时间**，两端不同源（与 `wire.py:17` 的臂帧同理）
⇒ 只能用于「同一主端相邻帧间隔」，**不能**算端到端延迟；从端的 watchdog 用
**本地**收帧时刻判活（见 §7.2）。

## §6 换算

### §6.1 公式（**修掉参考实现的一个缺陷**）

litegrip SDK 的换算是**带 `close_sign`** 的：

- `get_state`：`position_mm = (pos_closed_rad - position_rad) * close_sign * rad_to_mm`
  （`litegrip/src/litegrip/gripper.py:1561-1562`）
- `goto`：`position_rad = pos_closed_rad - close_sign * position_mm / rad_to_mm`
  （`litegrip/src/litegrip/gripper.py:1183-1185`）

而参考实现的 `_openness_to_rad`（`gripper_teleop.py:155-163`）**漏掉了 `close_sign`**：

```python
# 参考实现（litearm-device），只在 close_sign == +1 时正确
return self._pos_closed_rad - position_mm / self._rad_to_mm
```

⇒ **反装夹爪会镜像**。本仓按 SDK 公式补上：

```python
travel_mm = abs(cfg.pos_open_rad - cfg.pos_closed_rad) * cfg.rad_to_mm

def mm_to_openness(mm: float) -> float:
    return _clamp01(mm / travel_mm)

def openness_to_rad(o: float) -> float:
    o = _clamp01(o)
    return cfg.pos_closed_rad - cfg.close_sign * (o * travel_mm) / cfg.rad_to_mm
```

`close_sign` 直接用 SDK 的公开属性 `GripperConfig.close_sign`
（`litegrip/src/litegrip/models.py:109-118`，定义 `+1.0 if pos_closed_rad >= pos_open_rad else -1.0`），
**不碰私有字段**。`cfg` 取 `LiteGrip.config` 公开属性（`gripper.py:255`）。

> SDK 另有 `CalibrationData.travel_mm` 属性（`models.py:163`，`travel_range * rad_to_mm`）。
> 我们仍从 `cfg` 的两个限位自己算 —— 因为跑起来之后手头只有 `cfg`，
> 且这样 `travel_mm` 与 `close_sign` 出自**同一组数**，不会分叉。
> 两者在正常标定下相等（见 §6.2 的验算）。

### §6.2 两种装法的验算（用**真标定值**，非占位默认值）

取自 SDK 随包模板 `/home/llx/litegrip-python/src/litegrip/calibration_normal.json`
与 `calibration_reverse.json`：`rad_to_mm = 74.8`，`travel_range_rad = 1.605`
（限位映射见 `gripper.py:997-998`：`pos_closed_rad ← zero_position_rad`、
`pos_open_rad ← max_position_rad`）
⇒ `travel_mm = 1.605 × 74.8 = 120.054 mm`（与 `max_stroke_mm = 120.0` 吻合，`models.py:100`）。

| 装法 | `pos_closed_rad` | `pos_open_rad` | `close_sign` | `openness=0` → | `openness=1` → |
|---|---|---|---|---|---|
| 正装 | 0.114 | −1.491 | +1 | `0.114` = pos_closed ✓ | `0.114 − 1.605 = −1.491` = pos_open ✓ |
| 反装 | −1.491 | 0.114 | −1 | `−1.491` = pos_closed ✓ | `−1.491 + 1.605 = 0.114` = pos_open ✓ |

漏掉 `close_sign` 时，反装的 `openness=1` 会算出 `−1.491 − 1.605 = −3.096`
—— **镜像且远出标定限位**。

⚠ **故意不用** `pos_closed_rad=1.14 / pos_open_rad=0.0` 举例：那正是 §8.1 禁止当作真实限位的
**占位默认值**（`models.py:90-91`），拿它当例子会让读者以为那组数是可用的。

> 注：参考实现的**读侧是对的**（`position_mm` 由 SDK 算、已含 `close_sign`），
> **只有反算回弧度那一侧漏了**。

### §6.3 读侧（主端）

`openness = clamp01(st.position_mm / travel_mm)`，其中 `st` 是 `LiteGrip.get_state(wait=False)`
（`gripper.py:1530-1576`）。`position_mm` 已含 `close_sign`，`travel_mm` 恒正
⇒ 正装/反装都得到 `0 … travel_mm` → `0 … 1`。

⚠ **不用 `GripperState` 上不存在的 `travel_mm`**：参考实现读的是它自己 adapter 造的 dict
（`gripper_teleop.py:391-398`），而 litegrip 的 `GripperState` **没有这个字段**
（`models.py:29-46`）。我们按 §6.1 的公式自己算。

## §7 两端循环（50 Hz）

两端各自**一个后台线程**（照搬参考实现 `gripper_teleop.py:5-7` 的形态）。
`rate_hz` 默认 50（参考实现 `:106`）。

### §7.1 主端

```text
LiteGrip(channel=gcan, disable_on_disconnect=False)   # ⚠ 见 §8 rule 6
connect() → load_calibration()
断言 §8.1 的两条前置（标定 + 非零行程）—— ⚠ 必须在 enable() 之前
enable()
Listener(gport, gkey)
每拍：
    lg.send_mit_frame(q=0.0, kp=0.0, kd=0.0)   # 自己维持零重力
    lg.poll(timeout_s=0.0)
    st = lg.get_state(wait=False)
    openness = mm_to_openness(st.position_mm)
    pub.put(encode_gripper_teleop(openness, st.position_mm, st.force_n, time.monotonic()))
收尾：lg.exit_zero_gravity() → ep.close() → lg.disconnect()
```

⚠ **断言放在 `enable()` 之前**：否则一台未标定的夹爪会先被使能、再被拒 —— 白上一次电。

⚠ **不调 `enter_zero_gravity()`**：它只发**一帧** bootstrap，
docstring 明说「caller must call `update_state` or poll manually to sustain the mode」
（`gripper.py:496-518`）。我们自己每拍发零力矩帧才是持续维持
（参考实现 `:220-224` 同）。

⚠ 零力矩帧的 `q` 传 `0.0`：`kp=kd=0` 时 `q` 不参与力矩计算（参考实现 `:221-222` 同）。

### §7.2 从端

```text
LiteGrip(channel=gcan, disable_on_disconnect=False)   # ⚠ 见 §8 rule 6
connect() → load_calibration()
断言 §8.1 的两条前置（标定 + 非零行程）—— ⚠ 必须在 enable() 之前
enable()
slot = link.LatestSlot()
sub  = Connector(gpeer, gport, gkey, on_frame=_on_wire)

def _on_wire(payload: bytes) -> None:        # ⚠ 必须是**单参**包装
    slot.put(payload, time.monotonic())      # 在 zenoh 线程上 ⇒ 只写槽

# ⚠⚠ 循环入口必须先把 q 定义成**本机实测位置**。
#    首帧到达前 q 若为 0.0，那就是一个**真实位置指令** —— 按装法不同可能直冲机械限位
#    （正装 0.0 = 全开，反装 0.0 = 全闭）。
q     = clamp(lg.get_state(wait=False).position_rad, 从端自己的标定区间)
stale = False

对齐（默认开，见 §9.3）：等首帧（≤5 s）→ goto_rad(q0, kp, kd, duration=1.0)
    ⚠ 关掉对齐时**跳过这一步**，q 仍是上面那个实测位置，直接进跟随环
    ⚠ 5 s 内没等到首帧 → 不 goto，直接进跟随环（q 停在实测位置）

每拍：
    payload, _ts = slot.take()
    if payload 且 len == 32:
        openness = clamp01(decode(payload)[0]); q = openness_to_rad(openness)
        stale = False
    elif slot.ever_received 且 slot.peek_age(now) > 0.2:
        stale = True            # 持位：q 不变
    # ⚠ 「从未收到首帧」时既不 stale 也不跟随：q 保持入口那个实测位置，
    #   每拍照样发帧 ⇒ 电流连续、不掉力、不自动重连（与 §8 rule 3 同结局）
    q = clamp(q, 从端自己的标定区间)          # §8.2
    lg.send_mit_frame(q=q, kp=kp, kd=kd, dq=0.0)   # ⚠ stale 时也继续发，防掉力
    lg.poll(timeout_s=0.0)
收尾：再发一帧持位帧（q 取当前实测位置）→ ep.close() → lg.disconnect()
```

⚠ **`on_frame` 必须是单参包装**：`LatestSlot.put(self, payload, now)` 收两个参数
（`link.py:146`），而 `Connector._handler` 只用一个参数回调（`cb(bytes(sample.payload))`，
`link.py:118`）⇒ 直接写 `on_frame=slot.put` 会 **`TypeError`**。
本仓既有写法就是这个包装（`arm_worker.py:506-508`）。

- `dq` 恒 `0.0`：单自由度夹爪不用速度前馈（参考实现 `:289` 的理由 —— 传的是 openness，
  两端速度量纲不通用）。
- 复用既有的 `link.LatestSlot`（`liteteleop/link.py:127-183`），
  它已解决「从未收到」与「刚收到」的区分（`peek_age()` 对「从未收到」返回 `None`），
  正是 watchdog 需要的语义。
- watchdog = **200 ms**，与臂的 `WATCHDOG_MS`（`arm_worker.py:61`）同值。
- **对齐默认开**，界面上给一个复选框（默认勾选）可关。见 §9.3。

### §7.3 主从标定一致性诊断（参考实现没有的一条）

从端同时拿到 `openness` 与 `position_mm`，于是
`position_mm / openness ≈ 主端的 travel_mm`。从端把自己的 `travel_mm` 与它比，
差得超过阈值就**告警**「主从夹爪型号/标定不一致，运动会被按比例缩放」。

挡的是「主从装的是不同型号夹爪」——不报错、只是动作幅度悄悄不对，属于最难查的一类现场。

⚠ **两端都要设保护**：`openness` 接近 0 时比值无意义（除零）；
接近 1 时 `openness` 已被 `clamp01` 饱和，比值会报出主端的**原始** `position_mm`
（大于其真实 `travel_mm`）⇒ 会误报。故只在 `0.15 ≤ openness ≤ 0.85` 的窗口内采样，
且**只告警一次**。

### §7.4 与 litearm-device 的逐项对照（「逻辑一致」的可审版本）

**用户裁决（2026-09-28）：逻辑与 litearm-device 的遥操逻辑一致即可。**

对拍对象：`gripper_teleop.py` 的 `_master_loop`（:213-237）与 `_slave_loop`（:241-297）。

| 项 | litearm-device | 本设计 | |
|---|---|---|---|
| 主端维持零重力 | 每拍 `send_mit_frame(q=0,kp=0,kd=0)` + `poll` + `get_state(wait=False)`（:220-225） | 同 | 等价 |
| 主端发帧 | `encode(openness, position_mm, force_n, time.time())`（:230-231） | 同（本地时基用 `time.monotonic()`，理由见 §5.2 的 ⚠） | 等价 |
| 从端订阅 | `sub.drain_latest()`（:273） | `Connector` 回调 + `LatestSlot.take()`（`link.py:116-118`） | 等价 —— 都是 latest-wins，丢弃积压 |
| 帧长校验 | `len(m) == FRAME_SIZE` 才处理，否则忽略（:274） | 同 | 等价 |
| 对齐 | 等首帧 ≤5 s → `goto_rad(q, kp, kd, duration=1.0)`（:254-268） | 同（默认开，§9.3 给开关） | 等价 |
| 无首帧 | 告警后继续进跟随环（:266-268） | 同 | 等价 |
| watchdog | `_last_frame_ts > 0` 且超 200 ms ⇒ `stale`（:282-288） | `slot.ever_received` 且 `peek_age > 0.2` ⇒ `stale` | 等价（后者是 `LatestSlot` 为「从未收到」专门造的语义，`link.py:163-183`） |
| stale 行为 | 继续发帧持位、不重连、告警**只一次**（:284-288, 291-292） | 同 | 等价 |
| 速度前馈 | `dq_cmd = 0.0`（:289） | `dq=0.0` | 等价 |
| 收尾 | 循环退出即止（持位由退出前最后一帧保证，:188） | 再显式发一帧持位帧 | 等价（更明确） |

#### 有意偏离（共 4 处，逐条给理由）

1. ⚠ **`close_sign`（换算）** —— 参考实现 `_openness_to_rad`（:155-163）漏了它，反装夹爪会镜像（§6.1）。
   本设计按 SDK 公式补上。**只在反装夹爪上有行为差异**；正装时两者逐位相同。
   线格式不变（传的仍是 `openness`）⇒ 不影响 §11.2.1 的互通。
   **→ 需用户确认保留。**
2. ⚠ **从端每拍 `poll(0.0)`** —— 参考实现的从端循环**不读** CAN；它能拿到夹爪自身状态
   （过温/过流/失能）是因为 **daemon 那 10 Hz 状态广播在替它读**
   （该文件 :20-25 的「CAN 并发约定」就是写这件事的）。
   本仓是**独立进程、没有 daemon** ⇒ 不 poll 就**永远看不到夹爪自己的故障**
   （`send_mit_frame` 只在未使能时返回 `False`，不报故障码）。
   `poll(0.0)` 是非阻塞的，不拖节拍。**→ 需用户确认保留。**
3. **从端每拍显式钳位**（§8.2）—— 参考实现不在环里钳位，靠 `openness∈[0,1]` 隐含。
   标定自洽时**输出逐位相同**（§6.1 的公式把 `o∈[0,1]` 映到 `[pos_closed, pos_open]` 内），
   只是主从标定不一致时多一道保险。**无行为差异，属冗余保护。**
4. **`q` 初值取 `position_rad`** —— 参考实现走 `mm → openness → rad` 的往返
   （`_read_own_openness` :311-316 → `_openness_to_rad`）。二者**代数恒等**
   （正装验算：`pos_rad=1.0 → mm=0.14r → o=0.1228 → q=1.14−0.14=1.0`），
   本设计直接取 `position_rad` 少一次换算。**无行为差异。**

除上述 4 条外，**其余逐项一致**。§11.2 的对拍就是按这张表逐行核。

## §8 安全约束

| # | 约束 | 依据 |
|---|---|---|
| 1 | **未标定 / 零行程拒绝启动**，且在 `enable()` **之前**判 | `send_mit_frame` 与 `goto_rad` **都不检查 `calibrated`** —— `_check_calibrated` 只保护 `open`/`close`/`grasp`（`litegrip/src/litegrip/actions.py:200-215`）。不拦就会拿占位默认值 `pos_closed_rad=1.14`/`pos_open_rad=0.0`（`models.py:90-91`）当真实限位用。零行程另判：`cfg.calibrated` 为真**不蕴含**非零行程（`load_calibration` 直接取 `data.get("calibrated", True)`，`gripper.py:1015-1016`），而 `travel_mm == 0` 会让 §6.1 的 `mm_to_openness` 抛 `ZeroDivisionError`。⚠ 零行程判据必须写在 **rad 空间**：`abs(pos_closed_rad - pos_open_rad) <= 1e-6`，与 SDK 的 `_check_calibrated`（`actions.py:212-215`）**逐字同款** —— 写成 mm 空间的 `travel_mm == 0` 是**更弱**的条件，`abs(Δrad) ∈ (0, 1e-6]` 时会放过 SDK 会拒的配置。⚠ 再加一条 SDK 自己也没有的：`rad_to_mm == 0` 也拒（§6.1 的 `openness_to_rad` 拿它做除数；SDK 的 `goto` 同样有这个缺口） |
| 2 | 从端每帧把 `q` 钳进**自己**的标定区间 `[min(pos_closed,pos_open), max(...)]` | 主从标定不一致时不能撞穿从端机械限位。与 `goto_rad` 自己的钳位同源（`gripper.py:1208-1210`）。`openness∈[0,1]` 在标定一致时已隐含此界，但**不一致时不能靠它** |
| 3 | watchdog 超时 → **持位**，不自动重连 | 用户裁决（2026-09-28）。与臂的 `should_stop()`（`arm_worker.py:487-497`）同语义 |
| 4 | 从端**绝不 `disable()`** —— 遥操期间与收尾都不 | 掉力会当场松掉正夹着的物体，对抓取任务是比「继续夹着」更糟的结果。与臂的「绝不 disable」（`arm_worker.py:366`，`servo.hold_at_current` 而**非** `disable`）同规 |
| 5 | 收尾顺序：主端 `exit_zero_gravity()` / 从端发一帧持位 → **`ep.close()`** → `lg.disconnect()` | `ep.close()` 缺了**进程永久挂死**，本仓已实测（`liteteleop/link.py:6-7`） |
| 6 | ⚠⚠ **构造时必须显式传 `disable_on_disconnect=False`** —— 否则 rule 4 与 rule 5 自相矛盾 | `LiteGrip.__init__` 的默认值是 **`True`**（`gripper.py:205`）⇒ `disconnect()` 会走 `self._can.disconnect(disable=True)`（`gripper.py:338-339`）⇒ `can_bus.disconnect` 里的 `self.disable()`（`src/litegrip/protocols/can_bus.py:92`）⇒ **掉力、松开**。这会让收尾那帧持位帧白做，并产生 rule 4 明令禁止的结果。**两端都要传** |
| 7 | ⛔ **绝不调 `lg.close()`** —— 那是**合爪**不是断开（`gripper.py:1103`） | 断开只有 `disconnect()`（`gripper.py:329`）。这是 SDK 的真实命名陷阱 |
| 8 | 主端零重力期间不调任何 `open`/`close`/`grasp` | 那些会走自己的 MIT 斜坡流，与我们的零力矩帧抢 CAN |

## §9 配置与界面

### §9.1 新增设置（`liteteleop/settings.py`）

```python
gcan:    str = ""          # 夹爪 CAN 通道，空 = 不启用夹爪遥操（默认不启用）
gpeer:   str = "127.0.0.1" # 从端填：主端 IP（独立字段，不读臂的 peer）
gport:   int = 17448       # 夹爪 zenoh 端口（独立 session ⇒ 必须独立端口）
grip_id: str = "gripA"     # 决定夹爪 topic
```

⚠ `gcan` 默认空 ⇒ **默认不启用**，与臂遥操的默认关闭一致（安全默认）。
⚠ **`gcan` 只是通道名，不是开关**。启用与否由界面上那个**独立的「夹爪遥操」开关**决定（§9.3）。
界面**不得**在显示/初始化时把 `can0` 写进 `gcan` —— 那会把「默认不启用」
**静默**变成「默认启用」，丢掉这条安全默认。
⚠ 四个字段**都不读臂的同名字段**（§2 铁律第 3 条）。

### §9.2 新增命令行（`liteteleop/__main__.py`）

`--gcan` / `--gpeer` / `--gport` / `--grip-id`，语义与既有 `--cdc`/`--peer`/`--jport`/`--arm-id` 对称。

### §9.3 界面（`gui/pages.py` + `gui/main_window.py` + `gui/bridge.py`）

在既有的 `TeleopPage` 里加一个**夹爪分区**（这是 §2 说的「界面上放在一起显示」）：

- **CAN 通道输入框**（纯文本框，`can0` 只作**占位提示**，**未启用时不写回设置**）。
  ⚠ **不做扫描按钮**：`ports.py` 只枚举 **CDC/串口**并按 STM32 的 VID:PID `1d50:606f` 过滤
  （`ports.py:5`、`:60`），本仓**没有**任何枚举 SocketCAN 的代码，
  而 `litegrip` 的 `channel` 就是个字符串（`gripper.py:198`）⇒ 加枚举＝新增一条我们没验过的代码路径。
  折中：在输入框旁**只读显示** `/sys/class/net/` 下 `can*` 的名字作为提示（约 3 行，零依赖）。
- 「夹爪遥操」独立开关（**与臂的遥操开关是两个独立控件**）
- 「启动时对齐」复选框（默认勾选，见 §7.2）
- 实时读数：`openness` / `position_mm` / `force_n` / 环频 / 帧数 / 匹配状态 / 帧龄 / `stale`
- 告警行（§7.3 的标定不一致告警落在这里）

`WorkerBridge` 加一个 `grip_state = pyqtSignal(object)`（2 行，纯增量）。
GUI 只做两件事：**投命令**、**读快照** —— 与 `ArmWorker` 同一纪律。

### §9.4 ⚠⚠ SDK 必须**钉死并断言**，否则会静默走错仓

**用户裁决（2026-09-28）：SDK 采用 `litegrip-python`。**
正因为下面这份环境现实，「采用哪份」不能只靠约定 —— 必须靠断言。

**本机同时存在两份包名都叫 `litegrip` 的仓库**（已实测）：

| 路径 | 远端 | 布局 | 状态 |
|---|---|---|---|
| `/home/llx/litegrip-python` | github `nexform-tech/litegrip-python` | `src/litegrip/` | **本设计的目标**，未安装 |
| `/home/llx/moduangongju/lite-grip` | gitee `yudao_hz_1/lite-grip` | `litegrip/`（平铺） | 被 **editable 安装**为 `litegrip` 2.2.0 |

⇒ 裸 `import litegrip` 在本机**落到 gitee 那份**（实测
`/home/llx/moduangongju/lite-grip/litegrip/__init__.py`，v2.2.0）。
二者的 API 面并不相同（`close_sign` / `calibrated` / `load_template` 等是本设计赖以成立的东西），
走错仓的后果从 `AttributeError` 到**静默的语义漂移**都有可能。

这**正是** `litearm` 踩过的同一类坑 —— 见 `liteteleop/__main__.py:1-21` 的
`_pin_sdk()` 及其注释：**「判据不是『我设了 PYTHONPATH』，而是导入后断言 + 打印」**。
故照同一纪律办：

```python
GRIP_SDK_SRC = "/home/llx/litegrip-python/src"   # 与 SDK_SRC 同款硬编码约定

def _pin_grip_sdk() -> None:
    if GRIP_SDK_SRC not in sys.path:
        sys.path.insert(0, GRIP_SDK_SRC)
    import litegrip
    if not str(litegrip.__file__).startswith(GRIP_SDK_SRC):
        raise SystemExit(
            f"⛔ litegrip 导入自 {litegrip.__file__}，不是 {GRIP_SDK_SRC} —— 环境里有另一份抢先了")
    print(f"夹爪 SDK: {litegrip.__file__}")
```

⚠ **断言必须有，不能只 insert。** 已实测：该 editable 安装是用
`sys.meta_path.append(_EditableFinder)` 注册的（`install()` 的实现），
`_EditableFinder` 排在 `PathFinder` **之后**，所以 `sys.path.insert(0, ...)` 目前**压得住**
（实测 pin 后落在 `/home/llx/litegrip-python/src/litegrip/__init__.py`）。
但这是**实现细节**：一旦它改成 `sys.meta_path.insert` 式注册，
`sys.path` 插入就会**静默失效** —— 只有那条断言能把它抓住。

⚠ 只在 `gcan` 非空（即启用夹爪遥操）时才 import `litegrip`；
不启用夹爪的机器上不该因为缺这个包而起不来。

## §10 文件清单

### 新增

| 文件 | 职责 |
|---|---|
| `liteteleop/grip_wire.py` | 帧编解码 + topic（与参考实现逐字对齐） |
| `liteteleop/grip_worker.py` | `GripWorker` 线程 + `GripSnapshot` |
| `tests/fake_grip.py` | **本仓自带**的极简夹爪替身（理由见 §11.1 开头的 ⚠） |
| `tests/test_grip_wire.py` | 帧长/往返/topic/大端 |
| `tests/test_grip_worker.py` | `fake_grip` + 真 zenoh 回环 |

### 修改

| 文件 | 改动 |
|---|---|
| `liteteleop/settings.py` | +4 字段 |
| `liteteleop/__main__.py` | +4 参数 + §9.4 的 `_pin_grip_sdk()` |
| `liteteleop/gui/pages.py` | 夹爪分区 |
| `liteteleop/gui/main_window.py` | 夹爪开关接线 |
| `liteteleop/gui/bridge.py` | +1 信号 |
| `README.md` | 更新状态与用法 |

**一行不动**：`link.py`、`arm_worker.py`、`servo.py`、`safety.py`、`wall.py`、`wire.py`、`ports.py`。

## §11 验收（三道闸门）

### §11.1 编译 / 单测

`python3 -m pytest tests -q`（CI 命令，`.github/workflows/ci.yml:49`）。

⚠ **测试替身必须放在本仓**：CI 只装 `pytest` 与 `zenoh>=1.6`（`ci.yml`），**不装 litegrip**；
且 `litegrip-python/tests/fake_can.py:14` 有 `import _sdkpath`（要它自己的目录在
`sys.path` 上），而本仓 `tests/` 是包（有 `tests/__init__.py`）而 litegrip 的 `tests/` 不是
⇒ **直接 import 那份 `fake_can.py` 在 CI 上起不来**（本机已实测：按本仓习惯直接调用会报错）。
故 §10 新增本仓自带的 `tests/fake_grip.py`（只需 `send_mit_frame` / `poll` / `get_state` /
`goto_rad` / `disconnect` / `config` 这几样），下面 2~5 用它。
litegrip 那份 `fake_can.py` **只用于 §11.2 的对拍**（本机手工跑）。

必须有的用例（每条都要**有判别力**，即：实现错时它必红）：

1. **正装/反装换算**：参数化跑 §6.2 的两组真标定值 —— 正装
   （`pos_closed=0.114`, `pos_open=-1.491`）与反装（两者对调），
   断言 `openness=0 → pos_closed_rad`、`openness=1 → pos_open_rad`。
   **两组数值直接写死在用例里**（不从 SDK 树加载 fixture ⇒ 否则又依赖外部路径）。
   **漏掉 `close_sign` 时反装那组必红** —— §6.1 那条缺陷的回归网。

2. ⚠⚠ **收尾不失能**（对应 §8 rule 6）—— **这条最容易写成零判别力**，三个坑一个都不能踩：

   - ⛔ **不能断言 `fake.disconnected`**：`FakeLiteGripCAN.disconnect(self, disable=True)`
     的函数体只有 `self.disconnected = True`（`litegrip-python/tests/fake_can.py:203-204`），
     **`disable` 参数被完全忽略** ⇒ 传 `True`、传 `False` 都通过。
   - ⛔ **不能断言 `g.is_enabled`**：`LiteGrip.disconnect()` **无条件**把
     `self._enabled = False`（`gripper.py:343`），即使电机实际仍使能 ⇒ **恒红**。
   - ⛔ **不能直接用 litegrip 的 `make_gripper()`**：它不接 `disable_on_disconnect`
     （`fake_can.py:207-215`），默认构造走 `True` ⇒ 根本走不到要断言的那条路。
   - ✅ **正确写法**：把断言**打在传输层**。照抄 litegrip 自己的先例
     `tests/test_actions.py` 的 `TestDisableOnDisconnect.test_can_keep_enabled`
     （已实测绿）：

     ```python
     seen = {}
     fake.disconnect = lambda disable=True: seen.update(disable=disable)
     g.disconnect()
     assert seen == {"disable": False}   # 传 True 时这里是 {"disable": True} ⇒ 必红
     ```

   自建实例、显式传 `disable_on_disconnect=False`（也可用公开 setter，`gripper.py:288-290`）。
   **不传 `disable_on_disconnect=False` 时 `seen` 是 `{"disable": True}` ⇒ 必红。**

3. **watchdog 持位**：停发帧后从端 `q` 不变，且 `send_mit_frame` 仍在被调用。
4. **未标定拒绝启动**：`cfg.calibrated=False` 时启动即抛，且 `enable()` **未被调用**。
   （零行程、`rad_to_mm == 0` 各来一组，对应 §8.1 的三条判据。）
5. **首帧未到时不发零位**（对应 §7.2 的入口兜底）：5 s 内无帧 ⇒ 发出的 `q` 等于
   入口实测位置，**不是 `0.0`**。**入口不取实测位置（直接初始化成 0.0）时必红。**
6. 帧长 32、解错帧长不崩、topic 串正确。
7. **SDK 钉死断言**（对应 §9.4）：构造一个「`litegrip.__file__` 不在 `GRIP_SDK_SRC` 下」的场景，
   断言 `_pin_grip_sdk()` **抛 `SystemExit`**。**去掉断言只留 `sys.path.insert` 时必红**
   —— 否则压不住 editable MetaPathFinder 就会静默走错仓。

`tests/fake_grip.py` 可以**参照**（不是复制）litegrip 自带的
`litegrip-python/tests/fake_can.py`（`FakeLiteGripCAN` + `make_gripper()`，
纯 stdlib、无硬件）—— 但**必须去掉 `import _sdkpath` 那行**，否则本仓 CI 起不来。
⚠ 且**不要**照搬它的 `disconnect(disable=True)`：那个函数体忽略 `disable`
（正是上面用例 2 的坑），我们的替身必须**记录 `disable` 实参**，
否则用例 2 又变回零判别力。

### §11.2 对拍

1. **线契约对拍**：`diff` 我们的 `grip_wire.py` 与
   `litearm-device/src/litearm_device/gripper_teleop.py:38-75` ——
   格式串 `">4d"`、字段序、`FRAME_SIZE=32`、topic 模板逐项相同。
2. **换算对拍**：`openness_to_rad` 与 `lg.goto()` 的公式在**同输入**下**同输出**
   （对拍对象：`gripper.py:1183-1185`）。二者在 `position_mm = o * travel_mm` 时代数恒等。
3. **解耦对拍**（验 §2 铁律）：`grep` 证明 `arm_worker.py` 不引用 `grip_worker`/`grip_wire`，
   `grip_worker.py` 除 `link`/`grip_wire` 外不引用臂侧模块。
4. **逻辑对拍**（验「与 litearm-device 逻辑一致」，对应 §7.4 的表）：并排核
   `_master_loop`（`gripper_teleop.py:213-237`）/ `_slave_loop`（:241-297）与 §7.1/§7.2，
   确认**§7.4 列的 4 处「有意偏离」之外没有别的差异**。

### §11.3 真机

1. 主端手掰、从端跟随（正装夹爪）
2. **拔网线 / 杀主端** → 从端**持位**：不掉力、不松开、不自动重连
3. 未标定的夹爪 → 启动即拒绝，且**电机没被使能过**
4. 收尾后：**夹爪仍夹着**（验 §8 rule 6，这是最容易做错的一条）+ 进程**正常退出**（验 `ep.close()`）
5. **启动日志里打印的夹爪 SDK 路径是 `/home/llx/litegrip-python/src/litegrip/…`**，
   **不是** `/home/llx/moduangongju/lite-grip/…`（验 §9.4）。
   两台机上都要看这一行 —— 这是唯一能区分「跑的是哪份 SDK」的一手证据。

**已知限制**：与 `litearm-device` 的互通性经**对拍**（§11.2.1）证明，
**不做跨仓真机联调**。

## §12 未决项

1. 从端的 `kp`/`kd` 是否暴露到界面？（默认用 `cfg.kp=100.0` / `cfg.kd=2.0`，`models.py:83-84`）
   —— 建议**先不暴露**，真机跑出手感问题再加。
