# 夹爪遥操设计（litearm-teleop-isomorphic）

- 日期：2026-09-28
- 状态：待评审
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
除了 GUI 层的展示代码。见 §11.2。

## §3 范围

**做**：

- 主端：夹爪零重力 + 定频发布开合度
- 从端：订阅 + 首帧对齐 + 高频位置跟随 + watchdog 持位
- 未标定拒绝启动、从端钳位、收尾持位
- GUI 夹爪面板（独立开关、独立状态显示）
- 线协议与 `litearm-device` 逐字节对齐（跨仓可互通）

**不做**（YAGNI）：

- 夹爪力控遥操 / 触觉回传
- 跨仓真机联调（只做对拍，见 §11.3）
- 多夹爪（一台机器一个夹爪）
- 夹爪录制 / 回放
- 改 `link.py` / `arm_worker.py` / `servo.py` / `wire.py` —— **这几个文件一行不动**

## §4 架构

```
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

### §6.2 两种装法的验算

| 装法 | `pos_closed_rad` | `pos_open_rad` | `close_sign` | `openness=0` → | `openness=1` → |
|---|---|---|---|---|---|
| 正装 | 1.14 | 0.0 | +1 | `1.14` = pos_closed ✓ | `0.0` = pos_open ✓ |
| 反装 | 0.0 | 1.14 | −1 | `0.0` = pos_closed ✓ | `1.14` = pos_open ✓ |

漏掉 `close_sign` 时，反装的 `openness=1` 会算出 `-1.14` —— **镜像且越出标定限位**。

> 注：参考实现的**读侧是对的**（`position_mm` 由 SDK 算、已含 `close_sign`），
> **只有反算回弧度那一侧漏了**。

### §6.3 读侧（主端）

`openness = clamp01(st.position_mm / travel_mm)`，其中 `st` 是 `LiteGrip.get_state(wait=False)`
（`gripper.py:1530-1576`）。`position_mm` 已含 `close_sign`，`travel_mm` 恒正
⇒ 正装/反装都得到 `0 … travel_mm` → `0 … 1`。

⚠ **不用 `GripperState` 上不存在的 `travel_mm`**：参考实现读的是它自己 adapter 造的 dict
（`gripper_teleop.py:391-398`），而 litegrip 的 `GripperState` **没有这个字段**
（`models.py:29-46`）。我们按 §6.1 的公式自己算，来源是 `cfg` 的两个限位与 `rad_to_mm`。

## §7 两端循环（50 Hz）

两端各自**一个后台线程**（照搬参考实现 `gripper_teleop.py:5-7` 的形态）。
`rate_hz` 默认 50（参考实现 `:106`）。

### §7.1 主端

```text
LiteGrip(channel=gcan) → connect() → load_calibration() → enable()
断言 cfg.calibrated 为真（否则拒绝启动，见 §8.1）
Listener(gport, gkey)
每拍：
    lg.send_mit_frame(q=0.0, kp=0.0, kd=0.0)   # 自己维持零重力
    lg.poll(timeout_s=0.0)
    st = lg.get_state(wait=False)
    openness = mm_to_openness(st.position_mm)
    pub.put(encode_gripper_teleop(openness, st.position_mm, st.force_n, time.monotonic()))
收尾：lg.exit_zero_gravity()
```

⚠ **不调 `enter_zero_gravity()`**：它只发**一帧** bootstrap，
docstring 明说「caller must call `update_state` or poll manually to sustain the mode」
（`gripper.py:496-518`）。我们自己每拍发零力矩帧才是持续维持
（参考实现 `:220-224` 同）。

⚠ 零力矩帧的 `q` 传 `0.0`：`kp=kd=0` 时 `q` 不参与力矩计算（参考实现 `:221-222` 同，
它记着「传 0 与 SDK `enter_zero_gravity` 一致」）。

### §7.2 从端

```text
LiteGrip(channel=gcan) → connect() → load_calibration() → enable()
断言 cfg.calibrated 为真
Connector(gpeer, gport, gkey, on_frame=slot.put)   # 回调只写 latest 槽
对齐：等首帧（≤5 s）→ goto_rad(q0, kp, kd, duration=1.0)
每拍：
    payload, _ts = slot.take()
    if payload 且 len == 32:
        openness = clamp01(decode(payload)[0]); q = openness_to_rad(openness)
        stale = False
    elif slot.ever_received 且 slot.peek_age(now) > 0.2:
        stale = True            # 持位：q 不变
    q = clamp(q, 从端自己的标定区间)         # §8.2
    lg.send_mit_frame(q=q, kp=kp, kd=kd, dq=0.0)   # ⚠ stale 时也继续发，防掉力
    lg.poll(timeout_s=0.0)
收尾：再发一帧持位帧 → 关订阅
```

- `dq` 恒 `0.0`：单自由度夹爪不用速度前馈（参考实现 `:289` 的理由 —— 传的是 openness，
  两端速度量纲不通用）。
- `gap` 复用既有的 `link.LatestSlot`（`liteteleop/link.py:127-183`），
  它已经解决了「从未收到」与「刚收到」的区分（`peek_age()` 对「从未收到」返回 `None`），
  这正是 watchdog 需要的语义。
- watchdog = **200 ms**，与臂的 `WATCHDOG_MS`（`arm_worker.py:61`）同值。

### §7.3 主从标定一致性诊断（参考实现没有的一条）

从端同时拿到 `openness` 与 `position_mm`，于是
`position_mm / openness ≈ 主端的 travel_mm`。从端把自己的 `travel_mm` 与它比，
差得超过阈值就**告警**「主从夹爪型号/标定不一致，运动会被按比例缩放」。

挡的是「主从装的是不同型号夹爪」——不报错、只是动作幅度悄悄不对，属于最难查的一类现场。
成本几行，`grasp` 一类的静默缩放风险值得它。

⚠ 只在 `openness` 不接近 0 时算（除数保护），且只告警一次。

## §8 安全约束

| # | 约束 | 依据 |
|---|---|---|
| 1 | **未标定拒绝启动**（`cfg.calibrated` 为假即报错退出） | `send_mit_frame` 与 `goto_rad` **都不检查 `calibrated`** —— `_check_calibrated` 只保护 `open`/`close`/`grasp`（`litegrip/src/litegrip/actions.py:200-215`）。不拦就会拿占位默认值 `pos_closed_rad=1.14`/`pos_open_rad=0.0`（`models.py:90-91`）当真实限位用 |
| 2 | 从端每帧把 `q` 钳进**自己**的标定区间 `[min(pos_closed,pos_open), max(...)]` | 主从标定不一致时不能撞穿从端机械限位。与 `goto_rad` 自己的钳位同源（`gripper.py:1208-1210`）。`openness∈[0,1]` 在标定一致时已隐含此界，但**不一致时不能靠它** |
| 3 | watchdog 超时 → **持位**，不自动重连 | 用户裁决（2026-09-28）。与臂的 `should_stop()`（`arm_worker.py:487-497`）同语义 |
| 4 | 从端**绝不 `disable()`** | 同上。掉力会当场松掉正夹着的物体 —— 对抓取任务是比「继续夹着」更糟的结果。与臂的「绝不 disable」（`arm_worker.py:366`）同规 |
| 5 | 收尾顺序：主端 `exit_zero_gravity()` → 从端发持位帧 → **`ep.close()`** → `lg.disconnect()` | `ep.close()` 缺了**进程永久挂死**，本仓已实测（`liteteleop/link.py:6-7`） |
| 6 | ⛔ **绝不调 `lg.close()`** —— 那是**合爪**不是断开（`gripper.py:1103`） | 断开只有 `disconnect()`（`gripper.py:329`）。这是 SDK 的真实命名陷阱 |
| 7 | 主端零重力期间不调任何 `open`/`close`/`grasp` | 那些会走自己的 MIT 斜坡流，与我们的零力矩帧抢 CAN |

## §9 配置与界面

### §9.1 新增设置（`liteteleop/settings.py`）

```python
gcan:   str = ""         # 夹爪 CAN 通道，空 = 不启用夹爪遥操（默认不启用）
gpeer:  str = "127.0.0.1" # 从端填：主端 IP（独立字段，不读臂的 peer）
gport:  int = 17448       # 夹爪 zenoh 端口（独立 session ⇒ 必须独立端口）
grip_id: str = "gripA"    # 决定夹爪 topic
```

⚠ `gcan` 默认空 ⇒ **默认不启用**，与臂遥操的默认关闭一致（安全默认）。
⚠ 四个字段**都不读臂的同名字段**（§2 铁律第 3 条）。

### §9.2 新增命令行（`liteteleop/__main__.py`）

`--gcan` / `--gpeer` / `--gport` / `--grip-id`，语义与既有 `--cdc`/`--peer`/`--jport`/`--arm-id` 对称。

### §9.3 界面（`gui/pages.py` + `gui/main_window.py` + `gui/bridge.py`）

在既有的 `TeleopPage` 里加一个**夹爪分区**（这是 §2 说的「界面上放在一起显示」）：

- CAN 通道输入框 + 重扫
- 「夹爪遥操」独立开关（**与臂的遥操开关是两个独立控件**）
- 实时读数：`openness` / `position_mm` / `force_n` / 环频 / 帧数 / 匹配状态 / 帧龄 / `stale`
- 告警行（§7.3 的标定不一致告警落在这里）

`WorkerBridge` 加一个 `grip_state = pyqtSignal(object)`（2 行，纯增量）。
GUI 只做两件事：**投命令**、**读快照** —— 与 `ArmWorker` 同一纪律。

## §10 文件清单

**新增**

| 文件 | 职责 |
|---|---|
| `liteteleop/grip_wire.py` | 帧编解码 + topic（与参考实现逐字对齐） |
| `liteteleop/grip_worker.py` | `GripWorker` 线程 + `GripSnapshot` |
| `tests/test_grip_wire.py` | 帧长/往返/topic/大端 |
| `tests/test_grip_worker.py` | fake 夹爪 + 真 zenoh 回环 |

**修改**

| 文件 | 改动 |
|---|---|
| `liteteleop/settings.py` | +4 字段 |
| `liteteleop/__main__.py` | +4 参数 |
| `liteteleop/gui/pages.py` | 夹爪分区 |
| `liteteleop/gui/main_window.py` | 夹爪开关接线 |
| `liteteleop/gui/bridge.py` | +1 信号 |
| `README.md` | 更新状态与用法 |

**一行不动**：`link.py`、`arm_worker.py`、`servo.py`、`safety.py`、`wall.py`、`wire.py`。

## §11 验收（三道闸门）

### §11.1 编译 / 单测

`python3 -m pytest tests -q`（CI 命令，`.github/workflows/ci.yml:49`）。

新增用例中**必须**有一条**有判别力**的：参数化跑正装 / 反装两种标定，断言
`openness=0 → pos_closed_rad` 且 `openness=1 → pos_open_rad`。
**漏掉 `close_sign` 时反装那组必红** —— 这是 §6.1 那条缺陷的回归网。

其余：watchdog 持位（停发帧后从端 `q` 不变且仍在发帧）、对齐、帧长 32、解错帧长不崩。

### §11.2 对拍

1. **线契约对拍**：`diff` 我们的 `grip_wire.py` 与
   `litearm-device/src/litearm_device/gripper_teleop.py:38-75` ——
   格式串 `">4d"`、字段序、`FRAME_SIZE=32`、topic 模板逐项相同。
2. **换算对拍**：`openness_to_rad` 与 `lg.goto()` 的公式在**同输入**下**同输出**
   （对拍对象：`gripper.py:1183-1185`）。
3. **解耦对拍**（验 §2 铁律）：`grep` 证明 `arm_worker.py` 不引用 `grip_worker`/
   `grip_wire`，`grip_worker.py` 除 `link`/`grip_wire` 外不引用臂侧模块。

### §11.3 真机

1. 主端手掰、从端跟随（正装夹爪）
2. **拔网线 / 杀主端** → 从端**持位**：不掉力、不松开、不自动重连
3. 未标定的夹爪 → 启动即拒绝（§8.1）
4. 收尾后进程**正常退出**（验 `ep.close()`，本仓踩过永久挂死）

**已知限制**：与 `litearm-device` 的互通性经**对拍**（§11.2.1）证明，
**不做跨仓真机联调**。

## §12 未决项

1. 跑在哪个 git 分支？（当前在 `feat/teleop-stage1-protocol`，本地分支，远端只有 `origin/main`）
2. 从端是否要「对齐」开关（参考实现默认 `align=True`，我们建议照做，但界面给个开关）
3. 夹爪 `kp`/`kd` 是否暴露到界面（默认用 `cfg.kp=100.0`/`cfg.kd=2.0`，`models.py:83-84`）
