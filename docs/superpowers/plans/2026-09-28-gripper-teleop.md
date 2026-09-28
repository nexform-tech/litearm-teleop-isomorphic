# 夹爪遥操 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 给同构遥操上位机 `liteteleop` 加上夹爪遥操 —— 主端夹爪零重力被人手掰动，从端夹爪跟着开合，走 zenoh 纯点对点。

**Architecture:** 新增一个与 `ArmWorker` **代码上完全解耦**的 `GripWorker`（独立线程 / 独立 CAN / 独立 zenoh session / 独立端口 / 独立开关）。线协议逐字节对齐 `litearm-device` 的 `gripper_teleop.py`。既有的臂侧代码（`link.py` / `arm_worker.py` / `servo.py` / `ports.py`）**一行不动**。

**Tech Stack:** Python ≥3.8（本机 3.13）、zenoh（peer 模式，关全部广播发现）、`litegrip` SDK（SocketCAN / 达妙 MIT 协议）、PyQt5、pytest。

**分支：** `feat/gripper-teleop`（已存在，spec 已在上面提交）。

**Spec：** [docs/superpowers/specs/2026-09-28-gripper-teleop-design.md](../specs/2026-09-28-gripper-teleop-design.md)

> ⚠⚠ **本计划的代码块已在 `/tmp/grip_proto` 里真编真跑过**（24 passed），并做了 **5 处变异测试**逐条确认判据有判别力（详见 Task 7）。粘贴时**照抄**，不要"顺手优化"。

---

## 文件结构

| 文件 | 责任 | 状态 |
|---|---|---|
| `liteteleop/grip_wire.py` | 32 B 帧编解码 + topic（与参考实现逐字对齐） | 新增 |
| `liteteleop/grip_worker.py` | `GripWorker` 线程 + `GripSnapshot` + 换算/前置纯函数 + SDK 钉死 | 新增 |
| `tests/fake_grip.py` | **本仓自带**的极简夹爪替身（夹爪 SDK 那份 `fake_can.py` 在 CI 上起不来） | 新增 |
| `tests/test_grip_wire.py` | 帧契约 | 新增 |
| `tests/test_grip_worker.py` | 换算 / 前置 / 收尾 / 两端环 | 新增 |
| `liteteleop/settings.py` | +4 字段 | 修改 |
| `liteteleop/__main__.py` | +4 参数 + SDK 钉死接线 | 修改 |
| `liteteleop/gui/bridge.py` | +1 信号 | 修改 |
| `liteteleop/gui/pages.py` | 夹爪分区 | 修改 |
| `liteteleop/gui/main_window.py` | 夹爪开关接线 | 修改 |
| `README.md` | 更新用法 | 修改 |

**冻结（一行不动）**：`liteteleop/link.py`、`arm_worker.py`、`servo.py`、`safety.py`、`wall.py`、`wire.py`、`ports.py`。

---

## Task 1: 线协议 `grip_wire.py`

**Files:**
- Create: `liteteleop/grip_wire.py`
- Test: `tests/test_grip_wire.py`

- [ ] **Step 1: 写失败测试**

创建 `tests/test_grip_wire.py`：

```python
"""夹爪线协议 —— 与 litearm-device 逐字节对齐（spec §5 / §11.1.6）。"""
import struct

import pytest

from liteteleop import grip_wire


def test_frame_is_32_bytes_big_endian():
    """格式串与帧长与参考实现 `gripper_teleop.py:51-52` 逐字相同。"""
    assert grip_wire.GRIP_FORMAT == ">4d"
    assert grip_wire.GRIP_FRAME_BYTES == 32


def test_roundtrip():
    p = grip_wire.encode_gripper_teleop(0.25, 30.0, 12.5, 1234.5)
    assert len(p) == 32
    assert grip_wire.decode_gripper_teleop(p) == (0.25, 30.0, 12.5, 1234.5)


def test_field_order_puts_openness_first():
    """⚠ 参考实现的**模块 docstring**（`:14-18`）写的是 `position_rad` 在前，
    那是陈旧文字；真代码（`:51` + `:55-56` 的签名）是 `openness` 在前。
    本用例钉住**代码**那一版。"""
    p = grip_wire.encode_gripper_teleop(0.5, 60.0, 20.0, 7.0)
    assert struct.unpack(">4d", p)[0] == 0.5
    assert struct.unpack(">4d", p)[2] == 20.0


def test_topic_matches_device_template():
    """与 `gripper_teleop.py:73-75` 的模板逐字相同。"""
    assert grip_wire.gripper_teleop_topic("gripA") == "litearm/v4/gripA/gripper_teleop"


def test_wrong_length_raises():
    """不做"尽量解"的兜底 —— 与 `wire.py:64-66` 的臂帧同规。"""
    with pytest.raises(struct.error):
        grip_wire.decode_gripper_teleop(b"\x00" * 31)
```

- [ ] **Step 2: 跑测试，确认失败**

Run: `python3 -m pytest tests/test_grip_wire.py -q`
Expected: FAIL —— `ModuleNotFoundError: No module named 'liteteleop.grip_wire'`

- [ ] **Step 3: 写实现**

创建 `liteteleop/grip_wire.py`：

```python
"""夹爪遥操线协议 —— **逐字照搬 litearm-device**（`gripper_teleop.py:38-75`）。

⛔ 本模块**不是**新设计。litearm-device 的夹爪遥操是已经真机跑通的实现
（用户裁决 2026-09-28：「逻辑和 litearm-device 中遥操逻辑一致就可以」），
所以连**格式串、字段序、topic 模板**都保持一致，以便逐行对账（`diff` 得出来）。

布局（**大端**，定长）::

    struct ">4d"  =  32 字节
    [0] openness     f64 BE   归一化开合度 [0,1]：0=闭合，1=全开  ← **主传输量**
    [1] position_mm  f64 BE   开合宽度 mm（单侧），仅诊断/显示
    [2] force_n      f64 BE   夹持力 N，仅诊断
    [3] timestamp    f64 BE   发帧方的本地时间（秒），从端 watchdog 判活

⚠ **为什么传 openness 而不是弧度**（参考实现 `:44-49` 的真机结论）：
   每台夹爪的电机零点/方向/标定各不相同（如 A 张开=-1.42 rad，B 张开=+1.14 rad），
   直接传弧度无法跨夹爪通用。openness 由各自标定的 position_mm/travel_mm 归一化得到，
   方向统一、量纲无关，从端再按自己的标定映射回弧度。

⚠⚠ **参考实现的模块 docstring 与代码不一致，别照抄那段文字** ——
   `gripper_teleop.py:14-18` 仍写着 `[0] position_rad, [1] velocity_rad_s`，
   而 `:51` 的真代码是 `struct.Struct(">4d")` 的
   `(openness, position_mm, force_n, timestamp)`。**以代码为准**；
   那段 docstring 是改设计时漏改的陈旧文字（`:44-49` 的注释解释了为何改）。

⚠ `timestamp` 是**发帧方的本地时间**，两端不同源（与 `wire.py:17` 的臂帧同理）
   ⇒ 只能算「同一主端相邻两帧的间隔」，**不能**算端到端延迟；
   从端的 watchdog 必须用**本地**收帧时刻判活（见 `grip_worker.py`）。
"""
from __future__ import annotations

import struct

__all__ = ["GRIP_FORMAT", "GRIP_FRAME_BYTES", "gripper_teleop_topic",
           "encode_gripper_teleop", "decode_gripper_teleop"]


#: 与 `gripper_teleop.py:51` 的 `_FRAME` 逐字相同。
GRIP_FORMAT = ">4d"

#: 帧长（字节）。`struct.calcsize(">4d") == 32`，与参考实现 `FRAME_SIZE` 同值。
GRIP_FRAME_BYTES = struct.calcsize(GRIP_FORMAT)


def gripper_teleop_topic(grip_id: str) -> str:
    """夹爪遥操 zenoh topic。主端用自己的 `grip_id` 发布，从端用**主端的** `grip_id` 订阅。

    与 `gripper_teleop.py:73-75` 逐字相同（那边参数名是 `device_id`）。
    帧格式也已对齐 ⇒ 与 litearm-device 的两端**可以**互通。
    """
    return f"litearm/v4/{grip_id}/gripper_teleop"


def encode_gripper_teleop(openness: float, position_mm: float,
                          force_n: float, timestamp: float) -> bytes:
    """编码一帧夹爪遥操数据（32 字节，大端）。

    Args:
        openness: 归一化开合度 [0,1]（0=闭合，1=全开），**主传输量**。
        position_mm: 开合宽度 mm（单侧），诊断/显示用。
        force_n: 夹持力 N，诊断用。
        timestamp: 发帧方的本地时间（秒），从端 watchdog 判活。
    """
    return struct.pack(GRIP_FORMAT, openness, position_mm, force_n, timestamp)


def decode_gripper_teleop(payload: bytes) -> tuple:
    """解码 → ``(openness, position_mm, force_n, timestamp)``。

    ⚠ 帧长不对时 `struct.unpack` 抛 `struct.error` —— **这是刻意的**，
    与参考实现和本仓 `wire.py:64-66` 的臂帧同规：不做"尽量解"的兜底，
    解不出来就是解不出来。调用方自己先判 `len(payload) == GRIP_FRAME_BYTES`。
    """
    return struct.unpack(GRIP_FORMAT, payload)
```

- [ ] **Step 4: 跑测试，确认通过**

Run: `python3 -m pytest tests/test_grip_wire.py -q`
Expected: `5 passed`

- [ ] **Step 5: 提交**

```bash
git add liteteleop/grip_wire.py tests/test_grip_wire.py
git commit -m "feat(grip): 夹爪遥操线协议 32 B 帧，与 litearm-device 逐字对齐"
```

---

## Task 2: 夹爪替身 `fake_grip.py` + `GripWorker` + 纯函数用例

**Files:**
- Create: `tests/fake_grip.py`
- Create: `liteteleop/grip_worker.py`
- Test: `tests/test_grip_worker.py`

- [ ] **Step 1: 写替身 `tests/fake_grip.py`**

⚠ 关键约束（spec §11.1 开头）：**不复用** `litegrip-python/tests/fake_can.py` —— CI 只装 `pytest` 与 `zenoh`，不装 litegrip；且那份第 14 行有 `import _sdkpath`，本仓 `tests/` 是包而 litegrip 的不是，**在 CI 上起不来**（本机实测）。

⚠⚠ 替身**故意不实现 `disable()`** —— 任何"顺手失能"的路径会当场 `AttributeError`。

```python
"""本仓自带的极简夹爪替身 —— 让 `GripWorker` 的用例不依赖外部仓、也不碰硬件。

⚠ **不复用 `litegrip-python/tests/fake_can.py`**：CI 只装 `pytest` 与 `zenoh`
（`.github/workflows/ci.yml`），**不装 litegrip**；且那份第 14 行有 `import _sdkpath`
（要它自己的目录在 `sys.path` 上），而本仓 `tests/` 是包、litegrip 的 `tests/` 不是
⇒ 在 CI 上起不来（本机已实测）。那份**只用于对拍**（本机手工跑）。

⚠⚠ 本替身**故意没有 `disable()` 方法** —— 任何"顺手失能"的代码路径会当场
`AttributeError`，这本身就是一道结构性护栏（spec §8 rule 4）。

⚠⚠ `disconnects` 只是调用计数。要断言「收尾不失能」**不能**看这里 ——
该断言打在**构造参数**上（见 `test_factory_keeps_motor_enabled_on_disconnect`），
因为本设计里 worker 是直接 `g.disconnect()` 的，`disable_on_disconnect`
在**构造时**决定行为。
（litegrip 那份 `fake_can.py` 的 `disconnect(self, disable=True)` 函数体
只有 `self.disconnected = True`、**忽略 `disable`** ⇒ 拿它断言是零判别力的。）
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional

__all__ = ["FakeCfg", "FakeState", "FakeGrip", "NORMAL", "REVERSE"]

#: 取自 SDK 随包模板 `litegrip-python/src/litegrip/calibration_normal.json`
#: （`rad_to_mm=74.8`，`travel_range_rad=1.605` ⇒ `travel_mm ≈ 120.05`）。
NORMAL = dict(pos_closed_rad=0.114, pos_open_rad=-1.491, rad_to_mm=74.8)
#: 同一份模板的反装版（两个限位对调）。
REVERSE = dict(pos_closed_rad=-1.491, pos_open_rad=0.114, rad_to_mm=74.8)


@dataclass
class FakeCfg:
    """`GripperConfig` 的最小鸭子类型。字段名与真货逐字相同
    （`pos_closed_rad` / `pos_open_rad` / `rad_to_mm` / `calibrated` / `kp` / `kd`）。"""

    pos_closed_rad: float = 0.114
    pos_open_rad: float = -1.491
    rad_to_mm: float = 74.8
    calibrated: bool = True
    kp: float = 100.0
    kd: float = 2.0

    @property
    def close_sign(self) -> float:
        """与 `models.py:109-118` 逐字相同。"""
        return 1.0 if self.pos_closed_rad >= self.pos_open_rad else -1.0


@dataclass
class FakeState:
    """`GripperState` 的最小子集。"""

    position_rad: float = 0.0
    position_mm: float = 0.0
    force_n: float = 0.0


class FakeGrip:
    """鸭子类型的夹爪替身。**记录每一次调用**，供断言。"""

    def __init__(self, cfg: Optional[FakeCfg] = None,
                 position_rad: Optional[float] = None,
                 calibrated: bool = True):
        self.config = cfg or FakeCfg()
        self._calibrated = calibrated
        self._pos = (self.config.pos_closed_rad if position_rad is None
                     else float(position_rad))

        self.connected = False
        self.loaded = False
        self.enable_calls = 0
        self.zero_gravity_exits = 0
        self.polls = 0
        self.sent: List[dict] = []          # 每次 send_mit_frame 的实参
        self.gotos: List[dict] = []         # 每次 goto_rad 的实参
        self.disconnects = 0
        self.raise_on_enable: Optional[BaseException] = None

    # ── 位置：测试可以"用手掰" ──
    @property
    def position_rad(self) -> float:
        return self._pos

    def drag_to(self, rad: float) -> None:
        """模拟人手动把夹爪掰到某个弧度。"""
        self._pos = float(rad)

    # ── 生命周期 ──
    def connect(self):
        self.connected = True
        return True

    def load_calibration(self, *a, **k):
        self.loaded = True
        self.config.calibrated = self._calibrated
        return True

    def enable(self, *a, **k):
        self.enable_calls += 1
        if self.raise_on_enable is not None:
            raise self.raise_on_enable
        return True

    def disconnect(self):
        """⚠ 只记调用次数。**真货的 `disable_on_disconnect` 语义在构造参数上**，
        本替身不模拟它（那会造出零判别力的断言）—— 见模块 docstring。"""
        self.disconnects += 1
        self.connected = False

    # ── 读写 ──
    def get_state(self, wait: bool = True) -> FakeState:
        if not wait:
            self.polls += 1
        # 复刻 SDK 的公式（`gripper.py:1561-1562`），含 close_sign
        s = self.config.close_sign
        mm = ((self.config.pos_closed_rad - self._pos) * s * self.config.rad_to_mm)
        return FakeState(position_rad=self._pos, position_mm=mm, force_n=0.0)

    def poll(self, timeout_s: float = 0.0) -> bool:
        self.polls += 1
        return True

    def send_mit_frame(self, q: float, kp: float, kd: float,
                       dq: float = 0.0, tau: float = 0.0) -> bool:
        self.sent.append({"q": float(q), "kp": float(kp), "kd": float(kd),
                          "dq": float(dq), "tau": float(tau)})
        # kp != 0 ⇒ 这是位置指令，夹爪会朝它走（零力矩帧不改位置）
        if kp:
            self._pos = float(q)
        return True

    def goto_rad(self, position_rad: float, kp=None, kd=None,
                 dq_target: float = 0.0, tau_feedforward: float = 0.0,
                 duration: float = 0.5) -> bool:
        self.gotos.append({"q": float(position_rad), "kp": kp, "kd": kd,
                           "duration": duration})
        self._pos = float(position_rad)
        return True

    def exit_zero_gravity(self) -> None:
        self.zero_gravity_exits += 1

    # ── 供断言的小工具 ──
    def last_sent_q(self) -> Optional[float]:
        return self.sent[-1]["q"] if self.sent else None

    def clear_sent(self) -> None:
        self.sent.clear()
```

- [ ] **Step 2: 写实现 `liteteleop/grip_worker.py`**

⚠ 照抄。三处**不能改**：`disable_on_disconnect=False`、`close_sign`、从端 `q_cmd` 的初值。

```python
"""`GripWorker` —— **唯一持有夹爪的线程**。

⚠⚠ **与 `ArmWorker` 代码上完全解耦**（用户裁决 2026-09-28）：独立线程、独立对象、
独立 zenoh session、独立端口、独立生命周期、零共享状态。两者之间**没有任何直接引用**。
界面把两者放在一起显示，仅此而已。

拓扑（与 litearm-device 的 `_master_loop`/`_slave_loop` 同构）::

    主端: 夹爪零重力（自己每拍发零力矩帧，人可掰动）+ 50 Hz pub 32 B 帧
    从端: 连主端 IP:port → 订阅 → 首帧 goto_rad 对齐 → send_mit_frame 位置跟随
          watchdog 超时 **持位**（继续发帧，不掉力、不松开、不自动重连）

⚠ 收尾**绝不失能**（spec §8 rule 4/6）：构造夹爪时必须传
   `disable_on_disconnect=False` —— 它的**默认值是 True**，
   默认走 `disconnect()` 会把电机关掉、当场松掉正夹着的物体。
"""
from __future__ import annotations

import logging
import sys
import threading
import time
from dataclasses import dataclass
from typing import Callable, Optional

from . import link
from .grip_wire import (GRIP_FRAME_BYTES, decode_gripper_teleop,
                        encode_gripper_teleop, gripper_teleop_topic)

log = logging.getLogger("liteteleop.grip")

__all__ = ["ROLE_MASTER", "ROLE_SLAVE", "GRIP_RATE_HZ", "GRIP_WATCHDOG_MS",
           "GRIP_SDK_SRC", "GripNotReady", "GripSnapshot", "GripWorker",
           "assert_sdk_pinned", "pin_grip_sdk",
           "clamp_to_calibrated", "mm_to_openness", "openness_to_rad",
           "travel_mm_of"]

ROLE_MASTER = "master"
ROLE_SLAVE = "slave"

#: 与 litearm-device 的 `rate_hz=50.0`（`gripper_teleop.py:106`）同值。
GRIP_RATE_HZ = 50.0

#: 与臂的 `WATCHDOG_MS`（`arm_worker.py:61`）和参考实现（`:110`）同值。
GRIP_WATCHDOG_MS = 200.0

#: 主从 `travel_mm` 偏差超过这个比例就告警（spec §7.3）。
_MISMATCH_TOL = 0.15

#: 只在 `openness` **没有被 `clamp01` 饱和**的窗口里做标定一致性比对。
#: `openness→1` 时比值会报出主端**原始** `position_mm`（大于其真实 travel）⇒ 会误报。
_MISMATCH_WINDOW = (0.15, 0.85)

#: ⚠⚠ 夹爪 SDK 必须钉死。**本机有两份包名都叫 `litegrip` 的仓**：
#:   `/home/llx/litegrip-python`（github nexform-tech，本设计的目标，src 布局）
#:   `/home/llx/moduangongju/lite-grip`（gitee，被 editable 装成 `litegrip` 2.2.0）
#: 裸 `import litegrip` **落到后者**（实测）。两者的 API 面并不相同
#: （`close_sign`/`calibrated`/`load_template` 只在目标那份有）⇒ 走错仓从
#: `AttributeError` 到**静默语义漂移**都可能。
#: 用户裁决 2026-09-28：SDK 采用 `litegrip-python`。
GRIP_SDK_SRC = "/home/llx/litegrip-python/src"


class GripNotReady(RuntimeError):
    """夹爪没做好遥操准备（未标定 / 零行程 / `rad_to_mm` 为 0）。"""


# ────────────────────────────── SDK 钉死（spec §9.4）──────────────────────────────

def assert_sdk_pinned(mod_file: str, src: str = GRIP_SDK_SRC) -> None:
    """判据：导入后 `litegrip.__file__` 必须落在 `src` 下，否则**抛 `SystemExit`**。

    ⚠ **断言不能省，只 `sys.path.insert` 不算数。**
    本机那份 editable 安装是用 `sys.meta_path.append(_EditableFinder)` 注册的
    （`__editable___litegrip_2_2_0_finder.py` 的 `install()`），排在 `PathFinder`
    **之后** ⇒ `sys.path.insert(0, ...)` 目前**压得住**（实测）。
    但这是**实现细节**：一旦改成 `sys.meta_path.insert` 式注册，
    `sys.path` 插入就会**静默失效** —— 只有这条断言能抓住它。
    与 `liteteleop/__main__.py:11-21` 的 `_pin_sdk()` 同款纪律。
    """
    if not str(mod_file).startswith(str(src)):
        raise SystemExit(
            f"⛔ litegrip 导入自 {mod_file}，不是 {src} —— 环境里有另一份抢先了")


def pin_grip_sdk(src: str = GRIP_SDK_SRC) -> str:
    """把夹爪 SDK 钉到 `src`，断言后返回实际导入路径（供启动日志打印）。"""
    if src not in sys.path:
        sys.path.insert(0, src)
    import litegrip                          # noqa: PLC0415 - 故意延迟到 pin 之后
    assert_sdk_pinned(litegrip.__file__, src)
    return str(litegrip.__file__)


# ────────────────────────────── 换算（spec §6）──────────────────────────────

def travel_mm_of(cfg) -> float:
    """满行程 mm。与 SDK 的 `CalibrationData.travel_mm`（`models.py:163`）同义，
    但我们从 `cfg` 的**两个限位**自己算 —— 跑起来之后手头只有 `cfg`，
    且这样 `travel_mm` 与 `close_sign` 出自**同一组数**，不会分叉。"""
    return (abs(float(cfg.pos_open_rad) - float(cfg.pos_closed_rad))
            * float(cfg.rad_to_mm))


def _clamp01(x: float) -> float:
    return 0.0 if x < 0.0 else (1.0 if x > 1.0 else x)


def mm_to_openness(cfg, position_mm: float) -> float:
    """读侧：`position_mm`（SDK 算的，**已含 `close_sign`**）→ `openness[0,1]`。"""
    t = travel_mm_of(cfg)
    return _clamp01(float(position_mm) / t) if t > 0 else 0.0


def openness_to_rad(cfg, openness: float) -> float:
    """`openness[0,1]` → 电机弧度（用**本机**标定）。

    复刻 SDK 的 `goto` 公式（`litegrip/src/litegrip/gripper.py:1183-1185`）::

        position_rad = pos_closed_rad - close_sign * position_mm / rad_to_mm
        position_mm  = openness * travel_mm

    ⚠⚠ `close_sign` **不能省**。参考实现 `gripper_teleop.py:155-163` 漏了它
    ⇒ 反装夹爪会**镜像**（`openness=1` 算出 -3.096 rad，远出标定限位）。
    正装时 `close_sign == +1` ⇒ 与参考实现**逐位相同**，不影响互通。
    """
    t = travel_mm_of(cfg)
    return (float(cfg.pos_closed_rad)
            - float(cfg.close_sign) * (_clamp01(openness) * t) / float(cfg.rad_to_mm))


def clamp_to_calibrated(cfg, q: float) -> float:
    """把 `q` 钳进**本机**标定区间（spec §8.2）。

    两个限位谁大谁小都可能（反装），所以取 min/max 而不是假定顺序 ——
    与 SDK 自己的 `goto_rad` 钳位同源（`gripper.py:1208-1210`）。
    """
    lo = min(float(cfg.pos_closed_rad), float(cfg.pos_open_rad))
    hi = max(float(cfg.pos_closed_rad), float(cfg.pos_open_rad))
    return max(lo, min(hi, float(q)))


def check_ready(cfg) -> None:
    """spec §8.1 的三条前置。⚠ **必须在 `enable()` 之前调。**

    `send_mit_frame` 与 `goto_rad` **都不检查 `calibrated`** —— SDK 的
    `_check_calibrated` 只保护 `open`/`close`/`grasp`
    （`litegrip/src/litegrip/actions.py:200-215`）。不拦就会拿**占位默认值**
    `pos_closed_rad=1.14` / `pos_open_rad=0.0`（`models.py:90-91`）当真实限位用。

    ⚠ 零行程判据写在 **rad 空间**（`<= 1e-6`），与 SDK 同款 ——
    写成 mm 空间的 `travel_mm == 0` 是**更弱**的条件。
    """
    if not bool(getattr(cfg, "calibrated", False)):
        raise GripNotReady(
            "夹爪未标定：pos_closed_rad / pos_open_rad 还是占位默认值，方向是猜的。"
            "先 load_calibration() / load_template()，或跑一次 zero()。")
    if abs(float(cfg.pos_closed_rad) - float(cfg.pos_open_rad)) <= 1e-6:
        raise GripNotReady("行程为零：pos_closed_rad 与 pos_open_rad 相同 —— 重新标定。")
    if not float(cfg.rad_to_mm):
        raise GripNotReady("rad_to_mm 为 0：openness↔rad 换算会除零。")


def _default_gripper_factory(channel: str):
    """默认夹爪工厂。⚠ `disable_on_disconnect=False` **不能省**（spec §8 rule 6）。

    另注 SDK 的一个怪癖（`gripper.py:209-212`）：`channel` 只有在**与默认值不同**
    或 `config is None` 时才生效。我们这里不传 `config` ⇒ `channel` 生效。
    """
    import litegrip                          # noqa: PLC0415
    return litegrip.LiteGrip(channel=channel, disable_on_disconnect=False)


# ────────────────────────────── 快照 ──────────────────────────────

@dataclass
class GripSnapshot:
    """**只读**快照 —— 界面只看这个，不碰夹爪。"""

    connected: bool = False
    role: str = ""
    topic: str = ""
    openness: float = 0.0
    position_mm: float = 0.0
    force_n: float = 0.0
    travel_mm: float = 0.0
    frames_sent: int = 0
    frames_received: int = 0
    loop_hz: float = 0.0
    matching: bool = False              # 主端：有无订阅者（**是布尔不是计数**）
    frame_age: Optional[float] = None   # 从端：最新一帧有多老（s）；None = 从未收到
    stale: bool = False                 # 从端：watchdog 超时、正在持位
    watchdog_trips: int = 0
    mismatch: str = ""                  # 主从标定不一致告警（§7.3），空 = 无
    error: str = ""


# ────────────────────────────── worker ──────────────────────────────

class GripWorker:
    """独占夹爪的线程。生命周期：`start()` → … → `stop()`。"""

    def __init__(self, role: str, gcan: str, grip_id: str = "gripA",
                 gpeer: str = "127.0.0.1", gport: int = 17448,
                 rate_hz: float = GRIP_RATE_HZ,
                 kp: Optional[float] = None, kd: Optional[float] = None,
                 align: bool = True, watchdog_ms: float = GRIP_WATCHDOG_MS,
                 gripper_factory: Optional[Callable[[str], object]] = None,
                 on_state: Optional[Callable[[GripSnapshot], None]] = None,
                 on_log: Optional[Callable[[str], None]] = None):
        if role not in (ROLE_MASTER, ROLE_SLAVE):
            raise ValueError(f"role 必须是 {ROLE_MASTER}/{ROLE_SLAVE}，收到 {role!r}")
        self.role = role
        self.gcan = gcan
        self.grip_id = grip_id
        self.gpeer = gpeer
        self.gport = int(gport)
        self.rate_hz = float(rate_hz)
        self.kp = kp
        self.kd = kd
        self.align = bool(align)
        self.watchdog_ms = float(watchdog_ms)
        self._topic = gripper_teleop_topic(grip_id)
        self._factory = gripper_factory or _default_gripper_factory
        self._on_state = on_state
        self._on_log = on_log

        self._grip = None
        self._cfg = None
        self._pub = None            # 主端：link.Listener
        self._sub = None            # 从端：link.Connector
        self._lock = threading.Lock()
        self._snap = GripSnapshot(role=role, topic=self._topic)
        self._stop = threading.Event()
        self._want = False
        self._thread: Optional[threading.Thread] = None
        self._frames_sent = 0
        self._frames_received = 0
        self._watchdog_trips = 0
        self._mismatch = ""
        self._mismatch_checked = False
        self._loops = 0
        self._loop_hz = 0.0
        self._hz_t0 = 0.0
        self._hz_n0 = 0

    # ────────────────────── 生命周期 ──────────────────────

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name="GripWorker", daemon=True)
        self._thread.start()

    def stop(self, timeout: float = 5.0) -> None:
        """停止并收尾。**幂等**。⚠ 收尾**不失能**（spec §8 rule 4/6）。"""
        self._stop.set()
        self._want = False
        t = self._thread
        if t is not None:
            t.join(timeout=timeout)
            if t.is_alive():
                self._log("⚠ GripWorker 未在超时内退出")
        self._thread = None

    def set_teleop(self, on: bool) -> None:
        """请求启动/停止夹爪遥操。**与臂的开关是两个独立控件**（spec §2）。"""
        self._want = bool(on)

    def snapshot(self) -> GripSnapshot:
        with self._lock:
            s = self._snap
            s.frames_sent = self._frames_sent
            s.frames_received = self._frames_received
            s.watchdog_trips = self._watchdog_trips
            s.loop_hz = self._loop_hz
            s.mismatch = self._mismatch
            if self._pub is not None:
                s.matching = self._pub.matching
            return s

    def _log(self, msg: str) -> None:
        log.info(msg)
        if self._on_log is not None:
            try:
                self._on_log(msg)
            except Exception:                    # noqa: BLE001
                log.exception("on_log 回调炸了")

    def _publish_state(self) -> None:
        if self._on_state is not None:
            try:
                self._on_state(self.snapshot())
            except Exception:                    # noqa: BLE001
                log.exception("on_state 回调炸了")

    # ────────────────────── 主循环 ──────────────────────

    def _run(self) -> None:
        try:
            self._grip = self._factory(self.gcan)
            self._grip.connect()
            self._grip.load_calibration()
            self._cfg = self._grip.config
            # ⚠⚠ **必须在 enable() 之前** —— 否则未标定的夹爪会先被使能、再被拒
            check_ready(self._cfg)
            self._grip.enable()
            with self._lock:
                self._snap.connected = True
                self._snap.travel_mm = travel_mm_of(self._cfg)
            self._log(f"夹爪已就绪：{self.gcan}  travel={travel_mm_of(self._cfg):.2f} mm  "
                      f"close_sign={float(self._cfg.close_sign):+.0f} rad_to_mm="
                      f"{float(self._cfg.rad_to_mm):.2f}")
            while not self._stop.is_set():
                if self._want:
                    self._run_teleop()
                else:
                    time.sleep(0.1)
                    self._publish_state()
        except Exception as e:                   # noqa: BLE001
            with self._lock:
                self._snap.error = str(e)
            self._log(f"⛔ 夹爪 {e}")
        finally:
            self._teardown()

    def _run_teleop(self) -> None:
        try:
            if self.role == ROLE_MASTER:
                self._master_loop()
            else:
                self._slave_loop()
        except Exception as e:                   # noqa: BLE001
            self._log(f"⛔ 夹爪遥操异常退出: {e}")
            with self._lock:
                self._snap.error = str(e)
        finally:
            self._want = False
            self._handoff()

    def _handoff(self) -> None:
        """遥操停下时的交接：**发一帧持位帧**，绝不失能。"""
        g = self._grip
        if g is None:
            return
        try:
            if self.role == ROLE_MASTER:
                # 内部就是「按当前位、kp/kd 取 cfg」的一帧（gripper.py:520-526）
                g.exit_zero_gravity()
            else:
                st = g.get_state(wait=False)
                q = clamp_to_calibrated(self._cfg, st.position_rad)
                g.send_mit_frame(q=q, kp=self._kp(), kd=self._kd(), dq=0.0)
            self._log("已交接：夹爪保持当前位置，**不失能**")
        except Exception as e:                   # noqa: BLE001
            self._log(f"⚠ 交接持位帧失败: {e}")
        self._publish_state()

    # ────────────────────── 主端 ──────────────────────

    def _master_loop(self) -> None:
        """零重力（每拍自己发零力矩帧）+ 定频发布当前位置。

        ⚠ **不调 `enter_zero_gravity()`**：它只发**一帧** bootstrap，
        docstring 明说「caller must poll/sustain to sustain the mode」
        （`gripper.py:496-518`）。自己每拍发才是持续维持（参考实现 `:220-224` 同）。
        ⚠ 零力矩帧的 `q` 传 `0.0`：`kp=kd=0` 时 `q` 不参与力矩计算。
        """
        g = self._grip
        self._pub = link.Listener(self.gport, self._topic)
        self._log(f"主端夹爪：零重力拖动 · 发布 {self._topic} @ 端口 {self.gport} "
                  f"· {self.rate_hz:.0f} Hz")
        self._hz_reset()
        dt = 1.0 / self.rate_hz
        nxt = time.monotonic() + dt
        while self._want and not self._stop.is_set():
            t0 = time.monotonic()
            g.send_mit_frame(q=0.0, kp=0.0, kd=0.0)
            g.poll(timeout_s=0.0)
            st = g.get_state(wait=False)
            openness = mm_to_openness(self._cfg, st.position_mm)
            self._pub.put(encode_gripper_teleop(openness, st.position_mm,
                                                st.force_n, time.monotonic()))
            self._frames_sent += 1
            with self._lock:
                self._snap.openness = openness
                self._snap.position_mm = float(st.position_mm)
                self._snap.force_n = float(st.force_n)
            self._publish_state()
            nxt = self._pace(t0, nxt, dt)

    # ────────────────────── 从端 ──────────────────────

    def _slave_loop(self) -> None:
        g = self._grip
        slot = link.LatestSlot()

        def _on_wire(payload: bytes) -> None:
            # ⚠ **必须单参包装**：`LatestSlot.put(payload, now)` 收两个参数，
            #   而 zenoh 回调只给一个（`link.py:118`）⇒ 直接 `on_frame=slot.put` 会 TypeError。
            #   且在 zenoh 自己的线程上 ⇒ **只许写槽**。
            slot.put(payload, time.monotonic())

        self._sub = link.Connector(self.gpeer, self.gport, self._topic,
                                  on_frame=_on_wire)
        self._log(f"从端夹爪：订阅 {self._topic} @ {self.gpeer}:{self.gport} "
                  f"· align={self.align} · watchdog={self.watchdog_ms:.0f} ms")

        # ⚠⚠ 循环入口先把 q_cmd 定成**本机实测位置**。
        #    首帧到达前 q_cmd 若是 0.0，那是一个**真实位置指令** —— 按装法不同
        #    可能直冲机械限位（正装 0.0 = 全开，反装 0.0 = 全闭）。
        q_cmd = clamp_to_calibrated(self._cfg, g.get_state(wait=False).position_rad)
        stale = False

        if self.align:
            first = self._wait_first_frame(slot, timeout_s=5.0)
            if first is not None:
                o0 = _clamp01(first[0])
                q_cmd = openness_to_rad(self._cfg, o0)
                self._log(f"对齐首帧 → openness={o0:.3f} → {q_cmd:+.4f} rad")
                try:
                    g.goto_rad(q_cmd, kp=self.kp, kd=self.kd, duration=1.0)
                except Exception as e:           # noqa: BLE001
                    self._log(f"⚠ 对齐 goto_rad 失败: {e}")
            else:
                self._log("⚠ 5 s 内没收到主端帧 —— 保持当前位置继续发帧")

        self._hz_reset()
        dt = 1.0 / self.rate_hz
        nxt = time.monotonic() + dt
        while self._want and not self._stop.is_set():
            t0 = time.monotonic()
            payload, _ts = slot.take()
            if payload is not None and len(payload) == GRIP_FRAME_BYTES:
                openness, position_mm, force_n, _t = decode_gripper_teleop(payload)
                q_cmd = openness_to_rad(self._cfg, _clamp01(openness))
                stale = False
                self._frames_received += 1
                self._check_mismatch(_clamp01(openness), float(position_mm))
                with self._lock:
                    self._snap.openness = _clamp01(openness)
                    self._snap.position_mm = float(position_mm)
                    self._snap.force_n = float(force_n)
            else:
                # `peek_age` 对「从未收到」返回 None（`link.py:170-183`）
                # ⇒ `is not None` 正好等于「收到过首帧之后 watchdog 才生效」
                age = slot.peek_age(time.monotonic())
                if age is not None and age * 1000.0 > self.watchdog_ms:
                    if not stale:
                        self._watchdog_trips += 1
                        self._log(f"⚠ watchdog 超时 {self.watchdog_ms:.0f} ms "
                                  "—— **持位**（继续发帧：不掉力、不松开、不重连）")
                    stale = True
            # §8.2：即使 stale 也照发，只是 q_cmd 不变
            q_cmd = clamp_to_calibrated(self._cfg, q_cmd)
            g.send_mit_frame(q=q_cmd, kp=self._kp(), kd=self._kd(), dq=0.0)
            g.poll(timeout_s=0.0)
            with self._lock:
                self._snap.stale = stale
                self._snap.frame_age = slot.peek_age(time.monotonic())
            self._publish_state()
            nxt = self._pace(t0, nxt, dt)

    def _wait_first_frame(self, slot, timeout_s: float) -> Optional[tuple]:
        deadline = time.monotonic() + timeout_s
        while (self._want and not self._stop.is_set()
               and time.monotonic() < deadline):
            payload, _ts = slot.take()
            if payload is not None and len(payload) == GRIP_FRAME_BYTES:
                return decode_gripper_teleop(payload)
            time.sleep(0.01)
        return None

    def _check_mismatch(self, openness: float, position_mm: float) -> None:
        """spec §7.3：从端能同时拿到 `openness` 与 `position_mm`，
        于是 `position_mm / openness ≈ 主端的 travel_mm`。差太多就告警。

        挡的是「主从装的是不同型号夹爪」—— 不报错、只是动作幅度悄悄不对。
        ⚠ 只在**没被 `clamp01` 饱和**的窗口里比（`openness→1` 会把主端**原始**
        `position_mm` 报出来 ⇒ 误报）；⚠ **只告警一次**。
        """
        if self._mismatch_checked:
            return
        lo, hi = _MISMATCH_WINDOW
        if not (lo <= openness <= hi):
            return
        implied = float(position_mm) / openness
        mine = travel_mm_of(self._cfg)
        if implied <= 0 or mine <= 0:
            return
        dev = abs(implied - mine) / mine
        if dev > _MISMATCH_TOL:
            self._mismatch_checked = True
            self._mismatch = (f"主从夹爪标定可能不一致：主端 travel≈{implied:.1f} mm，"
                              f"本端 {mine:.1f} mm（偏差 {dev * 100:.0f}%）"
                              " —— 运动幅度会被按比例缩放")
            self._log(f"⚠ {self._mismatch}")

    # ────────────────────── 辅助 ──────────────────────

    def _kp(self) -> float:
        """增益：显式给的优先，否则用 SDK 装好标定后的 `cfg.kp`。

        ⚠ 参考实现的兜底是硬编码 `100.0`（`:319`），恰好等于 SDK 默认 `kp`
        （`models.py:83`）⇒ 正常情况等价；用 `cfg.kp` 还能带上标定文件里的值。
        """
        return float(self.kp) if self.kp is not None else float(self._cfg.kp)

    def _kd(self) -> float:
        return float(self.kd) if self.kd is not None else float(self._cfg.kd)

    def _hz_reset(self) -> None:
        self._loops = 0
        self._loop_hz = 0.0
        self._hz_t0 = 0.0
        self._hz_n0 = 0

    def _pace(self, t0: float, nxt: float, dt: float) -> float:
        """实测环频（每 1 s 更新一次）+ 定拍。返回下一个应醒时刻。"""
        self._loops += 1
        now = time.monotonic()
        if self._hz_t0 == 0.0:
            self._hz_t0, self._hz_n0 = now, self._loops
        elif now - self._hz_t0 >= 1.0:
            self._loop_hz = (self._loops - self._hz_n0) / (now - self._hz_t0)
            self._hz_t0, self._hz_n0 = now, self._loops
        r = nxt - now
        if r > 0:
            time.sleep(r)
        nxt += dt
        if nxt < time.monotonic():
            nxt = time.monotonic() + dt
        return nxt

    def _teardown(self) -> None:
        """收尾顺序（spec §8 rule 5）：交接持位 → 关 zenoh → 断 CAN（**不失能**）。"""
        self._want = False
        for ep in (self._pub, self._sub):
            if ep is not None:
                try:
                    ep.close()                   # ⛔ 不 close ⇒ 进程永久挂死
                except Exception:                # noqa: BLE001
                    log.exception("close zenoh 端点失败")
        self._pub = self._sub = None
        if self._grip is not None:
            try:
                # ⚠ 构造时传了 disable_on_disconnect=False ⇒ 这一步**不失能**
                self._grip.disconnect()
            except Exception:                    # noqa: BLE001
                log.exception("close 夹爪失败")
        with self._lock:
            self._snap.connected = False
        self._publish_state()
```

- [ ] **Step 3: 写非端到端用例 `tests/test_grip_worker.py`**

```python
"""`GripWorker` —— 换算 / 前置 / 收尾 / SDK 钉死 / 失配诊断。

（端到端那几条在 Task 3 追加。）
"""
import socket
import sys
import time
import types

import pytest

from liteteleop import grip_worker as gw
from liteteleop.grip_worker import (ROLE_MASTER, ROLE_SLAVE, GripNotReady,
                                    GripWorker, check_ready,
                                    clamp_to_calibrated, mm_to_openness,
                                    openness_to_rad, travel_mm_of)
from tests.fake_grip import NORMAL, REVERSE, FakeCfg, FakeGrip


def _free_port() -> int:
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


# ════════════════════ §6.1 换算：close_sign 回归网 ════════════════════

@pytest.mark.parametrize("mount,cfg", [
    ("normal", FakeCfg(**NORMAL)),
    ("reverse", FakeCfg(**REVERSE)),
])
def test_openness_maps_onto_both_calibrated_limits(mount, cfg):
    """`openness=0` → 闭合限位；`openness=1` → 张开限位。**两种装法都必须成立**。

    ⚠ 反装那组在**漏掉 `close_sign`** 时必红 —— 那正是参考实现
    （`gripper_teleop.py:155-163`）的真实缺陷，这条就是它的回归网。
    """
    assert openness_to_rad(cfg, 0.0) == pytest.approx(cfg.pos_closed_rad)
    assert openness_to_rad(cfg, 1.0) == pytest.approx(cfg.pos_open_rad)


def test_travel_mm_matches_sdk_template():
    """正装模板 `1.605 rad × 74.8` ≈ 120.05 mm，与 `max_stroke_mm=120` 吻合。"""
    assert travel_mm_of(FakeCfg(**NORMAL)) == pytest.approx(120.054, abs=0.01)


def test_mm_to_openness_inverts_openness_to_rad():
    """读侧（SDK 的 mm）与写侧（我们的 rad）必须互逆。"""
    cfg = FakeCfg(**NORMAL)
    for o in (0.0, 0.2, 0.5, 0.8, 1.0):
        q = openness_to_rad(cfg, o)
        mm = (cfg.pos_closed_rad - q) * cfg.close_sign * cfg.rad_to_mm
        assert mm_to_openness(cfg, mm) == pytest.approx(o)


def test_clamp_to_calibrated_handles_both_mounts():
    n, r = FakeCfg(**NORMAL), FakeCfg(**REVERSE)
    assert clamp_to_calibrated(n, 99.0) == pytest.approx(n.pos_closed_rad)
    assert clamp_to_calibrated(n, -99.0) == pytest.approx(n.pos_open_rad)
    assert clamp_to_calibrated(r, 99.0) == pytest.approx(r.pos_open_rad)
    assert clamp_to_calibrated(r, -99.0) == pytest.approx(r.pos_closed_rad)


# ════════════════════ §8.1 前置：拒启动 ════════════════════

def test_check_ready_rejects_uncalibrated():
    with pytest.raises(GripNotReady, match="未标定"):
        check_ready(FakeCfg(calibrated=False))


def test_check_ready_rejects_zero_travel():
    with pytest.raises(GripNotReady, match="行程为零"):
        check_ready(FakeCfg(pos_closed_rad=0.5, pos_open_rad=0.5))


def test_check_ready_rejects_zero_rad_to_mm():
    with pytest.raises(GripNotReady, match="rad_to_mm"):
        check_ready(FakeCfg(rad_to_mm=0.0))


def test_worker_refuses_before_enabling():
    """⚠ 断言 `enable()` **一次都没被调用** —— 未标定的夹爪不该先上一次电再被拒。"""
    g = FakeGrip(calibrated=False)
    w = GripWorker(ROLE_MASTER, "can0", gripper_factory=lambda _c: g)
    try:
        w.start()
        time.sleep(0.3)
    finally:
        w.stop(timeout=5.0)
    assert g.enable_calls == 0, "拒绝启动必须发生在 enable() 之前"
    assert "未标定" in w.snapshot().error


# ════════════════════ §8 rule 6：收尾不失能 ════════════════════

def test_factory_keeps_motor_enabled_on_disconnect(monkeypatch):
    """⚠⚠ `_default_gripper_factory` 必须显式传 `disable_on_disconnect=False`。

    `LiteGrip.__init__` 的默认值是 **True**（`gripper.py:205`）—— 不传的话
    `disconnect()` 会走 `self._can.disconnect(disable=True)` → `self.disable()`
    （`protocols/can_bus.py:92`）⇒ **掉力、松开**，让收尾那帧持位帧白做。
    **删掉这个 kwarg 时本用例必红。**

    ⚠ 断言打在**构造参数**上：本设计里 worker 是直接 `g.disconnect()` 的，
    行为在**构造时**就已决定（不是 litearm-device 那种 adapter 加锁层）。
    """
    seen = {}

    class _Recorder:
        def __init__(self, **kw):
            seen.update(kw)

    mod = types.ModuleType("litegrip")
    mod.LiteGrip = _Recorder
    monkeypatch.setitem(sys.modules, "litegrip", mod)
    gw._default_gripper_factory("can0")
    assert seen.get("channel") == "can0"
    assert seen.get("disable_on_disconnect") is False


def test_master_handoff_exits_zero_gravity():
    """主端停下时必须退零重力（那一步本身就是一帧按当前位的持位帧）。"""
    g = FakeGrip()
    w = GripWorker(ROLE_MASTER, "can0", gripper_factory=lambda _c: g)
    try:
        w.start()
        time.sleep(0.3)
        w.set_teleop(True)
        time.sleep(0.4)
    finally:
        w.stop(timeout=5.0)
    assert g.zero_gravity_exits >= 1
    assert g.disconnects >= 1, "收尾必须 disconnect（否则进程可能挂死）"


# ════════════════════ §9.4 SDK 钉死断言 ════════════════════

def test_sdk_pin_accepts_the_target_and_rejects_the_other_copy():
    """⚠ 断言必须有 —— 只 `sys.path.insert` 压不住未来改成 insert 式注册的
    editable MetaPathFinder，那时 import 会**静默**落到 gitee 那份。

    本用例不需要真的装 litegrip（CI 上也没有）⇒ 直接喂路径判据。
    """
    ok = "/home/llx/litegrip-python/src/litegrip/__init__.py"
    bad = "/home/llx/moduangongju/lite-grip/litegrip/__init__.py"
    gw.assert_sdk_pinned(ok)                     # 不抛
    with pytest.raises(SystemExit, match="另一份抢先了"):
        gw.assert_sdk_pinned(bad)


# ════════════════════ §7.3 主从标定一致性诊断 ════════════════════

def test_mismatch_warning_fires_on_different_travel():
    w = GripWorker(ROLE_SLAVE, "can1")
    w._cfg = FakeCfg(**NORMAL)                   # travel ≈ 120 mm
    w._check_mismatch(0.5, 30.0)                 # 主端 travel ≈ 60 mm
    assert "不一致" in w._mismatch


def test_mismatch_warns_once_only():
    w = GripWorker(ROLE_SLAVE, "can1")
    w._cfg = FakeCfg(**NORMAL)
    w._check_mismatch(0.5, 30.0)
    first = w._mismatch
    w._check_mismatch(0.5, 3.0)                  # 后来一致了也不撤回、不再叠加
    assert w._mismatch == first


def test_mismatch_window_excludes_clamped_openness():
    """⚠ `openness→1` 时比值会报出主端**原始** `position_mm`（大于其真实 travel）
    ⇒ 会误报「不一致」。饱和区必须排除。"""
    w = GripWorker(ROLE_SLAVE, "can1")
    w._cfg = FakeCfg(**NORMAL)
    w._check_mismatch(1.0, 400.0)
    assert not w._mismatch
    w._check_mismatch(0.05, 3.0)
    assert not w._mismatch
```

- [ ] **Step 4: 跑测试，确认通过**

Run: `python3 -m pytest tests/test_grip_worker.py -q`
Expected: `15 passed`

- [ ] **Step 5: 提交**

```bash
git add tests/fake_grip.py liteteleop/grip_worker.py tests/test_grip_worker.py
git commit -m "feat(grip): GripWorker 与臂侧完全解耦的夹爪遥操环"
```

---

## Task 3: 端到端用例（真 zenoh 回环）

**Files:**
- Modify: `tests/test_grip_worker.py`（**追加**，不改上面的内容）

- [ ] **Step 1: 追加端到端用例**

在 `tests/test_grip_worker.py` **末尾**追加：

```python
# ════════════════════ 两端环（真 zenoh 回环）════════════════════

def _pair(port: int, rate_hz: float = 100.0):
    mg, sg = FakeGrip(), FakeGrip()
    m = GripWorker(ROLE_MASTER, "can0", grip_id="gA", gport=port,
                   rate_hz=rate_hz, gripper_factory=lambda _c: mg)
    s = GripWorker(ROLE_SLAVE, "can1", grip_id="gA", gpeer="127.0.0.1", gport=port,
                   rate_hz=rate_hz, align=True, gripper_factory=lambda _c: sg)
    return m, s, mg, sg


def test_master_to_slave_follows():
    """主端手掰 → 从端跟到位（端到端，真 zenoh）。"""
    port = _free_port()
    m, s, mg, sg = _pair(port)
    try:
        m.start()
        time.sleep(0.3)
        m.set_teleop(True)
        time.sleep(0.5)
        s.start()
        time.sleep(0.3)
        s.set_teleop(True)
        time.sleep(1.5)
        assert s.snapshot().frames_received > 0, "从端一帧都没收到"

        mg.drag_to(openness_to_rad(mg.config, 0.5))
        deadline = time.monotonic() + 3.0
        want = openness_to_rad(sg.config, 0.5)
        while time.monotonic() < deadline:
            if sg.last_sent_q() is not None and abs(sg.last_sent_q() - want) < 0.05:
                break
            time.sleep(0.05)
        assert sg.last_sent_q() == pytest.approx(want, abs=0.05)
    finally:
        s.stop(timeout=5.0)
        m.stop(timeout=5.0)


def test_watchdog_holds_position_when_master_stops():
    """⚠ 主端停发 ⇒ 从端 **持位**：q 一动不动，但**继续发帧**（否则掉力）。"""
    port = _free_port()
    m, s, mg, sg = _pair(port)
    try:
        m.start()
        time.sleep(0.3)
        m.set_teleop(True)
        time.sleep(0.5)
        s.start()
        time.sleep(0.3)
        s.set_teleop(True)
        time.sleep(1.0)

        mg.drag_to(openness_to_rad(mg.config, 0.7))
        deadline = time.monotonic() + 3.0
        want = openness_to_rad(sg.config, 0.7)
        while time.monotonic() < deadline:
            if sg.last_sent_q() is not None and abs(sg.last_sent_q() - want) < 0.05:
                break
            time.sleep(0.05)
        held = sg.last_sent_q()
        assert held == pytest.approx(want, abs=0.05)

        m.set_teleop(False)                      # 主端停发
        time.sleep(0.4)                          # 等过 200 ms watchdog
        sg.clear_sent()
        time.sleep(0.5)

        assert s.snapshot().stale is True, "watchdog 应该已判 stale"
        assert s.snapshot().watchdog_trips >= 1
        assert len(sg.sent) > 0, "stale 之后仍必须继续发帧（MIT 模式停发会掉力）"
        assert all(f["q"] == pytest.approx(held, abs=1e-9) for f in sg.sent), \
            "持位：q 必须在整段 stale 期间一动不动"
        assert all(f["kp"] > 0 for f in sg.sent), "持位帧必须带刚度，不能是零力矩帧"
    finally:
        s.stop(timeout=5.0)
        m.stop(timeout=5.0)


def test_slave_without_first_frame_sends_measured_position_not_zero():
    """⚠⚠ 首帧没到时若把 `q` 初始化成 `0.0`，那是一个**真实位置指令**
    （正装 `0.0 rad` = 全开）⇒ 可能直冲机械限位。
    入口必须取**本机实测位置** —— 这条就是那个兜底的回归网。"""
    port = _free_port()                          # ⚠ 故意没有主端在监听
    sg = FakeGrip(position_rad=-0.7)
    s = GripWorker(ROLE_SLAVE, "can1", grip_id="gA", gpeer="127.0.0.1", gport=port,
                   rate_hz=100.0, align=False, gripper_factory=lambda _c: sg)
    try:
        s.start()
        time.sleep(0.3)
        s.set_teleop(True)
        time.sleep(0.6)
        assert len(sg.sent) > 0, "没收到帧也必须持续发帧"
        assert sg.sent[0]["q"] == pytest.approx(-0.7), \
            "入口必须取本机实测位置（-0.7），不能是 0.0"
    finally:
        s.stop(timeout=5.0)


def test_align_moves_slave_toward_master_first_frame():
    """对齐：先 `goto_rad` 到首帧位置，再进高频跟随。"""
    port = _free_port()
    m, s, mg, sg = _pair(port)
    try:
        m.start()
        time.sleep(0.3)
        mg.drag_to(openness_to_rad(mg.config, 0.8))
        m.set_teleop(True)
        time.sleep(0.6)
        s.start()
        time.sleep(0.3)
        s.set_teleop(True)
        s._want = True                            # 已经在 _run 里
        deadline = time.monotonic() + 4.0
        while time.monotonic() < deadline and not sg.gotos:
            time.sleep(0.05)
        assert sg.gotos, "对齐应当调用 goto_rad"
        assert sg.gotos[0]["q"] == pytest.approx(openness_to_rad(sg.config, 0.8), abs=0.05)
    finally:
        s.stop(timeout=5.0)
        m.stop(timeout=5.0)
```

- [ ] **Step 2: 跑，确认通过**

Run: `python3 -m pytest tests/test_grip_worker.py -q`
Expected: `19 passed`（约 9 s；这几条起真 zenoh，慢是正常的）

- [ ] **Step 3: 提交**

```bash
git add tests/test_grip_worker.py
git commit -m "test(grip): 端到端回环 —— 跟随、watchdog 持位、首帧缺失兜底"
```

---

## Task 4: 配置与命令行

**Files:**
- Modify: `liteteleop/settings.py`
- Modify: `liteteleop/__main__.py`
- Test: `tests/test_grip_settings.py`

- [ ] **Step 1: 写失败测试**

创建 `tests/test_grip_settings.py`：

```python
"""夹爪那 4 个设置字段 —— 独立、且默认不启用（spec §9.1）。"""
from liteteleop.settings import Settings


def test_gripper_defaults_are_off_and_independent():
    s = Settings()
    assert s.gcan == "", "默认必须是「不启用」"
    assert s.gport == 17448
    assert s.grip_id == "gripA"
    assert s.gpeer == "127.0.0.1"


def test_gripper_fields_do_not_read_the_arm_fields():
    """⚠ spec §2 铁律第 3 条：夹爪字段**不读**臂的同名字段。

    改臂的 `peer` / `arm_id` 不得影响夹爪的。
    """
    s = Settings(peer="10.0.0.9", arm_id="armZ", jport=1234)
    assert s.gpeer == "127.0.0.1"
    assert s.grip_id == "gripA"
    assert s.gport == 17448
    assert s.gcan == ""
```

- [ ] **Step 2: 跑，确认失败**

Run: `python3 -m pytest tests/test_grip_settings.py -q`
Expected: FAIL —— `TypeError: __init__() got an unexpected keyword argument 'gcan'` 之类

- [ ] **Step 3: 改 `liteteleop/settings.py`**

在 `Settings` 的 `extra` 字段**之前**插入这 4 个字段：

```python
    # ── 夹爪遥操（与臂侧**完全独立**，spec §2 铁律第 3 条：不读臂的字段）──
    gcan: str = ""                       # 夹爪 CAN 通道；空 = 不启用夹爪遥操（默认关）
    gpeer: str = "127.0.0.1"             # 从端填：主端 IP（**独立字段**）
    gport: int = 17448                   # 夹爪 zenoh 端口（独立 session ⇒ 必须独立端口）
    grip_id: str = "gripA"               # 决定夹爪 topic
```

同时把模块 docstring 第 1 行改成：

```python
"""界面设置的持久化（角色 / 地址 / 端口 / arm_id / 夹爪）。"""
```

- [ ] **Step 4: 跑，确认通过**

Run: `python3 -m pytest tests/test_grip_settings.py -q`
Expected: `2 passed`

- [ ] **Step 5: 加命令行参数**

在 `liteteleop/__main__.py` 的 `ap.add_argument("--arm-id", ...)` **之后**插入：

```python
    ap.add_argument("--gcan", default=None, help="夹爪 CAN 通道（空=不启用夹爪遥操）")
    ap.add_argument("--gpeer", default=None, help="从臂夹爪填：主臂的 IP")
    ap.add_argument("--gport", type=int, default=None, help="夹爪 zenoh 端口")
    ap.add_argument("--grip-id", default=None, help="夹爪 topic 里的 grip_id（两端须一致）")
```

并在 `if a.arm_id: s.arm_id = a.arm_id` **之后**插入：

```python
    if a.gcan: s.gcan = a.gcan
    if a.gpeer: s.gpeer = a.gpeer
    if a.gport: s.gport = a.gport
    if a.grip_id: s.grip_id = a.grip_id
```

- [ ] **Step 6: 接上 SDK 钉死（spec §9.4）**

⚠ 只在**启用了夹爪**时才 import `litegrip` —— 不启用夹爪的机器不该因为缺这个包而起不来。

在 `liteteleop/__main__.py` 的 `s = load_settings()` 之后、`import sys as _sys` 之前插入：

```python
    if s.gcan:
        # ⚠⚠ 必须在任何人 import litegrip **之前**钉死，否则本机那份 editable 的
        #    gitee 克隆会被静默抢先（实测：裸 import 落在 moduangongju/lite-grip）。
        from .grip_worker import pin_grip_sdk
        print(f"夹爪 SDK: {pin_grip_sdk()}")
```

- [ ] **Step 7: 跑全套测试 + 手工验参数**

Run: `python3 -m pytest tests -q`
Expected: `26 passed`（5 + 19 + 2）

Run: `python3 -m liteteleop --help`
Expected: 输出里出现 `--gcan` / `--gpeer` / `--gport` / `--grip-id`

- [ ] **Step 8: 提交**

```bash
git add liteteleop/settings.py liteteleop/__main__.py tests/test_grip_settings.py
git commit -m "feat(grip): 夹爪的独立设置、命令行与 SDK 钉死接线"
```

---

## Task 5: 界面（夹爪分区 + 独立开关）

**Files:**
- Modify: `liteteleop/gui/bridge.py`
- Modify: `liteteleop/gui/pages.py`
- Modify: `liteteleop/gui/main_window.py`
- Test: `tests/test_gui_smoke.py`（追加）

- [ ] **Step 1: 加信号**

在 `liteteleop/gui/bridge.py` 的 `teleop_changed` 之后插入：

```python
    #: 夹爪快照（每来一帧发一次）。⚠ 与臂的 `state` 是**两条独立信号**
    grip_state = QtCore.pyqtSignal(object)
```

并把模块 docstring 的 `__all__` 行不动（`WorkerBridge` 没变）。

- [ ] **Step 2: 加夹爪分区**

在 `liteteleop/gui/pages.py` 的 `TeleopPage` 类里：

(a) 类体顶部加信号（在 `payload_applied` 之后）：

```python
    #: 夹爪分区：开/关、通道、端口、对齐、id —— 由 main_window 接到 GripWorker
    grip_toggled = QtCore.pyqtSignal(bool)
    grip_settings_changed = QtCore.pyqtSignal()
```

(b) `__init__` 的 `lay.addWidget(box)` 之后追加夹爪分区：

```python
        # ── 夹爪遥操（与臂侧**完全独立**：独立通道 / 独立端口 / 独立开关）──
        gbox = QtWidgets.QGroupBox("夹爪遥操（与臂遥操各自独立）")
        gl = QtWidgets.QVBoxLayout(gbox)

        grow = QtWidgets.QHBoxLayout()
        self.ed_gcan = QtWidgets.QLineEdit()
        self.ed_gcan.setPlaceholderText("can0")
        self.ed_gcan.setMaximumWidth(120)
        self.sp_gport = QtWidgets.QSpinBox()
        self.sp_gport.setRange(1, 65535)
        self.ed_grip_id = QtWidgets.QLineEdit()
        self.ed_grip_id.setMaximumWidth(120)
        self.ed_gpeer = QtWidgets.QLineEdit()
        grow.addWidget(QtWidgets.QLabel("CAN"))
        grow.addWidget(self.ed_gcan)
        grow.addSpacing(10)
        grow.addWidget(QtWidgets.QLabel("端口"))
        grow.addWidget(self.sp_gport)
        grow.addSpacing(10)
        grow.addWidget(QtWidgets.QLabel("grip_id"))
        grow.addWidget(self.ed_grip_id)
        grow.addSpacing(10)
        grow.addWidget(QtWidgets.QLabel("主端 IP"))
        grow.addWidget(self.ed_gpeer)
        gl.addLayout(grow)

        self.btn_grip = QtWidgets.QPushButton("启动夹爪遥操")
        self.btn_grip.setCheckable(True)
        self.chk_align = QtWidgets.QCheckBox("启动时对齐")
        self.chk_align.setChecked(True)
        grow2 = QtWidgets.QHBoxLayout()
        grow2.addWidget(self.btn_grip)
        grow2.addWidget(self.chk_align)
        grow2.addStretch(1)
        gl.addLayout(grow2)

        self.lab_grip = QtWidgets.QLabel("未启动")
        self.lab_grip.setWordWrap(True)
        gl.addWidget(self.lab_grip)

        self.lab_grip_mismatch = QtWidgets.QLabel("")
        self.lab_grip_mismatch.setWordWrap(True)
        self.lab_grip_mismatch.setStyleSheet("color:#b02020;")
        gl.addWidget(self.lab_grip_mismatch)
        lay.addWidget(gbox)
```

(c) `__init__` 末尾（`lay.addWidget(box)` 之后）接线：

```python
        # ⚠⚠ 与臂的 `btn_teleop` 同一纪律：`clicked` 发出时状态**已经切换**，
        #    `isChecked()` **就是**用户想要的新值 —— 不要加 `not`。
        self.btn_grip.clicked.connect(
            lambda: self.grip_toggled.emit(self.btn_grip.isChecked()))
        for w in (self.ed_gcan, self.ed_grip_id, self.ed_gpeer):
            w.editingFinished.connect(self.grip_settings_changed.emit)
        self.sp_gport.valueChanged.connect(self.grip_settings_changed.emit)
        self.chk_align.toggled.connect(self.grip_settings_changed.emit)
```

(d) 加两个方法（放在 `_apply_payload` 之后）：

```python
    def gripper_values(self) -> dict:
        """把夹爪分区的当前值读出来（含「未填通道 = 不启用」的安全默认）。"""
        return {
            "gcan": self.ed_gcan.text().strip(),
            "gpeer": self.ed_gpeer.text().strip() or "127.0.0.1",
            "gport": int(self.sp_gport.value()),
            "grip_id": self.ed_grip_id.text().strip() or "gripA",
            "align": bool(self.chk_align.isChecked()),
        }

    def apply_grip(self, g) -> None:
        """夹爪快照 → 界面。⚠ 与 `apply()` 分开：夹爪状态由**另一条**信号送来
        （`WorkerBridge.grip_state`），两条链路不共享任何对象。"""
        if g.error:
            self.lab_grip.setText(f"⛔ {g.error}")
        elif not g.connected:
            self.lab_grip.setText("未启动")
        elif g.role == "master":
            self.lab_grip.setText(
                f"主端夹爪：零重力 · {g.topic} · 发 {g.frames_sent} 帧 · "
                f"开合 {g.openness:.2f} · {g.position_mm:.1f} mm · "
                + ("已匹配订阅者" if g.matching else "⚠ 未匹配（发了没人在收）"))
        else:
            age = "—" if g.frame_age is None else f"{g.frame_age * 1000:.0f} ms"
            self.lab_grip.setText(
                f"从端夹爪：{'持位（watchdog 超时）' if g.stale else '跟随中'} · "
                f"环频 {g.loop_hz:.0f} Hz · 收 {g.frames_received} 帧 · 帧龄 {age} · "
                f"开合 {g.openness:.2f} · {g.position_mm:.1f} mm")
        self.lab_grip_mismatch.setText(g.mismatch)
        # ⚠ 按钮文本/勾选**按真实状态**刷新，不是按点击 —— 与 `apply()` 对
        #    `btn_teleop` 的做法逐字同款（`pages.py:143-144`）
        self.btn_grip.setEnabled(bool(g.connected))
        self.btn_grip.setText("停止夹爪遥操" if g.teleop_active else "启动夹爪遥操")
        self.btn_grip.setChecked(bool(g.teleop_active))
```

(e) `GripSnapshot` 加一个字段（`liteteleop/grip_worker.py`）：

```python
    teleop_active: bool = False         # 由 snapshot() 从 _want 填
```

并在 `GripWorker.snapshot()` 的 `with self._lock:` 块里补一行：

```python
            s.teleop_active = bool(self._want)
```

- [ ] **Step 3: 接线到主窗口**

在 `liteteleop/gui/main_window.py`：

(a) import 行改成：

```python
from ..grip_worker import GripSnapshot, GripWorker
```

(b) `__init__` 里 `self.worker: ArmWorker | None = None` 之后加：

```python
        self.grip: GripWorker | None = None
```

(c) 顶部信号接线区（`self.bridge.log.connect(self._log)` 之后）加：

```python
        self.bridge.grip_state.connect(self._on_grip_state)
```

(d) 构造 `TeleopPage()` 之后、接线区里加：

```python
        self.page_teleop.grip_toggled.connect(self._toggle_grip)
        self.page_teleop.grip_settings_changed.connect(self._save_grip)
```

(e) 加方法（放在 `_apply_payload` 之后）：

```python
    def _save_grip(self) -> None:
        v = self.page_teleop.gripper_values()
        self.s.gcan = v["gcan"]
        self.s.gpeer = v["gpeer"]
        self.s.gport = v["gport"]
        self.s.grip_id = v["grip_id"]
        self._save()

    def _ensure_grip_worker(self) -> GripWorker | None:
        """按需建 `GripWorker`。⚠ 与 `ArmWorker` **完全独立**（spec §2）。"""
        if self.grip is not None:
            return self.grip
        v = self.page_teleop.gripper_values()
        if not v["gcan"]:
            self._log("⚠ 未填夹爪 CAN 通道 —— 夹爪遥操未启用")
            return None
        self._save_grip()
        self.grip = GripWorker(
            role=self.page_link.role(), gcan=v["gcan"], grip_id=v["grip_id"],
            gpeer=v["gpeer"], gport=v["gport"], align=v["align"],
            on_state=self.bridge.grip_state.emit, on_log=self.bridge.on_log)
        try:
            from ..grip_worker import pin_grip_sdk
            self._log(f"夹爪 SDK: {pin_grip_sdk()}")
        except SystemExit as e:
            self._log(str(e))
            self.grip = None
            return None
        self.grip.start()
        return self.grip

    def _toggle_grip(self, on: bool) -> None:
        if not on:
            if self.grip is not None:
                self._log("停止夹爪遥操（夹爪保持当前位置，不失能）")
                self.grip.set_teleop(False)
            return
        w = self._ensure_grip_worker()
        if w is None:
            self.page_teleop.btn_grip.setChecked(False)
            return
        self._log("启动夹爪遥操")
        w.set_teleop(True)

    def _on_grip_state(self, g: GripSnapshot) -> None:
        self.page_teleop.apply_grip(g)
```

(f) `closeEvent` 里 `self.worker.shutdown()` 之后加：

```python
        if self.grip is not None:
            self.grip.stop()
```

- [ ] **Step 4: 追加冒烟用例**

在 `tests/test_gui_smoke.py` 末尾追加（沿用该文件既有的 `qapp` fixture）：

```python
def test_grip_panel_defaults_are_off(qapp):
    """⚠ 夹爪分区默认**不启用**（通道空），且开关与臂的开关是两个独立控件。"""
    from liteteleop.gui.pages import TeleopPage

    p = TeleopPage()
    v = p.gripper_values()
    assert v["gcan"] == "", "默认必须不启用"
    assert p.chk_align.isChecked() is True
    assert p.btn_grip is not p.btn_teleop
```

- [ ] **Step 5: 跑，确认通过**

Run: `python3 -m pytest tests -q`
Expected: `27 passed`

- [ ] **Step 6: 提交**

```bash
git add liteteleop/gui/bridge.py liteteleop/gui/pages.py liteteleop/gui/main_window.py tests/test_gui_smoke.py
git commit -m "feat(grip): 界面夹爪分区与独立开关"
```

---

## Task 6: README

**Files:**
- Modify: `README.md`

- [ ] **Step 1: 更新用法与状态**

在 README 的启动示例里补上夹爪的两个进程，并订正已过时的「GUI not in yet」状态行（GUI 早已在 `liteteleop/gui/`）：

```bash
# 臂 + 夹爪，主端
python -m liteteleop --role master --cdc /dev/ttyACM0 --gcan can0
# 臂 + 夹爪，从端
python -m liteteleop --role slave  --cdc /dev/ttyACM1 --peer 192.168.31.10 \
                     --gcan can1 --gpeer 192.168.31.10
```

并加一段说明：夹爪遥操与臂遥操**各自独立**（独立 CAN、独立 zenoh session、独立端口 17448、独立开关）。

- [ ] **Step 2: lint（CI 会跑）**

Run: `npx --yes markdownlint-cli2@0.23.3 "README.md"`
Expected: `0 issues`（⚠ CI 只 lint `README.md`，见 `.github/workflows/ci.yml:52`）

- [ ] **Step 3: 提交**

```bash
git add README.md
git commit -m "docs: README 补夹爪遥操用法，订正过时的 GUI 状态"
```

---

## Task 7: 对拍与变异复核（**验收闸门，不可跳过**）

**Files:** 无（只跑命令）

- [ ] **Step 1: 线契约对拍（spec §11.2.1）**

Run:

```bash
diff <(sed -n '44,75p' /home/llx/litearm-device/src/litearm_device/gripper_teleop.py) \
     <(sed -n '/^GRIP_FORMAT/,/^GRIP_FRAME_BYTES/p' liteteleop/grip_wire.py)
```

Expected: 格式串 `">4d"`、`FRAME_SIZE`/`GRIP_FRAME_BYTES` = 32、字段序、topic 模板四项对上。
输出不会逐字相同（注释不同），**逐项核这四项**即可。

- [ ] **Step 2: 换算对拍（spec §11.2.2）**

Run:

```bash
python3 -c "
import sys; sys.path.insert(0,'/home/llx/litegrip-python/src')
sys.path.insert(0,'.')
from litegrip.models import GripperConfig
from liteteleop.grip_worker import openness_to_rad
for name, kw in (('normal', dict(pos_closed_rad=0.114,pos_open_rad=-1.491)),
                 ('reverse', dict(pos_closed_rad=-1.491,pos_open_rad=0.114))):
    c = GripperConfig(rad_to_mm=74.8, calibrated=True, **kw)
    for o in (0.0, 0.25, 0.5, 0.75, 1.0):
        ours = openness_to_rad(c, o)
        # SDK 的 goto 公式逐字：rad = pos_closed - close_sign*mm/rad_to_mm
        sdk = c.pos_closed_rad - c.close_sign*(o*abs(c.pos_open_rad-c.pos_closed_rad)*c.rad_to_mm)/c.rad_to_mm
        assert abs(ours-sdk) < 1e-12, (name, o, ours, sdk)
        print(f'{name:8s} o={o:.2f}  ours={ours:+.6f}  sdk={sdk:+.6f}  ✓')
"
```

Expected: 10 行全 `✓`，两种装法都对上。

- [ ] **Step 3: 解耦对拍（spec §11.2.3）**

Run:

```bash
echo '--- arm_worker 是否引用夹爪模块（应为空）---'
grep -n "grip_worker\|grip_wire\|GripWorker" liteteleop/arm_worker.py || echo "✓ 无引用"
echo '--- grip_worker 除 link/grip_wire 外是否引用臂侧模块（应为空）---'
grep -n "arm_worker\|servo\|safety\|wall\|from .wire\|from .ports" liteteleop/grip_worker.py || echo "✓ 无引用"
```

Expected: 两条都输出 `✓ 无引用`

- [ ] **Step 4: 逻辑对拍（spec §11.2.4）**

逐项核 spec §7.4 的表：`_master_loop`/`_slave_loop` 与 §7.1/§7.2 一致，
**4 处「有意偏离」之外没有别的差异**（已裁决全部保留）。

- [ ] **Step 5: 变异复核（判据有判别力）**

⚠ 本计划的代码已在 `/tmp/grip_proto` 通过这 5 组变异验证。
在这里**再跑一遍**，确认落到本仓后判据依然有效 —— 每条都要**红**：

```bash
cp liteteleop/grip_worker.py /tmp/gw.plan.bak
m() { python3 -c "$1" && python3 -m pytest tests/test_grip_worker.py -q 2>&1 | tail -1; cp /tmp/gw.plan.bak liteteleop/grip_worker.py; }

echo "① 去掉 close_sign（应只红反装那组）"
m "p='liteteleop/grip_worker.py';s=open(p).read();o='- float(cfg.close_sign) * (_clamp01(openness) * t) / float(cfg.rad_to_mm)';assert o in s;open(p,'w').write(s.replace(o,'- (_clamp01(openness) * t) / float(cfg.rad_to_mm)'))"

echo "② 去掉 disable_on_disconnect=False"
m "p='liteteleop/grip_worker.py';s=open(p).read();o='return litegrip.LiteGrip(channel=channel, disable_on_disconnect=False)';assert o in s;open(p,'w').write(s.replace(o,'return litegrip.LiteGrip(channel=channel)'))"

echo "③ 从端 q_cmd 初值改 0.0"
m "p='liteteleop/grip_worker.py';s=open(p).read();o='q_cmd = clamp_to_calibrated(self._cfg, g.get_state(wait=False).position_rad)';assert o in s;open(p,'w').write(s.replace(o,'q_cmd = 0.0'))"

echo "④ 失配窗口放宽到 [0,1]"
m "p='liteteleop/grip_worker.py';s=open(p).read();o='_MISMATCH_WINDOW = (0.15, 0.85)';assert o in s;open(p,'w').write(s.replace(o,'_MISMATCH_WINDOW = (0.0, 1.0)'))"

echo "⑤ stale 时停发帧"
m "p='liteteleop/grip_worker.py';s=open(p).read();o='            q_cmd = clamp_to_calibrated(self._cfg, q_cmd)\n            g.send_mit_frame(q=q_cmd, kp=self._kp(), kd=self._kd(), dq=0.0)';n='            q_cmd = clamp_to_calibrated(self._cfg, q_cmd)\n            if not stale:\n                g.send_mit_frame(q=q_cmd, kp=self._kp(), kd=self._kd(), dq=0.0)';assert o in s;open(p,'w').write(s.replace(o,n))"

echo "── 恢复后确认全绿 ──"
python3 -m pytest tests -q 2>&1 | tail -1
```

Expected: ①②③⑤ 各 `1 failed`；④ 至少 `1 failed`；最后 `27 passed`。

---

## Task 8: 真机验证（spec §11.3）

⚠ **需要两台机 + 两个夹爪 + 已拉的 CAN**。在每台机上：

```bash
sudo ip link set can0 type can bitrate 1000000 && sudo ip link set can0 up
```

- [ ] **Step 1: 主端手掰、从端跟随**（正装夹爪）
- [ ] **Step 2: 拔网线 / 杀主端** ⇒ 从端**持位**：不掉力、不松开、不自动重连
- [ ] **Step 3: 未标定的夹爪** ⇒ 启动即拒绝，且**电机没被使能过**（日志里 `enable()` 之前的报错）
- [ ] **Step 4: 收尾后夹爪仍夹着**（spec §8 rule 6 的现场验证）+ 进程**正常退出**
- [ ] **Step 5: 启动日志里的夹爪 SDK 路径是 `/home/llx/litegrip-python/src/litegrip/…`**，不是 `moduangongju/lite-grip/…`。**两台机都要看这一行** —— 这是唯一能区分「跑的是哪份 SDK」的一手证据
- [ ] **Step 6: 确认两台机装的是哪份标定**（`travel_mm` 会打进启动日志）。⚠ 记忆里 gitee 那份有「87mm 口径」的改动，与 `litegrip-python` 的 120mm 模板不是一套；设计本身不受影响（`travel_mm` 永远取运行时实际加载的标定），但主从差异过大会触发 §7.3 的告警

---

## 收尾

- [ ] **确认没碰冻结文件**：`git diff --stat main...HEAD` 里不应出现 `link.py` / `arm_worker.py` / `servo.py` / `safety.py` / `wall.py` / `wire.py` / `ports.py`
- [ ] **不主动 push**（用户规矩）。要推时先说。
- [ ] 更新 `README.md` 的「已知陷阱」表（若适用）
