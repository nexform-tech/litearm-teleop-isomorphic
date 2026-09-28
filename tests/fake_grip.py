"""本仓自带的极简夹爪替身 —— 让 `GripWorker` 的用例不依赖外部仓、也不碰硬件。

⚠ **不复用 `litegrip-python/tests/fake_can.py`**：CI 只装 `pytest` 与 `zenoh`
（`.github/workflows/ci.yml`），**不装 litegrip**；且那份第 14 行有 `import _sdkpath`
（要它自己的目录在 `sys.path` 上），而本仓 `tests/` 是包、litegrip 的 `tests/` 不是
⇒ 在 CI 上起不来（本机已实测）。那份**只用于 §11.2 的对拍**（本机手工跑）。

⚠⚠ 本替身**故意没有 `disable()` 方法** —— 任何"顺手失能"的代码路径会当场
`AttributeError`，这本身就是一道结构性护栏（spec §8 rule 4）。

⚠⚠ `disconnects` 记录的是**每次 `disconnect()` 调用**。要断言
「收尾不失能」不能看这里 —— 该断言打在**构造参数**上（见 `test_grip_worker.py`
的 `test_factory_keeps_motor_enabled_on_disconnect`），因为本设计里 worker 是
直接 `g.disconnect()` 的，`disable_on_disconnect` 在**构造时**决定行为。
（litegrip 那份 `fake_can.py` 的 `disconnect(self, disable=True)` 函数体
只有 `self.disconnected = True`、**忽略 `disable`** ⇒ 拿它断言是零判别力的。）
"""
from __future__ import annotations

from dataclasses import dataclass, field
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
    """鸭子类型的夹爪替身。**记录每一次调用**，供断言。

    Args:
        cfg: 标定配置；缺省用正装模板。
        position_rad: 起始电机弧度；缺省取闭合限位。
        calibrated: `load_calibration()` 之后 `cfg.calibrated` 的值。
    """

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
        """⚠ 记录调用次数。**真货的 `disable_on_disconnect` 语义在构造参数上**，
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
