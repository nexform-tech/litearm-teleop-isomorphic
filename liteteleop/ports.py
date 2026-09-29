"""CDC 口的枚举与选择。

## ⚠⚠ 为什么必须按**序列号**选，不能按 VID:PID

同一型号的两条臂 **VID/PID 完全相同**（`1d50:606f`），`ttyACM0`/`ttyACM1` 的编号
**还会互换**。实测（2026-09-28，本机接两条臂）：

    ttyACM1  serial=3244386E3233      ← 后插的
    ttyACM0  serial=355035373333      ← 先插的
    SDK 的 find_cdc_port() -> /dev/ttyACM1

`find_cdc_port()` 取**第一个匹配**，于是**主臂进程和从臂进程会抢到同一个口，
而且不报任何错** —— 这正是"静默失败"的教科书形态。

⇒ 本模块的规矩：**恰好一条臂时才允许自动选；两条及以上必须显式指定**，
   拿不准就**抛**。（同 spec §7.1 的精神：宁可拒启动，不要静默退化。）
"""
from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import List, Optional

log = logging.getLogger("liteteleop.ports")

__all__ = ["ArmPort", "list_arms", "resolve_port", "PortAmbiguous"]

_STM32_VID = 0x1D50
_STM32_PID = 0x606F


class PortAmbiguous(RuntimeError):
    """接了两条及以上同型号臂，必须显式指定哪一条。"""


@dataclass(frozen=True)
class ArmPort:
    device: str                 # /dev/ttyACMx
    serial: str = ""            # STM32 唯一序列号
    description: str = ""

    @property
    def label(self) -> str:
        """界面显示用 —— **序列号才是唯一标识**，别让用户按 ttyACM 编号认。"""
        if self.serial:
            return f"{self.device}  (序列号 {self.serial})"
        return self.device


def list_arms() -> List[ArmPort]:
    """列出所有 STM32 CDC 口（按设备名排序，保证顺序稳定）。"""
    try:
        from serial.tools import list_ports
    except Exception as e:                                # noqa: BLE001
        log.warning("pyserial 不可用: %s", e)
        return []
    out = [
        ArmPort(device=p.device, serial=p.serial_number or "",
                description=p.description or "")
        for p in list_ports.comports()
        if p.vid == _STM32_VID and p.pid == _STM32_PID
    ]
    return sorted(out, key=lambda a: a.device)


def resolve_port(explicit: Optional[str] = None) -> str:
    """定下用哪个口。

    - 显式给了 `explicit` ⇒ 原样用它（并只做"存不存在"的提示，不拦）
    - 没给且**恰好一条臂** ⇒ 自动选它
    - 没给且**两条及以上** ⇒ 抛 `PortAmbiguous`（**绝不静默挑一个**）
    - 没给且**一条都没有** ⇒ 抛 `RuntimeError`
    """
    if explicit:
        return explicit
    arms = list_arms()
    if len(arms) == 1:
        return arms[0].device
    if not arms:
        raise RuntimeError("找不到 STM32 CDC 口（VID:PID 1d50:606f）—— 检查 USB 与权限")
    raise PortAmbiguous(
        f"检测到 {len(arms)} 条臂，**必须显式指定用哪一条**：\n"
        + "\n".join(f"  · {a.label}" for a in arms)
        + "\n⚠ 同型号的两条臂 VID:PID 相同，自动挑会挑错而且不报错 —— "
          "在界面上选一个 CDC 口，或把 cdc_port 写进设置。")
