"""遥操线协议 —— **逐字照搬 litearm-server**（`teleop_manager.py:19-45`）。

⛔ 本模块**不是**新设计。litearm-server 的遥操是经过真机验证的实现，用户裁决
「必须严格按照 litearm-server 来写」。所以这里连**函数名、格式串、topic 模板**
都保持一致，以便逐行对账（`diff` 得出来）。

布局（**大端**，定长）::

    struct ">15d"  =  7q + 7dq + 1 timestamp  =  120 字节
    q[i]        f64 BE   主臂实测关节角 rad
    dq[i]       f64 BE   主臂实测关节速度 rad/s
    timestamp   f64 BE   主臂的时基（同一主臂相邻帧可比；跨机不可比）

⚠ **大端、f64、定长 120 B、关节数写死 7** —— 与 litearm-server 完全一致。
   早先本仓用的是「小端 f32 + 变长 n + 版本字节」的 70 B 帧，那是**自创**的，
   已按用户裁决废弃。
⚠ `timestamp` 只能算**同一主臂**相邻两帧的间隔，**不能**算端到端延迟（跨机时钟不同源）。
"""
from __future__ import annotations

import struct
from typing import Dict, List, Sequence

__all__ = ["N_JOINTS", "TELEOP_FORMAT", "TELEOP_FRAME_BYTES", "teleop_topic",
           "encode_teleop", "decode_teleop"]


#: 关节数。litearm-server 的 `encode_teleop` 写死 7（`if len(q) != 7: raise`），
#: 这里照做 —— 本臂就是 7 关节，不预留变长。
N_JOINTS = 7

#: 与 `teleop_manager._TELEOP_FORMAT` 逐字相同。
TELEOP_FORMAT = ">15d"

#: 帧长（字节）。`struct.calcsize(">15d") == 120`。
TELEOP_FRAME_BYTES = struct.calcsize(TELEOP_FORMAT)


def teleop_topic(arm_id: str) -> str:
    """遥操 zenoh topic。Master 发布，Slave 订阅（用 Master 的 arm_id）。

    与 `teleop_manager.teleop_topic` 逐字相同 —— 帧格式也已对齐，所以两端
    **可以**互通（这是刻意的：格式不一致时共用 topic 才会静默解错）。
    """
    return f"litearm/v4/{arm_id}/teleop"


def encode_teleop(q: Sequence[float], dq: Sequence[float],
                  timestamp: float) -> bytes:
    """编码遥操数据帧。`q`/`dq` 各 7 元素，`timestamp` 为主臂时基。

    与 `teleop_manager.encode_teleop` 逐字相同（含那两条 `ValueError`）。
    """
    if len(q) != N_JOINTS:
        raise ValueError(f"q 必须是 {N_JOINTS} 元素，收到 {len(q)}")
    if len(dq) != N_JOINTS:
        raise ValueError(f"dq 必须是 {N_JOINTS} 元素，收到 {len(dq)}")
    return struct.pack(TELEOP_FORMAT, *q, *dq, timestamp)


def decode_teleop(payload: bytes) -> Dict[str, object]:
    """解码遥操数据帧 → ``{"q": [...7], "dq": [...7], "timestamp": float}``。

    与 `teleop_manager.decode_teleop` 逐字相同。⚠ 帧长不对时 `struct.unpack`
    抛 `struct.error` —— **这是刻意的**，与 server 行为一致：不做"尽量解"的兜底，
    解不出来就是解不出来。
    """
    values = struct.unpack(TELEOP_FORMAT, payload)
    return {
        "q": list(values[0:N_JOINTS]),
        "dq": list(values[N_JOINTS:2 * N_JOINTS]),
        "timestamp": values[2 * N_JOINTS],
    }


def check_frame_size(payload: bytes) -> List[float]:  # pragma: no cover - 便利函数
    """调试用：帧长是否等于 120。返回 `[]` 表示对，否则返回一句人话。"""
    if len(payload) != TELEOP_FRAME_BYTES:
        return [f"帧长 {len(payload)} B ≠ {TELEOP_FRAME_BYTES} B"]
    return []
