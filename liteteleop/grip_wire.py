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
