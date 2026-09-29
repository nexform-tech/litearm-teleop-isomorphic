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
