"""遥操编解码单元测试 —— **照搬 litearm-server 的 `tests/test_teleop_codec.py`**。

用户裁决「必须严格按照 litearm-server 来写」，所以连用例也搬过来：
格式（`>15d` / 120 B / 大端）、往返、错误路径、topic 模板逐条对齐。
"""
import struct

import pytest

from liteteleop.wire import (
    TELEOP_FORMAT,
    TELEOP_FRAME_BYTES,
    decode_teleop,
    encode_teleop,
    teleop_topic,
)


class TestTeleopCodec:
    def test_roundtrip(self):
        q = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]
        dq = [1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6]
        ts = 12.345
        frame = decode_teleop(encode_teleop(q, dq, ts))
        assert frame["q"] == pytest.approx(q)
        assert frame["dq"] == pytest.approx(dq)
        assert frame["timestamp"] == pytest.approx(ts)

    def test_frame_is_120_bytes(self):
        payload = encode_teleop([0.0] * 7, [0.0] * 7, 0.0)
        assert len(payload) == 120
        assert TELEOP_FRAME_BYTES == 120

    def test_format_is_big_endian_15_doubles(self):
        assert TELEOP_FORMAT == ">15d"
        assert struct.calcsize(TELEOP_FORMAT) == 120

    def test_decode_returns_plain_lists(self):
        frame = decode_teleop(encode_teleop([0.0] * 7, [0.0] * 7, 0.0))
        assert isinstance(frame["q"], list)
        assert isinstance(frame["dq"], list)
        assert len(frame["q"]) == 7
        assert len(frame["dq"]) == 7

    def test_decode_bad_length_raises(self):
        with pytest.raises(struct.error):
            decode_teleop(b"\x00" * 100)

    def test_encode_bad_length_raises(self):
        with pytest.raises(ValueError):
            encode_teleop([0.0] * 6, [0.0] * 7, 0.0)

    def test_encode_bad_dq_length_raises(self):
        with pytest.raises(ValueError):
            encode_teleop([0.0] * 7, [0.0] * 6, 0.0)

    def test_bytes_are_literally_interchangeable_with_server(self):
        """⚠ 最关键的一条：本仓编出来的帧 = litearm-server 编出来的帧。

        server 的 `encode_teleop` 就是 `struct.pack(">15d", *q, *dq, ts)`，
        这里把同一表达式**再写一遍**做对拍 —— 一旦有人改了本仓的格式，
        这条会红，而不是等到两端各自"自洽地错"。
        """
        q = [0.5, -1.5, 2.5, -3.5, 0.25, -0.25, 1.0]
        dq = [0.0] * 7
        ts = 99.5
        assert encode_teleop(q, dq, ts) == struct.pack(">15d", *q, *dq, ts)


class TestTeleopTopic:
    def test_topic_format(self):
        assert teleop_topic("armA") == "litearm/v4/armA/teleop"
