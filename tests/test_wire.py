"""线协议 v1 —— 黄金字节钉死布局（见 spec §4.2）。

⚠ 这里**刻意不用往返测试**：往返对字节序没有判别力（两端同错也全绿）。
期望值由**独立于实现**的字面格式串算出，实现改动布局时先红。
"""
import struct

import pytest

from liteteleop import wire


def test_golden_bytes_n7():
    """n=7 的黄金字节 —— 小端、字段顺序、70 B 全长，全部钉死。"""
    q = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]
    dq = [0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07]
    got = wire.encode(q, dq, ts=1234.5, seq=42)
    # 期望由字面格式串独立算出，不引用 wire 里的任何常量
    want = struct.pack("<BB7f7fdI", 1, 7, *q, *dq, 1234.5, 42)
    assert got == want
    assert got.hex() == (
        "0107cdcccc3dcdcc4c3e9a99993ecdcccc3e0000003f9a99193f3333333f"
        "0ad7233c0ad7a33c8fc2f53c0ad7233dcdcc4c3d8fc2753d295c8f3d"
        "00000000004a93402a000000"
    )
    assert len(got) == 70


def test_golden_bytes_n1():
    """变长关节数：n=1 也成立（1J 台架/单轴测试用）。"""
    got = wire.encode([0.25], [0.5], ts=1.0, seq=7)
    assert got == struct.pack("<BB1f1fdI", 1, 1, 0.25, 0.5, 1.0, 7)
    assert got.hex() == "01010000803e0000003f000000000000f03f07000000"
    assert len(got) == 22


def test_roundtrip_values():
    q = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7]
    dq = [1.0, -1.0, 2.0, -2.0, 3.0, -3.0, 4.0]
    f = wire.decode(wire.encode(q, dq, ts=99.25, seq=65535), expect_n=7)
    assert f.n == 7
    assert f.q == pytest.approx(q)
    assert f.dq == pytest.approx(dq)
    assert f.ts == 99.25
    assert f.seq == 65535


def test_reject_bad_version():
    bad = bytearray(wire.encode([0.0] * 7, [0.0] * 7, ts=0.0, seq=0))
    bad[0] = 2                                   # version 改成不认识的
    with pytest.raises(wire.WireError, match="协议版本"):
        wire.decode(bytes(bad), expect_n=7)


def test_reject_n_mismatch():
    """帧里 n=3、本端 7 ⇒ 必须拒收，不许照 n 去解一帧垃圾。"""
    payload = wire.encode([0.0] * 3, [0.0] * 3, ts=0.0, seq=0)
    with pytest.raises(wire.WireError, match="关节数不符"):
        wire.decode(payload, expect_n=7)


def test_reject_short_frame():
    payload = wire.encode([0.0] * 7, [0.0] * 7, ts=0.0, seq=0)
    with pytest.raises(wire.WireError, match="帧长不符"):
        wire.decode(payload[:-1], expect_n=7)
    with pytest.raises(wire.WireError, match="帧太短"):
        wire.decode(b"\x01", expect_n=7)


def test_reject_bad_n_at_encode():
    with pytest.raises(wire.WireError, match="关节数"):
        wire.encode([], [], ts=0.0, seq=0)
    with pytest.raises(wire.WireError, match="长度不符"):
        wire.encode([0.0] * 7, [0.0] * 6, ts=0.0, seq=0)


def test_bad_n_in_frame():
    """帧里 n=0 / n=200 ⇒ 拒收（防越界与畸形头）。"""
    with pytest.raises(wire.WireError, match="关节数"):
        wire.decode(bytes([1, 0]) + b"\x00" * 8, expect_n=None)
    with pytest.raises(wire.WireError, match="关节数"):
        wire.decode(bytes([1, 200]) + b"\x00" * 8, expect_n=None)
