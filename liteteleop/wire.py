"""线协议 v1 —— 主臂 → 从臂的关节角流帧（见 spec §4.2）。

布局（**小端**，变长关节数）::

    offset  size   field
    0       1      version = 1
    1       1      n                 关节数 (1..MAX_JOINTS)
    2       4n     q[n]    f32 LE    主臂实测关节角 rad
    2+4n    4n     dq[n]   f32 LE    主臂实测关节速度 rad/s
    2+8n    8      ts      f64 LE    主臂 time.monotonic()
    2+8n+8  4      seq     u32 LE    主臂帧序号

n = 7 时共 70 B。

⚠ `version` 字节是防「静默解错」的那道判据：两端版本不符时**拒收**，
而不是照 `n` 去解一帧垃圾。
⚠ `ts` 取主臂的 `time.monotonic()`：跨机两端时钟不同源 ⇒ 它**只能**算同一主臂
相邻两帧的间隔，**不能**算端到端延迟（spec §4.2）。
⚠ 本格式与 litearm-server 的 `litearm/v4/{arm_id}/teleop`（`>15d` 120 B）**不兼容**，
key 也不同，别混用。
"""
from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Sequence, Tuple

__all__ = ["VERSION", "MAX_JOINTS", "WireError", "Frame", "encode", "decode", "frame_size"]

#: 协议版本。**改布局必须同时改它**（否则两端会照旧格式静默解错）。
VERSION = 1

#: 关节数上界 —— 防畸形头导致的越界分配。7 关节臂 + 余量。
MAX_JOINTS = 32


class WireError(ValueError):
    """帧不合法（版本不符 / 长度不符 / 关节数不符）。"""


@dataclass(frozen=True)
class Frame:
    """一帧解出来的值。`q`/`dq` 是 tuple（不可变，避免跨线程被就地改）。"""

    q: Tuple[float, ...]
    dq: Tuple[float, ...]
    ts: float
    seq: int

    @property
    def n(self) -> int:
        return len(self.q)


def _fmt(n: int) -> str:
    return f"<BB{n}f{n}fdI"


def frame_size(n: int) -> int:
    """按关节数算帧长（收端用来校验长度）。"""
    return struct.calcsize(_fmt(n))


def encode(q: Sequence[float], dq: Sequence[float], ts: float, seq: int) -> bytes:
    """编码一帧。`q`/`dq` 长度必须相等且在 `1..MAX_JOINTS`。"""
    n = len(q)
    if n != len(dq):
        raise WireError(f"q/dq 长度不符: {n} vs {len(dq)}")
    if not 1 <= n <= MAX_JOINTS:
        raise WireError(f"关节数 {n} 越界 (1..{MAX_JOINTS})")
    return struct.pack(_fmt(n), VERSION, n, *q, *dq, float(ts), int(seq) & 0xFFFFFFFF)


def decode(payload: bytes, expect_n: int | None = None) -> Frame:
    """解码一帧。

    `expect_n` 给定时，帧里的 `n` 必须与之相等 —— 这是「别照 n 去解一帧垃圾」的落点。
    本端关节数是**已知**的（`arm.n`），所以调用方**应该**总是传 `expect_n`。
    """
    if len(payload) < 2:
        raise WireError(f"帧太短: {len(payload)} B")
    version, n = struct.unpack_from("<BB", payload, 0)
    if version != VERSION:
        raise WireError(f"协议版本不符: 帧={version} 本端={VERSION}")
    if not 1 <= n <= MAX_JOINTS:
        raise WireError(f"关节数 {n} 越界 (1..{MAX_JOINTS})")
    if expect_n is not None and n != expect_n:
        raise WireError(f"关节数不符: 帧={n} 本端={expect_n}")
    want = frame_size(n)
    if len(payload) != want:
        raise WireError(f"帧长不符: 收到 {len(payload)} B, 按 n={n} 应为 {want} B")
    vals = struct.unpack(_fmt(n), payload)
    # vals = (version, n, *q, *dq, ts, seq)
    q = vals[2:2 + n]
    dq = vals[2 + n:2 + 2 * n]
    ts = vals[2 + 2 * n]
    seq = vals[3 + 2 * n]
    return Frame(q=q, dq=dq, ts=ts, seq=seq)
