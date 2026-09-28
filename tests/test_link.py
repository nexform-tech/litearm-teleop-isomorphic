"""Zenoh 点对点链路 —— 起**真** session 走回环（非 mock），并钉住 close() 纪律。"""
import subprocess
import sys
import textwrap
import time

import pytest

from liteteleop import link

KEY = "litearm/teleop/test"


def _free_port() -> int:
    import socket
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def test_roundtrip_bytes():
    """端到端收发：100 帧逐字节全等。"""
    port = _free_port()
    m = link.Listener(port, KEY)
    got = []
    s = link.Connector("127.0.0.1", port, KEY, on_frame=got.append)
    try:
        time.sleep(0.5)
        frames = [bytes([i % 256]) * 70 for i in range(100)]
        for f in frames:
            m.put(f)
            time.sleep(0.002)
        deadline = time.monotonic() + 5.0
        while len(got) < len(frames) and time.monotonic() < deadline:
            time.sleep(0.01)
        assert got == frames
    finally:
        s.close()
        m.close()


def test_sustained_100hz_no_loss():
    """100 Hz 持续 2 s 零丢包 —— spec §2.1 实测过的能力，回归钉住。"""
    port = _free_port()
    m = link.Listener(port, KEY)
    got = []
    s = link.Connector("127.0.0.1", port, KEY, on_frame=got.append)
    try:
        time.sleep(0.5)
        got.clear()
        n, t0 = 0, time.monotonic()
        dt, nxt = 0.01, t0 + 0.01
        while time.monotonic() - t0 < 2.0:
            m.put(b"x" * 70)
            n += 1
            r = nxt - time.monotonic()
            if r > 0:
                time.sleep(r)
            nxt += dt
            if nxt < time.monotonic():
                nxt = time.monotonic() + dt
        deadline = time.monotonic() + 3.0        # 给在途帧留到达窗口
        while len(got) < n and time.monotonic() < deadline:
            time.sleep(0.01)
        assert n >= 190, f"只发出 {n} 帧"
        assert len(got) == n, f"丢包: 发 {n} 收 {len(got)}"
    finally:
        s.close()
        m.close()


def test_matching_flag():
    """主臂端能拿到 matching 布尔（⚠ 是布尔不是计数，spec §8）。"""
    port = _free_port()
    m = link.Listener(port, KEY)
    try:
        assert m.matching is False
        s = link.Connector("127.0.0.1", port, KEY, on_frame=lambda b: None)
        try:
            deadline = time.monotonic() + 5.0
            while not m.matching and time.monotonic() < deadline:
                time.sleep(0.05)
            assert m.matching is True
        finally:
            s.close()
    finally:
        m.close()

def test_latest_slot_keeps_only_newest():
    """latest-wins：迟到帧覆盖、不排队（spec §3.2）。被覆盖的帧要计数，不静默。"""
    slot = link.LatestSlot()
    slot.put(b"a", now=1.0)
    slot.put(b"b", now=2.0)
    assert slot.take() == (b"b", 2.0)
    assert slot.dropped == 1, "被覆盖的那帧要计入 dropped（不静默）"


def test_latest_slot_take_clears():
    slot = link.LatestSlot()
    assert slot.take() == (None, 0.0), "空槽取回 (None, 0.0)"
    slot.put(b"x", now=5.0)
    assert slot.take() == (b"x", 5.0)
    assert slot.take() == (None, 0.0), "取走后槽必须清空"


def test_latest_slot_age_is_local_and_zero_when_never_received():
    """⚠ `peek_age` 是**本机**时间差，与帧里的 `ts`（主臂时钟）无关（spec §4.2）。

    从未收到过时返回 0.0 —— **不是** `now`（那会让 watchdog 以为"刚收到"）。
    """
    slot = link.LatestSlot()
    assert slot.peek_age(now=100.0) == 0.0, "从未收到 ⇒ 0.0"
    slot.put(b"x", now=10.0)
    assert slot.peek_age(now=10.25) == pytest.approx(0.25)


@pytest.mark.slow
def test_close_is_mandatory_for_exit():
    """⚠ 不 close ⇒ 进程永久挂死（spec §2.1 `[实测]`）。

    用**子进程**验证，因为症状是"解释器退不出去"，同进程测不到。
    """
    prog = textwrap.dedent("""
        import sys
        sys.path.insert(0, ".")
        from liteteleop import link
        m = link.Listener(int(sys.argv[1]), "x/y")
        m.put(b"hi")
        if sys.argv[2] == "close":
            m.close()
        print("reached-end")
    """)
    port = _free_port()
    ok = subprocess.run([sys.executable, "-c", prog, str(port), "close"],
                        capture_output=True, text=True, timeout=30)
    assert ok.returncode == 0, f"显式 close 后应正常退出: {ok.stderr}"
    assert "reached-end" in ok.stdout

    # ⚠ 症状是"解释器退不出去" ⇒ 表现为 subprocess.run **超时抛 TimeoutExpired**，
    # 不是返回非零码。**超时就是通过**（下面这行是唯一正确的判据写法）。
    with pytest.raises(subprocess.TimeoutExpired):
        subprocess.run([sys.executable, "-c", prog, str(port + 1), "noclose"],
                       capture_output=True, text=True, timeout=20)
