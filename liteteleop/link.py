"""Zenoh **纯点对点**链路（见 spec §2.1 / §4.1）。

两端都 `mode="peer"`，关掉 multicast/gossip 发现 —— **不用广播**，只走显式
`listen`/`connect` 端点。主臂监听端口，从臂连过去。

⚠ **进程退出前必须 `close()`**，否则**永久挂死**（spec §2.1 `[实测]`：不 close 时
解释器退不出去，`timeout` 才能杀掉）。
⚠ key 与 litearm-server 的 `litearm/v4/{arm_id}/teleop` **不可混用**：那边是 `>15d` 120 B，
本文是 70 B，共用 key 会让两端把对方的帧静默解错。
"""
from __future__ import annotations

import threading
from typing import Callable, Optional

import zenoh

__all__ = ["DEFAULT_KEY", "Listener", "Connector", "LatestSlot"]

#: 默认 key —— 主臂发布、从臂订阅，两端必须一致（界面可改）。
DEFAULT_KEY = "litearm/teleop/isomorphic"


def _base_config() -> zenoh.Config:
    """纯点对点的公共配置：**关掉全部广播发现**。"""
    c = zenoh.Config()
    c.insert_json5("scouting/multicast/enabled", "false")
    c.insert_json5("scouting/gossip/enabled", "false")
    c.insert_json5("mode", '"peer"')
    return c


class _Endpoint:
    def __init__(self) -> None:
        self._session: Optional[zenoh.Session] = None
        self._closed = False

    def close(self) -> None:
        """⚠ 幂等，但**必须被调用**（否则进程退不出去）。"""
        if self._closed:
            return
        self._closed = True
        s, self._session = self._session, None
        if s is not None:
            try:
                s.close()
            except Exception:                       # noqa: BLE001 - 退出路径不抛
                pass

    def __enter__(self):
        return self

    def __exit__(self, *exc) -> bool:
        self.close()
        return False


class Listener(_Endpoint):
    """主臂端：监听端口，等从臂连进来。"""

    def __init__(self, port: int, key: str = DEFAULT_KEY):
        super().__init__()
        cfg = _base_config()
        cfg.insert_json5("listen/endpoints", f'["tcp/0.0.0.0:{int(port)}"]')
        cfg.insert_json5("connect/endpoints", "[]")
        self._session = zenoh.open(cfg)
        self._pub = self._session.declare_publisher(key)
        self._matching = False

        def _on_match(status) -> None:
            self._matching = bool(status.matching)

        self._pub.declare_matching_listener(_on_match)

    def put(self, payload: bytes) -> None:
        """发布一帧。非阻塞。"""
        self._pub.put(payload)

    @property
    def matching(self) -> bool:
        """是否有订阅者匹配。

        ⚠ **是布尔不是计数** —— `zenoh.MatchingStatus` 只有 `.matching`
        （在**跑测试的那个解释器**上核过：`dir(zenoh.MatchingStatus) == ['matching']`；
        ⚠ 本机 `pip` 与 `python3 -m pip` 指向**不同解释器**，版本也不同 —— 别用裸 `pip` 量版本）。
        界面显示「已匹配/未匹配」。
        """
        return self._matching


class Connector(_Endpoint):
    """从臂端：连到主臂的 IP:端口，订阅其流。

    `on_frame` 在 **zenoh 自己的线程**上被调用 ⇒ **只许写一个 latest 槽**，
    不许在里面做阻塞操作（`move_js` 每帧等 ACK，放进去会把 zenoh 线程拖死）。见 spec §3.2。
    """

    def __init__(self, host: str, port: int, key: str = DEFAULT_KEY,
                 on_frame: Optional[Callable[[bytes], None]] = None):
        super().__init__()
        cfg = _base_config()
        cfg.insert_json5("listen/endpoints", "[]")
        cfg.insert_json5("connect/endpoints", f'["tcp/{host}:{int(port)}"]')
        self._session = zenoh.open(cfg)
        cb = on_frame or (lambda _b: None)
        #: 计数（只读，供界面显示）。
        #: ⚠ 安全性来自**同一个 subscriber 的回调在同一条 zenoh 线程上串行**，
        #:   不是「GIL 下 `+=` 原子」—— 属性 `+=` 本身不是原子字节码，别照这句去推广。
        self.received = 0

        def _handler(sample) -> None:
            self.received += 1
            cb(bytes(sample.payload))

        self._sub = self._session.declare_subscriber(key, _handler)

    def latest(self) -> int:
        """已收帧数（供界面显示频率）。"""
        return self.received


class LatestSlot:
    """latest-wins 槽 —— zenoh 回调与伺服环之间的唯一交接面（spec §3.2）。

    只保留**最新**一帧：迟到帧直接覆盖，不排队。遥操只要最新姿态，
    积压帧会让从臂去追一条过期的轨迹。
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._payload: Optional[bytes] = None
        self._recv_ts = 0.0
        self.dropped = 0

    def put(self, payload: bytes, now: float) -> None:
        with self._lock:
            if self._payload is not None:
                self.dropped += 1
            self._payload = payload
            self._recv_ts = now

    def take(self):
        """取走最新一帧（`(payload_or_None, recv_ts)`）。取走后槽清空。"""
        with self._lock:
            p, t = self._payload, self._recv_ts
            self._payload, self._recv_ts = None, 0.0
        return p, t

    def peek_age(self, now: float) -> float:
        """距最近一帧的**本地**时间（秒）。0.0 = 从未收到过。

        ⚠ 这是**本机**时间差，与帧里的 `ts`（主臂时钟）无关 —— 跨机两端不同源。
        """
        with self._lock:
            return 0.0 if self._recv_ts == 0.0 else now - self._recv_ts
