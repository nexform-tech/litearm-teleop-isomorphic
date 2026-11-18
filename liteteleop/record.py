"""可选 CSV 记录器 —— **只在设置 `LITEARM_RECORD_DIR` 时启用**，用于真机诊断。

四个文件（文件名带 PID，主/从两个进程各一套）：

    <pid>-state.csv         SDK 读线程收到的每一帧状态（≈100 Hz，含静止抖动、力矩）
    <pid>-master_pub.csv    主臂实际 publish 的 q/dq（200 Hz，含重复帧）
    <pid>-slave_rx.csv      从臂收到的每一帧主臂数据（含主臂 timestamp）
    <pid>-slave_loop.csv    从臂每拍下发的 q_cmd/dq_cmd 与实测 q/dq（250 Hz）

⚠ 记录器**只写文件**，不参与控制律；关闭环境变量时 `from_env()` 返回 `None`，
  所有调用点都走短路，不改变原有行为。
"""
from __future__ import annotations

import csv
import os
import threading
import time
from pathlib import Path
from typing import Iterable, Optional, Sequence

__all__ = ["Recorder", "from_env"]

STATE_HEADER = (["t"] + [f"q{i}" for i in range(7)]
                + [f"dq{i}" for i in range(7)]
                + [f"tau{i}" for i in range(7)]
                + [f"tmos{i}" for i in range(7)]
                + ["enabled", "flags", "joint_fault", "seq"])
PUB_HEADER = (["t"] + [f"q{i}" for i in range(7)] + [f"dq{i}" for i in range(7)])
RX_HEADER = (["t", "master_t"] + [f"q{i}" for i in range(7)]
             + [f"dq{i}" for i in range(7)])
LOOP_HEADER = (["t"] + [f"cmd_q{i}" for i in range(7)]
               + [f"cmd_dq{i}" for i in range(7)]
               + [f"meas_q{i}" for i in range(7)]
               + [f"meas_dq{i}" for i in range(7)])


class Recorder:
    """线程安全的带缓冲 CSV 写入器；每 0.5 s flush 一次（进程被杀也最多丢半秒）。"""

    def __init__(self, path: Path, header: Sequence[str]):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._f = open(self.path, "w", newline="", encoding="utf-8")
        self._w = csv.writer(self._f)
        self._w.writerow(list(header))
        self._f.flush()
        self._lock = threading.Lock()
        self._last_flush = time.monotonic()
        self.rows = 0

    def row(self, values: Iterable[float], t: Optional[float] = None) -> None:
        stamp = time.time() if t is None else float(t)
        with self._lock:
            self._w.writerow([f"{stamp:.6f}", *values])
            self.rows += 1
            now = time.monotonic()
            if now - self._last_flush >= 0.5:
                self._f.flush()
                self._last_flush = now

    def close(self) -> None:
        with self._lock:
            try:
                self._f.flush()
            finally:
                self._f.close()


def from_env(pid: Optional[int] = None) -> Optional[dict]:
    """按 `LITEARM_RECORD_DIR` 创建四个记录器；未设置就返回 `None`。"""
    root = os.environ.get("LITEARM_RECORD_DIR", "").strip()
    if not root:
        return None
    pid = os.getpid() if pid is None else int(pid)
    base = Path(root)
    return {
        "state": Recorder(base / f"{pid}-state.csv", STATE_HEADER),
        "pub": Recorder(base / f"{pid}-master_pub.csv", PUB_HEADER),
        "rx": Recorder(base / f"{pid}-slave_rx.csv", RX_HEADER),
        "loop": Recorder(base / f"{pid}-slave_loop.csv", LOOP_HEADER),
    }
