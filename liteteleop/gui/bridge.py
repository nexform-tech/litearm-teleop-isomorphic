"""worker → Qt 的信号桥。

⚠⚠ **两条线程纪律**（写错就会炸 Qt，而且是偶发、难查的那种）：

1. **`on_state` 回调在 SDK 自己的读线程上跑**（`_Ack._on_status`）⇒ 桥里**只许 `emit`**，
   绝不碰控件。`emit` 跨线程是 Qt 支持的（`AutoConnection` 会排队到接收者线程）。
2. **`on_log` 可能从任何线程来** ⇒ 同样只 `emit`。

⚠ **信号参数必须是可跨线程传递的值**：`Snapshot` 是普通 dataclass（不是 QObject），
`pyqtSignal(object)` 传它没问题；不要传 QWidget / QModelIndex 之类。
"""
from __future__ import annotations

from PyQt5 import QtCore

from ..arm_worker import Snapshot

__all__ = ["WorkerBridge"]


class WorkerBridge(QtCore.QObject):
    """把 worker 的回调转成 Qt 信号。"""

    #: 每来一帧状态就发一次（**SDK 读线程**上 emit）
    state = QtCore.pyqtSignal(object)
    #: 一条日志文本
    log = QtCore.pyqtSignal(str)
    #: 遥操状态变了（True=已启动）
    teleop_changed = QtCore.pyqtSignal(bool)

    def __init__(self, parent=None):
        super().__init__(parent)

    # ---- 这两个是给 worker 直接当回调用的（非 Qt 线程调用）----
    def on_state(self, snap: Snapshot) -> None:
        self.state.emit(snap)

    def on_log(self, text: str) -> None:
        self.log.emit(text)
