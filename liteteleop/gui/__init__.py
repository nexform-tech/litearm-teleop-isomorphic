"""PyQt5 界面（仿 `litearm-tool-stm32/litetool` 的形状）。

分层与工具仓一致：`bridge`（信号桥）/ `widgets`（常驻控件）/ `pages`（页面）/ `main_window`。
GUI **只投命令、只读快照** —— 任何 SDK 调用都在 `ArmWorker` 线程里（spec §3.1）。
"""
from .main_window import MainWindow, run

__all__ = ["MainWindow", "run"]
