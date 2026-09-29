"""PyQt5 界面。

**版式**照用户给的设计稿，**视觉语言**照 `litearm-studio`
（React 那套设计令牌，见 `theme.py` 的出处注释）。

分层：

| 模块 | 管什么 |
| --- | --- |
| `theme` | 设计令牌 + 全局 QSS（**颜色的唯一真源**） |
| `cards` | 控件语汇：卡片 / 徽标 / 分段控件 / 指标块 / 大 STOP / 滚动列 |
| `chart` | 实时指标曲线卡（自绘，行为照 studio 的 `MetricsPanel`） |
| `shell` | 应用外壳：左导航栏 + 顶栏 |
| `widgets` | 7 轴关节表（`err != 1 才红` 的**唯一真源**） |
| `pages` | 连接 / 遥操 / 日志 / 设置 四页 |
| `bridge` | worker → Qt 的信号桥（跨线程只许 `emit`） |
| `main_window` | 装配 + 安全闸门 + worker 生命周期 |

⚠ GUI **只投命令、只读快照** —— 任何 SDK 调用都在 `ArmWorker` 线程里（spec §3.1）。
⚠⚠ `main_window` 的模块 docstring 列了**九条不许弄丢的安全纪律**，改界面前先读它。
"""
from .main_window import MainWindow, run

__all__ = ["MainWindow", "run"]
