"""liteteleop —— 同构遥操上位机（主臂零重力拖动 → zenoh 点对点 → 从臂 move_js 跟随）。

设计依据见 `docs/superpowers/specs/2026-09-28-isomorphic-teleop-design.md`。
本包**不依赖 litearm / pylitearm**：线协议与安全逻辑都是纯 Python。
"""
__version__ = "0.0.0"
