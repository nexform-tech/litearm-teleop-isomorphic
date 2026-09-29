"""测试侧也把 SDK 路径钉死 —— **与 `liteteleop.__main__._pin_sdk()` 同一判据**。

⚠ 为什么必须有这个文件：`pytest` **不经过** `liteteleop.__main__`，所以那条 pin 对测试
   完全不生效。本机 `sys.path` 上还挂着**另一份** `litearm`（gitee 克隆，停在
   `chore/sync-repo-standards`）—— 两份**子模块同名同构**，导错了照样能跑，
   只有在用到新接口（如 `Arm.joint_follow`）时才炸，而那时已经在真机上了。

`SDK_SRC` 从 `__main__` 导入（**单一真源**，不在这里再写一份路径）；判据也是同一条：
**导入后核落点**，而不是一个"我设过 PYTHONPATH"的声明。
"""
from __future__ import annotations

import sys

from liteteleop.__main__ import SDK_SRC

if SDK_SRC not in sys.path:
    sys.path.insert(0, SDK_SRC)

import litearm as pa  # noqa: E402  (必须在 insert 之后)

if not str(pa.__file__).startswith(SDK_SRC):
    raise RuntimeError(
        f"⛔ litearm 导入自 {pa.__file__}，不是 {SDK_SRC} —— 环境里有另一份抢先了。"
        f"修法：`python -m pip install -e {SDK_SRC.rsplit('/src', 1)[0]} --no-deps`")
