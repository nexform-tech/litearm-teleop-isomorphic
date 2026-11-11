"""测试也走 `liteteleop.arm_sdk.import_arm_sdk()` —— 与真实启动同一条导入路径。

⚠ 为什么必须有这个文件：`pytest` **不经过** `liteteleop.__main__`，入口处那次调用
   对测试不生效。这里主动调一次：它把 `_vendor` 顶到 `sys.path` 最前，并清掉内存里
   可能已经导入的另一份 `litearm`，所以测试加载的也是随包副本 ——
   site-packages 里用户自己的 `litearm-python` 不会被用到。
"""
from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from liteteleop.arm_sdk import import_arm_sdk  # noqa: E402

import_arm_sdk()
