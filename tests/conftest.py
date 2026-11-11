"""测试也核对 `litearm` 的落点 —— **与 `liteteleop.arm_sdk.import_arm_sdk()` 同一判据**。

⚠ 为什么必须有这个文件：`pytest` **不经过** `liteteleop.__main__`，所以入口处那道检查
   对测试完全不生效。本机 `sys.path` 上可能挂着另一份 `litearm`（例如旧的 editable
   安装）—— 两份子模块同名同构，导错了照样能跑，只有用到新接口时才炸。

仓库根目录必须先于 site-packages 出现在 `sys.path`，这样 `import litearm` 才会落到
随包的 `litearm/`；然后由 `import_arm_sdk()` 断言落点。
"""
from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from liteteleop.arm_sdk import import_arm_sdk  # noqa: E402

import_arm_sdk()
