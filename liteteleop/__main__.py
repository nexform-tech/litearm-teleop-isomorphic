"""入口：`python -m liteteleop`。

⛔ **SDK 入口钉死在 `/home/llx/litearm-python/src`**（用户裁决）——
本机 `sys.path` 上还挂着另一份 `litearm`（gitee 克隆，停在 `chore/sync-repo-standards`），
会被**静默**抢先 import。判据不是"我设了 PYTHONPATH"，而是**导入后断言 + 打印**。
"""
from __future__ import annotations

import sys

SDK_SRC = "/home/llx/litearm-python/src"


def _pin_sdk() -> None:
    if SDK_SRC not in sys.path:
        sys.path.insert(0, SDK_SRC)
    import litearm as pa
    if not str(pa.__file__).startswith(SDK_SRC):
        raise SystemExit(
            f"⛔ litearm 导入自 {pa.__file__}，不是 {SDK_SRC} —— 环境里有另一份抢先了")
    print(f"SDK: {pa.__file__}")


def main() -> int:
    _pin_sdk()
    from .gui import run
    return run()


if __name__ == "__main__":
    raise SystemExit(main())
