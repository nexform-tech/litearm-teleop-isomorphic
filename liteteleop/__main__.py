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


def main(argv=None) -> int:
    """命令行：两个进程各管一条臂时用得上 —— 免得两个进程抢同一份设置文件。

        python -m liteteleop --role master --cdc /dev/ttyACM0
        python -m liteteleop --role slave  --cdc /dev/ttyACM1 --peer 127.0.0.1
    """
    import argparse
    _pin_sdk()
    ap = argparse.ArgumentParser(prog="liteteleop", description="同构遥操上位机")
    ap.add_argument("--role", choices=["master", "slave"], default=None,
                    help="主臂=监听 / 从臂=连接")
    ap.add_argument("--cdc", default=None, help="CDC 口（两条同型号臂时必须指定）")
    ap.add_argument("--peer", default=None, help="从臂填：主臂的 IP")
    ap.add_argument("--jport", type=int, default=None, help="zenoh 端口")
    ap.add_argument("--arm-id", default=None, help="topic 里的 arm_id（两端必须一致）")
    a = ap.parse_args(argv)

    from .settings import load_settings
    from .gui import run
    s = load_settings()
    if a.role: s.role = a.role
    if a.cdc: s.cdc_port = a.cdc
    if a.peer: s.peer = a.peer
    if a.jport: s.jport = a.jport
    if a.arm_id: s.arm_id = a.arm_id
    import sys as _sys
    # ⚠ 只把程序名交给 Qt：`--role`/`--cdc` 这些是**我们的**参数，Qt 不认识。
    return run([_sys.argv[0]], settings=s)


if __name__ == "__main__":
    raise SystemExit(main())
