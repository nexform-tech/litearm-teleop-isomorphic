"""入口：`python -m liteteleop`。

臂 SDK 与夹爪 SDK 都随包发布（`liteteleop/_vendor/litearm/`、`litegrip/`），不要求
用户克隆或单独安装。启动时把臂 SDK 的私有目录顶到 `sys.path` 最前：即使机器上另装了
`litearm-python`（可能是用户自己的项目在用），本应用也总是加载随包副本。
"""
from __future__ import annotations

import sys


def main(argv=None) -> int:
    """命令行：两个进程各管一条臂时用得上 —— 免得两个进程抢同一份设置文件。

        python -m liteteleop --role master --cdc /dev/ttyACM0
        python -m liteteleop --role slave  --cdc /dev/ttyACM1 --peer 127.0.0.1
    """
    import argparse
    import logging
    try:
        from .arm_sdk import import_arm_sdk
        print(f"SDK: {import_arm_sdk()}")
    except BaseException as e:                           # noqa: BLE001
        # 命令行语义：SDK 不可用就**明确报错退出**，不要甩一坨 traceback。
        print(f"⛔ 随包臂 SDK 不可用：{e}", file=sys.stderr)
        print("   臂 SDK 随本包发布，重新安装 liteteleop", file=sys.stderr)
        return 2
    # ⚠ 必须配 handler：界面里那份 `_log` 走 `log.info`，而**没配 handler 就什么都不落盘**
    #    —— 只有 warning 级以上才进文件。真机排查时日志文件是唯一的一手记录（界面关了就没了）。
    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(levelname).1s %(name)s: %(message)s",
                        stream=sys.stderr)
    ap = argparse.ArgumentParser(prog="liteteleop", description="同构遥操上位机")
    ap.add_argument("--role", choices=["master", "slave"], default=None,
                    help="主臂=监听 / 从臂=连接")
    ap.add_argument("--cdc", default=None, help="CDC 口（两条同型号臂时必须指定）")
    ap.add_argument("--peer", default=None, help="从臂填：主臂的 IP")
    ap.add_argument("--jport", type=int, default=None, help="zenoh 端口")
    ap.add_argument("--arm-id", default=None, help="topic 里的 arm_id（两端必须一致）")
    ap.add_argument("--gcan", default=None, help="夹爪 CAN 通道（空=不启用夹爪遥操）")
    ap.add_argument("--gpeer", default=None, help="从臂夹爪填：主臂的 IP")
    ap.add_argument("--gport", type=int, default=None, help="夹爪 zenoh 端口")
    ap.add_argument("--grip-id", default=None, help="夹爪 topic 里的 grip_id（两端须一致）")
    a = ap.parse_args(argv)

    from .settings import load_settings
    from .gui import run
    s = load_settings()
    if a.role: s.role = a.role
    if a.cdc: s.cdc_port = a.cdc
    if a.peer: s.peer = a.peer
    if a.jport: s.jport = a.jport
    if a.arm_id: s.arm_id = a.arm_id
    if a.gcan: s.gcan = a.gcan
    if a.gpeer: s.gpeer = a.gpeer
    if a.gport: s.gport = a.gport
    if a.grip_id: s.grip_id = a.grip_id
    # ⚠⚠ 钉死必须排在**上面那 4 行赋值之后** —— 否则 `s.gcan` 还是空串，
    #    这段就永远不跑（本计划的初稿就写错成了「load_settings() 之后」）。
    if s.gcan:
        # ⚠ 只认**随包 vendored** 的 litegrip（见 `litegrip/VENDORED.md`），
        #   不往 sys.path 塞任何检出目录，也不要求单独安装。
        try:
            from .grip_worker import import_grip_sdk
            print(f"夹爪 SDK: {import_grip_sdk()}")
        except BaseException as e:                       # noqa: BLE001
            # 命令行语义：缺 SDK 就**明确报错退出**，不要甩一坨 traceback。
            print(f"⛔ 夹爪 SDK 不可用：{e}", file=sys.stderr)
            print("   夹爪 SDK 随本包发布，重新安装 liteteleop；"
                  "去掉 --gcan 可只跑臂遥操", file=sys.stderr)
            return 2
    import sys as _sys
    # ⚠ 只把程序名交给 Qt：`--role`/`--cdc` 这些是**我们的**参数，Qt 不认识。
    return run([_sys.argv[0]], settings=s)


if __name__ == "__main__":
    raise SystemExit(main())
