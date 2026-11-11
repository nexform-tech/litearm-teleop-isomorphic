"""随包发布的 `litearm` SDK —— 总是从私有 vendored 副本导入。

SDK 放在 `liteteleop/_vendor/litearm/`（来源见同目录的 `VENDORED.md`），不占用
顶层 `litearm` 安装位：用户可能为自己的项目装了 `litearm-python`，两份可以共存。
本模块把 `_vendor` 顶到 `sys.path` 最前，并清掉进程里已经导入的别的 `litearm`，
所以本应用永远加载随包副本。要升级 SDK，就更新这份副本，而不是依赖机器上装的那份。
"""
from __future__ import annotations

import sys
from pathlib import Path

VENDORED_SDK_DIR = (Path(__file__).resolve().parent / "_vendor" / "litearm").resolve()
VENDORED_PARENT = VENDORED_SDK_DIR.parent


class ArmSdkUnavailable(RuntimeError):
    """随包的 `litearm` 缺失、损坏，或者仍被另一份同名包抢先。"""


def _drop_foreign_modules() -> None:
    """清掉不是随包副本的 `litearm`（含子模块），让下一次 import 重新解析。"""
    module = sys.modules.get("litearm")
    if module is not None:
        path = Path(getattr(module, "__file__", "") or "").resolve()
        if path.is_relative_to(VENDORED_SDK_DIR):
            return
    for name in [n for n in sys.modules if n == "litearm" or n.startswith("litearm.")]:
        del sys.modules[name]


def _put_vendor_first() -> None:
    """把私有 `_vendor` 目录放到 `sys.path` 最前，压掉 site-packages 里的同名包。"""
    parent = str(VENDORED_PARENT)
    while parent in sys.path:
        sys.path.remove(parent)
    sys.path.insert(0, parent)


def import_arm_sdk() -> str:
    """导入随包 SDK，返回其 `__init__.py` 的路径；落点不对就抛错。"""
    _drop_foreign_modules()
    _put_vendor_first()
    try:
        import litearm
    except ImportError as e:
        raise ArmSdkUnavailable(
            f"随包的 litearm 导入失败（{e}）—— 安装包不完整，重新安装 liteteleop") from e
    module = Path(getattr(litearm, "__file__", "") or "").resolve()
    if not module.is_relative_to(VENDORED_SDK_DIR):
        raise ArmSdkUnavailable(
            f"litearm 导入自 {module}，不是随包的 {VENDORED_SDK_DIR} —— "
            "仍有另一份同名包抢先")
    if not hasattr(getattr(litearm, "Arm", None), "joint_follow"):
        raise ArmSdkUnavailable(
            f"{module} 不是本仓随包的 litearm-python：Arm 没有 joint_follow —— "
            "安装包不完整，重新安装 liteteleop")
    return str(module)
