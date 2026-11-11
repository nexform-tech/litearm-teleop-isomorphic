"""随包发布的 `litearm` SDK —— 导入并核对落点。

只认仓库根目录下的 `litearm/`（来源见 `litearm/VENDORED.md`）。另一份同名包
（例如本机可编辑安装的 `litearm-python`）如果抢先，必须当场报错：它可能停在
旧版本，缺 `joint_follow` 之类的入口，而这种错误要等到真机连上才暴露。
"""
from __future__ import annotations

from pathlib import Path

VENDORED_SDK_DIR = (Path(__file__).resolve().parent.parent / "litearm").resolve()


class ArmSdkUnavailable(RuntimeError):
    """随包的 `litearm` 导入失败，或者被另一份同名包抢先了。"""


def import_arm_sdk() -> str:
    """导入随包 SDK，返回 `litearm/__init__.py` 的路径；落点不对就抛错。"""
    try:
        import litearm
    except ImportError as e:
        raise ArmSdkUnavailable(
            f"随包的 litearm 导入失败（{e}）—— 重新安装本包；"
            "litearm 不需要单独安装") from e
    module = Path(getattr(litearm, "__file__", "") or "").resolve()
    if not module.is_relative_to(VENDORED_SDK_DIR):
        raise ArmSdkUnavailable(
            f"litearm 导入自 {module}，不是随包的 {VENDORED_SDK_DIR} —— "
            "另一份同名包抢先了；不要 pip install litearm-python")
    if not hasattr(getattr(litearm, "Arm", None), "joint_follow"):
        raise ArmSdkUnavailable(
            f"{module} 不是本仓随包的 litearm-python：Arm 没有 joint_follow")
    return str(module)
