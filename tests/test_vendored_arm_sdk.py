"""随包发布的 `litearm` SDK —— 来源、树哈希，以及"总是加载随包副本"。

本文件检查三件事：一份干净的 checkout 不装任何东西也能导入随包 SDK；`_vendor/litearm/`
里的文件与 `VENDORED.md` 记录的树哈希一致；即使 site-packages 里有另一份 `litearm`
（用户自己的 `litearm-python`），导入助手也会把它换成随包副本。树哈希那条能抓住
"就地改 SDK"——这种改动在下次 re-vendor 时会被静默覆盖。
"""
from __future__ import annotations

import hashlib
import re
import subprocess
import sys
import types
from pathlib import Path

import pytest

from liteteleop.arm_sdk import (
    VENDORED_SDK_DIR,
    ArmSdkUnavailable,
    import_arm_sdk,
)

REPO_ROOT = Path(__file__).resolve().parent.parent
VENDORED_SDK = REPO_ROOT / "liteteleop" / "_vendor" / "litearm"
PROVENANCE = VENDORED_SDK / "VENDORED.md"


@pytest.fixture
def isolated_litearm(monkeypatch):
    """卸掉已加载的 SDK，并让测试对 `sys.path` 的改动在结束后还原。"""
    monkeypatch.setattr(sys, "path", list(sys.path))
    for name in [n for n in sys.modules if n == "litearm" or n.startswith("litearm.")]:
        monkeypatch.delitem(sys.modules, name)


def tree_hash(root: Path) -> str:
    """按 `VENDORED.md` 的配方算树哈希（排除本文件与 `__pycache__`）。"""
    digest = hashlib.sha256()
    for path in sorted(root.rglob("*")):
        if not path.is_file() or "__pycache__" in path.parts \
                or path.name == "VENDORED.md":
            continue
        digest.update((path.relative_to(root).as_posix() + "\n").encode())
        digest.update(path.read_bytes())
    return digest.hexdigest()


def recorded_hash() -> str:
    """`VENDORED.md` 的 `## Tree hash` 一节里记录的哈希。"""
    text = PROVENANCE.read_text(encoding="utf-8")
    match = re.search(r"^## Tree hash\n+```text\n([0-9a-f]{64})\n```",
                      text, re.MULTILINE)
    assert match, "VENDORED.md 里没有 `## Tree hash` 与 64 位哈希"
    return match.group(1)


def foreign_module(tmp_path: Path) -> types.ModuleType:
    """一份落在随包目录之外、但接口看起来齐全的 `litearm`。"""
    module = types.ModuleType("litearm")
    module.__file__ = str(tmp_path / "site-packages" / "litearm" / "__init__.py")
    module.Arm = type("Arm", (), {"joint_follow": lambda self, *a: None})
    return module


def test_the_vendored_tree_matches_its_recorded_hash():
    """判别力：就地改 `_vendor/litearm/` 里任何一个字节，本用例红。"""
    assert tree_hash(VENDORED_SDK) == recorded_hash()


def test_a_clean_interpreter_imports_the_vendored_sdk():
    """不装 `litearm-python`、不读 `PYTHONPATH` / site-packages，也能 import。

    `-I` 忽略环境变量与用户 site，`-S` 跳过 site-packages。子进程里显式插入仓库
    根目录，经真实入口的导入助手加载，断言落点就在私有 `_vendor/litearm/` 下。
    """
    source = (
        "import sys; "
        f"sys.path.insert(0, {str(REPO_ROOT)!r}); "
        "from liteteleop.arm_sdk import import_arm_sdk; "
        "print(import_arm_sdk())"
    )
    result = subprocess.run(
        [sys.executable, "-I", "-S", "-c", source],
        cwd=REPO_ROOT, capture_output=True, text=True,
    )
    assert result.returncode == 0, result.stderr
    assert Path(result.stdout.strip()).resolve().is_relative_to(VENDORED_SDK)


def test_the_sdk_exposes_the_entry_points_the_worker_uses(isolated_litearm):
    """随包副本必须是能跑本仓 worker 的那一版，而不是同名的旧包。"""
    module = Path(import_arm_sdk()).resolve()
    assert module.is_relative_to(VENDORED_SDK)

    import litearm

    for name in ("connect", "enable", "disable", "get_state",
                 "movej", "move_js", "joint_follow", "close"):
        assert hasattr(litearm.Arm, name), f"Arm 缺少 {name}()"
    assert (VENDORED_SDK / "py.typed").is_file()
    assert (VENDORED_SDK / "LICENSE").is_file()


def test_import_arm_sdk_replaces_a_foreign_copy(
        isolated_litearm, monkeypatch, tmp_path):
    """用户自己装的 `litearm-python` 可以共存，但本进程只用随包副本。"""
    foreign = foreign_module(tmp_path)
    monkeypatch.setitem(sys.modules, "litearm", foreign)
    monkeypatch.setitem(sys.modules, "litearm.arm", types.ModuleType("litearm.arm"))

    module = Path(import_arm_sdk()).resolve()

    assert module.is_relative_to(VENDORED_SDK)
    assert sys.modules["litearm"] is not foreign
    assert Path(sys.modules["litearm.arm"].__file__).resolve().is_relative_to(
        VENDORED_SDK)


def test_import_arm_sdk_puts_the_vendored_copy_first(
        isolated_litearm, monkeypatch, tmp_path):
    """site-packages 里的同名包即使排在前面，也不能被导入。"""
    decoy = tmp_path / "site-packages"
    (decoy / "litearm").mkdir(parents=True)
    (decoy / "litearm" / "__init__.py").write_text(
        "raise RuntimeError('wrong litearm')\n", encoding="utf-8")
    monkeypatch.syspath_prepend(str(decoy))

    assert Path(import_arm_sdk()).resolve().is_relative_to(VENDORED_SDK)


def test_import_arm_sdk_rejects_an_incomplete_vendored_copy(
        isolated_litearm, monkeypatch, tmp_path):
    """落点对了但缺 `joint_follow`，仍要报错，而不是连上臂才暴露。"""
    import liteteleop.arm_sdk as arm_sdk

    fake_root = tmp_path / "litearm"
    fake_root.mkdir()
    (fake_root / "__init__.py").write_text("class Arm: pass\n", encoding="utf-8")
    monkeypatch.setattr(arm_sdk, "VENDORED_SDK_DIR", fake_root.resolve())
    monkeypatch.setattr(arm_sdk, "VENDORED_PARENT", fake_root.parent.resolve())

    with pytest.raises(ArmSdkUnavailable, match="joint_follow"):
        arm_sdk.import_arm_sdk()


def test_the_entry_point_uses_the_vendored_sdk(
        isolated_litearm, monkeypatch, tmp_path, capsys):
    """启动入口不因另一份 `litearm` 退出；它换用随包副本并继续。"""
    monkeypatch.setitem(sys.modules, "litearm", foreign_module(tmp_path))

    from liteteleop.__main__ import main

    with pytest.raises(SystemExit) as exc:
        main(["--help"])
    assert exc.value.code == 0
    assert str(VENDORED_SDK) in capsys.readouterr().out
