"""随包发布的 `litearm` SDK —— 来源、树哈希，以及"只靠本仓就能 import"。

本文件检查三件事：一份干净的 checkout 不装任何东西也能 `import litearm`；
`litearm/` 里的文件与 `litearm/VENDORED.md` 记录的树哈希一致；另一份同名包
抢先时，导入助手与命令行入口都明确失败，而不是静默用错版本。树哈希那条能抓住
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
VENDORED_SDK = REPO_ROOT / "litearm"
PROVENANCE = VENDORED_SDK / "VENDORED.md"


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
    assert match, "litearm/VENDORED.md 里没有 `## Tree hash` 与 64 位哈希"
    return match.group(1)


def foreign_module(tmp_path: Path) -> types.ModuleType:
    """一份落在随包目录之外、但接口看起来齐全的 `litearm`。"""
    module = types.ModuleType("litearm")
    module.__file__ = str(tmp_path / "litearm" / "__init__.py")
    module.Arm = type("Arm", (), {"joint_follow": lambda self, *a: None})
    return module


def test_the_vendored_tree_matches_its_recorded_hash():
    """判别力：就地改 `litearm/` 里任何一个字节，本用例红。"""
    assert tree_hash(VENDORED_SDK) == recorded_hash()


def test_a_clean_interpreter_imports_the_vendored_sdk():
    """不装 `litearm-python`、不读 `PYTHONPATH` / site-packages，也能 import。

    `-I` 忽略环境变量与用户 site，`-S` 跳过 site-packages。子进程里显式插入
    仓库根目录，断言落点就在 `litearm/` 下。
    """
    source = (
        "import sys; "
        f"sys.path.insert(0, {str(REPO_ROOT)!r}); "
        "import litearm; print(litearm.__file__)"
    )
    result = subprocess.run(
        [sys.executable, "-I", "-S", "-c", source],
        cwd=REPO_ROOT, capture_output=True, text=True,
    )
    assert result.returncode == 0, result.stderr
    assert Path(result.stdout.strip()).resolve().is_relative_to(VENDORED_SDK)


def test_the_sdk_exposes_the_entry_points_the_worker_uses():
    """随包副本必须是能跑本仓 worker 的那一版，而不是同名的旧包。"""
    module = Path(import_arm_sdk()).resolve()
    assert module.is_relative_to(VENDORED_SDK)

    import litearm

    for name in ("connect", "enable", "disable", "get_state",
                 "movej", "move_js", "joint_follow", "close"):
        assert hasattr(litearm.Arm, name), f"Arm 缺少 {name}()"
    assert (VENDORED_SDK / "py.typed").is_file()
    assert (VENDORED_SDK / "LICENSE").is_file()


def test_import_arm_sdk_rejects_a_copy_outside_the_vendored_directory(
        monkeypatch, tmp_path):
    monkeypatch.setitem(sys.modules, "litearm", foreign_module(tmp_path))
    with pytest.raises(ArmSdkUnavailable, match="不是随包的"):
        import_arm_sdk()


def test_import_arm_sdk_rejects_a_vendored_file_without_joint_follow(monkeypatch):
    module = types.ModuleType("litearm")
    module.__file__ = str(VENDORED_SDK / "__init__.py")
    module.Arm = object
    monkeypatch.setitem(sys.modules, "litearm", module)
    with pytest.raises(ArmSdkUnavailable, match="joint_follow"):
        import_arm_sdk()


def test_import_arm_sdk_reports_a_missing_copy(monkeypatch):
    monkeypatch.setitem(sys.modules, "litearm", None)
    with pytest.raises(ArmSdkUnavailable, match="导入失败"):
        import_arm_sdk()


def test_the_entry_point_exits_when_another_litearm_wins(
        monkeypatch, tmp_path, capsys):
    monkeypatch.setitem(sys.modules, "litearm", foreign_module(tmp_path))

    from liteteleop.__main__ import main

    assert main(["--help"]) == 2
    assert "臂 SDK 不可用" in capsys.readouterr().err
