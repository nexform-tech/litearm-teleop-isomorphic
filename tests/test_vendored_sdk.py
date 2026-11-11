"""随包发布的 `litegrip` SDK —— 来源、树哈希，以及"只靠本仓就能 import"。

本文件检查两件事：一份干净的 checkout 不装任何东西也能 `import litegrip`；
`litegrip/` 里的文件与 `litegrip/VENDORED.md` 记录的树哈希一致。后者能抓住
"就地改 SDK"——这种改动在下次 re-vendor 时会被静默覆盖。
"""
from __future__ import annotations

import hashlib
import json
import re
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
VENDORED_SDK = REPO_ROOT / "litegrip"
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
    assert match, "litegrip/VENDORED.md 里没有 `## Tree hash` 与 64 位哈希"
    return match.group(1)


def test_the_vendored_tree_matches_its_recorded_hash():
    """判别力：就地改 `litegrip/` 里任何一个字节，本用例红。"""
    assert tree_hash(VENDORED_SDK) == recorded_hash()


def test_a_clean_interpreter_imports_the_vendored_sdk():
    """不装 `litegrip-python`、不读 `PYTHONPATH` / site-packages，也能 import。

    `-I` 忽略环境变量与用户 site，`-S` 跳过 site-packages。子进程里显式插入
    仓库根目录，断言落点就在 `litegrip/` 下。
    """
    source = (
        "import sys; "
        f"sys.path.insert(0, {str(REPO_ROOT)!r}); "
        "import litegrip; print(litegrip.__file__)"
    )
    result = subprocess.run(
        [sys.executable, "-I", "-S", "-c", source],
        cwd=REPO_ROOT, capture_output=True, text=True,
    )
    assert result.returncode == 0, result.stderr
    assert Path(result.stdout.strip()).resolve().is_relative_to(VENDORED_SDK)


def test_the_data_files_the_sdk_resolves_by_path_ship_with_it():
    """轮子里必须带这些文件：SDK 用 `dirname(__file__)` 找它们。"""
    assert (VENDORED_SDK / "py.typed").is_file()
    assert (VENDORED_SDK / "calibration_normal.json").is_file()
    assert (VENDORED_SDK / "calibration_reverse.json").is_file()
    factory = VENDORED_SDK / "factory_calibration.json"
    data = json.loads(factory.read_text(encoding="utf-8"))
    assert data["calibrated"] is True
