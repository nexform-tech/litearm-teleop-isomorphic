# The vendored LiteArm SDK

This note records where the `litearm` package in this directory came from and how to move it to a new upstream commit. Read it before editing anything here.

The arm worker drives both arms through this SDK. Shipping it in-tree is what makes `python -m liteteleop --cdc /dev/ttyACM0` work on a machine that has only this repository; users do not clone or install `litearm-python`.

The copy lives under `liteteleop/_vendor/litearm/`, not at the top level, so a `litearm-python` that the user installed for another project can coexist in the same environment. `liteteleop/arm_sdk.py` puts this directory first on `sys.path` and drops any other `litearm` already imported in the process, so this application always loads the vendored copy.

## What is vendored, and from where

| | |
| --- | --- |
| Upstream | <https://github.com/nexform-tech/litearm-python> |
| Commit | `5f757686a11aebbbca808bfddf53aaff29a7976b` (tag `v1.3.0`, 2026-10-28) |
| License | Apache-2.0 — `LICENSE` is that commit's file, copied verbatim |

`liteteleop/_vendor/litearm/` holds the complete `src/litearm` tree at that commit: the modules, `py.typed`, and the `__main__.py` entry used by upstream's console script. Upstream's `tests/`, `examples/`, `docs/`, `.github/`, `README*`, `pyproject.toml`, and `__pycache__/` are deliberately not copied. This repository does not register the `litearm-python` console script.

The package needs `pyserial>=3.4` at runtime. `transport.py` imports it lazily, so `import litearm` alone works without it; opening a CDC port does not. The dependency is declared in this repository's `pyproject.toml`.

### Why v1.3.0 and not v1.4.0

v1.4.0 (`33dc2c64d8d1ff52cb0c545dc018760e5a99a5fb`, 2026-11-07) adds SDK-local kinematics and guarded pose PTP, together with `src/litearm/assets/`: about 70 MB of STL meshes and a URDF. This teleoperation stack does not call those features; it uses the CDC transport and `joint_follow` (`0x08`), which v1.3.0 already provides. Vendoring v1.4.0 would put tens of megabytes of unused binaries in this repository and in every wheel.

Do not "upgrade" to v1.4.0 by copying only the modules and leaving `assets/` behind. The new kinematics entry points resolve those files at runtime and would fail. Re-vendor the whole tree, and decide explicitly whether this package should ship the meshes or stay on the newest tagged release that does not need them.

## Do not edit these files

An edit here is a fork of upstream that nothing records, and the next re-vendor copies each file wholesale and would silently drop it. Fix the SDK upstream and re-vendor, or record the divergence in this file with its own hash and exclude it from the tree-hash recipe.

`tests/test_vendored_arm_sdk.py` recomputes the tree hash below and fails when a file no longer matches it.

## Tree hash

```text
48c736321566cfa30731026a65060c7986912568a228b4e9f13a9958522acbbb
```

The recipe: take every file under `liteteleop/_vendor/litearm/` except this note and `__pycache__`, ordered by its POSIX path relative to that directory; feed each path, a newline, then the file's bytes into one SHA-256 stream; read the hex digest.

```bash
python3 - <<'PY'
import hashlib, pathlib
root = pathlib.Path("liteteleop/_vendor/litearm")
h = hashlib.sha256()
for p in sorted(root.rglob("*")):
    if not p.is_file() or "__pycache__" in p.parts or p.name == "VENDORED.md":
        continue
    h.update((p.relative_to(root).as_posix() + "\n").encode())
    h.update(p.read_bytes())
print(h.hexdigest())
PY
```

## How to re-vendor

Run from this repository's root. The upstream remote is GitHub, so `gh api` fetches a commit without cloning it.

```bash
COMMIT=5f757686a11aebbbca808bfddf53aaff29a7976b
tmp=$(mktemp -d)
gh api "repos/nexform-tech/litearm-python/tarball/$COMMIT" > "$tmp/src.tar.gz"
mkdir "$tmp/extracted"
tar -xzf "$tmp/src.tar.gz" -C "$tmp/extracted" --strip-components=1
rsync -a --delete --exclude '__pycache__' --exclude 'VENDORED.md' \
    "$tmp/extracted/src/litearm/" liteteleop/_vendor/litearm/
cp "$tmp/extracted/LICENSE" liteteleop/_vendor/litearm/LICENSE
```

Do not copy a checkout's working tree. A checkout sits on whatever branch it was last switched to, and that branch decides which modules and version string are copied. Ask the commit instead.

Then, in one commit:

1. Update the commit and date in the table above.
2. Recompute the tree hash with the recipe above and replace the value.
3. Run the suite. `tests/test_vendored_arm_sdk.py` checks the tree hash and that a clean interpreter imports this copy.
4. Read the upstream log between the old and new commits before trusting the diff. A behaviour change that the worker does not test for will not announce itself.

## Version string

`litearm.__version__` is hardcoded to `2.1.0` at this commit, and upstream also reports `2.1.0` for v1.4.0. Do not use it to tell the two apart; the commit recorded above and the tree hash are the provenance.
