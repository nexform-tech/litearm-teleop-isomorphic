# The vendored LiteGrip SDK

This note records where the `litegrip` package in this directory came from and how to move it to a new upstream commit. Read it before editing anything here.

The teleoperation worker drives the gripper through this SDK. Shipping it in-tree is what makes `python -m liteteleop --gcan can0` work on a machine that has only this repository; users do not install `litegrip-python`.

## What is vendored, and from where

| | |
| --- | --- |
| Upstream | <https://github.com/nexform-tech/litegrip-python> |
| Commit | `4ce8482e10f5ca12a12e90db2a7222f814b07cea` (tag `v1.2.1`, 2026-10-30) |
| License | Apache-2.0 — `LICENSE` is that commit's file, copied verbatim |

`litegrip/` holds the modules, the `can/` and `protocols/` subpackages, `py.typed`, and the three JSON data files the code resolves through `dirname(__file__)`: `factory_calibration.json`, `calibration_normal.json`, and `calibration_reverse.json`. The worker loads the factory file explicitly; the other two are upstream's legacy templates and must travel with the code.

Upstream's `tests/`, `.github/`, `.releaserc.json`, `AGENTS.md`, `README*`, `examples/`, `pyproject.toml`, and `__pycache__/` are deliberately not copied.

## Do not edit these files

An edit here is a fork of upstream that nothing records, and the next re-vendor copies each file wholesale and would silently drop it. Fix the SDK upstream and re-vendor, or record the divergence in this file with its own hash and exclude it from the tree-hash recipe.

`tests/test_vendored_sdk.py` recomputes the tree hash below and fails when a file no longer matches it.

## Tree hash

```text
0a085f3259227688ff5da535ba1079b3d02efe73f7c5aa56d3caf63d113cd029
```

The recipe: take every file under `litegrip/` except this note and `__pycache__`, ordered by its POSIX path relative to `litegrip/`; feed each path, a newline, then the file's bytes into one SHA-256 stream; read the hex digest.

```bash
python3 - <<'PY'
import hashlib, pathlib
root = pathlib.Path("litegrip")
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

Run from this repository's root. Replace `NEW_SHA` with the upstream commit you are moving to, and check out upstream first.

```bash
git -C ../litegrip-python fetch origin
git -C ../litegrip-python rev-parse origin/main          # the commit to record above
git -C ../litegrip-python archive --format=tar NEW_SHA src/litegrip \
    | tar -x -C . --strip-components=1
git -C ../litegrip-python show NEW_SHA:LICENSE > litegrip/LICENSE
```

Do not copy the checkout's working tree (`cp -r ../litegrip-python/src/litegrip .`). A checkout sits on whatever branch it was last switched to, and that branch decides the numbers in `factory_calibration.json`. Ask the commit instead:

```bash
git -C ../litegrip-python show NEW_SHA:src/litegrip/factory_calibration.json
```

Then, in one commit:

1. Update the commit and date in the table above.
2. Recompute the tree hash with the recipe above and replace the value.
3. Run the suite. `tests/test_vendored_sdk.py` checks the tree hash and that a clean interpreter imports this copy.
4. Read `git -C ../litegrip-python log --oneline OLD_SHA..NEW_SHA` before trusting the diff. A behaviour change that the worker does not test for will not announce itself.

## Known divergence: the reported version

`litegrip.__version__` reads the installed distribution's metadata through `importlib.metadata.version("litegrip-python")`, so it reports the version of whichever `litegrip-python` distribution is installed in the interpreter, which is not necessarily this code. A machine with no such installation reports `0.0.0+source`; a machine where an unrelated `litegrip-python` is installed reports that distribution's number.

This is upstream's design, copied unchanged. Nothing in this repository reads `litegrip.__version__`; the vendored commit above is the version that matters, and the tree hash pins it.
