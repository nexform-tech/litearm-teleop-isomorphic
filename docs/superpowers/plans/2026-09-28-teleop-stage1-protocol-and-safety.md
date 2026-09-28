# 同构遥操 阶段一（真机 spike + 协议层 + 安全层）实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use
> superpowers:subagent-driven-development (recommended) or superpowers:executing-plans
> to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 交付同构遥操的第一层 —— 先真机证伪 `move_js` 伺服路径，再实现与 `litearm-python` 零耦合的线协议层（`wire.py`）与 zenoh
点对点链路层（`link.py`），以及与硬件无关的纯逻辑安全层（`safety.py`：`slew_target` 移植、软限位闸门、状态机、watchdog）。

**Architecture:** 本阶段**不碰 GUI、不碰 `ArmWorker`**（那是阶段二）。所有产出要么是纯函数（可离线 TDD），要么是独立可跑的 spike 脚本。`safety.py` 里的 `slew_target` 是**从
`pylitearm` 逐字移植**的 —— 见 spec §5.1，它不是新设计，是既有验证实现的搬运。层次上 `wire.py` / `link.py` / `safety.py` **互不依赖**，可以并行做；三者都**不 import
 `litearm`**（`safety.py` 只用 stdlib `math`），这样离线测试不需要硬件也不需要 SDK。

**Tech Stack:** Python 3.13、`eclipse-zenoh` 1.6.2（**按 `python3 -m pip` 量**）、`pytest`；spike 脚本额外需要
 `litearm-python`（`PYTHONPATH=/home/llx/litearm-python/src`）与一条 7 关节整臂。

**Spec:** `docs/superpowers/specs/2026-09-28-isomorphic-teleop-design.md`（**唯一权威**；本计划与 spec 冲突时以 spec 为准）

---

## ⚠ 维护本计划文档时必读

**本计划的代码块就是交付物。**改动本文档后，**必须真编译每个 ` ```python ` 块**，不能只数空行：

```bash
python3 - <<'PY'
import ast
blocks=[];inb=False;cur=[];lang=""
for L in open("docs/superpowers/plans/2026-09-28-teleop-stage1-protocol-and-safety.md",encoding="utf-8"):
    L=L.rstrip("\n")
    if L.startswith("```"):
        if not inb: inb=True;lang=L[3:].strip();cur=[]
        else: inb=False;blocks.append((lang,"\n".join(cur)))
        continue
    if inb: cur.append(L)
bad=0
for i,(l,b) in enumerate(blocks):
    if l!="python": continue
    try: ast.parse(b)
    except SyntaxError as e: bad+=1;print(f"#{i}: {e.msg} 行 {e.lineno}")
print(f"python 块 {sum(1 for l,_ in blocks if l=='python')} 个，失败 {bad}")
raise SystemExit(1 if bad else 0)
PY
```

**为什么写这条**：2026-09-28 我用 `\n{3,}\n\n` 批量折叠空行（为了过 MD012），**连围栏内的代码一起折了**，
把 `@dataclass(frozen=True)` 和它的 `class Frame:` 之间插进了 2 个空行 —— 装饰器语法直接坏掉。
而当时的自检只数空行（"残留 0 处"），**对"代码还能不能编译"零判别力**，于是静默通过。
**⇒ 判据要挑真正会坏的那个性质：含代码的文档，判据是「编译得过」，不是「格式看着对」。**

**⚠ 还差一条 —— 编译检查抓不到「整块内容凭空消失」。** 2026-09-28 同一轮里，一次编辑把 Task 5 的
**Step 2~4 与整个 `safety.py` 实现块（约 155 行）静默删掉**了；当时"所有 python 块都编译得过 + lint 0"
**全绿**（少一个块当然不会编译失败）。执行者 grep `def slew_target` 才发现。
⇒ **改完还要核对 Step 清单与规模**：

```bash
# 每个 Task 的 Step 序列必须连续；缺号即内容丢失
grep -n "^## Task \|^- \[ \] \*\*Step" docs/superpowers/plans/2026-09-28-teleop-stage1-protocol-and-safety.md
wc -l docs/superpowers/plans/2026-09-28-teleop-stage1-protocol-and-safety.md   # 与改动前对比，只减不增要问为什么
```

**元教训**：`[实测]` 判据要覆盖**"少了东西"**这个方向 —— 编译检查只覆盖"写错了"，
不覆盖"没写"。**内容完整性要用清单/规模核对，不能用"跑得通"。**

## 交付边界（阶段一不做什么）

- ❌ 不做 GUI、不做 `arm_worker.py`、不做 `safety.py` 的遥操循环接线（阶段二）
- ❌ 不 import `litearm`（除 spike 脚本）
- ❌ 不改 `litearm-python`、不改 `litearm-stm32`
- ❌ 不 push、不开 PR（等用户裁决）

## 分支与提交约定

- **本计划的所有产出提交到 `feat/teleop-stage1-protocol`**，从**当前 HEAD**（含 spec 的 `docs/isomorphic-teleop-design`）切出 —— 因为实现要以 spec 为依据，而 spec
  还没进 `main`。
- 提交信息用 Conventional Commits（仓规 `AGENTS.md`）：`feat:` / `test:` / `chore:` / `docs:`。
- **提交信息里永不出现 `claude` 字样**，不加 `Co-Authored-By`。
- **不 `amend`、不 `rebase`、不 `add .`、不 `commit -a`**；每步只 `git add` 本步明确列出的文件。
- 收尾建议（**不擅自执行**）：先把 `docs/isomorphic-teleop-design` 合进 `main`（`docs:` 不发版），再让本分支 rebase 到 `main`。

## 文件结构

| 文件 | 职责 | 依赖 |
| --- | --- | --- |
| `liteteleop/__init__.py` | 版本号、包导出 | — |
| `liteteleop/wire.py` | 线协议 v1 帧编解码（纯函数） | stdlib `struct` |
| `liteteleop/link.py` | zenoh 点对点封装（监听/连接、`close()` 纪律） | `zenoh` |
| `liteteleop/safety.py` | `slew_target` 移植 + 软限位闸门 + 钳位 + 状态机 + watchdog（纯逻辑） | stdlib `math` |
| `tests/test_wire.py` | 黄金字节、版本/长度/关节数拒收 | `pytest` |
| `tests/test_link.py` | 真 zenoh 回环、`close()` 纪律（子进程） | `pytest` |
| `tests/test_safety.py` | `slew_target` 与 pylitearm 原版对拍、闸门、状态机 | `pytest` |
| `scripts/spike_move_js.py` | 真机 spike S1~S5 | `litearm` |
| `docs/spike-2026-09-28-move-js.md` | spike 原始结论（回写 spec §2.3 / §5.1 的依据） | — |

**为什么 `safety.py` 一个文件放四样东西**：它们全是同一件事（"从主臂 q 到从臂命令"的纯逻辑），且互相耦合紧密（状态机调钳位、钳位调限位、watchdog 触发状态迁移）。拆开只会让人来回跳文件。**但它们必须在 `safety.py`
 内部保持清晰的函数边界**，每个函数可单独测。

---

## Task 1: 仓库骨架与分支

**Files:**

- Create: `liteteleop/__init__.py`
- Create: `tests/__init__.py`（空文件，让 pytest 稳定发现）
- Create: `pyproject.toml`（**本仓当前没有此文件**，本步新建）

- [x] **Step 1: 确认在正确的分支上**

> ⚠ 分支 `feat/teleop-stage1-protocol` **由控制器（主会话）预先建好**，执行者**不要再建** ——
> `git switch -c` 会报 `already exists`。

```bash
cd /home/llx/litearm-teleop-isomorphic
git branch --show-current
```

Expected: 输出 `feat/teleop-stage1-protocol`。若不是 ⇒ **停下来报 BLOCKED**，别自己切分支。

- [x] **Step 2: 建包骨架**

```bash
mkdir -p liteteleop tests scripts
```

`liteteleop/__init__.py`：

```python
"""liteteleop —— 同构遥操上位机（主臂零重力拖动 → zenoh 点对点 → 从臂 move_js 跟随）。

设计依据见 `docs/superpowers/specs/2026-09-28-isomorphic-teleop-design.md`。
本包**不依赖 litearm / pylitearm**：线协议与安全逻辑都是纯 Python。
"""
__version__ = "0.0.0"
```

`tests/__init__.py`：空文件（0 字节）。

`pyproject.toml` —— **本仓当前没有这个文件**，本步创建（只放最小可用元数据，不加构建后端；
本阶段只是包，还不发版）：

```toml
[project]
name = "liteteleop"
version = "0.0.0"
description = "LiteArm 同构遥操上位机（主臂零重力拖动 → zenoh 点对点 → 从臂 move_js 跟随）"
requires-python = ">=3.10"
dependencies = ["zenoh>=1.6", "PyQt5>=5.15"]      # PyQt5 阶段二才用，先声明

[project.optional-dependencies]
dev = ["pytest>=8"]

[tool.pytest.ini_options]
testpaths = ["tests"]
markers = ["slow: 慢测（起子进程 / 数十秒），用 -m \"not slow\" 跳过"]
```

> ⚠ `version = "0.0.0"` 是**占位**：仓规 `AGENTS.md` 规定版本由 semantic-release 从 git tag 推导，
> **这个字段永不手工改**。
> ⚠ `markers` 与 `testpaths` 在这里一次配好，Task 4 只做确认、不再改此文件。

- [x] **Step 3: 确认 pytest 能发现空套件**

```bash
cd /home/llx/litearm-teleop-isomorphic
python3 -m pytest tests -q
```

Expected: `no tests ran`（退出码 5）—— 这是**预期的**，说明 pytest 能跑；套件为空是正常的。

- [x] **Step 4: 提交**

```bash
git add liteteleop/__init__.py tests/__init__.py
git commit -m "chore: scaffold liteteleop package"
```

---

## Task 2: 真机 spike（S1~S5）

> ⛔ **必须在 7 关节整臂固件上做，不许用 `LITEARM_BENCH_1J` 台架**（spec §11：台架 `hold_kp_gain` 未设 ⇒ 0，`ht_on` 会下发 `kp=0`，S5 结论无效；台架 `kd`/`tau_max`
> 也是另一套值）。
> ⛔ **臂此刻接不上时跳过本 Task，先做 Task 3~6**（它们与 `dq` 语义无关，不会白做），回头再补。
> ⚠ **本 Task 会真的让臂动**（S2 移动 0.2 rad、S3/S5 各跑 30 s）。开跑前确认：臂周围无障碍、急停可及、**不是 1J 台架**。

**Files:**

- Create: `scripts/spike_move_js.py`
- Create: `docs/spike-2026-09-28-move-js.md`（跑完填）

### 背景（写代码前必读）

`move_js(q, dq)` 的 `dq` 有两个角色（spec §2.3(a)/(a′)）：

1. `|dq|` 决定固件里 `q_ref` 的**走位速率**（`v_lim = clamp(|target_dq|, 0, speed_limit·gov_ratio)`）⇒ **`dq=0` 则该轴一步都不动**。
2. `dq` **本身**原样进电机 MIT 帧的速度项 ⇒ 它同时是**速度前馈**。

已知实证（2026-09-20，`litearm-server/scripts/e2e_movejs_real.py`）：`dq=0` 时臂纹丝不动、`dq` 取解析导数时跟踪误差 rms 0.0043 rad。**但那个脚本刻意把 `|dq|` 压在 0.5
rad/s 以下（实际峰值 0.032）** —— 遥操跟随会跑到 1~2 rad/s，**S3 补的就是这个空档**。

- [ ] **Step 1: 写 spike 脚本**

`scripts/spike_move_js.py`：

```python
#!/usr/bin/env python3
"""真机 spike S1~S5 —— 验证 move_js 伺服路径（同构遥操的地基）。

用法:
    PYTHONPATH=/home/llx/litearm-python/src python3 scripts/spike_move_js.py --port /dev/ttyACM0
    # 加 --yes 跳过交互确认

⚠ 会真的让臂动。跑前确认：臂周围无障碍、急停可及、**是 7 关节整臂不是 1J 台架**。
依据: docs/superpowers/specs/2026-09-28-isomorphic-teleop-design.md §2.3 / §11
"""
from __future__ import annotations

import argparse
import math
import statistics
import sys
import time

import litearm as pa

SEND_HZ = 100.0
DT = 1.0 / SEND_HZ
ALIGN_SPEED = 0.3

# S1: dq=0 冻结
S1_OFFSET = 0.10
S1_SEC = 2.0
S1_MOVE_TOL = 0.005          # rad，超过就算"动了"（判据必须能失败）
# S2: dq 幅值 -> 实测速率
S2_OFFSET = 0.20
S2_DQ = 0.30
S2_RATE_TOL = 0.20           # ±20%
# S3: 遥操速度量级（既有实证的上限是 0.5，这里要越过它）
S3_HZ = 0.5
S3_AMP = 0.25
S3_SEC = 20.0
S3_RMS_TOL = 0.05            # rad
# S4: 100 Hz ACK 稳定性
S4_SEC = 30.0
# S5: 收尾 A/B 对照
S5_DRIFT_SEC = 30.0

results: list[tuple] = []


def check(name: str, ok: bool, detail: str = "") -> bool:
    results.append((name, ok, detail))
    print(f"  {'✓' if ok else '✗'} {name}" + (f"  [{detail}]" if detail else ""), flush=True)
    return ok


def read_q(arm):
    """读实测 q（**等一帧新的**）。⚠ get_state() 返回 Msg 信封，取 .value（spec §2.2）。"""
    st = arm.get_state(refresh=True).value
    if st is None:
        raise RuntimeError("取不到状态帧")
    return list(st.q)


def read_q_cached(arm):
    """读缓存里的最近一帧 q —— **不取帧**，用于 100 Hz 紧循环（取帧会拖垮发送节拍）。"""
    st = arm.get_state(refresh=False).value
    if st is None:
        raise RuntimeError("状态缓存为空（链路刚起？）")
    return list(st.q)


def spin(arm, q, dq, secs: float) -> None:
    """按 100 Hz 喂 move_js，持续 secs 秒。q/dq 为序列。"""
    t0 = time.monotonic()
    nxt = t0 + DT
    while time.monotonic() - t0 < secs:
        arm.move_js(list(q), list(dq))
        r = nxt - time.monotonic()
        if r > 0:
            time.sleep(r)
        nxt += DT
        if nxt < time.monotonic():
            nxt = time.monotonic() + DT


def s1_dq_zero_freezes(arm, q0):
    print("\n=== S1: dq=0 发偏移 —— 按控制律应【纹丝不动】 ===", flush=True)
    tgt = [x for x in q0]
    tgt[1] += S1_OFFSET
    qs = []
    t0 = time.monotonic()
    nxt = t0 + DT
    while time.monotonic() - t0 < S1_SEC:
        arm.move_js(tgt, [0.0] * arm.n)
        qs.append(read_q_cached(arm))
        r = nxt - time.monotonic()
        if r > 0:
            time.sleep(r)
        nxt += DT
        if nxt < time.monotonic():
            nxt = time.monotonic() + DT
    moved = max(abs(qs[-1][i] - qs[0][i]) for i in range(arm.n))
    check(f"S1 dq=0 时臂没动 (|Δq| < {S1_MOVE_TOL})", moved < S1_MOVE_TOL, f"{moved:.5f} rad")


def _rate_probe(arm, q0, dq_cmd, offset):
    """发 `dq=dq_cmd` 的偏移，量【运动进行中】的实测速率。

    ⛔ **测量窗口必须落在运动期间**：走完 `offset/dq_cmd` 秒后臂就停了，
    在停住之后量只会得到 0 —— 本计划第一版就是这么错的（命令 0.30 rad/s、0.2 rad
    在 **0.667 s** 走完，而窗口取的是 1.25~1.75 s）。
    """
    tgt = [x for x in q0]
    tgt[1] += offset
    arm.movej(q0, speed=ALIGN_SPEED)
    time.sleep(0.3)
    move_s = offset / max(dq_cmd, 1e-6)
    qs, ts = [], []
    t0 = time.monotonic()
    nxt = t0 + DT
    while time.monotonic() - t0 < move_s * 0.8:      # 只跑到走完的 80%
        arm.move_js(tgt, [dq_cmd] * arm.n)
        st = arm.get_state(refresh=False).value
        if st is None:
            raise RuntimeError("状态缓存为空 —— 见 spec §5.1：拿不到实测 q 不许下发/判据")
        qs.append(list(st.q))
        ts.append(time.monotonic())
        r = nxt - time.monotonic()
        if r > 0:
            time.sleep(r)
        nxt += DT
        if nxt < time.monotonic():
            nxt = time.monotonic() + DT
    # 用【后 60%】的窗口：跳过头几拍的伺服起步滞后
    lo = max(0, int(len(qs) * 0.4))
    hi = len(qs) - 1
    moved = qs[hi][1] - qs[lo][1]
    return moved / max(ts[hi] - ts[lo], 1e-6), moved


def s2_dq_sets_rate(arm, q0):
    print("\n=== S2: dq 幅值 → 实测走位速率（含饱和） ===", flush=True)
    for dq_cmd, offset in ((S2_DQ, S2_OFFSET), (1.0, 0.5)):
        rate, moved = _rate_probe(arm, q0, dq_cmd, offset)
        # ⭐ 先证明"确实动了" —— 否则速率≈0 也能"接近"某些期望值，判据会失去意义
        check(f"S2 dq={dq_cmd} 确实驱动了运动", abs(moved) > 0.3 * offset,
              f"位移 {moved:.4f} rad")
        ok = abs(rate - dq_cmd) <= S2_RATE_TOL * dq_cmd
        check(f"S2 dq={dq_cmd} 时实测速率 ≈{dq_cmd} rad/s (±{S2_RATE_TOL:.0%})",
              ok, f"{rate:.3f} rad/s")
    # 饱和：dq 远高于 speed_limit 时实测速率被固件的 speed_limit 封住（本机 J1=2.0）
    rate, _ = _rate_probe(arm, q0, 5.0, 1.0)
    check("S2 dq=5.0 被 speed_limit 封顶（实测 < 3.0 rad/s）", rate < 3.0,
          f"{rate:.3f} rad/s（J1 speed_limit=2.0）")


def s3_high_speed_feedforward(arm, q0):
    print(f"\n=== S3: 遥操速度量级（正弦 {S3_AMP}rad @{S3_HZ}Hz）下的速度前馈 ===", flush=True)
    print("     既有实证的上限是 0.5 rad/s；本节峰值 dq 约 "
          f"{2 * math.pi * S3_HZ * S3_AMP:.2f} rad/s", flush=True)
    # 用 spec §5.1 的 slew_target 生成参考（与阶段二同一份代码，但此处先内联，
    # 避免 spike 依赖还没实现的 safety.py；Task 5 会把它抽出去）
    arm.movej(q0, speed=ALIGN_SPEED)
    time.sleep(0.3)
    peak_dq = 2 * math.pi * S3_HZ * S3_AMP
    q_cmd = list(q0)
    dq_cmd = [0.0] * arm.n
    errs = []
    t0 = time.monotonic()
    nxt = t0 + DT
    while time.monotonic() - t0 < S3_SEC:
        t = time.monotonic() - t0
        raw = [x for x in q0]
        raw[1] += S3_AMP * math.sin(2 * math.pi * S3_HZ * t)
        # 梯形曲线：限速 peak_dq*1.35、限加速 4*peak_dq（足够跟得上，不做瓶颈）
        q_cmd, dq_cmd = _slew(raw, q_cmd, dq_cmd, peak_dq * 1.35, 4.0 * peak_dq, DT)
        arm.move_js(q_cmd, dq_cmd)
        st = arm.get_state(refresh=False).value
        errs.append(max(abs(q_cmd[i] - st.q[i]) for i in range(arm.n)))
        r = nxt - time.monotonic()
        if r > 0:
            time.sleep(r)
        nxt += DT
        if nxt < time.monotonic():
            nxt = time.monotonic() + DT
    # 丢掉前 20% 的起步段
    body = errs[len(errs) // 5:]
    rms = statistics.fmean([e * e for e in body]) ** 0.5
    check(f"S3 跟踪误差 rms ≤ {S3_RMS_TOL} rad（峰值 dq≈{peak_dq:.2f}）",
          rms <= S3_RMS_TOL, f"rms={rms:.4f} rad, max={max(body):.4f}")
    # 抖动判据：后半段误差的峰峰值不应比前半段显著放大（自激会单调放大）
    h = len(body) // 2
    pp1 = max(body[:h]) - min(body[:h])
    pp2 = max(body[h:]) - min(body[h:])
    check("S3 无自激（后半段误差峰峰值未显著放大）", pp2 <= pp1 * 2.0 + 0.01,
          f"前半 {pp1:.4f} / 后半 {pp2:.4f}")


def _slew(raw_target, q_cmd, dq_cmd, speed_limit, accel_limit, dt):
    """spike 内联版 slew_target —— 与 pylitearm/src/pylitearm/control/joint_follow.py:45-88 逐字同构。

    Task 5 会把正式版放进 liteteleop/safety.py 并与原版对拍；
    此处内联只是为了让 spike 不依赖尚未实现的安全层。
    """
    dt = max(dt, 1e-4)
    for i in range(len(raw_target)):
        v_limit = max(1e-4, speed_limit)
        a_limit = max(1e-4, accel_limit)
        dv_max = a_limit * dt
        diff = raw_target[i] - q_cmd[i]
        v = dq_cmd[i]
        if abs(diff) < 1e-5 and abs(v) < dv_max:
            q_cmd[i] = raw_target[i]
            dq_cmd[i] = 0.0
            continue
        stopping_dist = (v * v) / (2.0 * a_limit) if a_limit > 0.0 else 0.0
        moving_toward = diff * v > 0.0
        if moving_toward and abs(diff) <= stopping_dist:
            desired_v = 0.0
        else:
            desired_v = math.copysign(v_limit, diff)
        v += max(-dv_max, min(dv_max, desired_v - v))
        v = max(-v_limit, min(v_limit, v))
        step = v * dt
        if diff * step > 0.0 and abs(step) >= abs(diff):
            q_cmd[i] = raw_target[i]
            dq_cmd[i] = 0.0
        else:
            q_cmd[i] += step
            dq_cmd[i] = v
    return q_cmd, dq_cmd


def s4_ack_rate(arm, q0):
    print(f"\n=== S4: {SEND_HZ:.0f} Hz move_js 连续 {S4_SEC:.0f}s 的 ACK 返回率 ===", flush=True)
    sent, ack_fail, worst_ms = 0, 0, 0.0
    t0 = time.monotonic()
    nxt = t0 + DT
    while time.monotonic() - t0 < S4_SEC:
        t = time.monotonic()
        try:
            arm.move_js(list(q0), [0.0] * arm.n)   # dq=0 ⇒ 不发散，纯测 ACK
            dt_ms = (time.monotonic() - t) * 1000.0
            worst_ms = max(worst_ms, dt_ms)
        except Exception as e:                      # noqa: BLE001 - spike 要看到全部失败
            ack_fail += 1
            if ack_fail <= 3:
                print(f"    ✗ 第 {sent} 帧异常: {type(e).__name__}: {e}", flush=True)
        sent += 1
        r = nxt - time.monotonic()
        if r > 0:
            time.sleep(r)
        nxt += DT
        if nxt < time.monotonic():
            nxt = time.monotonic() + DT
    el = time.monotonic() - t0
    rate = sent / el
    ok_rate = (sent - ack_fail) / max(sent, 1)
    check("S4 ACK 成功率 ≥ 99.9%", ok_rate >= 0.999, f"{ok_rate:.4%} ({ack_fail}/{sent} 失败)")
    check("S4 实际下发频率 ≥ 95 Hz", rate >= 95.0, f"{rate:.1f} Hz")
    # _cmd 超时是 1.2 s ⇒ 任何一次超时都会表现为一帧耗时逼近 1200 ms
    check("S4 无 1.2s 级卡顿", worst_ms < 1200.0, f"最坏 {worst_ms:.1f} ms")


def s5_stop_ab(arm, q0):
    print(f"\n=== S5: 收尾 A/B —— 只停发 vs movej(q_now)，各录 {S5_DRIFT_SEC:.0f}s ===", flush=True)

    def drift_after(finish):
        arm.movej(q0, speed=ALIGN_SPEED)
        time.sleep(0.5)
        spin(arm, q0, [0.0] * arm.n, 1.0)          # 进 MOVE_JS 态
        q_a = read_q(arm)
        finish()
        t0 = time.monotonic()
        drift = 0.0
        while time.monotonic() - t0 < S5_DRIFT_SEC:
            try:
                drift = max(drift, max(abs(read_q_cached(arm)[i] - q_a[i]) for i in range(arm.n)))
            except Exception:                       # noqa: BLE001
                pass
            time.sleep(0.5)
        return drift

    d_a = drift_after(lambda: None)                                  # A: 只停发
    print(f"    A 只停发:       漂移 {d_a:.4f} rad", flush=True)
    d_b = drift_after(lambda: arm.movej(read_q(arm), speed=ALIGN_SPEED))  # B: movej 收尾
    print(f"    B movej(q_now): 漂移 {d_b:.4f} rad", flush=True)
    check("S5 B（movej 收尾）漂移显著小于 A（只停发）", d_b < d_a * 0.5,
          f"A={d_a:.4f} B={d_b:.4f} 比值 {d_b / max(d_a, 1e-9):.2f}")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=None, help="不填则自动找 CDC 口")
    ap.add_argument("--skip", default="", help="逗号分隔要跳过的节，如 S3,S5")
    ap.add_argument("--yes", action="store_true", help="跳过交互确认")
    a = ap.parse_args(argv)

    skip = {s.strip().upper() for s in a.skip.split(",") if s.strip()}
    # ⚠ move_timeout 必须显式传并断言（spec §5.3）—— 默认 15s，
    # 一次链路故障就能把收尾阻塞 15 秒
    arm = pa.Arm(port=a.port, move_timeout=3.0)
    arm.connect()
    assert arm.move_timeout == 3.0, f"move_timeout 被改成 {arm.move_timeout} —— 见 spec §5.3"
    print(f"已连接: n={arm.n}", flush=True)
    if arm.n != 7:
        print(f"⛔ 本 spike 只对 7 关节整臂有效，当前 n={arm.n} —— 疑似 1J 台架，退出", flush=True)
        arm.close()
        return 2
    if not a.yes:
        input("确认臂周围无障碍、急停可及、是 7 关节整臂。回车继续，Ctrl-C 中止: ")
    arm.enable()
    st = arm.get_state(refresh=True).value
    if st.faulted or st.joint_fault:
        print(f"⛔ 起始状态不干净: {st.fault_detail}", flush=True)
        arm.close()
        return 3
    q0 = list(st.q)
    print(f"起点 q = {[round(x, 4) for x in q0]}", flush=True)

    try:
        if "S1" not in skip:
            s1_dq_zero_freezes(arm, q0)
        if "S2" not in skip:
            s2_dq_sets_rate(arm, q0)
        if "S3" not in skip:
            s3_high_speed_feedforward(arm, q0)
        if "S4" not in skip:
            s4_ack_rate(arm, q0)
    finally:
        # 收尾纪律（spec §5.3）：受控接管，绝不 disable（那是失能自由落体）
        try:
            arm.movej(read_q(arm), speed=ALIGN_SPEED)
        except Exception as e:                      # noqa: BLE001
            print(f"⚠ 收尾 movej 失败: {e}", flush=True)
    if "S5" not in skip:
        try:
            s5_stop_ab(arm, q0)
        finally:
            try:
                arm.movej(read_q(arm), speed=ALIGN_SPEED)
            except Exception as e:                  # noqa: BLE001
                print(f"⚠ 收尾 movej 失败: {e}", flush=True)

    # ⚠ arm.close() 必须**无条件**执行（spec §7.3）：中途抛异常也不能跳过它，
    # 否则进程退出会挂死。
    try:
        print("\n================ 结果 ================", flush=True)
        for name, ok, detail in results:
            print(f"{'✓' if ok else '✗'} {name}" + (f"  [{detail}]" if detail else ""), flush=True)
        n_fail = sum(1 for _, ok, _ in results if not ok)
        print(f"\n{len(results) - n_fail}/{len(results)} 通过", flush=True)
    finally:
        arm.close()
    return 1 if n_fail else 0

if __name__ == "__main__":
    sys.exit(main())
```

- [ ] **Step 2: 冒烟（不接臂，确认脚本语法与参数解析没问题）**

```bash
cd /home/llx/litearm-teleop-isomorphic
python3 -c "import ast,sys; ast.parse(open('scripts/spike_move_js.py').read()); print('语法 OK')"
python3 -m pytest tests -q
```

Expected: `语法 OK`；pytest 仍 `no tests ran`。

- [ ] **Step 3: 真机跑（**唯一需要人的一步**）**

```bash
cd /home/llx/litearm-teleop-isomorphic
PYTHONPATH=/home/llx/litearm-python/src python3 scripts/spike_move_js.py --port /dev/ttyACM0
```

Expected: 逐节打印 `✓/✗`。**记下完整的原始输出** —— 下一步要抄进 spike 报告。

**先跑 `--skip S3,S5` 确认 S1/S2/S4 通过**，再单独跑 S3、S5（它们耗时最长、动得最多）：

```bash
PYTHONPATH=/home/llx/litearm-python/src python3 scripts/spike_move_js.py --skip S3,S5 --yes
PYTHONPATH=/home/llx/litearm-python/src python3 scripts/spike_move_js.py --skip S1,S2,S4 --yes
```

- [ ] **Step 4: 写 spike 报告**

`docs/spike-2026-09-28-move-js.md`：

```markdown
# 真机 spike：move_js 伺服路径（S1~S5）

> 日期：<实际执行日期>
> 板卡与固件版本串：<必填！spec §11 要求写明，否则结论不可比>
> 臂：7 关节整臂（**不是** LITEARM_BENCH_1J 台架）
> 脚本：`scripts/spike_move_js.py`
> 依据：spec §2.3 / §11

## 原始输出

<把 Step 3 的完整输出粘这里，不删不改>

## 逐条结论

| # | 验什么 | 结论 | 实测值 |
| --- | --- | --- | --- |
| S1 | `dq=0` 冻结该轴 | | |
| S2 | `dq` 幅值 → 走位速率 | | |
| S3 | 遥操速度量级下的速度前馈 | | |
| S4 | 100 Hz ACK 返回率 | | |
| S5 | 收尾 A/B（只停发 vs `movej`） | | |

## 与 spec 的关系

- S1 通过 ⇒ spec §2.3(a) 的「`dq=0` 冻结」**由 `[源码]` 升级为 `[实测]`**。
- ⛔ **本 spike 不覆盖**：spec §2.3(b) 的刚度三段表（S5 只量了 A/B 净漂移差，没量刚度）、
  spec §5.3 的**主臂**收尾序列（脚本从不进零重力，`zero_g_*`/`park` 一次没调）、
  §2.3(d) 急停的失能后果。**这三项留待另一次真机验证**，别顺手升级它们。
- **S3 是本轮的核心未知**（spec §11）。若 S3 未通过 ⇒ **停下来找用户裁决**：
  spec §5.1 的 `dq` 公式要改，候选退路见 spec §11 的 S3 段落。
```

- [ ] **Step 5: 回写 spec 的来源等级（**只写 spike 真正验过的东西**）**

Step 4 的结论**只覆盖下表**，别扩大：

| 由 spike 升级为 `[实测]` | 依据 |
| --- | --- |
| `dq = 0` ⇒ 该轴冻结（`\|Δq\| < 0.005 rad`） | S1 |
| `dq` 幅值 ⇒ 走位速率；超过 `speed_limit` 后饱和 | S2 |
| 遥操速度量级（**>0.5 rad/s**）下速度前馈的跟踪误差与稳定性 | S3 |
| 100 Hz `move_js` 连续 30 s 的 ACK 返回率 | S4 |
| **从臂**收尾：`movej(q_now)` 的漂移显著小于只停发 | S5 |

⛔ **spike 没测、必须保持 `[源码]` 的**（**别顺手一起改**，那是伪造证据）：

- **§2.3(b) 的刚度三段表**（1.0× / 0.6× / 2× 与 `ht_on` 的重力前馈）—— S5 只量了 A/B 的
  **净漂移差**，既没量刚度，也没把 1.0× 与 0.6× 两段分开量。
- **§5.3 的「主臂」收尾序列**（`park()` → `zero_g_stop()` → `movej`）—— 本脚本**从不进零重力**
  （grep 可见：`zero_g_start`/`zero_g_stop`/`park` 一次都没调），主臂那条路径**一行都没验**。
- **急停的失能后果**（§2.3(d)）—— 没测，也不该在 spike 里测。

⇒ 所以：**§2.3(b) 与 §5.3 里没有 `[源码]` 标记可改** —— 标记只挂在 §2.2 / §2.3 的
**节标题**上（grep 可查）。本次唯一该改的是 **§2.3(a) 那条「`dq=0` 冻结」**，
从 `[源码]` 升级为 `[实测]`，并在 §2.3(a) 处加一行指向本报告。
**主臂收尾与刚度表留待另一次真机验证**（阶段二，或单独一轮）—— 在 spec 里显式记一笔，
不要让它静默地一直挂着 `[源码]` 而没人知道。

- [ ] **Step 6: 提交**

```bash
git add scripts/spike_move_js.py docs/spike-2026-09-28-move-js.md docs/superpowers/specs/2026-09-28-isomorphic-teleop-design.md
git commit -m "test: add move_js real-machine spike and its findings"
```

> ⛔ **S3 没过时的动作**：**不要继续 Task 3 以后的实现**，也不要自行改 spec。停下来，
> 把 spike 报告与"卡在哪一条判据"报给用户。

---

## Task 3: 线协议 `wire.py`

**Files:**

- Create: `liteteleop/wire.py`
- Test: `tests/test_wire.py`

**为什么测试用黄金字节而不是往返**：往返测试（`decode(encode(x)) == x`）**对字节序没有判别力** —— 两端都用大端也全绿。而线协议跨语言、跨机器，字节序错了就是全错。所以测试里**硬编码** `struct`
 格式串与期望的十六进制串；实现若换字节序/换字段顺序，测试立刻红。

- [x] **Step 1: 写失败测试**

`tests/test_wire.py`：

```python
"""线协议 v1 —— 黄金字节钉死布局（见 spec §4.2）。

⚠ 这里**刻意不用往返测试**：往返对字节序没有判别力（两端同错也全绿）。
期望值由**独立于实现**的字面格式串算出，实现改动布局时先红。
"""
import struct

import pytest

from liteteleop import wire


def test_golden_bytes_n7():
    """n=7 的黄金字节 —— 小端、字段顺序、70 B 全长，全部钉死。"""
    q = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]
    dq = [0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07]
    got = wire.encode(q, dq, ts=1234.5, seq=42)
    # 期望由字面格式串独立算出，不引用 wire 里的任何常量
    want = struct.pack("<BB7f7fdI", 1, 7, *q, *dq, 1234.5, 42)
    assert got == want
    assert got.hex() == (
        "0107cdcccc3dcdcc4c3e9a99993ecdcccc3e0000003f9a99193f3333333f"
        "0ad7233c0ad7a33c8fc2f53c0ad7233dcdcc4c3d8fc2753d295c8f3d"
        "00000000004a93402a000000"
    )
    assert len(got) == 70


def test_golden_bytes_n1():
    """变长关节数：n=1 也成立（1J 台架/单轴测试用）。"""
    got = wire.encode([0.25], [0.5], ts=1.0, seq=7)
    assert got == struct.pack("<BB1f1fdI", 1, 1, 0.25, 0.5, 1.0, 7)
    assert got.hex() == "01010000803e0000003f000000000000f03f07000000"
    assert len(got) == 22


def test_roundtrip_values():
    q = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7]
    dq = [1.0, -1.0, 2.0, -2.0, 3.0, -3.0, 4.0]
    f = wire.decode(wire.encode(q, dq, ts=99.25, seq=65535), expect_n=7)
    assert f.n == 7
    assert f.q == pytest.approx(q)
    assert f.dq == pytest.approx(dq)
    assert f.ts == 99.25
    assert f.seq == 65535


def test_reject_bad_version():
    bad = bytearray(wire.encode([0.0] * 7, [0.0] * 7, ts=0.0, seq=0))
    bad[0] = 2                                   # version 改成不认识的
    with pytest.raises(wire.WireError, match="协议版本"):
        wire.decode(bytes(bad), expect_n=7)


def test_reject_n_mismatch():
    """帧里 n=3、本端 7 ⇒ 必须拒收，不许照 n 去解一帧垃圾。"""
    payload = wire.encode([0.0] * 3, [0.0] * 3, ts=0.0, seq=0)
    with pytest.raises(wire.WireError, match="关节数不符"):
        wire.decode(payload, expect_n=7)


def test_reject_short_frame():
    payload = wire.encode([0.0] * 7, [0.0] * 7, ts=0.0, seq=0)
    with pytest.raises(wire.WireError, match="帧长不符"):
        wire.decode(payload[:-1], expect_n=7)
    with pytest.raises(wire.WireError, match="帧太短"):
        wire.decode(b"\x01", expect_n=7)


def test_reject_bad_n_at_encode():
    with pytest.raises(wire.WireError, match="关节数"):
        wire.encode([], [], ts=0.0, seq=0)
    with pytest.raises(wire.WireError, match="长度不符"):
        wire.encode([0.0] * 7, [0.0] * 6, ts=0.0, seq=0)


def test_bad_n_in_frame():
    """帧里 n=0 / n=200 ⇒ 拒收（防越界与畸形头）。"""
    with pytest.raises(wire.WireError, match="关节数"):
        wire.decode(bytes([1, 0]) + b"\x00" * 8, expect_n=None)
    with pytest.raises(wire.WireError, match="关节数"):
        wire.decode(bytes([1, 200]) + b"\x00" * 8, expect_n=None)
```

- [x] **Step 2: 跑测试确认失败**

```bash
cd /home/llx/litearm-teleop-isomorphic
python3 -m pytest tests/test_wire.py -q
```

Expected: **collection error** —— 模块尚不存在。实测文案随 pytest 版本而异
（`ModuleNotFoundError: No module named ...` 或 `ImportError: cannot import name 'wire' from 'liteteleop'`）
⇒ **两者都算通过**；判据是「收集期就失败」，不是那条具体文案。

- [x] **Step 3: 实现**

`liteteleop/wire.py`：

```python
"""线协议 v1 —— 主臂 → 从臂的关节角流帧（见 spec §4.2）。

布局（**小端**，变长关节数）::

    offset  size   field
    0       1      version = 1
    1       1      n                 关节数 (1..MAX_JOINTS)
    2       4n     q[n]    f32 LE    主臂实测关节角 rad
    2+4n    4n     dq[n]   f32 LE    主臂实测关节速度 rad/s
    2+8n    8      ts      f64 LE    主臂 time.monotonic()
    2+8n+8  4      seq     u32 LE    主臂帧序号

n = 7 时共 70 B。

⚠ `version` 字节是防「静默解错」的那道判据：两端版本不符时**拒收**，
而不是照 `n` 去解一帧垃圾。
⚠ `ts` 取主臂的 `time.monotonic()`：跨机两端时钟不同源 ⇒ 它**只能**算同一主臂
相邻两帧的间隔，**不能**算端到端延迟（spec §4.2）。
⚠ 本格式与 litearm-server 的 `litearm/v4/{arm_id}/teleop`（`>15d` 120 B）**不兼容**，
key 也不同，别混用。
"""
from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Sequence, Tuple

__all__ = ["VERSION", "MAX_JOINTS", "WireError", "Frame", "encode", "decode", "frame_size"]

#: 协议版本。**改布局必须同时改它**（否则两端会照旧格式静默解错）。
VERSION = 1

#: 关节数上界 —— 防畸形头导致的越界分配。7 关节臂 + 余量。
MAX_JOINTS = 32


class WireError(ValueError):
    """帧不合法（版本不符 / 长度不符 / 关节数不符）。"""


@dataclass(frozen=True)
class Frame:
    """一帧解出来的值。`q`/`dq` 是 tuple（不可变，避免跨线程被就地改）。"""

    q: Tuple[float, ...]
    dq: Tuple[float, ...]
    ts: float
    seq: int

    @property
    def n(self) -> int:
        return len(self.q)


def _fmt(n: int) -> str:
    return f"<BB{n}f{n}fdI"


def frame_size(n: int) -> int:
    """按关节数算帧长（收端用来校验长度）。"""
    return struct.calcsize(_fmt(n))


def encode(q: Sequence[float], dq: Sequence[float], ts: float, seq: int) -> bytes:
    """编码一帧。`q`/`dq` 长度必须相等且在 `1..MAX_JOINTS`。"""
    n = len(q)
    if n != len(dq):
        raise WireError(f"q/dq 长度不符: {n} vs {len(dq)}")
    if not 1 <= n <= MAX_JOINTS:
        raise WireError(f"关节数 {n} 越界 (1..{MAX_JOINTS})")
    return struct.pack(_fmt(n), VERSION, n, *q, *dq, float(ts), int(seq) & 0xFFFFFFFF)


def decode(payload: bytes, expect_n: int | None = None) -> Frame:
    """解码一帧。

    `expect_n` 给定时，帧里的 `n` 必须与之相等 —— 这是「别照 n 去解一帧垃圾」的落点。
    本端关节数是**已知**的（`arm.n`），所以调用方**应该**总是传 `expect_n`。
    """
    if len(payload) < 2:
        raise WireError(f"帧太短: {len(payload)} B")
    version, n = struct.unpack_from("<BB", payload, 0)
    if version != VERSION:
        raise WireError(f"协议版本不符: 帧={version} 本端={VERSION}")
    if not 1 <= n <= MAX_JOINTS:
        raise WireError(f"关节数 {n} 越界 (1..{MAX_JOINTS})")
    if expect_n is not None and n != expect_n:
        raise WireError(f"关节数不符: 帧={n} 本端={expect_n}")
    want = frame_size(n)
    if len(payload) != want:
        raise WireError(f"帧长不符: 收到 {len(payload)} B, 按 n={n} 应为 {want} B")
    vals = struct.unpack(_fmt(n), payload)
    # vals = (version, n, *q, *dq, ts, seq)
    q = vals[2:2 + n]
    dq = vals[2 + n:2 + 2 * n]
    ts = vals[2 + 2 * n]
    seq = vals[3 + 2 * n]
    return Frame(q=q, dq=dq, ts=ts, seq=seq)
```

- [x] **Step 4: 跑测试确认通过**

```bash
python3 -m pytest tests/test_wire.py -q
```

Expected: 全绿（`passed`，无 `failed`/`error`）

- [x] **Step 5: 反向验证判据有判别力（临时把实现改坏，确认测试会红）**

```bash
# 把字节序从 '<' 改成 '>'，测试必须红 —— 这正是往返测试抓不到的那个错
sed -i 's/return f"<BB{n}f{n}fdI"/return f">BB{n}f{n}fdI"/' liteteleop/wire.py
find . -name __pycache__ -prune -exec rm -rf {} +      # ⚠ 见下方说明
python3 -m pytest tests/test_wire.py -q
sed -i 's/return f">BB{n}f{n}fdI"/return f"<BB{n}f{n}fdI"/' liteteleop/wire.py
find . -name __pycache__ -prune -exec rm -rf {} +
python3 -m pytest tests/test_wire.py -q
```

> ⚠ **改完必须清 `__pycache__`**：CPython 的 `.pyc` 头部存的是 mtime（**整秒**）+ 文件大小。
> 同尺寸的就地修改（如 `<` → `>`）若落在与上次编译**同一秒**内，变异**不会被发现**，
> 测试照样全绿（实测复现过）。不要用"看起来红了"当证据。

Expected: 第一次 **FAILED**（黄金字节与变长测试都红），第二次全绿。
**这一步不能省** —— 它证明这组判据真的在测字节序，而不是自我印证。

- [x] **Step 6: 提交**

```bash
git add liteteleop/wire.py tests/test_wire.py
git commit -m "feat: add wire protocol v1 codec with golden-byte tests"
```

---

## Task 4: Zenoh 点对点链路 `link.py`

**Files:**

- Create: `liteteleop/link.py`
- Test: `tests/test_link.py`

### 背景

spec §2.1 实测过的配置：关 `scouting/multicast/enabled` + `scouting/gossip/enabled`，两端 `mode="peer"`，主臂 `listen/endpoints`、从臂
`connect/endpoints` ⇒ 100 Hz 零丢包、回环延迟 p50 0.091 ms。
⚠ **不调 `close()` 就退出的进程永久挂死**（实测）。

- [x] **Step 1: 写失败测试**

`tests/test_link.py`：

```python
"""Zenoh 点对点链路 —— 起**真** session 走回环（非 mock），并钉住 close() 纪律。"""
import subprocess
import sys
import textwrap
import time

import pytest

from liteteleop import link

KEY = "litearm/teleop/test"


def _free_port() -> int:
    import socket
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def test_roundtrip_bytes():
    """端到端收发：100 帧逐字节全等。"""
    port = _free_port()
    m = link.Listener(port, KEY)
    got = []
    s = link.Connector("127.0.0.1", port, KEY, on_frame=got.append)
    try:
        time.sleep(0.5)
        frames = [bytes([i % 256]) * 70 for i in range(100)]
        for f in frames:
            m.put(f)
            time.sleep(0.002)
        deadline = time.monotonic() + 5.0
        while len(got) < len(frames) and time.monotonic() < deadline:
            time.sleep(0.01)
        assert got == frames
    finally:
        s.close()
        m.close()


def test_sustained_100hz_no_loss():
    """100 Hz 持续 2 s 零丢包 —— spec §2.1 实测过的能力，回归钉住。"""
    port = _free_port()
    m = link.Listener(port, KEY)
    got = []
    s = link.Connector("127.0.0.1", port, KEY, on_frame=got.append)
    try:
        time.sleep(0.5)
        got.clear()
        n, t0 = 0, time.monotonic()
        dt, nxt = 0.01, t0 + 0.01
        while time.monotonic() - t0 < 2.0:
            m.put(b"x" * 70)
            n += 1
            r = nxt - time.monotonic()
            if r > 0:
                time.sleep(r)
            nxt += dt
            if nxt < time.monotonic():
                nxt = time.monotonic() + dt
        deadline = time.monotonic() + 3.0        # 给在途帧留到达窗口
        while len(got) < n and time.monotonic() < deadline:
            time.sleep(0.01)
        assert n >= 190, f"只发出 {n} 帧"
        assert len(got) == n, f"丢包: 发 {n} 收 {len(got)}"
    finally:
        s.close()
        m.close()


def test_matching_flag():
    """主臂端能拿到 matching 布尔（⚠ 是布尔不是计数，spec §8）。"""
    port = _free_port()
    m = link.Listener(port, KEY)
    try:
        assert m.matching is False
        s = link.Connector("127.0.0.1", port, KEY, on_frame=lambda b: None)
        try:
            deadline = time.monotonic() + 5.0
            while not m.matching and time.monotonic() < deadline:
                time.sleep(0.05)
            assert m.matching is True
        finally:
            s.close()
    finally:
        m.close()

def test_latest_slot_keeps_only_newest():
    """latest-wins：迟到帧覆盖、不排队（spec §3.2）。被覆盖的帧要计数，不静默。"""
    slot = link.LatestSlot()
    slot.put(b"a", now=1.0)
    slot.put(b"b", now=2.0)
    assert slot.take() == (b"b", 2.0)
    assert slot.dropped == 1, "被覆盖的那帧要计入 dropped（不静默）"


def test_latest_slot_take_clears():
    slot = link.LatestSlot()
    assert slot.take() == (None, 0.0), "空槽取回 (None, 0.0)"
    slot.put(b"x", now=5.0)
    assert slot.take() == (b"x", 5.0)
    assert slot.take() == (None, 0.0), "取走后槽必须清空"


def test_latest_slot_age_is_local_and_zero_when_never_received():
    """⚠ `peek_age` 是**本机**时间差，与帧里的 `ts`（主臂时钟）无关（spec §4.2）。

    从未收到过时返回 0.0 —— **不是** `now`（那会让 watchdog 以为"刚收到"）。
    """
    slot = link.LatestSlot()
    assert slot.peek_age(now=100.0) == 0.0, "从未收到 ⇒ 0.0"
    slot.put(b"x", now=10.0)
    assert slot.peek_age(now=10.25) == pytest.approx(0.25)


@pytest.mark.slow
def test_close_is_mandatory_for_exit():
    """⚠ 不 close ⇒ 进程永久挂死（spec §2.1 `[实测]`）。

    用**子进程**验证，因为症状是"解释器退不出去"，同进程测不到。
    """
    prog = textwrap.dedent("""
        import sys
        sys.path.insert(0, ".")
        from liteteleop import link
        m = link.Listener(int(sys.argv[1]), "x/y")
        m.put(b"hi")
        if sys.argv[2] == "close":
            m.close()
        print("reached-end")
    """)
    port = _free_port()
    ok = subprocess.run([sys.executable, "-c", prog, str(port), "close"],
                        capture_output=True, text=True, timeout=30)
    assert ok.returncode == 0, f"显式 close 后应正常退出: {ok.stderr}"
    assert "reached-end" in ok.stdout

    # ⚠ 症状是"解释器退不出去" ⇒ 表现为 subprocess.run **超时抛 TimeoutExpired**，
    # 不是返回非零码。**超时就是通过**（下面这行是唯一正确的判据写法）。
    with pytest.raises(subprocess.TimeoutExpired):
        subprocess.run([sys.executable, "-c", prog, str(port + 1), "noclose"],
                       capture_output=True, text=True, timeout=20)
```

- [x] **Step 2: 跑测试确认失败**

```bash
cd /home/llx/litearm-teleop-isomorphic
python3 -m pytest tests/test_link.py -q
```

Expected: **collection error** —— 模块尚不存在。实测文案随 pytest 版本而异
（`ModuleNotFoundError: No module named ...` 或 `ImportError: cannot import name 'link' from 'liteteleop'`）
⇒ **两者都算通过**；判据是「收集期就失败」，不是那条具体文案。

- [x] **Step 3: 实现**

`liteteleop/link.py`：

```python
"""Zenoh **纯点对点**链路（见 spec §2.1 / §4.1）。

两端都 `mode="peer"`，关掉 multicast/gossip 发现 —— **不用广播**，只走显式
`listen`/`connect` 端点。主臂监听端口，从臂连过去。

⚠ **进程退出前必须 `close()`**，否则**永久挂死**（spec §2.1 `[实测]`：不 close 时
解释器退不出去，`timeout` 才能杀掉）。
⚠ key 与 litearm-server 的 `litearm/v4/{arm_id}/teleop` **不可混用**：那边是 `>15d` 120 B，
本文是 70 B，共用 key 会让两端把对方的帧静默解错。
"""
from __future__ import annotations

import threading
from typing import Callable, Optional

import zenoh

__all__ = ["DEFAULT_KEY", "Listener", "Connector", "LatestSlot"]

#: 默认 key —— 主臂发布、从臂订阅，两端必须一致（界面可改）。
DEFAULT_KEY = "litearm/teleop/isomorphic"


def _base_config() -> zenoh.Config:
    """纯点对点的公共配置：**关掉全部广播发现**。"""
    c = zenoh.Config()
    c.insert_json5("scouting/multicast/enabled", "false")
    c.insert_json5("scouting/gossip/enabled", "false")
    c.insert_json5("mode", '"peer"')
    return c


class _Endpoint:
    def __init__(self) -> None:
        self._session: Optional[zenoh.Session] = None
        self._closed = False

    def close(self) -> None:
        """⚠ 幂等，但**必须被调用**（否则进程退不出去）。"""
        if self._closed:
            return
        self._closed = True
        s, self._session = self._session, None
        if s is not None:
            try:
                s.close()
            except Exception:                       # noqa: BLE001 - 退出路径不抛
                pass

    def __enter__(self):
        return self

    def __exit__(self, *exc) -> bool:
        self.close()
        return False


class Listener(_Endpoint):
    """主臂端：监听端口，等从臂连进来。"""

    def __init__(self, port: int, key: str = DEFAULT_KEY):
        super().__init__()
        cfg = _base_config()
        cfg.insert_json5("listen/endpoints", f'["tcp/0.0.0.0:{int(port)}"]')
        cfg.insert_json5("connect/endpoints", "[]")
        self._session = zenoh.open(cfg)
        self._pub = self._session.declare_publisher(key)
        self._matching = False

        def _on_match(status) -> None:
            self._matching = bool(status.matching)

        self._pub.declare_matching_listener(_on_match)

    def put(self, payload: bytes) -> None:
        """发布一帧。非阻塞。"""
        self._pub.put(payload)

    @property
    def matching(self) -> bool:
        """是否有订阅者匹配。

        ⚠ **是布尔不是计数** —— `zenoh.MatchingStatus` 只有 `.matching`
        （在**跑测试的那个解释器**上核过：`dir(zenoh.MatchingStatus) == ['matching']`；
        ⚠ 本机 `pip` 与 `python3 -m pip` 指向**不同解释器**，版本也不同 —— 别用裸 `pip` 量版本）。
        界面显示「已匹配/未匹配」。
        """
        return self._matching


class Connector(_Endpoint):
    """从臂端：连到主臂的 IP:端口，订阅其流。

    `on_frame` 在 **zenoh 自己的线程**上被调用 ⇒ **只许写一个 latest 槽**，
    不许在里面做阻塞操作（`move_js` 每帧等 ACK，放进去会把 zenoh 线程拖死）。见 spec §3.2。
    """

    def __init__(self, host: str, port: int, key: str = DEFAULT_KEY,
                 on_frame: Optional[Callable[[bytes], None]] = None):
        super().__init__()
        cfg = _base_config()
        cfg.insert_json5("listen/endpoints", "[]")
        cfg.insert_json5("connect/endpoints", f'["tcp/{host}:{int(port)}"]')
        self._session = zenoh.open(cfg)
        cb = on_frame or (lambda _b: None)
        #: 计数（只读，供界面显示）。
        #: ⚠ 安全性来自**同一个 subscriber 的回调在同一条 zenoh 线程上串行**，
        #:   不是「GIL 下 `+=` 原子」—— 属性 `+=` 本身不是原子字节码，别照这句去推广。
        self.received = 0

        def _handler(sample) -> None:
            self.received += 1
            cb(bytes(sample.payload))

        self._sub = self._session.declare_subscriber(key, _handler)

    def latest(self) -> int:
        """已收帧数（供界面显示频率）。"""
        return self.received


class LatestSlot:
    """latest-wins 槽 —— zenoh 回调与伺服环之间的唯一交接面（spec §3.2）。

    只保留**最新**一帧：迟到帧直接覆盖，不排队。遥操只要最新姿态，
    积压帧会让从臂去追一条过期的轨迹。
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._payload: Optional[bytes] = None
        self._recv_ts = 0.0
        self.dropped = 0

    def put(self, payload: bytes, now: float) -> None:
        with self._lock:
            if self._payload is not None:
                self.dropped += 1
            self._payload = payload
            self._recv_ts = now

    def take(self):
        """取走最新一帧（`(payload_or_None, recv_ts)`）。取走后槽清空。"""
        with self._lock:
            p, t = self._payload, self._recv_ts
            self._payload, self._recv_ts = None, 0.0
        return p, t

    def peek_age(self, now: float) -> float:
        """距最近一帧的**本地**时间（秒）。0.0 = 从未收到过。

        ⚠ 这是**本机**时间差，与帧里的 `ts`（主臂时钟）无关 —— 跨机两端不同源。
        """
        with self._lock:
            return 0.0 if self._recv_ts == 0.0 else now - self._recv_ts
```

- [x] **Step 4: 跑测试确认通过**

`slow` marker 已在 Task 1 的 `pyproject.toml` 里注册好（本步只确认，别再改那个文件）：

```bash
grep -q "slow:" pyproject.toml && echo "marker 已注册" || echo "⚠ marker 缺失，回 Task 1 补"
```

```bash
python3 -m pytest tests/test_link.py -q -m "not slow"
python3 -m pytest tests/test_link.py -q -m slow
```

Expected: 第一条全绿；第二条 `1 passed`（子进程测试，约 20~40 s）。

> 若第二条红：说明「不 close 会挂死」这条已不再成立（zenoh 改了行为）。**不要删测试** ——
> 它是那条纪律的唯一守卫；改成断言新行为并在此处留下说明。

- [x] **Step 5: 提交**

```bash
git add liteteleop/link.py tests/test_link.py
git commit -m "feat: add zenoh point-to-point link layer with close discipline"
```

---

## Task 5: `slew_target` 移植与 `safety.py` 的限位/闸门

**Files:**

- Create: `liteteleop/safety.py`
- Test: `tests/test_safety.py`

> **这是本阶段最要紧的一个 Task**：`slew_target` 是 spec §5.1 的**唯一**跟随机制，
> 且它是**从既有验证实现逐字移植**的（`pylitearm/src/pylitearm/control/joint_follow.py:45-88`）。
> ⛔ **不要"改进"它** —— 任何偏离都必须先报用户裁决。移植已知边界见 Step 6。

- [x] **Step 1: 写全部纯逻辑测试（含**与 pylitearm 原版逐拍全等**的对拍）**

`tests/test_safety.py`：

```python
"""safety.py —— 纯逻辑层（不 import litearm，不碰硬件）。

⭐ 核心判据：移植的 `slew_target` 与 `pylitearm` 原版**同一组输入逐拍全等**。
这是"照抄而非重写"的唯一硬证据；没有它，"我照抄了"只是自述。
"""
import math

import pytest

from liteteleop import safety

# ── 参照实现：从 pylitearm/src/pylitearm/control/joint_follow.py:45-88 抄成独立副本 ──────────
# ⚠ 这份副本是**测试的对照组**，刻意与实现分开写。两边都错才可能同时通过。


def _ref_slew(raw_target, q_cmd, dq_cmd, speed_limit, accel_limit, dt):
    dt = max(dt, 1e-4)
    for i in range(len(raw_target)):
        v_limit = max(1e-4, speed_limit[i])
        a_limit = max(1e-4, accel_limit[i])
        dv_max = a_limit * dt
        diff = raw_target[i] - q_cmd[i]
        v = dq_cmd[i]
        if abs(diff) < 1e-5 and abs(v) < dv_max:
            q_cmd[i] = raw_target[i]
            dq_cmd[i] = 0.0
            continue
        stopping_dist = (v * v) / (2.0 * a_limit) if a_limit > 0.0 else 0.0
        moving_toward = diff * v > 0.0
        if moving_toward and abs(diff) <= stopping_dist:
            desired_v = 0.0
        else:
            desired_v = math.copysign(v_limit, diff)
        v += max(-dv_max, min(dv_max, desired_v - v))
        v = max(-v_limit, min(v_limit, v))
        step = v * dt
        if diff * step > 0.0 and abs(step) >= abs(diff):
            q_cmd[i] = raw_target[i]
            dq_cmd[i] = 0.0
        else:
            q_cmd[i] += step
            dq_cmd[i] = v
    return q_cmd, dq_cmd


def _traj(t, q0):
    """确定的合成轨迹 —— 固定输入，对拍才可复现。"""
    tgt = list(q0)
    tgt[1] += 0.30 * math.sin(2 * math.pi * 0.5 * t)
    tgt[2] += 0.20 * math.sin(2 * math.pi * 0.3 * t + 0.7)
    return tgt


def test_slew_matches_pylitearm_reference():
    """⭐ 移植版与原版**逐拍全等**（60 拍 × 3 组参数 = 180 次比对）。"""
    n, dt = 7, 0.01
    for sp, ac in [(0.3, 14.0), (2.0, 24.0), (1.2, 45.0)]:
        q0 = [0.1 * i for i in range(n)]
        a_cmd, a_dq = list(q0), [0.0] * n
        b_cmd, b_dq = list(q0), [0.0] * n
        for k in range(60):
            tgt = _traj(k * dt, q0)
            a_cmd, a_dq = safety.slew_target(tgt, a_cmd, a_dq, [sp] * n, [ac] * n, dt)
            b_cmd, b_dq = _ref_slew(tgt, b_cmd, b_dq, [sp] * n, [ac] * n, dt)
            assert a_cmd == pytest.approx(b_cmd, abs=1e-15), f"sp={sp} ac={ac} 第 {k} 拍 q_cmd 分叉"
            assert a_dq == pytest.approx(b_dq, abs=1e-15), f"sp={sp} ac={ac} 第 {k} 拍 dq_cmd 分叉"


def test_slew_speed_limited():
    """|dq_cmd| 恒 ≤ speed_limit —— spec §5.1 那条「accel_limit 不抬高天花板」的依据。"""
    n, dt, sp, ac = 7, 0.01, 0.5, 20.0
    q_cmd, dq_cmd = [0.0] * n, [0.0] * n
    for k in range(200):
        q_cmd, dq_cmd = safety.slew_target([3.0] * n, q_cmd, dq_cmd, [sp] * n, [ac] * n, dt)
        assert all(abs(v) <= sp + 1e-12 for v in dq_cmd), f"第 {k} 拍越过 speed_limit"


def test_slew_converges_and_stops():
    """收敛且到目标即停（不超冲、不振荡）。"""
    n, dt = 7, 0.01
    q_cmd, dq_cmd = [0.0] * n, [0.0] * n
    for _ in range(1000):
        q_cmd, dq_cmd = safety.slew_target([1.0] * n, q_cmd, dq_cmd, [2.0] * n, [14.0] * n, dt)
    assert q_cmd == pytest.approx([1.0] * n, abs=1e-6)
    assert dq_cmd == pytest.approx([0.0] * n, abs=1e-9)


def test_slew_brakes_before_target():
    """⭐ 制动距离减速**真的生效** —— 判据是「最长减速连跑」拍数。

    为什么不用"有没有过冲"当判据：**过冲为 0 是被别的东西挡住的**（见下方模块注释与
    Task 5 Step 5 的实测），去掉制动距离后过冲**依然是 0** ⇒ 用过冲当判据是**没有判别力**的。

    实测（本计划作者量化过）：`sp=2.0, ac=14.0` 下 ——
    原版最长减速连跑 **14 拍**；去掉制动距离 **0 拍**；去掉吸附 **14 拍**。
    """
    n, dt, tgt = 7, 0.01, 0.5
    q_cmd, dq_cmd = [0.0] * n, [0.0] * n
    seq, worst = [], 0.0
    for _ in range(400):
        q_cmd, dq_cmd = safety.slew_target([tgt] * n, q_cmd, dq_cmd, [2.0] * n, [14.0] * n, dt)
        seq.append(abs(dq_cmd[0]))
        worst = max(worst, max(q_cmd) - tgt)
    run = best = 0
    prev = None
    for v in seq:
        run = run + 1 if (prev is not None and 0.0 < v < prev) else 0
        best = max(best, run)
        prev = v
    assert best >= 5, f"没有减速段（最长连跑 {best} 拍）—— 制动距离减速没生效"
    # 顺带：不过冲。⚠ 这条**单独没有判别力**（制动与吸附都去掉才为假），
    # 只作为上面那条强判据的附带检查，别当成独立覆盖。
    assert worst <= 1e-9, f"过冲 {worst}"


# ── 软限位闸门 ──────────────────────────────────────────────────────────────


def test_limits_ok():
    lim = safety.read_limits_ok([-1.0] * 7, [1.0] * 7, 7)
    assert lim.n == 7

@pytest.mark.parametrize("lo,hi,n,match", [
    ([-1.0] * 6, [1.0] * 7, 7, "长度不符"),
    ([-1.0] * 7, [1.0] * 7, 8, "长度不符"),
    ([float("nan")] * 7, [1.0] * 7, 7, "非有限值"),
    ([1.0] * 7, [-1.0] * 7, 7, "上下界反了"),
    ([0.0] * 7, [0.0] * 7, 7, "上下界反了"),
])


def test_limits_reject(lo, hi, n, match):
    """限位任何一处不合法都**拒启动** —— 不静默退化（spec §7.1）。"""
    with pytest.raises(safety.LimitsError, match=match):
        safety.read_limits_ok(lo, hi, n)


def test_clamp_flags_saturated_axes():
    """被钳住的轴要被标出来（抗饱和的输入）。"""
    lim = safety.read_limits_ok([-1.0] * 7, [1.0] * 7, 7)
    q, sat = safety.clamp_to_limits([0.5, 2.0, -0.3, -5.0, 0.0, 1.0, 0.9], lim)
    assert q == pytest.approx([0.5, 1.0, -0.3, -1.0, 0.0, 1.0, 0.9])
    assert sat == [False, True, False, True, False, False, False]


def test_saturate_dq_zeroes_clamped_axes():
    """spec §5.1：被钳位的轴 `dq` 必为 0（这条不变量的具名载体）。"""
    dq = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
    sat = [False, True, False, True, False, False, False]
    assert safety.saturate_dq(dq, sat) == [1.0, 0.0, 1.0, 0.0, 1.0, 1.0, 1.0]
    assert safety.saturate_dq([2.0] * 7, [False] * 7) == [2.0] * 7, "没被钳的不许动"


def test_saturate_dq_in_j4_upper_limit_case():
    """⚠ J4 上端只有 +0.0175 rad —— 主臂一拖过零就吃到，不是边角情况。"""
    lim = safety.read_limits_ok([-3.0] * 7, [3.0] * 7, 7)
    lim = safety.Limits(lo=lim.lo[:3] + (0.0175,) + lim.lo[4:],
                        hi=lim.hi[:3] + (0.0175,) + lim.hi[4:])
    q, sat = safety.clamp_to_limits([0.0, 0.0, 0.0, 0.20, 0.0, 0.0, 0.0], lim)
    assert sat[3] is True, "J4 超出 0.0175 应被判被钳"
    dq = safety.saturate_dq([1.0] * 7, sat)
    assert dq[3] == 0.0, "被钳的 J4 必须把 dq 置 0（否则 kd·dq 持续顶限位）"


def test_clamp_uses_range_not_float_equality():
    """⚠ 判据用区间比较，不用浮点 `!=`（spec §5.1 的 `saturated` 定义）。"""
    lim = safety.read_limits_ok([0.0] * 7, [1.0] * 7, 7)
    _, sat = safety.clamp_to_limits([0.0, 1.0, 0.5, 0.5, 0.5, 0.5, 0.5], lim)
    assert sat == [False, False, False, False, False, False, False], "边界值不应算被钳"

# ── kd 预算闸门 ─────────────────────────────────────────────────────────────


def test_speed_limit_from_kd_matches_spec_table():
    """spec §5.1 那张表：本机 7 轴 @30% 预算。"""
    kd = [11.0, 11.0, 10.0, 11.0, 2.5, 2.5, 2.5]          # mit_kd + kd_extra
    tau = [78.0, 78.0, 21.0, 21.0, 10.0, 10.0, 10.0]
    got = safety.speed_limit_from_kd(kd, tau, kd_budget=0.30)
    assert got == pytest.approx([2.127, 2.127, 0.63, 0.573, 1.2, 1.2, 1.2], abs=0.01)


def test_speed_limit_j4_is_the_binding_case():
    """⛔ J4 是全局最紧的轴 —— 它决定了「高速跟随」到底多快。"""
    kd = [11.0, 11.0, 10.0, 11.0, 2.5, 2.5, 2.5]
    tau = [78.0, 78.0, 21.0, 21.0, 10.0, 10.0, 10.0]
    got = safety.speed_limit_from_kd(kd, tau)
    assert min(got) == pytest.approx(got[3]), "最紧的应是 J4"
    assert min(got) < 0.7, "J4 的上限必须明显低于它的 speed_limit=1.75"

@pytest.mark.parametrize("budget", [0.0, -0.1, 1.5])
def test_speed_limit_rejects_bad_budget(budget):
    with pytest.raises(safety.LimitsError, match="kd_budget"):
        safety.speed_limit_from_kd([1.0] * 7, [1.0] * 7, kd_budget=budget)


def test_speed_limit_rejects_bad_kd():
    with pytest.raises(safety.LimitsError, match="非法"):
        safety.speed_limit_from_kd([0.0] * 7, [1.0] * 7)
```

- [x] **Step 2: 跑测试确认失败**

```bash
cd /home/llx/litearm-teleop-isomorphic
python3 -m pytest tests/test_safety.py -q
```

Expected: **collection error** —— 模块尚不存在（文案随 pytest 版本而异，两种都算通过）。

- [x] **Step 3: 实现**

`liteteleop/safety.py`：

```python
"""安全层（一/二）—— 纯逻辑：跟随平滑 + 软限位闸门 + kd 预算闸门。

设计依据 spec §5.1（跟随律与限幅）与 §7.1（限位不合法就**拒启动**，不静默退化）。

本模块**不 import litearm / pylitearm，不碰硬件**：它是可以在无臂机器上单测的纯函数层。
状态机与 watchdog 是第二部分（见 spec §5.2），本文件只放第一步的纯逻辑。

⚠ `slew_target` 是**从既有验证实现逐字移植**的（`pylitearm/src/pylitearm/control/joint_follow.py:45-88`），
⛔ **不是**重新设计。理由：跟随手感与制动距离是**整条从臂跟随行为**的唯一来源，
"照抄"才有既有真机行为的可预期性；任何"改进"都必须先报用户裁决（spec §11 S3）。
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

__all__ = [
    "DEFAULT_KD_BUDGET",
    "LimitsError",
    "Limits",
    "read_limits_ok",
    "clamp_to_limits",
    "saturate_dq",
    "speed_limit_from_kd",
    "slew_target",
]

#: 速度上限的默认 kd 预算：`speed_limit_i = kd_budget * tau_max_i / kd_i`（spec §5.1）。
#: 含义是「按 kd 折算出来的指令速度所对应的力矩不超过 tau_max 的 30%」。
DEFAULT_KD_BUDGET = 0.30


class LimitsError(ValueError):
    """限位/预算配置不合法 —— **一律拒启动**，绝不静默退化（spec §7.1）。"""


@dataclass(frozen=True)
class Limits:
    """一组关节软限位（rad）。`lo`/`hi` 逐轴成对，长度 = 轴数。

    `n` 可以省略（默认 = `len(lo)`）；显式给出时必须与 `lo`/`hi` 自洽。
    """

    lo: Tuple[float, ...]
    hi: Tuple[float, ...]
    n: Optional[int] = None

    def __post_init__(self) -> None:
        object.__setattr__(self, "lo", tuple(float(v) for v in self.lo))
        object.__setattr__(self, "hi", tuple(float(v) for v in self.hi))
        if self.n is None:
            object.__setattr__(self, "n", len(self.lo))
        if len(self.lo) != len(self.hi) or len(self.lo) != self.n:
            raise LimitsError(
                f"长度不符: lo={len(self.lo)} hi={len(self.hi)} n={self.n}"
            )


def _check_finite(vals: Sequence[float], what: str) -> None:
    for i, v in enumerate(vals):
        if not math.isfinite(v):
            raise LimitsError(f"非有限值: {what}[{i}] = {v}")


def read_limits_ok(lo: Sequence[float], hi: Sequence[float], n: int) -> Limits:
    """校验并固化一对软限位；任何一处不合法都**拒启动**（spec §7.1）。

    依次判：①长度（两数组各自 = `n`）②有限 ③`lo[i] < hi[i]`（**相等也算反了**：
    零宽度的轴意味着该关节被锁死，是配置错误而不是"恰好不动"）。
    """
    if n <= 0:
        raise LimitsError(f"轴数非法: n={n}")
    if len(lo) != n or len(hi) != n:
        raise LimitsError(f"长度不符: lo={len(lo)} hi={len(hi)} n={n}")
    _check_finite(lo, "lo")
    _check_finite(hi, "hi")
    for i, (a, b) in enumerate(zip(lo, hi)):
        if not a < b:
            raise LimitsError(f"第 {i} 轴上下界反了: lo={a} hi={b}")
    return Limits(lo=tuple(lo), hi=tuple(hi), n=n)


def clamp_to_limits(
    q: Sequence[float], limits: Limits
) -> Tuple[List[float], List[bool]]:
    """把 `q` 逐轴钳进软限位，返回 `(q_clamped, saturated)`。

    ⚠ 是否"被钳"的判据是**区间比较**（`q < lo` 或 `q > hi`），**不是**浮点 `!=`
    （spec §5.1）：恰在边界上的值**不算**被钳 —— 它数值上并未越界，
    把它判成饱和会平白把那个轴的 `dq` 打成 0（见 `saturate_dq`），
    即"边界姿态被误判为顶住限位"。

    ⚠ 非有限值**拒算**（`LimitsError`）而不是放行：NaN 与任何边界比较都是 False，
    照原样传下去会**悄悄**把一个 NaN 目标送到从臂 —— 这是静默失败，不是限位。
    """
    if len(q) != limits.n:
        raise LimitsError(f"长度不符: q={len(q)} 限位轴数={limits.n}")
    _check_finite(q, "q")
    out: List[float] = []
    sat: List[bool] = []
    for v, a, b in zip(q, limits.lo, limits.hi):
        if v < a:
            out.append(a)
            sat.append(True)
        elif v > b:
            out.append(b)
            sat.append(True)
        else:
            out.append(float(v))
            sat.append(False)
    return out, sat


def saturate_dq(dq: Sequence[float], saturated: Sequence[bool]) -> List[float]:
    """把被钳轴的 `dq` 置 0 —— spec §5.1 那条不变量的**具名载体**。

    被钳的轴必须 `dq = 0`（否则 `kd·dq` 会持续把限位顶住：位置钳住了，
    速度前馈还在推，力矩一直压在限位上）。没被钳的轴**一个数都不许动**。
    """
    if len(dq) != len(saturated):
        raise LimitsError(f"长度不符: dq={len(dq)} saturated={len(saturated)}")
    return [0.0 if s else float(v) for v, s in zip(dq, saturated)]


def speed_limit_from_kd(
    kd: Sequence[float],
    tau_max: Sequence[float],
    kd_budget: float = DEFAULT_KD_BUDGET,
) -> List[float]:
    """按 kd 预算把力矩上限折算成速度上限（spec §5.1）::

        speed_limit_i = kd_budget * tau_max_i / kd_i     (rad/s)

    含义：速度跟随里 `tau ≈ kd·dq`，让 `kd·speed_limit ≤ kd_budget·tau_max`
    ⇒ 纯阻尼项就把力矩预算吃掉 `kd_budget` 那份，剩下的留给刚度与重力前馈。

    ⛔ J4 通常是**全局最紧的轴**（`tau_max/kd` 比最小）⇒ 它决定了"高速跟随"
    到底能有多快；这条闸门就是 spec 里"高速跟随时 J4 会不会顶住"的落点。

    任何不合法（长度/有限/kd≤0/tau_max≤0/预算越界）都 `LimitsError` 拒启动。
    """
    n = len(kd)
    if len(tau_max) != n:
        raise LimitsError(f"长度不符: kd={n} tau_max={len(tau_max)}")
    if n == 0:
        raise LimitsError("长度不符: 轴数 0")
    if not 0.0 < kd_budget <= 1.0:
        raise LimitsError(f"kd_budget 越界: {kd_budget} 不在 (0, 1] 内")
    _check_finite(kd, "kd")
    _check_finite(tau_max, "tau_max")
    out: List[float] = []
    for i in range(n):
        if kd[i] <= 0.0 or tau_max[i] <= 0.0:
            raise LimitsError(
                f"第 {i} 轴 kd/tau_max 非法: kd={kd[i]} tau_max={tau_max[i]}（须 > 0）"
            )
        out.append(kd_budget * tau_max[i] / kd[i])
    return out


def slew_target(raw_target, q_cmd, dq_cmd, speed_limit, accel_limit, dt):
    """速度/加速度限制的目标位置平滑（梯形速度曲线）。

    ⚠ **逐字移植自 `pylitearm/src/pylitearm/control/joint_follow.py:45-88`**（唯一偏离：原版循环
    上界是模块常量 `N = 7`，这里取 `len(raw_target)`；7 轴输入下两者**完全等价**）。
    对每个关节：

    - 限制最大速度为 `speed_limit[i]`
    - 限制最大加速度为 `accel_limit[i]`
    - 当接近目标时自动减速（基于制动距离 `v²/(2a)`）
    - 落在死区（`|diff| < 1e-5` 且 `|v| < dv_max`）时吸附到目标并停住

    Args:
        raw_target: 原始目标位置 [N] (rad)
        q_cmd: 当前指令位置 [N] (rad)，**会被就地修改**
        dq_cmd: 当前指令速度 [N] (rad/s)，**会被就地修改**
        speed_limit: 每关节最大速度 [N] (rad/s)
        accel_limit: 每关节最大加速度 [N] (rad/s²)
        dt: 控制周期 (s)

    Returns:
        (q_cmd, dq_cmd): 平滑后的位置和速度

    ⚠ **移植已知边界（照抄，不在本计划修）**：`diff` 恰为 0 或落在 `±1e-5` 死区内、
    而 `|v| ≥ dv_max` 时，`math.copysign(v_limit, diff)` 在 `diff == 0` 会返回 `+v_limit`
    ⇒ 该轴可能继续正向加速而非停住（原版同样如此）。实践中 `diff` 极少恰为 0，
    且 `abs(diff) < 1e-5 and abs(v) < dv_max` 已挡掉绝大多数情形。
    **忠实移植优先**；若真机 S3 观察到自激，这是嫌疑点之一（见 spec §11 S3）。
    """
    dt = max(dt, 1e-4)
    for i in range(len(raw_target)):
        v_limit = max(1e-4, speed_limit[i])
        a_limit = max(1e-4, accel_limit[i])
        dv_max = a_limit * dt

        diff = raw_target[i] - q_cmd[i]
        v = dq_cmd[i]

        # 已到达目标
        if abs(diff) < 1e-5 and abs(v) < dv_max:
            q_cmd[i] = raw_target[i]
            dq_cmd[i] = 0.0
            continue

        # 期望速度（考虑制动距离）
        stopping_dist = (v * v) / (2.0 * a_limit) if a_limit > 0.0 else 0.0
        moving_toward = diff * v > 0.0
        if moving_toward and abs(diff) <= stopping_dist:
            desired_v = 0.0  # 开始减速
        else:
            desired_v = math.copysign(v_limit, diff)

        # 限加速度
        v += max(-dv_max, min(dv_max, desired_v - v))
        v = max(-v_limit, min(v_limit, v))

        # 更新位置
        step = v * dt
        if diff * step > 0.0 and abs(step) >= abs(diff):
            q_cmd[i] = raw_target[i]
            dq_cmd[i] = 0.0
        else:
            q_cmd[i] += step
            dq_cmd[i] = v

    return q_cmd, dq_cmd
```

- [x] **Step 4: 跑测试确认通过**

```bash
python3 -m pytest tests/test_safety.py -q
```

Expected: 全绿（`passed`，无 `failed`/`error`）

- [x] **Step 4.5: 记下 phase 2 的接法（不在本任务实现，但必须留痕）**

`clamp_to_limits` 对**非有限的目标值**（NaN/Inf）**拒算**并抛 `LimitsError` —— 这是刻意的：
NaN 与任何边界比较都是 `False`，放行就等于**悄悄**把一个 NaN 目标发给从臂（静默失败）。
⇒ phase 2 的伺服环必须**逐帧捕获 `LimitsError`**，按 spec §7.1 处理：**本拍不下发 + 计数 +
按"状态缺失"报警**，绝不让它冒泡把伺服循环打断。

- [x] **Step 5: 确认判据的判别力 —— 并**如实记下哪条其实没有****

> ⚠ **本节第一版把判别力说错了两次**（一次靠推断没实测、一次张冠李戴）。
> 下面每个数字都是**量化过的**，不是推断。**别凭感觉写"这条会红"。**

**变异 A：去掉制动距离减速** ⇒ 该红的是 **parity** 与 **brakes_before_target** 两条：

```bash
cp liteteleop/safety.py /tmp/safety.bak
python3 - <<'PY'
p="liteteleop/safety.py"; s=open(p,encoding="utf-8").read()
s=s.replace("if moving_toward and abs(diff) <= stopping_dist:\n            desired_v = 0.0  # 开始减速\n        else:",
            "if False:\n            desired_v = 0.0\n        else:")
open(p,"w",encoding="utf-8").write(s)
PY
find . -name __pycache__ -prune -exec rm -rf {} +
python3 -m pytest tests/test_safety.py -q -k "matches_pylitearm or brakes_before_target"
cp /tmp/safety.bak liteteleop/safety.py
find . -name __pycache__ -prune -exec rm -rf {} +
```

Expected: **两条都红**（`2 failed`）。

**变异 B：去掉"吸附"** ⇒ ⛔ **本套件里没有任何一条会红**（实测：parity 红、其余全绿；
`converges_and_stops` 仍绿，因为最终吸附是**死区分支**做的，不是这句吸附做的）。
**这不是缺陷，是覆盖边界** —— 记下来，别以为它被覆盖了：

```bash
cp liteteleop/safety.py /tmp/safety.bak
python3 - <<'PY'
p="liteteleop/safety.py"; s=open(p,encoding="utf-8").read()
s=s.replace("if diff * step > 0.0 and abs(step) >= abs(diff):", "if False:")
open(p,"w",encoding="utf-8").write(s)
PY
find . -name __pycache__ -prune -exec rm -rf {} +
python3 -m pytest tests/test_safety.py -q -k "matches_pylitearm"       # 应当只有这条红
cp /tmp/safety.bak liteteleop/safety.py
find . -name __pycache__ -prune -exec rm -rf {} +
python3 -m pytest tests/test_safety.py -q
```

**⇒ 结论（写进这个 Task 的提交信息或注释里）**：

| 判据 | 对变异 A | 对变异 B | 真实判别力 |
| --- | --- | --- | --- |
| `test_slew_matches_pylitearm_reference`（parity） | 红 | 红 | ⭐ **唯一**能守住"逐字移植"的那条 |
| `test_slew_brakes_before_target` | 红 | 绿 | ⭐ 独立于对照实现，守住制动段 |
| `test_slew_speed_limited` | 绿 | 绿 | 弱（不变量真实，但这两个变异都碰不到它） |
| `test_slew_converges_and_stops` | 绿 | 绿 | 弱（最终吸附由死区分支做） |

**⇒ "逐字移植"的守卫是 parity 那一条，不是这一堆。** 别用后三条的绿去论证移植正确；
它们绿是因为它们测的是别的东西。

- [x] **Step 6: 记录移植已知边界**

在 `safety.py` 的 `slew_target` docstring 末尾追加（**不改逻辑**）：

```text
    ⚠ **移植已知边界（照抄，不在本计划修）**：`diff` 恰为 0 或落在 `±1e-5` 死区内、
    而 `|v| ≥ dv_max` 时，`math.copysign(v_limit, diff)` 在 `diff == 0` 会返回 `+v_limit`
    ⇒ 该轴可能继续正向加速而非停住（原版同样如此）。实践中 `diff` 极少恰为 0，
    且 `abs(diff) < 1e-5 and abs(v) < dv_max` 已挡掉绝大多数情形。
    **忠实移植优先**；若真机 S3 观察到自激，这是嫌疑点之一（见 spec §11 S3）。
```

- [x] **Step 7: 提交**

```bash
git add liteteleop/safety.py tests/test_safety.py
git commit -m "feat: port slew_target from pylitearm with parity tests"
```

---

## Task 6: `safety.py` 状态机与 watchdog

**Files:**

- Modify: `liteteleop/safety.py`（追加第 4 块）
- Modify: `tests/test_safety.py`（追加）

### 状态机（spec §5）

```text
                启动遥操
   IDLE ─────────────────────▶ ALIGN_FAST ──到位──▶ FOLLOWING
     ▲                            ▲                    │
     │                     连续收帧 5 拍          watchdog 超时
     │                            │                    ▼
     │                            └──────────────  HOLDING
     │                                          │ 持续 ≥ 2 s ⇒ 升级 movej(q_now) 一次
     └────────── 用户点「停止」 ──── movej(q_now) ◀────┘
```

- [x] **Step 1: 写失败测试**

追加到 `tests/test_safety.py`：

```python
# ── 状态机与 watchdog ───────────────────────────────────────────────────────


def _sm(**kw):
    defaults = dict(n=7, watchdog_ms=200.0, recover_frames=5, hold_escalate_s=2.0)
    defaults.update(kw)
    return safety.TeleopState(**defaults)


def test_starts_idle():
    assert _sm().state == safety.IDLE


def test_start_goes_to_align_fast():
    sm = _sm()
    sm.start(now=0.0)
    assert sm.state == safety.ALIGN_FAST


def test_align_done_goes_to_following():
    sm = _sm()
    sm.start(now=0.0)
    sm.align_done(now=1.0)
    assert sm.state == safety.FOLLOWING


def test_watchdog_trips_to_holding():
    sm = _sm()
    sm.start(now=0.0)
    sm.align_done(now=1.0)
    assert sm.tick(now=1.0 + 0.1, frame_age=0.1) is safety.FOLLOWING
    # 200 ms 无新帧 ⇒ HOLDING
    assert sm.tick(now=1.0 + 0.35, frame_age=0.35) is safety.HOLDING


def test_hold_escalates_once_after_2s():
    """HOLDING ≥ 2 s ⇒ 升级 movej(q_now) **一次**，且**仍留在 HOLDING**。"""
    sm = _sm()
    sm.start(now=0.0)
    sm.align_done(now=1.0)
    sm.tick(now=1.5, frame_age=0.5)
    assert sm.state == safety.HOLDING
    assert sm.wants_stop_command() is False, "刚进 HOLDING 不该升级"
    sm.tick(now=2.6, frame_age=1.6)                 # 进入 HOLDING 已 1.1 s
    assert sm.wants_stop_command() is False
    sm.tick(now=3.6, frame_age=2.6)                 # 进入 HOLDING 已 2.1 s
    assert sm.wants_stop_command() is True
    assert sm.state == safety.HOLDING, "升级后仍留在 HOLDING"
    sm.consume_stop_command()
    sm.tick(now=4.0, frame_age=2.0)
    assert sm.wants_stop_command() is False, "升级只做一次"


def test_hold_recovers_to_align_fast_not_following():
    """恢复必须重走 ALIGN_FAST —— 主臂在断链期间可能已经动了（spec §5.2）。"""
    sm = _sm()
    sm.start(now=0.0)
    sm.align_done(now=1.0)
    sm.tick(now=1.5, frame_age=0.5)
    assert sm.state == safety.HOLDING
    for k in range(5):                              # 连续 5 拍收到帧
        sm.tick(now=2.0 + 0.01 * k, frame_age=0.01)
    assert sm.state == safety.ALIGN_FAST


def test_hold_recovery_needs_consecutive_frames():
    """不足 5 拍不许恢复（防抖动来回切）。"""
    sm = _sm()
    sm.start(now=0.0)
    sm.align_done(now=1.0)
    sm.tick(now=1.5, frame_age=0.5)
    for k in range(4):
        sm.tick(now=2.0 + 0.01 * k, frame_age=0.01)
    assert sm.state == safety.HOLDING
    sm.tick(now=2.10, frame_age=0.5)                # 又断了 ⇒ 计数清零
    sm.tick(now=2.20, frame_age=0.01)
    assert sm.state == safety.HOLDING


def test_align_failed_stays_put():
    """对齐失败必须停在 ALIGN_FAST —— 且 tick 永远不把它推进 FOLLOWING（spec §5.1）。"""
    sm = _sm()
    sm.start(now=0.0)
    sm.align_failed(now=1.0)
    assert sm.state == safety.ALIGN_FAST
    for k in range(20):
        sm.tick(now=1.0 + 0.01 * k, frame_age=0.01)
        assert sm.state == safety.ALIGN_FAST, "只有 align_done() 才能进 FOLLOWING"


def test_user_stop_from_any_state():
    for prep in (None, "align", "following", "holding"):
        sm = _sm()
        sm.start(now=0.0)
        if prep == "align":
            pass
        else:
            sm.align_done(now=1.0)
        if prep in ("following", "holding"):
            if prep == "holding":
                sm.tick(now=2.0, frame_age=0.5)
        sm.user_stop(now=9.0)
        assert sm.state == safety.IDLE, f"prep={prep}"
        assert sm.wants_stop_command() is True


def test_state_missing_q_blocks_dispatch():
    """拿不到实测 q 就不许下发（spec §5.1 第 4 点）—— 不许拿 0 去猜 err。"""
    sm = _sm()
    sm.start(now=0.0)
    sm.align_done(now=1.0)
    assert sm.may_dispatch(now=1.0, have_slave_q=True) is True
    assert sm.may_dispatch(now=1.0, have_slave_q=False) is False
```

- [x] **Step 2: 跑测试确认失败**

```bash
cd /home/llx/litearm-teleop-isomorphic
python3 -m pytest tests/test_safety.py -q
```

Expected: `AttributeError: module 'liteteleop.safety' has no attribute 'TeleopState'`

- [x] **Step 3: 实现**

**先把文件顶部的 `__all__` 换成下面这一份**（否则 `import *` 拿不到状态机）：

```python
__all__ = [
    "DEFAULT_KD_BUDGET", "LimitsError", "Limits",
    "read_limits_ok", "clamp_to_limits", "saturate_dq", "speed_limit_from_kd",
    "slew_target",
    "IDLE", "ALIGN_FAST", "FOLLOWING", "HOLDING", "TeleopState",
]
```

> ⚠ **`__all__` 必须与文件里实际存在的顶层名字逐一对得上。** part 1 里**没有 `clamp` 函数**
> —— 边界夹取是内联的 `max(lo, min(hi, x))`（见 `slew_target` 内），别照抄"应该有"的名字。
> ⇒ 改完**必须**跑这条判据（它才是"导出面与实现一致"的真判据）：

```bash
python3 -c "
import liteteleop.safety as s
missing = [n for n in s.__all__ if not hasattr(s, n)]
print('缺失:', missing)
raise SystemExit(1 if missing else 0)"
```

Expected: `缺失: []`，退出码 0。

然后追加到 `liteteleop/safety.py` 末尾：

```python
# ────────────────────────── 4. 从臂状态机与 watchdog ──────────────────────────

#: 状态常量（字符串，便于日志与界面直接显示）
IDLE = "IDLE"
ALIGN_FAST = "ALIGN_FAST"
FOLLOWING = "FOLLOWING"
HOLDING = "HOLDING"


class TeleopState:
    """从臂状态机 + watchdog（spec §5）。**纯逻辑，不碰硬件**。

    之所以把状态机独立于"谁发命令"，是因为它的判据全是时间与计数，可完全离线测；
    真正的 `movej`/`move_js`/`park` 由阶段二的 `ArmWorker` 按 `wants_*` 问询式驱动。
    问询式（而不是回调式）让"这个状态该不该发命令"变成可断言的事实。

    状态迁移（spec §5.1）::

        IDLE --start--> ALIGN_FAST --align_done--> FOLLOWING
          ^                ^                          |
          |        连续收帧 recover_frames      frame_age > watchdog
          |                |                          v
          |                +----------------------  HOLDING
          |                                     | 停留 >= hold_escalate_s
          +--user_stop--> (movej 收尾)           ⇒ wants_stop_command() 一次
    """

    def __init__(self, n: int, watchdog_ms: float = 200.0,
                 recover_frames: int = 5, hold_escalate_s: float = 2.0):
        if n < 1:
            raise ValueError(f"关节数需 >= 1（给的是 {n}）")
        if watchdog_ms <= 0.0:
            raise ValueError(f"watchdog_ms 需 > 0（给的是 {watchdog_ms}）")
        if recover_frames < 1:
            raise ValueError(f"recover_frames 需 >= 1（给的是 {recover_frames}）")
        self.n = int(n)
        self.watchdog_s = float(watchdog_ms) / 1000.0
        self.recover_frames = int(recover_frames)
        self.hold_escalate_s = float(hold_escalate_s)

        self.state = IDLE
        self._hold_since: Optional[float] = None
        self._good_frames = 0
        self._stop_pending = False
        self._escalated = False

    # ── 迁移入口 ──────────────────────────────────────────────────────────

    def start(self, now: float) -> None:
        """用户点「启动遥操」。"""
        self.state = ALIGN_FAST
        self._hold_since = None
        self._good_frames = 0
        self._stop_pending = False
        self._escalated = False

    def align_done(self, now: float) -> None:
        """`ALIGN_FAST` 的 `movej` 已到位（含 spec §5.1 的交接补丁）。"""
        if self.state == ALIGN_FAST:
            self.state = FOLLOWING

    def align_failed(self, now: float) -> None:
        """`ALIGN_FAST` 的 `movej` 失败/超时 ⇒ **保持原地**，不自动进 `FOLLOWING`（spec §5.1）。

        刻意比 server 严格：server 的对齐失败只 `log.warning` 一句就继续进 `joint_follow`
        （`teleop_manager.py` 的 `_do_align`，注释写「跟随会逐步修正」）。本仓停住 ——
        因为那一步失败意味着 `q_cmd` **没有同步到对齐位**（server 补丁的前提不成立），
        进 `FOLLOWING` 就是带着一个未知的大误差起步。
        **这是本设计唯一一处刻意比 server 严格的地方**（spec §5.1）。
        """
        self.state = ALIGN_FAST
        self._stop_pending = False

    def user_stop(self, now: float) -> None:
        """用户点「停止」⇒ 收尾 `movej(q_now)`，回 IDLE（spec §5.3）。"""
        self.state = IDLE
        self._hold_since = None
        self._good_frames = 0
        self._escalated = False
        self._stop_pending = True

    # ── 每拍 ──────────────────────────────────────────────────────────────

    def tick(self, now: float, frame_age: float) -> str:
        """每拍调一次。`frame_age` = 距最近一帧的**本地**秒数（0.0 = 从未收到）。

        Returns: 当前状态（与 `self.state` 相同）。
        """
        fresh = frame_age > 0.0 and frame_age <= self.watchdog_s
        if self.state in (FOLLOWING,):
            if not fresh:
                self.state = HOLDING
                self._hold_since = now
                self._good_frames = 0
                self._escalated = False
        elif self.state == HOLDING:
            if fresh:
                self._good_frames += 1
                if self._good_frames >= self.recover_frames:
                    # ⚠ 回 ALIGN_FAST，**不是** FOLLOWING：主臂在断链期间可能已经动了，
                    # 直接跟会阶跃（spec §5.2）
                    self.state = ALIGN_FAST
                    self._hold_since = None
                    self._good_frames = 0
                    self._escalated = False
            else:
                self._good_frames = 0
                if (not self._escalated and self._hold_since is not None
                        and now - self._hold_since >= self.hold_escalate_s):
                    # 长时间持位：`park()` 只把刚度钉回 1.0×、tau 仍为 0 ⇒ 会缓慢下垂。
                    # 补一次 `movej(q_now)` 进 `ht_on`（2× 刚度 + 重力前馈）才真稳。
                    self._stop_pending = True
                    self._escalated = True
        return self.state

    # ── 问询式出口（由阶段二的 ArmWorker 消费） ────────────────────────────

    def wants_stop_command(self) -> bool:
        """是否该发一条收尾 `movej(q_now, speed=0.3)`。读后须 `consume_stop_command()`。"""
        return self._stop_pending

    def consume_stop_command(self) -> None:
        self._stop_pending = False

    def may_dispatch(self, now: float, have_slave_q: bool) -> bool:
        """本拍能否下发 `move_js`。

        ⚠ 拿不到实测 `q` 就**不许下发** —— 公式要 `q_slave实测`，`get_state().value`
        可能为 `None`；**绝不用 0 或上一次的值去猜**（猜出来的 `dq` 会被当速度前馈发出去）。
        （spec §5.1 第 4 点）
        """
        return self.state == FOLLOWING and have_slave_q

    def wants_movej(self) -> bool:
        """是否处于需要 `movej` 的状态（`ALIGN_FAST` / 收尾）。"""
        return self.state == ALIGN_FAST or self._stop_pending
```

- [x] **Step 4: 跑测试确认通过**

```bash
python3 -m pytest tests/test_safety.py -q
```

Expected: 全绿（`passed`，无 `failed`/`error`）

- [x] **Step 5: 跑全套**

```bash
python3 -m pytest tests -q -m "not slow"
```

Expected: 全绿（wire + link + safety，无 `failed`/`error`）

- [x] **Step 6: 提交**

```bash
git add liteteleop/safety.py tests/test_safety.py
git commit -m "feat: add teleop state machine and watchdog to safety layer"
```

---

## Task 7: 收口

**Files:**

- Create: `README.md` 的陷阱章节（**只追加，不重写既有内容**）
- Modify: `pyproject.toml`（若缺 `zenoh` 依赖则补）

- [x] **Step 1: 把 spec §10 的陷阱登记搬进 README**

在 `README.md` 末尾追加一节（内容逐条抄 spec §10 的表格，**不要重新措辞** —— 两份说法会分叉）：

```markdown
## 已知陷阱（同构遥操）

> 完整版见 `docs/superpowers/specs/2026-09-28-isomorphic-teleop-design.md` §10。
> 本表是给"直接看 README 的人"的入口，**条目文字以 spec 为准**。

<table: 逐条抄 spec §10>
```

- [x] **Step 2: 补依赖**

确认 `pyproject.toml` 里有 `zenoh>=1.6`。缺则补，**不要重排既有内容**：

```bash
cd /home/llx/litearm-teleop-isomorphic
grep -n "zenoh" pyproject.toml || echo "⚠ 缺 zenoh 依赖，需手工加进 dependencies"
```

- [x] **Step 3: 全量验收**

```bash
cd /home/llx/litearm-teleop-isomorphic
python3 -m pytest tests -q                       # 含 slow
python3 -m pytest tests -q -m "not slow"
git status --short
git log --oneline main..HEAD | cat
```

Expected: 全绿；`git status` 干净；提交列表见下。

- [x] **Step 4: 提交**

```bash
git add README.md pyproject.toml
git commit -m "docs: add teleop known-traps section to readme"
```

---

## 阶段一验收清单

- [x] `python3 -m pytest tests -q` 全绿（含 `slow`）
- [x] `wire.py` 的字节序有**判别力**证据：改 `>` 后测试变红（Task 3 Step 5 的输出）
- [x] `slew_target` 有**逐拍全等**对拍证据；**变异 A（去制动距离）打红 parity 与
      `brakes_before_target` 两条**（Task 5 Step 5 的输出）；并已知**变异 B 本套件抓不到**，
      该覆盖边界已记入计划 —— **"逐字移植"的守卫是 parity 那一条，不是那一堆**
- [x] 真机 spike S1~S5 的**原始输出**在 `docs/spike-2026-09-28-move-js.md` 里，**板卡与固件版本串已写明**
- [x] Step 5 的来源升级**只动了 §2.3(a) 那条**；§2.3(b) 刚度表与 §5.3 主臂收尾**仍标 `[源码]`**，
      并在 spec 里显式记了一笔"留待另一次真机验证"（**没测的不许升**）
- [x] 若 S3 未过 ⇒ **已停下来报用户**，未继续实现
- [x] `git log main..HEAD` 的提交信息全为 Conventional Commits，**无 `claude` 字样**
- [x] **未 push、未开 PR**

## ⚠ 阶段二必须处理的 4 条（本阶段实现的自查发现，**别丢**）

这四条都是 Task 5/6 交付后由实现者自查发现的，**属于 phase 2 的接法**，不是本阶段的缺陷；
但**没有载体就会静默失效**，所以记在这里：

1. **`may_dispatch(now, have_slave_q)` 忽略 `now`** —— 它不是自足的：只读 `self.state`，
   陈旧性完全依赖 `tick()` 已经把 `FOLLOWING` 降级。⇒ phase 2 的伺服环**必须"先 tick 再 poll"、
   每一拍都如此**；只 poll 不 tick 就会用陈旧数据下发。**这条不变量需要具名载体**（例如把
   `may_dispatch` 改成要求传入"本拍已 tick 过"的凭据），不能只写在文档里。
2. **`_good_frames` 数的是"看到新鲜帧的 tick 数"，不是"不同的帧数"** —— tick 快于帧率时同一帧会被
   重复计数 ⇒ `recover_frames=5` 实际含义是"连续 5 个 tick 周期内帧龄都小于 watchdog"，
   **恢复延迟与 phase 2 的 tick 频率相关**，不是固定 5 帧。文档措辞（「连续收帧 5 拍」）本身如此，
   但真实延迟会随 tick 率变。
3. **`align_failed()` 没有状态守卫**（与对称的 `align_done()` 不同，后者被 `state == ALIGN_FAST` 门控）
   —— 从 `FOLLOWING`/`HOLDING` 调用会**把状态拽回 `ALIGN_FAST`**。按 spec 的用法（只在 align 的
   `movej` 失败时调，那时必在 `ALIGN_FAST`）是良性的；但 phase 2 若从任何状态触发的超时路径里调它，
   这个不对称就变成真实的跳变。⇒ **要么加守卫，要么在 phase 2 明确只从 `ALIGN_FAST` 调，并写测试钉住。**
4. **`user_stop()` 之后 `IDLE` 里 `_stop_pending = True`** ⇒ `wants_movej()` 在 `IDLE` 也为真
   （那正是收尾的 `movej`，刻意如此）。但 `tick()` **永不在 `IDLE` 清 `_stop_pending`**，
   只有 `start()` 与 `consume_stop_command()` 会 ⇒ **phase 2 若忘了 `consume_stop_command()`，
   会每拍重发一次收尾 `movej`。**

## 阶段二（不在本计划内，列此备查）

`arm_worker.py`（独占 `Arm` 的线程 + 状态钩子）、`gui/`（三页 + 闸门）、`ports.py`、`settings.py`、`app.py`、端到端联调。
其中 `arm_worker.py` 的伺服环要用本阶段的 `safety.slew_target` 与 `safety.TeleopState`；
急停旁路线程（spec §7.4）与收尾序列（spec §5.3）在阶段二实现。
