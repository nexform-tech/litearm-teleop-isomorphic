# 同构遥操 阶段一（真机 spike + 协议层 + 安全层）实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use
> superpowers:subagent-driven-development (recommended) or superpowers:executing-plans
> to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 交付同构遥操的第一层 —— 先真机证伪 `move_js` 伺服路径，再实现与 `litearm-python` 零耦合的线协议层（`wire.py`）与 zenoh
点对点链路层（`link.py`），以及与硬件无关的纯逻辑安全层（`safety.py`：`slew_target` 移植、软限位闸门、状态机、watchdog）。

**Architecture:** 本阶段**不碰 GUI、不碰 `ArmWorker`**（那是阶段二）。所有产出要么是纯函数（可离线 TDD），要么是独立可跑的 spike 脚本。`safety.py` 里的 `slew_target` 是**从
`pylitearm` 逐字移植**的 —— 见 spec §5.1，它不是新设计，是既有验证实现的搬运。层次上 `wire.py` / `link.py` / `safety.py` **互不依赖**，可以并行做；三者都**不 import
 `litearm`**（`safety.py` 只用 stdlib `math`），这样离线测试不需要硬件也不需要 SDK。

**Tech Stack:** Python 3.13、`eclipse-zenoh` 1.7.2、`pytest`；spike 脚本额外需要
 `litearm-python`（`PYTHONPATH=/home/llx/litearm-python/src`）与一条 7 关节整臂。

**Spec:** `docs/superpowers/specs/2026-09-28-isomorphic-teleop-design.md`（**唯一权威**；本计划与 spec 冲突时以 spec 为准）

---

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
- Create: `pyproject.toml` 的补充（若已存在则只加依赖，不重写）

- [ ] **Step 1: 建分支**

```bash
cd /home/llx/litearm-teleop-isomorphic
git switch -c feat/teleop-stage1-protocol
git branch --show-current
```

Expected: 输出 `feat/teleop-stage1-protocol`

- [ ] **Step 2: 建包骨架**

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

- [ ] **Step 3: 确认 pytest 能发现空套件**

```bash
cd /home/llx/litearm-teleop-isomorphic
python3 -m pytest tests -q
```

Expected: `no tests ran`（退出码 5）—— 这是**预期的**，说明 pytest 能跑；套件为空是正常的。

- [ ] **Step 4: 提交**

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


def s2_dq_sets_rate(arm, q0):
    print(f"\n=== S2: dq={S2_DQ} 应让实测速率 ≈{S2_DQ} rad/s ===", flush=True)
    tgt = [x for x in q0]
    tgt[1] += S2_OFFSET
    # 先回到起点，避免上一节的残留
    arm.movej(q0, speed=ALIGN_SPEED)
    time.sleep(0.3)
    qs, ts = [], []
    t0 = time.monotonic()
    nxt = t0 + DT
    while time.monotonic() - t0 < 3.0:
        arm.move_js(tgt, [S2_DQ] * arm.n)
        st = arm.get_state(refresh=False).value
        qs.append(list(st.q))
        ts.append(time.monotonic())
        r = nxt - time.monotonic()
        if r > 0:
            time.sleep(r)
        nxt += DT
        if nxt < time.monotonic():
            nxt = time.monotonic() + DT
    # 取中段（避开起步加速与末段到位的减速）
    k = len(qs) // 2
    lo, hi = max(0, k - 25), min(len(qs) - 1, k + 25)
    dq_meas = (qs[hi][1] - qs[lo][1]) / max(ts[hi] - ts[lo], 1e-6)
    ok = abs(dq_meas - S2_DQ) <= S2_RATE_TOL * S2_DQ
    check(f"S2 实测速率 ≈ {S2_DQ} rad/s (±{S2_RATE_TOL:.0%})", ok, f"{dq_meas:.3f} rad/s")


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
    """spike 内联版 slew_target —— 与 pylitearm/control/joint_follow.py:45-88 逐字同构。

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

    print("\n================ 结果 ================", flush=True)
    for name, ok, detail in results:
        print(f"{'✓' if ok else '✗'} {name}" + (f"  [{detail}]" if detail else ""), flush=True)
    n_fail = sum(1 for _, ok, _ in results if not ok)
    print(f"\n{len(results) - n_fail}/{len(results)} 通过", flush=True)
    arm.close()          # ⚠ 必须 close（spec §7.3）
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

- S1/S2/S4/S5 通过 ⇒ spec §5.3 的收尾序列与 §2.3(b) 的三段表**由 `[源码]` 升级为 `[实测]`**。
- **S3 是本轮的核心未知**（spec §11）。若 S3 未通过 ⇒ **停下来找用户裁决**：
  spec §5.1 的 `dq` 公式要改，候选退路见 spec §11 的 S3 段落。
```

- [ ] **Step 5: 若 S1~S5 全绿，回写 spec 的来源等级**

把 spec §2.3(b) 三段表与 §5.3 标题里的 `[源码]` 改成 `[实测]`，并在 §2.3 顶部补一行指向本报告。

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

- [ ] **Step 1: 写失败测试**

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

- [ ] **Step 2: 跑测试确认失败**

```bash
cd /home/llx/litearm-teleop-isomorphic
python3 -m pytest tests/test_wire.py -q
```

Expected: `ModuleNotFoundError: No module named 'liteteleop.wire'`（collection error）

- [ ] **Step 3: 实现**

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

- [ ] **Step 4: 跑测试确认通过**

```bash
python3 -m pytest tests/test_wire.py -q
```

Expected: 全绿（`passed`，无 `failed`/`error`）

- [ ] **Step 5: 反向验证判据有判别力（临时把实现改坏，确认测试会红）**

```bash
# 把字节序从 '<' 改成 '>'，测试必须红 —— 这正是往返测试抓不到的那个错
sed -i 's/return f"<BB{n}f{n}fdI"/return f">BB{n}f{n}fdI"/' liteteleop/wire.py
python3 -m pytest tests/test_wire.py -q
sed -i 's/return f">BB{n}f{n}fdI"/return f"<BB{n}f{n}fdI"/' liteteleop/wire.py
python3 -m pytest tests/test_wire.py -q
```

Expected: 第一次 **FAILED**（黄金字节与变长测试都红），第二次全绿。
**这一步不能省** —— 它证明这组判据真的在测字节序，而不是自我印证。

- [ ] **Step 6: 提交**

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

- [ ] **Step 1: 写失败测试**

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

- [ ] **Step 2: 跑测试确认失败**

```bash
cd /home/llx/litearm-teleop-isomorphic
python3 -m pytest tests/test_link.py -q
```

Expected: `ModuleNotFoundError: No module named 'liteteleop.link'`

- [ ] **Step 3: 实现**

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

__all__ = ["DEFAULT_KEY", "Listener", "Connector"]

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
        （已核 1.7.2：`dir(zenoh.MatchingStatus) == ['matching']`）。界面显示「已匹配/未匹配」。
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
        #: 计数（只读，供界面显示；累加只在 zenoh 线程内，GIL 下原子）
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

- [ ] **Step 4: 跑测试确认通过**

先在 `pyproject.toml` 注册 marker（未注册的 mark 会产生警告；用 `--strict-markers` 时直接报错）：

```toml
[tool.pytest.ini_options]
markers = ["slow: 慢测（起子进程 / 数十秒），用 -m \"not slow\" 跳过"]
```

```bash
python3 -m pytest tests/test_link.py -q -m "not slow"
python3 -m pytest tests/test_link.py -q -m slow
```

Expected: 第一条全绿；第二条 `1 passed`（子进程测试，约 20~40 s）。

> 若第二条红：说明「不 close 会挂死」这条已不再成立（zenoh 改了行为）。**不要删测试** ——
> 它是那条纪律的唯一守卫；改成断言新行为并在此处留下说明。

- [ ] **Step 5: 提交**

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
> 且它是**从既有验证实现逐字移植**的（`pylitearm/control/joint_follow.py:45-88`）。
> ⛔ **不要"改进"它** —— 任何偏离都必须先报用户裁决。移植已知边界见 Step 6。

- [ ] **Step 1: 写对拍测试（**与 pylitearm 原版逐拍全等**）**

`tests/test_safety.py`：

```python
"""safety.py —— 纯逻辑层（不 import litearm，不碰硬件）。

⭐ 核心判据：移植的 `slew_target` 与 `pylitearm` 原版**同一组输入逐拍全等**。
这是"照抄而非重写"的唯一硬证据；没有它，"我照抄了"只是自述。
"""
import math

import pytest

from liteteleop import safety


# ── 参照实现：从 pylitearm/control/joint_follow.py:45-88 抄成独立副本 ──────────
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


def test_slew_does_not_overshoot():
    """制动距离减速生效：全程不过冲（这是原版比"朴素限速"强的地方）。"""
    n, dt = 7, 0.01
    tgt = 0.5
    q_cmd, dq_cmd = [0.0] * n, [0.0] * n
    worst = 0.0
    for _ in range(400):
        q_cmd, dq_cmd = safety.slew_target([tgt] * n, q_cmd, dq_cmd, [2.0] * n, [14.0] * n, dt)
        worst = max(worst, max(q_cmd) - tgt)
    assert worst <= 1e-9, f"过冲 {worst}"
```

- [ ] **Step 2: 跑测试确认失败**

```bash
cd /home/llx/litearm-teleop-isomorphic
python3 -m pytest tests/test_safety.py -q
```

Expected: `ModuleNotFoundError: No module named 'liteteleop.safety'`

- [ ] **Step 3: 实现 `safety.py` 第一部分（`slew_target` + 软限位 + 钳位）**

`liteteleop/safety.py`：

```python
"""遥操的纯逻辑层 —— 不 import litearm / zenoh，不碰硬件，可整段离线 TDD。

分四块（都在这一个文件里，因为它们耦合成一条链：限位 → 钳位 → 参考生成 → 状态机）:

1. `slew_target` —— **从 `pylitearm/control/joint_follow.py:45-88` 逐字移植**
   的梯形速度曲线参考生成器（spec §5.1）。它不是新设计，是既有验证实现的搬运。
   ⛔ 不要"改进"它；任何偏离都先报用户裁决。
2. `read_limits` / `clamp_to_limits` —— 软限位闸门与钳位（spec §7.1）。
3. `speed_limit_from_kd` —— 逐轴速度上限的 kd 预算闸门（spec §5.1）。
4. `TeleopState` —— 从臂状态机与 watchdog（spec §5）。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

__all__ = [
    "clamp", "slew_target", "clamp_to_limits", "LimitsError",
    "read_limits_ok", "speed_limit_from_kd",
]


def clamp(v: float, lo: float, hi: float) -> float:
    """与 `pylitearm/hal/hardware.py:52` 的 `clamp` 同义。"""
    return max(lo, min(hi, v))


# ────────────────────────── 1. slew_target（逐字移植） ──────────────────────────

def slew_target(raw_target: Sequence[float],
                q_cmd: List[float],
                dq_cmd: List[float],
                speed_limit: Sequence[float],
                accel_limit: Sequence[float],
                dt: float) -> Tuple[List[float], List[float]]:
    """速度/加速度限制的目标位置平滑（梯形速度曲线）。

    **逐字移植自 `pylitearm/control/joint_follow.py:45-88`**（spec §5.1）。
    唯一的形式改动：把原版的模块级全局 `N` 换成 `len(raw_target)`，
    以支持变长关节数（线协议是变长的，spec §4.2）。

    对每个关节：限最大速度 `speed_limit[i]`、限最大加速度 `accel_limit[i]`、
    接近目标时**按制动距离 `v²/(2a)` 自动减速**。

    ⚠ **`|dq_cmd|` 恒 ≤ `speed_limit`**（原版 `v = clamp(v, -v_limit, v_limit)`）
    ⇒ `accel_limit` 只决定爬到上限有多快，**不抬高天花板**（spec §5.1）。

    Args:
        raw_target: 原始目标位置 [n] (rad)
        q_cmd: 当前指令位置 [n]，**就地修改**
        dq_cmd: 当前指令速度 [n]，**就地修改**
        speed_limit: 每关节最大速度 [n] (rad/s)
        accel_limit: 每关节最大加速度 [n] (rad/s^2)
        dt: 控制周期 (s)

    Returns:
        (q_cmd, dq_cmd)
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
        v += clamp(desired_v - v, -dv_max, dv_max)
        v = clamp(v, -v_limit, v_limit)

        # 更新位置
        step = v * dt
        if diff * step > 0.0 and abs(step) >= abs(diff):
            q_cmd[i] = raw_target[i]
            dq_cmd[i] = 0.0
        else:
            q_cmd[i] += step
            dq_cmd[i] = v

    return q_cmd, dq_cmd


# ────────────────────────── 2. 软限位闸门与钳位 ──────────────────────────

class LimitsError(RuntimeError):
    """软限位不可用 —— **必须拒绝启动跟随**，不许静默退化（spec §7.1）。"""


@dataclass(frozen=True)
class Limits:
    lo: Tuple[float, ...]
    hi: Tuple[float, ...]

    @property
    def n(self) -> int:
        return len(self.lo)


def read_limits_ok(lo: Sequence[float], hi: Sequence[float], n: int) -> Limits:
    """校验并封装软限位。任何一处不合法 ⇒ 抛 `LimitsError`。

    ⚠ **不学 litearm-server 的 ±9 兜底告警**（spec §7.1）：它拿不到限位时用一个哨兵值
    继续跑并只打一条 warning，等于把位置护栏降级成"看起来在工作"。
    本工具**拒绝启动**。

    限位来源：`arm.params.all_joint_params()` 的 `q_min`/`q_max`
    （= 固件 `ctrl_accept_move_js` 里钳位用的**同一个** `g_params.joint[].q_min/q_max`
    ⇒ 与固件同源，比 pylitearm 的 `kin.q_min/q_max` 更权威。spec §9.2）
    """
    if len(lo) != n or len(hi) != n:
        raise LimitsError(f"软限位长度不符: lo={len(lo)} hi={len(hi)} 关节数={n}")
    for i, (a, b) in enumerate(zip(lo, hi)):
        if not (math.isfinite(a) and math.isfinite(b)):
            raise LimitsError(f"J{i + 1} 软限位非有限值: lo={a} hi={b}")
        if a >= b:
            raise LimitsError(f"J{i + 1} 软限位上下界反了: lo={a} >= hi={b}")
    return Limits(lo=tuple(float(x) for x in lo), hi=tuple(float(x) for x in hi))


def clamp_to_limits(q: Sequence[float], lim: Limits) -> Tuple[List[float], List[bool]]:
    """把目标钳到软限位，并返回**哪些轴被钳住了**。

    被钳住的轴必须把 `dq` 置 0（抗饱和）—— 否则它以 `kd·dq` 持续顶进硬限位（spec §5.1）。
    这不是边角情况：本机 **J4 上端只有 +0.0175 rad**。
    """
    if len(q) != lim.n:
        raise LimitsError(f"目标长度不符: {len(q)} vs {lim.n}")
    out, sat = [], []
    for i, x in enumerate(q):
        c = clamp(x, lim.lo[i], lim.hi[i])
        out.append(c)
        sat.append(x < lim.lo[i] or x > lim.hi[i])
    return out, sat


# ────────────────────────── 3. kd 预算闸门 ──────────────────────────

def speed_limit_from_kd(kd: Sequence[float], tau_max: Sequence[float],
                        kd_budget: float = 0.30) -> List[float]:
    """由 kd 预算反推逐轴速度上限（spec §5.1）。

    为什么需要它：`dq` 进的是 τ 域的 `kd·(dq_ref − dq)`，而 `move_js` **给不了 `K`/`B`**
    —— 刚度/阻尼由固件定死（`kp=mit_kp`，有效阻尼 `mit_kd + kd_extra`；`kd_extra`
    在 τ 域叠加，`control_loop.c:2443`）。本机 J4 的 `kd` 是 **11.0**，
    而 litearm-server 自己发 MIT 帧时用的是 `B=1.0` ⇒ **差 11 倍**。
    τ 最终钳到 `±tau_max` ⇒ 前馈吃不坏硬件，但会**吃掉整个力矩预算**。

    ⇒ `speed_limit_j = kd_budget · tau_max_j / kd_j`
    """
    if len(kd) != len(tau_max):
        raise LimitsError(f"kd/tau_max 长度不符: {len(kd)} vs {len(tau_max)}")
    if not 0.0 < kd_budget <= 1.0:
        raise LimitsError(f"kd_budget 需 ∈ (0, 1]（给的是 {kd_budget}）")
    out = []
    for i, (k, t) in enumerate(zip(kd, tau_max)):
        if not (math.isfinite(k) and math.isfinite(t)) or k <= 0.0 or t <= 0.0:
            raise LimitsError(f"J{i + 1} 的 kd/tau_max 非法: kd={k} tau_max={t}")
        out.append(kd_budget * t / k)
    return out
```

- [ ] **Step 4: 补限位与闸门的测试，并跑**

把下面这些**追加**到 `tests/test_safety.py` 末尾：

```python
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

```bash
python3 -m pytest tests/test_safety.py -q
```

Expected: 全绿（`passed`，无 `failed`/`error`）

- [ ] **Step 5: 确认对拍判据有判别力（临时"改进"移植版，测试必须红）**

```bash
# 把制动距离减速去掉（这正是"朴素限速"与"原版"的差别），对拍必须红
cp liteteleop/safety.py /tmp/safety.bak
python3 - <<'PY'
p="liteteleop/safety.py"; s=open(p,encoding="utf-8").read()
s=s.replace("if moving_toward and abs(diff) <= stopping_dist:\n            desired_v = 0.0  # 开始减速\n        else:",
            "if False:\n            desired_v = 0.0\n        else:")
open(p,"w",encoding="utf-8").write(s)
PY
python3 -m pytest tests/test_safety.py -q -k "matches_pylitearm or overshoot"
cp /tmp/safety.bak liteteleop/safety.py
python3 -m pytest tests/test_safety.py -q
```

Expected: 中间那次 **FAILED**（对拍与过冲两条都红），最后一次全绿。
**这一步证明这组判据真的在测"逐字移植"，不是在自我印证。**

- [ ] **Step 6: 记录移植已知边界**

在 `safety.py` 的 `slew_target` docstring 末尾追加（**不改逻辑**）：

```python
    ⚠ **移植已知边界（照抄，不在本计划修）**：`diff` 恰为 0 或落在 `±1e-5` 死区内、
    而 `|v| ≥ dv_max` 时，`math.copysign(v_limit, diff)` 在 `diff == 0` 会返回 `+v_limit`
    ⇒ 该轴可能继续正向加速而非停住（原版同样如此）。实践中 `diff` 极少恰为 0，
    且 `abs(diff) < 1e-5 and abs(v) < dv_max` 已挡掉绝大多数情形。
    **忠实移植优先**；若真机 S3 观察到自激，这是嫌疑点之一（见 spec §11 S3）。
```

- [ ] **Step 7: 提交**

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

- [ ] **Step 1: 写失败测试**

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

- [ ] **Step 2: 跑测试确认失败**

```bash
cd /home/llx/litearm-teleop-isomorphic
python3 -m pytest tests/test_safety.py -q
```

Expected: `AttributeError: module 'liteteleop.safety' has no attribute 'TeleopState'`

- [ ] **Step 3: 实现**

追加到 `liteteleop/safety.py` 末尾：

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

- [ ] **Step 4: 跑测试确认通过**

```bash
python3 -m pytest tests/test_safety.py -q
```

Expected: 全绿（`passed`，无 `failed`/`error`）

- [ ] **Step 5: 跑全套**

```bash
python3 -m pytest tests -q -m "not slow"
```

Expected: 全绿（wire + link + safety，无 `failed`/`error`）

- [ ] **Step 6: 提交**

```bash
git add liteteleop/safety.py tests/test_safety.py
git commit -m "feat: add teleop state machine and watchdog to safety layer"
```

---

## Task 7: 收口

**Files:**

- Create: `README.md` 的陷阱章节（**只追加，不重写既有内容**）
- Modify: `pyproject.toml`（若缺 `zenoh` 依赖则补）

- [ ] **Step 1: 把 spec §10 的陷阱登记搬进 README**

在 `README.md` 末尾追加一节（内容逐条抄 spec §10 的表格，**不要重新措辞** —— 两份说法会分叉）：

```markdown
## 已知陷阱（同构遥操）

> 完整版见 `docs/superpowers/specs/2026-09-28-isomorphic-teleop-design.md` §10。
> 本表是给"直接看 README 的人"的入口，**条目文字以 spec 为准**。

<table: 逐条抄 spec §10>
```

- [ ] **Step 2: 补依赖**

确认 `pyproject.toml` 里有 `zenoh>=1.7`。缺则补，**不要重排既有内容**：

```bash
cd /home/llx/litearm-teleop-isomorphic
grep -n "zenoh" pyproject.toml || echo "⚠ 缺 zenoh 依赖，需手工加进 dependencies"
```

- [ ] **Step 3: 全量验收**

```bash
cd /home/llx/litearm-teleop-isomorphic
python3 -m pytest tests -q                       # 含 slow
python3 -m pytest tests -q -m "not slow"
git status --short
git log --oneline main..HEAD | cat
```

Expected: 全绿；`git status` 干净；提交列表见下。

- [ ] **Step 4: 提交**

```bash
git add README.md pyproject.toml
git commit -m "docs: add teleop known-traps section to readme"
```

---

## 阶段一验收清单

- [ ] `python3 -m pytest tests -q` 全绿（含 `slow`）
- [ ] `wire.py` 的字节序有**判别力**证据：改 `>` 后测试变红（Task 3 Step 5 的输出）
- [ ] `slew_target` 有**逐拍全等**对拍证据，且"去掉制动距离"后测试变红（Task 5 Step 5 的输出）
- [ ] 真机 spike S1~S5 的**原始输出**在 `docs/spike-2026-09-28-move-js.md` 里，**板卡与固件版本串已写明**
- [ ] 若 S3 未过 ⇒ **已停下来报用户**，未继续实现
- [ ] `git log main..HEAD` 的提交信息全为 Conventional Commits，**无 `claude` 字样**
- [ ] **未 push、未开 PR**

## 阶段二（不在本计划内，列此备查）

`arm_worker.py`（独占 `Arm` 的线程 + 状态钩子）、`gui/`（三页 + 闸门）、`ports.py`、`settings.py`、`app.py`、端到端联调。
其中 `arm_worker.py` 的伺服环要用本阶段的 `safety.slew_target` 与 `safety.TeleopState`；
急停旁路线程（spec §7.4）与收尾序列（spec §5.3）在阶段二实现。
