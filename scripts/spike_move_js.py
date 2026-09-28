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
import os
import statistics
import sys
import time

#: ⚠⚠ SDK 的**唯一入口**（用户裁决）。本机 `sys.path` 上还挂着另一份 `litearm`
#: （`/home/llx/gitee/litearm-python/src`，停在 `chore/sync-repo-standards` 分支）
#: —— 它会被**静默**import 到。顶到最前压掉它，并**断言**导入来源。
#: 记忆：旧 editable 会把漏改名静默兜住 ⇒ 唯一硬判据是把旧仓从 `sys.path` 剥掉。
SDK_SRC = "/home/llx/litearm-python/src"
sys.path.insert(0, SDK_SRC)

import litearm as pa                                            # noqa: E402

assert pa.__file__.startswith(SDK_SRC), (
    f"⛔ litearm 导入自 {pa.__file__}，不是 {SDK_SRC} —— 环境里有另一份抢先了")

# ⚠ spike 必须用**出货的** `safety.slew_target`，不许内联副本 ——
# 内联过一版取**标量**上限的 `_slew`，与出货版（逐轴 `speed_limit[i]`）不是同一个函数，
# 那样 spike 报的数字出自另一个实现，而且照它的调用约定调出货版会 TypeError。
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from liteteleop import safety      # noqa: E402

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
# ⚠ 幅值/频率**不在这里定**：由「承重轴」的 `speed_limit_j` 现算（见 s3_...）。
#   早先写死 `S3_AMP=0.25` 驱动 J2，只到它设计上限的 37% —— 永远碰不到承重轴 J4。
S3_SEC = 20.0
S3_RMS_TOL = 0.05            # rad

#: spike 的**基准位姿** —— ⛔ 不许从 URDF 零位跑。两个理由都是硬事实：
#:  ① URDF 零位是**奇异**位姿（σmin≈0）；`TEST_Q0` 的 σmin=0.0795 良态
#:     （`litearm-stm32/tools/cartesian_check.py:118,1480`）。
#:  ② ⚠ **J4 在零位几乎没有余量**：它的软限位是 [-3.0715, +0.0175]，而零位 q4≈0
#:     ⇒ S3（驱动承重轴 J4 ±0.5·speed_limit_j）的**正向会被全部钳住**，
#:     量到的是"顶住限位"而不是跟随 —— 假绿或假红。`TEST_Q0` 的 J4=-1.20 ⇒ 到上端还有 +1.2175。
BASELINE_Q = [0.0, -0.60, 0.90, -1.20, 0.0, 0.50, 0.0]      # = cartesian_check.TEST_Q0
#: 各轴在本次 spike 里需要的最大偏移（用于余量守卫）：S2 驱动 J2 +1.0；S3 驱动承重轴 ±0.5·speed_limit_j
NEED_OFFSET = [0.0, 1.0, 0.0, 0.60, 0.0, 0.0, 0.0]
# S4: 100 Hz ACK 稳定性
S4_SEC = 30.0
# S5: 收尾 A/B 对照
S5_DRIFT_SEC = 30.0

results: list[tuple] = []

#: 当前会话的 `Arm`（0 或 1 个）。存在的唯一理由是让**顶层** `finally` 能无条件
#: `close()` —— 2026-09-28 那次 spike 崩在 S5，异常从 `s5_stop_ab` 一路穿出 `main()`，
#: 跳过了尾部那句 `arm.close()` ⇒ **进程永久挂死**，被 `timeout` 杀成 `EXIT=124`。
#: ⚠ 用"往列表里放"而不是"给全局赋值"，这样 `main()` 里不需要 `global` 声明。
_OPEN_ARM: list = []


def _hard_close() -> None:
    """⛔ **无条件** close，幂等。忘了它进程退出会**永久挂死**（spec §2.1 实测）。"""
    while _OPEN_ARM:
        a = _OPEN_ARM.pop()
        try:
            a.close()
        except Exception as e:                      # noqa: BLE001
            print(f"⚠ close() 失败: {e}", flush=True)


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


def send_move_js(arm, q, dq, tag: str = "") -> None:
    """发 `move_js`；**被固件拒时把发出去的值打出来**再抛。

    ⚠ 这**不是预检** —— 真闸门在固件侧（spec §2.3：固件在协议边界逐值判 `f32_finite`），
    本函数不改任何行为。它存在的唯一理由是：2026-09-28 两次 `ERR{0x03,0x02}`
    （"q/dq(/tau_ff) 含非有限值"）崩了之后**没人知道是哪一维坏的**，只能靠猜。
    现在它会把 `q`/`dq` 全打出来，并把非有限的分量点名。
    """
    try:
        arm.move_js(list(q), list(dq))
    except Exception as e:                          # noqa: BLE001 - spike 要看全部失败
        nf = [(k, i, x) for k, arr in (("q", q), ("dq", dq))
              for i, x in enumerate(arr) if not math.isfinite(x)]
        print(f"    ⛔ move_js 被拒 [{tag or '?'}]: {type(e).__name__}: {e}\n"
              f"       q  = {[round(x, 4) for x in q]}\n"
              f"       dq = {[round(x, 4) for x in dq]}\n"
              f"       非有限分量 = {nf if nf else '（无 —— 客户端发出去的值全是有限的）'}",
              flush=True)
        raise


def spin(arm, q, dq, secs: float) -> None:
    """按 100 Hz 喂 move_js，持续 secs 秒。q/dq 为序列。"""
    t0 = time.monotonic()
    nxt = t0 + DT
    while time.monotonic() - t0 < secs:
        send_move_js(arm, q, dq, "spin")
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


def _axis_limits(arm, kd_budget=0.30):
    """读各轴 `kd` / `tau_max` / `kd_extra`，算**逐轴** `speed_limit_j`（spec §5.1）。

    ⚠ 这是判据③④的基准。早先版本用 `peak_dq * 1.35` 这个拍脑袋的标量，
    于是**永远碰不到承重轴**（§5.1 论证承重的是 J4，0.573 rad/s）——
    S3 按原样跑永远回答不了 spec 要它回答的问题。
    """
    jp = [arm.params.get_joint_param(i).value for i in range(arm.n)]
    # ⚠⚠ `kd_extra` 住在 **0x26 向量表** (`FF_VEC_ITEMS[15]`)，**不是** 0x28 标量表。
    #    `get_ff_scalar(15, sub)` 取到的是标量表的 item 15 = `zg_engage_kp`（≈304，
    #    一个完全不相干的数）被当成 kd_extra 加上去；而 `sub` 只容许 0..2 ⇒ J4~J7
    #    抛 `InvalidCommandError` 后又被 except 吞掉退回 0。
    #    ⇒ 2026-09-28 真机就是这么崩的：承重轴被算成 **J3**、上限 0.021 rad/s。
    #    正确读法是一次拿全 7 轴：`get_ff_vec(15).value -> list[float]`。
    # ⚠ 读失败必须**往安全一侧倒**：kd_extra 记 0 会让 kd 变**小** ⇒ speed_limit 变**宽**
    #    （J4 从 0.573 放到 1.260，最紧轴还会从 J4 误变成 J5）—— 那是错在危险一侧。
    #    固件 `params.c:153` 的钳幅上界是 50.0，取它作保守替代。
    try:
        kd_extra = [float(x) for x in arm.get_ff_vec(15).value]
        if len(kd_extra) != arm.n:
            raise ValueError(f"长度 {len(kd_extra)} != n={arm.n}")
    except Exception as e:                          # noqa: BLE001
        print(f"    ⚠ 读 kd_extra 失败: {e} —— 按固件钳幅上界 50.0 保守估算"
              f"（⛔ 不许退回 0：那会让 speed_limit 变**宽**）", flush=True)
        kd_extra = [50.0] * arm.n
    print(f"    mit_kd={[round(p.kd, 2) for p in jp]}  "
          f"kd_extra={[round(x, 2) for x in kd_extra]}", flush=True)
    kd = [p.kd + e for p, e in zip(jp, kd_extra)]
    tau_max = [p.tau_max for p in jp]
    return kd, tau_max, safety.speed_limit_from_kd(kd, tau_max, kd_budget)


def s3_high_speed_feedforward(arm, q0):
    print("\n=== S3: 遥操速度量级下的速度前馈（**驱动承重轴**）===", flush=True)
    kd, tau_max, sl = _axis_limits(arm)
    print(f"    逐轴 speed_limit_j = {[round(x, 3) for x in sl]}", flush=True)
    j = min(range(arm.n), key=lambda k: sl[k])          # 承重轴 = 上限最小的那根
    print(f"    承重轴 = J{j + 1}：上限 {sl[j]:.3f} rad/s"
          f"（kd={kd[j]:.1f}, tau_max={tau_max[j]:.1f}）", flush=True)
    amp, hz = 0.5 * sl[j], 0.5
    peak = 2 * math.pi * hz * amp
    sp = [sl[j] * 1.2] * arm.n
    ac = [max(4.0 * peak, 14.0)] * arm.n
    arm.movej(q0, speed=ALIGN_SPEED)
    time.sleep(0.3)
    q_cmd, dq_cmd = list(q0), [0.0] * arm.n
    errs, tau_seen, dq_seen = [], [], []
    t0 = time.monotonic()
    nxt = t0 + DT
    while time.monotonic() - t0 < S3_SEC:
        t = time.monotonic() - t0
        raw = list(q0)
        raw[j] += amp * math.sin(2 * math.pi * hz * t)
        q_cmd, dq_cmd = safety.slew_target(raw, q_cmd, dq_cmd, sp, ac, DT)
        send_move_js(arm, q_cmd, dq_cmd, f"S3 t={t:.2f} amp={amp:.4f} hz={hz}")
        st = arm.get_state(refresh=False).value
        if st is None:
            raise RuntimeError("状态缓存为空 —— 见 spec §5.1：拿不到实测 q 不许下发/判据")
        errs.append(abs(q_cmd[j] - st.q[j]))
        tau_seen.append(abs(st.tau[j]))                 # ← 判据③的经验交叉核对
        dq_seen.append(abs(dq_cmd[j]))
        r = nxt - time.monotonic()
        if r > 0:
            time.sleep(r)
        nxt += DT
        if nxt < time.monotonic():
            nxt = time.monotonic() + DT

    # ④ |dq_cmd| ≤ speed_limit_j —— 纯逻辑，与硬件无关，但也必须成立
    worst_dq = max(dq_seen)
    check(f"S3④ |dq_cmd| ≤ speed_limit_j={sl[j]:.3f}",
          worst_dq <= sl[j] + 1e-9, f"峰值 {worst_dq:.3f} rad/s")
    # ③ kd·|dq_cmd| 不超 kd_budget·tau_max（算式）
    kd_tau = kd[j] * worst_dq
    budget = safety.DEFAULT_KD_BUDGET * tau_max[j]
    check(f"S3③ kd·|dq_cmd| ≤ 预算 {budget:.2f} Nm",
          kd_tau <= budget + 1e-6, f"{kd_tau:.2f} Nm")
    print(f"    （经验交叉核对：实测峰值 |tau(J{j + 1})| = {max(tau_seen):.2f} Nm，"
          f"tau_max = {tau_max[j]:.1f}；⚠ 它含重力与刚度项，不是纯前馈）", flush=True)
    # 丢掉前 20% 的起步段
    body = errs[len(errs) // 5:]
    rms = statistics.fmean([e * e for e in body]) ** 0.5
    check(f"S3① 跟踪误差 rms ≤ {S3_RMS_TOL} rad（承重轴 J{j + 1}）",
          rms <= S3_RMS_TOL, f"rms={rms:.4f} rad, max={max(body):.4f}")
    h = len(body) // 2
    pp1 = max(body[:h]) - min(body[:h])
    pp2 = max(body[h:]) - min(body[h:])
    check("S3② 无自激（后半段误差峰峰值未显著放大）", pp2 <= pp1 * 2.0 + 0.01,
          f"前半 {pp1:.4f} / 后半 {pp2:.4f}")


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
    _OPEN_ARM.append(arm)          # 登记，供顶层 finally 兜底 close
    arm.connect()
    assert arm.move_timeout == 3.0, f"move_timeout 被改成 {arm.move_timeout} —— 见 spec §5.3"
    print(f"已连接: n={arm.n}", flush=True)
    print(f"SDK: {pa.__file__}", flush=True)      # ⚠ 每次都核，别信"我应该设对了"
    if arm.n != 7:
        print(f"⛔ 本 spike 只对 7 关节整臂有效，当前 n={arm.n} —— 疑似 1J 台架，退出", flush=True)
        _hard_close()
        return 2
    if not a.yes:
        input("确认臂周围无障碍、急停可及、是 7 关节整臂。回车继续，Ctrl-C 中止: ")
    arm.enable()
    st = arm.get_state(refresh=True).value
    if st.faulted or st.joint_fault:
        print(f"⛔ 起始状态不干净: {st.fault_detail}", flush=True)
        _hard_close()
        return 3

    # ── 余量守卫：本 spike 会让 J2 走 +1.0、承重轴走 ±0.5·speed_limit_j ──
    # ⚠ 少了这一步，从 URDF 零位起跑时 J4 的正向偏移会被软限位**全部钳住**
    # （J4 上端只有 +0.0175），S3 于是量到"顶住限位"而不是跟随 —— 假结论。
    lp = arm.params.all_joint_params()
    for i, need in enumerate(NEED_OFFSET):
        if need <= 0.0:
            continue
        rng = (lp[i].q_max - lp[i].q_min) / 2.0
        if need > rng:
            print(f"⛔ J{i + 1} 需要 ±{need} rad 的余量，但它全行程只有 {2 * rng:.3f} rad "
                  f"（[{lp[i].q_min:.4f}, {lp[i].q_max:.4f}]）—— 该轴的判据会量到钳位而非跟随。"
                  f"换个位姿或跳过相关节。", flush=True)
            _hard_close()
            return 4
    print("余量守卫通过（各轴行程足够）", flush=True)

    # ── 移到基准位姿：不许从 URDF 零位跑（奇异 + J4 无余量，见 BASELINE_Q 注释）──
    print(f"移到基准位姿 {BASELINE_Q} ...", flush=True)
    arm.movej(BASELINE_Q, speed=0.3)
    q0 = read_q_cached(arm)
    print(f"基准位姿就位 q = {[round(x, 4) for x in q0]}", flush=True)
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

    try:
        print("\n================ 结果 ================", flush=True)
        for name, ok, detail in results:
            print(f"{'✓' if ok else '✗'} {name}" + (f"  [{detail}]" if detail else ""), flush=True)
        n_fail = sum(1 for _, ok, _ in results if not ok)
        print(f"\n{len(results) - n_fail}/{len(results)} 通过", flush=True)
    finally:
        _hard_close()
    return 1 if n_fail else 0
if __name__ == "__main__":
    rc = 1
    try:
        rc = main()
    except BaseException:
        # ⛔ 异常退出也必须**受控接管**（spec §5.3）：绝不 disable
        #    （失能 ⇒ 自由落体）。正常路径的收尾在 main 里，这里兜的是
        #    connect/enable/余量守卫/基准 movej 这些 try 之外的分支。
        try:
            if _OPEN_ARM:
                _OPEN_ARM[-1].movej(read_q(_OPEN_ARM[-1]), speed=ALIGN_SPEED)
        except Exception as _e:                 # noqa: BLE001
            print(f"⚠ 异常收尾 movej 失败: {_e}", flush=True)
        raise
    finally:
        _hard_close()
    sys.exit(rc)
