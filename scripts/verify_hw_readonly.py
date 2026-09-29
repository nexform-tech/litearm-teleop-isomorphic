#!/usr/bin/env python3
"""主臂**只读**核对 —— 不使能、不动作、不改任何参数。

用法::

    python scripts/verify_hw_readonly.py                       # 自动按 VID:PID 找口
    python scripts/verify_hw_readonly.py --port /dev/ttyACM0

本脚本对臂**只发查询类命令**（spec §2 的只读面）：`0x42 GET_FIRMWARE`（`connect()`
里那条）、被动 100 Hz 状态流、`0x24 GET_JOINT_PARAM`、`0x2F GET_LICENSE`。
⛔ 本文件**不包含** `enable/disable/movej/move_js/move_p/park/zero_g_*/emergency_stop`
任何一条 —— 加进来之前请先想清楚：那就不再是"核对"而是"动臂"了。

它回答四个问题：
  1. 链路通不通、固件版本够不够（`MIN_FW`）
  2. **是不是 7 关节整臂**（⛔ 1J 台架与整臂是两张互斥的默认值表，见 spec §2.3）
  3. 此刻每轴的温度 / 力矩 / `err`（`err==1` 使能、`err==0` 失能 —— 用户裁决口径）
  4. 由**实测** `kd`/`tau_max` 现算的逐轴 `speed_limit`（spec §5.1 的闸门）

⚠ 收尾**必须显式 `close()`** —— 忘了它进程退出会永久挂死（见 memory）。
"""
from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

#: ⚠⚠ SDK 的**唯一入口**（用户裁决）。本机 `sys.path` 上还挂着另一份 `litearm`
#: （`/home/llx/gitee/litearm-python/src`，停在 `chore/sync-repo-standards` 分支）
#: —— 它会被**静默**import 到。本行把 SDK 路径顶到最前，压掉那份。
#: 判据不是"我设过 PYTHONPATH"，而是**导入后打印出来核**（见下方 `实际导入`）。
SDK_SRC = "/home/llx/litearm-python/src"
sys.path.insert(0, SDK_SRC)

import litearm as pa                                            # noqa: E402
from litearm.arm import MIN_FW                                  # noqa: E402
from liteteleop import safety, servo                            # noqa: E402

#: 整臂轴数。⛔ 不是"期望值"而是**判据**：1J 台架板在这里会显形。
ARM_JOINTS = 7

#: `err` 口径（用户裁决）：1 = 使能, 0 = 失能, 其余 = 电机自报故障。
ERR_ENABLED = 1
ERR_DISABLED = 0


def v(x: float, w: int = 8, p: int = 4) -> str:
    return f"{x:>{w}.{p}f}"


def main() -> int:
    ap = argparse.ArgumentParser(description="主臂只读核对（不动臂）")
    ap.add_argument("--port", default=None,
                    help="CDC 口；缺省按 VID:PID 自动找（⚠ 别按 ttyACM0/1 编号认）")
    args = ap.parse_args()

    port = args.port or pa.find_cdc_port()
    if not port:
        print("⛔ 找不到 STM32 CDC 口（VID:PID 1d50:606f）。检查 USB 与权限。")
        return 2
    # ⚠ 把**实际导入到的那份**打出来核 —— 别信"我应该设对了"
    print(f"SDK: {pa.__file__}")
    if not pa.__file__.startswith(SDK_SRC):
        print(f"⛔ 导入到的不是 {SDK_SRC} —— 环境里有另一份 litearm 抢先了。")
        return 5
    print(f"端口: {port}")
    print("=" * 78)

    # ⚠ `move_timeout` 调小：本脚本不发运动命令，但一旦哪天有人加了，15s 的默认窗口
    #   会让"等到位"变成一个摸不到头的阻塞。3s 是 spec 里给 spike 定的同款值。
    arm = pa.Arm(port=port, move_timeout=3.0)
    try:
        arm.connect()
        print(f"固件: {arm.firmware!r}   n = {arm.n}")
        if arm.fw_version and arm.fw_version < tuple(MIN_FW):
            print(f"⚠ 固件 {arm.fw_version} 低于 MIN_FW {tuple(MIN_FW)}")
        print(f"轴数判定: {'✓ 7 关节整臂' if arm.n == ARM_JOINTS else f'⛔ 不是整臂 (n={arm.n})'}")
        if arm.n != ARM_JOINTS:
            print("⛔ 停在这里 —— 1J 台架与整臂的默认值表互斥，本脚本的结论对它无效。")
            return 3

        # ---- 授权（只读 0x2F）----
        try:
            lic = arm.license()
            print(f"授权: state={lic.state} activated={lic.activated} "
                  f"cust_id={lic.cust_id} issued={lic.issued}")
        except Exception as e:                                   # noqa: BLE001
            print(f"授权: 读不到 ({type(e).__name__}: {e})")

        # ---- 状态帧 ----
        msg = arm.get_state(refresh=True)
        st = msg.value
        if st is None:
            print("⛔ 取不到状态帧 —— 链路静默。")
            return 4
        print(f"模式: {st.mode_name} (mode={st.mode})  flags={st.flags:#x} "
              f"{st.flag_names}  seq={st.seq}")
        print(f"enabled(flags bit9)={st.enabled}   joint_fault={st.joint_fault:#x} "
              f"{st.fault_axes}")
        print(f"状态帧 hz={msg.hz:.1f}")

        # ---- 参数（逐轴一次往返）----
        jp = arm.params.all_joint_params()

        print()
        hdr = ("轴", "q", "dq", "tau", "t_mos", "t_coil", "err", "kp", "kd",
               "tau_max", "q_min", "q_max")
        print("  ".join(f"{h:>8}" for h in hdr))
        errs = []
        for i, (j, p) in enumerate(zip(st.joints, jp)):
            errs.append(j.err)
            print("  ".join([
                f"J{i + 1:<7}",
                v(j.q), v(j.dq), v(j.tau), v(j.t_mos, p=1), v(j.t_coil, p=1),
                f"{j.err:>8}",
                v(p.kp, p=2), v(p.kd, p=2), v(p.tau_max, p=2),
                v(p.q_min, p=3), v(p.q_max, p=3),
            ]))

        # ---- 逐轴 speed_limit（本仓 safety 的闸门，spec §5.1）----
        # ⚠⚠ 必须用 **kd_eff = mit_kd + kd_extra**，不是读回的 `p.kd`：
        #    `kd_extra` 住在 **0x26 向量表**（`FF_VEC_ITEMS[15]`），一次读全 7 轴。
        #    只用 `mit_kd` 会把 J4 的限速从 0.573 放到 1.260（宽 2.2 倍），
        #    而且**最紧轴会从 J4 误报成 J5** —— 错在危险一侧。
        try:
            kd_extra = [float(x) for x in arm.get_ff_vec(15).value]
            if len(kd_extra) != arm.n:
                raise ValueError(f"长度 {len(kd_extra)} != n={arm.n}")
        except Exception as e:                                   # noqa: BLE001
            print(f"⚠ 读 kd_extra 失败 ({type(e).__name__}: {e}) —— "
                  f"按固件钳幅上界 50.0 保守估算（⛔ 退回 0 会让限速变**宽**）")
            kd_extra = [50.0] * arm.n
        kd = [p.kd + e for p, e in zip(jp, kd_extra)]
        tm = [p.tau_max for p in jp]
        # ⚠ 这里**不再**推导速度上限。旧版算的是 `kd_budget·tau_max/kd`，那是为
        # `move_js`（刚度由固件定死）造的；litearm-server 用 joint_follow，
        # 速度限幅是**配置真值** `joint_follow.speed_limit`。列出来只为对照。
        spd = list(servo.DEFAULT_SPEED_LIMIT)
        print()
        print(f"mit_kd   = {[round(x, 2) for x in (p.kd for p in jp)]}")
        print(f"kd_extra = {[round(x, 2) for x in kd_extra]}")
        print(f"kd_eff   = {[round(x, 2) for x in kd]}")
        print()
        print("litearm-server 的 joint_follow.speed_limit（配置真值，非本臂推导）:")
        tight = min(range(arm.n), key=lambda i: spd[i])
        for i, s in enumerate(spd):
            mark = "  ← 最紧（决定高速跟随上限）" if i == tight else ""
            print(f"  J{i + 1}: {s:7.3f} rad/s{mark}")

        # ---- err 汇总（用户口径）----
        enabled = [i + 1 for i, e in enumerate(errs) if e == ERR_ENABLED]
        disabled = [i + 1 for i, e in enumerate(errs) if e == ERR_DISABLED]
        fault = [i + 1 for i, e in enumerate(errs) if e not in (ERR_ENABLED, ERR_DISABLED)]
        print()
        print(f"err 汇总: 使能 {enabled} / 失能 {disabled} / 故障 {fault}")

        # ---- 与基准位姿的余量（spike 的守卫，spec §11）----
        baseline = [0.0, -0.60, 0.90, -1.20, 0.0, 0.50, 0.0]
        need = [0.0, 1.0, 0.0, 0.60, 0.0, 0.0, 0.0]
        print()
        print("基准位姿余量（BASELINE_Q vs 实测软限位）:")
        for i, (need_i, p) in enumerate(zip(need, jp)):
            if need_i <= 0.0:
                continue
            hi = baseline[i] + need_i
            lo = baseline[i] - need_i
            ok = (lo >= p.q_min) and (hi <= p.q_max)
            print(f"  J{i + 1}: 需要 [{lo:.4f}, {hi:.4f}]  实测 [{p.q_min:.4f}, "
                  f"{p.q_max:.4f}]  {'✓' if ok else '⛔ 不够'}")
        cur_ok = all(p.q_min <= st.joints[i].q <= p.q_max for i, p in enumerate(jp))
        print(f"当前位姿在软限位内: {'✓' if cur_ok else '⛔ 有轴越限'}")

        print()
        print("⚠ 本次**没有**发过任何运动/使能/参数写入命令。")
        return 0
    finally:
        arm.close()          # ⛔ 忘了它进程退出会永久挂死


if __name__ == "__main__":
    raise SystemExit(main())
