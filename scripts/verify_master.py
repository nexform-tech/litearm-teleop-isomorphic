#!/usr/bin/env python3
"""**主臂**数据通路核对 —— 采样 → 编码 → zenoh 发出 → 收到 → 解码。

主臂的功能（spec §6）是：零重力拖动 → 100 Hz 采样实测值 → 发出去。
**主臂不做任何钳位**（spec §6 末段：发布的是实测值，钳位是从臂的事）。

⛔ 本脚本**不动臂**：不 `zero_g_start()`、不 `movej`、不 `move_js`、不写参数。
   它只做「读状态 + 发帧」，所以随时可跑、跑完不留痕。
   ⚠ **零重力那一段是另一回事** —— `zero_g_start()` 会让臂**变软**（重力补偿托举、
   人手可拖），没人扶着就会垂下去。那一步必须人在场、单独授权，见 §6.2。

它回答四个问题：
  1. 100 Hz 采样**实际**能到多少（`get_state(refresh=False)` 的节拍）
  2. 状态帧有没有丢（按 `seq` 差值数，不靠猜）
  3. 编码出来的信封对不对（70 B / n=7）
  4. zenoh 点对点**发出去-收得到**吗，端到端多老

用法::

    python scripts/verify_master.py                 # 默认 8 秒
    python scripts/verify_master.py --sec 20
    python scripts/verify_master.py --port /dev/ttyACM0 --jport 17450
"""
from __future__ import annotations

import argparse
import os
import statistics
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

#: ⚠⚠ SDK 的**唯一入口**（用户裁决）。本机 `sys.path` 上还挂着另一份 `litearm`
#: （gitee 克隆，停在 `chore/sync-repo-standards` 分支）会被**静默**import 到。
SDK_SRC = "/home/llx/litearm-python/src"
sys.path.insert(0, SDK_SRC)

import litearm as pa                                            # noqa: E402

assert pa.__file__.startswith(SDK_SRC), (
    f"⛔ litearm 导入自 {pa.__file__}，不是 {SDK_SRC} —— 环境里有另一份抢先了")

from liteteleop import link, wire                               # noqa: E402

SEND_HZ = 100.0
DT = 1.0 / SEND_HZ
ARM_JOINTS = 7


def main() -> int:
    ap = argparse.ArgumentParser(description="主臂数据通路核对（不动臂）")
    ap.add_argument("--port", default=None)
    ap.add_argument("--sec", type=float, default=8.0, help="采样时长")
    ap.add_argument("--jport", type=int, default=17450, help="zenoh 监听端口")
    a = ap.parse_args()

    port = a.port or pa.find_cdc_port()
    if not port:
        print("⛔ 找不到 STM32 CDC 口（VID:PID 1d50:606f）")
        return 2
    print(f"SDK : {pa.__file__}")
    print(f"端口: {port}")

    arm = pa.Arm(port=port, move_timeout=3.0)
    recv: list[bytes] = []
    try:
        arm.connect()
        st = arm.get_state(refresh=True).value
        if st is None:
            print("⛔ 取不到状态帧")
            return 4
        if st.n != ARM_JOINTS:
            print(f"⛔ 只对 {ARM_JOINTS} 关节整臂有效，当前 n={st.n}")
            return 3
        # ⛔ 不调 enable()：已使能的臂不该被重复使能；未使能的臂要人看着才使能。
        #    这里只**断言**它是使能且干净的 —— 主臂要在使能态下才能零重力拖动。
        print(f"固件: {arm.firmware!r}   mode={st.mode_name}  enabled={st.enabled}  "
              f"flags={st.flags:#06x} {st.flag_names}")
        if not st.enabled or st.faulted or st.joint_fault:
            print(f"⛔ 起始状态不干净: enabled={st.enabled} faulted={st.faulted} "
                  f"joint_fault={st.joint_fault:#x}")
            return 3

        with link.Listener(a.jport, link.DEFAULT_KEY) as pub, \
             link.Connector("127.0.0.1", a.jport, link.DEFAULT_KEY,
                            on_frame=recv.append) as sub:
            t0 = time.monotonic()
            while not pub.matching and time.monotonic() - t0 < 20.0:
                time.sleep(0.1)
            if not pub.matching:
                print("⛔ 20 s 内 zenoh 没匹配上 —— 点对点链路没建起来")
                return 5
            print(f"zenoh: 已匹配（等待 {time.monotonic() - t0:.1f}s）")

            print(f"\n采样 {a.sec:.0f} s @ {SEND_HZ:.0f} Hz ...")
            sent = 0
            ticks: list[float] = []
            seqs: list[int] = []
            none_cnt = 0
            dq_peak = 0.0
            t0 = time.monotonic()
            nxt = t0 + DT
            while time.monotonic() - t0 < a.sec:
                st = arm.get_state(refresh=False).value
                if st is None:
                    none_cnt += 1
                else:
                    seqs.append(st.seq)
                    dq_peak = max(dq_peak, max(abs(x) for x in st.dq))
                    pub.put(wire.encode(st.q, st.dq, time.time(), st.seq))
                    sent += 1
                    ticks.append(time.monotonic())
                r = nxt - time.monotonic()
                if r > 0:
                    time.sleep(r)
                nxt += DT
                if nxt < time.monotonic():
                    nxt = time.monotonic() + DT
            el = time.monotonic() - t0

            time.sleep(0.5)                       # 让 zenoh 把在途的收完
        # ---- 判据 ----
        print("\n================ 结果 ================")
        gaps = [b - x for x, b in zip(ticks, ticks[1:])]
        rate = sent / el
        print(f"  采样下发      {sent} 帧 / {el:.2f} s = {rate:.1f} Hz"
              f"   (目标 {SEND_HZ:.0f})")
        if gaps:
            print(f"  节拍抖动      p50 {statistics.median(gaps)*1e3:.2f} ms / "
                  f"p95 {sorted(gaps)[int(len(gaps)*0.95)]*1e3:.2f} ms / "
                  f"max {max(gaps)*1e3:.2f} ms")
        print(f"  状态帧取空    {none_cnt} 次")
        # 丢帧：按 seq 差值数（不靠"我以为发了多少"）
        lost = 0
        for x, b in zip(seqs, seqs[1:]):
            d = (b - x) & 0xFFFFFFFF
            if d > 1:
                lost += d - 1
        print(f"  状态帧 seq    跨度 {seqs[-1] - seqs[0] if seqs else 0}，"
              f"**丢 {lost} 帧**")
        print(f"  zenoh 收到    {len(recv)} / {sent}"
              f"{'  ✓ 无丢失' if len(recv) == sent else '  ⚠ 有丢失'}")
        if recv:
            f = wire.decode(recv[-1], expect_n=ARM_JOINTS)
            print(f"  解码回读      n={f.n} seq={f.seq} 帧长 "
                  f"{len(recv[-1])}B (frame_size={wire.frame_size(ARM_JOINTS)})")
        print(f"  拖动速度 dq   峰值 {dq_peak:.4f} rad/s"
              f"   （⚠ 人没拖时接近 0 是正常的）")
        st = arm.get_state(refresh=True).value
        print(f"  末态          mode={st.mode_name} enabled={st.enabled} "
              f"err={[j.err for j in st.joints]}")
        print("\n⚠ 全程没有发过运动/使能/零重力/参数写入命令。")
        return 0
    finally:
        arm.close()          # ⛔ 忘了它进程退出会永久挂死


if __name__ == "__main__":
    raise SystemExit(main())
