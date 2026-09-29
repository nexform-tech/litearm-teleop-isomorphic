#!/usr/bin/env python3
"""从臂 `send_mit` 路线的**往返成本**实测 —— 纯只读，⛔ 不使能、不发任何运动命令。

## 为什么测 `get_gravity` 就等于测了 `send_mit_all`

两者都是**一次请求/应答**。固件在 `control_loop_step()` 里**每个 tick 只处理一条**
USB 命令（spec §2），所以"一次往返"的耗时由 **tick 周期**决定，**与命令内容无关**。
⇒ 测 `get_gravity` 等价于测 `send_mit_all` 的往返，而且**不驱动电机**。

本文件**不包含** `enable/disable/movej/move_js/move_p/park/zero_g_*/emergency_stop`
任何一条 —— 加进来之前请先想清楚：那就不再是"测量"而是"动臂"了。

## 它回答什么

server 的 `joint_follow.step()` 每拍做两件事：算一次 `G(q)`、发一次 `send_mit`。
本脚本给出两个"一次"各自多贵，从而定下结构：

    t = 一次往返
    每拍 2 次往返        ⇒ 上限 1/(2t)
    G 每 N 拍算一次      ⇒ 上限 1/((1+1/N)·t)，G 滞后 ≤ N·t

判据：
  · t ≈ 3.33 ms ⇒ 两次 6.66 ms ⇒ **150 Hz** ⇒ **必须给 G 降频**才够 250 Hz
  · t ≲ 2.0 ms  ⇒ 两次 4.00 ms ⇒ **250 Hz 达标** ⇒ 不必降频（结构更简单）

⚠ 收尾**必须显式 `close()`** —— 忘了它进程退出会永久挂死（见 memory）。
"""
from __future__ import annotations

import argparse
import os
import statistics
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

#: ⚠⚠ SDK 的**唯一入口**（用户裁决）。本机 `sys.path` 上还挂着另一份 `litearm`
#: （`/home/llx/gitee/litearm-python/src`）—— 它会被**静默** import 到。
#: 判据不是"我设过 PYTHONPATH"，而是**导入后打印出来核**。
SDK_SRC = "/home/llx/litearm-python/src"
sys.path.insert(0, SDK_SRC)

import litearm as pa                                            # noqa: E402


def bench(label: str, fn, n: int, warmup: int = 10) -> float:
    """跑 n 次取中位/p90/最大。返回中位数（ms）。"""
    for _ in range(warmup):                                     # 预热：躲开懒初始化
        fn()
    ts = []
    for _ in range(n):
        t0 = time.perf_counter()
        fn()
        ts.append((time.perf_counter() - t0) * 1e3)
    ts.sort()
    med = statistics.median(ts)
    print(f"  {label:26s} 中位 {med:6.3f} ms   p90 {ts[int(n * 0.9)]:6.3f}   "
          f"最大 {ts[-1]:6.3f}   ⇒ {1000.0 / med:6.1f} Hz")
    return med


def main() -> int:
    ap = argparse.ArgumentParser(description="只读往返测量（不动臂）")
    ap.add_argument("--port", required=True, help="CDC 口（两条臂时必须显式给）")
    ap.add_argument("-n", type=int, default=200, help="每项采样次数")
    a = ap.parse_args()

    print(f"SDK: {pa.__file__}")
    if not pa.__file__.startswith(SDK_SRC):
        print(f"⛔ 导入到的不是 {SDK_SRC} —— 环境里有另一份 litearm 抢先了。")
        return 5

    arm = pa.Arm(port=a.port, move_timeout=3.0)
    try:
        arm.connect()
        st = arm.get_state(refresh=True).value
        q0 = [float(x) for x in st.q]
        print(f"端口: {a.port}   n={getattr(arm, 'n', '?')}")
        print(f"模式: {st.mode_name}   enabled={st.enabled}   faulted={st.faulted}")
        print(f"err : {[j.err for j in st.joints]}")
        print(f"q   : {[round(x, 3) for x in q0]}")
        print("=" * 78)
        print(f"往返耗时（n={a.n}，纯查询）:")

        bench("get_state(refresh=False)", lambda: arm.get_state(refresh=False), a.n)
        bench("get_state(refresh=True) ", lambda: arm.get_state(refresh=True), a.n)
        t = bench("model.get_gravity      ", lambda: arm.model.get_gravity(q0), a.n)
        bench("params.all_joint_params", lambda: arm.params.all_joint_params(), a.n)

        print("=" * 78)
        two = 2.0 * t
        print(f"t(一次往返) = {t:.3f} ms")
        print(f"每拍 2 次往返        ⇒ {two:6.3f} ms/拍 ⇒ {1000.0 / two:6.1f} Hz")
        for n_div in (1, 2, 4, 5, 8):
            per = (1.0 + 1.0 / n_div) * t
            print(f"  G 每 {n_div} 拍算一次  ⇒ {per:6.3f} ms/拍 ⇒ {1000.0 / per:6.1f} Hz"
                  f"   (G 滞后 ≤ {n_div * t:.1f} ms)")
        return 0
    finally:
        # ⚠ 必须写 close()：否则进程退出永久挂死（见 memory）。
        arm.close()


if __name__ == "__main__":
    raise SystemExit(main())
