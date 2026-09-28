# litearm-teleop-isomorphic

Isomorphic (leader-follower) teleoperation for the **LiteArm robotic manipulator
series**.

> **Status:** stage 1 on `feat/teleop-stage1-protocol`, plus the PyQt5 GUI and
> **gripper teleoperation** on `feat/gripper-teleop`.
> Covered: wire protocol codec, zenoh point-to-point link, the pure-logic safety
> layer, the firmware `joint_follow` (0x08) servo loop, and the gripper link that
> runs **fully decoupled** from the arm's
> (`liteteleop/{wire,link,safety,servo,arm_worker,grip_wire,grip_worker}.py`).
> ⚠ The gripper link's **real-machine validation is not done yet** — see
> `docs/superpowers/plans/2026-09-28-gripper-teleop.md` Task 8.

## Usage

Two processes, one per machine. The arm link and the gripper link are
**independent**: separate CAN buses, separate zenoh sessions, separate ports
(`--jport`, default 17447 for the arm; `--gport`, default 17448 for the gripper),
separate topics, separate toggles in the UI.

```bash
# master machine — the arm is hand-dragged in zero gravity; the gripper likewise
python -m liteteleop --role master --cdc /dev/ttyACM0 --gcan can0

# follower machine — subscribes and follows
python -m liteteleop --role slave  --cdc /dev/ttyACM1 --peer 192.168.31.10 \
                     --gcan can1 --gpeer 192.168.31.10
```

The gripper is published on `litearm/v4/{grip_id}/gripper_teleop` as a 32-byte
big-endian frame `(openness, position_mm, force_n, timestamp)`; `openness ∈ [0,1]`
is the payload that actually drives the follower. **Omit `--gcan` to leave gripper
teleoperation off — that is the default.**

⚠ Bring the CAN bus up first, and note the gripper needs its own bus
(`can0`/`can1`), not the arm's CDC port:

```bash
sudo ip link set can0 type can bitrate 1000000 && sudo ip link set can0 up
```

## Scope

| | |
| --- | --- |
| Product | LiteArm robotic manipulator series |
| Repository role | Isomorphic leader-follower teleoperation stack |
| Status | Arm + gripper teleoperation implemented; arm real-machine validated, gripper pending |

## Related repositories

| Repository | Role |
| --- | --- |
| [litearm-teleop-vr](https://github.com/nexform-tech/litearm-teleop-vr) | VR teleoperation |
| [litearm-ros2](https://github.com/nexform-tech/litearm-ros2) | ROS 2 driver |
| [litearm-ros1](https://github.com/nexform-tech/litearm-ros1) | ROS 1 driver |
| [litearm-python](https://github.com/nexform-tech/litearm-python) | Python SDK |
| [litearm-docs](https://github.com/nexform-tech/litearm-docs) | Product documentation |

## Repository standards

This repository follows the shared NEXFORM ROBOTICS repository standards: the
agent operating rules in [AGENTS.md](AGENTS.md), Conventional Commits, and
automated semantic-release versioning on every merge to `main`.

All changes land through a pull request; direct pushes to `main` are blocked by
branch protection.

## License

Copyright © 2026 NEXFORM ROBOTICS. Licensed under the
[Apache License 2.0](LICENSE).

## Known traps

> Full version: `docs/superpowers/specs/2026-09-28-isomorphic-teleop-design.md` §10.
> The table below is copied **verbatim** from there — when the two disagree, the spec wins.
> (Entries are kept in the original Chinese to avoid two divergent wordings of the same fact.)

| # | 陷阱 | 等级 |
| --- | --- | --- |
| 1 | `move_js` 的 `dq`：**`\|dq\|` 是 `q_ref` 走位速率上限，`dq=0` 冻结该轴**；符号被 `fabsf` 丢弃。SDK `DEVELOPER_GUIDE.md:259` 只说了"不是 limit"，漏了这半句 | `[源码]` |
| 2 | `dq` 是**双角色**：`\|dq\|` 定走位速率，`dq` **本身**还进电机速度前馈（`τ` 里的 `kd·(dq_ref − dq)`）。⇒ 必须用**参考生成器的输出速度**（本仓照抄 server 的 `slew_target` ⇒ 用它的 `dq_cmd`）。**不许拿主臂 `dq` 顶上**（那不是路径速度，且主臂一停就归零），**也不许填常数**（那是编造前馈）。另：被限位钳住的轴 `dq` 置 0 | `[实测]`（`e2e_movejs_real.py` 负控/正控）+ `[源码]` |
| 2b | 上一条的**实证只覆盖到 0.5 rad/s**（脚本写死上限，实际峰值 0.032 rad/s）。**遥操跟随的 1~2 rad/s 量级无人验过** —— 这是 §11 的 S3。⚠ 本仓的前馈系数是 server 的 **11 倍**（`kd` 11.0 vs `B` 1.0），**server 的运行经验在这个量级上不能直接外推** | `[实测]` |
| 3 | watchdog fail-soft = **0.6× 刚度 + `τ=0` ⇒ 缓慢下垂**（既非自由落体，也非稳住） | `[源码]` |
| 4 | 只有 `movej` 到位后的 `ht_on` 态才真稳（2× 刚度 + 重力前馈）⇒ **收尾必须 `movej(q_now)`** | `[源码]` |
| 5 | `park()` 只把 fail-soft 刚度**拉回** 1.0×（阻止 0.6× 那次降刚度），**`tau` 仍为 0 ⇒ 仍会垂 `G/mit_kp`**。不是 `request_stop` 的等价物 | `[源码]` |
| 6 | **急停 = EMERGENCY = 全失能 ⇒ 自由落体**，与"停止遥操"是两件事 | `[源码]` |
| 7 | `set_speed()` 经 `gov_ratio` **整体缩小走位速率** ⇒ 本工具绝不调用 | `[源码]` |
| 8 | `litearm-python` **没有** `joint_follow`/`request_stop`/`zero_gravity(on_sample=)` ⇒ **别照抄 litearm-server 的遥操实现** | `[源码]` |
| 9 | Zenoh session **不 `close()` ⇒ 进程退出永久挂死** | `[实测]` |
| 10 | STM32 CDC 在 Linux **不独占** ⇒ 一个口只允许一个进程 | 仓外实测 |
| 11 | **1J 台架 vs 7J 整臂是两张互斥的默认值表**：台架 `speed_limit=3.5`、`hold_kp_gain` **未设 ⇒ 0** ⇒ 台架板上 `ht_on` 会下发 `kp=0`（`movej` 收尾反而失力）。**任何涉及 `speed_limit`/`hold_kp_gain`/`can_dyn` 的结论必须写明板卡** | `[源码]` |
| 12 | 本项目引固件/SDK 事实**一律按符号定位、不按行号** —— 实测行号会漂（`clampf` 716 vs 722、`watchdog_kick` 725 vs 733、`ARCHITECTURE` 262 vs 261），且条件编译表会让人读错表（我自己就把台架表的 3.5 当成 J1 的） | `[实测]` |
| 13 | 本仓 key 与 server 的 topic **现在刻意共用**（帧格式已对齐）。⚠ 但**跨版本仍不许混用**：格式一旦分叉，共用 key 会让两端静默解错 | 设计约定 |
| 14 | 夹爪 SDK 在本机有**两份同名的仓**：`/home/llx/litegrip-python`（github，本仓的目标）与 `/home/llx/moduangongju/lite-grip`（gitee，被 editable 装成 `litegrip` 2.2.0）。**裸 `import litegrip` 落到后者**（实测）⇒ 必须 `grip_worker.pin_grip_sdk()` 断言 + 打印日志，只 `sys.path.insert` 压不住未来改成 `meta_path.insert` 的注册方式 | `[实测]` |
| 15 | `LiteGrip.close()` 是**合爪**，不是断开 —— 断开只有 `disconnect()` | `[源码]` |
| 16 | `LiteGrip.__init__` 的 `disable_on_disconnect` **默认 True** ⇒ `disconnect()` 会先失能、当场松掉正夹着的物体。要「收尾仍夹着」必须**显式**传 `False` | `[源码]` |
| 17 | 夹爪 `open()`/`close()` 的 `ok=True` 语义是**顶到机械限位堵转**（`stalled=True, reached=False`，与直觉相反）；且未标定时 `send_mit_frame`/`goto_rad` **不报错**，会拿占位默认限位当真实限位用 | `[源码]` |
