# litearm-teleop-isomorphic（中文版）

本文说明如何在两台 LiteArm 机械臂上跑同构（主从）遥操作，以及不按规矩来会踩到什么。
首次启动那两个进程之前，请先读完本文。

> 英文版是权威版本：[README.md](README.md)。两者内容不一致时，以英文版为准。

## 这是什么

LiteArm 机械臂系列的同构遥操作。人手在零重力模式下拖动主臂，从臂经 zenoh 点对点链路跟随。
夹爪也可以在同一对机器上遥操，走自己的 CAN 总线和自己的链路。

## 状态

主臂链路与夹爪链路都已实现。主臂链路已在真机验证；夹爪链路尚未验证。

离线测试覆盖：线协议编解码、zenoh 点对点链路、纯逻辑安全层、固件侧的 `joint_follow`
（`0x08`）伺服环，以及夹爪链路。

## 用法

跑两个进程，一台机器一个。臂链路与夹爪链路相互独立：各自的总线、各自的 zenoh 会话、
各自的端口（`--jport`，臂默认 17447；`--gport`，夹爪默认 17448）、各自的话题、
界面上各自的开关。

```bash
# 主臂侧：臂以零重力模式被人手拖动；夹爪同理
python -m liteteleop --role master --cdc /dev/ttyACM0 --gcan can0

# 从臂侧：订阅并跟随
python -m liteteleop --role slave  --cdc /dev/ttyACM1 --peer 192.168.31.10 \
                     --gcan can1 --gpeer 192.168.31.10
```

夹爪发布在 `litearm/v4/{grip_id}/gripper_teleop`，是一个 32 字节大端帧
`(openness, position_mm, force_n, timestamp)`。真正驱动从端的是 `openness ∈ [0,1]`。
**不传 `--gcan` 就不启用夹爪遥操**，这也是默认状态。

先把 CAN 总线拉起来。夹爪需要自己的总线（`can0` 或 `can1`），不是臂的 CDC 口：

```bash
sudo ip link set can0 type can bitrate 1000000 && sudo ip link set can0 up
```

## 适用范围

| | |
| --- | --- |
| 产品 | LiteArm 机械臂系列 |
| 本仓角色 | 同构主从遥操作栈 |
| 状态 | 臂与夹爪遥操均已实现；臂已真机验证，夹爪待验证 |

## 相关仓库

| 仓库 | 作用 |
| --- | --- |
| [litearm-teleop-vr](https://github.com/nexform-tech/litearm-teleop-vr) | VR 遥操作 |
| [litearm-ros2](https://github.com/nexform-tech/litearm-ros2) | ROS 2 驱动 |
| [litearm-ros1](https://github.com/nexform-tech/litearm-ros1) | ROS 1 驱动 |
| [litearm-python](https://github.com/nexform-tech/litearm-python) | Python SDK |
| [litearm-docs](https://github.com/nexform-tech/litearm-docs) | 产品文档 |

## 仓库规范

本仓遵循 NEXFORM ROBOTICS 的共享仓库规范：agent 操作规则见 [AGENTS.md](AGENTS.md)，
提交信息用 Conventional Commits，每次合入 `main` 由 semantic-release 自动打版本。
所有改动都走 pull request，直接推 `main` 被分支保护拦住。

## 已知陷阱

下面每一条都是实测或读源码得到的，每一条都让人花过时间。

### 运动与前馈

- **不要**把主臂实测的 `dq` 当作从臂的速度参考。在 `move_js` 里 `dq` 有两个身份。
- 它的**幅值**限制 `q_ref` 的走位速率，它的**值**又进电机速度前馈，即 `kd·(dq_ref − dq)`。
- 应当用参考生成器自己的输出。本仓的 `slew_target` 把它作为 `dq_cmd` 返回。
- **不要**拿常数顶上，那是凭空编造一个前馈力矩。
- 被限位钉住的轴会被置 `dq = 0`，等于冻结该轴。
- **不要**在这里信 `DEVELOPER_GUIDE.md`。它只说了 `dq`"不是 limit"，那是半句话。
- **不要**把 `dq` 前馈外推到没测过的量级。真机证据只覆盖到 0.5 rad/s，脚本写死了上限，实际峰值 0.032 rad/s。
- 遥操跑在 1~2 rad/s，而那个量级无人验证过。本仓的前馈系数是 server 的 11 倍（`kd` 11.0 对 `B` 1.0），server 的经验不能直接搬过来。
- **不要**调用 `set_speed()`，它经 `gov_ratio` 把走位速率整体缩小。

### 停止与持位

- **不要**把"停止遥操"和"急停"当成同一件事。急停是 `EMERGENCY`：它让所有轴失能，臂在重力下掉落。
- **不要**不收尾就结束会话。必须 `movej` 回当前位姿；只有 `movej` 到位后的 `ht_on` 态才真的托得住臂。
- **不要**以为停下来就稳了。没有 `movej`，臂会下垂，因为没有任何东西在补重力前馈。
- **不要**指望 `park()` 托住臂。它把 fail-soft 刚度拉回 1.0 倍（抵消那次 0.6 倍降刚度），但 `tau` 仍是 0，臂照样按 `G/mit_kp` 下垂。
- `park()` **不是** `request_stop` 的等价物。
- **不要**靠看门狗维持稳定持位。看门狗 fail-soft 是 0.6 倍刚度加 `tau = 0`：缓慢下垂，既不等于自由落体，也不等于稳住了。

### 硬件与环境

- **不要**让两个进程指向同一个 CDC 口。STM32 CDC 在 Linux 上不独占，第二个进程会静默分吃同一份字节流，而不是打不开口报错。
- **不要**在没写明哪块板卡的情况下引用 `speed_limit` 或 `hold_kp_gain` 的值。1J 台架和 7J 整臂有两张互斥的默认值表。
- 台架上 `speed_limit` 是 3.5，而 `hold_kp_gain` 没设，即 0。于是 `ht_on` 会下发 `kp = 0`，`movej` 交接反而掉力而不是持位。
- **不要**把不同固件或 SDK 版本混在同一条链路上，哪怕话题名是刻意共用的。格式一旦分叉，共用的话题名会让两端静默解错。

### 依赖与收尾

- **不要**让 zenoh session 不做 `close()`，进程退出会永久挂死。
- **不要**让 `litearm` 由"谁先被 import 谁赢"。要显式指到目标检出位置并断言结果，因为一个陈旧的 editable 安装会静默满足 `import litearm`。
- **不要**在有同名两份检出的机器上裸 `import litegrip`。两者都能 import 成功，而赢的是错的那份。
- 应当走 `grip_worker.pin_grip_sdk()`，它会断言并打日志。只做一个 `sys.path.insert` 挡不住将来改用 `meta_path.insert` 的注册方式。
- **不要**以为 `LiteGrip.close()` 是断开。它是合爪；断链是 `disconnect()`。
- **不要**在想让夹爪继续夹住时，让 `disable_on_disconnect` 留在默认值。它默认 `True`，所以 `disconnect()` 会先失能，把正夹着的东西松掉。要显式传 `False`。
- **不要**把夹爪 `open()` / `close()` 返回的 `ok=True` 当作成功。它的含义是夹爪顶到机械限位堵转，与直觉相反。
- **不要**指望未标定的夹爪会报错。`send_mit_frame` 和 `goto_rad` 都不抛异常，它们会拿占位限位当真实限位用。

### 读代码

- **不要**按行号引用固件或 SDK 的事实，行号会漂：`clampf` 在 716 和 722 之间移动过，`watchdog_kick` 在 725 和 733 之间。
- 条件编译表也容易让人读错表。应当按**符号**引用。

## 许可证

Copyright © 2026 NEXFORM ROBOTICS。以 [Apache License 2.0](LICENSE) 授权。
