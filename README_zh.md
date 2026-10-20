# litearm-teleop-isomorphic（中文版）

面向客户的 LiteArm 同构主从遥操作控制台使用说明：操作者用手拖动主臂，从臂实时跟随。

[English](README.md) | [快速上手](QUICKSTART_zh.md) | [开发说明](DEVELOPMENT.md)

## 概述

本仓库是 LiteArm 机械臂系列的同构遥操作桌面控制台。主臂运行在零重力模式，操作者可以直接用手拖动。主臂的关节状态通过 zenoh 点对点链路发布，从臂由固件侧的 `joint_follow` 伺服环跟踪。

机械臂遥操作与夹爪遥操作相互独立：各自使用独立的 CAN 总线、zenoh 会话、端口、话题和界面控件。夹爪不是必须的，可以只运行机械臂。

控制台是基于 PyQt5 的 Linux 桌面程序，界面当前为中文。

## 环境要求

| 项目 | 要求 |
| --- | --- |
| 主机 | 带桌面环境的 Linux（已在 Ubuntu 22.04 验证），Python 3.10 或更高 |
| 机械臂 | 两条 LiteArm 机械臂，固件 1.5.0 或更高 |
| 串口权限 | 每条臂通过 USB CDC 连接；用户账号需加入 `dialout` 组 |
| 网络 | 主从机之间可访问 TCP 17447（机械臂）；夹爪使用 17448 |
| 夹爪（可选） | LiteGrip 夹爪，接在 1 Mbit/s 的 SocketCAN 接口上 |
| SDK | `litearm-python`；使用夹爪时还需要 `litegrip-python` |

## 安装

两台机器都要安装 SDK 检出和控制台依赖。下面的步骤假设检出目录位于 `~/litearm`，也可以换成任意绝对路径。

```bash
mkdir -p ~/litearm && cd ~/litearm
git clone https://github.com/nexform-tech/litearm-python.git
git clone https://github.com/nexform-tech/litegrip-python.git
git clone https://github.com/nexform-tech/litearm-teleop-isomorphic.git

cd litearm-teleop-isomorphic
python3 -m pip install "pyserial>=3.4" "eclipse-zenoh>=1.6" "PyQt5>=5.15"

export LITEARM_SRC="$HOME/litearm/litearm-python/src"
export LITEGRIP_SRC="$HOME/litearm/litegrip-python/src"
```

`LITEARM_SRC` 和 `LITEGRIP_SRC` 必须是指向对应检出 `src` 目录的绝对路径。把两条 `export` 写进 `~/.bashrc` 可以持久生效。不要另外安装一份同名 SDK 去遮蔽这里指定的检出；控制台会核对实际导入位置，发现被其他副本抢先就会拒绝启动。

Linux 还需要串口权限：

```bash
sudo usermod -aG dialout "$USER"
```

重新登录后组权限才会生效。

完整的首次运行步骤（包括串口查找和控制台操作）见 [QUICKSTART_zh.md](QUICKSTART_zh.md)。

## 运行

一台机器跑一个进程，并在仓库根目录下执行。

```bash
# 主臂侧：机械臂进入零重力模式，由人手拖动
python3 -m liteteleop --role master --cdc /dev/serial/by-id/<主臂串口>

# 从臂侧：订阅主臂并跟随
python3 -m liteteleop --role slave --cdc /dev/serial/by-id/<从臂串口> --peer <主臂IP>
```

要启用夹爪遥操就加上夹爪参数；不传 `--gcan` 时夹爪链路默认关闭。

```bash
# 主臂侧并启用夹爪
python3 -m liteteleop --role master --cdc /dev/serial/by-id/<主臂串口> --gcan can0

# 从臂侧并启用夹爪
python3 -m liteteleop --role slave --cdc /dev/serial/by-id/<从臂串口> --peer <主臂IP> \
                      --gcan can0 --gpeer <主臂IP>
```

默认值为 `arm_id=armA`、端口 17447，夹爪为 `grip_id=gripA`、端口 17448。两端必须使用相同的 ID 和端口；需要修改时用 `--arm-id`、`--jport`、`--grip-id`、`--gport` 在两端同时改。

请使用稳定的 `/dev/serial/by-id/` 路径，不要依赖 `/dev/ttyACM0`、`/dev/ttyACM1`。连接两条同型号臂时，`ttyACM` 编号可能互换。检测到多条臂时必须显式传 `--cdc`，控制台不会自动挑一个。

## 控制台操作

1. 选择 CDC 串口，点击“连接臂”。
2. 主臂侧选择“主臂”；从臂侧选择“从臂”，并填写主臂 IP。
3. 确认两端显示的 arm ID 和端口一致。
4. 清空工作空间并确认急停可及后，勾选安全确认框。
5. 点击“启动遥操”。

从臂会等待主臂首帧，先以低速 `movej` 对齐到主臂位置，然后开始跟随。对齐完成前请让从臂工作空间保持清空。首次运行时请缓慢移动主臂。

角色、arm ID、对端地址和端口都在点击“启动遥操”时读取，而不是连接时读取。连接后可以修改，但新值要停止并重新启动遥操后才生效。

## 夹爪遥操作

夹爪遥操使用独立的 CAN 总线和独立的 zenoh 会话。两台机器都需要把 LiteGrip 夹爪接到各自的 SocketCAN 接口上。

启动控制台前先把总线拉到 1 Mbit/s，每次重启后也要重新执行：

```bash
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

从端报告就绪后，主端夹爪会变成松软状态，可以用手直接开合；从端按开口比例跟随。从端力矩上限默认为 1.0 Nm，可在界面修改；设为 `0` 表示关闭该保护。该值应按所夹打印件的强度来定，而不是按夹爪型号定。

控制台使用 `litegrip` SDK 自带的出厂标定文件。未标定的夹爪会在电机使能之前被拒绝。如果需要标定某一台夹爪，请按 [litegrip-python](https://github.com/nexform-tech/litegrip-python) 的标定章节操作。

夹爪参数在点击“启动夹爪遥操”时读取，而不是建立连接时读取。修改后请停止并重新启动会话。

## 安全须知

- 不要把“停止遥操”和“急停”当成同一个动作。停止遥操会受控 `movej` 回当前位置，机械臂保持使能并托住；急停会让所有轴失能，机械臂在重力下掉落。
- 按急停、断电或让失能状态的机械臂无人看管之前，先托住机械臂。
- 机械臂使能期间保持工作空间清空，并让急停按钮随手可及。
- 不要用手拖动从臂。只有主臂运行在零重力模式。
- 不要让两个控制台进程指向同一个 CDC 口。STM32 CDC 在 Linux 上不独占，第二个进程会静默共用同一份字节流，而不是提示串口被占用。
- 链路断开时，从臂会停止跟随并保持最后目标。重新启动遥操前先查看控制台日志。
- 不要在主从两端混用不同固件或 SDK 版本。话题名是刻意共用的，格式一旦分叉，两端会静默解出错误的值。

## 文档

| 文档 | 内容 |
| --- | --- |
| [QUICKSTART.md](QUICKSTART.md) | 英文快速上手 |
| [QUICKSTART_zh.md](QUICKSTART_zh.md) | 中文快速上手 |
| [README.md](README.md) | 英文 README（权威版本） |
| [DEVELOPMENT.md](DEVELOPMENT.md) | 开发环境与已知陷阱（英文） |
| [AGENTS.md](AGENTS.md) | 仓库自动化规则 |

## 相关仓库

| 仓库 | 作用 |
| --- | --- |
| [litearm-teleop-vr](https://github.com/nexform-tech/litearm-teleop-vr) | VR 遥操作 |
| [litearm-ros2](https://github.com/nexform-tech/litearm-ros2) | ROS 2 驱动 |
| [litearm-ros1](https://github.com/nexform-tech/litearm-ros1) | ROS 1 驱动 |
| [litearm-python](https://github.com/nexform-tech/litearm-python) | Python SDK |
| [litegrip-python](https://github.com/nexform-tech/litegrip-python) | 夹爪 SDK；夹爪遥操的链路与伺服环归它 |
| [litearm-docs](https://github.com/nexform-tech/litearm-docs) | 产品文档 |

## 许可证

Copyright © 2026 NEXFORM ROBOTICS。以 [Apache License 2.0](LICENSE) 授权。
