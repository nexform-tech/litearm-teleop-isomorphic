# 快速上手

面向首次部署的操作者或集成人员：在一对机器上完成 LiteArm 同构遥操作控制台的第一次运行。

本指南假设每台机器连接一条 LiteArm，夹爪（LiteGrip）为可选，且同样每台一个。请在对应机器上执行每一组命令。

## 1. 检查环境

- 两条 LiteArm 机械臂，固件 1.5.0 或更高。
- 两台带桌面环境的 Linux 主机。已验证配置为 Ubuntu 22.04。
- 两台主机均为 Python 3.10 或更高。
- 两台主机网络互通，TCP 17447（机械臂）可访问；使用夹爪时还需要 17448。
- 夹爪遥操：每台主机一个 LiteGrip 夹爪和一个 SocketCAN 适配器。
- 机械臂使能前，工作空间要清空，并准备好托住机械臂的方式。

## 2. 安装软件

两台机器都执行。

```bash
mkdir -p ~/litearm && cd ~/litearm
git clone https://github.com/nexform-tech/litearm-python.git
git clone https://github.com/nexform-tech/litegrip-python.git
git clone https://github.com/nexform-tech/litearm-teleop-isomorphic.git

cd litearm-teleop-isomorphic
python3 -m pip install "pyserial>=3.4" "eclipse-zenoh>=1.6" "PyQt5>=5.15"
```

用绝对路径把控制台指向 SDK 检出：

```bash
export LITEARM_SRC="$HOME/litearm/litearm-python/src"
export LITEGRIP_SRC="$HOME/litearm/litegrip-python/src"
```

要跨终端保留，把同样两行写进 `~/.bashrc` 后重新打开终端。控制台会核对 `litearm` 和 `litegrip` 的实际导入位置；如果被其他副本抢先，就会拒绝启动。

机械臂 USB CDC 串口权限：

```bash
sudo usermod -aG dialout "$USER"
```

重新登录后生效。检查依赖：

```bash
python3 -c "import serial, zenoh, PyQt5; print('dependencies OK')"
```

## 3. 查找机械臂串口

只连接一条臂时，列出稳定的设备路径：

```bash
ls -l /dev/serial/by-id/
```

如果看不出是哪一条，打印 USB 详情，查找 VID:PID `1d50:606f`：

```bash
python3 -m serial.tools.list_ports -v
```

把 `/dev/serial/by-id/` 下的路径用于 `--cdc`。不要使用 `/dev/ttyACM0` 或 `/dev/ttyACM1`；连接两条同型号臂时编号可能互换。

在每台机器上把路径存进 shell 变量，供下面的命令使用：

```bash
export LEADER_PORT="$(ls /dev/serial/by-id/* | head -n 1)"    # 主臂机器
export FOLLOWER_PORT="$(ls /dev/serial/by-id/* | head -n 1)"  # 从臂机器
```

如果主机上连接了多条臂，请把变量改成具体路径，不要取第一条。

## 4. 拉起夹爪 CAN 总线（可选）

不使用夹爪时跳过本步。在每台接夹爪的机器上执行：

```bash
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

每次重启后都要重新执行。适配器使用其他接口名时，把该名字用于 `--gcan`。

## 5. 准备网络

查看主臂机器的地址：

```bash
ip addr show
```

在从臂机器上确认能访问主臂：

```bash
ping -c 3 192.168.31.10
```

把 `192.168.31.10` 换成主臂的实际地址。两端必须使用相同的 arm ID 和端口；默认值为 `armA`、17447，夹爪为 `gripA`、17448。如果启用了防火墙，放行这些 TCP 端口。

## 6. 启动主臂控制台

在主臂机器上：

```bash
cd ~/litearm/litearm-teleop-isomorphic
export LITEARM_SRC="$HOME/litearm/litearm-python/src"
export LITEGRIP_SRC="$HOME/litearm/litegrip-python/src"

# 只跑机械臂
python3 -m liteteleop --role master --cdc "$LEADER_PORT"

# 机械臂 + 夹爪
python3 -m liteteleop --role master --cdc "$LEADER_PORT" --gcan can0
```

## 7. 启动从臂控制台

在从臂机器上：

```bash
cd ~/litearm/litearm-teleop-isomorphic
export LITEARM_SRC="$HOME/litearm/litearm-python/src"
export LITEGRIP_SRC="$HOME/litearm/litegrip-python/src"

# 只跑机械臂
python3 -m liteteleop --role slave --cdc "$FOLLOWER_PORT" --peer 192.168.31.10

# 机械臂 + 夹爪
python3 -m liteteleop --role slave --cdc "$FOLLOWER_PORT" --peer 192.168.31.10 \
                      --gcan can0 --gpeer 192.168.31.10
```

把 `192.168.31.10` 换成主臂的实际地址。

## 8. 启动机械臂遥操

两台控制台都按下面操作：

1. 确认 CDC 口是目标机械臂，点击“连接臂”。
2. 主臂侧选择“主臂”；从臂侧选择“从臂”，并填写主臂 IP。
3. 确认两端显示 arm ID `armA`、端口 `17447`。
4. 清空工作空间，确认急停可及，勾选安全确认框。
5. 点击“启动遥操”。

从臂会等待主臂首帧，先以低速 `movej` 对齐到主臂位置，然后开始跟随。对齐完成前请让从臂工作空间保持清空。首次运行时缓慢移动主臂。

如果从臂提示 5 秒内未对齐，请检查网络和 arm ID，然后停止并重新启动遥操。

## 9. 启动夹爪遥操（可选）

两台控制台都按下面操作：

1. 确认 CAN 通道正确。
2. 确认两端的夹爪 ID 和端口一致（默认 `gripA`、`17448`）。
3. 点击“启动夹爪遥操”。

从端报告就绪后，主端夹爪会变成松软状态，可以用手直接开合；从端按开口比例跟随。从端力矩上限默认为 1.0 Nm，请按所夹打印件的强度设置；设为 `0` 表示关闭该保护。

控制台会在使能电机之前拒绝未标定的夹爪。如果启动被拒绝，请按 [litegrip-python](https://github.com/nexform-tech/litegrip-python) 的说明加载或执行标定。

## 10. 安全停止

- 只想停止跟随、同时让机械臂保持使能并托住：点击“停止遥操”。控制台会受控 `movej` 回当前位置。
- 要结束本次会话：点击“断开”。机械臂保持使能并停在当前位置。
- 红色“急停”按钮会让所有轴失能，机械臂在重力下掉落。按下之前先托住机械臂。
- 断电或让机械臂无人看管之前，先托住机械臂。

## 11. 故障排查

| 现象 | 检查项 |
| --- | --- |
| `找不到 STM32 CDC 口` | USB 线、`dialout` 组、VID:PID `1d50:606f` |
| 检测到多条臂，或连错了臂 | 用 `--cdc` 指定 `/dev/serial/by-id/` 下的准确路径 |
| 从臂不动 | 主臂 IP、TCP 17447、两端 arm ID 与端口一致、主臂已启动遥操 |
| `litearm 导入自 ... 不是 ...` | 把 `LITEARM_SRC` 设为目标检出 `src` 目录的绝对路径 |
| `夹爪 SDK 不可用` | 设置 `LITEGRIP_SRC`，克隆 `litegrip-python`，安装 `eclipse-zenoh` |
| 夹爪启动被拒绝 | 夹爪未标定，或行程为零 |
| 控制台窗口打不开 | 需要桌面会话和 PyQt5 |
| `Address already in use` | 上一个控制台进程还在收尾；停掉后重试 |
| 进程被杀后机械臂下垂 | 被杀的进程无法托住机械臂；请使用“停止遥操”或“断开”，并先托住机械臂 |

这些检查背后的原因见 [DEVELOPMENT.md](DEVELOPMENT.md)。本指南的英文版为 [QUICKSTART.md](QUICKSTART.md)。
