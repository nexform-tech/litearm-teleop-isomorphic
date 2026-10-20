# Quickstart

A step-by-step first run of the LiteArm isomorphic teleoperation console on two machines, for an operator or integrator setting up the system for the first time.

This guide assumes one LiteArm on each machine and an optional LiteGrip gripper on each machine. Run every command on the machine it is listed under.

## 1. Check the requirements

- Two LiteArm arms, firmware 1.5.0 or later.
- Two Linux hosts with a desktop session. Ubuntu 22.04 is the tested configuration.
- Python 3.10 or later on both hosts.
- A network connection between the hosts, with TCP ports 17447 (arm) and optionally 17448 (gripper) reachable.
- For gripper teleoperation: one LiteGrip gripper and one SocketCAN adapter per host.
- A clear workspace and a way to support each arm before it is enabled.

## 2. Install the software

Run this on both machines.

```bash
mkdir -p ~/litearm && cd ~/litearm
git clone https://github.com/nexform-tech/litearm-python.git
git clone https://github.com/nexform-tech/litegrip-python.git
git clone https://github.com/nexform-tech/litearm-teleop-isomorphic.git

cd litearm-teleop-isomorphic
python3 -m pip install "pyserial>=3.4" "eclipse-zenoh>=1.6" "PyQt5>=5.15"
```

Point the console at the SDK checkouts with absolute paths:

```bash
export LITEARM_SRC="$HOME/litearm/litearm-python/src"
export LITEGRIP_SRC="$HOME/litearm/litegrip-python/src"
```

To keep them across shells, add the same two lines to `~/.bashrc` and start a new shell. The console checks where `litearm` and `litegrip` were imported from and refuses to start if another copy wins.

Serial permissions for the arm's USB CDC port:

```bash
sudo usermod -aG dialout "$USER"
```

Log out and back in. Check the dependencies:

```bash
python3 -c "import serial, zenoh, PyQt5; print('dependencies OK')"
```

## 3. Find the arm ports

With exactly one arm connected, list its stable device path:

```bash
ls -l /dev/serial/by-id/
```

If the entry is not obvious, print the USB details and look for vendor/product ID `1d50:606f`:

```bash
python3 -m serial.tools.list_ports -v
```

Use the `/dev/serial/by-id/` path for `--cdc`. Do not use `/dev/ttyACM0` or `/dev/ttyACM1`; those numbers can swap when two arms of the same model are connected.

On each host, save the path in a shell variable for the commands below:

```bash
export LEADER_PORT="$(ls /dev/serial/by-id/* | head -n 1)"    # on the leader machine
export FOLLOWER_PORT="$(ls /dev/serial/by-id/* | head -n 1)"  # on the follower machine
```

If more than one arm is connected to the host, replace the variable with the specific path instead of taking the first entry.

## 4. Bring up the gripper CAN bus (optional)

Skip this step if you are not using the gripper. Run it on each machine that has one:

```bash
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

Run it again after every reboot. If the adapter uses another interface name, use that name for `--gcan`.

## 5. Prepare the network

Find the leader's address:

```bash
ip addr show
```

From the follower, confirm the leader is reachable:

```bash
ping -c 3 192.168.31.10
```

Replace `192.168.31.10` with the leader's actual address. Both machines must use the
same arm ID and ports; the defaults are `armA` on 17447 and `gripA` on 17448. Open
those TCP ports if a firewall is active.

## 6. Start the leader console

On the leader machine:

```bash
cd ~/litearm/litearm-teleop-isomorphic
export LITEARM_SRC="$HOME/litearm/litearm-python/src"
export LITEGRIP_SRC="$HOME/litearm/litegrip-python/src"

# Arm only
python3 -m liteteleop --role master --cdc "$LEADER_PORT"

# Arm and gripper
python3 -m liteteleop --role master --cdc "$LEADER_PORT" --gcan can0
```

## 7. Start the follower console

On the follower machine:

```bash
cd ~/litearm/litearm-teleop-isomorphic
export LITEARM_SRC="$HOME/litearm/litearm-python/src"
export LITEGRIP_SRC="$HOME/litearm/litegrip-python/src"

# Arm only
python3 -m liteteleop --role slave --cdc "$FOLLOWER_PORT" --peer 192.168.31.10

# Arm and gripper
python3 -m liteteleop --role slave --cdc "$FOLLOWER_PORT" --peer 192.168.31.10 \
                      --gcan can0 --gpeer 192.168.31.10
```

Replace `192.168.31.10` with the leader's address.

## 8. Start arm teleoperation

On both consoles:

1. Confirm the CDC port is the intended arm and click 连接臂 (Connect arm).
2. On the leader, select 主臂 (Leader). On the follower, select 从臂 (Follower) and enter the leader's IP.
3. Confirm both consoles show arm ID `armA` and port `17447`.
4. Clear the workspace, check that the emergency stop is reachable, and tick the confirmation checkbox.
5. Click 启动遥操 (Start teleoperation).

The follower waits for the first frame, aligns to the leader with a low-speed
`movej`, and then follows. Keep the follower's workspace clear until alignment
finishes. Move the leader slowly for the first run.

If the follower reports that it did not align within 5 seconds, check the network and the arm ID, then stop and restart teleoperation.

## 9. Start gripper teleoperation (optional)

On both consoles:

1. Confirm the CAN channel is correct.
2. Confirm the gripper ID and port match on both ends (`gripA`, `17448` by default).
3. Click 启动夹爪遥操 (Start gripper teleoperation).

The master gripper goes slack once the follower reports ready, so move it by hand.
The follower mirrors the opening. The follower's torque limit is 1.0 Nm by default;
set a value that suits the printed part being gripped, or `0` to disable the guard.

The console refuses an uncalibrated gripper before energizing it. If the start is refused, load or perform a calibration as described in [litegrip-python](https://github.com/nexform-tech/litegrip-python).

## 10. Stop safely

- To stop following while keeping the arm enabled and held, click 停止遥操 (Stop teleoperation). The console performs a controlled `movej` back to the current pose.
- To close the session, click 断开 (Disconnect). The arm stays enabled and holds its position.
- The red 急停 (Emergency stop) button disables every axis and the arm falls under gravity. Support the arm before pressing it.
- Support the arm before removing power or leaving it unattended.

## 11. Troubleshooting

| Symptom | What to check |
| --- | --- |
| `找不到 STM32 CDC 口` | USB cable, the `dialout` group, and VID:PID `1d50:606f` |
| Two arms detected, or the wrong arm connects | Pass the exact `/dev/serial/by-id/` path with `--cdc` |
| The follower does not move | Leader IP, TCP port 17447, identical arm ID and port on both ends, and that the leader started teleoperation |
| `litearm 导入自 ... 不是 ...` | Set `LITEARM_SRC` to the absolute path of the intended checkout's `src` directory |
| `夹爪 SDK 不可用` | Set `LITEGRIP_SRC`, clone `litegrip-python`, and install `eclipse-zenoh` |
| Gripper start is refused | The gripper is not calibrated, or its travel is zero |
| The console window does not open | A desktop session and PyQt5 are required |
| `Address already in use` | An earlier console process is still shutting down; stop it and retry |
| The arm sags after a process is killed | A killed process cannot hold the arm; use Stop or Disconnect, and support the arm |

For the behavior behind these checks, see [DEVELOPMENT.md](DEVELOPMENT.md). The Chinese version of this guide is [QUICKSTART_zh.md](QUICKSTART_zh.md).
