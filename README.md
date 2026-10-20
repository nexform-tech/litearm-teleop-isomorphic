# litearm-teleop-isomorphic

A customer guide to the LiteArm isomorphic leader-follower teleoperation console: the operator hand-drags the leader arm, and the follower arm mirrors it.

[简体中文](README_zh.md) | [Quickstart](QUICKSTART.md) | [Development notes](DEVELOPMENT.md)

## Overview

This repository is the desktop console for isomorphic teleoperation on the LiteArm
robotic manipulator series. The leader arm runs in zero-gravity mode, so an operator
can move it by hand. Its joint state is published over a point-to-point zenoh link,
and the follower arm tracks it with a firmware-side `joint_follow` servo loop.

Arm teleoperation and gripper teleoperation are independent. They use separate CAN buses, zenoh sessions, ports, topics, and console controls. Running the gripper is optional.

The console is a PyQt5 application for Linux. Its user interface is currently in Chinese; the workflow below gives the English meaning of each control in parentheses.

## What you need

| Item | Requirement |
| --- | --- |
| Host | Linux with a desktop session (Ubuntu 22.04 tested), Python 3.10 or later |
| Arms | Two LiteArm arms, firmware 1.5.0 or later |
| Serial access | USB CDC connection to each arm; your user account in the `dialout` group |
| Network | Leader and follower reachable over TCP port 17447 (arm); port 17448 for the gripper |
| Gripper (optional) | LiteGrip gripper on a SocketCAN interface at 1 Mbit/s |
| SDKs | `litearm-python`; `litegrip-python` when the gripper is used |

## Install

Install the SDK checkouts and the console dependencies on both machines. The steps below assume the checkouts live under `~/litearm`; any absolute path works.

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

`LITEARM_SRC` and `LITEGRIP_SRC` must be absolute paths to the `src` directories
of the checkouts. Add the two `export` lines to `~/.bashrc` to make them persist.
Do not install a second copy of either SDK and let it shadow the checkout you point
at; the console checks where the imports landed and refuses to start if another
copy wins.

Linux needs serial permissions:

```bash
sudo usermod -aG dialout "$USER"
```

Log out and back in for the group change to apply.

The full first-run procedure, including port discovery and the console workflow, is in [QUICKSTART.md](QUICKSTART.md).

## Run

Run one process per machine, from the repository root.

```bash
# Leader: the arm is hand-dragged in zero-gravity mode.
python3 -m liteteleop --role master --cdc /dev/serial/by-id/<leader-port>

# Follower: subscribes to the leader and follows.
python3 -m liteteleop --role slave --cdc /dev/serial/by-id/<follower-port> --peer <leader-ip>
```

Add the gripper options to enable gripper teleoperation; without `--gcan` the gripper link stays off.

```bash
# Leader with gripper
python3 -m liteteleop --role master --cdc /dev/serial/by-id/<leader-port> --gcan can0

# Follower with gripper
python3 -m liteteleop --role slave --cdc /dev/serial/by-id/<follower-port> --peer <leader-ip> \
                      --gcan can0 --gpeer <leader-ip>
```

Defaults are `arm_id=armA` on port 17447 and `grip_id=gripA` on port 17448. The two ends must use the same IDs and ports. Use `--arm-id`, `--jport`, `--grip-id`, and `--gport` to change them on both ends.

Use the stable `/dev/serial/by-id/` path rather than `/dev/ttyACM0` or
`/dev/ttyACM1`. The `ttyACM` numbers can swap when two arms of the same model are
connected. When more than one arm is present, always pass `--cdc`; the console
refuses to pick one automatically.

## Console workflow

1. Select the CDC port and click 连接臂 (Connect arm).
2. On the leader, select 主臂 (Leader). On the follower, select 从臂 (Follower) and enter the leader's IP address.
3. Confirm that both consoles show the same arm ID and port.
4. Tick the confirmation checkbox after clearing the workspace and checking the emergency stop.
5. Click 启动遥操 (Start teleoperation).

The follower waits for the first frame, aligns to the leader with a low-speed
`movej`, then follows. Keep the follower's workspace clear until alignment finishes.
Move the leader slowly for the first run.

The role, arm ID, peer address, and port are read when you start teleoperation,
not when you connect. You can change them while connected, but the new values apply
only after you stop and start teleoperation again.

## Gripper teleoperation

Gripper teleoperation runs on its own CAN bus and its own zenoh session. It needs a LiteGrip gripper connected to a SocketCAN interface on each machine.

Bring up the bus at 1 Mbit/s before starting the console, and after every reboot:

```bash
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

The master gripper goes slack once the follower reports ready, so it can be moved by
hand. The follower mirrors the opening. The follower's torque limit defaults to
1.0 Nm and can be changed in the console; `0` disables the guard. Set it for the
printed part you are gripping, not for the gripper model.

The console loads the factory calibration shipped with the `litegrip` SDK. A gripper
that is not calibrated is refused before the motors are enabled. If you need to
calibrate a unit, follow the calibration section of
[litegrip-python](https://github.com/nexform-tech/litegrip-python).

The gripper parameters are read when you click 启动夹爪遥操 (Start gripper teleoperation), not when the connection is made. Stop and start the session to apply changes.

## Safety

- Do not treat 停止遥操 (Stop teleoperation) and 急停 (Emergency stop) as the same action.
- Stop performs a controlled `movej` back to the current pose and leaves the arm enabled and holding. Emergency stop disables every axis and the arm falls under gravity.
- Support the arm before pressing emergency stop, removing power, or leaving the arm unattended while disabled.
- Keep the workspace clear and the emergency stop within reach whenever an arm is enabled.
- Do not hand-drag the follower arm. Only the leader runs in zero-gravity mode.
- Do not aim two console processes at one CDC port. STM32 CDC is not exclusive on Linux, so the second process silently shares the same byte stream instead of reporting that the port is busy.
- If a link drops, the follower stops following and holds its last target. Check the console log before restarting teleoperation.
- Do not mix firmware or SDK versions across the two machines. The topic names are shared by design, so mismatched formats decode into wrong values without an error.

## Documentation

| Document | Contents |
| --- | --- |
| [QUICKSTART.md](QUICKSTART.md) | First run, step by step |
| [README_zh.md](README_zh.md) | Chinese README |
| [QUICKSTART_zh.md](QUICKSTART_zh.md) | Chinese quickstart |
| [DEVELOPMENT.md](DEVELOPMENT.md) | Development setup and known traps |
| [AGENTS.md](AGENTS.md) | Repository automation rules |

## Related repositories

| Repository | Role |
| --- | --- |
| [litearm-teleop-vr](https://github.com/nexform-tech/litearm-teleop-vr) | VR teleoperation |
| [litearm-ros2](https://github.com/nexform-tech/litearm-ros2) | ROS 2 driver |
| [litearm-ros1](https://github.com/nexform-tech/litearm-ros1) | ROS 1 driver |
| [litearm-python](https://github.com/nexform-tech/litearm-python) | Python SDK |
| [litegrip-python](https://github.com/nexform-tech/litegrip-python) | Gripper SDK; owns the gripper teleop link and loop |
| [litearm-docs](https://github.com/nexform-tech/litearm-docs) | Product documentation |

## License

Copyright © 2026 NEXFORM ROBOTICS. Licensed under the [Apache License 2.0](LICENSE).
