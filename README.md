# litearm-teleop-isomorphic

How to run leader-follower (isomorphic) teleoperation on a pair of LiteArm arms,
and what will bite you if you improvise. Read this before running the two
processes for the first time.

Isomorphic teleoperation for the LiteArm robotic manipulator series. A human
drags the leader arm in zero gravity; the follower arm tracks it over a zenoh
point-to-point link. A gripper can be teleoperated over the same pair of
machines, on its own bus and its own link.

## Status

The arm path and the gripper path are both implemented. The arm path has been
validated on hardware; the gripper path has not. Track the remaining gripper
work in `docs/superpowers/plans/2026-09-28-gripper-teleop.md`, task 8.

Implemented and covered by the offline suite: the wire protocol codec, the zenoh
point-to-point link, the pure-logic safety layer, the firmware `joint_follow`
(`0x08`) servo loop, and the gripper link.

## Usage

Run two processes, one per machine. The arm link and the gripper link are
independent: separate CAN buses, separate zenoh sessions, separate ports
(`--jport`, default 17447 for the arm; `--gport`, default 17448 for the gripper),
separate topics, separate toggles in the UI.

```bash
# Leader machine: the arm is hand-dragged in zero gravity; the gripper likewise.
python -m liteteleop --role master --cdc /dev/ttyACM0 --gcan can0

# Follower machine: subscribes and follows.
python -m liteteleop --role slave  --cdc /dev/ttyACM1 --peer 192.168.31.10 \
                     --gcan can1 --gpeer 192.168.31.10
```

The gripper is published on `litearm/v4/{grip_id}/gripper_teleop` as a 32-byte
big-endian frame `(openness, position_mm, force_n, timestamp)`. `openness ∈ [0,1]`
is the payload that drives the follower. Omit `--gcan` to leave gripper
teleoperation off; that is the default.

Bring the CAN bus up first. The gripper needs its own bus (`can0` or `can1`), not
the arm's CDC port:

```bash
sudo ip link set can0 type can bitrate 1000000 && sudo ip link set can0 up
```

## Scope

| | |
| --- | --- |
| Product | LiteArm robotic manipulator series |
| Repository role | Isomorphic leader-follower teleoperation stack |
| Status | Arm and gripper teleoperation implemented; arm validated on hardware, gripper pending |

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
automated semantic-release versioning on every merge to `main`. All changes land
through a pull request; direct pushes to `main` are blocked by branch protection.

## Known traps

Every entry below was observed, in source or on hardware, and each one has cost
someone time. The full version lives in
`docs/superpowers/specs/2026-09-28-isomorphic-teleop-design.md` §10; when the two
disagree, the spec wins.

### Motion and feedforward

- Do not feed the leader's measured `dq` as the follower's velocity reference. In `move_js`, `dq` has two roles.
- Its magnitude caps the slew rate of `q_ref`, **and** its value enters the motor's velocity feedforward as `kd·(dq_ref − dq)`.
- Use the reference generator's own output instead. This repository's `slew_target` returns it as `dq_cmd`.
- Do not substitute a constant for it. That fabricates a feedforward torque.
- An axis pinned by a limit gets `dq = 0`, which freezes it.
- Do not trust `DEVELOPER_GUIDE.md` here. It states only that `dq` is "not a limit", which is half the story.
- Do not extrapolate the `dq` feedforward beyond what was measured. The hardware evidence covers up to 0.5 rad/s only; the script capped it and the actual peak was 0.032 rad/s.
- Teleoperation runs at 1–2 rad/s and nobody has verified that range. This repository's coefficient is 11x the server's (`kd` 11.0 against `B` 1.0), so the server's experience does not carry over.
- Do not call `set_speed()`. It scales the slew rate down globally through `gov_ratio`.

### Stopping and holding

- Do not treat "stop teleoperation" and "emergency stop" as the same action. Emergency stop is `EMERGENCY`: it disables every axis and the arm falls under gravity.
- Do not end a session without a `movej` back to the current pose. Only the `ht_on` state after `movej` reaches its target actually holds the arm.
- Do not expect a bare stop to hold either. Without `movej` the arm sags, because nothing supplies the gravity feedforward.
- Do not expect `park()` to hold the arm. It restores the fail-soft stiffness to 1.0x, undoing the 0.6x reduction, but `tau` stays zero, so the arm still sags by `G/mit_kp`.
- `park()` is not an equivalent of `request_stop`.
- Do not rely on the watchdog for a stable hold. Watchdog fail-soft is 0.6x stiffness with `tau = 0`: a slow sag, not a free fall and not a hold.

### Hardware and environment

- Do not aim two processes at one CDC port. STM32 CDC is not exclusive on Linux, so a second process silently consumes the same byte stream instead of failing to open it.
- Do not quote a `speed_limit` or `hold_kp_gain` value without stating which board it came from. The 1J bench and the 7J arm have two mutually exclusive default tables.
- On the bench, `speed_limit` is 3.5 and `hold_kp_gain` is unset, which means 0. So `ht_on` sends `kp = 0` there and the `movej` handover loses force instead of holding.
- Do not mix firmware or SDK versions across the link, even though the topic keys are deliberately shared with the server.
- Once the formats diverge, a shared key makes both ends decode the wrong thing without any error.

### Dependencies and cleanup

- Do not let a zenoh session go without `close()`. The process hangs permanently at exit.
- Do not resolve `litearm` by whichever import wins. Point it at the intended checkout and assert the result; a stale editable install silently satisfies `import litearm`.
- Do not `import litegrip` bare on a machine that has two same-named checkouts. Both resolve, and the wrong one wins.
- Go through `grip_worker.pin_grip_sdk()`, which asserts and logs. A `sys.path.insert` alone does not hold against a future switch to `meta_path.insert`.
- Do not expect `LiteGrip.close()` to disconnect. It closes the jaws; `disconnect()` drops the link.
- Do not leave `disable_on_disconnect` at its default when you want the gripper to keep holding. It defaults to `True`, so `disconnect()` disables the motors first.
- That drops whatever is being gripped. Pass `False` explicitly.
- Do not read `ok=True` from the gripper's `open()` or `close()` as success. It means the jaws stalled against a mechanical limit, the opposite of the intuitive reading.
- Do not expect an uncalibrated gripper to complain. `send_mit_frame` and `goto_rad` do not raise; they use placeholder limits as if they were real.

### Reading the codebase

- Do not cite firmware or SDK facts by line number. They drift: `clampf` moved between 716 and 722, and `watchdog_kick` between 725 and 733.
- Conditional-compilation tables also invite reading the wrong one. Cite by symbol.
- Cite by symbol instead.

## License

Copyright © 2026 NEXFORM ROBOTICS. Licensed under the
[Apache License 2.0](LICENSE).
