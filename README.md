# litearm-teleop-isomorphic

How to run leader-follower (isomorphic) teleoperation on a pair of LiteArm arms,
and what will bite you if you improvise. Read this before running the two
processes for the first time.

Isomorphic teleoperation for the LiteArm robotic manipulator series. A human
drags the leader arm in zero gravity; the follower arm tracks it over a zenoh
point-to-point link. A gripper can be teleoperated over the same pair of
machines, on its own bus and its own link.

Chinese translation: [README.zh-CN.md](README.zh-CN.md). This file is the
authoritative one; where the two disagree, this one wins.

## Status

The arm path and the gripper path are both implemented. The arm path has been
validated on hardware; the gripper path has not.

Implemented and covered by the offline suite: the wire protocol codec, the zenoh
point-to-point link, the pure-logic safety layer, and the firmware `joint_follow`
(`0x08`) servo loop. Gripper teleoperation is **not** implemented here. This
repository delegates it to the `litegrip` SDK
(`LiteGrip.teleop_start`); the offline suite covers the thin worker that maps
that status into the UI.

## Usage

Run two processes, one per arm. Both arms normally connect to the same computer,
so both processes run there and the peer address defaults to loopback
(`127.0.0.1`). The arm link and the gripper link are independent: separate CAN
buses, separate zenoh sessions, separate ports (`--jport`, default 17447 for the
arm; `--gport`, default 17448 for the gripper), separate topics, separate
toggles in the UI.

```bash
# Leader process: the arm is hand-dragged in zero gravity; the gripper likewise.
python -m liteteleop --role master --cdc /dev/ttyACM0 --gcan can0

# Follower process: subscribes and follows.
python -m liteteleop --role slave  --cdc /dev/ttyACM1 --gcan can1
```

Pass `--peer` and `--gpeer` only when the two processes run on separate
machines; the default is loopback because a fixed LAN address breaks as soon as
the subnet changes.

The gripper link uses the topic `litearm/v4/{grip_id}/gripper_teleop` and the
32-byte big-endian frame `(openness, position_mm, force_n, timestamp)`, where
`openness ∈ [0,1]` drives the follower. Neither the frame nor the loop lives
here: both come from the `litegrip` SDK, which this repository calls through
`LiteGrip.teleop_start`. Omit `--gcan` to leave gripper teleoperation off; that
is the default.

The gripper teleoperation fields — peer, port, grip ID, align, and torque limit
— are read when you start gripper teleoperation, not when the worker is built.
Edit the port and start again, and the new port applies; the fields are not
re-read while a session runs, so stop and start to change them.

The arm SDK is vendored in `liteteleop/_vendor/litearm/` and ships with this
package; there is no separate install step. Provenance and the re-vendor
procedure are in `liteteleop/_vendor/litearm/VENDORED.md`. The private path lets
a `litearm-python` you installed for other work coexist: the worker imports the
vendored copy through `arm_sdk.import_arm_sdk()`, which puts that directory first
on `sys.path` and ignores any other `litearm` already loaded. To update the arm
SDK, update the vendored copy; this application never uses the installed one.

The gripper SDK is vendored in `litegrip/` and ships with this package; there is
no separate install step. Provenance and the re-vendor procedure are in
`litegrip/VENDORED.md`. Do not `pip install litegrip-python` alongside it: a
second distribution of the same top-level package can shadow the vendored copy.

The worker always loads the factory calibration bundled with that SDK
(`litegrip/factory_calibration.json`). It does not read `$LITEGRIP_CALIB` and
does not use a calibration measured on another machine.

Bring the CAN bus up first. The gripper needs its own bus (`can0` or `can1`), not
the arm's CDC port:

```bash
sudo ip link set can0 type can bitrate 1000000 && sudo ip link set can0 up
```

## Command filter

The follower filters the forwarded command before the per-tick speed limit. It is on by default and needs no flag.

| Setting | Default | Effect |
| --- | --- | --- |
| `LITEARM_CMD_FILTER=hybrid` | yes | J1–J4 second-order 20 Hz; J5–J7 One-Euro |
| `LITEARM_CMD_FILTER=lp20` / `lp25` | | all axes, fixed second-order |
| `LITEARM_CMD_FILTER=off` | | A/B only; restores the 98.5 Hz staircase |
| `LITEARM_OE_MIN_HZ` | `2` | hybrid: lower = smoother at low speed, more lag |
| `LITEARM_OE_BETA` | `5` | hybrid: higher = cutoff rises faster with speed (less lag in fast motion) |
| `LITEARM_OE_MAX_HZ` | `25` | hybrid: upper cutoff |
| `LITEARM_OE_KNEE` | `6` | hybrid: below this speed the wrist stays at `MIN_HZ`; raise = keep filtering at faster motion (more lag) |
| `LITEARM_J5_LP_HZ` | `0` (off) | hybrid: second-order low-pass added after J5's One-Euro; `12` cuts the 10–45 Hz wrist content at speed for about 20 ms of extra lag |

Why: the leader's ~98.5 Hz frames are held on the follower's 250 Hz loop, creating a 30–120 Hz staircase.
The 20 Hz path cuts that energy about 10x.
The wrist One-Euro additionally cuts the 6–10 Hz jitter; `LITEARM_OE_KNEE=6` keeps that smoothing up to 6 rad/s and only opens above it.

`LITEARM_J5_LP_HZ` addresses the fast-motion 10–45 Hz residue the One-Euro passes once its cutoff opens.
On the same recorded trajectories, `12` lowers J5's 10–20 Hz command content from 1.69 to 1.04 mrad and 20–45 Hz from 1.23 to 0.23 mrad, with no slow-speed penalty.
Do not replace it with a speed-scheduled switch or crossfade between the two filters: their phase differs, and blending raises the 6–10 Hz content from 3.04 to 8.16 mrad.

Do not set `off` for normal use. Do not lower `LITEARM_OE_MIN_HZ` below 1.5 Hz without checking the added wrist lag.

After alignment, if the master has moved, the follower catches up at **0.5 rad/s** until within **0.02 rad**, then returns to the normal speed limit.

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
| [litearm-python](https://github.com/nexform-tech/litearm-python) | Python SDK; this app uses a private vendored copy |
| [litegrip-python](https://github.com/nexform-tech/litegrip-python) | Gripper SDK; owns the gripper teleop link and loop |
| [litearm-docs](https://github.com/nexform-tech/litearm-docs) | Product documentation |

## Repository standards

This repository follows the shared NEXFORM ROBOTICS repository standards: the
agent operating rules in [`.nexform-standards.json`](.nexform-standards.json), Conventional Commits, and
automated semantic-release versioning on every merge to `main`. All changes land
through a pull request; direct pushes to `main` are blocked by branch protection.

## Known traps

Every entry below was observed, in source or on hardware, and each one has cost
someone time. This is the only copy; the internal design notes it was once
condensed from are not published.

### Motion and feedforward

- Do not feed the leader's measured `dq` as the follower's velocity reference **on `move_js`**: there it caps the `q_ref` slew rate *and* enters the feedforward.
- On `joint_follow` (`0x08`) the rate is capped by `speed_limit·dt` and `slew_linear`; `dq` only feeds the velocity term. `servo.follow` forwards leader `dq` as `dq_cmd`.
- That forwarding is what removes the steady-state lag: a target below `speed_limit` passes through unchanged, with no braking-distance offset.
- Do not substitute a constant for it. That fabricates a feedforward torque.
- `safety.slew_target` is now reference-only. The live path does not call it.
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
- Do not import `litearm` before calling `arm_sdk.import_arm_sdk()`.
- That helper puts the private `liteteleop/_vendor/litearm/` first on `sys.path` and replaces any other `litearm` already loaded, so a user's own `litearm-python` is never used.
- Do not edit files under `liteteleop/_vendor/litearm/`. Re-vendor instead (`liteteleop/_vendor/litearm/VENDORED.md`); the tree-hash test fails on a local edit.
- To update the arm SDK, update the vendored copy. Installing or upgrading `litearm-python` in the environment does not change what this application runs.
- Do not `pip install litegrip-python` beside this application. The SDK is vendored in `litegrip/`; a same-named distribution or a `sys.path` entry can shadow it and decide which calibration loads.
- Do not add a checkout to `sys.path` either. Use `grip_worker.import_grip_sdk()`; it imports the vendored copy and rejects a `litegrip` imported from outside `litegrip/`.
- Do not edit files under `litegrip/`. Re-vendor instead (`litegrip/VENDORED.md`); the tree-hash test fails on a local edit.
- Do not call argument-less `load_calibration()` here. It reads `$LITEGRIP_CALIB` and filters by `channel`; pass `grip_worker.factory_calibration_path()`.
- Do not point `--peer` or `--gpeer` at a LAN address when both arms are on one computer. The default is loopback; a subnet-specific address silently produces a link that receives no frames.
- Do not expect `LiteGrip.close()` to disconnect. It closes the jaws; `disconnect()` drops the link.
- Do not leave `disable_on_disconnect` at its default when you want the gripper to keep holding. It defaults to `True`, so `disconnect()` disables the motors first.
- That drops whatever is being gripped. Pass `False` explicitly.
- Do not let the SDK refuse an uncalibrated gripper. `teleop_start` checks, but only after the caller enabled the motors. Check first, in `grip_worker.check_ready()`.
- Do not read a sessionless `teleop_status()` as a snapshot. It returns only `{"active": False, "mode": None}` — no `topic`/`frames` yet. Overwriting the readout with it zeroes the UI.

### Reading the codebase

- Do not cite firmware or SDK facts by line number. They drift: `clampf` moved between 716 and 722, and `watchdog_kick` between 725 and 733.
- Conditional-compilation tables also invite reading the wrong one. Cite by symbol.
- Cite by symbol instead.

## License

Copyright © 2026 NEXFORM ROBOTICS. Licensed under the
[Apache License 2.0](LICENSE).
