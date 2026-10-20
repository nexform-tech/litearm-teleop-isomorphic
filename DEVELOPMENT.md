# Development notes

Setup, architecture, and the hardware traps behind this repository, for developers maintaining or extending the teleoperation console.

The customer-facing usage is in [README.md](README.md) and [QUICKSTART.md](QUICKSTART.md). This file records behavior that is easy to get wrong when changing the code.

## Development setup

Install the same checkouts and dependencies as a customer:

```bash
mkdir -p ~/litearm && cd ~/litearm
git clone https://github.com/nexform-tech/litearm-python.git
git clone https://github.com/nexform-tech/litegrip-python.git
git clone https://github.com/nexform-tech/litearm-teleop-isomorphic.git
cd litearm-teleop-isomorphic
python3 -m pip install "pyserial>=3.4" "eclipse-zenoh>=1.6" "PyQt5>=5.15" pytest
export LITEARM_SRC="$HOME/litearm/litearm-python/src"
export LITEGRIP_SRC="$HOME/litearm/litegrip-python/src"
```

Run the offline suite before every commit and push:

```bash
python3 -m pytest tests -q
```

Skip the slow tests (subprocesses, tens of seconds) while iterating:

```bash
python3 -m pytest tests -q -m "not slow"
```

The suite does not pass through `liteteleop.__main__`, so `tests/conftest.py` pins
`litearm` the same way the entry point does and asserts the import location. Set
`LITEARM_SRC` to the checkout you intend to test.

Lint the README the way CI does:

```bash
npx --yes markdownlint-cli2@0.23.3 "README.md"
```

## Architecture at a glance

| Module | Role |
| --- | --- |
| `liteteleop/__main__.py` | CLI entry point; pins the `litearm` SDK path, then opens the GUI |
| `liteteleop/arm_worker.py` | The only thread that holds `Arm`; master publishes, slave runs the follow loop |
| `liteteleop/grip_worker.py` | The only thread that holds the gripper; delegates the link and loop to `litegrip.teleop_start` |
| `liteteleop/link.py` | Point-to-point zenoh listener and connector, with discovery disabled |
| `liteteleop/wire.py` | Arm teleoperation frame codec and topic names |
| `liteteleop/safety.py` | Soft-limit clamping and `slew_target`; no SDK import |
| `liteteleop/servo.py` | Slave follow loop, gains, and speed limits |
| `liteteleop/gui/` | PyQt5 console |

The arm worker and gripper worker are fully decoupled: separate threads, objects, zenoh sessions, ports, and lifecycles. The GUI displays them together but does not couple them.

## Known traps

Every entry below was observed in source or on hardware, and each one has cost someone time.

### Motion and feedforward

- Do not feed the leader's measured `dq` as the follower's velocity reference. In `move_js`, `dq` has two roles.
- Its magnitude caps the slew rate of `q_ref`, and its value enters the motor's velocity feedforward as `kd·(dq_ref − dq)`.
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
- Do not let the SDK refuse an uncalibrated gripper. `teleop_start` checks, but only after the caller enabled the motors. Check first, in `grip_worker.check_ready()`.
- Do not read a sessionless `teleop_status()` as a snapshot. It returns only `{"active": False, "mode": None}` — no `topic` or `frames` yet. Overwriting the readout with it zeroes the UI.

### Reading the codebase

- Do not cite firmware or SDK facts by line number. They drift, both for functions such as `clampf` and for calls such as `watchdog_kick`.
- Conditional-compilation tables also invite reading the wrong one.
- Cite by symbol instead.

## Repository standards

This repository follows the shared NEXFORM ROBOTICS repository standards: the agent
operating rules in [AGENTS.md](AGENTS.md), Conventional Commits, and automated
semantic-release versioning on every merge to `main`. All changes land through a
pull request; direct pushes to `main` are blocked by branch protection.
