# 3. Bring up the Franka

**Goal:** a real FCI handshake: `rwrl doctor` reports `PASS arm.handshake: robot connected` and
ends with `Result: no blocking findings`, and `rwrl status` reads the arm. This page uses the
doctor to debug the arm; nothing here moves it.

All commands run on the NUC, from the repository root, with the project environment active.

## Before you start

The same checks as [02](02-setup.md#before-you-start), now required:

1. The arm is powered, the emergency stop released, joints unlocked, End Effector power on.
2. The NUC is on the **Control** port and `ping -c 3 -W 1 172.16.0.2` replies.
3. Desk on the NUC: you hold control (refresh to be sure) and FCI is **activated**.
4. No recorder, actor or daemon holds the arm: Franka FCI allows one control connection.

`local/station.yaml` exists ([02](02-setup.md)).

## 1. Run the doctor

```bash
rwrl doctor --station local/station.yaml
```

For two arms, check one arm at a time:

```bash
rwrl doctor --station local/station.yaml --robot left
rwrl doctor --station local/station.yaml --robot right
```

The doctor connects to each arm and disconnects again; it never homes or moves it.
Initializing a configured device can still have side effects, such as GELLO torque-off.

## 2. Read the report

Every line is `STATUS id: message`, the status padded to 14 characters, and every line that is
not `PASS` is followed by `    hint: ...`. On a terminal the status is coloured
([00](00-overview.md#status-words)). The rows come in two groups:

- **Host rows** (`host.*`, `usb.*`, `gello.*`, `network.route.<ip>`, `realtime.*`, `deps.*`,
  `sdk.*`, `coverage.*`): this NUC, its permissions and the project environment.
- **Device rows** `<robot>.<check>` (here `arm.<check>`), in six layers:

| Layer | Rows | What it means |
|---|---|---|
| 0 deps | `arm.deps` | the pip extras this station needs are installed |
| 1 sdk | `arm.sdk:arm`, `arm.sdk:gripper`, `arm.sdk:camera:<name>`, `arm.sdk:teleop:<backend>` | each backend's SDK imports |
| 2 os | `arm.os:ping`, `arm.os:tcp`, `arm.os:camera:<name>`, `arm.os:serial`, `arm.os:groups`, `arm.os:brltty`, `arm.os:latency_timer` | network, device nodes and permissions |
| 3 handshake | `arm.handshake` | the device claim and `connect()`; a failure skips layers 4-5 |
| 4 feedback | `arm.feedback:arm`, `arm.feedback:gripper`, `arm.feedback:camera:<name>`, `arm.feedback:teleop:<name>` | each device returns data |
| 5 mode | `arm.mode:robot_mode`, `arm.mode:libfranka_version`, `arm.mode:realtime_kernel` | the arm's mode permits use |

After the rows come `Report:` (this run's report directory), a `Needs attention:` block that
repeats every `FAIL`, `BLOCKED` and `PENDING_ACTION` row with its first hint line (only when
there is one), and the `Result:` line. The exit code is 0 with only `PASS`, `WARN` and
`SKIPPED`, and 1 otherwise.

To see the shape without hardware, run the doctor on the example station with fake backends.
On the NUC, repository root, project environment active:

```bash
rwrl doctor --station examples/stations/robot.franka.example.yaml --mock
```

*You should see* (the example robot is named `franka`; your platform, interpreter path and
report directory differ):

<!-- doctor-example: mock -->
```text
PASS           host.platform: Linux ubuntu 22.04 x86_64; container=False; wsl=False
PASS           host.python: /home/robot/real-world-rl/.venv/bin/python
PASS           franka.deps: no extras needed (fake or SDK-free backends)
PASS           franka.sdk:arm: no SDK needed
PASS           franka.sdk:gripper: no SDK needed
PASS           franka.sdk:camera:wrist: no SDK needed
SKIPPED        franka.os:ping: skipped (--mock)
SKIPPED        franka.os:tcp: skipped (--mock)
SKIPPED        franka.os:camera:wrist: skipped (--mock)
PASS           franka.handshake: robot connected
PASS           franka.feedback:arm: arm feedback received
PASS           franka.feedback:gripper: gripper width 0.0800 m (1.00 of max_width_m 0.080), grasped=unknown, source FakeGripper
PASS           franka.feedback:camera:wrist: frames arriving
PASS           franka.mode:none: backend declares no mode checks
Report: /home/robot/real-world-rl/local/doctor/20261112T101500-3f2a9c1d
Result: no blocking findings; review SKIPPED/WARN coverage
```

`--mock` replaces every backend with a fake one and skips layer 2, so it checks the YAML and
the install, not the robot. On the real station the device rows name the real backends, for
example `PASS           arm.sdk:arm: 'franky' importable`,
`PASS           arm.handshake: robot connected` and
`PASS           arm.mode:robot_mode: robot mode is Idle`. A `SKIPPED` row is not evidence that a
part works: find out why it was skipped.

## 3. Red lights

Find the first red row and its hint. The common ones:

| Row | Usual cause | What to do |
|---|---|---|
| `arm.os:ping` FAIL `172.16.0.2 does not reply` | power, cable on the Arm port, wrong address | [Network and FCI](../doctor.md#network-and-fci) |
| `arm.handshake` FAIL `cannot reach Franka at 172.16.0.2: ...` | FCI not activated, another machine holds control in Desk, firewall blocks UDP | [Network and FCI](../doctor.md#network-and-fci) |
| `arm.handshake` FAIL `libfranka mismatch: robot server N, installed library M` | the franky wheel does not match the robot server | `rwrl doctor --station local/station.yaml --install-sdk`; [SDK and libfranka version](../doctor.md#sdk-and-libfranka-version) |
| `arm.deps` or `arm.sdk:*` FAIL | a missing extra | the hint's `pip install -e '.[...]'`; [Python environment and extras](../doctor.md#python-environment-and-extras) |
| `arm.os:serial`, `arm.os:groups`, `arm.os:brltty`, `arm.os:latency_timer` | GELLO serial access | `rwrl doctor --station local/station.yaml --fix`; [Serial, input groups and USB latency](../doctor.md#serial-input-groups-and-usb-latency) |
| `arm.feedback:camera:<name>` FAIL `no frames within 2s` | USB bandwidth, cable, udev rules | [Cameras](../doctor.md#cameras) |
| `arm.device` BLOCKED `device inspection requires an idle station: ...` | a recorder, actor or daemon holds the arm | finish that process; [Busy devices](../doctor.md#busy-devices-blocked) |
| `arm.mode:robot_mode` FAIL `robot mode is Reflex` | a reflex stop, user stop or guiding mode | step 5 below; [Robot mode](../doctor.md#robot-mode-reflex-userstopped-guiding) |

For a libfranka mismatch the guided repair asks you to cross-check the robot server version
(Desk → Settings → System, through the compatibility matrix) and installs the matching wheel
only after you confirm:

```bash
rwrl doctor --station local/station.yaml --install-sdk
```

For serial and input permissions, the guided repair previews every change, asks once, applies
it and inspects again:

```bash
rwrl doctor --station local/station.yaml --fix
```

Log out and back in if it changed your group membership, then rerun the plain doctor of step 1.

## 4. Read the arm's state

With the doctor green and nothing else holding the arm:

```bash
rwrl --station local/station.yaml status
rwrl --station local/station.yaml q
rwrl --station local/station.yaml tcp
```

*You should see,* for example:

```text
connection: direct franky
robot: arm (franka)
arm: FrankyArm, ok=True, mode=idle, errors=[]
gripper width 0.0800 m (1.00 of max_width_m 0.080), grasped=False, source FrankyGripper
cameras: wrist
```

followed by the teleop name, then `q (rad): [...]` with seven joint positions and
`tcp xyz (m): [...]; quat (xyzw): [...]`. `connection: direct franky` means `rwrl` opened the
arm itself; a station with a `daemon:` block reports `connection: daemon HOST:PORT` instead
([connection ownership](../cli.md#connection-ownership)). Each command connects for its run and
disconnects afterwards. `--station local/station.yaml` can be left out when you run from the
directory that holds `local/`.

## 5. After a reflex stop

A collision, joint-limit violation or the external activation device puts the arm in Reflex or
UserStopped; the doctor then reports `arm.mode:robot_mode` FAIL with the hint
`release the external activation device / clear errors in Desk, then run rwrl recover`. Inspect
the cell, release the activation device or clear the error in Desk, then:

```bash
rwrl --station local/station.yaml recover
rwrl doctor --station local/station.yaml
```

*You should see:* `recover: ok`, then `arm.mode:robot_mode` back to `PASS`. `rwrl recover` uses
the same connection as `status`: on the direct route, finish any recorder or actor first.

Comparing an env preset with this station (`rwrl doctor --preset ...`) needs the preset's teleop
files and is done on [08](08-record-demos.md).

## A robot someone else has already commissioned

Reuse the existing `local/station.yaml` and project environment on the NUC, and do just these
three things before a session:

1. Open this arm's Desk on the NUC, **Request control** if necessary, and refresh.
2. Confirm that this arm's FCI session is **activated**.
3. With other device-owning processes stopped, run the doctor of step 1.

*You should see:* a real handshake and no failed rows. Do not overwrite a commissioned station
or recalibrate it just because another browser kept control.

## If it fails

Look at which doctor row is red, then open the matching section of
[rwrl doctor: red lights](../doctor.md#red-lights-what-to-do). Fix the first red row before the
others: a failed handshake skips every feedback and mode row.

Next: [04 Home, gripper, cameras](04-first-motion.md).
