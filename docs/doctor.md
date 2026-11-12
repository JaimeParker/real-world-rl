# rwrl doctor

`rwrl doctor` is the one diagnostic of a robot host. It checks the host, the project
environment and, for each station robot, the layered device checks, and prints them in one
report with one set of statuses and exit codes. `rwrl setup` and the `rwrl daemon` preflight
print their device checks in the same form. The tutorial uses it on
[01](tutorial/01-install.md) (no hardware) and [03](tutorial/03-franka-bringup.md) (the real
handshake); this page is the reference.

## Running it

Run it on the robot host with the project environment active (a source checkout or an
installed package; the repository root is not required). A station is optional:

```bash
rwrl doctor
rwrl doctor --station local/station.yaml
rwrl doctor --station local/station.yaml --robot left
rwrl doctor --station local/station.yaml --mock
rwrl --station local/station.yaml doctor --fix
```

| Option | Meaning |
|---|---|
| `--station PATH` | the station to check; default: the [station lookup](cli.md#station-lookup). Without any station only the host is checked |
| `--robot NAME` | check one robot of a multi-robot station |
| `--preset PATH` | also read an env preset: its teleop devices are expected, and its robot is compared with the station ([below](#comparing-a-preset-with-the-station)) |
| `--mock` | fake backends for every device; skips the host and OS layers |
| `--fix` | preview, confirm once, apply host and software repairs, inspect again |
| `--install-sdk` | repair only a handshake-confirmed franky mismatch |
| `--report-dir PATH` | report base directory (overrides `$RWRL_DOCTOR_REPORT_DIR`) |
| `--json` | one JSON object on stdout; the text goes to stderr |
| `--yes` | skip the repair-plan confirmation; every precondition still applies |

`--station`, `--robot`, `--preset`, `--mock`, `--json` and `--yes` work before or after
`doctor`; a value after `doctor` wins. `rwrl doctor --help` lists the flags.

The doctor connects to each configured device and disconnects again. It never recovers, homes
or moves a robot, never stops or takes over another process, and never changes network
addresses, firmware, kernels or boot settings. Initializing a configured device can still have
side effects, such as GELLO torque-off.

## Reading the output

```text
PASS           host.python: /venv/bin/python
SKIPPED        coverage.gello: GELLO not configured
    hint: run rwrl setup
WARN           arm.preset:robot_ip: station="a"; preset="b"
    hint: regenerate the preset
FAIL           arm.device: Traceback (most recent call last):
  ...
ConnectionError: robot unreachable
    hint: check the Ethernet cable
          and the robot IP in Desk
BLOCKED        arm.os:claim: device inspection requires an idle station
    hint: finish the controlling program
PENDING_ACTION groups.dialout: log out and back in
Report: local/doctor/20261112T101500-3f2a9c1d
Needs attention: 1 FAIL, 1 BLOCKED, 1 PENDING_ACTION
  FAIL           arm.device: ConnectionError: robot unreachable
    hint: check the Ethernet cable
  BLOCKED        arm.os:claim: device inspection requires an idle station
    hint: finish the controlling program
  PENDING_ACTION groups.dialout: log out and back in
Result: unresolved failures / blocked checks / pending user actions
```

- **Check lines:** `STATUS id: message`, the status padded to 14 characters. A line that is not
  `PASS` is followed by `    hint: ...`; continuation lines of a hint line up under its text.
- **Plan lines** (when repairs are possible): `PLAN <id>: <title>`, then the command or the file
  and its content, `  BLOCKED: <reason>` when the repair cannot run here, and
  `  Afterward: <step>` for what you must do next. `--fix` asks `Apply this plan? [y/N]` once.
- **`Report:`** the directory of this run's saved report ([below](#report-directory)).
- **`Needs attention:`** only when something is unresolved: the counts, then every `FAIL`,
  `BLOCKED` and `PENDING_ACTION` check and repair with the last line of its message and the
  first line of its hint.
- **`Result:`** `no blocking findings; review SKIPPED/WARN coverage` or
  `unresolved failures / blocked checks / pending user actions`.

After a confirmed repair the doctor prints every repair and check again with its new status.

### Statuses and colours

| Status | Meaning | Colour | Exit code |
|---|---|---|---|
| `PASS` | checked and fine | green | 0 |
| `WARN` | fine, but act on the hint (a stale preset field, a high latency timer) | yellow | 0 |
| `SKIPPED` | not checked: not configured, `--mock`, or an earlier layer failed; never evidence of hardware | grey | 0 |
| `FAIL` | checked and broken | red | 1 |
| `BLOCKED` | could not be checked or repaired: a busy device, a missing capability, permission or dependency, an unconfirmed plan | bold magenta | 1 |
| `PENDING_ACTION` | a repair ran and needs a step from you (log in again, replug) before it can be verified | yellow | 1 |

`Report:` and `Needs attention:` are bold, `PLAN` is cyan, and `Result:` is bold green or bold
red. Colour is used only when the stream is a terminal; a set `NO_COLOR` (any value),
`TERM=dumb`, Windows consoles and piped output keep the text plain. Saved reports never contain
colour codes.

## What it checks

### Host rows

| Ids | Checks |
|---|---|
| `host.platform`, `host.python`, `host.tools` | the platform; that the interpreter is a project venv or non-base conda environment; the diagnostic tools |
| `deps.<distribution>`, `deps.<distribution>.source`, `sdk.<extra>` | the project's and each expected device's Python dependencies, with their version constraints; that SDKs import |
| `system.library.<name>` | shared libraries such as libusb and libudev |
| `usb.discovery`, `usb.<kind>`, `usb.spacemouse.table`, `usb.spacemouse.unknown.<vid>:<pid>` | visible USB devices, SpaceMouse and RealSense access and udev rules, the SpaceMouse device table |
| `gello.<n>`, `gello.<n>.group`, `gello.<n>.events`, `gello.<n>.latency`, `gello.<n>.brltty`, `gello.candidates` | commissioned GELLO serial and event access, user groups, FTDI latency, a brltty conflict; unconfigured serial adapters are only candidates |
| `network.route.<ip>` | a route to each robot address |
| `realtime.kernel`, `realtime.limits` | PREEMPT_RT kernel and realtime limits (`MAJOR CHANGE / DOUBLE CHECK` when kernel work may be needed; nothing is changed) |
| `configuration.*` | station loading; `configuration.legacy.*` WARNs for pre-0.1.0 station names, with the command that moves them |
| `station` | `SKIPPED` when no station is configured |
| `read.<kind>`, `coverage.<kind>` | an SDK read of an unconfigured SpaceMouse or RealSense; `SKIPPED` for device kinds that nothing configured or identified |
| `sdk.franky.conflict` | robots that need different franky wheels |

### Device rows

Each station robot gets rows `<robot>.<check>` (for example `arm.handshake`), in six layers.
A `teleop:` section adds rows to layers 1, 2 and 4.

| Layer | Rows | Notes |
|---|---|---|
| 0 deps | `deps` | the extras this robot needs: one per backend, plus `grpc` with a `daemon:` block; without one a missing `grpc` is only a `WARN` |
| 1 sdk | `sdk:arm`, `sdk:gripper`, `sdk:camera:<name>`, `sdk:teleop:<backend>` | each backend resolves and its SDK imports; an unknown backend skips layers 3-5 |
| 2 os | `os:ping`, `os:tcp`, `os:camera:<name>`, `os:teleop:<name>`, `os:hid`, `os:serial`, `os:groups`, `os:brltty`, `os:latency_timer`, `os:can_*` | network, device nodes, permissions; `os:tcp` is informational |
| 3 handshake | `handshake` | takes the robot's device claim (unless `claim: false`), then `connect()`; a failure skips layers 4-5 |
| 4 feedback | `feedback:arm`, `feedback:gripper`, `feedback:camera:<name>`, `feedback:teleop:<name>` | each device returns data; a camera must deliver a frame within 2 s |
| 5 mode | `mode:robot_mode`, `mode:libfranka_version`, `mode:realtime_kernel`, or `mode:none` | backend preflight |

`<robot>.device` is `FAIL` when the device probe itself fails and `BLOCKED` when another
process holds the station. The [detailed examples](#device-diagnostic-details) below show the device messages.

### Expected devices

The station and preset decide which devices are expected. A missing configured device is
`BLOCKED`; an unconfigured, unidentified device is `SKIPPED`, not accepted as working.
Recognized SpaceMouse and RealSense USB devices are also inspected on their own.

Only processes that hold a hardware connection count as owners: the daemon, the recorder,
actor and deploy entrypoints, `setup`, the connecting primitives (`status`, `q`, `tcp`, `home`,
`gripper`, `camera`, `stop`, `recover`), `teleop run/read/calibrate/diagnose/probe-hid`,
`workspace capture` and the learning commands. Offline commands (`--help`,
`workspace show/set/validate/frame`, `teleop profiles`, `teleop inspect-stop`,
`archive inspect`) do not make the doctor report `BLOCKED`.

## `--mock`

`--mock` swaps every backend for its fake (a teleop's private options are dropped) and skips
layer 2 and the host's OS, USB and dependency inspection. It checks the station YAML and the
install with no hardware; it does not validate a device.

## Comparing a preset with the station

With a station and `--preset`, the doctor compares the preset's robot (the inline
`real_robot.robot` or bare `robot` mapping, or the station file it names) with the station
robot: `robot_ip`, `arm_backend`, `gripper`, cameras by name (`serial`, `width`, `height`,
`fps`, `enable_depth`), `home_joints`, `joint_limits`, `safety_box`, `task_frame` and
`workspace`. Matching fields give one `PASS` row `<robot>.preset`
(`preset hardware fields match the station`). Each differing field is a `WARN`
`<robot>.preset:<field>`, for example
`arm.preset:robot_ip: station="172.16.0.2"; preset="172.16.0.3"` or
`arm.preset:cameras.wrist.fps: station=30; preset=15`, with the hint
`the preset copy of the station is stale; regenerate it or edit the field`. WARN does not change
the exit code, so read the rows. An inline robot on a multi-robot station needs `--robot`;
otherwise the comparison is `SKIPPED`.

A preset whose teleop is a GELLO also names its leader options file (`teleop_options_yaml`);
that file must exist, or the doctor stops with a usage error.

## Repairs: `--fix`

```bash
rwrl --station local/station.yaml doctor --fix
```

The doctor shows the plan (commands, the content of dedicated configuration files, the steps
left to you), asks once, applies the approved plan and inspects again. Only `y` (any case)
confirms. Independent repairs continue after a failure; dependent repairs are `BLOCKED`. New
problems found by the second inspection need another reviewed plan. Without a terminal and
without `--yes`, nothing is repaired and the plan is reported `BLOCKED`
(`plan not confirmed; no repairs executed`).

Repairs can install missing host tools, add user groups, write dedicated
`/etc/udev/rules.d/99-rwrl-*.rules` and current-user realtime limits, reload udev rules, and
repair the project environment's Python packages with the running interpreter (never
`sudo pip`). When a GELLO serial port or SpaceMouse node is not readable and writable, the plan
also offers `setfacl -m u:<user>:rw -- <node>`: immediate, and lost on replug. Previous file
contents are backed up and concurrent edits are refused. Replugging a device and logging in
again remain `PENDING_ACTION` until the doctor can verify them. A missing sibling project
distribution is repaired from a verified local source tree; otherwise it is `BLOCKED` with the
instruction to install it from your trusted package source.

| Host | Available |
|---|---|
| Native Linux | host and USB diagnostics, device probes through installed backends |
| Debian/Ubuntu family with `apt-get` | missing tools, libusb/libudev packages |
| Native Linux with udev, user-group tools, PAM limits or running systemd | the matching rule, group, limit or proven service-conflict repair |
| Containers, WSL, macOS, Windows | Python metadata and import diagnostics and Python repairs; native probes and system repairs are `BLOCKED` |

System repairs run individual commands with sudo when not root, and a missing sudo
authorization is `BLOCKED`; passwords are never read from a pipe. For an unattended run,
authorize sudo in a terminal first:

```bash
sudo -v
rwrl --station local/station.yaml --yes --json doctor --fix
```

## `--install-sdk`

The `franky` extra installs the default `franky-control` wheel, which bundles one libfranka
version. A robot whose robot server version differs refuses the handshake, and the doctor
reports `<robot>.handshake` FAIL `libfranka mismatch: robot server <N>, installed library <M>`.
Then:

```bash
rwrl doctor --station local/station.yaml --install-sdk
```

The doctor plans one repair,
`PLAN sdk:franky: Install Franky wheel for handshake-confirmed server version <N>`, with the
exact `pip install franky-control --extra-index-url ...` command, asks once and installs into
the running project interpreter only after `y` (`--yes` skips the question). The server version
comes from libfranka's handshake error; cross-check it with the system version in Desk →
Settings → System and the
[compatibility matrix](https://frankarobotics.github.io/docs/compatibility.html). Do not use the
FCI feature version (`FCI 1.0.0`). `--install-sdk` without `--fix` runs no other repair. Robots
that need different server versions are `BLOCKED` as `sdk.franky.conflict`: use separate
project environments. Rerun the plain doctor afterwards.

## Report directory

Each run saves credential-redacted diagnostics: `before.json`, `plan.json`, `report.json` and
`commands.log` (the raw output of every command it ran), plus `error.json` on a configuration
error and backups or staged files when needed. The doctor does not save its own console text.
The base directory is the first of:

1. `--report-dir PATH`;
2. `$RWRL_DOCTOR_REPORT_DIR`;
3. the source checkout's git-ignored `local/doctor/`, when writable;
4. the user state directory: `${XDG_STATE_HOME:-~/.local/state}/rwrl/doctor/` on Linux,
   `~/Library/Application Support/rwrl/doctor/` on macOS, `%LOCALAPPDATA%/rwrl/doctor/` on
   Windows.

Each run creates a `<UTC timestamp>-<id>/` subdirectory, mode 0700 with files 0600 on POSIX.
An explicitly requested unwritable destination is an error; an unwritable checkout falls back
to user state. Backups keep exact previous bytes: treat the whole directory as private.

## JSON output

With `--json`, stdout carries one object: `schema_version: 1`, `status` (`PASS` or `FAIL`),
`checks`, `repairs`, `remaining_plan`, `report_directory` and per-robot low-level `results`
(each a list of `{layer, name, ok, message, hint, skipped, warning}`). The check lines, plan
and prompt go to stderr. Argument, configuration and report-creation errors return exit code 2
and a JSON error object with `schema_version`, `error` and `report_directory`.

## Exit codes

| Result | Exit code |
|---|---|
| only `PASS`, `WARN` or `SKIPPED` | 0; still read the `SKIPPED` and `WARN` rows before calling hardware accepted |
| any `FAIL`, `BLOCKED` or `PENDING_ACTION` | 1 |
| invalid arguments or configuration | 2 |

## Red lights: what to do

Start with the first red row: a failed handshake skips every feedback and mode row, and a
missing extra fails its SDK rows too. The tables quote the messages and hints; `<ip>` and
similar values come from your station. Piper and CAN rows are in the
[device-layer troubleshooting table](doctor.md#troubleshooting).

### Network and FCI

| Row | Cause | What to do |
|---|---|---|
| `<robot>.os:ping` FAIL `<ip> does not reply` | the arm is off, the cable is on the Arm port, the address or subnet is wrong | power, the **Control** port, the NUC address ([02](tutorial/02-setup.md#before-you-start)) |
| `<robot>.handshake` FAIL `cannot reach Franka at <ip>: ...`, hint `check: robot powered on and brakes released; Desk reachable at https://<ip>/desk; you hold control in Desk; FCI activated ...` | FCI not activated, another machine holds control, a firewall drops UDP | work through the hint in order; refresh Desk, **Request control**, activate FCI |
| `network.route.<ip>` FAIL | no route to the robot subnet | add the address on the Control interface ([02](tutorial/02-setup.md#before-you-start)) |

### SDK and libfranka version

| Row | Cause | What to do |
|---|---|---|
| `<robot>.handshake` FAIL `libfranka mismatch: robot server <N>, installed library <M>` | the franky wheel does not match the robot server | `rwrl doctor --station local/station.yaml --install-sdk` ([above](#--install-sdk)) |
| `<robot>.sdk:arm` FAIL `FrankyArm needs the 'franky' package` | the `franky` extra is missing | the hint's `pip install -e '.[franky]'` from the repository root |
| `sdk.franky.conflict` BLOCKED | two robots need different franky wheels | one project environment per server version |

### Python environment and extras

| Row | Cause | What to do |
|---|---|---|
| `host.python` WARN | the interpreter is not a project venv or non-base conda environment | activate the project environment ([01](tutorial/01-install.md)) |
| `<robot>.deps` FAIL `missing extras for this station: franky, gello` | extras for the station's backends are missing | the hint's `pip install -e '.[franky,gello]'` from the repository root |
| `deps.<distribution>` FAIL | a dependency is missing or violates its version constraint | `rwrl doctor --fix` reinstalls it in the project environment |
| `system.library.<name>` FAIL | a shared library (libusb, libudev) is absent | `rwrl doctor --fix` on apt systems; otherwise install it with the system package manager |

### Serial, input groups and USB latency

| Row | Cause | What to do |
|---|---|---|
| `<robot>.os:serial` FAIL `no read/write access to <port>` | this user cannot open the GELLO port | `rwrl doctor --fix` (group, udev rule, or an immediate `setfacl` lost on replug) |
| `<robot>.os:groups` FAIL `not in group(s) dialout, input; ...` | the login session lacks `dialout` (serial) or `input` (evdev) | `rwrl doctor --fix`, or the commands below, then log out and back in |
| `<robot>.os:brltty` FAIL `brltty active and a CH340 adapter (USB id 1a86) is attached; ...` | brltty claims the CH340 adapter | `rwrl doctor --fix` when the conflict is proven, or the commands below, then replug |
| `<robot>.os:latency_timer` WARN `<tty> latency_timer is 16 ms; 1 ms is recommended ...` | an FTDI adapter reads slowly | `rwrl doctor --fix` writes the udev rule; replug. CH340 adapters have no latency timer |

The manual equivalents, for when `--fix` offers no repair for your case. On the robot host, no
environment needed. Use the stable `/dev/serial/by-id/` path, not an assumed `ttyUSB0`:

```bash
ls -l /dev/serial/by-id/
sudo usermod -aG dialout "$(id -un)"
sudo usermod -aG input "$(id -un)"
```

Log out and back in, reactivate the project environment and rerun the doctor. `input` is only
needed for an evdev keyboard or pedal; terminal keys need no group, and a `pynput` listener
needs the desktop session, not just an SSH shell in `input`.

For a CH340 adapter that appears and disappears, check the log first; if it confirms brltty and
no braille device needs it, release the adapter until reboot, then unplug and replug it:

```bash
journalctl -b -u brltty-udev.service --no-pager
sudo systemctl mask --runtime --now brltty-udev.service
```

For a high FTDI `latency_timer`, a temporary write (the path is your adapter's):

```bash
GELLO_PORT=/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter-if00-port0
GELLO_TTY="$(basename "$(readlink -f "$GELLO_PORT")")"
echo 1 | sudo tee "/sys/bus/usb-serial/devices/$GELLO_TTY/latency_timer"
```

An absent `latency_timer` file means the driver has none; it is not a permission failure.

### Cameras

| Row | Cause | What to do |
|---|---|---|
| `<robot>.os:camera:<name>` FAIL `camera '<name>' serial '<s>' not found`, hint `set serial; discovered: [...]` | the serial is wrong or the camera is unplugged | copy a serial from `rwrl discover`; with more than one camera `serial` is required |
| `<robot>.feedback:camera:<name>` FAIL `no frames within 2s` | USB 2 port or cable, a shared hub, missing udev rules | a USB 3 port and cable, the librealsense udev rules; reconnect, then `rwrl status` |
| `<robot>.sdk:camera:<name>` FAIL | `pyrealsense2` missing | `python -m pip install -e '.[realsense]'` |

### SpaceMouse and HID

| Row | Cause | What to do |
|---|---|---|
| `<robot>.sdk:teleop:spacemouse` FAIL `SpaceMouse needs the 'hidapi==0.14.0.post4' distribution (import hid)` | the `spacemouse` extra is missing | `pip install -e '.[spacemouse]'` |
| `<robot>.os:hid` FAIL `no HID device found` or `no read/write access to [...]` | unplugged, or no udev rule | [SpaceMouse udev steps](teleop.md#spacemouse-installation-and-linux-permissions), or `rwrl doctor --fix` |
| `usb.spacemouse.unknown.<vid>:<pid>` WARN | a device missing from the device table | `rwrl teleop probe-hid` ([site device table](spacemouse-commissioning.md#site-device-table)) |

### Busy devices (BLOCKED)

| Row | Cause | What to do |
|---|---|---|
| `<robot>.device` BLOCKED `device inspection requires an idle station: ...` | a recorder, actor, daemon or connecting `rwrl` command holds the robot | finish that program and rerun; the doctor never stops it |
| `<robot>.handshake` BLOCKED `... claimed by another process` | another process holds the robot's device claim | the same |
| `read.<kind>` BLOCKED | a device owner prevents an SDK read | the same |
| `usb.<kind>` BLOCKED | a configured USB device is missing | plug it in, check the cable |

### Robot mode: Reflex, UserStopped, Guiding

`<robot>.mode:robot_mode` FAIL `robot mode is Reflex` (or `UserStopped`, `Guiding`), hint
`release the external activation device / clear errors in Desk, then run rwrl recover`.
Inspect the cell, release the activation device or clear the error in Desk, then run
`rwrl recover` and the doctor again ([03](tutorial/03-franka-bringup.md#5-after-a-reflex-stop)).
`--fix` never recovers a robot.

### Realtime kernel and limits

| Row | Cause | What to do |
|---|---|---|
| `realtime.kernel` WARN | not a PREEMPT_RT kernel | `realtime: ignore` permits it (control jitter is possible); `realtime: enforce` needs an RT kernel, installed by hand |
| `realtime.limits` FAIL or `PENDING_ACTION` | rtprio and memlock limits are missing or not yet effective | `rwrl doctor --fix` writes them; log out and back in |
| `<robot>.mode:realtime_kernel` FAIL `realtime: enforce but the kernel is not PREEMPT_RT` | the station enforces realtime | install an RT kernel or set `realtime: ignore` |

## Device diagnostic details

On the robot host, from the repository root with the project environment active:


```bash
rwrl doctor --station robot.yaml
```

`rwrl doctor` runs the device checks below for every station robot, next to its host and
project checks, and reports each one as a `<robot>.<check>` row (for example `arm.handshake`)
with `PASS`, `FAIL`, `WARN` or `SKIPPED`; see [cli.md](cli.md#robot-host-diagnosis-and-repair)
for its report, flags and exit codes. `rwrl setup` and the `rwrl daemon` preflight print their
device checks in the same form.

Without `--station` the doctor checks the [station file](tutorial/02-setup.md). A station with several
robots is checked robot by robot; `--robot NAME` checks one.

The doctor runs six layers in order, 0 to 5 (a `teleop:` section adds checks to layers 0, 1, 2 and 4):

| Layer | Checks | Notes |
|---|---|---|
| 0 `deps` | the pip extras this robot needs are installed: one per backend (`franky`, `pyagxarm`, `piper-sdk`, `realsense`, `spacemouse`, `gello`, plus `gello-events` for a gello `event_device`), and `grpc` when the robot's yaml has a `daemon:` block. FAIL `missing extras for this station: franky, gello`, hint `pip install -e '.[franky,gello]' (from the repository root)`; OK `extras ok: franky, gello, grpc`. Without a `daemon:` block (the single-process route) a missing `grpc` is a WARN `optional, not installed: grpc (daemon route)`; a missing `jpeg` is only mentioned | with `--mock` the fakes need none and grpc is never required |
| 1 `sdk` | each backend name resolves and its SDK imports (`sdk:arm`, `sdk:gripper`, `sdk:camera:<name>`, `sdk:teleop:<backend>`) | an unknown backend skips layers 3-5 |
| 2 `os` | ping `robot_ip` and port 1337 (informational); CAN interface exists / is up / bitrate; camera serial attached; `os:teleop:<name>` (a device of that backend is discovered); `os:hid` (a readable and writable `/dev/hidraw*` exists, SpaceMouse only); `os:serial` (the gello `port` exists and is readable and writable; `rwrl doctor --fix` offers `setfacl -m u:<user>:rw -- <port>`, lost on replug); `os:groups` (the user's login session is in `dialout` for the gello port and `input` for its `event_device`); `os:brltty` (brltty would claim a CH340 adapter); `os:latency_timer` (an FTDI adapter's `latency_timer` is 1 ms) | driven by config fields; skipped for backends that need no SDK (fakes); never runs sudo |
| 3 `handshake` | takes the robot's device claim (unless `claim: false`), then `robot.connect()` | a failure (including a held claim) skips layers 4-5 |
| 4 `feedback` | the arm returns state; the gripper reports its width; each camera delivers a frame within 2 s; `feedback:teleop:<name>`: the teleop poll thread is alive and healthy | |
| 5 `mode` | backend preflight: Franka RT kernel, robot mode and libfranka version; Piper status, follower feedback and firmware profile (pyagxarm) or gripper stroke (piper_sdk) | |

`feedback:gripper` prints the width in metres, its fraction of `max_width_m`, grasp status and
backend source; non-finite values, a fraction outside [0, 1] or `max_width_m <= 0` produce a
`WARN` with a hint to check `max_width_m` and the SDK's width units.

Flags:

- `--mock` swaps every backend for its fake and skips layer 2. Use it to check the YAML and
  the install with no hardware.
- `rwrl doctor --fix` repairs the host and software only; it never recovers or reopens a
  device. A Franka in Reflex/UserStopped/Guiding mode (`mode:robot_mode`) is cleared with
  `rwrl recover`, and a camera that delivers no frames is checked again with `rwrl status`
  after reconnecting it; the device rows' hints say so.
- `--json` prints `rwrl doctor`'s report (`schema_version: 1`, `status`, `checks`,
  `repairs`); the device checks of each robot are under `results` as a list of
  `{layer, name, ok, message, hint, skipped, warning}` objects keyed by robot name.
- `--install-sdk` (with `--yes`: without asking) installs the franky wheel that matches the
  robot after a libfranka version mismatch; see
  [Installing the right franky wheel](doctor.md#installing-the-right-franky-wheel).
- Exit code: `1` if any check failed (on any robot), else `0`. Skipped checks do not fail the run.
  A `WARN` line is an ok check with a hint worth acting on (for example a CH340 adapter that
  brltty may claim on the next replug); it never fails the run.

The doctor connects to the robot and disconnects it again. It never powers anything off.

The device rows of a `--mock` run of the Franka example (`rwrl doctor` prints its host rows
beside them; `rwrl setup` and the daemon preflight print exactly these rows and the result):

```text
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
Result: no blocking findings; review SKIPPED/WARN coverage
```

The same config on a machine with no franky, no robot and no camera (needs-attention block
trimmed):

```text
FAIL           franka.deps: missing extras for this station: franky
    hint: pip install -e '.[franky]' (from the repository root); if the handshake then reports a libfranka mismatch, rerun the doctor with --install-sdk
FAIL           franka.sdk:arm: FrankyArm needs the 'franky' package
    hint: pip install franky-control (or, from the repository root: pip install -e '.[franky]')
FAIL           franka.sdk:gripper: FrankyGripper needs the 'franky' package
    hint: pip install franky-control (or, from the repository root: pip install -e '.[franky]')
PASS           franka.sdk:camera:wrist: 'pyrealsense2' importable
FAIL           franka.os:ping: 172.16.0.2 does not reply
    hint: check: robot powered on and brakes released; Desk reachable at https://172.16.0.2/desk; you hold control in Desk; FCI activated (Desk → Activate FCI); cable on the Control port; firewall allows UDP
PASS           franka.os:tcp: port 1337 closed (informational)
FAIL           franka.os:camera:wrist: no RealSense device found (serial unset)
    hint: plug in the camera and run `rwrl discover`
FAIL           franka.handshake: FrankyArm needs the 'franky' package
    hint: pip install franky-control (or, from the repository root: pip install -e '.[franky]')
SKIPPED        franka.feedback: skipped: handshake failed
SKIPPED        franka.mode: skipped: handshake failed
Needs attention: 6 FAIL
  FAIL           franka.deps: missing extras for this station: franky
    hint: pip install -e '.[franky]' (from the repository root); if the handshake then reports a libfranka mismatch, rerun the doctor with --install-sdk
  ...
Result: unresolved failures / blocked checks / pending user actions
```

With a `teleop: {backend: spacemouse}` section, `--mock` swaps the teleop for the fake one
(its options are dropped) and the extra lines are:

```text
...
PASS           franka.sdk:teleop:fake: no SDK needed
SKIPPED        franka.os:teleop:teleop: skipped (--mock)
...
PASS           franka.feedback:teleop:teleop: teleop poll thread alive
```

Without `--mock`, on a machine without `hidapi` and without a SpaceMouse, the teleop
lines read (the `os:hid` line is only checked for the SpaceMouse backend):

```text
FAIL           franka.sdk:teleop:spacemouse: SpaceMouse needs the 'hidapi==0.14.0.post4' distribution (import hid)
    hint: pip install -e '.[spacemouse]'
FAIL           franka.os:teleop:teleop: no 'spacemouse' teleop device found
    hint: plug in the teleop device and run `rwrl discover`
FAIL           franka.os:hid: no HID device found
    hint: no /dev/hidraw* node: check the USB cable and port; inside Docker pass it with `docker run --device=/dev/hidrawN`
```

When a `/dev/hidraw*` node exists but this user cannot open it, `os:hid` fails with
`no read/write access to ['/dev/hidraw3']` and a hint to install the
[packaged vendor rules](teleop.md#spacemouse-installation-and-linux-permissions) with `sudo cp`,
join `plugdev`, reload the rules, log in again and replug. A missing Python SDK
instead gives the pip command above (NUC, repository root, project environment
active); native import failures report wheel/platform loading separately.

The Piper example with no CAN adapter plugged in (trimmed):

```text
FAIL           piper.os:can_exists: CAN interface can0 not found
    hint: run `bash can_activate.sh can0 1000000` from piper_sdk, or `sudo ip link set can0 up type can bitrate 1000000`
FAIL           piper.os:can_up: can0 operstate=missing
FAIL           piper.os:can_bitrate: can0 bitrate=None, expected 1000000
FAIL           piper.handshake: CAN interface 'can0' not found
    hint: plug in the USB-CAN adapter and run: bash can_activate.sh can0 1000000  (piper_sdk) or: sudo ip link set can0 up type can bitrate 1000000
```

Without pyAgxArm installed, the SDK hint reads `pip install -e '.[pyagxarm]' (from the repository root; pyAgxArm is installed from GitHub)`.
pyAgxArm is not on PyPI, so the extra installs it from GitHub.

### Troubleshooting

The table quotes the messages and hints these checks print. `<ip>`, `can0` and similar
values are filled in from your config.

| Symptom | Where / what you see | What to do |
|---|---|---|
| Missing pip extras | `deps` FAIL `missing extras for this station: franky, gello`, hint `pip install -e '.[franky,gello]' ...` | Run the quoted command from the repository root (the list is exactly the missing extras). |
| Franka unreachable: robot off, brakes locked, no control or FCI in Desk, cable on the Arm port, firewall | `os:ping` FAIL `<ip> does not reply` or `handshake` FAIL `cannot reach Franka at <ip>: ...` (for example `libfranka: Connection to FCI refused. Please install FCI feature or enable FCI mode in Desk.`), hint `check: robot powered on and brakes released; Desk reachable at https://<ip>/desk; you hold control in Desk; FCI activated (Desk → Activate FCI); cable on the Control port; firewall allows UDP` | Work through the hint in order. A Desk page that loads is not enough: you must hold control and FCI must be activated. Plug the PC into the Control box (not the arm base). Allow UDP from the robot through the firewall. |
| libfranka version mismatch | `handshake` FAIL `libfranka mismatch: robot server 10, installed library 9`, hint `the robot server version 10 comes from libfranka's handshake error (authoritative); to cross-check, read the system version in Desk → Settings → System and look it up in the compatibility matrix https://frankarobotics.github.io/docs/compatibility.html; then: rwrl doctor --station <cfg> --install-sdk` | See [Installing the right franky wheel](doctor.md#installing-the-right-franky-wheel). `mode:libfranka_version` shows the installed libfranka once connected. |
| User not in `dialout` / `input` | `os:groups` FAIL `not in group(s) dialout, input; no read/write access to [...]` (WARN when an ACL or udev rule grants access anyway), hint `sudo usermod -aG dialout,input $USER, then log out and back in` | Run the command, then log in again: a running session keeps its old groups. |
| Gello port not readable | `os:serial` FAIL `no read/write access to <port>`, hint `grant this user access now: setfacl -m u:$USER:rw <port> ...` | `--fix` tries the `setfacl` without sudo; otherwise prefix sudo. An ACL is lost on replug; the group or a udev rule is permanent. |
| brltty claims a CH340 adapter | `os:brltty` FAIL `brltty active and a CH340 adapter (USB id 1a86) is attached; brltty has likely claimed <port>` (WARN while the port still exists), hint `sudo systemctl mask --now brltty-udev.service brltty.service, then replug the adapter` | Mask brltty (a `--runtime` mask is lost on reboot), then replug. |
| Slow FTDI reads | `os:latency_timer` WARN `ttyUSB0 latency_timer is 16 ms; 1 ms is recommended for Dynamixel reads`, hint with the udev rule `ACTION=="add", SUBSYSTEM=="usb-serial", DRIVER=="ftdi_sio", ATTR{latency_timer}="1"` | Install the udev rule (the hint names the file and the reload command) and replug. CH340 adapters have no `latency_timer`. |
| No PREEMPT_RT kernel | with `realtime: enforce`: `mode:realtime_kernel` FAIL `realtime: enforce but the kernel is not PREEMPT_RT`, or `handshake` FAIL `libfranka realtime setup failed ...`; with `realtime: ignore` the check passes with `control jitter possible` | Install a PREEMPT_RT kernel and realtime group limits, or use `realtime: ignore`. |
| Franka in Reflex / UserStopped / Guiding | `mode:robot_mode` FAIL `robot mode is Reflex`, hint `release the external activation device / clear errors in Desk, then run rwrl recover` | Release the activation device, then run `rwrl recover`. |
| CAN interface missing / down / wrong bitrate | `os:can_exists`, `os:can_up`, `os:can_bitrate` FAIL (see output above); `handshake` FAIL `CAN interface 'can0' is down, not up` or `runs at N bit/s, expected 1000000` | `sudo ip link set can0 up type can bitrate 1000000` (or piper_sdk's `can_activate.sh can0 1000000`). |
| Piper does not enable (power, e-stop) | `handshake` FAIL `Piper on 'can0' did not enable within 5s`, hint `check that the arm is powered, that 'can0' is up at 1000000 bit/s, and that its emergency stop is released` | Power the arm, release the e-stop, rerun. `mode:piper_status` names the arm status once connected. |
| No CAN feedback (arm in master/teach mode) | `handshake` FAIL `... enabled but sent no joint feedback within 2s`, or `mode:follower` FAIL `no joint feedback`; hint `no CAN feedback: the arm may be in master/teach mode; switch it to follower (slave) mode and power-cycle` | Switch the arm to follower (slave) mode and power-cycle it. |
| Wrong Piper firmware profile (pyagxarm) | `handshake` FAIL `firmware probe ... failed` / `cannot map Piper firmware ...`, hint `set firmware_profile explicitly (default\|v183\|v188\|v189) in the robot config` | Set `firmware_profile` in the yaml. |
| Camera serial not found | `os:camera:<name>` FAIL `camera '<name>' serial '<s>' not found`, hint `set serial; discovered: [...]` (with `serial` unset: `no RealSense device found (serial unset)` or `serial unset and N devices attached; set serial to one of [...]`); `handshake` FAIL `RealSense serial '<s>' for camera '<name>' not attached` | Copy a serial from `rwrl discover`. With more than one camera attached, `serial` is required. |
| Camera gives no frames | `feedback:camera:<name>` FAIL `no frames within 2s`, hint `check USB 3 bandwidth; try another port; rerun rwrl status after reconnecting the camera`; or `RealSense '<s>' failed to start`, hint `check the USB 3 cable/port and udev rules (librealsense scripts/setup_udev_rules.sh)` | Use a USB 3 port and cable (not a hub shared with other cameras), install the librealsense udev rules, reconnect the camera, then run `rwrl status`. |
| Daemon unreachable | client `TransportError: robot daemon unreachable at <addr>` | Start the daemon, check `--bind`, the address and the firewall. |
| Token mismatch | client `TransportError: token mismatch`, hint `pass the same --token to the daemon and the remote config` | Use the same token on both sides. |
| Watchdog stopped the arm | client `DeviceFault: the daemon watchdog stopped the arm after 0.62s without actions (limit 0.5s)`, hint `... call set_control_mode(...) again to resume servoing, and keep the action rate above 1/watchdog_s` | Send actions faster than `1/watchdog_s`, or start the daemon with a larger `--watchdog-s`. Then call `set_control_mode` again. |
| Robot already in use | client `DeviceBusy: device 'franka:172.16.0.2' is claimed by another process`, hint `stop the other process using 'franka:172.16.0.2', or set `claim: false` in the robot config` | Stop the other process on this host that serves this robot through `RobotService` (a daemon or a `LocalClient` setup). Plain `Robot.from_yaml` scripts do not take the claim. To opt out deliberately, set `claim: false` in the yaml. |

### Installing the right franky wheel

The `franky` extra installs the default `franky-control` wheel, which bundles one libfranka
version. A robot whose robot server version differs refuses the handshake, and the doctor
reports `libfranka mismatch: robot server <N>, installed library <M>`. Then:

```bash
rwrl doctor --station robot.yaml --install-sdk         # asks before installing
rwrl doctor --station robot.yaml --install-sdk --yes   # installs without asking
```

The failed `<robot>.handshake` row carries the robot server version from libfranka's handshake
error (authoritative). Desk does not show that number: to cross-check, read the system version
(for example 5.9.2) in Desk → Settings → System and look it up in the
[compatibility matrix](https://frankarobotics.github.io/docs/compatibility.html), which maps
system versions to robot server versions. The doctor then plans one repair,
`PLAN sdk:franky: Install Franky wheel for handshake-confirmed server version <N>`, with the
exact command

```text
<this python> -m pip install franky-control --extra-index-url https://timschneider42.github.io/franky/whl/by-robot-server-version/<N>/ --force-reinstall --no-deps
```

and asks `Apply this plan? [y/N]` once. Only `y` (case-insensitive, surrounding whitespace
ignored) installs, into the interpreter running the doctor and only inside a project
environment; `--install-sdk` without `--fix` runs no other repair. The doctor then inspects
again; its device checks run in a fresh interpreter, so they load the new libfranka. Without
`--yes` and without a terminal to ask on, nothing is installed and the plan is reported
`BLOCKED`. Robots that report different server versions are not installed automatically (one
Python environment holds one franky wheel): the doctor reports `sdk.franky.conflict` as
`BLOCKED`. `--no-deps` assumes franky's dependencies (`numpy`, `websockets`) are already
installed, which installing the `franky` extra first takes care of.

Beside the device checks, `rwrl doctor` checks the host and the project environment and
previews targeted repairs (`--fix` applies host and software repairs, after one
confirmation); the device checks themselves never repair. Run it on the NUC with the project environment active (see
[rwrl](cli.md#robot-host-diagnosis-and-repair)).
