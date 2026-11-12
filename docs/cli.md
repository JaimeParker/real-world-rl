# rwrl: station command line

`rwrl` unifies training, recording, deployment and station operations. People and agents use
the same commands; `--json`, `--yes` and exit codes make the built-in primitives usable
without interaction. This page is the reference; the [tutorial](tutorial/00-overview.md)
walks through the commands in order.

After installing the `real-world-rl` package with the station's device extras, `rwrl` is
available in that environment ([installation](tutorial/01-install.md)); it is equivalent to
`python -m real_world_rl.cli`. `train`, `record`, `deploy`, `collect-reward`, `teleop run` and
`archive inspect` also need the `learning` extra; without it they exit 2 with the hint
`pip install -e ./rl-garden -e '.[learning]'`. Run hardware commands on the **robot host**
connected to the devices, with the **project environment active**. Relative station and preset
paths resolve from the current directory. Each primitive connects for its invocation and
disconnects afterwards, including on failure.

Place global options **before the command**, including `--preset` before `home` or `doctor`.
`rwrl --help` lists the commands; `rwrl <command> --help` shows one command's flags.

## Global options

| Option | Default and meaning |
|---|---|
| `--station PATH` | Unset: the [station lookup](#station-lookup). |
| `--robot NAME` | Unset: the single robot; a multi-robot station requires a name for primitives. |
| `--preset PATH` | Unset: use the station home. For `home`, an env preset (a bare mapping or settings under `real_robot`); required by `home set`. `doctor` uses it to find expected devices and to compare its robot with the station. For learning commands it becomes `--config PATH`. |
| `--direct` | Off: follow the station's daemon declaration. Force direct access only when its daemon port is unreachable. |
| `--mock` | Off: fake backends for a **direct** connection. Still requires a station file; does not bypass daemon selection. |
| `--json` | Off: built-in primitives print one JSON result on stdout; diagnostics, motion targets and prompts go to stderr. |
| `--yes` | Off: skip the motion confirmation or the doctor repair-plan confirmation. Preconditions still apply. |

Commands without JSON output (`setup`, `daemon`, `supervise` and the learning commands) reject
`--json` with exit code 2. `doctor` also accepts `--station`, `--robot`, `--preset`, `--mock`,
`--json` and `--yes` after the command; a value after the command wins.

## Station lookup

Every command that reads a station takes the first of:

1. `--station PATH` (also `rwrl setup --station`, `rwrl daemon --station`, `rwrl discover --station`);
2. `$RWRL_STATION`;
3. `./local/station.yaml`;
4. `$XDG_CONFIG_HOME/rwrl/station.yaml` (default `~/.config/rwrl/station.yaml`).

Explicit and environment paths are used as given. For one release the pre-0.1.0 names are still
read after their replacements, `$ROBOT_INFRA_CONFIG` after `$RWRL_STATION` and
`~/.config/robot_infra/station.yaml` last; `rwrl doctor` reports either as a `WARN` with the
command that moves it. For `setup`, `--station` supplies the existing defaults, **not** the
output path (`--scope local|user` or `--output PATH`).

## Learning commands

`train`, `record`, `deploy` and `collect-reward` call the training, demonstration, deployment
and reward-collection entrypoints. Their arguments, including `--help`, belong to those
entrypoints and are passed through unchanged; `rwrl train --help` shows the training parser.
Global `--preset PATH` prepends `--config PATH`; giving both is a usage error. Global
`--station`, `--robot`, `--direct`, `--mock`, `--json` and `--yes` are inapplicable and return
exit code 2. The console scripts, module entrypoints and example scripts remain available.

## Connection ownership

If the selected station robot declares `daemon:`, `rwrl` probes its bind port with a 0.5-second
TCP timeout and uses the daemon when reachable (a wildcard `0.0.0.0` bind becomes `127.0.0.1`).
`status` then starts with `connection: daemon HOST:PORT`. A declared but unreachable daemon is
an error (exit 1) with the hint to start `rwrl daemon` or pass `--direct`; there is no automatic
direct fallback.

Without `daemon:`, access is direct: a Franka station using franky reports
`connection: direct franky`, a mocked one `direct fake`. `--direct` is refused with exit 2 while
the declared daemon's port is reachable: `error: the daemon owns the arm (hint: omit --direct)`.

Franka FCI allows one control connection. On the direct route, finish the recorder or actor
before opening another hardware connection with `rwrl`; on the daemon route, use the same
daemon. `stop` and `recover` follow this selection too; they do not bypass ownership of a
directly held arm.

## Commands

"Confirm" means the wrapper's motion prompt (`--yes` skips it). Unless noted, built-in commands
exit 0 on success, 1 on a runtime or device failure and 2 on a usage or configuration error or a
refused confirmation ([exit codes](#confirmation-exit-codes-and-json)).

**Station primitives** (each opens the robot connection)

| Command | Effect | Confirm |
|---|---|---|
| `status` | connection path, robot, arm health, gripper feedback, camera and teleop names; daemon RTT when remote. `--json` adds `arm_health` details and safety clip/reject counts | no |
| `q` | joint positions in radians | no |
| `tcp` | TCP position in metres and quaternion in `xyzw` | no |
| `home show` | resolved home, current joints, `home - q` and its largest value | no |
| `home set` | save the measured joints into `--preset` ([home](#home-resolve-save-and-move)) | no |
| `home go` | blocking move to home, then the preset's reset gripper action | yes |
| `gripper show` | width in metres and normalized, grasp state, backend | no |
| `gripper open` / `close` / `move F` | normalized width 1, 0, or `F` with `0 <= F <= 1` | yes |
| `stop` | arm `stop()`; prints `stop: ok` | no |
| `recover` | arm `recover()`; prints `recover: ok` | no |
| `camera show [--name NAME] [--frames N]` | configuration, frame rate over N reads (default 30), frame age, shapes and health; exit 1 when a camera has no frames | no |
| `camera grab --name NAME --output DIR` | one frame as `NAME-color.png`, `NAME-depth.png` (with depth) and `NAME.json`; never overwrites; `--timeout-s` (default 2) | no |

**Diagnosis and configuration**

| Command | Effect |
|---|---|
| `doctor` | inspect the host and the station's devices; `--fix` repairs host and software after one confirmation. Reference: [rwrl doctor](doctor.md) |
| `setup` | the station wizard: discover, ask, calibrate GELLO, write the station, run the doctor ([tutorial 02](tutorial/02-setup.md)) |
| `discover` | print visible hardware without connecting to robot hosts ([details](#discover)) |
| `daemon` | serve the station's robots over the network ([remote actor](remote-actor.md)) |
| `supervise -- COMMAND` | run and supervise a robot process with heartbeat and stop escalation |

`setup`, `daemon`, `supervise` and `discover` are native device-layer commands
(`real_world_rl.cli.device_commands`); their exit codes are their own (see below), and
`setup`, `daemon` and `supervise` have no JSON output. Supervisor arguments after `--` belong to
the child.

**Teleop**

| Command | Effect | Exit codes |
|---|---|---|
| `teleop read` | live samples of the station teleop ([details](#teleop-read)) | 0 / 1 (device failure, unhealthy teleop) / 2 |
| `teleop calibrate` | calibrate the station teleop; `--backend spacemouse\|gello` ([details](#teleop-calibrate)) | 0 / 1 / 2 |
| `teleop profiles list/show/use` | SpaceMouse direction profiles ([guide](spacemouse-commissioning.md)) | 0 / 1 / 2 |
| `teleop diagnose` | SpaceMouse input-only diagnosis ([guide](spacemouse-commissioning.md)) | 0 / 1 / 2 |
| `teleop run` | supervised recording with episode holding; `--backend gello` uses the GELLO cartesian leader; asks before reset ([guide](spacemouse-commissioning.md#supervised-recording-sessions)) | child status or wrapper error |
| `teleop probe-hid` | record a SpaceMouse-like HID device for the site device table ([details](#teleop-probe-hid)) | 0 / 1 (no such device, busy) / 2 |
| `teleop inspect-stop --trace FILE` | evaluate a saved stop trace offline ([guide](spacemouse-commissioning.md#live-input-calibration-and-diagnosis)) | 0 PASS / 3 FAIL / 4 BLOCKED |

**Workspace and archives** (these read the station file only, except `workspace capture`)

| Command | Effect | Exit codes |
|---|---|---|
| `workspace show` | constraints, `joint_limits`, `max_delta_per_step`, `home_joints`, `home_within_limits` (`null` without home or limits; `home_violations` per joint) | 0 / 1 / 2 |
| `workspace capture --name N --output PTS --save` | record the current TCP as a named point; no motion | 0 / 1 / 2 |
| `workspace set` / `validate` | propose or save a workspace; check a pose ([guide](spacemouse-commissioning.md#spacemouse-profiles-and-task-frame-workspace)) | 0 / 1 / 2; text validation failure 5 |
| `workspace frame show` / `frame set` | the station `task_frame` ([details](#workspace-frame-set)) | 0 / 1 / 2 |
| `archive inspect ARCHIVE` | read a recorder SQLite archive offline ([details](#archive-inspect)) | 0; with `--stop-time-s`: 0 PASS / 3 FAIL / 4 BLOCKED; 2 |

**Learning:** `train`, `record`, `deploy`, `collect-reward` ([above](#learning-commands)).

### Command details

#### discover

`--json` prints one object with `schema_version: 1` and six lists, each empty when nothing is
found: `cameras` (`backend`, `serial`), `teleops` (`backend`, `device`), `can_interfaces`
(`name`, `operstate`, `bitrate`), `hosts` (`ip`, `ping_ok`, `open_ports`; the station's robot
IPs), `hid_devices` (paths) and `unknown_hid` (`vid`, `pid`, `product`, `path`, `message`; see the
[site device table](spacemouse-commissioning.md#site-device-table)). The text output adds a
`warnings:` section only when `unknown_hid` is not empty.

#### teleop operations

The operations other than `read` and `inspect-stop` run in the device slice of a teleop backend
and follow the station teleop backend (`calibrate` and `run` also take `--backend`, and need it when the
station has no teleop). An operation the backend does not implement is a usage error naming the
backends that do, for example `backend 'gello' does not implement teleop diagnose`.
`probe-hid` probes the host's HID devices whatever the station teleop is.
`rwrl teleop <operation> --help` lists the flags grouped by backend.

#### teleop read

Prints `q` or `twist`, gripper, intervening and fired events, one line per sample at `--rate-hz`
(default 3) until Ctrl-C or `--duration-s`, then a summary line; `--json` prints JSON lines and
a closing `{"summary": ...}`. The direct route opens only the teleop part. `--with-follower`
(daemon route only) adds the follower `follower_q` and, for `joint_abs` leaders, the per-joint
leader-minus-follower `delta`. No robot motion.

#### teleop calibrate

SpaceMouse fits and validates a direction profile (`--profile` required). GELLO runs the guided
leader calibration, takes unset `--port`, `--joint-ids`, `--gripper-id`, `--baudrate` and
`--joint-signs` from the station `teleop.options` and the wrap centres from the station
`joint_limits`, and prints the `teleop:` YAML block ([gello.md](gello.md#2-calibration)). `--use`
selects the profile (SpaceMouse) or writes the calibrated `teleop.options` into the station
after a reviewed diff (GELLO). Flags of the other backend are a usage error.
`setup --only teleop --calibrate` runs the same calibration inside the wizard.

#### teleop probe-hid

Records an attached 3Dconnexion HID device (or a Logitech multi-axis controller): its report
descriptor and `--duration-s` (default 10) of raw input reports, into a new
`<station dir>/teleop/probes/<vid>-<pid>/` (`--output DIR` instead; required without a station;
an existing directory is a usage error) holding `descriptor.hex`, `reports.jsonl` and
`candidate.yaml`, a draft device-table entry. `--device VID:PID` picks one of several devices.
It never writes the device table and never records the serial number.

#### workspace frame set

`--name NAME` with either `--base-T-frame` (16 row-major numbers, metres) or
`--from-points PTS --origin A --x-axis B --y-axis C` (point names saved by
`workspace capture --save`; right-handed, y orthogonalised to x, degenerate points refused).
`--source TEXT` records provenance; `--save` writes only `task_frame` after a reviewed diff. It
is refused when the station workspace is in another frame; the same frame name rebases that
workspace and says so.

#### archive inspect

Reads the archive read-only and prints indented JSON: outcomes, mask counts, hold drift, and
take-over jump and residual per `human_action_mask` segment. `--csv PATH` and
`--measurements PATH` write new step and measurement CSVs; `--drift-window-s` (default 10) sets
the hold-drift window; each `--stop-time-s T` (repeatable) adds a settle evaluation with
`--settle-speed-mm-s` (default 2) and `--settle-hold-s` (default 0.5). `--json` prints the same
fields on one line, non-finite numbers as `null`, plus `status` (`REPORT`, `PASS`, `FAIL` or
`BLOCKED`; `BLOCKED` is a stop time without a spanning TCP episode). No station is loaded:
`--station`, `--robot`, `--preset`, `--direct`, `--mock` and `--yes` are usage errors.
`python examples/inspect_teleop_archive.py` is an alias whose exit code ignores the settle
status.

## Robot-host diagnosis and repair

`rwrl doctor` checks the host, the project environment and every station robot, previews
targeted repairs and applies them with `--fix`. Its checks, statuses, repairs, report directory,
JSON and exit codes are on its own page: [rwrl doctor](doctor.md).

## Home: resolve, save and move

Home resolution matches `env.reset()`, in this order:

1. Preset `task.reset.home_joints`.
2. Preset `home_joints`.
3. Station `home_joints` (the daemon arm's `home_joints` when remote).

Null values fall through to the next source; no home is an error (exit 1). `home go` runs only
reset's home phase: a blocking joint move, then `task.reset.gripper` if it is `open` or `close`
and a gripper exists (`keep` leaves it alone). It skips lift, waypoints, jitter and settle.

`home set` requires an existing `--preset PATH`. It reads the measured joints, rounds them to
four decimals and writes `task.reset.home_joints`, creating missing task/reset mappings. Both
measured and rounded values are checked against the station/arm joint limits and the preset's
`joint_limits` and `robot.joint_limits`; out-of-bounds values fail with exit 1 before anything is
written. YAML edits (here and for station and profile files) preserve comments, quotes, blank
lines and key order; the exact previous bytes are saved to a unique `<file>.<id>.bak` beside the
file first, and concurrent edits are rejected. The worked sequence is
[tutorial 04](tutorial/04-first-motion.md#1-home).

## Confirmation, exit codes and JSON

`home go` and gripper motion print the target and the largest change first, even with `--yes`.
Without `--yes`, a terminal prompts `proceed? [y/N] `; only `y` (any case, surrounding
whitespace ignored) proceeds. Other answers or end of input cancel with exit 2; non-terminal
input refuses motion with exit 2 (`error: refusing to move without --yes`).

| Built-in exit code | Meaning and examples |
|---|---|
| 0 | Command completed (or help displayed). |
| 1 | Runtime/device failure: unreachable declared daemon, missing home/gripper, invalid home joint values, bounds failure or other exception. |
| 2 | Usage/config-selection error: unknown arguments, missing station, missing `--robot` in a multi-robot station, missing `--preset` for `home set`, invalid `gripper move F`, reachable daemon with `--direct`, or refused/cancelled motion. |

Errors print `error: MESSAGE` on stderr, adding ` (hint: HINT)` when a hint exists. `--json` also
prints `{"error": "MESSAGE", "hint": "HINT"}` on stdout (the hint can be empty).
`RWRL_DEBUG=1` additionally prints a traceback to stderr. The exit-code constants are in
`real_world_rl.cli.errors`. Native command errors also use stderr. Supervisor child statuses are
preserved; a failed launch returns 127, stop escalation 1 and an interrupted setup 130.
Stop-trace results keep 3/4 and text workspace validation keeps 5. Doctor's codes are on
[its page](doctor.md#exit-codes).

For built-in success, stdout is one JSON object with `command`, `path` and command-specific
fields; for example, on the mock station of [tutorial 04](tutorial/04-first-motion.md):

```text
$ rwrl --station local/mock-station.yaml --mock --json q
{"command": "q", "path": "direct fake", "joint_pos": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}
```

## Calling from an agent

Start with `rwrl status` to inspect the connection and hardware. For unattended built-in
primitives, use `--json --yes` for motion, or read-only commands with `--json`. Keep global
options before the command and inspect the exit code. Read stderr's `error: … (hint: …)` as well
as any JSON error; `--json` reserves stdout for the result. `--yes` authorizes the requested
motion: verify the station, home and workspace before issuing it. Apply the device command's own
rules when calling doctor, setup, daemon or supervise.

Missing required arguments return exit code 2. With `--json`, the error includes
`required_fields` (CLI flag names, or a missing subcommand destination) alongside `error` and
`hint`. SpaceMouse runtime checks also report missing preset, capture output/name and
sample-file requirements. This describes missing inputs; it does not authorize motion or waive
confirmation. Calibration's `NEEDS_SAMPLES` result instead lists missing direction labels.

## SpaceMouse commissioning

Profiles, task-frame workspaces and supervised recording and diagnosis sessions are in the
[SpaceMouse commissioning guide](spacemouse-commissioning.md).
