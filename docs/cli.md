# rwrl: station command line

`rwrl` unifies training, recording, deployment and station operations. Its hardware
primitives let you inspect the station, save a home pose, return home, operate the gripper,
stop or recover the arm. People and agents use the same commands; `--json`,
`--yes` and exit codes make the built-in primitives usable without interaction.

## Where to run

After installing the `real-world-rl` package and the station's `robot_infra`
dependencies, `rwrl` is available in that environment. It is equivalent to
`python -m real_world_rl.cli`. See [installation](tutorial/01-install.md).
Run hardware commands on the **robot host** connected to the devices, with the
**project environment active**. A NUC is one option; no particular computer model
is required. Both source and installed-package use are supported. Relative
station and preset paths resolve from the current working directory; a source
checkout is convenient for repository-relative examples. Each primitive connects
for its invocation and disconnects afterward, including on failure.

Place global options **before the command**, including `--preset` before `home` or `doctor`.
Use `rwrl --help` for wrapper help; `rwrl doctor --help`, `rwrl setup --help`,
`rwrl daemon --help` and `rwrl supervise --help` display each CLI's help.

## Global options

| Option | Default and meaning |
|---|---|
| `--station PATH` | Resolve explicit path → `$ROBOT_INFRA_CONFIG` → `local/station.yaml` → `~/.config/robot_infra/station.yaml`. The last location follows `$XDG_CONFIG_HOME` when set. Explicit and environment paths are used as given; the last two are searched for the first existing file. |
| `--robot NAME` | Unset: select the single robot; a multi-robot station requires a name for primitives. |
| `--preset PATH` | Unset: use station home. For `home`, load an existing env preset, either a bare mapping or settings under `real_robot`. Required by `home set`; `doctor` also uses it to identify expected devices and to compare its robot with the station. |
| `--direct` | Off: follow the station's daemon declaration. Force direct access only when its daemon port is unreachable. |
| `--mock` | Off: replace backends with doctor's fake backends for a **direct** connection. Still requires a station file; does not bypass daemon selection. |
| `--json` | Off: built-in primitives emit one JSON result on stdout; diagnostics and motion target/prompt text go to stderr. |
| `--yes` | Off: skip the built-in motion confirmation or doctor repair-plan confirmation. Preconditions still apply. |

Commands without JSON output (`setup`, `daemon`, `supervise`, `gello-calibrate`
and the learning commands) reject `--json` with usage exit code 2; see below.

## Learning commands

`train`, `record`, `deploy` and `collect-reward` call the existing training,
demonstration, deployment and reward-collection entrypoints. Their arguments,
including `--help`, belong to those entrypoints and are passed through unchanged.
For learning commands, global `--preset PATH` prepends `--config PATH`; specifying
both is a usage error. Global `--station`, `--robot`, `--direct`, `--mock`,
`--json` and `--yes` are inapplicable and return exit code 2. Supply each
entrypoint's own flags after the command instead.

From the repository root in the project environment on the training host,
inspect the algorithm-specific parser without starting training with
`rwrl train --help`.

The four existing console scripts, module entrypoints and example scripts
remain available. Training arguments continue to be defined by the training
entrypoints; the rwrl layer does not define or reinterpret them.

## Connection ownership

If the selected station robot declares `daemon:`, `rwrl` probes its bind port
with a 0.5-second TCP timeout and uses the daemon when reachable. A wildcard
`0.0.0.0` bind becomes `127.0.0.1` for the client. Status then starts with
`connection: daemon HOST:PORT`. If the daemon is declared but unreachable,
the default is an error (exit 1), with the hint to start `rwrl daemon` or pass
`--direct`; there is no automatic direct fallback.

Without `daemon:`, access is direct. A Franka station using franky reports
`connection: direct franky`; a mocked direct connection reports `direct fake`.
`--direct` is refused with exit 2 while the declared daemon's port is reachable:
`error: the daemon owns the arm (hint: omit --direct)`.

Franka FCI allows only one control connection. On the direct route, finish the
recorder/actor before opening another hardware connection with `rwrl`. On the
daemon route, use the same declared daemon. `stop` and `recover` follow this
connection selection too; they do not bypass ownership of a directly held arm.

## Commands

“Confirmation” below means the wrapper's motion prompt. Reading a primitive
still opens the configured devices. Codes for built-in primitives are 0 on
success, 1 on runtime/device failure, and 2 for usage/config-selection errors or
refused confirmation; details follow the table.

| Command | Effect | Robot action | Confirmation | Exit codes |
|---|---|---|---|---|
| `status` | Connection path, robot description, arm health, gripper feedback, camera names, teleop name; daemon RTT when remote. Text shows the arm's `ok`, control mode and errors; `arm_health.robot_mode`, `detail`, `last_feedback_age_s` and the safety clip/reject counts appear only with `--json` | Read | No | 0 / 1 / 2 |
| `q` | Current joint positions in radians | Read | No | 0 / 1 / 2 |
| `tcp` | TCP position in metres and quaternion in `xyzw` order | Read | No | 0 / 1 / 2 |
| `home show` | Resolved home, current joints, signed delta and largest absolute delta | Read | No | 0 / 1 / 2 |
| `home set` | Save measured joints into the supplied preset | Read; writes file | No | 0 / 1 / 2 |
| `home go` | Blocking move to home, then configured reset gripper action | Arm and possibly gripper motion | Yes, or `--yes` | 0 / 1 / 2 |
| `gripper show` | Measured width in metres and normalized units, grasp state, backend source | Read | No | 0 / 1 / 2 |
| `gripper open` | Open to normalized width 1 | Gripper motion | Yes, or `--yes` | 0 / 1 / 2 |
| `gripper close` | Close to normalized width 0 | Gripper motion | Yes, or `--yes` | 0 / 1 / 2 |
| `gripper move F` | Set normalized width to finite `F`, with `0 <= F <= 1` | Gripper motion | Yes, or `--yes` | 0 / 1 / 2 |
| `stop` | Call arm `stop()`; success prints `stop: ok` | Stop arm | No | 0 / 1 / 2 |
| `recover` | Call arm `recover()`; success prints `recover: ok` | Clear arm errors | No | 0 / 1 / 2 |
| `camera show` | Every station camera, or only `--name NAME`: configuration, frame rate measured over `--frames N` reads (default 30; `0` reads none and reports "not measured"), frame age, image shapes, dtype and health | Read; opens the robot connection like `status` | No | 0; 1 when a camera has no frames or is unhealthy (the others are still reported); 2 |
| `camera grab --name NAME --output DIR` | Save one frame as `NAME-color.png`, `NAME-depth.png` (when the camera delivers depth; 16-bit, float depth stored as millimetres) and a `NAME.json` sidecar; creates `DIR`, never overwrites a file; `--timeout-s S` (default 2) | Read; writes files | No | 0 / 1 / 2 |
| `doctor` | Inspect host and configured devices; preview targeted repairs | Read; `--fix` repairs host/software, without recovery or motion | One repair-plan confirmation, or `--yes` | 0 / 1 / 2 |
| `setup` | Discover and configure a station | Discovery, calibration and doctor | Native wizard questions | Native status, unchanged |
| `daemon` | Serve robots over the network | Preflight and serve hardware for client motion | No wrapper prompt | Native status, unchanged |
| `supervise` | Supervise a robot process | Depends on child command | Depends on child | Native status, unchanged |
| `teleop read` | Print live samples of the station teleop (GELLO or SpaceMouse): `q` or `twist`, gripper, intervening, fired events; one text line per sample at `--rate-hz` (default 3) until Ctrl-C or `--duration-s`, then a summary line. `--json` prints JSON lines and a closing `{"summary": ...}`. The direct route opens only the teleop part; `--with-follower` (daemon route only) adds the follower `follower_q` and, for `joint_abs` leaders, the per-joint leader-minus-follower `delta` | Input only; no robot motion | No | 0 / 1 (device failure or unhealthy teleop) / 2 |
| `teleop calibrate` | Calibrate the station teleop; `--backend spacemouse\|gello` (default: the station teleop backend). SpaceMouse fits and validates a direction profile (`--profile` required; see [spacemouse-commissioning.md](spacemouse-commissioning.md)); GELLO runs the guided leader calibration, takes unset `--port`, `--joint-ids`, `--gripper-id`, `--baudrate`, `--joint-signs` from station `teleop.options` and the wrap centres from the station `joint_limits`, and prints the `teleop:` YAML block ([gello.md](gello.md#2-calibration)). Flags of the other backend are a usage error | Input only; optional file writes | Save/use confirmation; `--use` selects the profile (SpaceMouse) or writes the calibrated `teleop.options` (GELLO) into the station | 0 / 1 / 2 |
| `teleop profiles list/show/use` | (SpaceMouse) Inspect or select a named profile; see [spacemouse-commissioning.md](spacemouse-commissioning.md) | No motion; use writes station | Save confirmation for use | 0 / 1 / 2 |
| `teleop diagnose` | (SpaceMouse) Input-only diagnosis; see [spacemouse-commissioning.md](spacemouse-commissioning.md) | No robot motion | No | 0 / 1 / 2 |
| `teleop run` | (SpaceMouse) Supervised recording with episode holding; `--diagnose translation\|rotation\|all` enables motion diagnostics; see [spacemouse-commissioning.md](spacemouse-commissioning.md#supervised-recording-sessions) | Reset and teleoperation | Yes, or `--yes` | Child status or wrapper error |
| `workspace show/capture/set/validate` | Inspect, sample or configure constraints; see [spacemouse-commissioning.md](spacemouse-commissioning.md). `show` also reports `joint_limits`, `max_delta_per_step`, `home_joints` and `home_within_limits` (`null` without home or limits; `home_violations` lists each joint outside its limits) | Capture reads TCP; no motion | Save confirmation when applicable | 0 / 1 / 2; text validation failure 5 |
| `workspace frame set --name NAME` | Propose a station `task_frame` from `--base-T-frame` (16 row-major numbers, metres) or from `--from-points PTS --origin A --x-axis B --y-axis C` (point names saved by `workspace capture --save`; right-handed, y orthogonalised to x, degenerate points refused); `--source TEXT` records provenance; `--save` writes only `task_frame`. Refused when the station workspace is in another frame; the same frame name rebases that workspace and says so | No motion; `--save` writes station | Save confirmation | 0 / 1 / 2 |
| `workspace frame show` | Station `task_frame` with its origin and unit axes in the base frame | Reads the station file only | No | 0 / 1 / 2 |
| `teleop inspect-stop` | Evaluate saved stop evidence offline; see [spacemouse-commissioning.md](spacemouse-commissioning.md#live-input-calibration-and-diagnosis) | No hardware connection | No | 0 PASS / 3 FAIL / 4 BLOCKED; errors otherwise |
| `archive inspect ARCHIVE` | Read a recorder SQLite archive read-only: outcomes, mask counts, hold drift, take-over jump and residual per human_action_mask segment, printed as indented JSON. `--csv PATH` and `--measurements PATH` write new step and measurement CSVs; `--drift-window-s` (default 10) sets the hold-drift window; each `--stop-time-s T` (repeatable) adds a settle evaluation using `--settle-speed-mm-s` (default 2) and `--settle-hold-s` (default 0.5). `--json` prints the same fields on one line, non-finite numbers as `null`, plus `status` (`REPORT`, `PASS`, `FAIL` or `BLOCKED`). No station is loaded: `--station`, `--robot`, `--preset`, `--direct`, `--mock` and `--yes` are usage errors. `python examples/inspect_teleop_archive.py` remains an alias with the same arguments and output, but its exit code ignores the settle status | No hardware connection | No | 0 without `--stop-time-s`; with it 0 PASS / 3 FAIL / 4 BLOCKED (a stop time without a spanning TCP episode); 2 for usage or an unreadable archive |

`setup`, `daemon`, `supervise`, `discover` and `gello-calibrate` are native
commands provided by `robot_infra`, also available through `robot-infra`.
Both CLIs use the same registered command objects and implementations.
Station input is `--station PATH`; daemon, discover and the device doctor keep
`--config PATH`, and setup keeps `--from PATH`, as temporary aliases marked
"(alias, will be removed)" in help. For setup, station input supplies existing
defaults, **not the output path**. A command-local value overrides the root
`--station` value; the station lookup order is unchanged.
`--mock` and `--robot` retain their
applicable device meanings. Other unrelated globals do not configure these
commands. `setup`, `daemon`, `supervise` and `gello-calibrate` have no JSON
output: `--json` is a usage error (exit 2), rather than silently producing text.
Supervisor child arguments after `--` belong to the child.

`discover` prints visible hardware without connecting to robot hosts. With
`--json` it prints one object with `schema_version: 1` and five lists, each empty
when nothing is found: `cameras` (`backend`, `serial`), `teleops` (`backend`,
`device`), `can_interfaces` (`name`, `operstate`, `bitrate`), `hosts` (`ip`,
`ping_ok`, `open_ports`) and `hid_devices` (paths). Its text output is unchanged.

`setup` takes three per-arm camera stream flags, one value per arm or one for all
arms: `--camera-resolution WIDTHxHEIGHT` (for example `1280x720`),
`--camera-fps N` and `--camera-depth yes|no`. They apply to that arm's cameras;
unset, new cameras keep the camera defaults and cameras kept from `--station`
keep their values. They add no wizard question.

`rwrl gello-calibrate` is an alias of `rwrl teleop calibrate --backend gello`
and will be removed; it keeps the device command's flags (`--port` and
`--joint-ids` required, no station defaults). `robot-infra gello-calibrate` is
unchanged. Both print the GELLO `teleop:` YAML block.
`setup --only teleop --calibrate` runs that calibration inside the wizard and
writes the station YAML.
`robot-infra doctor` remains the device-layer diagnostic; `rwrl doctor` is the
host and project diagnostic described below.

Doctor is registered directly in the `rwrl` parser; `rwrl doctor --help` displays
rwrl's own doctor help. It shares the global `--station`, `--robot`, `--preset`,
`--mock`, `--json` and `--yes` options. For compatibility, `--config PATH` after
`doctor` aliases `--station PATH`, and `--robot`, `--preset`, `--mock`, `--json`
and `--yes` are also accepted after `doctor`; `--station` is accepted there too.
A trailing value takes precedence
when the same option is supplied before and after the command.
`python -m real_world_rl.cli.doctor` remains a transitional entry point that
delegates to `rwrl doctor`. In both doctors, `--yes` skips confirmation while
all preflight checks still run, including when no repair is requested.
Both JSON reports share `schema_version: 1`, `status` (`PASS` or `FAIL`),
`checks` and `repairs`. These retain rwrl doctor's existing field names.
The device doctor's unchanged list/map (or its install-SDK object) is wrapped
under `results`; its host-only `checks` and `repairs` are empty lists.
See the [device-layer README](../robot_infra/README.md#3-check-with-the-doctor)
for the Python doctor, setup, daemon and supervision interfaces.

## Robot-host diagnosis and repair

Activate the project venv or non-base conda environment on the robot host.
Commands work from a source checkout or an installed package; a repository
root is not required. A fresh machine must first have Python, an environment
and the project installed through [installation](tutorial/01-install.md);
doctor repairs that environment, rather than bootstrapping Python itself.
No station file is required for host and USB discovery:

```bash
rwrl doctor
rwrl --station local/station.yaml --preset hil.yaml doctor
rwrl --station local/station.yaml doctor --fix
```

The first two commands inspect and display a repair plan without applying it.
`--fix` displays commands, dedicated configuration file contents and required
follow-up actions, asks once, applies the approved plan, then inspects again.
Independent repairs continue after a failure; dependent repairs are blocked.
New problems discovered during rechecking require another reviewed plan.

Support is selected per operation, without a machine-model, distribution-version
or CPU-architecture allowlist. Hardware SDKs still need compatible wheels,
drivers and supported devices; detecting an OS does not certify that hardware.

| Host capability | Available behavior |
|---|---|
| Native Linux | Host/USB diagnostics and configured device probes through installed backends. |
| Debian/Ubuntu family with `apt-get` | Missing diagnostic tools and libusb/libudev package repairs. |
| Native Linux with udev, user-group tools, PAM limits or running systemd | Only the corresponding rule, group, limit or proven service-conflict repairs. Other distributions can use these repairs even without apt. |
| Containers, WSL, macOS or Windows | Python metadata/import diagnostics and project-environment Python repairs; native hardware probes and host-system repairs are blocked. |

Unavailable repair capabilities produce an explicit `BLOCKED` reason and manual
next steps; independent supported operations can continue. Package installation
on non-apt distributions remains manual. A missing sibling project distribution
is repaired from a verified local source tree when available; installed-package
users without that source receive a `BLOCKED` instruction to install the matching
distribution from their trusted project package source. Doctor does not invent a
source path or silently substitute a similarly named public package.

Checks cover:

- Missing diagnostic tools and project/device Python dependencies, including
  version constraints from package metadata. Satisfied packages are retained;
  broken SDK imports are repaired at their installed version.
- SpaceMouse USB/libusb access and supported product IDs, loadable system
  libusb/libudev libraries with apt repairs when absent or broken, plus hidraw rules.
  A readable hidraw node alone does not establish USB SDK access.
- Commissioned GELLO serial/event access, effective user groups, targeted FTDI
  latency settings and evidence of a brltty conflict. An uncommissioned serial
  adapter is only a candidate; doctor does not choose its calibration.
- RealSense USB permissions and a frame read, and configured Franka/Hand
  connectivity, SDK handshake, feedback and realtime permissions.
- Kernel realtime status, with a manual **MAJOR CHANGE / DOUBLE CHECK** warning
  when kernel work may be needed. No kernel or boot changes are performed.

Station and preset configuration determine expected devices; recognized
SpaceMouse/RealSense USB devices can also be inspected independently. Missing
configured devices are `BLOCKED`. Unconfigured, unidentified devices are
`SKIPPED`, not accepted as working. Doctor never recovers, homes or commands
robot motion. An active daemon, controller, or device owner blocks exclusive
probes/repairs; doctor does not stop or take over another process.
Only processes that hold a hardware connection count as owners: the daemon,
recorder, actor and deploy entrypoints, `setup`, `gello-calibrate`, the
connecting primitives (`status`, `q`, `tcp`, `home`, `gripper`, `camera`,
`stop`, `recover`), `teleop run/read/calibrate/diagnose`, `workspace capture`
and the learning commands. Offline `rwrl` commands (`--help`, `workspace
show/set/validate/frame`, `teleop profiles`, `teleop inspect-stop`, `archive
inspect`) do not make doctor report `BLOCKED`.

Device rows keep the device doctor's checks, but `rwrl doctor --fix` never
recovers or reopens devices, so their hints name the rwrl command instead:
"--fix (recover)" becomes `run rwrl recover (or robot-infra doctor --fix)`, and
"--fix to reopen" becomes `rerun rwrl status after reconnecting the camera (or
robot-infra doctor --fix)`.

With a station and `--preset`, doctor also compares the preset's robot (the
inline `real_robot.robot` or bare `robot` mapping, or the station file it names)
with the station robot: `robot_ip`, `arm_backend`, `gripper`, cameras by name
(`serial`, `width`, `height`, `fps`, `enable_depth`), `home_joints`,
`joint_limits`, `safety_box`, `task_frame` and `workspace`. Each differing field
is a `WARN` with id `<robot>.preset:<field>` (for example `arm.preset:home_joints`
or `arm.preset:cameras.wrist.fps`) that shows both values; matching fields give
one `PASS` row `<robot>.preset`. WARN does not change the exit code. An inline
robot on a multi-robot station needs `--robot`; otherwise the comparison is
`SKIPPED`.

System repairs use sudo for individual commands when not already root; Python repairs use the running
project interpreter, never sudo pip. Interactive execution can request sudo
authorization. For unattended execution, first authorize sudo in a terminal:

```bash
sudo -v
rwrl --station local/station.yaml --yes --json doctor --fix
```

Only `y` (case-insensitive, surrounding whitespace ignored) confirms a repair.
`--yes` skips plan confirmation, while platform, ownership, environment and
sudo checks still apply. Without a TTY or `--yes`, repairs are refused. Missing
sudo authorization produces `BLOCKED`; passwords are never read from a pipe.
`doctor --install-sdk` selects only a Franky replacement supported by a failed
handshake's server version. Conflicting server versions require separate
environments. It does not select an SDK based on a guessed firmware version.

Repairs can install missing host tools, add user groups, write dedicated
`/etc/udev/rules.d/99-rwrl-*.rules` and current-user realtime limits, and reload
udev rules. Previous managed-file contents are backed up; concurrent changes
are refused. Device replugging and a fresh login may still be required, which
remain `PENDING_ACTION` until independently verified. NIC addresses/routes,
firmware, kernels, boot configuration and reboot are manual operations.

Each run saves credential-redacted diagnostics: `before.json`, `plan.json`,
`report.json` and `commands.log`, plus exact configuration backups/staged files
when needed. The report base directory is selected in this order:

1. `doctor --report-dir PATH`.
2. `$RWRL_DOCTOR_REPORT_DIR`.
3. The source checkout's gitignored `local/doctor/`, when writable.
4. The user state directory: `${XDG_STATE_HOME:-~/.local/state}/rwrl/doctor/`
   on Linux, `~/Library/Application Support/rwrl/doctor/` on macOS, or
   `%LOCALAPPDATA%/rwrl/doctor/` on Windows.

Each run creates a `<UTC timestamp>-<id>/` subdirectory. An explicitly requested
unwritable destination is an error; an unwritable checkout falls back to user
state. Reports are never written into `site-packages`. POSIX report directories
are mode 0700 and files 0600; on Windows, keep the report base private using its
ACLs. Backups retain exact previous bytes, so treat the entire directory as
private. With `--json`, stdout contains one schema-version-1 object with
`status`, `checks`, `repairs`, `remaining_plan`, `report_directory` and per-robot low-level
`results`; diagnostics and prompts use stderr. Argument, configuration and report
creation errors return exit 2 and a JSON error object when requested.
Configuration errors also save `error.json` when a report directory was created.

| Doctor result | Exit code |
|---|---|
| Only `PASS`, `WARN` or `SKIPPED` | 0; inspect coverage before hardware acceptance |
| Any `FAIL`, `BLOCKED` or `PENDING_ACTION` | 1 |
| Invalid arguments or configuration | 2 |

Advanced users can call `real_world_rl.cli.doctor.main(argv)` or retain the
lower-level `robot_infra.doctor` API. The latter has its own repair semantics;
`rwrl doctor` always runs its device probes with `fix=False`.


## Home: resolve, save and move

Home resolution matches `env.reset()`, in this order:

1. Preset `task.reset.home_joints`.
2. Preset `home_joints`.
3. Station `home_joints` (the daemon arm's `home_joints` when remote).

Null values fall through to the next source. No home is an error (exit 1).
`home show` prints the source, home and current joints, `home - q`, and its
maximum absolute value.

To reproduce every mock example on this page, use a separate terminal on the
robot host, at the repository root for these relative-path examples, with the project environment active. Create `local/`
if needed and write your own `local/mock-station.yaml` with this content:

```yaml
type: franka
name: mock_station
arm_backend: fake
claim: false
gripper:
  backend: fake
home_joints: [0.1, 0.0, 0.0, -1.5, 0.0, 1.5, 0.7]
```

Select this file in that terminal before running the mock commands:

```bash
export ROBOT_INFRA_CONFIG=local/mock-station.yaml
```

Run all subsequent mock examples in that same terminal. This configuration
has no cameras, teleop or daemon declaration. Its home display is:

```bash
rwrl --mock home show
```

```text
source: station home_joints
home (rad): [0.1000, 0.0000, 0.0000, -1.5000, 0.0000, 1.5000, 0.7000]
q (rad): [0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000]
delta (home - q, rad): [0.1000, 0.0000, 0.0000, -1.5000, 0.0000, 1.5000, 0.7000]
max delta: 1.500000 rad
```

`home go` runs only reset's home phase: a blocking joint move followed by
`task.reset.gripper` if it is `open` or `close` and a gripper exists. `keep`
leaves the gripper alone. It skips lift, waypoints, jitter and settle; this is
not a complete task reset. The result reports joints and delta after the move.

`home set` requires an existing `--preset PATH`. It reads measured joints,
rounds to four decimal places, and writes `task.reset.home_joints`, creating
missing task/reset mappings. It validates both measured and rounded values
against available station/arm, preset `joint_limits` and preset
`robot.joint_limits`. Out-of-bounds values fail with exit 1 before changing the
preset or its backup. No motion confirmation is requested.

YAML edits preserve comments, quotes, blank lines and key order outside the edited
keys. Before atomic replacement, exact previous bytes are saved to a unique
`P.<id>.bak` beside the preset (or station/profile file). Concurrent edits are
rejected. Text output shows the old → new home and names the backup.

## Typical workflow

On the robot host, repository root, project environment active, use your existing local
`hil.yaml` preset for the same physical arm and limits as the station. Finish
any directly connected recorder/actor first.

```bash
rwrl status
# Manually guide the arm to the desired home using the station's guiding procedure.
# Leave guiding mode before commanding a move.
rwrl --preset hil.yaml home set
rwrl --preset hil.yaml --yes home go
rwrl q
```

For reference, the mock station YAML you wrote above reports the following
when run in the separate mock terminal:

```bash
rwrl --mock status
```

```text
connection: direct fake
robot: mock_station (franka)
arm: FakeArm, ok=True, mode=idle, errors=[]
gripper width 0.0800 m (1.00 of max_width_m 0.080), grasped=unknown, source FakeGripper
cameras: none
teleop: none
```

## Confirmation, exit codes and JSON

`home go` and gripper motion print the target and maximum delta first, even
with `--yes`. Without `--yes`, a TTY prompts `proceed? [y/N] `; only `y`
(case-insensitive, surrounding whitespace ignored) proceeds. Other answers or
EOF cancel with exit 2. Non-TTY input refuses motion with exit 2.
For the same mock station, with non-TTY input:

```bash
rwrl --mock home go
```

Stdout:

```text
target home [0.1000, 0.0000, 0.0000, -1.5000, 0.0000, 1.5000, 0.7000]; max delta 1.5000 rad
```

Stderr:

```text
error: refusing to move without --yes
```

An explicitly confirmed gripper command on that station:

```bash
rwrl --mock --yes gripper close
```

```text
target gripper close 0.0000; max delta 1.0000 norm
gripper width 0.0000 m (0.00 of max_width_m 0.080), grasped=unknown, source FakeGripper
```

| Built-in exit code | Meaning and examples |
|---|---|
| 0 | Command completed (or help displayed). |
| 1 | Runtime/device failure: unreachable declared daemon, missing home/gripper, invalid home joint values, bounds failure or other exception. |
| 2 | Usage/config-selection error: unknown arguments, missing station, missing `--robot` in a multi-robot station, missing `--preset` for `home set`, invalid `gripper move F`, reachable daemon with `--direct`, or refused/cancelled motion. |

Errors print `error: MESSAGE` on stderr, adding ` (hint: HINT)` when a hint
exists. `--json` also prints `{"error": "MESSAGE", "hint": "HINT"}` on stdout
(the hint can be empty). `RWRL_DEBUG=1` additionally prints a traceback to stderr
for reported exceptions. Both binaries use the exit-code constants in
`robot_infra.cli.errors`.
Native device command errors also use stderr. Supervisor child statuses are
preserved; failure to launch returns 127, stop escalation returns 1, and setup
interruption returns 130. Stop-trace results retain 3/4 and text workspace
validation retains 5.

For built-in success, stdout is one JSON object with `command`, `path` and
command-specific fields. The following is a complete JSON example derived
from the fake arm's initial joint state:

```bash
rwrl --mock --json q
```

```json
{"command": "q", "path": "direct fake", "joint_pos": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}
```

## Calling from an agent

Start with `rwrl status` to inspect the connection and hardware. For unattended
built-in primitives, use `--json --yes` for motion, or use read-only commands
with `--json`. Keep global options before the command and inspect the exit
code. Read stderr's `error: … (hint: …)` diagnostics as well as any JSON error;
`--json` reserves stdout for the result, moving SDK chatter, targets and prompts
to stderr. `--yes` authorizes the requested motion; verify the target station,
home and workspace before issuing it. Apply the device command's own rules
when calling doctor, setup, daemon or supervise.

Missing required arguments return exit code 2. With `--json`, the error includes
`required_fields` (CLI flag names, or a missing subcommand destination) alongside
`error` and `hint`. SpaceMouse runtime checks also report missing preset, capture
output/name and sample-file requirements. This describes missing inputs; it does
not authorize a motion or waive confirmation. Calibration's `NEEDS_SAMPLES`
result instead lists missing direction labels and means acquisition is incomplete.

## SpaceMouse commissioning

Calibrate SpaceMouse profiles, review task-frame workspaces, and run supervised
recording and diagnosis sessions with the
[SpaceMouse commissioning guide](spacemouse-commissioning.md).
