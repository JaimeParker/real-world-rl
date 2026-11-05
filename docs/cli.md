# rwrl: station command line

`rwrl` provides hardware primitives between recording, training and deployment
scripts: inspect the station, save a home pose, return home, operate the gripper,
stop or recover the arm. People and agents use the same commands; `--json`,
`--yes` and exit codes make the built-in primitives usable without interaction.

## Where to run

After installing the `real-world-rl` package and the station's `robot_infra`
dependencies, `rwrl` is available in that environment. It is equivalent to
`python -m real_world_rl.cli`. See [installation](tutorial/01-install.md).
Run all commands below on the **NUC**, from the **repository root**, with the
**project environment active**. Relative station and preset paths are relative
to that directory. Each primitive connects for its invocation and disconnects
afterward, including on failure.

Place global options **before the command**, including `--preset` before `home`.
Use `rwrl --help` for wrapper help; `rwrl doctor --help`, `rwrl setup --help`,
`rwrl daemon --help` and `rwrl supervise --help` display the forwarded CLI's help.

## Global options

| Option | Default and meaning |
|---|---|
| `--station PATH` | Resolve explicit path → `$ROBOT_INFRA_CONFIG` → `local/station.yaml` → `~/.config/robot_infra/station.yaml`. The last location follows `$XDG_CONFIG_HOME` when set. Explicit and environment paths are used as given; the last two are searched for the first existing file. |
| `--robot NAME` | Unset: select the single robot; a multi-robot station requires a name for primitives. |
| `--preset PATH` | Unset: use station home. For `home`, load an existing env preset, either a bare mapping or settings under `real_robot`. Required by `home set`. |
| `--direct` | Off: follow the station's daemon declaration. Force direct access only when its daemon port is unreachable. |
| `--mock` | Off: replace backends with doctor's fake backends for a **direct** connection. Still requires a station file; does not bypass daemon selection. |
| `--json` | Off: built-in primitives emit one JSON result on stdout; diagnostics and motion target/prompt text go to stderr. |
| `--yes` | Off: skip the built-in motion confirmation. The target and maximum delta are still printed. |

The last two options have different scope for forwarded commands; see below.

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
| `status` | Connection path, robot description, arm health, gripper feedback, camera names, teleop name; daemon RTT when remote | Read | No | 0 / 1 / 2 |
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
| `doctor` | Forward to `robot_infra.doctor.cli` | Checks hardware; `--fix` may recover | Native doctor rules | Native status, unchanged |
| `setup` | Forward to `robot_infra.setup.__main__` | Discovery, calibration and doctor | Native wizard questions | Native status, unchanged |
| `daemon` | Forward to `robot_infra.server.daemon` | Preflight and serve hardware for client motion | No wrapper prompt | Native status, unchanged |
| `supervise` | Forward to `robot_infra.supervise.supervisor` | Depends on child command | Depends on child | Native status, unchanged |

Forwarding preserves remaining arguments, output and return status; there is no
primitive JSON envelope or wrapper motion prompt. Options after a forwarded
command belong to that CLI. The wrapper translates only these global options:

| Forwarded command | Global options translated |
|---|---|
| `doctor` | `--station` → `--config`, `--robot`, `--mock`, `--json`, `--yes` |
| `setup` | `--station` → `--from` (existing defaults, **not output path**), `--mock` |
| `daemon` | `--station` → `--config`, `--robot` |
| `supervise` | None; supply supervisor and child options after `supervise` |

Other wrapper globals have no effect on those forwarded CLIs. In particular,
doctor's `--yes` applies only with its `--install-sdk`; supplying it alone is a
doctor usage error. Doctor JSON is its native report, not a `status` object.
See the [device-layer README](../robot_infra/README.md#3-check-with-the-doctor)
for doctor, setup, daemon and supervision behavior.

## Home: resolve, save and move

Home resolution matches `env.reset()`, in this order:

1. Preset `task.reset.home_joints`.
2. Preset `home_joints`.
3. Station `home_joints` (the daemon arm's `home_joints` when remote).

Null values fall through to the next source. No home is an error (exit 1).
`home show` prints the source, home and current joints, `home - q`, and its
maximum absolute value.

To reproduce every mock example on this page, use a separate terminal on the
NUC, at the repository root, with the project environment active. Create `local/`
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

**PyYAML rewrites the preset, so its comments are lost.** For a preset `P`,
the exact previous file bytes are saved to `P.bak` (replacing any older backup)
before atomic replacement. Text output shows the old → new home and warns that
comments are not preserved, naming the backup.

## Typical workflow

On the NUC, repository root, project environment active, use your existing local
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
for reported exceptions. Forwarded commands retain their own statuses and
output rules, including any exceptions handled by the wrapper.

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
home and workspace before issuing it. Apply the forwarded CLI's own rules
when calling doctor, setup, daemon or supervise.
