# 2. Configure the station

`python -m robot_infra.setup` is an interactive wizard. Run it on the robot PC with the arm, the
cameras and the GELLO plugged in and powered. It probes the hardware, asks a few questions,
calibrates the GELLO inline, writes one annotated `station.yaml`, runs the doctor on it and prints
the next commands. The questions are in `robot_infra/setup/wizard.py`; every one has a flag
(`python -m robot_infra.setup --help`). All commands on this page run on the NUC,
from the repository root, with `.venv` active.

```bash
python -m robot_infra.setup
```

## Select the hardware

All commands on this page run on the NUC, from the repository root, with
`.venv` active (see [01](01-install.md#4-install-in-a-clean-venv)). Stop any
recorder, actor or daemon using the devices before running the wizard.
It is interactive unless `--non-interactive` is supplied. For example:

```bash
python -m robot_infra.setup --arms 1 --arm-names arm --robot-ip 172.16.0.2 --teleop gello --scope local
```

| Flag | Meaning and default when omitted |
|---|---|
| `--arms 1` or `--arms 2` | number of arms; a new interactive station suggests 1 |
| `--arm-names arm` or `--arm-names left right` | names in config; suggestions are `arm` or `left right` |
| `--robot-ip 172.16.0.2` | addresses to probe/select, in arm order |
| `--cameras none` | cameras for an arm; otherwise choose discovered cameras interactively |
| `--cameras SERIAL=wrist` | serial and observation name; use actual discovered serials, one comma-separated value per arm |
| `--teleop gello` | select GELLO; a single value applies to every arm |
| `--no-calibrate` | defer calibration; default is to ask interactively, or skip in noninteractive mode |

Do not use deferred calibration values for motion. The wizard probes Franka hosts,
camera serials and serial buses; choose each device by identity, not discovery order.
*You should see:* an annotated station YAML and doctor results. *If not:* use
[01's failure layers](01-install.md#5-configure-once-then-run-doctor) before proceeding.

Answer notes:

- **Robot IP, cameras, GELLO port** take a list number or the value itself. A value the probe did not
  find is refused unless you pass `--allow-unprobed`.
- **Cameras** are `SERIAL[=NAME]` or a list number, comma separated. The `NAME` is the observation
  key later used in `obs.rgb` (`rgb_<name>`); this tutorial uses one camera named `wrist`.
- **Intervention.** Keep `always` when the actor supplies the keyboard toggle; its event source
  replaces this setting. The target key contract is in [00](00-overview.md#operator-keys). `toggle` is for an evdev
  pedal or keyboard on the robot PC ([gello.md](../gello.md#6-hil-training)).
- **Without `--output` or `--scope`** it asks last where to write
  (`write the station to (local = ./local/station.yaml, user = ~/.config/robot_infra/station.yaml, or a path) [local]:`).
  An existing file is first copied to `station.yaml.bak.<UTC timestamp>`.
- **GELLO gripper.** A new GELLO gripper channel gets `gripper_binarize_threshold: 0.5`: the stored
  gripper action is exactly `+1` (open) or `-1` (closed), the clean labels the discrete gripper heads
  of [07](07-hil-training.md) need.

## The inline GELLO calibration

Answering `yes` to the calibration question runs the same procedure as
`python -m robot_infra.devices.backends.teleops.gello_calibrate` (torque is off, so you move the
leader by hand). In order:

1. For each of the seven joints: move that joint in the robot's positive direction and press Enter.
   It prints `joint N: observed sign +1 (current +1); use it? [Y/n]`. Press Enter to accept; answer `n`
   to keep the current sign. A joint moved by less than 0.1 rad keeps its sign (in a `--mock` run
   nothing moves, so you see `moved less than 0.1 rad; keeping sign +1`).
2. Hold the leader in the calibration pose and press Enter. The prompt prints the pose,
   `[0.0, 0.0, 0.0, -1.5708, 0.0, 1.5708, 0.0]`: all joints at 0 except joint 4 at -90 degrees and
   joint 6 at +90 degrees. It then prints `offset residuals (rad): [...]`. A residual above pi/8
   prints a warning that the leader was not in the pose or a sign is wrong; run the calibration
   again.
3. Hold the trigger fully open, press Enter; hold it fully closed, press Enter. It fails if the
   trigger moved less than 0.05 rad.

If you answer `no`, the yaml gets zero offsets, all `+1` signs and a `# TODO calibrate` comment;
calibrate later with the command below. A failed calibration asks `retry the calibration? [Y/n]`.

```bash
python -m robot_infra.setup --from local/station.yaml --only teleop --calibrate
```

## Where the file lands

The wizard writes `./local/station.yaml` (relative to the directory you run it in; `local/` is
git-ignored) by default (`--scope local`), or `~/.config/robot_infra/station.yaml` with
`--scope user`, or any path with `--output`. The daemon, the doctor and discovery find the file
without `--config`, taking the first of:

1. `$ROBOT_INFRA_CONFIG`
2. `./local/station.yaml`
3. `$XDG_CONFIG_HOME/robot_infra/station.yaml` (default `~/.config/robot_infra/station.yaml`)

`--config <yaml>` always wins. Run all later `robot_infra` commands from the same directory, or set
`ROBOT_INFRA_CONFIG`.

## Multiple arms

Each arm has its own IP, its own Desk page and an independently activated FCI
session. In this example **left is `172.16.0.2`, right is `172.16.0.4`**; verify
against the physical arm before assigning cameras, GELLO ports or a home pose.
A shared switch is supported. Refresh each Desk page, request control on this NUC,
and activate FCI separately on both arms.

On the NUC, repository root, `.venv` active, an interactive two-arm example with
no cameras and calibration deferred is:

```bash
python -m robot_infra.setup --arms 2 --arm-names left right \
    --robot-ip 172.16.0.2 172.16.0.4 --cameras none none \
    --teleop gello --no-calibrate --scope local
```

Select a distinct serial port for each GELLO. Calibrate both before motion. For a
station like the commissioned example, select no gripper for the left arm and a
Franka Hand for the right; a software width reading does not prove a Hand exists.
Never copy one arm's home joints to another. Home is a seven-joint target tied to
that physical robot; a recorded TCP is a reference measurement, not an additional
independent reset target.

*You should see:* `robots.left` and `robots.right` with the intended IPs and
per-arm devices. Check one arm at a time on the NUC, repository root, `.venv` active:

```bash
python -m robot_infra.doctor --config local/station.yaml --robot left
python -m robot_infra.doctor --config local/station.yaml --robot right
```

*If not:* check the identity and FCI session on the failing arm's Desk. A Hand
initialization failure often means the wrong arm's Desk is open or an old page
shows stale state. Refresh the correct IP, confirm control ownership, physical
Hand presence and end-effector power. Hand reinitialization can move the fingers;
it is not a read-only diagnostic. Further bimanual experiment settings are in
[bimanual.md](../bimanual.md).

## Check the safety limits

The wizard writes the Panda joint limits and a servo step cap
(`joint_limits`, `max_delta_per_step: {xyz_m: 0.05, joint_rad: 0.04}`) and asks for the
`safety_box`, suggesting a table-top workspace box. Open `station.yaml` and check the three against
your cell before the first motion: the box is in the robot base frame (`tcp_pose = O_T_EE`, metres)
and must contain the home pose, the object and the bin; `home_joints` must lie inside `joint_limits`.
The device layer clips servo targets to these limits in either process layout and refuses out-of-range `move_to_*` targets
([Safety rules](../../robot_infra/README.md#safety-rules)). The training env can only be tighter than
the yaml. Re-running the wizard with `--from` keeps the values you edited.

## Regenerating and rehearsing

```bash
python -m robot_infra.setup --from local/station.yaml                    # old values are the defaults
python -m robot_infra.setup --from local/station.yaml --only cameras     # redo one section: cameras | teleop | daemon
python -m robot_infra.setup --mock --output /tmp/station.mock.yaml       # rehearse without hardware
```

Use `--output` for a rehearsal so it does not replace your real `local/station.yaml`. For scripted
use see `--non-interactive` in the [robot_infra README](../../robot_infra/README.md#setup-wizard).

## The env preset (`hil.yaml`)

The default workflow records demos and runs the HIL actor **directly on the NUC**.
Use `real_robot.robot`, an embedded robot configuration, with `arm_backend: franky`;
do not start a daemon. `real_robot.remote` is for an actor on another machine
([03](03-daemon.md)). Exactly one of `robot` and `remote` must be set.

The checked-in [single-arm preset](../../examples/presets/hil_single_arm.yaml)
still describes the older remote, 10 Hz joint-delta workflow. Copy it on the NUC,
repository root, `.venv` active, then edit the copy before running anything:

<!-- tutorial-test: skip -->
```bash
cp examples/presets/hil_single_arm.yaml hil.yaml
```

For the direct, cartesian workflow:

- Remove `real_robot.remote` and `real_robot.token`.
- Under `real_robot.robot`, copy the single-arm station's robot mapping, including
  `type: franka`, device options, calibrated `teleop`, home and safety limits.
  Exclude the station's `daemon` block. For a multi-arm station take each named
  robot's mapping separately, following the bimanual guide.
- Set `real_robot.action_mode: cartesian_delta` and `real_robot.control_hz: 50`.
  Include `tcp_pose` in `real_robot.state_fields` for cartesian teleoperation.
- Set `obs.rgb` to the station's camera names (`[wrist]` for the worked example),
  or `[]` for no cameras. Keep `obs.image_size: [128, 128]` with images.
- With a physical Hand, retain `gripper_pos` and `gripper_in_action: true`.
  Without one, use `robot.gripper: null`, `gripper_in_action: false` and omit
  `gripper_pos` from the state fields.

`robot` is a mapping, not a path to `station.yaml`. Keep the copied hardware
settings synchronized when the station changes. Configure GELLO's cartesian
mapping through the `leader_cartesian` source; the raw GELLO reports joint
positions and cannot be substituted directly for a cartesian twist source.
The target terminal key path is pending implementation (see
[00](00-overview.md#operator-keys)); do not interpret a successful config parse
as verification of that runtime path.

Check without touching hardware, on the NUC, repository root, `.venv` active:

```bash
python examples/train_real_world.py hil_serl --config hil.yaml --print-config
```

*You should see:* JSON with `env_backend: real_robot`, a `real_robot.robot`
mapping, no remote address, `action_mode: cartesian_delta` and `control_hz: 50`.
*If not:* check YAML indentation and ensure the old remote connection and
joint-delta settings were removed. The examples on later tutorial pages still
need synchronization to this target workflow; keep one action/observation
contract across recording, BC/ACT, HIL and deployment.

## Task, images and control profiles

The tutorial preset leaves `real_robot.task.name` at `none`: the operator or classifier supplies
success labels. The task/reset, depth and per-camera preprocessing options are described in
[Real-world experiments](../real-world-experiments.md#task-presets-and-observation-settings).
Set them before recording demos and keep them consistent through BC, HIL and deployment.
`preprocess.<camera>.roi` is `[top, left, height, width]` in pixels and requires the actual
`source_resolution: [height, width]`. RGB resizing is bilinear; depth is one-channel metres,
uses nearest-neighbor resizing and encodes invalid values as zero. Changing those settings
changes the policy's observation contract.

Station YAML accepts an optional per-arm `control:` impedance profile; `real_robot.control`
can override it for an experiment. Configure while IDLE. Overrides may only tighten declared
torque-slew and error-clip bounds, and the installed SDK must support the requested keywords.
See [Arm control profiles](../../robot_infra/docs/control-profiles.md) for fields, effective
settings and the wizard's `--control` option. The wizard preserves existing profiles and
uses `control: null` for a new station. Choose gains for the actual tool and payload.

Use `real_robot.control_hz` as the robot's control frequency. A nondefault global
`--control-hz` can fill an otherwise default robot frequency, but conflicting nondefault
global and environment frequencies are rejected. Record, train and deploy at the resolved
frequency; it is part of the experiment contract.

Next: [03 Process layout and optional daemon](03-daemon.md).
