# 2. Configure the station

`python -m robot_infra.setup` is an interactive wizard. Run it on the robot PC with the arm, the
cameras and the GELLO plugged in and powered. It probes the hardware, asks a few questions,
calibrates the GELLO inline, writes one annotated `station.yaml`, runs the doctor on it and prints
the next commands. The questions are in `robot_infra/setup/wizard.py`; every one has a flag
(`python -m robot_infra.setup --help`).

```bash
python -m robot_infra.setup
```

## What it asks

The transcript below is a `--mock` run (canned hardware, so you can try it anywhere) with the answers
typed after each colon; an empty answer accepts the default in brackets. A real run prints what it
actually found: the Franka hosts that answered on TCP 22, 443 or 1337, the RealSense serials, and
each U2D2 or OpenRB-150 port with the servo ids that answered a broadcast ping.

```text
$ python -m robot_infra.setup --mock --output /tmp/station.mock.yaml
hardware probe (--mock: canned results):
franka hosts:
  172.16.0.2: open ports 22, 443, 1337
  172.16.1.2: open ports 22, 443, 1337
realsense cameras:
  123422070001 (D400)
  123422070002 (D400)
  123422070003 (D400)
gello / dynamixel buses:
  /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_MOCK0-if00-port0: 57600 baud, servo ids [1, 2, 3, 4, 5, 6, 7, 8]
  /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_MOCK1-if00-port0: 57600 baud, servo ids [1, 2, 3, 4, 5, 6, 7, 8]
spacemouse devices:
  none
number of arms (1 or 2) [1]:
arm name(s) [arm]:
-- arm 'arm' (1/1) --
franka hosts that answered:
  1) 172.16.0.2
  2) 172.16.1.2
robot IP of 'arm' (number or IP) [172.16.0.2]:
arm backend (franky | serl_http | fake) [franky]:
gripper backend (franky | serl_http | fake | none) [franky]:
home joints of 'arm' (7 values in rad, or none) [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]:
realsense cameras:
  1) 123422070001 (D400)
  2) 123422070002 (D400)
  3) 123422070003 (D400)
cameras of 'arm': numbers or serials, each optionally =name (e.g. 1=wrist,2=front), or none [123422070001=wrist,123422070002=front,123422070003=front_2]: 1=wrist
teleop device of 'arm' (gello | spacemouse | fake | none) [gello]:
gello / dynamixel buses:
  1) /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_MOCK0-if00-port0: 57600 baud, ids [1, 2, 3, 4, 5, 6, 7, 8]
  2) /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_MOCK1-if00-port0: 57600 baud, ids [1, 2, 3, 4, 5, 6, 7, 8]
GELLO serial port of 'arm' (number or path) [/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_MOCK0-if00-port0]:
GELLO baud rate [57600]:
GELLO joint servo ids (7, in the Panda's joint order) [1,2,3,4,5,6,7]:
GELLO gripper trigger servo id (or none) [8]:
intervention (always | toggle: toggle needs a pedal or keyboard) [always]:
event device for toggle/episode keys (/dev/input/by-id/... or none) [none]:
calibrate the GELLO on /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_MOCK0-if00-port0 now (yes | no) [yes]:
  ... the calibration prompts (below) ...
daemon port of 'arm' [50051]:
wrote /tmp/station.mock.yaml
checking it with the doctor (--mock):
  ... one doctor line per check, then "summary: ..." ...
next steps:
  ...
```

Answer notes:

- **Robot IP, cameras, GELLO port** take a list number or the value itself. A value the probe did not
  find is refused unless you pass `--allow-unprobed`.
- **Cameras** are `SERIAL[=NAME]` or a list number, comma separated. The `NAME` is the observation
  key later used in `obs.rgb` (`rgb_<name>`); this tutorial uses one camera named `wrist`.
- **Intervention.** Keep `always`: the keyboard of the actor machine supplies the toggle
  (`--teleop-event-device keyboard`), and its toggle replaces this setting. `toggle` is for an evdev
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

## Check the safety limits

The wizard writes the Panda joint limits and a servo step cap
(`joint_limits`, `max_delta_per_step: {xyz_m: 0.05, joint_rad: 0.04}`) and asks for the
`safety_box`, suggesting a table-top workspace box. Open `station.yaml` and check the three against
your cell before the first motion: the box is in the robot base frame (`tcp_pose = O_T_EE`, metres)
and must contain the home pose, the object and the bin; `home_joints` must lie inside `joint_limits`.
The daemon clips servo targets to these limits and refuses out-of-range `move_to_*` targets
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

Every training, recording and collection command takes the env settings from one YAML preset
(`--config`): the camera names of the station, the daemon address and the action contract. The
canonical single-arm preset is [`examples/presets/hil_single_arm.yaml`](../../examples/presets/hil_single_arm.yaml).
Copy it next to the repository root and edit the copy:

<!-- tutorial-test: skip -->
```bash
cp examples/presets/hil_single_arm.yaml hil.yaml
```

- `real_robot.remote`: `ROBOT_PC:50051` (the daemon port from `station.yaml`), or `127.0.0.1:50051`
  when the actor runs on the robot PC;
- `obs.rgb`: the camera names of `station.yaml` (`[wrist]`);
- `real_robot.token`: uncomment only if you set `daemon.token` in the station file.

Check it without touching the robot:

```bash
python examples/train_real_world.py hil_serl --config hil.yaml --print-config
```

*You should see* the resolved configuration as JSON, with `env_backend: real_robot` and
`real_robot.action_mode: joint_delta`.

> **joint_abs.** Make a second copy, `hil_abs.yaml`, with `action_mode: joint_abs` and the
> `joint_limits` line uncommented (a list of pairs, which the command line cannot express); the
> tutorial names that copy wherever a `joint_abs` run differs. `joint_abs` also needs `max_delta_per_step.joint_rad`
> in `station.yaml` (the wizard writes it): consecutive `joint_abs` actions can jump across the whole
> joint range.

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

Next: [03 Daemon](03-daemon.md).
