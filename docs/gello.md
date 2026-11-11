# GELLO leader arm: hardware to HIL training

A GELLO is a passive, 3D-printed copy of the Franka Panda whose joints are Dynamixel servos read
as absolute joint angles. This guide goes from the bench to human-in-the-loop training:
hardware and permissions, calibration, the robot yaml, demo recording and the `hil_serl`,
`residual_hil_serl` and `silri` commands. The teleop source semantics (device table, onset
policies, the `info` keys of the intervention wrapper) are in [teleop.md](teleop.md); demo
recording has its own guide, [recording-demos.md](recording-demos.md). For a first-day walkthrough
that uses the setup wizard (it runs the calibration of section 2 inline and writes the yaml of
section 3) and continues through BC, ACT, HIL training and deployment, see the
[tutorial](tutorial/00-overview.md).

The hardware design comes from GELLO (MIT license, Philipp Wu,
https://github.com/wuphilipp/gello_software). `robot_infra`'s `gello` teleop backend is a
re-implementation on the ROBOTIS `dynamixel_sdk`; no GELLO code is used.

Which path to use:

| Follower stack | Action mode | Teleop device | GELLO plugged into |
| --- | --- | --- | --- |
| `real_robot` (robot_infra, franky) | `joint_delta` (the tutorial's, for every stage) or `joint_abs`; demos, base policy and HIL runs must share one | `--teleop-device leader_arm` (the robot daemon serves the GELLO as the robot's `teleop:` part) | the robot PC |
| `real_robot` | same | `--teleop-device gello` (`GelloTeleopSource`, opened locally) | the actor's machine |
| `franka_real` (SERL HTTP bridge, cartesian only) | `cartesian_delta` | `--teleop-device leader_cartesian` (Panda forward kinematics) | the robot host, through a stub-robot daemon |

## 1. Hardware

- **Bus adapter.** A ROBOTIS U2D2 (FTDI chip, shows up as
  `/dev/serial/by-id/usb-FTDI_*`, `ttyUSB*`) or an OpenRB-150 running a USB-to-Dynamixel
  pass-through sketch (`/dev/serial/by-id/usb-ROBOTIS_*`, `ttyACM*`). Always configure the stable
  `/dev/serial/by-id/...` path; `ttyUSB0` changes with the plug order.
- **Servo ids and baud rate.** Joints 1-7 and the trigger as id 8 are the defaults used below.
  The backend opens the port at `baudrate` (default 57600, the XL330 factory setting) and never
  writes it to the servos' EEPROM: if you changed the servo baud rate with Dynamixel Wizard (for
  example to 1 Mbps for faster reads), set `baudrate:` to match. One sync read of 8 servos takes
  about 35 ms at 57600 baud, which suits a 10 Hz control loop; raise the baud rate for faster
  loops.
- **FTDI latency timer (U2D2).** The FTDI driver buffers serial data for 16 ms by default, which
  adds up to 16 ms to every read. Set it to 1 ms:

  ```bash
  cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer          # 16 by default
  echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
  # persistent, /etc/udev/rules.d/99-ftdi-latency.rules:
  # ACTION=="add", SUBSYSTEM=="usb-serial", DRIVER=="ftdi_sio", ATTR{latency_timer}="1"
  ```

  The OpenRB-150 is a USB CDC device and has no latency timer.
- **Permissions.** The user needs the `dialout` group (`sudo usermod -aG dialout $USER`, then
  log in again) or a udev rule; in Docker pass the node with `--device=/dev/serial/by-id/...`.
  `rwrl doctor --station robot.yaml` checks that the configured port exists and is
  readable and writable (`os:serial`). The backend never kills processes or changes permissions.
- **Power.** The servos must be powered to be read (a U2D2 does not power the Dynamixel bus from
  USB; use a power hub or the OpenRB-150's supply). The backend writes torque off to every servo
  when it connects and again when it closes, and never enables torque: the leader is moved by
  hand only.

> **Warning, any torque-on use.** Enabling torque on the GELLO servos (Dynamixel Wizard tests,
> or the future leader tracking in section 9) makes the leader move on its own. Use the supply
> voltage the servos are rated for (XL330: 5 V nominal; never a 12 V supply), a supply that can
> deliver the stall current of all servos at once, low current or PWM limits, and keep hands and
> cables clear. Torque-on behaviour of the XL330s on this leader has not been verified.

Install the backend on the machine the GELLO is plugged into:

```bash
pip install -e "robot_infra[gello]"          # dynamixel-sdk
pip install -e "robot_infra[gello-events]"   # evdev, only for an event_device (pedal/keyboard) on that machine
```

## 2. Calibration

The calibration CLI finds the per-unit joint signs, the joint offsets and the trigger range, and
prints the `teleop:` yaml block. (`rwrl setup` runs these steps inline per arm and
writes the result into the station yaml, so with the wizard you can skip the paste below; see the
[robot_infra README](../robot_infra/README.md#setup-wizard).)

```bash
rwrl gello-calibrate \
    --port /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_XXXXXXXX-if00-port0 \
    --joint-ids 1 2 3 4 5 6 7 --gripper-id 8 \
    --joint-limits-yaml robot.yaml
```

1. **Sign check.** For each joint the CLI asks you to move that joint in the robot's positive
   direction and press Enter, shows the observed sign and takes it unless you answer `n`. A joint
   moved by less than 0.1 rad keeps its current sign (`--joint-signs` gives the initial signs,
   default all `+1`; `--skip-sign-check` takes them as they are).
2. **Offsets.** Hold the leader in the calibration pose and press Enter. The default pose is the
   Franka pose `0 0 0 -pi/2 0 pi/2 0`; pass `--calibration-pose` for another one. Each offset is
   solved on the quarter-turn grid (servo horns mount in 90 degree steps); a residual above pi/8 is
   reported and means the leader was not in the pose or a sign is wrong.
3. **Trigger.** Hold the trigger fully open, press Enter, then fully closed, press Enter.
4. **Wrap centres.** With `--joint-limits-yaml` (the robot yaml with `joint_limits`), each joint
   wraps around the midpoint of its limits; without it the centres are zeros and the CLI warns
   (wrong for Panda joint 6, whose range reaches past pi).

Paste the printed block into the robot yaml (section 3). Every option, with its default:

```yaml
teleop:
  name: gello
  backend: gello
  hold_s: 0.0
  options:
    port: /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_XXXXXXXX-if00-port0
    baudrate: 57600                 # must match the servos; never written to them
    joint_ids: [1, 2, 3, 4, 5, 6, 7]
    joint_offsets_rad: [...]        # from the calibration CLI
    joint_signs: [...]              # from the calibration CLI
    wrap_centers_rad: [...]         # from the calibration CLI (midpoints of joint_limits)
    gripper_id: 8                   # null: no gripper channel (the actor holds the last gripper action)
    gripper_open_rad: ...           # raw trigger angle, released
    gripper_closed_rad: ...         # raw trigger angle, squeezed
    gripper_binarize_threshold: 0.5 # 1/0 width instead of a continuous one; see section 7
    intervention: always            # always: every poll intervenes | toggle: follows the toggle key of event_device
    event_device: null              # /dev/input/by-id/...-event-kbd (evdev, on this machine)
    event_keys: {toggle: KEY_SPACE, episode_end: KEY_ESC, success: KEY_S, failure: KEY_F, rerecord: KEY_R}
    poll_interval_s: 0.0
```

Check the result before using it on the robot: read the leader with the daemon running (or the
`teleop` part of `Robot.from_yaml`) in the calibration pose and in a few other poses and compare
the joint angles with the follower's.

## 3. The robot yaml (`real_robot`, franky)

Start from `robot_infra/docs/robot.franka.example.yaml` and add the safety limits and the GELLO. The
alternative is `rwrl setup`, which writes a station yaml with the robot, cameras,
GELLO calibration and daemon port (`./local/station.yaml`, found by the daemon without `--config`);
it writes Panda `joint_limits`, a `max_delta_per_step` cap and a requested `safety_box`.
Inspect those values against the actual station before motion; hand-written files need the
same limits:

```yaml
type: franka
name: franka
arm_backend: franky
robot_ip: 172.16.0.2
home_joints: [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
joint_limits:
  - [-2.8, 2.8]
  - [-1.7, 1.7]
  - [-2.8, 2.8]
  - [-3.0, -0.1]
  - [-2.8, 2.8]
  - [0.0, 3.7]
  - [-2.8, 2.8]
max_delta_per_step: {xyz_m: 0.05, joint_rad: 0.04}   # recommended joint_rad: 0.03 to 0.05
gripper:
  backend: franky
  mode: binary
  binary_threshold: 0.5
cameras:
  - {name: wrist, backend: realsense, width: 640, height: 480, fps: 30}
teleop:
  name: gello
  backend: gello
  options: {...}                    # the calibration block of section 2
```

- `max_delta_per_step.joint_rad` caps every servo target at that distance from the measured
  joints. Keep it at 0.03 to 0.05 rad per step (0.3 to 0.5 rad/s at 10 Hz): it bounds what any
  action, policy or human, can do in one step. `joint_abs` refuses to run without it.
- On the actor side use `joint_scale_rad` 0.05 (the `real_robot` default): a `joint_delta` action
  of `+-1` asks for `+-0.05` rad per step, and the daemon's cap clips anything beyond
  `joint_rad`. `state_fields` must contain `joint_pos` (the leader source compares against it).

Serve it on the robot PC:

```bash
robot-infra daemon --station robot.yaml --bind 0.0.0.0:50051 --token T
```

The actor's preset (`hil.yaml`, passed with `--config`; the README's
"Running on hardware" section has the full field list):

```yaml
env_backend: real_robot
env_id: my_task
obs:
  rgb: [wrist]
  image_size: [128, 128]
real_robot:
  remote: ROBOT_PC:50051
  token: T
  num_joints: 7
  action_mode: joint_delta          # or joint_abs; the same for demos, base policy and HIL runs
  joint_scale_rad: 0.05
  joint_limits: [[-2.8, 2.8], [-1.7, 1.7], [-2.8, 2.8], [-3.0, -0.1], [-2.8, 2.8], [0.0, 3.7], [-2.8, 2.8]]
  state_fields: [joint_pos, gripper_pos]
  control_hz: 10
  max_episode_steps: 200
```

A GELLO on the actor's machine instead (`--teleop-device gello`): leave the `teleop:` section
out of the robot yaml and give the calibration `options` as a flat mapping with
`--teleop-options-yaml gello.yaml` ([teleop.md](teleop.md#devices)).

## 4. `franka_real`: the stub-robot daemon

`franka_real` talks to the SERL HTTP bridge, which takes only cartesian targets. The GELLO is
plugged into the robot host and served by a stub robot (`type: franka`, `arm_backend: fake`, no
gripper, no cameras) whose only real device is the GELLO,
`robot_infra/docs/robot.gello_station.example.yaml`:

```bash
# on the robot host, next to the SERL bridge
robot-infra daemon --station robot.gello_station.yaml --bind 0.0.0.0:50052
```

The actor turns the leader joints into a cartesian twist with Panda forward kinematics
(`--teleop-device leader_cartesian --teleop-options-yaml station.yaml`, with
`remote_address: robot-host:50052` in `station.yaml`). The follower tracks a persistent command
target: a fixed hold pose while nobody drives, a command that leads the measured TCP by at most
`--teleop-lead-cap-m 0.05` / `--teleop-lead-cap-rad 0.3` and advances at most
`--teleop-target-speed-m-s 0.05` / `--teleop-target-speed-rad-s 0.3`. The env's scales must be at
least the lead caps (`--real-robot.xyz-scale-m 0.05 --real-robot.rot-scale-rad 0.3`, or the
`franka_real` action scales). The same source drives a `real_robot` arm in `cartesian_delta`;
`examples/presets/gello_cartesian.yaml` is a single-arm 50 Hz state-only preset for it. Mapping,
tracker, gripper and the DH check are in
[teleop.md](teleop.md#leader-arm-on-a-cartesian-robot-leader_cartesian).

## 5. Recording demos

`examples/record_real_demos.py` builds the env exactly like the actor and records only the
intervened steps, in the demo pkl format the learners load (`--demo-dataset-paths`), with the
action-contract sidecar. Record in the action mode of the run that will load the demos: the
learner rejects a pkl whose sidecar does not match its env (`joint_delta` demos for every stage of
a `joint_delta` pipeline, a separate `joint_abs` set for a `joint_abs` one; [tutorial 04](tutorial/04-record-demos.md)).

```bash
rwrl record --config hil.yaml \
    --teleop-device leader_arm --robot-teleop-action-mode joint_delta \
    --teleop-event-device keyboard --output-path demos/task.pkl --num-successes 20
```

Unlike the HIL actor, the recorder leaves the ramp's syncing steps out by default
(`--keep-syncing-steps` keeps them). The other flags (`--start-tolerance-rad`, the episode-start
sync, events) are in [recording-demos.md](recording-demos.md).

## 6. HIL training

The three HIL methods take the same teleop flags. `--robot-teleop-action-mode joint_delta`
declares the robot's teleop part before the remote robot connects; for a leader arm it only has to
name a joint space, the env's `action_mode` decides what is emitted.

The full learner and actor commands for `hil_serl`, `residual_hil_serl` (BC or ACT base) and
`silri`, with `--teleop-device leader_arm --robot-teleop-action-mode joint_delta
--teleop-event-device keyboard`, are in [tutorial 07](tutorial/07-hil-training.md); the operator's
keys and the onset choice are in [tutorial 06](tutorial/06-intervening.md).

Where the intervention toggle comes from:

- `--teleop-event-device keyboard` (the tutorial's route): the actor machine's keyboard through `pynput` (`t`
  takes over, `h` hands back, Esc ends the episode, `s`/`f`/`r` raise success/failure/rerecord,
  `q` ends the episode and stops the run after it; `pynput` needs the desktop session of that
  machine). `--teleop-event-device stdin` reads the same keys from the terminal running the
  command (no desktop session needed, works over SSH). Leave the yaml at `intervention: always`;
  the event device's state replaces it.
- An evdev pedal or keyboard on the robot PC: set `intervention: toggle` and `event_device:
  /dev/input/by-id/...` in the yaml's `teleop:` options (extra `gello-events`, user in the `input`
  group) and drop `--teleop-event-device`. The keys are `event_keys` (defaults above).

### Two GELLOs, two arms

For a two-arm station (`--env-backend real_robot_bimanual`, [bimanual.md](bimanual.md)) give each
robot its own GELLO. `rwrl setup` writes the station yaml for both: it probes the
GELLO buses, runs the calibration of each unit inline and writes `robots: {left, right}`, each with
its `teleop:` section and daemon port (the robot_infra README's
[Setup wizard](../robot_infra/README.md#setup-wizard);
`robot_infra/docs/station.bimanual.example.yaml` is an annotated example). Then train or record
with `--teleop-device bimanual`: `--teleop-arm-device leader_arm` reads each GELLO through its
arm's daemon; `--teleop-arm-device gello --teleop-options-yaml <station.yaml>` opens both GELLOs on
the actor's machine with the options from the same file. One `--teleop-event-device keyboard`
(or `stdin`) takes over and hands back both arms; [teleop.md](teleop.md#two-arms-bimanual) has
the intervention rule.

## 7. What the HIL rows record

Under intervention the actor executes, and pushes as the transition's `action`, the leader
source's action. In detail (these are the scripted end-to-end checks in
`tests/test_leader_arm_hil_loops.py`):

- **`joint_delta`.** The action is `clip((q_leader - q_follower) / joint_scale_rad, -1, 1)`: a
  far leader saturates at `joint_scale_rad` per step until the follower catches up, and the action
  is zero once they are aligned. Under `ramp`, the initial catch-up until the leader first comes
  within `joint_scale_rad` is marked syncing. Later saturated teaching steps after that alignment
  remain ordinary supervision.
- **`joint_abs` with `--teleop-onset ramp`.** The first steps after the toggle are rate-limited:
  the target is `q_follower + clip(q_leader - q_follower, +-ramp_step_rad)`, and the stored action
  is that target (the executed command). These steps carry
  `info["teleop_events"]["teleop_syncing"] = True`. The archive keeps their actual actions and human
  masks, while training excludes their expert supervision. After the ramp, the action is the
  leader's normalised joint target.
- **`--teleop-onset arm`.** Until the leader is within `--teleop-arm-tolerance-rad` (0.1) of the
  follower, the toggle does not count: the policy keeps acting, the rows are not intervened and the
  per-joint error is logged once per second. Then every step is intervened until the toggle
  releases.
- **`residual_hil_serl`.** The stored `action` is the human's total action (not an offset from the
  base); `base_actions` is the base policy's action for the same observation, and
  `policy_action` what the residual policy proposed.
- **`silri`.** `intervention_onset` tags the transition right before the human's first intervened
  step: with `ramp`, the policy step before the toggle; with `arm`, the last step before the leader
  was armed (the steps between the toggle and the arming are still the policy's).
- **Gripper.** The trigger width `w` in `[0, 1]` becomes `2w - 1`. The hybrid gripper heads
  (`hil_serl`'s DQN, `--residual-variant hybrid`, `silri`'s discrete gripper) label each stored
  gripper action with the nearest of `-1`/`0`/`+1`, so a part-squeezed trigger becomes the "stay"
  label. Set `gripper_binarize_threshold: 0.5` in the GELLO options for these methods: the stored
  gripper action is then exactly `+1` (open) or `-1` (closed).

### Onset: `ramp` or `arm`

`ramp` (default) takes over at once and slews the follower onto the leader at `ramp_step_rad`
per step (`joint_abs`) or `joint_scale_rad` per step (`joint_delta`). Both modes mark catch-up as
syncing and the training projection excludes it from expert supervision. `arm` keeps the transient out of the data
and off the robot, at the cost of the operator first matching the leader to the follower by hand
before every takeover (every env reset disarms). Prefer `arm` when demonstrations should start
from a still robot, `ramp` for quick corrections.

## 8. Not verified yet

- The pipeline has only been exercised with fakes (CPU tests with a scripted leader part and
  a fake Dynamixel SDK); no run on a physical GELLO or robot has been made yet.
- The hardware facts in section 1 (adapter, baud rate, latency, read time) and the per-unit
  offsets and signs are defaults and estimates; check them on your unit.
- XL330 torque-on behaviour and power (the warning in section 1); the backend keeps torque off.
- The Panda DH table used by `leader_cartesian` has not been compared with a real robot
  ([teleop.md](teleop.md#verify-the-dh-table-on-the-robot) has the check).

## 9. Future work: leader tracking (WP8)

With torque on, the leader could follow the follower while the policy acts, so a takeover starts
with the two arms aligned and neither onset policy is needed. This needs the torque-on power and
safety work of section 1 (current-limited position control, a compliant hand-over when the
operator grabs the leader) and is not implemented.
