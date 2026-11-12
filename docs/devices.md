# The device layer

The torch-free device layer lives in `real_world_rl/{devices,server,discovery,setup,supervise,doctor}`.
It supplies typed Arm, Gripper, Camera and Teleop contracts, hardware backends,
device-side safety limits, discovery and diagnosis. It does not import `rl_garden`
or the learning subpackages. `rwrl` exposes its commands through one parser.
Previously named `robot_infra` (and the standalone `rlgarden-robot-infra`), its
history is preserved. The learning extra supplies the gym environments and training.
The [robot daemon](daemon.md) serves these devices and exposes RemoteRobot to clients.

## Device commands in rwrl

| Command | Purpose | Reference |
|---|---|---|
| `rwrl setup` | Discover and commission a station. | [CLI](cli.md) |
| `rwrl discover` | Probe attached hardware without connecting to the robot. | [CLI](cli.md) |
| `rwrl doctor` | Check the host, environment and station devices. | [CLI](cli.md#robot-host-diagnosis-and-repair) |
| `rwrl daemon` | Serve station robots over gRPC. | [CLI](cli.md), [daemon](daemon.md) |
| `rwrl supervise` | Watch a child process through heartbeats. | [CLI](cli.md), [supervision](daemon.md#supervising-a-single-process-robot-run) |
| `rwrl teleop calibrate` | Print calibrated device YAML. | [CLI](cli.md) |

## Install and setup

Follow [Install](tutorial/01-install.md), [station setup](tutorial/02-setup.md),
and [Franka bringup](tutorial/03-franka-bringup.md). SpaceMouse installation and
permissions are covered in the [teleop guide](teleop.md#spacemouse-installation-and-linux-permissions).

## Station file reference

Examples run on the robot PC, from the repository root with the project environment active.

A robot config is one YAML mapping with a `type` key. It can also be nested under a
top-level `robot:` key. Unknown keys are rejected with the list of valid ones. Omitted
keys take the defaults shown below. Full annotated copies are in
[`examples/stations/robot.franka.example.yaml`](../examples/stations/robot.franka.example.yaml) and
[`examples/stations/robot.piper.example.yaml`](../examples/stations/robot.piper.example.yaml). The
[setup wizard](tutorial/02-setup.md) writes this file for a Franka station.

A station with several robots (for example two Frankas for bimanual work) lists one robot
mapping per name under `robots:`; a mapping without `name` takes its key. Each robot may carry a
`daemon:` block (also allowed in a single-robot file), which the daemon reads and strips before
the robot config is parsed:

```yaml
robots:
  left:
    type: franka
    robot_ip: 172.16.0.2
    # ... any robot key, as above
    daemon: {bind: 0.0.0.0:50051, token: null}   # default bind 0.0.0.0:50051, no token
  right:
    type: franka
    robot_ip: 172.16.1.2
    daemon: {bind: 0.0.0.0:50052, token: null}
```

`rwrl daemon` then starts one daemon process per robot, and
`rwrl doctor` checks each robot in turn; `--robot <name>` picks one. A full
example with two GELLOs and three cameras is
[`examples/stations/station.bimanual.example.yaml`](../examples/stations/station.bimanual.example.yaml).
`Robot.from_yaml` reads single-robot files without a `daemon:` block only.

Upper-layer packages can register additional robot-mapping sections through
`parse_station`, `load_station`, or `load_station_with_env` using
`sections={name: validator}`. Like `daemon:`, these sections describe the robot
without being device configuration: the loader removes each registered key before
`RobotConfig.from_dict`, validates its value, and stores the returned value in
`StationRobot.sections`. The device layer does not interpret section contents.
Without sections (`sections=None` or `{}`) these loaders, and `load_config` and
`load_config_with_env`, parse strictly; there is no entry-point discovery. real_world_rl's
sections (`teleop_profiles`) are registered by `real_world_rl.station.load_station` and
`load_station_with_env`, which every `rwrl` command uses, the daemon, setup, discover and
the device doctor included. An unknown key names that loader and the registered section names.
Unregistered unknown keys still fail; single-robot and `robots:` mappings both
support sections. The setup wizard renders device configuration, not upper-layer
sections.

**Franka Panda + Franka Hand + one RealSense**

```yaml
type: franka
name: franka                  # default "franka"
arm_backend: franky           # franky (default) | serl_http (legacy ROS1 bridge) | fake
robot_ip: 172.16.0.2          # FCI IP set in Desk (default 172.16.0.2)
# bridge_url: http://127.0.0.1:5000   # serl_http only: URL of the legacy Flask bridge
realtime: ignore              # ignore (default) | enforce (requires a PREEMPT_RT kernel)
relative_dynamics_factor: 0.2 # franky velocity/accel/jerk scale for move_to_* (default 0.2)
home_joints: [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]  # 7 values (default shown)
safety_box: [0.3, -0.3, 0.05, 0.7, 0.3, 0.6]  # [xmin, ymin, zmin, xmax, ymax, zmax] m, base frame of tcp_pose (O_T_EE)
joint_limits:                 # 7 [lo, hi] pairs in rad; home_joints must lie inside
  - [-2.8, 2.8]
  - [-1.7, 1.7]
  - [-2.8, 2.8]
  - [-3.0, -0.1]
  - [-2.8, 2.8]
  - [0.0, 3.7]
  - [-2.8, 2.8]
max_delta_per_step: {xyz_m: 0.05, joint_rad: 0.1}  # servo step cap from the measured state
claim: true                   # daemon takes a per-host exclusive lock while connected
gripper:                      # null = no gripper
  backend: franky             # franky (default) | serl_http | fake
  max_width_m: 0.08
  mode: binary                # binary (default) | continuous
  binary_threshold: 0.5       # binary mode: an action > threshold opens
  shares_arm_session: false   # the Franka Hand opens its own connection
cameras:                      # default: []
  - name: wrist
    backend: realsense        # realsense (default) | fake
    serial: ""                # "" = use the only attached camera; required when >1 is attached
    width: 640
    height: 480
    fps: 30
    enable_depth: false
```

**AgileX Piper (CAN)**

```yaml
type: piper
name: piper
arm_backend: pyagxarm         # pyagxarm (default) | piper_sdk | fake
can_interface: can0           # default can0
bitrate: 1000000              # must match the interface (default 1000000)
firmware_profile: null        # pyagxarm only: default | v183 | v188 | v189; null probes the arm
home_joints: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 6 values, rad
safety_box: [0.1, -0.3, 0.1, 0.5, 0.3, 0.5]   # m, base frame; tcp_pose is the FLANGE pose (no tool offset)
joint_limits: [[-2.6, 2.6], [0.0, 3.1], [-2.9, 0.0], [-1.7, 1.7], [-1.2, 1.2], [-2.0, 2.0]]  # rad
max_delta_per_step: {xyz_m: 0.02, joint_rad: 0.05}
claim: true
gripper:
  backend: pyagxarm           # must match the arm backend: pyagxarm | piper_sdk
  max_width_m: 0.07
  mode: continuous            # Piper default
  binary_threshold: 0.5
  shares_arm_session: true    # required: the Piper gripper borrows the arm's CAN session
cameras: []
```

Safety limits and teleop (all optional, all three keys are read from the robot PC's yaml):

- `safety_box`: `[xmin, ymin, zmin, xmax, ymax, zmax]` in metres, in the frame of
  `read_state()["tcp_pose"]` (Franka: `O_T_EE`, Piper: the **flange**, not the gripper tip).
  Only the position is limited; the orientation is left untouched.
- `joint_limits`: one `[lo, hi]` pair per joint, in rad. `home_joints` must lie inside it.
- `max_delta_per_step`: `xyz_m` and/or `joint_rad`, a cap on how far a servo target may be from
  the measured state. Servo only; `move_to_*` is not capped.
- The yaml on the robot PC is the source of truth. `servo_*` targets outside the limits are
  **clipped** and counted (`safety_clips`); `move_to_*` targets outside them are **refused** with
  `SafetyViolation` and nothing is sent (`safety_rejects`). The same applies through the daemon,
  `send_action`, and the local API, see [Safety rules](#safety-rules).
- `teleop:` adds an operator input device as a part of the robot (see [step 7](devices.md#add-a-teleop-device)).

Notes:

- `shares_arm_session: true` is mandatory for the `pyagxarm` and `piper_sdk` grippers and
  forbidden for `franky` and `serl_http`. A mismatch raises `ValueError` at build time.
- With `arm_backend: piper_sdk`, also set `gripper.backend: piper_sdk`. That backend ignores
  `firmware_profile` and always opens the bus at 1 Mbit/s, so `bitrate` is only checked
  against the interface.
- `serl_http` has no joint-space control and needs the legacy bridge running (see [Legacy](#legacy)).

Build a robot without touching hardware to check the file:

```python
from real_world_rl.devices import Robot

robot = Robot.from_yaml("robot.yaml")   # builds parts; does not connect
print(robot.describe_text())
```

```text
franka (franka)
  arm: FrankyArm [sdk=franky, connected=False]
  gripper: FrankyGripper [sdk=franky, connected=False]
  cameras: PartGroup [sdk=None, connected=False]
    wrist: RealSenseCamera [sdk=pyrealsense2, connected=False]
```

With a `teleop:` section a last line is added, for example `  teleop: SpaceMouseTeleop [sdk=hid, connected=False]`.

### Station lookup and setup semantics

It probes the hardware (TCP 22/443/1337 of `172.16.0.2`, `172.16.1.2`, `$ROBOT_IP` and
`--robot-ip`, plus every host of `--scan-subnet CIDR`; RealSense serials; a Dynamixel broadcast
ping on every `/dev/serial/by-id/usb-FTDI_*` and `usb-ROBOTIS_*` port at 57600 to 4000000 baud;
SpaceMouse devices), then asks per arm for the robot IP, arm and gripper backend, home joints,
cameras (`1=wrist,2=front`), teleop device (GELLO port and servo ids, the last id offered as the
gripper trigger), intervention mode and daemon port, calibrates each GELLO inline (the same
steps as the calibration CLI in [device contract](#add-a-teleop-device); skipped, it writes zero offsets
with a `# TODO calibrate` comment), writes the annotated station yaml, runs the doctor on it and
prints the next commands.

Every arm gets device-side safety limits: the Panda datasheet `joint_limits` (`--joint-limits`),
a servo step cap `max_delta_per_step: {xyz_m: 0.05, joint_rad: 0.04}` (`--max-delta-xyz-m`,
`--max-delta-joint-rad`) and a `safety_box` it asks for (base frame, metres, suggested
`[0.3, -0.3, 0.05, 0.7, 0.3, 0.6]`, `--safety-box`), which must contain the home pose.


On the robot PC, from the repository root with the project environment active.
See the [setup walkthrough](tutorial/02-setup.md) for commands.

The daemon, the doctor and discovery find the station yaml without `--station`. They take the
first of:

1. `$RWRL_STATION`
2. `./local/station.yaml` (relative to the current directory; the wizard's default, `--scope local`; `local/` is git-ignored)
3. `$XDG_CONFIG_HOME/rwrl/station.yaml`, default `~/.config/rwrl/station.yaml` (`--scope user`)

For one release the pre-0.1.0 names are still read after their replacements: `$ROBOT_INFRA_CONFIG`
after `$RWRL_STATION`, and `~/.config/robot_infra/station.yaml` after the user location; `rwrl doctor`
reports either as a `WARN` with the migration command.

`--station <yaml>` still wins over all three.

Every question has a flag (`rwrl setup --help`); per-arm flags take one value
per arm or one for all arms. `--non-interactive` fails on the first question that has neither a
flag nor a `--station` value; `--non-interactive --station X` rewrites `X` unchanged. An existing file
is first copied to `station.yaml.bak.<UTC %Y%m%dT%H%M%SZ>`, then replaced atomically. A probed
value is required unless `--allow-unprobed`.


## Discover

See [station setup](tutorial/02-setup.md) and the [CLI](cli.md).
Discovery only probes; it never connects to a robot and does not list fake backends.
Without a station, host discovery prints `none`.

## Doctor

See [doctor](doctor.md) for the layers, messages, repairs and troubleshooting,
and [Franka bringup](tutorial/03-franka-bringup.md) for the first real handshake.
Claims are per-host exclusive locks held by RobotService while connected; plain
`Robot.from_yaml` scripts take no claim. `claim: false` deliberately opts out.
Only processes holding a hardware connection count as owners ([expected devices](doctor.md#expected-devices)).

## Add a teleop device

The examples below run on the robot PC, from the repository root with the project
environment active. Application intervention semantics are in [Teleoperation](teleop.md).

A `teleop:` section makes an operator input device a part of the robot. Hardware-free, with
the fake backend (a script advanced one step per `read()`):

```yaml
teleop:
  name: teleop                # the daemon serves teleop.<name>.read / .reset
  backend: fake               # fake | spacemouse | gello
  hold_s: 0.0                 # keep intervening this long after the last activity (default 0)
  options:                    # backend-private
    action_space: joint_abs   # fake: cartesian_delta (default) | joint_abs | joint_delta
    num_joints: 7             # joint spaces only
    has_gripper: true         # default true
    on_end: hold_last         # hold_last (default) | idle | loop, what happens after the script
    script:
      - {action: [0, 0, 0, 0, 0, 0, 0], active: false, hold: 2}
      - {action: [0.5, 0, 0, 0, 0, 0, 0], gripper: 0.0, active: true, hold: 2}
      - {events: {episode_end: true}}
```

A step is `{action, gripper, buttons, events, active, hold}`; `hold` is the number of reads the
step lasts and its events fire on the first of them. `fail_after: n` makes every poll after the
n-th sample fail, to test the error path.

```python
robot = Robot.from_yaml("robot.yaml")          # local; RemoteRobot works the same way
robot.connect()
sample = robot.teleop.read()                   # blocks for a new sample, consumes the events
print(sample.seq, sample.action_space, sample.action, sample.gripper, sample.intervening, sample.events["episode_end"])
latest = robot.teleop.latest()                 # no blocking, does not consume events
robot.teleop.reset()                           # drops latched events and the hold_s hysteresis
```

`read()` returns a `TeleopSample`; over the daemon (`teleop.<name>.read`) it travels as a dict
with the same keys:

| Field | Type | Meaning |
|---|---|---|
| `action` | `float32` ndarray | `cartesian_delta`: `(6,)` normalized twist in [-1, 1]; `joint_abs` / `joint_delta`: `(N,)` joint values in rad |
| `action_space` | `str` | `cartesian_delta`, `joint_abs` or `joint_delta` |
| `gripper` | `float` or `None` | normalized width target in [0, 1]; `None` = the device has no gripper channel |
| `buttons` | `dict[str, bool]` | current levels (SpaceMouse: `button_0`, `button_1`) |
| `events` | `dict[str, bool]` | latched edges `episode_end`, `success`, `failure`, `rerecord` (always all four keys), plus any backend-specific edge that fired (SpaceMouse: `button_0`, `button_1` presses); `read()` clears them |
| `intervening` | `bool` | a human is operating (backend's `active`, extended by `hold_s`) |
| `timestamp` | `float` | `time.monotonic()` on the machine that polled, not comparable across hosts |
| `seq` | `int` | increases with every polled sample, also across reconnects; a gap means dropped samples |

`read(timeout_s)` raises `DeviceFault` on a timeout and a recoverable `DeviceFault` once 5 polls
in a row failed (reconnect the robot); it raises `DeviceNotConnected` if the robot is
disconnected while it waits. There is a single consumer: two clients reading the same teleop
split the events between them.

`backend: spacemouse` (extra `spacemouse`; options `device_index` 0, `deadzone` 1e-3,
`invert_axes`, `poll_interval_s` 0.005) is a `cartesian_delta` device with a gripper: the puck
gives the twist `[-y, x, z, -roll, -pitch, -yaw]`, every button press is latched as the event
`button_0`/`button_1` (so a press between two reads is not lost), and button 0 toggles the sticky
gripper target (1.0 = open at connect). Neither button raises `episode_end`. The application's button mapping is described in the [teleop guide](teleop.md#spacemouse). It needs read/write access to
`/dev/hidraw*`; `rwrl doctor` checks this as `os:hid` and prints the udev
rule installation command. See [Known limitations](#known-limitations-and-needs-hardware-verification).

`backend: gello` (extra `gello`, plus `gello-events` for `event_device`) is a GELLO leader arm
(passive Dynamixel servos, Protocol 2.0, through a U2D2 or OpenRB-150): a `joint_abs` device
with one joint per `joint_ids` entry and a gripper channel when `gripper_id` is set. Each poll
is one sync read; joint `i` is `wrap((raw_rad - joint_offsets_rad[i]) * joint_signs[i])` into
`[wrap_centers_rad[i] - pi, wrap_centers_rad[i] + pi)`, so no start pose is needed at connect
(set the centres to the midpoints of `joint_limits`; Panda joint 6 reaches past pi and needs a centre > 0).
The gripper is `clip((raw - gripper_closed_rad) / (gripper_open_rad - gripper_closed_rad), 0, 1)`,
1 = open, optionally binarized with `gripper_binarize_threshold`. `intervention: always`
(default) reports `intervening` on every poll; `toggle` follows a toggle key read with `evdev`
from `event_device` (a pedal or keyboard on the robot PC), whose other keys raise
`episode_end`/`success`/`failure`/`rerecord` (`event_keys`). Torque is turned off on connect and
on disconnect; a missing servo or a failed sync read fails the poll (no stale samples). The
full option list with defaults is in [`examples/stations/robot.franka.example.yaml`](../examples/stations/robot.franka.example.yaml).
Calibrate once per unit (torque off, the leader is moved by hand); the [setup wizard](tutorial/02-setup.md)
runs these steps inline and writes the result into the station yaml. Standalone, the CLI prints
the block to paste:

```bash
rwrl teleop calibrate --backend gello \
    --port /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_XXXX-if00-port0 \
    --joint-ids 1 2 3 4 5 6 7 --gripper-id 8 \
    --calibration-pose 0 0 0 -1.57 0 1.57 0 --joint-limits-yaml robot.yaml
```

It checks each joint's sign (move one joint in the robot's positive direction when asked),
solves the offsets on the quarter-turn grid from the calibration pose, captures the trigger's
open and closed angles, and takes the wrap centres from the yaml's `joint_limits`. The port
needs read/write access (`dialout` group); the doctor checks it as `os:serial` and `os:groups`,
and warns about brltty on CH340 adapters (`os:brltty`) and a slow FTDI `latency_timer`
(`os:latency_timer`). The baud rate
is never written to the servos: change it once in Dynamixel Wizard (1000000 gives about 130 Hz
for 8 servos, 57600 about 25 Hz).

## Architecture

```text
Connection (ABC)                 connect()/disconnect() lifecycle, owner borrowing, SDK, health()
└── Part                         observation_features, get_observation()
    ├── Camera   [registry]      background capture thread, read()/latest()/reopen()
    │   ├── realsense, fake
    │   └── (RemoteCamera is a Part, not a Camera)
    ├── Teleop   [registry]      background poll thread, read()/latest()/reset(), latched events
    │   ├── gello, spacemouse, fake
    │   └── (RemoteTeleop is a Part, not a Teleop)
    └── ControllablePart         action_features, send_action()
        ├── Arm      [registry]  read_state, servo_cartesian/joints, move_to_pose/joints
        │   │                    (template methods, see below), stop, recover, set_control_mode, preflight
        │   ├── franky, serl_http, pyagxarm, piper_sdk, fake
        │   └── RemoteArm
        ├── Gripper  [registry]  open/close/move(width in [0, 1]), width; binary | continuous
        │   ├── franky, serl_http, pyagxarm, piper_sdk, fake
        │   └── RemoteGripper
        └── PartGroup            named children; connect in order, disconnect in reverse
            └── Robot  [type registry]  config -> arm (+ gripper) (+ cameras group)
                ├── FrankaRobot "franka"   (FrankaConfig)
                ├── PiperRobot  "piper"    (PiperConfig)
                └── RemoteRobot "remote"   (RemoteRobotConfig; registered by importing real_world_rl.server)

Robot PC:   Robot ── RobotService (method table, watchdog, DeviceClaim) ── GrpcTransport
GPU side:   RemoteRobot ── GrpcClient (msgpack over gRPC, token metadata, read-only retry)
```

Backends register by name (`Arm.register("franky")`). `Arm.backends()`, `Gripper.backends()`,
`Camera.backends()` and `Teleop.backends()` list them, and `Robot.types()` lists robot types. A gripper with
`shares_arm_session: true` borrows the arm's session as its `owner` instead of opening its
own.

| Backend | SDK (extra) | Joint control | Wrench (`tcp_force`/`tcp_torque`) | Notes |
|---|---|---|---|---|
| arm `franky` | `franky` (`franky`) | yes | yes | torque-level impedance trackers for servo modes, Ruckig `move_to_*` (`IDLE` only); `realtime` and `relative_dynamics_factor` apply here |
| arm `serl_http` | none (HTTP) | no | yes | legacy client of the ROS1 bridge at `bridge_url`; Cartesian servo only; state has no `joint_pos`/`joint_vel` |
| arm `pyagxarm` | `pyAgxArm` (`pyagxarm`) | yes | no | Piper default; `firmware_profile` probed or set; state `tcp_pose`, `joint_pos`, `joint_vel` (flange pose) |
| arm `piper_sdk` | `piper_sdk` (`piper-sdk`) | yes | no | vendor SDK, fixed at 1 Mbit/s; `stop()` holds by re-sending the measured joints |
| arm `fake` | none | yes | yes | deterministic first-order arm for tests and `--mock` |
| gripper `franky` | `franky` | - | - | Franka Hand; own session |
| gripper `serl_http` | none | - | - | binary only, through the bridge |
| gripper `pyagxarm` / `piper_sdk` | as the arm | - | - | requires `shares_arm_session: true` and the matching arm backend |
| camera `realsense` | `pyrealsense2` (`realsense`) | - | - | RGB (+ aligned depth in metres with `enable_depth`) |
| `fake` gripper / camera | none | - | - | tests and `--mock` |
| teleop `spacemouse` | `hidapi==0.14.0.post4`, import `hid` (`spacemouse`) | - | - | cartesian twist + sticky gripper; `Teleop.backends()` is `fake`, `gello`, `spacemouse` |
| teleop `gello` | `dynamixel-sdk` (`gello`; `evdev` via `gello-events`) | - | - | GELLO leader arm, `joint_abs` + optional gripper; torque off; evdev toggle and events |
| teleop `fake` | none | - | - | scripted samples, `push()` injection, `fail_after`; tests and `--mock` |

**`Arm` is a template-method class.** The public `servo_cartesian`, `servo_joints`,
`move_to_pose` and `move_to_joints` sanitize the input (shape, finiteness, unit quaternion),
apply the safety limits and only then call the backend hooks `_servo_cartesian`,
`_servo_joints`, `_move_to_pose` and `_move_to_joints`. Likewise the public `stop()` clears
`last_cartesian_target` and then calls the backend hook `_stop`. A backend implements the
hooks (plus `read_state`, `recover`, `_enter_control_mode`) and never overrides the public
methods, so no path to the motors skips the guard. A backend's `read_state()` must return its
result through `self._remember_state(state)`: the step cap reuses that cached state (at most
`MAX_DELTA_STATE_AGE_S` = 0.5 s old, else it re-reads) instead of querying the robot on every
servo call. `Arm.from_config(config)` builds the backend with `ArmSafety.from_config(config)`.
A third-party backend that skips `_remember_state` still works but costs one extra
`read_state()` per capped servo call.

### Safety rules

- **`disconnect()` never powers off.** Releasing an arm only closes the SDK session. The
  Piper backends never call `disable()`/`reset()`/`DisableArm`/`ResetPiper`, because a
  powered-down Piper falls. `stop()` brings motion to rest and leaves power and brakes as
  they are.
- **Servo commands are refused when the arm is not ready.** Every backend refuses servo
  commands outside the matching servo mode. franky, pyagxarm and piper_sdk also refuse them
  while the arm is faulted (call `recover()` first). The Piper backends also refuse on stale
  feedback: no new joint frame for 0.5 s on pyagxarm, 1.0 s on piper_sdk. On the hardware
  backends, `move_to_*` requires `IDLE`. Actions are checked for shape and finiteness, and
  `tcp_pose` quaternions are checked for unit norm.
- **Connecting enables the motors.** Connecting a Piper (directly, through the doctor, or
  through the daemon when a client connects) enables its motors, and they stay enabled
  after `disconnect()`. Clear the workspace before running the doctor.
- **Safety limits.** `safety_box`, `joint_limits` and `max_delta_per_step` in the robot yaml on
  the robot host are the source of truth. `Arm` enforces them in the public command methods, so
  they hold on every path: local `Robot`, the daemon (`arm.servo_*`, `arm.move_to_*`,
  `robot.send_action`, `robot.step`) and `send_action`. A `RemoteArm` has no guard of its own and
  forwards to the daemon's.
  - `servo_*`: the target is first shortened to `max_delta_per_step` from the measured state, then
    clipped into the base box / joint limits. Any accepted change counts one `safety_clips`.
    With `workspace` configured, Cartesian targets must also satisfy its oriented position box
    and optional SO(3) orientation radius, both before and after base-box clipping. A workspace
    rejection raises `SafetyViolation`, increments `safety_rejects`, and sends nothing.
  - `move_to_*`: a target outside the box / joint limits raises `SafetyViolation`, counts one
    `safety_rejects`, and nothing is sent. `move_to_*` is not step-capped.
  - `SafetyViolation` is not a device fault: it is raised before the backend is touched, so it
    cannot latch a fault, needs no `recover()`, keeps the control mode, and through the daemon it
    does not refresh the watchdog. A client sees the same class.
  - `arm.health()` reports `safety_clips` / `safety_rejects`; `robot.describe` exposes the limits
    as `arm.safety`. The checks are validated at build time: `home_joints` must lie inside
    `joint_limits`, a box must have min <= max.
  - `safety_box` and the Cartesian step cap limit position only. A workspace may additionally
    constrain orientation with `orientation_xyzw` (in its frame) and `max_angle_rad` (SO(3)
    geodesic radius, avoiding Euler branch cuts). Orientation has no device-side step cap.
    Joint commands are not checked against Cartesian workspace boundaries: these limits do not
    certify a joint trajectory, whole-arm clearance, or collision-free motion.
  - Target checks use `1e-9` numerical tolerance. Feedback checks use workspace keys
    `measured_margin_m` (default `0.003 m`) and `measured_margin_rad` (default `0.03 rad`),
    both finite and nonnegative. The environment and teleop tracker use these feedback margins;
    they never relax target admission. A `task_frame` without a workspace is a display/sampling
    reference and adds no spatial contract. With a workspace, both geometries are compared
    against the daemon before reset.
- **Watchdog.** The daemon idles a servoing arm whose client went quiet (see step 4).
- **`DeviceClaim`.** An advisory, node-local file lock in `$XDG_CACHE_HOME/rwrl/claims/`
  (default `~/.cache/rwrl/claims/`), keyed `<type>:<robot_ip | can_interface | name>`.
  The daemon holds it while a client is connected, so a second daemon on the same robot gets `DeviceBusy`. It does not protect
  against other hosts or against code that does not take the claim. Set `claim: false`
  to opt out. For one release, an existing same-name lock in `~/.cache/robot_infra/claims/`
  is also checked: a held legacy lock raises `DeviceBusy` with its path in the hint; an
  unheld legacy lock is held alongside the new lock until release. The old directory is
  never created for compatibility. The SpaceMouse reader also claims its HID endpoint by
  default and honors the same `RobotConfig.claim` setting when built as a robot teleop part. Its built-in device
  table equals PySpaceMouse 1.0.3; a site's local device table adds or overrides models.

### Error model

Device errors subclass `DeviceError` (a `RuntimeError`) and carry `message`, `hint` and
`device`. `str(error)` prints the hint on a second line. Errors survive the
`to_dict()`/`from_dict()` round trip, so a client re-raises the same type the daemon raised.

| `kind` | Class | Typical cause |
|---|---|---|
| `sdk_missing` | `SdkMissing` | vendor package not installed |
| `device_not_found` | `DeviceNotFound` | robot unreachable, CAN interface or camera serial missing |
| `device_not_connected` | `DeviceNotConnected` | call before `connect()` |
| `device_already_connected` | `DeviceAlreadyConnected` | defined; not raised by current backends (`connect()` is idempotent) |
| `device_busy` | `DeviceBusy` | another process holds the claim |
| `device_fault` | `DeviceFault` (`recoverable: bool`) | reflex, e-stop, no feedback, wrong mode, watchdog |
| `version_mismatch` | `VersionMismatch` | libfranka vs robot server version; the wrong distribution `hid` without trezor's `hid.device()` API |
| `safety_violation` | `SafetyViolation` | a Cartesian servo/move target outside `workspace` (position or orientation), or a `move_to_*` target outside `safety_box` / `joint_limits`; refused before sending, counted by `safety_rejects` |
| `transport` | `TransportError` (a `DeviceFault`) | daemon unreachable, timeout, token mismatch |

### Command protocol

The device layer's native commands, `setup`, `daemon`, `supervise` and `discover`, are
`rwrl` subcommands; there is no separate binary and no `python -m` entry point. `rwrl doctor` runs the device checks of this layer
for every station robot, together with its host and project checks
([cli.md](cli.md#robot-host-diagnosis-and-repair)); all doctor code lives in
`real_world_rl.doctor`. The command protocol and parser do not import torch.
Station input is `--station PATH`. Setup uses that file as defaults, not as
its output path; station lookup defaults are unchanged. Errors use stderr,
and exit-code constants live in `real_world_rl.cli.errors`.

On the robot PC, from the repository root with the project environment active:

```bash
rwrl --help
rwrl daemon --help
rwrl doctor --station local/station.yaml --json
```

`rwrl doctor` accepts `--yes` to skip confirmation while still running preflight,
even without a repair flag. Repair confirmation accepts only `y` (ignoring
case and surrounding whitespace). The shared exit policy is 0 for success,
1 for runtime/device failure or unresolved doctor findings, and 2 for usage
or configuration errors. Setup interruption returns 130; supervisor launch
failure returns 127 and stop escalation returns 1, otherwise child status is
preserved.

Commands without JSON output (`setup`, `daemon`, `supervise`) reject the global `--json` option with usage exit code 2.
`doctor` and `discover` support `--json`; `rwrl --json discover` prints
`schema_version: 1` and the lists `cameras`, `teleops`, `can_interfaces`,
`hosts`, `hid_devices` and `unknown_hid` (see [Plug in and discover](devices.md#discover)).
`supervise` retains its `-- <child command...>` boundary; child flags are passed
to the child unchanged. `teleop calibrate --backend gello` prints a teleop YAML block;
`setup --only teleop --calibrate` runs the same calibration in the wizard and
writes station YAML.

### Backend dependencies

| Extra | Installs | You need it when | Machine |
|---|---|---|---|
| `franky` | `franky-control` | Franka arm/hand via libfranka (`arm_backend: franky`). The default wheel bundles one libfranka version; for other robot firmware, install the matching wheel (see [Installing the right franky wheel](doctor.md#installing-the-right-franky-wheel)). | robot PC |
| `pyagxarm` | `pyAgxArm` (from GitHub) | Piper via `arm_backend: pyagxarm` (the default for Piper) | robot PC |
| `piper-sdk` | `piper_sdk` | Piper via the vendor SDK (`arm_backend: piper_sdk`) | robot PC |
| `realsense` | `pyrealsense2` | RealSense cameras | robot PC |
| `spacemouse` | `hidapi==0.14.0.post4` (trezor/cython-hidapi, import `hid`) | `teleop: {backend: spacemouse}` (in-tree reader and discovery) | robot PC |
| `gello` | `dynamixel-sdk>=3.7.31,<3.9` | `teleop: {backend: gello}` (GELLO leader arm) and its calibration CLI | robot PC |
| `gello-events` | `evdev` | a gello `event_device` (pedal or keyboard toggle and events, Linux) | robot PC |
| `grpc` | `grpcio>=1.60` | the daemon (`rwrl daemon`) and `RemoteRobot` | robot PC **and** GPU machine |
| `jpeg` | `pillow`, `opencv-python-headless` (`cv2`) | `rwrl daemon --jpeg-quality N`; the JPEG path in `server/codec.py` imports `cv2` | robot PC **and** GPU machine |
| `legacy-bridge` | `flask`, `pyzmq`, `scipy`, `pyspacemouse` | legacy Flask bridge/gripper server and the legacy `teleop/spacemouse/` (old `read_all()` API, separate from the in-tree reader; ROS1 must be installed separately) | robot PC |
| `sim` | `mani_skill` | the ManiSkill impedance controllers in `controller/simulator/` and the ManiSkill examples | GPU machine |
| `dev` | `pytest`, `scipy` | running the tests | any |

Vendor SDKs are imported lazily. The core package, the CLIs and `--mock` runs work
without any SDK installed.

### Discovery report


Discovery only probes. It never connects to a robot. `--station` is optional: the given
config's (or else the [station file's](tutorial/02-setup.md)) `robot_ip` of every robot is pinged and
TCP-probed on ports 22, 80, 443, 1337 and 50051.

On a machine with nothing attached (real output):

```text
$ rwrl discover --station robot.yaml
cameras:
  none
teleops:
  none
can interfaces:
  none
hosts:
  172.16.0.2: ping=no reply open ports=none
hid devices:
  none
```

With hardware attached, the lines take the form `realsense: <serial>, <serial>`,
`spacemouse: <vid>:<pid>:<path>` (or `/dev/hidrawN` when `hidapi` is not installed),
`can0: operstate=up bitrate=1000000`, `172.16.0.2: ping=ok open ports=<list>` and
`/dev/hidraw0`. The `fake` backends are not listed.

`rwrl --station robot.yaml --json discover` (or `rwrl --json discover`) prints the same
report as one JSON object: `schema_version: 1` and six lists, each empty when nothing is found,
`cameras` (`backend`, `serial`), `teleops` (`backend`, `device`), `can_interfaces` (`name`,
`operstate`, `bitrate`), `hosts` (`ip`, `ping_ok`, `open_ports`), `hid_devices` (paths) and
`unknown_hid` (`vid`, `pid`, `product`, `path`, `message`: HID devices that look like a SpaceMouse
but are missing from the device table). The machine above gives (derived from the same report,
not a captured run):

```json
{"schema_version": 1, "cameras": [], "teleops": [], "can_interfaces": [], "hosts": [{"ip": "172.16.0.2", "ping_ok": false, "open_ports": []}], "hid_devices": [], "unknown_hid": []}
```

| Section | Source | What to look for |
|---|---|---|
| `cameras` | each camera backend's `discover()` (RealSense serials) | copy the serial into `cameras[].serial` |
| `teleops` | each teleop backend's `discover()` (SpaceMouse: `vid:pid:path` through `hidapi` when installed, else every `/dev/hidraw*`; gello: `/dev/serial/by-id/usb-FTDI_*` and `usb-ROBOTIS_*`) | the device you put in `teleop:` shows up |
| `can interfaces` | `/sys/class/net/*` with type CAN, bitrate from `ip -details link show` | `operstate=up` and the bitrate in your yaml |
| `hosts` | `ping` and TCP connect to the config's `robot_ip` | `ping=ok`. For Franka, 1337 open usually means FCI is reachable. |
| `hid devices` | `/dev/hidraw*` | SpaceMouse and other HID teleop devices |
| `warnings` (only when there are any) | 3Dconnexion (`256f`) HID devices, and Logitech (`046d`) multi-axis controllers, whose id is missing from the SpaceMouse device table | `WARN unknown 3Dconnexion-vendor HID device <vid>:<pid>; add it to spacemouse-devices.yaml (see docs/spacemouse-commissioning.md#site-device-table) (<path>)`: add a [site device table](spacemouse-commissioning.md#site-device-table) entry |

Without `--station` and without a station file, `hosts:` prints `none`.

### SpaceMouse device identity

The in-tree reader ports all 10 models from the vendored PySpaceMouse 1.0.3 table;
Enterprise is not in that table. Other models a site meets go into its local device table,
`spacemouse-devices.yaml` next to the station file (or the path in the teleop option
`device_table`), which adds entries or overrides the built-in entry of the same id; see
[Site device table](spacemouse-commissioning.md#site-device-table). `discover()`,
doctor and `open()` match the exact vendor and product id pair of that merged table, so
other devices of the same vendor (for example a Logitech `046d` keyboard receiver) are not
taken for a SpaceMouse. It uses `hid.device()` and does not import
`pyspacemouse`/`easyhid`. Do not install the different distribution named `hid`.
The legacy `legacy-bridge` extra and `SpaceMouseExpert` still use their existing API.
## Testing

The device-layer tests are `tests/devices/` of the repository test suite. From the repository
root, project environment active:

```bash
pip install -e '.[dev,grpc]'
python -m pytest tests/devices -q
```

The tests use the fake backends and SDK stubs. No hardware is needed. The gRPC tests are
skipped when `grpcio` is missing. The maintainers run them in a remote container:

```bash
docker exec <container> bash -lc 'cd <checkout> && python -m pytest tests/devices -q'
```

## Known limitations and needs hardware verification

Nothing below has been exercised on real hardware; the fakes cover the logic only.

- **SpaceMouse USB vendor ids.** `discover()` filters on `0x256f` (3Dconnexion) and `0x046d`
  (Logitech, older units). Both are unverified; without `hidapi` it lists every `/dev/hidraw*`.
- **udev and Docker.** The udev rule printed by `os:hid` and the `docker run --device=/dev/hidrawN`
  hint are untested on a robot PC. A container needs the node passed in before the daemon starts.
- **SpaceMouse wheel and timing.** The in-tree reader uses `set_nonblocking(1)` and
  polls one report every `poll_interval_s` (default 0.005 s, 200 Hz). Queue latency,
  split-report timing, device directions and disconnect behavior are not yet verified
  on hardware. The `hidapi==0.14.0.post4` wheel's ELF/system-library dependencies
  remain unverified on a clean Ubuntu 22.04 / CPython 3.10 / x86_64 machine.
- **Piper master arm** as a teleop (leader-follower) is not implemented; the leader-arm backend is
  `gello` (Dynamixel). `joint_delta` is exercised by the fake only.
- **GELLO on hardware.** The `gello` backend and its calibration CLI run against a fake
  `dynamixel_sdk` and a fake `evdev` only. Unverified: the U2D2/OpenRB-150 by-id names that
  `discover()` globs, sync-read timing per baud rate, the FTDI latency timer (16 ms by default;
  1 ms is the usual Dynamixel tip, not set by the backend), and that the present position stays
  continuous when a joint crosses a turn boundary.
- **Orientation step size.** A workspace can bound orientation relative to a reference;
  `safety_box` and `max_delta_per_step` still limit only position (`xyz_m`) and joints
  (`joint_rad`). There is no device orientation step cap. The gym env bounds its own rotation
  increment with `rot_scale_rad` and projects policy targets into the configured workspace.
- **Step cap baseline.** The cap is measured from the last read state (at most 0.5 s old), not
  from the previous target, so the worst distance between a target and the true position is about
  twice the cap, without accumulating.
- **Bypassing the guard.** The legacy ROS1 bridge (`serl_http`) is an HTTP server: a client
  that talks to `bridge_url` directly does not pass through `Arm`, so none of the limits apply.
  Third-party backends must implement the `_servo_*`/`_move_*` hooks, not override the public methods.
- **A configured teleop is mandatory.** `connect()` fails and rolls the whole robot back when the
  teleop device is absent, including for deployment. Use a yaml without `teleop:` to deploy.
- **Version skew.** A daemon with these features breaks a pre-upgrade client at `arm.health()`
  (new fields), and an old client cannot read JPEG frames. Upgrade the client first.
- **`--jpeg-quality` is not recorded** in a training run's config; deploy cannot check that it
  matches the value used for training.
- **Teleop read clamp and clocks.** `teleop.<name>.read` over the daemon waits at most 2 s;
  `timestamp` is the robot PC's monotonic clock. `RemoteTeleop.get_observation()` returns the
  client's last sample, while the daemon's `robot.get_observation()` contains a `teleop` entry
  taken from the daemon's latest sample (events not consumed).

## Legacy

These modules predate the device layer and are kept for existing users. Planned work is in
[`docs/devices-roadmap.md`](devices-roadmap.md).

- **`controller/`**: pure-torch Cartesian impedance and pose controllers (`core/`, `adapters/`),
  wrapped for ManiSkill in `controller/simulator/maniskill/` (extra `sim`). They serve
  GPU-batched simulation, not real hardware. See [`real_world_rl/legacy/controller/README.md`](../real_world_rl/legacy/controller/README.md).
- **`controller/real/franka_bridge.py`** and **`gripper_server.py`**: a Flask/HTTP forwarder to
  the ROS1 `serl_franka_controllers` impedance controller and the Franka Hand. The `serl_http`
  backend is their client. Run the bridge on the ROS machine with
  `python -m real_world_rl.legacy.controller.real.franka_bridge --robot_ip <ip> --port 5000` (extra
  `legacy-bridge`; ROS1 is installed separately, see the module docstring).
- **`teleop/`**: the pre-device-layer teleop (`teleop/spacemouse/` on the old `pyspacemouse`
  `read_all()` API, extra `legacy-bridge`; `teleop/utils/` twist teleop for Pico over ZeroMQ;
  its inspector is `examples/run_twist_input_inspector.py`). The `Teleop` part ([step 7](devices.md#add-a-teleop-device)) supersedes the
  SpaceMouse code. The Pico path is not migrated yet and `real-world-rl`'s
  `--teleop-device pico` still imports these modules, so they stay until it is.
