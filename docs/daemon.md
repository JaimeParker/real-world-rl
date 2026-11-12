# The robot daemon

The daemon runs on the robot PC and serves device RPCs to RemoteRobot on a GPU
machine. For an actor process running on the robot host and a remote learner, see
[Remote actor and learner](remote-actor.md); that page covers training transport.
Station fields and safety contracts are in the [device reference](devices.md).

## Serve the robot

On the robot PC, from the repository root with the project environment active:

```bash
pip install -e '.[grpc]'   # plus the robot's backend extras
rwrl daemon --station robot.yaml --bind 0.0.0.0:50051 --token T --watchdog-s 0.5
```

| Flag | Default | Meaning |
|---|---|---|
| `--station` | the [station file](tutorial/02-setup.md) | robot or station yaml |
| `--robot` | every robot | serve one robot of a station; without it a station with several robots starts one daemon process per robot |
| `--bind` | the robot's `daemon.bind`, else `0.0.0.0:50051` | `host:port` to listen on (single robot only) |
| `--token` | the robot's `daemon.token`, else none | shared secret; clients must send the same one (gRPC metadata `x-robot-token`) (single robot only) |

| `--watchdog-s` | `0.5` | idle a servoing arm after this many seconds without actions |
| `--no-preflight` | off | skip the doctor run before serving |
| `--transport` | `grpc` | the only transport |
| `--jpeg-quality` | none | JPEG quality 1-100 for every camera's `color` image; see below |

With several robots and no `--robot`, the command is a supervisor: it starts
`rwrl daemon --station <station> --robot <name>` per robot (each in its own
process session, with `--watchdog-s`, `--jpeg-quality`, `--no-preflight` and `--transport`
passed on), forwards SIGINT/SIGTERM to them, waits for all and exits with the first non-zero
child exit code. `--bind`/`--token` are then an error (set `daemon.bind`/`daemon.token` per
robot), and so are two robots on the same `daemon.bind`. Killing the supervisor with SIGKILL
leaves the daemons running; stop them with SIGTERM.


`--jpeg-quality N` (needs `pip install -e '.[jpeg]'`, Pillow or OpenCV, on the daemon **and** on
every client) sends each camera's `color` image as a JPEG instead of raw bytes, which cuts the
bandwidth over WiFi. It is **lossy** and daemon-global: only `color` is compressed, `depth` and
all other fields stay raw, and the client decodes back to a `uint8 (H, W, 3)` array. The policy
therefore sees compressed pixels, so **train and deploy with the same value** (or none). The
value is reported by `robot.describe` as the top-level `image_encoding` (`{"jpeg": 80}`, or `null`
when raw) and as `robot.cameras["wrist"].image_encoding` on the client. A client without
Pillow/OpenCV fails at `connect()` with `SdkMissing: JPEG frames need Pillow or OpenCV`
(hint `pip install -e '.[jpeg]' (from the repository root)`).

Daemon and client ship from the same repo; upgrade the **client first, then the daemon**. A new
daemon's `arm.health` carries two extra fields (`safety_clips`, `safety_rejects`) that a
pre-upgrade client cannot parse, and enabling `--jpeg-quality` needs a client that can decode
JPEG. A client that predates JPEG support cannot decode the frames (its error was not checked).

What happens:

1. **Config.** The yaml is loaded. Then every config field that is empty (`""` or null) is
   filled from the environment variable of the same name in upper case, for example
   `FIRMWARE_PROFILE=v188`. `ROBOT_IP` and `CAN_INTERFACE` apply only if the yaml sets
   `robot_ip: ""` / `can_interface: ""`, because both have non-empty defaults. A camera
   with an empty `serial` takes the serial when exactly one camera of its backend is
   attached. The daemon, the doctor and `rwrl discover --station` all apply
   this autofill; `Robot.from_yaml` reads the yaml as written.
2. **Preflight.** Unless you pass `--no-preflight`, the full doctor (not `--mock`) runs
   and prints its report. If any check fails, the daemon prints `preflight failed; fix the
   items above or start with --no-preflight` and exits with code 1. The doctor connects
   and disconnects the robot while holding its device claim, so a running daemon on the same
   robot is reported as a failed `handshake`. Fake backends skip the OS checks
   (`backend 'fake' needs no OS device`), so demo or fake configs start without
   `--no-preflight`.
3. **Startup output** (from a fake-backend config):

   ```text
   franka (franka)
     arm: FakeArm [sdk=None, connected=False]
     gripper: FakeGripper [sdk=None, connected=False]
     cameras: PartGroup [sdk=None, connected=False]
       wrist: FakeCamera [sdk=None, connected=False]
     teleop: FakeTeleop [sdk=None, connected=False]
   serving franka 'franka' on 127.0.0.1:50061 (token required)
   ```

   The robot is connected only when a client calls `connect()`. At that point the daemon
   takes the device claim, and it releases the claim on `disconnect()`.
4. **Watchdog.** While the arm is in `CARTESIAN_SERVO` or `JOINT_SERVO`, the daemon checks
   every `watchdog_s / 4` how long it has been since the last action. Actions are
   `send_action`, `step`, `servo_cartesian` and `servo_joints`; entering servo mode also
   starts the clock. After more than `watchdog_s` with no action, the daemon calls
   `arm.stop()`, switches the arm to `IDLE`, and logs `watchdog: no action for 0.624s in
   joint_servo mode; stopping arm 'arm' and idling it`. Later servo commands raise the
   watchdog `DeviceFault` until the client calls `set_control_mode(...)` again.
   `move_to_*` calls (which hardware backends allow only in `IDLE`) are not watched.
5. **Teleop.** If the yaml has a `teleop:` section the daemon serves `teleop.<name>.read` and
   `teleop.<name>.reset`. Reads do not refresh the watchdog, and a client's `timeout_s` is
   clamped to 2 s so a client cannot park a gRPC worker thread. A `teleop` section is
   mandatory once present: `connect()` fails when the device is absent and takes the whole
   robot down with it, so a deployment that needs no teleop should use a yaml without the section.
6. **Ctrl-C / SIGTERM.** The daemon stops the transport, stops the watchdog, disconnects the
   robot and releases the claim. It never powers the robot off and never disables the
   motors (see [Safety rules](devices.md#safety-rules)).

`robot.describe` is what a client builds its mirror from. For the fake robot above (trimmed):

```text
arm: {"num_joints": 7, "supported_state_fields": ["tcp_pose", "tcp_vel", "joint_pos", "joint_vel", "tcp_force", "tcp_torque"], "control_mode": "idle", "safety": {"safety_box": [-0.5, -0.5, -0.5, 0.5, 0.5, 0.5], "joint_limits": [[-1.0, 1.0], ...], "max_delta_per_step": {"xyz_m": 0.05, "joint_rad": 0.1}}, "home_joints": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}
image_encoding: {'jpeg': 80}
camera cfg: {'name': 'wrist', 'serial': '', 'width': 64, 'height': 48, 'fps': 30, 'enable_depth': False, 'backend': 'fake'}
teleop: {'name': 'teleop', 'backend': 'fake', 'action_space': 'joint_abs', 'has_gripper': True, 'hold_s': 0.0}
```

`arm.safety` is `null` when no limit is configured; `arm.home_joints` is the yaml's value (or `null`).
The description also includes `arm.safety_capabilities`: `task_workspace_v1` for Cartesian
workspace enforcement, `accepted_cartesian_target_v1` for accepted-target holding, and, on
Franky, `stop_trace_v1`. Clients negotiate capabilities before using new behavior; older
servers cannot certify support. Connected teleop descriptions carry `device_identity` (USB
vendor/product and available serial/model); `RemoteTeleop.device_identity` exposes that same
identity for profile verification. Before connecting, identity may be unavailable.

Four RPCs supplement the existing arm commands: `arm.last_cartesian_target`,
`arm.begin_stop_trace`, `arm.stop_with_trace` (`post_stop_s=2.1`), and
`arm.last_stop_trace`. `last_cartesian_target` is a copy of the latest Cartesian target in the
current control segment that passed the arm's safety limits and was handed to the backend.
Any `stop()`, `stop_with_trace()`, control-mode switch or disconnect ends the segment, and
the value is then `null` until the next successful `servo_cartesian`. It is never measured
feedback. `piper_sdk` and `pyagxarm` still convert the target in the backend (millimetre or
milli-degree rounding, quaternion to RPY), so it is not exactly what those arms received.
`Arm.stop()` is a template: it clears the target and then calls the backend's `_stop()`.
A backend implements `_stop()`, never `stop()`. `begin_stop_trace` starts bounded sampling
on the existing connection;
SDK fingerprints/static metadata are collected outside the stop/service locks and cached
per connection. Prior sampler joins also occur outside the stop lock.

`stop()` retains its ordinary Franky error semantics: failures are noted in health without
raising. Only `stop_with_trace` raises `DeviceFault` for stop failure, retaining FAIL evidence
in `last_stop_trace`. It sends stop before attempting to create a missing recorder; preparation
failure produces BLOCKED evidence after direct stop. Read/stop RPCs remain available during
post-stop observation, while control writes are refused. Slow camera acquisition in
`robot.step` does not hold the dispatch lock needed by `arm.stop`.

Stop traces record `control_mode` and `tracker_active` at stop start, and each sample's SDK
`robot_mode`. An already IDLE/inactive arm yields BLOCKED (`no active motion at stop`), even
if every pose is stationary. Robot-mode faults and stop failures yield FAIL; missing or
unknown mode/coverage evidence cannot yield PASS. Sampled stop acceptance is not a
continuous-time physical guarantee.

## Use the robot from the GPU machine

On the GPU machine, from this repository root with its project environment active:

```bash
pip install -e '.[grpc]'
```

```python
import real_world_rl.server  # registers robot type "remote"
from real_world_rl.devices import ControlMode, Robot

robot = Robot.from_dict({"type": "remote", "address": "robot-pc:50051", "token": "T"})
robot.connect()                       # builds arm/gripper/cameras from the daemon's describe
print(robot.describe_text())
robot.arm.set_control_mode(ControlMode.JOINT_SERVO)
q = robot.arm.read_state()["joint_pos"]
obs = robot.step({"arm": {"joint_pos": q}, "gripper": {"gripper": [1.0]}})  # action, then observation
print(robot.health())                 # Health(ok=True, detail='rtt=1.2 ms, watchdog_trips=0', ...)
robot.arm.set_control_mode(ControlMode.IDLE)
robot.disconnect()
```

Without `import real_world_rl.server`, `Robot.from_dict` raises
`unknown robot type 'remote'; registered: ['franka', 'piper']`.

- Actions are nested per part, and the arm key depends on the control mode: `tcp_pose`
  (7 values: xyz plus quaternion xyzw) in `CARTESIAN_SERVO`, `joint_pos` in
  `JOINT_SERVO`. The gripper takes a normalized width in [0, 1].
- `step()` returns `{"arm": {...}, "gripper": {"gripper_pos": ...}, "cameras": {<name>:
  {"color": ...}}}`. The arm fields are the backend's `SUPPORTED_STATE_FIELDS`.
- Other `RemoteRobotConfig` keys: `timeout_s` (5.0), `reconnect_attempts` (5),
  `reconnect_backoff_s` (0.5). After a dropped connection, the client reconnects and
  retries only read-only calls. Commands are never retried: reconnect and set the control
  mode again.
- For single-machine use without gRPC, wrap a local robot in-process:

  ```python
  from real_world_rl.devices import Robot
  from real_world_rl.server import RemoteRobot, RobotService
  from real_world_rl.server.transport import LocalClient

  service = RobotService(Robot.from_yaml("robot.yaml"), watchdog_s=0.5)
  robot = RemoteRobot.from_client(LocalClient(service))   # same API as above, incl. watchdog
  ```

- The daemon's device-side facts are mirrored on the client:
  `robot.arm.safety_limits` (the dict under `arm.safety` above, or `None`),
  `robot.arm.home_joints` (list or `None`), `robot.cameras["wrist"].image_encoding`, and
  `robot.teleop` (`None` without a `teleop:` section). The client does not enforce limits
  itself (`RemoteRobotConfig` ignores `safety_box`, `joint_limits`, `max_delta_per_step` and
  `teleop`); the daemon does.
- A command outside the limits behaves as follows on the client (run it before `robot.disconnect()`):

  ```python
  import numpy as np
  from real_world_rl.devices import SafetyViolation

  robot.arm.set_control_mode(ControlMode.JOINT_SERVO)
  robot.arm.servo_joints(np.full(7, 5.0))      # no error: clipped on the daemon, safety_clips += 1
  robot.arm.set_control_mode(ControlMode.IDLE)
  try:
      robot.arm.move_to_joints(np.full(7, 5.0))   # outside joint_limits
  except SafetyViolation as error:
      print(error.message, error.hint)            # nothing was sent; the arm and its mode are unchanged
  print(robot.arm.health().safety_clips, robot.arm.health().safety_rejects)   # cumulative counters since the daemon started, e.g. 2 1
  print(robot.remote_health()["arm"]["safety_clips"])                         # the same counters from the daemon's robot.health
  ```

  `SafetyViolation` is a `DeviceError` (not a `DeviceFault`): it does not latch a fault, does
  not need `recover()`, and does not refresh the daemon watchdog. `robot.health()` is a
  `Health`; the per-arm counters live in `robot.arm.health()` and `robot.remote_health()`.
- `real-world-rl` wraps this robot in a gym environment.

## Use the robot locally (no daemon)

On the robot PC, from the repository root with the project environment active,
you can drive the device layer directly:

```python
import numpy as np
from real_world_rl.devices import ControlMode, Robot

robot = Robot.from_yaml("robot.yaml")
robot.connect()
robot.arm.set_control_mode(ControlMode.CARTESIAN_SERVO)
pose = robot.arm.read_state()["tcp_pose"]
robot.send_action({"arm": {"tcp_pose": pose}, "gripper": {"gripper": np.array([0.0])}})
obs = robot.get_observation()
robot.arm.set_control_mode(ControlMode.IDLE)
robot.disconnect()
```

This path has no watchdog and takes no device claim. Those are daemon features. The safety
limits, however, are enforced here too: `Robot.from_yaml` builds the arm with the yaml's
`safety_box`/`joint_limits`/`max_delta_per_step`, so `robot.arm.safety_limits`,
`robot.arm.health().safety_clips` and `SafetyViolation` behave exactly as through the daemon.

## Supervising a single-process robot run

On the robot host, from the repository root with the full project environment active,
use `rwrl supervise` and `rwrl record`. Without a daemon, the robot process drives the arm through the in-process SDK, so nothing outside it can call `arm.stop()`. `rwrl supervise` runs it as the child of a separate watchdog process:

    rwrl supervise --log runs/<run>/supervise.jsonl -- rwrl record --config hil.yaml ...

| Option | Default | Meaning |
|---|---|---|
| `--heartbeat-timeout-s` | `2.0` | silence after which SIGINT, then SIGTERM, then SIGKILL go to the child's process group |
| `--grace-s` | `3.0` | SIGINT to SIGTERM |
| `--kill-s` | `2.0` | SIGTERM to SIGKILL |
| `--startup-s` | `30.0` | no heartbeat required until the first message or this elapses |
| `--long-op-s` | `60.0` | silence allowed after the child announces a long operation (an env reset) |
| `--log` | none | jsonl: pid, escalations with monotonic timestamps, exit code/signal |

The child runs in its own session and receives the write end of a pipe in `RWRL_HEARTBEAT_FD`. `real_world_rl.supervise.Heartbeat.from_env()` is the child side: `beat()` once per step, `begin_long_operation()` before a reset. Without the variable it does nothing. Ctrl-C, SIGTERM or a closed terminal reach only the supervisor, which forwards SIGINT and waits for the child; it exits with the child's code, non-zero after an escalation. If the supervisor dies, the child gets SIGINT and its next heartbeat raises `SupervisorLost`. Linux only (prctl, process groups).

The supervisor's only lever is the kill. libfranka then drops the control connection and the robot stops at the firmware level, in `Reflex`. Clear it in Desk or with `rwrl --station robot.yaml recover` (it calls the arm's `recover()`); `rwrl doctor --fix` repairs only the host and software and never recovers the arm. A controlled out-of-process `stop()` exists only on the daemon route (`--watchdog-s`). The in-process first layer is `StepWatchdog`, enabled with `real_robot.watchdog_s` (see `docs/real-world-experiments.md`).

