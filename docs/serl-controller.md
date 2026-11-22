# SERL controller (`cartesian_controller: serl`)

rwrl can run SERL's own Franka controllers: the Cartesian impedance controller and the joint reset
of `serl_franka_controllers`, the ros_control plugin SERL and HIL-SERL used with franka_ros. rwrl
runs them without ROS. `native/serl_controller/` is a C++/pybind11 package whose motions derive
from franky's own motion classes, and franky itself is unmodified. The golden tests compile
SERL's original sources and require the two to agree on every tick (abs and rel ≤ 1e-12). The
port is bit-identical. See [native/serl_controller/README.md](../native/serl_controller/README.md).

Reference versions: HIL-SERL (`rail-berkeley/hil-serl` @ `c32939b`), SERL (`rail-berkeley/serl`
@ `1fa2af7`) and `serl_franka_controllers` @ `1f140ef`. This page covers the Franka control and
SpaceMouse teleop feel. It does not cover the fidelity of HIL-SERL's RL training; see
[Known differences from HIL-SERL](#known-differences-from-hil-serl-rl-side-out-of-scope).
The hardware checklist is in [SERL controller: robot validation](serl-robot-validation.md).

## Choosing a controller

Two independent settings decide how a SpaceMouse moves a Franka:

- `cartesian_controller` in the **station** (robot PC) decides what executes a Cartesian servo
  target: `impedance` (franky's torque tracker, the default), `internal` (the robot's own
  controller, through velocity motions) or `serl` (SERL's impedance controller). See
  [devices: Cartesian servo controller](devices.md#station-file-reference).
- `spacemouse_mode` in the **preset** or `--spacemouse-mode` on the command line decides how the
  puck becomes a target: `tracker` (the default: button 0 takes over, a persistent command with a
  lead cap) or `serl` (HIL-SERL's intervention: moving the puck intervenes, and every step targets
  the measured pose plus action × scale). See [teleop: SpaceMouse](teleop.md#spacemouse).

| `cartesian_controller` \ SpaceMouse | `tracker` | `serl` |
|---|---|---|
| `impedance` | Default. The tracker's command, followed by franky's `CartesianImpedanceTracker` with the `control:` gains (`rwrl setup` writes SERL-like ones). Works on a non-realtime kernel. | HIL-SERL's intervention semantics, executed by franky's tracker: the gains approximate SERL's but the controller is not SERL's. |
| `internal` | The tracker's command, executed by the robot's own controller (velocity motions closing the error in 0.2 s, ≤ 0.25 m/s). Wants `realtime: enforce`. Not validated on hardware yet. | Each per-step jump of the HIL-SERL target becomes a velocity motion. Same realtime caveat. |
| `serl` | SERL's impedance controller follows the tracker's command. Needs franky + `serl_controller` built together and `realtime: enforce`. | Closest to HIL-SERL: HIL-SERL's intervention, SERL's controller and SERL's joint reset. Needs the pair and `realtime: enforce`. |

Switching is configuration only, and no code changes:

```yaml
# station (robot PC), e.g. local/station.yaml; restart `rwrl daemon` afterwards
cartesian_controller: serl        # impedance | internal | serl
realtime: enforce                 # internal and serl want a PREEMPT_RT kernel
robot_model: fr3                  # serl: the joint limits of SERL's joint reset
# preset (actor side), or --spacemouse-mode on the command line
spacemouse_mode: serl             # tracker | serl
```

Run `rwrl doctor` after every change. The arm's `control_metadata` records `cartesian_controller`,
and a SpaceMouse run records its compared `teleop_control` block. Resuming a saved experiment
after switching either one is therefore refused (`effective control/SDK metadata differs from the
saved experiment`, or a `teleop_control` mismatch) rather than silently continuing with other
dynamics. To go back, set the previous value and restart the daemon. The setup wizard keeps a
`station.yaml.bak.<UTC>` copy, and CLI edits a `station.yaml.<id>.bak` copy, of every file they
replace.

## Setup

### Station fields

```yaml
type: franka
arm_backend: franky                # serl needs franky
cartesian_controller: serl
realtime: enforce                  # SERL ran under franka_control's realtime_config: enforce
robot_model: panda                 # panda (default) | fr3; confirm in Desk, see below
serl: null                         # SERL compliance parameters; null = the .cfg defaults (table below)
serl_auto_recover: null            # null = {enabled: true, max_per_episode: 5}; see Auto-recovery
load: null                         # optional payload, every mode (see devices.md)
joint_limits: [...]                # still refuse move_to_joints targets outside them
```

- `robot_model` (`panda` | `fr3`, default `panda`) picks franka_description's joint limits for
  the soft-limit stage of SERL's joint reset. The setup wizard also offers that model's datasheet
  `joint_limits` as the default: Panda
  `[[-2.8973, 2.8973], [-1.7628, 1.7628], [-2.8973, 2.8973], [-3.0718, -0.0698], [-2.8973, 2.8973], [-0.0175, 3.7525], [-2.8973, 2.8973]]`,
  FR3 `[[-2.9007, 2.9007], [-1.8361, 1.8361], [-2.9007, 2.9007], [-3.0770, -0.1169], [-2.8763, 2.8763], [0.4398, 4.6216], [-3.0508, 3.0508]]`.
  An existing station keeps its own `joint_limits`.
- `serl` holds SERL's parameters (see [Parameters](#parameters) below). `control:`'s Cartesian
  tracker keys do nothing in this mode. The joint keys, `max_delta_tau` and
  `gains_time_constant` still set the joint servo tracker.
- `serl_auto_recover: {enabled, max_per_episode}` (booleans and an integer ≥ 0; unknown keys are
  errors).

### Installing franky and serl_controller

`serl_controller`'s motions derive from franky's C++ classes, and the extension links franky's own
`libfranky.so` and the libfranka franky was built against. The two must therefore come from the
**same build**; the official franky wheel does not work with it. franky itself stays unmodified
(v2.0.0).

**On the robot PC, natively (the supported path).** A Franka NUC is plain Ubuntu; no Docker is
needed. With the project environment active, from the repository root:

```bash
LIBFRANKA_VERSION=0.17.0 bash native/serl_controller/scripts/install-on-ubuntu.sh
```

The script installs the apt build packages; CMake ≥ 3.24, scikit-build-core and pybind11 3.0.4
(franky's) into the environment; clones franky v2.0.0 and checks the commit and that the checkout
is unmodified; builds Eigen, pybind11, ruckig and libfranka `LIBFRANKA_VERSION` (with poco and,
from 0.14, pinocchio) into `/usr/local` with franky's own, unmodified `docker/common/install-*`
scripts (an installed libfranka of that version is reused; another version is only replaced with
`REPLACE_LIBFRANKA=1`); pip-installs franky from source against it (version
`2.0.0+libfranka.X`); builds and installs `serl_controller` against that franky with
scikit-build-core (no build isolation); then runs CTest (including both golden comparisons with
SERL's original sources), the Python binding tests, a smoke import and the doctor's
`sdk:serl_controller` check. Logs are in `~/.cache/rwrl-native-build/logs/` (`WORK=` moves
them). Other knobs: `SKIP_APT=1`, `SKIP_DEPS=1`, `SKIP_TESTS=1`, `RETRIES=N` (network steps),
`PIP_INDEX_URL`. It was verified end to end in a clean Ubuntu 22.04 (CPython 3.10.12, x86_64,
libfranka 0.17.0); Ubuntu 20.04 needs a separately installed Python ≥ 3.10 for this project.

`LIBFRANKA_VERSION` follows from the robot server version (read it from `rwrl doctor`'s
`libfranka mismatch: robot server N, installed library M` line, or cross-check Desk → Settings →
System with Franka's [compatibility matrix](https://frankarobotics.github.io/docs/compatibility.html)).
franky v2.0.0's `doc/compatibility.json` gives the minimum libfranka per server version; the
suggested versions are the ones franky publishes wheels for:

| robot server version | libfranka range | suggested `LIBFRANKA_VERSION` |
|---|---|---|
| 10 | ≥ 0.18.0 | 0.21.2 |
| 9 | 0.15.0 – 0.17.x | 0.17.0 (verified) |
| 8 | 0.14.1 – 0.14.x | 0.14.2 |
| 7 | 0.13.3 – 0.14.0 | 0.13.3 |
| 6 | 0.10.0 – 0.13.2 | 0.12.1 |
| 5 | 0.9.1 – 0.9.x | 0.9.2 |
| 4 | 0.8.0 – 0.9.0 | 0.8.0 |
| ≤ 3 | < 0.8 | not supported (serl_controller needs libfranka ≥ 0.8) |

**Install order.** With `cartesian_controller: serl`, the script is the franky installer: run it
instead of `rwrl doctor --install-sdk` or `pip install franky-control`, which would put the
official wheel back and break the pair. The doctor refuses `--install-sdk` for a station with
`cartesian_controller: serl` (`sdk.franky.serl` BLOCKED, with the rebuild command), and for other
stations warns in the plan when a `serl_controller` is installed. An official wheel installed
earlier is fine: the script uninstalls it first. `pip install -e '.[franky]'` later keeps the
source build (its version satisfies the requirement).

**Wheels built elsewhere (optional).** `native/serl_controller/docker/build-on-host.sh` builds a
manylinux wheel pair (CPython 3.10, libfranka 0.17.0 by default) on a build machine with Docker,
for several identical robot PCs; see the native README. Install both wheels of one build
directory together:

```bash
pip uninstall -y franky-control serl-controller
W=path/to/wheels                                      # one build directory
(cd $W && sha256sum -c SHA256SUMS)
pip install $W/franky_control-*.whl $W/serl_controller-*.whl
rwrl doctor                                            # sdk:serl_controller must PASS
```

`serl_controller` ships a `build_info.json` with the franky version and commit, the sha256 of the
`libfranky.so` it was linked against, the layout (`wheel`: the libfranka bundled in the franky
wheel, `libfranka_file`; `source`: the system libfranka franky was built against, `libfranka_path`
and its sha256) and a build id. A mismatched pair fails `sdk:serl_controller` in the doctor (also
when the system libfranka was replaced after the build). It also fails at import with a
`VersionMismatch` that names the missing library. To go back to the official franky wheel, switch
the station to `impedance` or `internal`, then run `rwrl doctor --install-sdk`.

### Realtime kernel

SERL's 1 kHz torque loop ran with realtime enforced. Without a PREEMPT_RT kernel, scheduling jitter
can trip libfranka's communication reflexes. Install a PREEMPT_RT kernel on the robot PC by hand.
On Ubuntu with Ubuntu Pro that is `sudo pro enable realtime-kernel`; otherwise follow Franka's
"Setting up the real-time kernel" guide. Then run `rwrl doctor --fix` to write the rtprio and
memlock limits, log in again, and set `realtime: enforce`.

### Doctor checks

With `cartesian_controller: serl` (or `internal`), `rwrl doctor` adds these rows to the robot's
checks:

| Row | Layer | Result | Meaning and fix |
|---|---|---|---|
| `sdk:serl_controller` | 1 sdk | PASS `serl_controller X and franky_control Y from the same build (...)` / FAIL | `build_info.json` against the installed franky: version, `libfranky.so` sha256 and the libfranka (the wheel's bundled file, or the system library of a source build and its sha256), without importing either extension. On FAIL, rerun the native install script (or reinstall both wheels from one build directory). |
| `config:serl_control` | 1 sdk | PASS / WARN `cartesian_controller: serl ignores control: [...]` | The station's `control:` sets franky Cartesian tracker gains that serl mode never uses. Move them to `serl:` (SERL names) or drop them. |
| `mode:serl_realtime` | 5 mode | PASS / WARN `... with realtime: ignore ...` | Install a PREEMPT_RT kernel and set `realtime: enforce`. |
| `mode:internal_realtime` | 5 mode | PASS / WARN (`cartesian_controller: internal` only) | libfranka's Cartesian velocity interface aborts on any 1 kHz discontinuity. Install a PREEMPT_RT kernel and set `realtime: enforce`. |
| `mode:robot_model` | 5 mode | PASS `robot_model: fr3 (robot URDF: fr3)` / WARN | Every mode. It compares `robot_model` with the robot (see below). A WARN names the mismatch, or says that the model could not be cross-checked in serl mode. |

## robot_model: panda or fr3

`mode:robot_model` cross-checks the station value with what the connected robot reports. It uses
`Robot.server_version` when the franky build has it. franky 2.0 binds no robot server version (only
`Gripper.server_version`, the hand's own protocol), so it then uses the joint names in the URDF the
robot serves (`Robot.model_urdf`, libfranka ≥ 0.15): `fr3_joint*` means `fr3` and `panda_joint*`
means `panda`. When neither is known, the check passes in impedance and internal modes and warns in
serl mode (`could not be cross-checked`), because only SERL's joint reset uses the limits.

The robot server version also identifies the model. Franka's compatibility matrix maps server
versions ≤ 5 to robot system 4.x (Franka Emika Robot, `panda`) and ≥ 6 to system 5.x (Franka
Research 3, `fr3`). libfranka ≥ 0.10 only connects to system 5.x. For example, a handshake that
reports server version 9 belongs to a system 5.x robot, so most likely `fr3`. **Confirm it in Desk**:
the system version under Settings → System (4.x = Franka Emika Robot, 5.x = Franka Research 3)
and the robot type shown there. Then set `robot_model` in the station (`rwrl setup --robot-model`
asks it before the joint limits, so the datasheet default matches the model).

## Auto-recovery (`serl_auto_recover`)

HIL-SERL's `FrankaEnv` posts `/clearerr` before every `/pose`. A reflex (for example a gentle
collision) that stopped SERL's controller is therefore cleared, and the arm keeps following the
next target. rwrl does the same on the next servo target:

1. The controller is found stopped with a motion error. An ordinary `stop()` or a preempted motion
   is not a reflex and is not auto-recovered.
2. If `enabled` and fewer than `max_per_episode` auto-recoveries happened this episode, the arm logs
   `arm '<name>': SERL impedance controller stopped (<error>); auto-recovery N/M this episode:
   clearing errors and restarting the controller at the measured pose`. It then calls libfranka's
   automatic error recovery (`recover_from_errors`, what franka_control's error_recovery action
   runs), starts a **fresh** `SerlCartesianImpedanceMotion` at the measured pose, waits up to
   50 ms for its first tick (so `starting()` does not overwrite the new target) and applies the
   target.
3. If the recovery fails (for example the external activation device is pressed), or the cap is
   reached, the servo call raises a recoverable `DeviceFault`. Its message is `franka SERL
   impedance controller stopped: <error> (serl_auto_recover could not clear it)` or `...
   (serl_auto_recover: max_per_episode 5 reached, 5 auto-recoveries this episode)`. Servoing is
   then refused until `rwrl recover`.

**Episode.** The per-episode count goes back to 0 when the arm enters Cartesian servo
(`set_control_mode(CARTESIAN_SERVO)` from another mode) and on `reset_serl_auto_recovery()`,
which `RealRobotEnv` calls at the end of every reset. `arm.serl_auto_recovery` reports `enabled`,
`max_per_episode`, `episode_count`, `total_count` and `last` (error text, count, time and the
TCP pose at recovery). It is also available over the daemon (`arm.serl_auto_recovery`,
`arm.reset_serl_auto_recovery`) and through `RemoteArm`. `control_metadata["serl"]["auto_recover"]`
records the policy. Set `enabled: false` to make every reflex a fault, as before.

**Exact difference from HIL-SERL.** In HIL-SERL the controller *instance* survives the reflex.
After franka_control's error recovery, franka_control_node restarts the control loop and
controller_manager resets the running controllers, which calls `stopping()` / `starting()` on the
same `CartesianImpedanceController`. `starting()` only sets the pose target and the nullspace
posture to the measured state. The filtered stiffness, damping and nullspace gains keep their
values, as do the integrator `error_i` and the parameters. rwrl starts a fresh controller instead,
as on every rwrl start and like SERL's own relaunch of `impedance.launch`. K, D and Ki ramp from 0
again over about 0.2 s, the nullspace stiffnesses start from 20, and `error_i` starts at 0. The
current effective parameters, including live `update_serl_params` updates, are kept. So right
after an auto-recovery the arm is softer for about 0.2 s than in HIL-SERL. HIL-SERL also has no
per-episode cap.

## What runs

| rwrl call | SERL | How it runs |
|---|---|---|
| `set_control_mode(CARTESIAN_SERVO)` | `start_impedance` (roslaunch `impedance.launch`) | a fresh `SerlCartesianImpedanceMotion` with the current parameters; `robot.move(motion, asynchronous=True, limit_rate=True, cutoff_frequency=100.0)` (franka_control's `rate_limiting: true`, `cutoff_frequency: 100`) |
| `servo_cartesian(pose)` | `/pose` → `equilibrium_pose` | `motion.set_target(xyz, quat_xyzw)`, base frame (`O_T_EE`), applied at the next tick |
| `update_serl_params(partial)` | `/update_param` → dynamic_reconfigure | partial merge, clamped into the `.cfg` ranges, applied at the next tick (or at the next start) |
| `move_to_joints(q)` (IDLE) | `/jointreset` (`joint.launch`) | `SerlJointPositionMotion`: linear blend q = (t/10)·q_target + ((10−t)/10)·q_start over 10 s, then hold, passed through franka_hw's `PositionJointSoftLimitsInterface` (below), on the robot's internal joint impedance controller with the same rate limiting and cutoff; finished as below |
| `stop()`, leaving the mode | `stop_impedance` (kills the launch) | franky's `TorqueStopMotion` (ramps into joint damping and finishes) |
| `move_to_pose` | (none; SERL moves with the impedance controller) | Ruckig `CartesianMotion`, unchanged |
| joint servo | (none) | franky's `JointImpedanceTracker`, unchanged |

Joint reset finishing follows `franka_server.reset_joint`: wait 2 s, then poll the joints once
per second until `np.allclose(target - q, 0, atol=1e-2, rtol=1e-2)` or 30 more polls. A timeout
is logged ("SERL joint reset TIMEOUT") and the reset still completes, as in SERL. The motion is
then ended with `MotionFinished` once its 10 s blend is over (holding the target; SERL kills the
controller wherever it is), errors are cleared, and `arm.last_joint_reset` records `target`,
`reached`, `polls`, `max_abs_error_rad`, `duration_s`, `soft_limited_ticks` and `robot_model`
(also over the daemon: `arm.last_joint_reset`, `RemoteArm.last_joint_reset`). Like
franka_server, the impedance controller is restarted by the next
`set_control_mode(CARTESIAN_SERVO)`. A reflex during the reset raises a recoverable `DeviceFault`.
`blocking=False` starts the blend and returns; it holds until the next `stop()` or mode change.
Through `RemoteArm`, a blocking `move_to_joints` on a `cartesian_controller: serl` daemon waits
up to 60 s (`SERL_JOINT_RESET_DEADLINE_S`); every other call keeps `timeout_s`.

### Soft joint limits (franka_hw's `PositionJointSoftLimitsInterface`)

franka_hw passes every joint position command through
`joint_limits_interface::PositionJointSoftLimitsHandle` (ros_control), built from the URDF
franka_control loads (franka_description `robots/<robot_model>/joint_limits.yaml` through the `franka-limits` macro: `<limit lower upper
velocity>`, `<safety_controller k_position="100" soft_lower_limit=lower soft_upper_limit=upper>`).
The joint reset motion reproduces it at the same place in the command path: on the period-0
tick the limits are reset (the command is the start posture), on every other tick they run right
after the controller's update. Starting from the measured position on the first enforced tick and
from the previous sent command afterwards, each joint's command is clamped to
`[pos + v_min·dt, pos + v_max·dt] ∩ [lower, upper]` with `v = clamp(−k_position·(pos − soft_limit),
±max_velocity)`. So the blend never moves a joint faster than its URDF velocity limit per tick,
slows down exponentially (k_position 100 /s) as it nears a limit and stops at it, and a joint that
starts beyond a limit is pulled back. The port is ported line by line and golden-tested against
the original ros_control header (see the native README). The station's `robot_model` picks the
table (`panda`, the default, as franka_control's `robot:=panda`, or `fr3`; see
[robot_model](#robot_model-panda-or-fr3)). Station `joint_limits` still refuse targets outside
them before the motion starts.

A reflex that ends the Cartesian controller is cleared and the controller restarted on the next
servo target, up to `serl_auto_recover.max_per_episode` times per episode (see
[Auto-recovery](#auto-recovery-serl_auto_recover)); past that, or with auto-recovery disabled,
the next servo call raises a recoverable `DeviceFault`, servoing is refused until `recover()`
(`rwrl recover`), and the next target then starts a fresh controller, like SERL's relaunch.
Every start is a fresh controller (stiffness and damping ramp from 0 over ~0.2 s,
`filter_params = 0.005`), as in SERL.

## Parameters

The station `serl:` block holds the robot's defaults with the names, defaults
and ranges of SERL's `cfg/compliance_param.cfg` (`real_world_rl.devices.control.SERL_COMPLIANCE_PARAMS`);
omitted keys keep the `.cfg` defaults:

| key | default | range |
|---|---|---|
| `translational_stiffness` | 2000 | 0–6000 |
| `translational_damping` | 89 | 0–400 |
| `rotational_stiffness` | 150 | 0–300 |
| `rotational_damping` | 7 | 0–30 |
| `nullspace_stiffness` | 0.2 | 0–100 |
| `joint1_nullspace_stiffness` | 100 | 0–100 |
| `translational_clip_{x,y,z}`, `translational_clip_neg_{x,y,z}` | 0.01 | 0–0.1 |
| `rotational_clip_{x,y,z}`, `rotational_clip_neg_{x,y,z}` | 0.05 | 0–0.1 |
| `translational_Ki`, `rotational_Ki` | 0 | 0–100 |

The translational error is clipped per axis into `[-clip_neg, clip]`, the orientation error is
the quaternion vector part (sin(θ/2)·axis). Values outside a range are **clamped**, as
dynamic_reconfigure does (HIL-SERL relies on it: its `PRECISION_PARAM` sends rotational clips of
0.5, which run as 0.1); the clamped keys are logged and listed in
`control_metadata["serl"]["clamped"]`. Unknown keys and non-finite values are errors.

`arm.update_serl_params({...})` changes parameters while the controller runs (or stores them
for the next start); updates persist until changed, like dynamic_reconfigure values on the ROS
parameter server. `arm.serl_params` is the full effective set. Both work through the daemon
(`arm.update_serl_params`, `arm.serl_params`) and `RemoteArm`. `control_metadata["serl"]` reports
the `serl_controller` version and build, the station defaults (`station_params`), the current
`params`, `clamped`, `limit_rate` and `cutoff_frequency`. The `control:` block's Cartesian tracker
keys (`translational_stiffness`, `rotational_stiffness`, `cartesian_damping`,
`translational_error_clip`, `rotational_error_clip`) are ignored in this mode; loading the station
logs a warning naming them and `rwrl doctor` reports `config:serl_control` as WARN. The joint keys,
`max_delta_tau` and `gains_time_constant` still set the joint servo tracker.

### Profiles

A task preset can name parameter sets under `real_robot.serl_profiles`. As in
HIL-SERL (`requests.post(url + "update_param", json=PROFILE)`), switching to a profile is a
**partial update on top of the current parameters**: keys the profile leaves out keep whatever was
applied before. `real_world_rl.devices.control.serl_profile_update(profiles, name)` gives the
update to pass to `arm.update_serl_params`; `apply_serl_profile(current, profiles, name)` computes
the result. When profiles set different keys, validation logs a warning: the result then depends
on the switching order. HIL-SERL's `ram_insertion` example, as profiles:

```yaml
real_robot:
  serl_profiles:
    compliance:               # HIL-SERL ram_insertion COMPLIANCE_PARAM (the policy's episodes)
      translational_stiffness: 2000
      translational_damping: 89
      rotational_stiffness: 150
      rotational_damping: 7
      translational_Ki: 0
      translational_clip_x: 0.0075
      translational_clip_y: 0.0016
      translational_clip_z: 0.0055
      translational_clip_neg_x: 0.002
      translational_clip_neg_y: 0.0016
      translational_clip_neg_z: 0.005
      rotational_clip_x: 0.01
      rotational_clip_y: 0.025
      rotational_clip_z: 0.005
      rotational_clip_neg_x: 0.01
      rotational_clip_neg_y: 0.025
      rotational_clip_neg_z: 0.005
      rotational_Ki: 0
    precision:                # HIL-SERL ram_insertion PRECISION_PARAM (resets)
      translational_stiffness: 2000
      translational_damping: 89
      rotational_stiffness: 250
      rotational_damping: 9
      translational_Ki: 0.0
      translational_clip_x: 0.1
      translational_clip_y: 0.1
      translational_clip_z: 0.1
      translational_clip_neg_x: 0.1
      translational_clip_neg_y: 0.1
      translational_clip_neg_z: 0.1
      rotational_clip_x: 0.5    # runs as 0.1 (clamped, as in HIL-SERL)
      rotational_clip_y: 0.5
      rotational_clip_z: 0.5
      rotational_clip_neg_x: 0.5
      rotational_clip_neg_y: 0.5
      rotational_clip_neg_z: 0.5
      rotational_Ki: 0.0
```

These clips were tuned for HIL-SERL's RAM insertion setup; they are an example of the format,
not values for another task. Profiles are validated when the preset loads; switching them at
reset (HIL-SERL posts `PRECISION_PARAM` before a reset and `COMPLIANCE_PARAM` after it) is the
env's job; `RealRobotEnv` does not switch them itself (it records them in its metadata). The two
profiles above set the same keys, so the order does not matter for them.


## Deliberate differences from SERL

Stops are graceful (franky's `TorqueStopMotion`, and the joint reset ends in its hold phase)
instead of killing ROS processes. `set_target` rejects non-finite poses and non-unit
quaternions. Auto-recovery starts a fresh controller instead of calling `starting()` on the old
one (above). Validation on hardware: see [SERL controller: robot validation](serl-robot-validation.md).

## SpaceMouse in serl mode: the task frame

`--spacemouse-mode serl` reproduces HIL-SERL's `SpaceMouseExpert` + `SpacemouseIntervention`.
HIL-SERL applies no operator rotation: the device twist reaches the env in the robot **base
frame**, which assumes an upright base. rwrl rotates the twist (translation and rotation by the
same matrix) into the base frame before the end-effector target is formed, on every SpaceMouse
path; none of the controllers rotates again:

| Mode | Rotation into the base frame | `teleop_control` `input_frame` |
|---|---|---|
| serl (default) | the station `task_frame` rotation `base_R_table` alone (no profile yaw) | `task_frame` |
| serl, `use_profile_rotation: true` | the profile's `input_rotation` (`base_R_table · Rz(yaw)`) | `profile` |
| serl, no `task_frame` | none (raw base frame, as HIL-SERL) | `base` |
| tracker, a profile selected | the profile's `input_rotation` | (profile) |
| tracker, no profile, a `task_frame` | `base_R_table` alone | `task_frame` |
| tracker, neither | none | (base) |

With an upright base and no `task_frame` this is exactly HIL-SERL. On a rolled mount (for example
a base turned about its x axis by about 45°, `base_R_table = Rx(±45°)`) the raw base frame would
make "push up" move along the tilted base z axis; with the measured `task_frame` (`rwrl workspace
frame set --from-points`, see the station tutorial) "up" is vertical again. Policy actions are
already base-frame deltas and are never rotated.

**Placement (no calibration).** Put the SpaceMouse as HIL-SERL does: its front towards task-frame
+x (table forward, away from the robot base), its left towards +y, up = +z (vertical). Then check
the six single axes: each push moves the TCP along, and each twist turns it about, the same
task-frame axis. `rwrl workspace capture` reports the TCP position in the task frame
(`task_position`) before and after a one-axis push. When the SpaceMouse cannot sit that way, set
`use_profile_rotation: true` so that the calibrated profile yaw is applied too:

```yaml
# teleop_options_yaml (or the station's teleop.options)
input_rotation: [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
use_profile_rotation: true        # serl mode only; default false (task_frame rotation only)
```

`use_profile_rotation` must be `true` or `false`. It is a serl-mode option: tracker mode always
applies the profile rotation and rejects `use_profile_rotation: false`. The compared
`teleop_control` block records the choice and the input frame, so resuming a run recorded with
another frame is refused (a `teleop_control` mismatch) instead of silently changing the drive
frame. To continue such a run with its old behaviour, restore it under the code that recorded it,
or start a new run.

In serl mode the target is the measured pose plus action × `xyz_scale_m` / `rot_scale_rad` every
step (for example 0.02 m and 0.1 rad, SERL's peg-insertion values). There is no tracker and no lead cap.
Together with `cartesian_controller: serl`, a full deflection asks SERL's controller for 2 cm per
step, and SERL's `translational_clip_*` (0.01 m by default) limits the force that error produces.

## Known differences from HIL-SERL (RL side, out of scope)

These concern HIL-SERL's training stack, not the controller or the teleop feel. They are listed so
that nobody mistakes the controller port for a full HIL-SERL reproduction. None is changed here.

- **RelativeFrame action transform.** HIL-SERL @ `c32939b`'s `RelativeFrame` uses
  `construct_transform_matrix`, i.e. `blockdiag(R, R)`. SERL @ `1fa2af7` and RLinf use the adjoint
  with the `[p]×R` lower-left block (`construct_adjoint_matrix`). rwrl's RLinf path
  (`real_world_rl/envs/rlinf_cartesian.py`) keeps the SERL/RLinf adjoint. With a TCP away from the
  base origin, the same end-effector-frame action therefore maps to a different base twist than in
  HIL-SERL.
- **Grasp penalty.** HIL-SERL's `GripperPenaltyWrapper` subtracts `penalty = 0.1` from the reward
  when the executed gripper action would change the measured gripper state (`gripper_pos > 0.95`
  counts as open). rwrl's `GraspPenaltyWrapper` (`envs/wrappers/grasp_penalty.py`) reports a
  separate `grasp_penalty` of −0.05 from toggles of the executed action. The magnitude, the
  trigger and where the penalty lands all differ.
- **Euler observations.** HIL-SERL turns the relative `tcp_pose` quaternion into `xyz` Euler angles
  in a wrapper (`Quat2EulerWrapper`). rwrl's RLinf path computes
  `relative_rotation.as_euler("xyz")` itself. The conventions were not compared beyond the axis
  order.
- **Timeout.** HIL-SERL's `FrankaEnv.step` returns `done = path_length >= max_episode_length or
  reward or terminate` as *terminated* (truncated is always False). `RealRobotEnv` reports a time
  limit as **truncated**, and an operator's episode end as truncated too, so bootstrapping differs
  at timeouts.
- **Profile switching at reset.** HIL-SERL's `go_to_reset` posts `PRECISION_PARAM`, optionally
  runs `/jointreset`, interpolates to the reset pose and then posts `COMPLIANCE_PARAM`, and
  `reset()` posts `COMPLIANCE_PARAM` again. rwrl validates `real_robot.serl_profiles` and exposes
  `serl_profile_update` / `arm.update_serl_params`, but `RealRobotEnv` does not switch profiles
  during its reset.
- **`euler_2_quat` pitch sign.** HIL-SERL's `utils/rotations.euler_2_quat` (used for `RESET_POSE`,
  `interpolate_move` goals and random resets) builds a pyquaternion and returns its `[w, x, y, z]`
  elements, which `FrankaEnv` reads as `[x, y, z, w]`. For the usual "gripper down" poses
  (`[π, pitch, roll]`) the round trip `quat_2_euler(euler_2_quat(e))` returns the pitch with the
  opposite sign, so HIL-SERL's reset orientations are mirrored in pitch. rwrl does not reproduce
  this.
- **`action_indices`.** HIL-SERL's `SpacemouseIntervention(action_indices=...)` keeps only the
  listed expert action dimensions and zeroes the others. rwrl's serl mode passes all six axes.
  `diagnostic_axes` exists for commissioning only and is not part of the training contract.
