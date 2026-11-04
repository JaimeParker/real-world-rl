# Teleoperation

Human-in-the-loop intervention in `real_world_rl`: a `TeleopSource` reads one operator
device, `InterventionWrapper` swaps the operator's action in for the policy's, and the demo
scripts in `examples/` use the same sources.

## The `TeleopSource` contract

`real_world_rl.teleop` (`teleop/base.py`):

```python
@dataclass(frozen=True)
class TeleopReading:
    action: np.ndarray          # float32 (A,), env-normalized [-1, 1], gripper last when has_gripper
    intervened: bool            # the operator is driving; action only matters then
    episode_end: bool = False   # operator's episode-end button
    events: Mapping[str, bool] = {}   # success / failure / rerecord edges that fired, else empty
    timestamp: float = 0.0
    human_action_mask: Any = None  # actual human-controlled dimensions
    override_mask: Any = None      # overridden dimensions including hold supplements

class TeleopSource(ABC):
    action_mode: str            # "cartesian_delta" | "joint_delta" | "joint_abs"
    arm_action_dim: int
    has_gripper: bool
    def start(self) -> None     # open the device (default no-op)
    def poll(self) -> TeleopReading            # abstract
    def reset(self) -> None     # after an env reset: drop latched events and sticky state
    def close(self) -> None     # release the device (safe to call twice)
    @classmethod
    def from_env(cls, env, **kwargs)           # registry factory; default cls(**kwargs)
```

- `action_mode`, `arm_action_dim` and `has_gripper` are checked against the env's action contract
  when the wrapper is built; a mismatch (for example a cartesian SpaceMouse on a `joint_delta`
  robot) raises `ValueError` before the device is started.
- `poll()` must let device failures propagate: swallowing one would record policy actions as
  operator demonstrations.
- `as_teleop_source(obj)` returns a `TeleopSource` unchanged and wraps any duck-typed legacy
  object (`poll()` returning something with `.intervened`, `.action`, `.episode_end`; optional
  `reset()`, `close()` and `action_mode`/`arm_action_dim`/`has_gripper`, defaulting to
  cartesian_delta / 6 / True) in a `LegacyTeleopAdapter`.

## Devices

`TeleopInterventionWrapper(env, device=..., **kwargs)` builds the source with
`make_teleop_source(device, env, **kwargs)`; in hil_serl the device comes from `--teleop-device`.

In hil_serl (and residual, SiLRI) the source kwargs are derived from the args (`robot`/`leader_arm`:
the robot part declaration and limits; `gello`/`leader_arm`: the onset flags), then
`--teleop-options-yaml PATH` is merged on top: a yaml mapping of source kwargs that overrides
the derived ones. Use it for device options the CLI has no flags for, for example a local GELLO:

```yaml
# gello.yaml, for --teleop-device gello --teleop-options-yaml gello.yaml
port: /dev/serial/by-id/usb-FTDI_...
joint_ids: [1, 2, 3, 4, 5, 6, 7]
joint_offsets_rad: [...]   # from gello_calibrate
joint_signs: [1, 1, -1, 1, 1, 1, 1]
gripper_id: 8
gripper_open_rad: ...
gripper_closed_rad: ...
hold_s: 0.0
```

or `device_index: 1` (and `deadzone`, `invert_axes`) for a second SpaceMouse, or, for
`--teleop-device leader_cartesian` on the
`franka_real` stack, the robot daemon serving the leader arm (see
[Leader arm on a cartesian robot](#leader-arm-on-a-cartesian-robot-leader_cartesian)):

```yaml
# leader_cartesian.yaml, for --teleop-device leader_cartesian --teleop-options-yaml leader_cartesian.yaml
remote_address: <robot-host>:<port>
```

With `--teleop-event-device` the file feeds the motion half of the composite.

The wrapper passes its "last gripper action sent" to `robot`, `leader_arm`, `gello`,
`leader_cartesian`, `spacemouse` and `bimanual` (and to the motion half of a composite built on
one of them), so a leader part without a gripper holds the gripper instead of dropping a grasped
object, and the SpaceMouse starts every take-over from the gripper's current command. The
env action ends with one gripper dim per arm (the action contract's layout), so the getter
returns one entry per arm and a single-arm source reads entry 0 (`bimanual` hands each arm's
source its own entry); a source drives one arm unless it declares `num_arms`, and the wrapper
rejects a source whose arm count differs from the env's, or whose `arm_names` (when it declares
them) are not the env's in the env's order.

| `device` | Source | Reads | Action space | Notes |
| --- | --- | --- | --- | --- |
| `pico` | `PicoTeleopSource` | Pico VR controller over a ZeroMQ `SUB` socket (`EETwistTeleOpWrapper`) | cartesian_delta, 6 + gripper | kwargs: `zmq_url`, `hand`, `pos_scale`, `rot_scale`, `twist_limit`, `intervention_threshold`. Data format below. |
| `spacemouse` | `SpaceMouseTeleopSource` | a SpaceMouse on this machine, through the device layer's `SpaceMouseTeleop`, as a velocity input over the cartesian command tracker | cartesian_delta, 6 + gripper | button 0 takes over / hands back, button 1 toggles the gripper; see [SpaceMouse](#spacemouse). kwargs: `device_index=0`, `deadzone=0.05`, `invert_axes=()`, `poll_interval_s=0.005`, `read_timeout_s`, and the tracker's (`lead_cap_m`, `lead_cap_rad`, `target_speed_m_s`, `target_speed_rad_s`: the `--teleop-lead-cap-*`/`--teleop-target-speed-*` flags). Needs `pyspacemouse>=2,<3`, imported at `start()`. |
| `spacemouse_twist` | `SpaceMouseTwistTeleopSource` | the same SpaceMouse as a raw twist (the ManiSkill sim examples only; not for real robots) | cartesian_delta, 6 + gripper | the puck's twist goes to the env as is, moving the puck counts as intervening (`hold_s` extends it), button 0 toggles the sticky gripper (open after every reset), button 1 ends the episode. kwargs: `device_index=0`, `deadzone=1e-3`, `invert_axes=()`, `poll_interval_s=0.005`, `hold_s=0.0`, `read_timeout_s`. |
| `robot` | `TeleopPartSource` | the teleop part of the robot behind a `real_robot` env (robot yaml `teleop:` section, local or daemon-served) | cartesian_delta, joint_abs or joint_delta, mapped into the env space | kwargs: `teleop_contract`, `joint_limits`, `joint_scale_rad`, `read_timeout_s` (hil_serl: `--robot-teleop-action-mode`, `--no-robot-teleop-has-gripper`). The robot owns the part, so `close()` leaves it connected. |
| `fake` | `ScriptedTeleopSource` | scripted readings (tests; with no `readings` an operator who never intervenes) | follows the env's action contract, its arm count included | pass `readings=[TeleopReading, ...]` to script one; the last reading repeats. |
| `gello` | `GelloTeleopSource` | a GELLO on this machine, through the device layer's `gello` teleop (Dynamixel bus) | joint_abs or joint_delta (the env's), leader joints mapped as for `robot` | kwargs: the device options (`port`, `joint_ids`, `joint_offsets_rad`, `joint_signs`, ...; see [Leader arms](#leader-arms-gello)), `hold_s`, `read_timeout_s`, `onset`, `arm_tolerance_rad`, `ramp_step_rad`. Needs `dynamixel-sdk` (robot_infra extra `gello`), imported at `start()`. |
| `leader_arm` | `LeaderArmPartSource` | the teleop part of the robot behind a `real_robot` env, read as a joint-space leader arm (a GELLO served by the robot daemon) | joint_abs or joint_delta (the env's) | kwargs: those of `robot` plus `onset`, `arm_tolerance_rad`, `ramp_step_rad` (hil_serl: `--teleop-onset`, `--teleop-arm-tolerance-rad`, `--teleop-ramp-step-rad`). |
| `keyboard` | `KeyboardTeleopSource` | this machine's keyboard via `pynput` | follows the env's action contract; action always zeros | `t` takes over, `h` hands back, Esc ends the episode, `s`/`f`/`r` raise `success`/`failure`/`rerecord`, `q` ends the episode and raises `operator_quit` (the recorder and the actor loops stop after that episode). An event device: on its own it only suits event-only tests. Needs `pynput` (extra `reward-collection`) and a desktop session, imported at `start()`. |
| `stdin` | `StdinTeleopSource` | the terminal running the process (cbreak mode, restored on `close()`; works over SSH, no desktop session) | follows the env's action contract; action always zeros | the `keyboard` keys. kwargs: `fd` (default stdin), `poll_interval_s=0.02`. End of input raises `DeviceFault` on the next poll. |
| `composite` | `CompositeTeleopSource` | a motion source plus an event source | the motion source's | kwargs: `motion`, `events` (registry names), `motion_kwargs`, `events_kwargs`, `intervention_from="events"` (or `"motion"`). hil_serl builds it from `--teleop-event-device`. |
| `leader_cartesian` | `LeaderArmCartesianSource` | a joint-space leader arm turned into a cartesian twist by Panda forward kinematics (the `franka_real` stack) | cartesian_delta, 6 + gripper | see [Leader arm on a cartesian robot](#leader-arm-on-a-cartesian-robot-leader_cartesian). |
| `bimanual` | `BimanualTeleopSource` | one single-arm source per arm of a `real_robot_bimanual` env (`--teleop-arm-device`) | the env's, both arms then both grippers | kwargs: `device_per_arm`, `per_arm_kwargs`, `common_kwargs`, `intervention_from="any"` (or `"all"`). See [Two arms](#two-arms-bimanual). |

### SpaceMouse

For a `cartesian_delta` env (`real_robot` in `cartesian_delta` mode, or `franka_real`), with the
operator's labels on the terminal (the SpaceMouse Compact has two buttons, both taken):

```bash
python examples/record_real_demos.py --config hil.yaml \
    --teleop-device spacemouse --teleop-event-device stdin \
    --output-path demos/task_20.pkl --num-successes 20
```

The HIL actor takes the same two flags. `SpaceMouseTeleopSource`
(`real_world_rl/teleop/spacemouse.py`) opens the device with the device layer's
`SpaceMouseTeleop` (`TeleopConfig(backend="spacemouse")`, a poll thread feeding `read()`; the
twist is in `[-1, 1]` with axis map `[-y, x, z, -roll, -pitch, -yaw]` of the pyspacemouse state,
`invert_axes` flips the sign of the listed output indices 0-5) and drives the same
`CartesianTargetTracker` as `leader_cartesian` (see [Mapping](#mapping)):

| Input | Effect |
| --- | --- |
| button 0 | take over; the next press hands back. Moving the puck alone never intervenes, so a nudge does not take over. |
| button 1 | while driving: toggle the gripper between open (`+1`) and closed (`-1`) |
| puck | while driving: a velocity (below) |
| `s`/`f`/`r`, Esc, `q` on `--teleop-event-device stdin` (or `keyboard`) | `success`/`failure`/`rerecord`, episode end, quit |

- **Velocity.** Each intervened step requests the tracker's command moved by
  `twist * target_speed * dt` (`dt` the time since the previous intervened step, 0 at a take-over,
  so taking over never jumps): translation `twist[:3] * --teleop-target-speed-m-s * dt` in the
  base frame, rotation vector `twist[3:] * --teleop-target-speed-rad-s * dt` left-multiplied, as
  the env's step. Full deflection is the target speed (0.05 m/s, 0.3 rad/s); the tracker limits a
  diagonal to it. Each axis within `deadzone` (0.05 of full scale, above the Compact's rest noise)
  counts as zero.
- **Lead cap.** The command never leads the measured TCP pose by more than
  `--teleop-lead-cap-m`/`--teleop-lead-cap-rad` (0.05 m, 0.3 rad); the excess is dropped. The
  request starts from the command, not from the previous request, so an arm that lags behind
  never lets it run ahead: release the puck and the arm stops at the command, at most one lead cap
  from where it was. The env's `xyz_scale_m`/`rot_scale_rad` must be at least the lead caps
  (`--real-robot.xyz-scale-m 0.05 --real-robot.rot-scale-rad 0.3`; the `real_robot` defaults
  0.02/0.1 are rejected when the source is built).
- **Hand-back** holds the measured pose; the recorder's hold policy shares the tracker and keeps
  the arm on that pose (not a zero twist) until the next take-over. A requested position outside
  the safety box (`WORKSPACE HOLD: <axes>` in the log), an input or tracking failure hand back as
  well: press button 0 to take over again. A stalled control loop (more than 0.3 s between two
  intervened steps) holds the measured pose and the next step continues from there.
- **Gripper.** At each take-over the gripper action starts as the gripper's last command (the
  intervention wrapper's `last_gripper_action_getter`, kept as is, so taking over never opens a
  closed gripper); button 1 then switches to the other state. Button 1 does nothing while handed
  back.
- **Buttons act on press, once.** The device layer latches every press until the next read, so a
  press shorter than one env step is not lost; a held button counts once, and a button held
  through an env reset does not count.
- **The event device's `t`/`h` are ignored** (a warning says so): they set a take-over state,
  which can disagree with a take-over made by button 0. Button 0 is the only toggle.
- The SDK is imported when `start()` opens the device (the wrapper calls it), so building the
  source works without `pyspacemouse`; a missing SDK raises `SdkMissing` with the install hint.

A run with `--teleop-device spacemouse` records the tracker in the experiment contract
(`teleop_control`, the same block as `leader_cartesian`). `--teleop-event-device spacemouse` was
removed (the SpaceMouse no longer provides labels); an old command line or preset with it fails
with a message pointing at `--teleop-device spacemouse --teleop-event-device stdin`.

The robot-yaml route (`--teleop-device robot` with a `backend: spacemouse` teleop section) reads
the same device through the robot (and its daemon) with the device layer's own, older semantics:
the twist goes to the env as is, the operator intervenes while the puck moves or a button is held
(`hold_s` keeps it going), and button 0 toggles the device's sticky gripper. Neither button ends
the episode on any route; episode end is Esc on the event device.

### Robot teleop part (`--teleop-device robot`)

hil_serl on `--env-backend real_robot` can also take interventions from the teleop part of
the robot itself (the robot yaml's `teleop:` section: spacemouse, the `gello` leader arm or the
`fake` backend, served locally or by the robot daemon; a leader arm is better read with
`--teleop-device leader_arm`, see [Leader arms](#leader-arms-gello)). `TeleopPartSource`
(`real_world_rl/teleop/part.py`) adapts it, mapping cartesian or joint
(`joint_abs`/`joint_delta`) teleop actions into the env's `[-1, 1]` action space and forwarding
the device's `success`/`failure`/`rerecord` events; see the README's "Action contract" and
"Teleop (`--teleop-device robot`)" sections for the mapping, the `--robot-teleop-action-mode`
declaration and an example yaml.

### Leader arms (GELLO)

The end-to-end guide (hardware, calibration, robot yaml, demo recording and the HIL commands for
`hil_serl`, `residual_hil_serl` and `silri`) is [gello.md](gello.md); the first-day walkthrough
with the setup wizard, BC, ACT and deployment is the
[tutorial](tutorial/00-overview.md). This section describes the sources.

A leader arm reports absolute leader joint positions. `LeaderArmPartSource`
(`real_world_rl/teleop/leader_arm.py`) maps them into the env's joint action mode like
`TeleopPartSource` (`joint_abs`: normalized onto `joint_limits`; `joint_delta`:
`(q_leader - q_follower) / joint_scale_rad`, saturating at one `joint_scale_rad` per step) and
adds an onset policy for the moment the operator takes over. Two ways to read a GELLO:

- `--teleop-device leader_arm`: the GELLO is the `teleop:` part of the robot yaml the daemon
  serves on the robot PC (`python -m robot_infra.server --config robot.yaml ...`), read through
  `RemoteRobot`. `python -m robot_infra.setup` writes the section (calibration included) into a
  station yaml; or add it to the robot yaml by hand (for example
  `robot_infra/docs/robot.franka.example.yaml`);
  `python -m robot_infra.devices.backends.teleops.gello_calibrate --port ... --joint-ids 1 2 3 4 5 6 7 --gripper-id 8`
  prints the `options` block for a unit:

  ```yaml
  teleop:
    name: gello
    backend: gello
    options:
      port: /dev/serial/by-id/usb-FTDI_...
      joint_ids: [1, 2, 3, 4, 5, 6, 7]
      joint_offsets_rad: [...]    # from gello_calibrate
      joint_signs: [1, 1, -1, 1, 1, 1, 1]
      gripper_id: 8
      gripper_open_rad: ...
      gripper_closed_rad: ...
      intervention: toggle        # or always (pure teleop)
      event_device: /dev/input/by-id/...   # optional evdev pedal/keyboard on the robot PC
  ```

- `--teleop-device gello` (`GelloTeleopSource`): a GELLO plugged into the actor's machine, opened
  through the same device-layer backend with the options above as source kwargs; from the
  training CLI they come from `--teleop-options-yaml` (above).

HIL recipe over the daemon (a `joint_delta` follower; `--robot-teleop-action-mode` only has to
name a joint space for a leader arm, it declares the part before the remote robot connects):

```bash
python examples/train_real_world.py hil_serl --role actor --env-backend real_robot \
    --real_robot.remote ROBOT_PC:50051 --real_robot.action_mode joint_delta \
    --teleop-device leader_arm --robot-teleop-action-mode joint_delta \
    --teleop-onset ramp --teleop-event-device keyboard
```

Onset (`--teleop-onset`, source kwarg `onset`):

- `ramp` (default): in `joint_abs` the emitted target is
  `q_follower + clip(q_leader - q_follower, +-ramp_step_rad)` (`--teleop-ramp-step-rad`, 0.05),
  normalized onto the joint limits, so the recorded action is the executed command; those
  intervened steps carry `info["teleop_events"]["teleop_syncing"] = True`. The archive retains
  them, while training excludes syncing expert labels. In `joint_delta`, initial catch-up is
  marked until the leader first comes within `joint_scale_rad`; later saturated motion of the
  same takeover is ordinary teaching.
- `arm`: the toggle only takes effect once `|q_leader - q_follower|_inf <= arm_tolerance_rad`
  (`--teleop-arm-tolerance-rad`, 0.1). Until then the step is not intervened and the per-joint
  error is logged (at most once per second): move the leader onto the follower. Once armed the
  source stays armed until the toggle releases; every env reset disarms it.

`q_follower` is the `joint_pos` of the env's latest observation, so `state_fields` must contain
`joint_pos`. The toggle is the part's `intervening` (`intervention: toggle` with an evdev
`event_device` on the robot PC, or `always`) unless an event device supplies it:

`--teleop-event-device keyboard` (or `stdin`) wraps the motion device in a
`CompositeTeleopSource`: motion from `--teleop-device`, the intervention state and the
`episode_end`/`success`/`failure`/`rerecord`/`operator_quit` events from the event device on the
actor's machine (keyboard and stdin: `t` takes over, `h` hands back, Esc ends the episode,
`s`/`f`/`r`, `q` quits). The leader source still applies its onset policy to that state; the
part's own `intervention` setting is then ignored. (`--teleop-device spacemouse` keeps its own
take-over on button 0 and takes only the events from the event device, see
[SpaceMouse](#spacemouse).)

### Leader arm on a cartesian robot (`leader_cartesian`)

A GELLO leader arm reports joint angles. Envs that only accept a cartesian twist get those
angles through Panda forward kinematics: `LeaderArmCartesianSource`
(`real_world_rl/teleop/leader_cartesian.py`, `device="leader_cartesian"`) turns the
leader's joints into a requested end-effector pose, and a `CartesianTargetTracker`
(`real_world_rl/teleop/cartesian_tracker.py`) turns that into the twist that moves the follower
(see [Mapping](#mapping)). `examples/presets/gello_cartesian.yaml` is a single-arm 50 Hz preset.

#### Why

The legacy `franka_real` stack talks to the SERL HTTP bridge, which only knows `/pose` (an
absolute cartesian impedance target), `/open_gripper`/`/close_gripper`, `/jointreset`,
`/startimp`/`/stopimp` and `/getstate`. There is no streaming joint command, so the joint-space
leader sources (`--teleop-device leader_arm`/`gello`, see [Leader arms](#leader-arms-gello)) cannot drive it, and
`FrankaRealEnv` takes a 7-D action: base-frame twist (6) + gripper. The conversion therefore
happens on the actor side:

```
GELLO joints q --PandaKinematics--> T* = (p*, R*) --vs follower TCP (p, R)--> twist in [-1, 1]
```

The same source serves `real_robot` in `cartesian_delta` mode (for example a daemon with
`arm_backend: serl_http`).

#### Pieces

| Piece | Where | What |
| --- | --- | --- |
| `PandaKinematics` | `real_world_rl/kinematics/panda.py` | numpy forward kinematics, Craig modified DH; `flange_pose(q)`, `ee_pose(q)` (xyz, quat xyzw), `ee_transform(q)` (4x4). EE defaults: Franka Hand, 0.1034 m along the flange z and -pi/4 about it (`ee_offset_m`, `ee_yaw_rad`). |
| `LeaderArmCartesianSource` | `real_world_rl/teleop/leader_cartesian.py` | the teleop source; any `joint_abs` 7-joint teleop part is a valid leader. |
| `FrankaRealEnv.last_tcp_pose` | `real_world_rl/envs/franka_real/env.py` | the measured TCP pose (`O_T_EE`, xyz + quat xyzw) of the latest reset/step; the source's follower pose. |
| stub-robot yaml | `robot_infra/docs/robot.gello_station.example.yaml` | serves the GELLO from the robot host through the robot daemon. |

#### The stub-robot daemon on the robot host

The GELLO is plugged into the robot host (the machine that runs the SERL bridge). A stub robot,
`type: franka` with `arm_backend: fake`, no gripper and no cameras, carries it as its `teleop:`
part, and the robot daemon serves it:

```bash
# on the robot host, next to the SERL bridge
pip install -e "robot_infra[gello]"          # + [gello-events] for an evdev pedal/keyboard
python -m robot_infra.devices.backends.teleops.gello_calibrate --port /dev/serial/by-id/usb-FTDI_... \
    --joint-ids 1 2 3 4 5 6 7 --gripper-id 8            # paste the printed options into the yaml
python -m robot_infra.server --config robot.gello_station.yaml --bind 0.0.0.0:50052
```

The fake arm holds no hardware and is never commanded, so the stub does not compete with the
bridge for the robot. Use a port other than any real robot daemon on the same host.

#### Actor side

The source needs exactly one leader:

- `remote_address="host:port"` (+ optional `remote_token`): a `RemoteRobot` for the stub daemon,
  connected by `start()` (the intervention wrapper calls it) and disconnected by `close()`;
- `robot_yaml=<path>`: the same stub yaml loaded on this machine (the GELLO plugged into the
  actor's machine), connected by `start()`;
- `leader=<part or resolver>`: a teleop part the caller owns (tests, custom setups).

The follower pose, the per-step scales and the safety box come from the env:
`FrankaRealEnv.last_tcp_pose`, `cfg.action_scale` and `cfg.safety_box_low/high`, found down the
wrapper chain; on a `real_robot` cartesian env, `last_observation["state"]["tcp_pose"]`, the action
contract's `xyz_scale_m`/`rot_scale_rad` and the configured `safety_box` (`None`: no workspace
check in the source; the env and the device layer still clip). Pass `follower_pose_getter=`,
`pos_scale_m=`, `rot_scale_rad=` and `safety_box=` to override them. The scales must be at least
the lead caps (`xyz_scale_m >= lead_cap_m`, `rot_scale_rad >= lead_cap_rad`), else the source
refuses to build: the default scales (0.02 m / 0.1 rad) need either larger ones
(`--real-robot.xyz-scale-m 0.05 --real-robot.rot-scale-rad 0.3` as in the preset, or
`--franka-real.action-scale-pos`/`--franka-real.action-scale-rot`) or lower caps.

```python
from real_world_rl.envs.wrappers.intervention import TeleopInterventionWrapper

env = TeleopInterventionWrapper(
    franka_real_env,
    device="leader_cartesian",
    remote_address="robot-host:50052",
    mapping="anchored",
)
```

From the training CLI, select the source with `--teleop-device leader_cartesian` and pass its
kwargs as a yaml mapping with `--teleop-options-yaml` (merged into the motion source's kwargs):

```yaml
# station.yaml (actor side)
remote_address: robot-host:50052   # the stub daemon above; or robot_yaml: <path> for a local GELLO
mapping: anchored                  # or absolute
gripper_binarize_threshold: 0.5
```

```bash
python examples/train_real_world.py hil_serl --role actor --env-backend franka_real ... \
    --teleop-device leader_cartesian --teleop-options-yaml station.yaml
```

With the default `intervention: always` the operator drives on every step (pure teleop or demo
collection). For HIL, set `intervention: toggle` with an `event_device` on the robot host, or
take the intervention from an actor-side event device (`--teleop-event-device stdin` or
`keyboard`: `t` takes over, `h` hands back; see [Leader arms](#leader-arms-gello)).

#### Mapping

`RealRobotEnv` in `cartesian_delta` sends `measured + action * scale`, and an impedance backend
(Franky's cartesian impedance tracker) treats that pose as its equilibrium. A twist recomputed
from the measured pose every step therefore never holds still: a zero action re-anchors the
equilibrium wherever the arm drifted (137 mm in 24 s on the first hardware session). The source
keeps the target instead, in a `CartesianTargetTracker` per arm:

- a **hold pose**, sampled once at the env reset and once at every hand-back (the measured pose
  at that moment), then fixed while nobody drives;
- a persistent **command pose**. At the take-over it is the current command (the hold pose: no
  jump); while the operator drives it moves towards the requested pose by at most
  `--teleop-target-speed-m-s` / `--teleop-target-speed-rad-s` (0.05 m/s, 0.3 rad/s) and then is
  pulled back to within `--teleop-lead-cap-m` / `--teleop-lead-cap-rad` (0.05 m, 0.3 rad) of the
  measured pose. The excess is dropped, never queued: a leader that jumps and comes back does not
  make the follower overshoot.

The emitted arm action lands the follower on the command, in the env's normalized units:

```
a_pos = (p_cmd - p) / pos_scale_m
a_rot = rotvec(R_cmd R^T) / rot_scale_rad
```

the inverse of the env's step (`p + a_pos * pos_scale_m`, and `from_rotvec(a_rot *
rot_scale_rad) * R`: the rotation is left-multiplied, so the rotation vector is in the base
frame). With the scales at least the lead caps the action never leaves `[-1, 1]`; the tracker
raises (`CommandOutOfRange`) rather than clip one that would. A hand-back, an input or tracking
failure and an episode end put the tracker in hold at the measured pose. A stalled control loop
(more than 0.3 s between two intervened polls) counts as an input loss: the follower holds, a
warning gives the gap, and the next poll re-anchors the leader as at a take-over (a clock going
backwards raises `MissedControlDeadline`). A requested position outside the safety
box does too: the source logs `WORKSPACE HOLD: <axes>` and reads as not intervened until the
operator hands back and takes over again.

The recorder's hold policy (`real_world_rl/teleop/hold.py`, `CartesianHold`) shares the source's
tracker: while nobody drives it sends the action back onto the fixed hold pose (not a zero twist),
at most one lead cap per step. Without `leader_cartesian` (or `spacemouse`, which shares its tracker
the same way) it keeps a tracker of its own whose caps
are `min(0.05 m, xyz_scale_m)` / `min(0.3 rad, rot_scale_rad)`, so an arm pushed far off the hold
gets a saturated action back, never an error.
A run with `--teleop-device leader_cartesian` (or `spacemouse`) records the tracker in the
experiment contract (`teleop_control`: version `cartesian_tracker_v1` and the four values above,
compared): demos recorded without it, or with other values, do not load into it. The block names
the tracker, not the device, so GELLO and SpaceMouse demos with the same flags load together.

`mapping` chooses the requested pose:

- `anchored` (default): at each take-over (the first intervened poll after a non-intervened one,
  and the first after an env reset) the source stores the tracker's command `(p_c0, R_c0)` and
  the leader pose `(p_l0, R_l0) = FK(q_0)`. Afterwards the leader's base-frame displacement is
  applied to the command, translation and rotation decoupled:

  ```
  p* = p_c0 + (p_l - p_l0)
  R* = (R_l R_l0^T) R_c0
  ```

  The take-over keeps the hold action, so taking over never jerks the robot. A leader wrist
  rotation turns the follower in place about its own TCP, however far apart the two poses are
  (the composed form `FK(q) T_l0^{-1} T_f0` would instead swing the follower about the base
  origin, translating it by about `|p_f0| * theta`). FK errors only enter through how they change
  since onset. Leader and follower postures can differ (re-grab the leader between
  interventions like a clutch); the elbow / null-space motion of the leader is ignored, as in any
  cartesian command.
- `absolute`: `T* = FK(q)`. The follower goes to wherever the leader's end effector is, which
  needs `PandaKinematics.ee_pose` to match the robot's `O_T_EE` (verify it, below) and the leader
  to be posed near the follower before taking over, otherwise the follower travels from the hold
  pose to the leader's pose at the target speed.

Prefer `anchored` unless the leader is calibrated and verified against the robot.

#### Gripper

The leader's width target `w` in `[0, 1]` (1 = open) becomes `+1` (open) if
`w > gripper_binarize_threshold` and `-1` (close) otherwise; the default 0.5 closes the gripper
once the trigger passes half travel. The exact +-1 values are clean labels for the hybrid
(discrete) gripper critic, and `franka_real` (open on `a > 0.5`) executes them unambiguously;
`gripper_binarize_threshold=None` emits the continuous `2w - 1` instead. A leader
without a gripper channel holds the gripper action last sent to the env, which needs
`last_gripper_action_getter` (as for `--teleop-device robot`).

When the trigger disagrees with the gripper's last commanded state at the take-over (open:
action > 0; the intervention wrapper supplies it through `last_gripper_action_getter`), the
source keeps sending that last action until the trigger crosses to the gripper's state, and only
then follows the trigger. Taking over with an open trigger therefore never drops a grasped
object.

#### Verify the DH table on the robot

`PandaKinematics` matches Franka's published reference poses (flange at `q = 0`:
`(0.088, 0, 0.926)`, z down; flange at the ready pose `[0, -pi/4, 0, -3pi/4, 0, pi/2, pi/4]`:
about `(0.307, 0, 0.590)`), but has not been compared with a real robot yet. Before relying on
`mapping="absolute"`, compare it with the bridge's own pose at a few joint configurations
(the ready pose, and two or three poses spread over the workspace with the wrist rotated):

```python
import numpy as np
from scipy.spatial.transform import Rotation

from real_world_rl.envs.franka_real.bridge_client import FrankaBridgeClient
from real_world_rl.kinematics import PandaKinematics

state = FrankaBridgeClient("http://robot-host:5000").get_state()  # q and pose (O_T_EE)
position, quat = PandaKinematics().ee_pose(state["q"])
pose = np.asarray(state["pose"])
print("position error [m]:", np.linalg.norm(position - pose[:3]))
print("rotation error [rad]:", (Rotation.from_quat(quat) * Rotation.from_quat(pose[3:]).inv()).magnitude())
```

Expect errors below about 1 mm and 0.01 rad. A constant offset along the tool axis or about it
means Desk's end-effector setting differs from the Franka Hand default: pass
`kinematics=PandaKinematics(ee_offset_m=..., ee_yaw_rad=...)` to the source. Errors that change
with the configuration point at the DH table itself; `anchored` mapping is less sensitive to
both.

### Two arms (`bimanual`)

On `--env-backend real_robot_bimanual` (see [bimanual.md](bimanual.md)) the action is
`[left arm, right arm, left gripper, right gripper]`, and the single-arm devices (`robot`,
`leader_arm`, `gello`, `leader_cartesian`, `spacemouse`, `spacemouse_twist`, `pico`) are rejected. `--teleop-device
bimanual` (`BimanualTeleopSource`, `real_world_rl/teleop/bimanual.py`) builds one
single-arm source per arm over that arm's own `RealRobotEnv`, so each one reads its own robot and
follower, and assembles their readings into the two-arm action. `--teleop-arm-device` picks the
per-arm device:

| `--teleop-arm-device` | Each arm's source | Notes |
| --- | --- | --- |
| `leader_arm` (default) | `LeaderArmPartSource` over that arm robot's teleop part | a GELLO served by that arm's daemon (its robot yaml's `teleop:` section); `--robot-teleop-action-mode` declares both parts before the remote robots connect; the `--teleop-onset` flags apply to every arm |
| `robot` | `TeleopPartSource` over that arm robot's teleop part | any teleop part, mapped as for `--teleop-device robot` |
| `gello` | `GelloTeleopSource` | a GELLO plugged into the actor's machine, one per arm; each needs its own `port` and calibration (below) |

`leader_cartesian` is not available per arm: it turns one leader into the twist of one cartesian
arm.

The bimanual source requests intervention: `intervened` is true while any arm's operator
intervenes (`intervention_from: any`, the default) or only while both do (`all`). Under `any`,
the arm whose operator is not intervening is held still: its arm block is the hold action (the
follower's joints in `joint_abs`, zero in the delta modes) and its gripper the action last sent
to it, so an idle leader never moves its follower and an idle trigger never opens a gripper.
`episode_end` and the events are ORed over the arms; `info["teleop_events"]` carries each event
both as `<arm>_<event>` and as the plain `<event>` (`success`, `teleop_syncing`, ...).

With `--teleop-event-device keyboard` (or `stdin`) the bimanual source is the motion half of a
composite: `t`/`h` take over and hand back both arms at once (each arm's onset policy still
applies), and the keyboard carries the episode-end, outcome and quit keys.

`--teleop-options-yaml` for `bimanual` is either a mapping with one section per arm name plus
common keys (`intervention_from` is the source's own; every other common key goes to every arm,
the arm's section overrides it):

```yaml
# bimanual.yaml, for --teleop-device bimanual --teleop-arm-device gello
intervention_from: any
hold_s: 0.0
left:
  port: /dev/serial/by-id/usb-FTDI_..._LEFT-if00-port0
  joint_ids: [1, 2, 3, 4, 5, 6, 7]
  joint_offsets_rad: [...]
  joint_signs: [...]
right:
  port: /dev/serial/by-id/usb-FTDI_..._RIGHT-if00-port0
  joint_ids: [1, 2, 3, 4, 5, 6, 7]
  joint_offsets_rad: [...]
  joint_signs: [...]
```

or, with `--teleop-arm-device gello`, the robot_infra station yaml itself
(`robots: {left: ..., right: ...}`, as `python -m robot_infra.setup` writes it): each robot's
`teleop.options` (and `hold_s`) become that arm's kwargs, matched by robot name to the env's
`arm_names`. A single-robot station or robot yaml works the same way for `--teleop-device gello`
on one arm. With `leader_arm` or `robot` per arm, each daemon reads its own GELLO, so a station
yaml is refused there.

HIL over the two daemons:

```bash
python examples/train_real_world.py hil_serl --role actor --config bimanual.yaml \
    --teleop-device bimanual --teleop-arm-device leader_arm \
    --robot-teleop-action-mode joint_delta --teleop-event-device keyboard
```

`--teleop-device fake` declares the env's arm count too, so it dry-runs the two-arm actor
without an operator.

## `InterventionWrapper`

`real_world_rl/envs/wrappers/intervention.py`. `InterventionWrapper(env, source)` takes a
`TeleopSource`; `TeleopInterventionWrapper(env, teleop=None, device="pico", **kwargs)` builds one
by device name (or adapts the object passed as `teleop`). Both expose the source as `.source`
(alias `.teleop`).

`step(action)` polls the source first and intersects the source's human mask with the selected
parts. With a nonempty human mask, it composes human and idle-part actions as a `(1, A)` float32
tensor on the policy device. `idle_behavior="hold"` holds unselected parts; `"policy"` keeps the
proposal there. Actual human control and hold overrides are recorded separately:

| `info` key | Value |
| --- | --- |
| `intervene_action` | the executed `(1, A)` action; **only while intervening** |
| `policy_action` | the action the policy proposed, as received |
| `intervened` | python bool: the operator's action was executed |
| `intervention_onset` | python bool: intervened now and not on the previous step of this episode |
| `human_episode_end` | python bool: the operator's episode-end button |
| `teleop_events` | `dict` of fired `success`/`failure`/`rerecord` edges and leader catch-up `teleop_syncing`; **only when present** |
| `human_action_mask` | dimensions actually driven by the human, including their provenance during syncing |
| `override_mask` | dimensions replacing the policy proposal, including hold supplements |

An `episode_end` makes `step()` return `truncated=True` (never `terminated`), so actor loops
reset without treating it as terminal.

Order of events:

- Construction: validate the source against the env contract, then `source.start()`.
- `reset(**kw)`: `env.reset()` first, then `source.reset()`: presses while the robot homes are
  dropped. The "last gripper action" bookkeeping returns to open (+1) and the onset detector
  restarts.
- `close()`: `source.close()`, then `env.close()` (also if the source fails to close).

The wrapper remembers the gripper action last sent to the env (clipped to `[-1, 1]`); a teleop
without a gripper on an env with one holds it instead of dropping a grasped object.

## Adding a device

Subclass `TeleopSource`, declare the action space, implement `poll()`, and register it:

```python
from real_world_rl.teleop import TeleopReading, TeleopSource, register_teleop_source

@register_teleop_source("joystick")
class JoystickTeleopSource(TeleopSource):
    action_mode = "cartesian_delta"
    arm_action_dim = 6
    has_gripper = True

    def __init__(self, port: str = "/dev/input/js0") -> None: ...

    def start(self) -> None: ...      # open the device
    def poll(self) -> TeleopReading: ...
    def reset(self) -> None: ...      # drop latched presses
    def close(self) -> None: ...
```

`make_teleop_source("joystick", env, port=...)` then calls `JoystickTeleopSource.from_env(env,
port=...)`, which defaults to `cls(**kwargs)`; override `from_env` to read the env's action
contract (`real_world_rl.action_contract.get_env_action_contract(env)`). Unknown names
raise `ValueError` listing the registered ones. Registration must happen before the name is used,
so a downstream package imports its module (or exposes it through its own entry point) before
building the env. Built-in sources are imported on first use of their name, so importing
`real_world_rl.teleop` pulls in neither `zmq` nor `pyspacemouse`.

Actions are in the env's normalized `[-1, 1]` space, gripper last. For a cartesian device whose
twist is in physical units (as the Pico's), see how the demo scripts normalize it.

## EETwist TeleOp Interface (Pico)

`PicoTeleopSource` wraps `EETwistTeleOpWrapper` in `robot_infra/robot_infra/teleop/utils/telo_op_control_twist.py`
(in the in-repo `robot_infra` device layer). Its only device is `pico`; the wrapper receives device data
from a ZeroMQ `SUB` socket. The Pico server is expected to publish JSON arrays to:

```python
tcp://*:7777
```

The client connects with `--zmq-url`, for example:

```bash
--zmq-url tcp://192.168.6.2:7777
```

`PicoTeleopSource.reset(*, episode_end_pressed=False)` forwards the flag to the wrapper, which
keeps a still-held episode-end button from firing again; `examples/record_teleop_wsrl.py` uses it.

### Pico Data Format

Pico data can be either one hand (`13` values) or two hands (`26` values). For
two hands, the first `13` values are the left hand, and the second `13` values
are the right hand.

For each hand:

| Index | Meaning |
| --- | --- |
| `0:3` | hand position |
| `6:10` | hand quaternion, `[x, y, z, w]` |
| `10` | gripper button |
| `11` | bind / clutch button |
| `12` | episode end button |

`bind` is held while choosing the current reference pose. During bind, twist is
zero. After release, hand pose changes are converted to EE twist.

`EETwistTeleOpWrapper.poll()` returns a `TeleOpSample`, which `PicoTeleopSource` turns into a
`TeleopReading` (`action` = twist + gripper, `intervened`, `episode_end`):

```python
action       # twist + gripper
twist        # shape (6,)
gripper      # scalar gripper command
bind_pressed
episode_end
intervened
```

When no new device data arrives, `twist=0`, `gripper` keeps the previous value,
and `intervened=False`.

## Basic Usage

Visual teleoperation:

```bash
python examples/run_teleop_demo.py \
  --env-id PegInsertionSidePegOnly-v1 \
  --device pico \
  --zmq-url tcp://192.168.6.2:7777
```

Print incoming teleop state without connecting to an environment:

```bash
python robot_infra/robot_infra/teleop/examples/run_twist_input_inspector.py \
  --device pico \
  --zmq-url tcp://192.168.6.2:7777
```

The same demo with a SpaceMouse (`--spacemouse-index`, `--spacemouse-deadzone`,
`--spacemouse-invert-axes`, `--spacemouse-hold-s` tune it) or the idle `fake` source:

```bash
python examples/run_teleop_demo.py --env-id PegInsertionSidePegOnly-v1 --device spacemouse_twist
```

The sim examples use `spacemouse_twist`, the raw-twist SpaceMouse source: moving the puck counts
as intervening, its twist goes to the env as is, button 0 toggles the gripper and button 1 ends
the episode. It is for the sim only; on a real robot use `--teleop-device spacemouse`
([SpaceMouse](#spacemouse)).

`run_teleop_demo.py` creates the ManiSkill environment, builds the source with
`make_teleop_source`, polls teleop actions, applies controller action normalization to the
Pico's physical twist (SpaceMouse actions are already normalized), and steps the environment.

## Recording demos on the real robot

`examples/record_real_demos.py` records teleoperated episodes on the `real_robot`,
`real_robot_bimanual` or `franka_real` env of a HIL run (same env and teleop flags as its actor)
into the demo pkl format the learners
load with `--demo-dataset-paths`, keeping only the intervened steps. See
[recording-demos.md](recording-demos.md); for a GELLO, [gello.md](gello.md#5-recording-demos).

## Recording WSRL Data

`examples/record_teleop_wsrl.py` records teleoperation
episodes into the same H5 format used by the WSRL data generator.

Example:

```bash
python examples/record_teleop_wsrl.py \
  --output-path demos/peg_teleop_wsrl.h5 \
  --env-id PegInsertionSidePegOnly-v1 \
  --obs-mode rgb \
  --control-mode pd_ee_twist \
  --device pico \
  --zmq-url tcp://192.168.6.2:7777
```

`--output-path` must be an H5 file path, not a directory. A `.h5` suffix is
recommended. The file is opened with write mode, so an existing file at the same
path will be overwritten.

Only `intervened=True` samples are stepped in the environment and saved. When
`intervened=False`, the script keeps polling the device and does not record a
transition.

The operator's episode-end button (Pico episode-end button, `spacemouse_twist` button 1) ends the
current episode. After each episode, the terminal prints the episode length, `max_episode_steps`, success flag, and
return, then asks whether to save the episode.

### Important Arguments

- `output_path`: H5 dataset file to write.
- `device`: `pico`, `spacemouse_twist` or `fake` (with the same device-specific flags as the demo).
- `dt`: teleop polling period in seconds. The default is `1 / 30`, roughly
  `30 Hz`. Smaller values poll and step faster.
- `env_id`: ManiSkill environment id.
- `obs_mode`: observation mode, usually `state`, `rgb`, or `rgbd`.
- `include_state`: include proprioceptive state for visual observations.
- `control_mode`: should match the EE twist controller, usually `pd_ee_twist`.
- `sim_backend`, `render_backend`: ManiSkill simulation and rendering backends.
- `reward_mode`: reward mode passed to the environment.
- `robot_uids`: robot id for the environment.
- `fix_box`, `fix_peg_pose`, `peg_density`, `debug_pose_vis`: custom peg env
  options.
- `env_kwargs_json`: escape hatch for extra environment kwargs.
- `end_on_env_done`: if true, env termination, truncation, or success can end
  the episode. The default is false, so only teleop episode-end stops recording.
- `pos_scale`, `rot_scale`, `twist_limit`: optional overrides for twist mapping.
- `intervention_threshold`: twist norm threshold used to decide whether human
  input is active.

The saved H5 contains `traj_*` groups with:

```text
obs
actions
rewards
terminated
truncated
```

This file can be passed to WSRL, CQL, or Cal-QL offline training through
`--offline_dataset`.

For real-robot data where you only want to do offline pretraining (no sim
env, no eval), use rl-garden's `examples/pretrain_offline.py cql` or
`examples/pretrain_offline.py calql` for standalone offline CQL/Cal-QL checkpoints. Use
the `wsrl` subcommand when the checkpoint should be resumed by the WSRL
offline→online flow. This entrypoint infers obs/action specs from the H5 and
produces pretrained checkpoints that can later be loaded into compatible
live-env training runs for online fine-tuning. See
`rl-garden/docs/design/wsrl-overview.md#offline-only-pretraining-no-sim-env` in the
`rl-garden/` submodule for
the full workflow.


## Partial control and training provenance

Select parts with `--teleop-selected-parts` (`arm.arm` / `arm.gripper` on the tutorial station,
`left.arm`, `right.arm`, `left.gripper`, `right.gripper` on two arms). Default
`--teleop-idle-behavior hold` holds unselected parts; `policy` leaves them autonomous.
An empty selection uses the source's available parts. A source without a gripper channel
never supplies an expert gripper label. Continuous no-gripper HIL is supported; hybrid/discrete
branches require a gripper. Syncing rows remain executed/archived, but their training projection
has no expert supervision. See [Real-world experiments](real-world-experiments.md) for masked
BC/SiLRI/residual losses, explicit outcomes and durable records.
