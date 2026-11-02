# Teleoperation

Human-in-the-loop intervention in `rl_garden_real_world`: a `TeleopSource` reads one operator
device, `InterventionWrapper` swaps the operator's action in for the policy's, and the demo
scripts in `examples/` use the same sources.

## The `TeleopSource` contract

`rl_garden_real_world.teleop` (`teleop/base.py`):

```python
@dataclass(frozen=True)
class TeleopReading:
    action: np.ndarray          # float32 (A,), env-normalized [-1, 1], gripper last when has_gripper
    intervened: bool            # the operator is driving; action only matters then
    episode_end: bool = False   # operator's episode-end button
    events: Mapping[str, bool] = {}   # success / failure / rerecord edges that fired, else empty
    timestamp: float = 0.0

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

| `device` | Source | Reads | Action space | Notes |
| --- | --- | --- | --- | --- |
| `pico` | `PicoTeleopSource` | Pico VR controller over a ZeroMQ `SUB` socket (`EETwistTeleOpWrapper`) | cartesian_delta, 6 + gripper | kwargs: `zmq_url`, `hand`, `pos_scale`, `rot_scale`, `twist_limit`, `intervention_threshold`. Data format below. |
| `spacemouse` | `SpaceMouseTeleopSource` | a SpaceMouse on this machine, through the device layer's `SpaceMouseTeleop` | cartesian_delta, 6 + gripper | kwargs: `device_index=0`, `deadzone=1e-3`, `invert_axes=()`, `poll_interval_s=0.005`, `hold_s=0.0`, `read_timeout_s`. Needs `pyspacemouse>=2,<3`, imported at `start()`. |
| `robot` | `TeleopPartSource` | the teleop part of the robot behind a `real_robot` env (robot yaml `teleop:` section, local or daemon-served) | cartesian_delta, joint_abs or joint_delta, mapped into the env space | kwargs: `teleop_contract`, `joint_limits`, `joint_scale_rad`, `read_timeout_s` (hil_serl: `--robot-teleop-action-mode`, `--no-robot-teleop-has-gripper`). The robot owns the part, so `close()` leaves it connected. |
| `fake` | `ScriptedTeleopSource` | scripted readings (tests; with no `readings` an operator who never intervenes) | follows the env's action contract | pass `readings=[TeleopReading, ...]` to script one; the last reading repeats. |

### SpaceMouse

`SpaceMouseTeleopSource` opens the device with the device layer's `SpaceMouseTeleop`
(`TeleopConfig(backend="spacemouse")`, a poll thread feeding `read()`) and maps its readings
into the env space with `TeleopPartSource`. Semantics are the device layer's:

- The puck drives a cartesian twist in `[-1, 1]` with axis map `[-y, x, z, -roll, -pitch, -yaw]`
  of the pyspacemouse state; `invert_axes` flips the sign of listed output indices (0-5).
- The operator intervenes while the twist norm exceeds `deadzone` or a button is held; `hold_s`
  keeps the intervention going that long after the last activity.
- Button 0 toggles the sticky gripper (open at start and after every `reset()`; action `+1`
  open, `-1` closed). Button 1 raises `episode_end`.
- Buttons act on press. A button still held when the env is reset fires again afterwards,
  because `reset()` clears the remembered button state.
- The SDK is imported when `start()` opens the device (the wrapper calls it), so building the
  source works without `pyspacemouse`; a missing SDK raises `SdkMissing` with the install hint.

hil_serl: `--teleop-device spacemouse`. The robot-yaml route (`--teleop-device robot` with a
`backend: spacemouse` teleop section) reads the same device but through the robot (and its
daemon), with the same semantics.

### Robot teleop part (`--teleop-device robot`)

hil_serl on `--env-backend real_robot` can also take interventions from the teleop part of
the robot itself (the robot yaml's `teleop:` section: spacemouse or the `fake` backend; no
leader-arm backend exists yet, served locally or by the robot daemon). `TeleopPartSource`
(`rl_garden_real_world/teleop/part.py`) adapts it, mapping cartesian or joint
(`joint_abs`/`joint_delta`) teleop actions into the env's `[-1, 1]` action space and forwarding
the device's `success`/`failure`/`rerecord` events; see the README's "Action contract" and
"Teleop (`--teleop-device robot`)" sections for the mapping, the `--robot-teleop-action-mode`
declaration and an example yaml.

## `InterventionWrapper`

`rl_garden_real_world/envs/wrappers/intervention.py`. `InterventionWrapper(env, source)` takes a
`TeleopSource`; `TeleopInterventionWrapper(env, teleop=None, device="pico", **kwargs)` builds one
by device name (or adapts the object passed as `teleop`). Both expose the source as `.source`
(alias `.teleop`).

`step(action)` polls the source first. While `reading.intervened` the operator's action replaces
`action` (as a `(1, A)` float32 tensor on the policy action's device). It always writes:

| `info` key | Value |
| --- | --- |
| `intervene_action` | the executed `(1, A)` action; **only while intervening** |
| `policy_action` | the action the policy proposed, as received |
| `intervened` | python bool: the operator's action was executed |
| `intervention_onset` | python bool: intervened now and not on the previous step of this episode |
| `human_episode_end` | python bool: the operator's episode-end button |
| `teleop_events` | `dict` of the `success`/`failure`/`rerecord` edges that fired; **only when some did** |

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
from rl_garden_real_world.teleop import TeleopReading, TeleopSource, register_teleop_source

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
contract (`rl_garden_real_world.action_contract.get_env_action_contract(env)`). Unknown names
raise `ValueError` listing the registered ones. Registration must happen before the name is used,
so a downstream package imports its module (or exposes it through its own entry point) before
building the env. Built-in sources are imported on first use of their name, so importing
`rl_garden_real_world.teleop` pulls in neither `zmq` nor `pyspacemouse`.

Actions are in the env's normalized `[-1, 1]` space, gripper last. For a cartesian device whose
twist is in physical units (as the Pico's), see how the demo scripts normalize it.

## EETwist TeleOp Interface (Pico)

`PicoTeleopSource` wraps `EETwistTeleOpWrapper` in `robot_infra/teleop/utils/telo_op_control_twist.py`
(in the `robot_infra` device layer). Its only device is `pico`; the wrapper receives device data
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
python robot_infra/teleop/examples/run_twist_input_inspector.py \
  --device pico \
  --zmq-url tcp://192.168.6.2:7777
```

The same demo with a SpaceMouse (`--spacemouse-index`, `--spacemouse-deadzone`,
`--spacemouse-invert-axes`, `--spacemouse-hold-s` tune it) or the idle `fake` source:

```bash
python examples/run_teleop_demo.py --env-id PegInsertionSidePegOnly-v1 --device spacemouse
```

`run_teleop_demo.py` creates the ManiSkill environment, builds the source with
`make_teleop_source`, polls teleop actions, applies controller action normalization to the
Pico's physical twist (SpaceMouse actions are already normalized), and steps the environment.

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

The operator's episode-end button (Pico episode-end button, SpaceMouse button 1) ends the
current episode. After each episode, the terminal prints the episode length, `max_episode_steps`, success flag, and
return, then asks whether to save the episode.

### Important Arguments

- `output_path`: H5 dataset file to write.
- `device`: `pico`, `spacemouse` or `fake` (with the same device-specific flags as the demo).
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
env, no eval), use `examples/pretrain_offline.py cql` or
`examples/pretrain_offline.py calql` for standalone offline CQL/Cal-QL checkpoints. Use
the `wsrl` subcommand when the checkpoint should be resumed by the WSRL
offline→online flow. This entrypoint infers obs/action specs from the H5 and
produces pretrained checkpoints that can later be loaded into compatible
live-env training runs for online fine-tuning. See
`docs/design/wsrl-overview.md#offline-only-pretraining-no-sim-env` in the
main `rl-garden` repo for
the full workflow.
