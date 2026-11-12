# real-world-rl

**Documentation:** [English](docs/README.md) · [中文文档入口](docs/README.zh-CN.md)

Real-world RL orchestration split out of
[rl-garden](https://github.com/JaimeParker/rl-garden): `ActorLoop`/
`LearnerLoop`/sync (SERL, HIL-SERL, residual and SiLRI variants), the
`franka_real` and `real_robot` env backends, the `TeleopSource` registry and
teleop intervention wrapper, and the reward-classifier data-collection tool.

This package imports `rl_garden` as a normal dependency (algorithms,
buffers, policies, the `EnvBackend`/`BasePolicy` contracts) and registers
`franka_real` through rl-garden's `rlgarden.env_backends` entry-point group
— no file inside the `rl_garden` package itself needs editing to add it.
Hardware/communication code lives in this repository in the torch-free device layer of
`real_world_rl` ([device-layer guide](docs/devices/README.md)); the learning side (torch, rl-garden) is the `learning` extra, so a robot PC that
only serves the daemon installs `pip install -e '.[franky,grpc]'`. `franka_real`'s own coupling
to the device layer is HTTP/JSON only (`FrankaBridgeClient`), no shared Python types.

This repository is the top level for real-robot work. It carries rl-garden as a
git submodule under [`rl-garden/`](rl-garden), pinned to the rl-garden commit the
tests here pass against.

## Methods

Each method is a subcommand of `rwrl train <method> --role {actor,learner,eval}`
(`real_world_bc` has no roles). `residual_hil_serl` and `silri` are also registered on rl-garden's
online registry through the `rlgarden.algorithms` entry point, so `examples/deploy_real_robot.py`
can rebuild their agents from a checkpoint (`rl-garden-eval` cannot evaluate residual checkpoints:
they need a base provider).
Use `rwrl train`, `rwrl record`, `rwrl deploy` and `rwrl collect-reward` for training,
recording, deployment and reward-classifier data collection. The existing console scripts,
Python module entrypoints and scripts in `examples/` remain compatibility aliases.

SpaceMouse defaults to `--spacemouse-mode tracker`; opt into `--spacemouse-mode serl`
for raw motion takeover, release returning control to the policy, and held-button gripper control, with
configured xyz/RPY safety boxes ([mode guide](docs/teleop.md#spacemouse)).

| Method | What it is | Guide |
|---|---|---|
| `serl` | Two-process SERL: actor on the robot PC, learner on the GPU box, RLPD (state or image obs). | [Running on hardware](#running-on-hardware) |
| `hil_serl` | `serl` plus human interventions (teleop wrapper), a growing demo buffer, an optional reward classifier and a DQN gripper (`RLPDHybrid`). | [Running on hardware](#running-on-hardware), [`docs/hil-serl-roadmap.md`](docs/hil-serl-roadmap.md) |
| `residual_hil_serl` | `hil_serl` with a bounded residual on top of a frozen base policy (zero, rl-garden checkpoint or ACT); `--residual-variant continuous\|hybrid`. | [`docs/residual.md`](docs/residual.md) |
| `silri` | HIL-RL (SiLRI): actor pulled towards a learned model of the human's interventions under a state-wise Lagrange multiplier. | [`docs/silri.md`](docs/silri.md) |
| `real_world_bc` | Offline behaviour cloning over demo pkls; no actor/learner split. | [Demo and snapshot contract sidecar](#demo-and-snapshot-contract-sidecar) |

## Repository layout

```
real-world-rl/                   repository real-world-rl (top level)
  pyproject.toml                 distribution real-world-rl (package real_world_rl; rl_garden in the learning extra)
  rl-garden/                     git submodule: rl-garden, pinned to the tested commit
  real_world_rl/                 package real_world_rl
    loops/                       actor/learner/sync: base classes (actor, learner, sync) and one module per method (serl, hil_serl, residual, silri)
    algorithms/ policies/        real-world algorithms and policies (residual, SiLRI, base action providers)
    buffers/                     InterventionReplayBuffer (residual, SiLRI)
    envs/                        env backends (franka_real, real_robot, real_robot_bimanual) and wrappers
    teleop/ kinematics/          teleop sources; robot kinematics (numpy)
    training/                    registry, shared args, env building, entrypoints/ (one per method); python -m entry
    demos/ reward/               demo recording, reward-classifier data collection
    deploy.py                    eval-only deployment of a checkpoint
    common/ integrations/        small shared helpers; external integrations (integrations/rlinf)
    devices/ server/             torch-free device layer: devices and backends; the robot daemon and RemoteRobot
    discovery/ doctor/           hardware discovery and station loading; device checks
    setup/ supervise/            station setup wizard; process supervisor and heartbeat
    cli/                         the rwrl command
    legacy/                      legacy controllers, ROS1 Franka bridge and old teleop, kept for evaluation (no rwrl command)
  docs/                          README.md (index), tutorial/ (00-overview ... 08-deploy), devices/ (device-layer guide), bimanual.md, teleop.md, spacemouse-commissioning.md, gello.md, recording-demos.md, residual.md, silri.md, hil-serl-roadmap.md
  examples/                      train_real_world.py, deploy_real_robot.py, teleop demo and recording scripts; presets/hil_single_arm.yaml; stations/ (example station yamls)
  configs/                       RLinf config
  tests/                         tests/devices/ covers the device layer
```

The device layer (`real_world_rl/{devices,server,discovery,setup,supervise,doctor}`) was the
`robot_infra` package, before that the separate `rlgarden-robot-infra` repository; its history is
preserved. It does not import `rl_garden` or the
learning side of `real_world_rl`; its commands (`setup`, `daemon`, `supervise`, `discover`) run
as `rwrl` subcommands.

Formerly `rlgarden-real-world` (package `rl_garden_real_world`, console scripts `rlgarden-real-*`); renamed in November 2026.

## Installation

On the NUC or development machine, start in a parent directory with no environment active
and clone with the pinned rl-garden submodule:

```bash
git clone --recurse-submodules https://github.com/JaimeParker/real-world-rl.git
cd real-world-rl
```

From the repository root on that same machine, with no environment active, run:

```bash
bash scripts/install.sh
source .venv/bin/activate
```

The installer creates or reuses `.venv` using `python3.10` by default, clears inherited
`PYTHONPATH`, and installs rl-garden and this project in editable mode with the
`learning`, `franka-gello` and `test` extras. Without `--station`, the final doctor uses mock
backends: this does not verify hardware. See the [hardware tutorial](docs/tutorial/01-install.md)
for station setup; any venv or conda env works for the manual route in
[step 4](docs/tutorial/01-install.md#4-install-the-project-environment). To check an
existing station, run on the NUC, repository root, with the project environment active:

```bash
bash scripts/install.sh --station local/station.yaml
```

Doctor owns dependency checks (layer 0) and hardware diagnostics. For a libfranka
mismatch, follow its `--install-sdk` hint to select the wheel for the robot server.
The default `franka-gello` extra retains gRPC for the optional daemon route.
Use `--extras learning,franka-gello,spacemouse` for both controllers, `--no-doctor` to skip
the final check, or `--torch-index https://download.pytorch.org/whl/cpu` to install
torch from the CPU index first. `bash scripts/install.sh --help` lists all options.

`rl_garden==0.0.1` (in the `learning` extra) is supplied by the same pip invocation from the
`rl-garden/` submodule; the version is not a commit lock. Development uses the pinned
submodule. An isolated `pip install '.[learning]'` may not resolve it from a public index.
For an existing checkout with an uninitialized submodule, run `git submodule update --init`
on the NUC/development machine, from the repository root (no venv required), then retry the
installer.

A robot PC that only serves the daemon needs neither the submodule nor torch (see the
[device-layer guide](docs/devices/README.md)). On that PC, from the repository root with no
environment active, run `bash scripts/install.sh --daemon-only` (extras `franky,grpc`), or
in an environment of your own `pip install -e '.[franky,grpc]'` plus the extras for your hardware.
For SpaceMouse, on the NUC/device host, repository root, project environment active,
run `pip install -e '.[spacemouse]'` to add `hidapi==0.14.0.post4` (import `hid`) for
the in-tree reader; follow the [udev steps](docs/teleop.md#spacemouse-installation-and-linux-permissions).
This path is not yet verified on hardware.

## Tests

No hardware or network is needed; the tests use fake robots and teleops. Run them from the
repository root, after the editable install above or with the two source trees on `PYTHONPATH`:

```bash
python -m pytest tests
PYTHONPATH=.:rl-garden python -m pytest tests   # without installing
```

rl-garden's own suite runs inside the submodule (see `rl-garden/.agents/runbooks/testing.md`).

## Running on hardware

**New to the hardware?** The [tutorial](docs/tutorial/00-overview.md) (index:
[docs/README.md](docs/README.md)) is a first-day walkthrough for one Franka arm with a GELLO leader
arm: station setup, demo recording, BC and ACT, HIL-SERL / residual / SiLRI, and deployment; two
arms are in [docs/bimanual.md](docs/bimanual.md#two-arms-the-tutorial-steps). This section is the
reference.

Everything below drives a `real_robot` env (any device-layer robot). `franka_real` (the HTTP
bridge) is always cartesian 7-D and is not covered here. Install this package
(`pip install -e ./rl-garden -e '.[learning]'`) so that the `real_robot` backend is registered through the
`rlgarden.env_backends` entry point; without it `--env-backend real_robot` fails with
`Unknown env backend 'real_robot'`.
Two arms as one env: `--env-backend real_robot_bimanual`, see [`docs/bimanual.md`](docs/bimanual.md).

### Robot daemon

The robot daemon runs on the robot PC (see the device-layer guide,
[How to Start](docs/devices/README.md#how-to-start)):

```bash
rwrl setup                  # writes ./local/station.yaml (the setup wizard)
rwrl daemon --watchdog-s 5  # serves it: no --station needed
```

The [setup wizard](docs/devices/README.md#setup-wizard) writes the station yaml of a Franka station
(one daemon process per robot for a two-arm station). A hand-written robot yaml is served with
`rwrl daemon --station robot.yaml --bind 0.0.0.0:50051 --watchdog-s 5`.

The daemon idles a servoing arm that gets no action for `--watchdog-s` (default 0.5 s), and the
env enters servo mode at the end of `reset()`. If the first policy step after a reset can take
longer than that (first inference, a busy learner), raise `--watchdog-s`; otherwise the first
`step()` fails with `DeviceFault: the daemon watchdog stopped the arm ...`.

### Configuring the env

Cameras and image size come from the observation group, not from `--real_robot.*`:
`--obs.rgb wrist --obs.image-size 128 128` (a remote robot only reports its camera sizes after
connecting, so `--obs.image-size` is required with cameras). `--real_robot.cameras` is filled
in from `--obs.rgb`; setting it to something else raises `ObservationContractError`.
`--real_robot.joint_limits` is a list of `[lo, hi]` pairs, which the command line cannot
express (it flattens them); give it in a YAML preset passed with `--config`, which any
`train_real_world.py` run accepts (`--print-config` validates it without touching the robot).

The single-arm preset is
[`examples/presets/hil_single_arm.yaml`](examples/presets/hil_single_arm.yaml) (`joint_delta`, one
128 x 128 `wrist` camera, the daemon at `127.0.0.1:50051`); copy it to `hil.yaml` and edit
`remote` and `obs.rgb` ([tutorial 02](docs/tutorial/02-station.md#the-env-preset-hilyaml)). A
`joint_abs` preset changes the mode and adds the limits:

```yaml
real_robot:
  action_mode: joint_abs
  joint_limits: [[-2.8, 2.8], [-1.7, 1.7], [-2.8, 2.8], [-3.0, -0.1], [-2.8, 2.8], [0.0, 3.7], [-2.8, 2.8]]
```

### Action contract

`action_mode` fixes what a normalized action in [-1, 1] means; a trailing gripper dimension is
added when `gripper_in_action` is true (the default).

| `action_mode` | arm action | `joint_limits` | needs |
|---|---|---|---|
| `cartesian_delta` | 6-D twist: xyz scaled by `xyz_scale_m`, rotation by `rot_scale_rad` | optional | `tcp_pose` in the state |
| `joint_delta` | `num_joints` deltas scaled by `joint_scale_rad` | optional (clips) | `joint_pos` |
| `joint_abs` | `num_joints` values mapped linearly onto `joint_limits` | **required** (the mapping is built at construction) | `joint_pos`, and the robot must report `max_delta_per_step.joint_rad` |

hil_serl (`--env-backend real_robot`) also checks the teleop source against this contract when
the env is built (`TeleopInterventionWrapper`):

| teleop `action_mode` | gripper | support |
|---|---|---|
| `cartesian_delta` | yes | supported (spacemouse / pico / the robot's teleop part) |
| `joint_delta` / `joint_abs` | yes | supported through `--teleop-device robot` (the robot yaml's `teleop:` section), `leader_arm` / `gello` (a GELLO) and, for two arms, `bimanual` |
| any compatible mode | no (`gripper_in_action=false`) | supported by continuous HIL RLPD, continuous residual and continuous SiLRI; hybrid/discrete-gripper variants require at least one gripper |

### Safety limits

The limits live in the robot yaml on the robot PC (`safety_box`, `joint_limits`,
`max_delta_per_step`) and are enforced by the daemon; the env can only be **tighter**:

- After connecting (at the first `reset()`), an unset `safety_box` / `joint_limits` /
  `home_joints` is inherited from the robot. An env limit looser than the robot's raises
  `DeviceFault` on that first reset; tighten the env value or loosen the yaml.
- A `joint_abs` env additionally requires the robot to report `max_delta_per_step.joint_rad`,
  because consecutive `joint_abs` actions can jump across the whole joint range. This holds for
  every entry point (hil_serl, deploy). A robot without it fails the first reset with
  `DeviceFault: action_mode joint_abs needs the robot's per-step joint limit ...`.
- `info["safety_clips"]` of `step()` is the cumulative number of servo commands the device
  clipped. It is refreshed every 10 steps (it costs a daemon round trip) and is `None` when the
  robot cannot report it. A rising count means the policy keeps asking for targets outside the
  limits. `reset()` homes with `move_to_joints`, so a `home_joints` outside the robot's limits
  raises `SafetyViolation` there.

### Teleoperation

The HIL methods (`hil_serl`, `residual_hil_serl`, `silri`) take human interventions through a
`TeleopSource` selected with `--teleop-device` (`pico`, `spacemouse`, `robot`, `fake`, `gello`,
`leader_arm`, `keyboard`, `leader_cartesian` or, for two arms, `bimanual`; `spacemouse_twist` is
the raw-twist SpaceMouse of the sim examples, not for real robots). Only the actor and
eval processes open the device; the learner builds its env with the teleop wrapper disabled, so no
HID or ZMQ device is opened on the GPU box. `--teleop-device spacemouse` is a SpaceMouse on the
actor's machine, driven through the device layer's `SpaceMouseTeleop` (needs `hidapi==0.14.0.post4`, import `hid`).
The `TeleopSource` contract, the `info` keys the wrapper writes (`intervene_action`, `policy_action`,
`intervened`, `intervention_onset`, `human_episode_end`, `teleop_events`), the demo and recording
scripts, and how to add a device are in [`docs/teleop.md`](docs/teleop.md).

### Teleop (`--teleop-device robot`)

hil_serl can take interventions from the teleop part of the robot itself (the robot yaml's
`teleop:` section: SpaceMouse, a GELLO leader arm or the `fake` backend), served by the daemon. For
leader arms (`--teleop-device leader_arm`/`gello`, onset policies, event devices) see
[Leader arms (GELLO)](docs/teleop.md#leader-arms-gello).
`--teleop-options-yaml PATH` supplies device options the CLI has no flags for (GELLO port and
calibration, SpaceMouse index, daemon address), and `--teleop-event-device keyboard|stdin` takes
the intervention toggle and events from a second device; both are described in
[`docs/teleop.md`](docs/teleop.md).
`--teleop-device spacemouse` opens a SpaceMouse on the actor's own machine through the same
device-layer `SpaceMouseTeleop` instead (no robot yaml `teleop:` section needed): button 0 takes
over and hands back, button 1 toggles the gripper, the puck is a velocity through the cartesian
command tracker, and the labels come from `--teleop-event-device stdin`. The device
table, the `TeleopSource` contract and how to add a device are in [`docs/teleop.md`](docs/teleop.md).
For `--teleop-device robot`:

- `cartesian_delta` actions pass through as they are; `joint_abs` is normalized onto
  `joint_limits`; `joint_delta` becomes `(q_leader - q_follower) / joint_scale_rad` (needs
  `joint_pos` in `state_fields`).
- A teleop gripper `g` in [0, 1] becomes `2g - 1`. A teleop without a gripper holds the gripper
  action last sent to the env (open, +1, right after a reset), so taking over the arm does not
  drop a grasped object.
- A teleop `episode_end` event ends the episode as `truncated` and sets
  `info["human_episode_end"]`; events pressed while the robot homes during `reset()` are dropped.
  A teleop read failure stops the actor (`DeviceFault`) instead of silently dropping the
  intervention.
- A remote robot only reports its teleop part once connected, so declare it with
  `--robot-teleop-action-mode {cartesian_delta,joint_abs,joint_delta}` (and
  `--no-robot-teleop-has-gripper` for a device without a gripper). The declaration is checked
  against the real part on connect and a mismatch raises `DeviceFault`.
- A configured teleop must be present for the robot to connect. For unattended deployment, use
  `--no-enable-teleop` and a robot YAML without `teleop:`; keep the devices for assisted deployment.

Hardware-free try-out with the fake teleop (robot yaml; `on_end: loop` repeats the script so
episodes keep ending):

```yaml
teleop:
  name: leader
  backend: fake
  options:
    action_space: joint_abs
    num_joints: 7
    has_gripper: true
    on_end: loop
    script:
      - {action: [0, 0, 0, 0, 0, 0, 0], active: false, hold: 5}
      - {action: [0.1, 0, 0, -1.5, 0, 1.5, 0.7], gripper: 0.0, active: true, hold: 5}
      - {events: {episode_end: true}}
```

### Deploy

```bash
rwrl deploy --checkpoint runs/.../checkpoints/checkpoint_<step>.pt \
    --algo rlpd_hybrid --real_robot.remote ROBOT_PC:50051 --real_robot.token T \
    --episodes 5 --max-steps-per-episode 100
```

(`--algo rlpd` for a serl checkpoint, `rlpd_hybrid` for hil_serl, `real_world_bc` for a
`real_world_bc` run, `residual_hil_serl` / `residual_hil_serl_hybrid` and `silri` as in their
sections below; `--env-backend real_robot_bimanual` for a two-arm run, see
[`docs/bimanual.md`](docs/bimanual.md#deploy).) The training run's
`config.json` (next to `checkpoints/`) is the source of truth for the action/observation
contract:

- The deploy env takes `action_mode`, `gripper_in_action`, `num_joints`, `joint_limits`,
  `xyz_scale_m`, `rot_scale_rad`, `joint_scale_rad`, `state_fields`, `image_size`, `control_hz`
  and task/control/preprocessing settings from the run. Station-owned reset facts and episode caps
  can be adjusted for deployment. It re-applies the run's observation wrappers by itself:
  `obs.frame_stack` (stacking the run's `classifier_image_keys`) and `convert_obs_rotation`.
  `--real_robot.*` only supplies connection fields (`robot`, `remote`, `token`, `home_joints`,
  `safety_box`, `reset_settle_s`, ...).
- A contract field given on the CLI that differs from the run's is an error (exit with a
  `ValueError`: `--real_robot.* contradicts the training run's real_robot config ...`).
  **Limitation:** a CLI value equal to the dataclass default cannot be told apart from an unset
  one, so `--real_robot.action_mode cartesian_delta` against a `joint_abs` run silently uses
  the run's `joint_abs`.
- Runs without a `real_robot` config (older runs, `franka_real`) fall back to the CLI with a
  warning, and the contract is then not verified.
- A `DeviceError` (daemon unreachable, watchdog, safety) prints its message and hint and exits
  with code 2.
- The daemon's `--jpeg-quality` is not part of the run's config: start the deploy daemon with
  the same value (or none) the training daemon used.
- Deployment accepts `bundle_manifest.json` or a standalone `.pt`. `enable_teleop` defaults to
  true; an active task or explicit `--teleop-device` composes the operator setup. Use
  `--no-enable-teleop` for an unattended run. Classifiers/tasks can provide explicit outcomes;
  primary `success_rate` means autonomous success over completed episodes. See
  [tutorial 08](docs/tutorial/08-deploy.md) for both deployment modes.

### Residual RL (`residual_hil_serl`)

`residual_hil_serl` trains a bounded residual `clamp(base + scale * u, -1, 1)` on top of a
frozen base policy (`--base-provider zero | checkpoint:<path>?algo=<name> | act:<path>`),
with hil_serl's actor / learner / eval roles, demo buffer and interventions. Human control can
select parts; the optional BC term uses actual human dimensions with nonzero residual scales.
`--residual-variant continuous` (`ResidualRLPD`) puts the residual on every action dim,
`hybrid` (`ResidualRLPDHybrid`) on the arm dims with the DQN gripper. Only the actor, eval
and deploy build the base policy; deploy rebuilds it from the checkpoint metadata
(`--algo residual_hil_serl` or `residual_hil_serl_hybrid`). See
[`docs/residual.md`](docs/residual.md).

### HIL-RL (`silri`)

`silri` implements HIL-RL / SiLRI (arXiv 2512.24288): an actor-critic whose actor is pulled
towards a learned Gaussian model of the human's interventions under a state-wise Lagrange
multiplier, so consistent human behaviour constrains the policy and inconsistent behaviour
leaves it to the critic. Same actor / learner / eval roles, teleop devices and demo pkls as
hil_serl; the learner pretrains on the demo buffer, refreshes the human model as
interventions arrive, and the actor tags the transition before each takeover
(`intervention_onset`). Deploy with `--algo silri`. See [`docs/silri.md`](docs/silri.md).

### Demo and snapshot contract sidecar

The hil_serl learner's buffer and demo-buffer snapshots get a `<name>.contract.json` next to
each pkl (`transitions_N.pkl` -> `transitions_N.contract.json`); `--demo-dataset-paths` pkls
are read together with their sidecar. One line of sorted JSON:

```json
{"action_mode": "joint_abs", "arm_action_dim": 7, "gripper_in_action": true, "joint_limits": [[-2.8, 2.8], ...], "joint_scale_rad": 0.05, "num_joints": 7, "rot_scale_rad": 0.1, "state_fields": ["joint_pos", "gripper_pos"], "xyz_scale_m": 0.02}
```

On load the sidecar is compared with the env's contract (floats with `allclose`). A mismatch
raises `ValueError: ... recorded action contract does not match the env's (differs in
['joint_limits']) ...`. A sidecar written before the scales and limits existed is checked on the
keys it has, with a warning. A pkl without a sidecar is checked for the action width only, with
a warning (`... has no action-contract metadata (<name>.contract.json missing) ...`).

### End-to-end try-out with the fake robot

One machine, no hardware (verified in the project container; the training runs are short and
the policy is untrained). Robot PC side, with a fake-backend `robot.yaml` of type `franka`
(`arm_backend: fake`, `gripper: {backend: fake}`, a fake camera named `wrist`, joint limits and
`max_delta_per_step: {xyz_m: 0.05, joint_rad: 0.1}` as in the example station yamls, and the
fake teleop above):

```bash
rwrl daemon --station robot.yaml --bind 127.0.0.1:50061 --token T \
    --jpeg-quality 90 --watchdog-s 5 --no-preflight
```

Learner and actor (two terminals; `hil.yaml` is the preset above with `remote: 127.0.0.1:50061`,
`token: T`, and the `joint_abs` mode and limits):

```bash
COMMON="--config hil.yaml --teleop-device robot --robot-teleop-action-mode joint_abs \
    --log-dir runs --log-type none --exp-name smoke --batch-size 16 --learning-starts 32 \
    --buffer-size 2000 --checkpoint-freq 10 --sync-host 127.0.0.1 --sync-port 6055 --publish-freq 10"
rwrl train hil_serl --role learner $COMMON
rwrl train hil_serl --role actor $COMMON
```

Stop both with Ctrl-C. The learner writes `runs/smoke/config.json` and
`runs/smoke/checkpoints/checkpoint_<update>.pt`. Then deploy the last checkpoint:

```bash
rwrl deploy --checkpoint runs/smoke/checkpoints/checkpoint_<update>.pt \
    --algo rlpd_hybrid --real_robot.remote 127.0.0.1:50061 --real_robot.token T \
    --episodes 2 --max-steps-per-episode 15
```

which prints one `[deploy] episode=... steps=15 ... health_ok=True` line per episode and a
`[deploy] summary ...` line. On real hardware the same commands apply with the daemon on the
robot PC, `--bind 0.0.0.0:50051`, the real `ROBOT_PC:50051`, `--sync-host` set to the learner's
reachable address, and a real `control_hz`.

Two-process real-robot training for SERL (state-only or image) uses the same flags:

```bash
rwrl train serl --role learner ...
rwrl train serl --role actor --env-backend real_robot \
    --real_robot.remote ROBOT_PC:50051 --real_robot.num_joints 7 \
    --obs.rgb wrist --obs.image-size 128 128 ...
```

(the `serl` real-robot flags were only checked with `--print-config`; no `serl` run was done.)

RLinf: start from
[`configs/rlinf/realworld_rlgarden_franka.yaml`](configs/rlinf/realworld_rlgarden_franka.yaml).
The package must be importable in the Ray env-worker process, and RLinf kills
any roscore running on the env node.

**RLinf**: RLinf drives this package through Gymnasium registration, not
through rl-garden's adapter. In an RLinf real-world config set the env block's
`env_type: real` and `init_params.id:
"real_world_rl.integrations.rlinf.register:RLGardenReal-v0"`; RLinf's
`RealWorldEnv` then `gym.make`s `RealRobotEnv` (one robot per env worker) with
`override_cfg` as its config. See
[`configs/rlinf/realworld_rlgarden_franka.yaml`](configs/rlinf/realworld_rlgarden_franka.yaml)
for the env block and its hazards, and `tests/test_rlinf_registration.py` for
the contract. rl-garden's Pattern-A RLinf adapter
(`rl_garden/integrations/rlinf/{sac_actor,sac_rollout}.py`) runs RLinf's own
env worker and has no real-robot or image path, so it cannot drive this
package.

Run training with `rwrl train serl --role ...`
(see that file's module docstring for the two-process actor/learner launch
shape). See [`docs/teleop.md`](docs/teleop.md) for the teleop demo/recording
workflow, and [`docs/hil-serl-roadmap.md`](docs/hil-serl-roadmap.md) for what
HIL-SERL capability was and wasn't migrated, and why.


Experiment recovery, autonomous/assisted metrics, part takeover and trajectory archives are
documented in [Real-world experiments](docs/real-world-experiments.md). Human action masks
keep partial demonstrations aligned through BC, residual and SiLRI training; no-gripper
HIL configurations use continuous policies. Hardware validation remains pending.

Current learner recovery uses `bundle_manifest.json` together with its checkpoint directory and
SQLite journal. Actor uploads use a persistent outbox; keep actor identity/path across a restart.
Selected parts and observation preprocessing enter the experiment contract. See
[tutorial 07](docs/tutorial/07-hil-training.md#resume-an-interrupted-run) and
[Arm control profiles](docs/devices/control-profiles.md).
