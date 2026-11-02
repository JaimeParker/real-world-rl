# Two arms as one env (`real_robot_bimanual`)

`--env-backend real_robot_bimanual` drives two `robot_infra` arms as one env: one
`RealRobotEnv` per arm (each a local robot or its own daemon), stepped concurrently, with one
action and one observation for both. `real_robot` (one arm) is unchanged.

For a first-day walkthrough of a two-arm station (the setup wizard, demos, BC, HIL training and
deploy in order) see [getting-started-hardware.md](getting-started-hardware.md).

## Config

`--real_robot_bimanual.*` is a `BimanualRealRobotEnvConfig`: one `RealRobotEnvConfig` per arm
under `left` and `right`, plus `arm_names`, `max_episode_steps` and `reset_settle_s`.

| field | where | notes |
|---|---|---|
| `robot` / `remote` / `token`, `safety_box`, `joint_limits`, `home_joints`, `cameras` | per arm | `cameras` must be unique across arms; `joint_limits` on both arms or on neither |
| `action_mode`, `num_joints`, `control_hz`, `gripper_in_action`, `xyz_scale_m`, `rot_scale_rad`, `joint_scale_rad`, `state_fields`, `image_size` | per arm, equal on both | a mismatch raises `ValueError` when the config is built |
| `arm_names` | two-arm config | default `[left, right]`; names the observation keys and the contract's arms |
| `max_episode_steps`, `reset_settle_s` | two-arm config | apply to both arms; the per-arm `max_episode_steps` must stay unset |

Per-arm lists such as `joint_limits` go in a YAML preset (`--config`), as for `real_robot`:

```yaml
env_backend: real_robot_bimanual
env_id: my_bimanual_task
obs:
  rgb: [left_wrist, right_wrist]   # every arm's cameras, in arm order
  image_size: [64, 64]
real_robot_bimanual:
  max_episode_steps: 200
  left:
    remote: LEFT_PC:50051
    token: T
    action_mode: joint_delta
    cameras: [left_wrist]
    home_joints: [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
  right:
    remote: RIGHT_PC:50051
    token: T
    action_mode: joint_delta
    cameras: [right_wrist]
    home_joints: [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
```

`--obs.rgb` must list exactly the arms' cameras in arm order (left's, then right's). Unlike
`real_robot`, the backend does not fill the camera lists from `--obs.rgb`: it cannot tell which
arm's robot serves a camera, so set `--real_robot_bimanual.<arm>.cameras`.

## Action

The action-contract layout (`rl_garden_real_world/action_contract.py`) with `num_arms=2`:

```
[left arm block, right arm block, left gripper, right gripper]
```

Each arm block is 6 dims (`cartesian_delta`) or `num_joints` (joint modes) and means what it
means for one arm (see the action contract in the [README](../README.md#action-contract)); the
two gripper dims exist with `gripper_in_action`. The env's `action_contract` carries
`num_arms: 2`, `arm_names` and `joint_limits` per arm (`[left limits, right limits]`, or null).

## Observation

The env's own observation is `{"state": {"<arm>_<field>": ...}, "frames": {cam: ...}}`
(`left_tcp_pose`, `right_joint_pos`, ...). The backend flattens it to rl-garden keys:

- `state`: the left arm's `state_fields` in config order, then the right arm's;
- `rgb_<cam>`: one per camera, in arm order.

`--convert-obs-rotation` converts both arms' `tcp_pose` quaternions to rotation vectors (state
loses one dim per arm).

Step `info` carries every arm's keys with its prefix (`left_tcp_pose`, `right_rtt_ms`,
`left_safety_clips`, ...), plus `elapsed_steps` and `rtt_ms` (the slower arm's round trip).

## Running two daemons

Each arm is served by its own `robot_infra` daemon with its own robot yaml and port (see the
robot_infra README, [How to Start](../robot_infra/README.md#how-to-start)), for example on two
robot PCs or on one PC with two ports:

```bash
python -m robot_infra.server --config left.yaml --bind 0.0.0.0:50051 --watchdog-s 5
python -m robot_infra.server --config right.yaml --bind 0.0.0.0:50052 --watchdog-s 5
```

A robot_infra station yaml with a `robots:` list describes both arms of a station.
`python -m robot_infra.setup` writes it (the [setup wizard](../robot_infra/README.md#setup-wizard))
and `python -m robot_infra.server --watchdog-s 5` with no `--config` starts one daemon process per
robot on the ports in the file's `daemon.bind` entries, so the two commands above are only needed
for separate yaml files. Point `left.remote` / `right.remote` at the two addresses.

## Teleop and demo recording

`--teleop-device bimanual` drives both arms, with one leader per arm; `--teleop-arm-device` picks
each arm's device (`leader_arm`, the default: a GELLO served by that arm's daemon as its robot's
`teleop:` part; `robot`: any teleop part of that arm's robot; `gello`: a GELLO plugged into the
actor's machine). The operator overrides the whole action; while one arm's operator intervenes
and the other does not, the other arm is held still with its gripper unchanged.
[teleop.md](teleop.md#two-arms-bimanual) has the details and the per-arm options yaml.

HIL actor over the two daemons, one keyboard for the toggle and the buttons:

```bash
python examples/train_real_world.py hil_serl --role actor --config bimanual.yaml \
    --teleop-device bimanual --teleop-arm-device leader_arm \
    --robot-teleop-action-mode joint_delta --teleop-event-device keyboard
```

Two GELLOs on the actor's machine instead, with their options taken from the robot_infra station
yaml (`robots: {left, right}`, each with its `teleop:` section):

```bash
python examples/train_real_world.py hil_serl --role actor --config bimanual.yaml \
    --teleop-device bimanual --teleop-arm-device gello \
    --teleop-options-yaml local/station.yaml --teleop-event-device keyboard
```

Demo recording takes the same flags ([recording-demos.md](recording-demos.md)):

```bash
python examples/record_real_demos.py --config bimanual.yaml \
    --teleop-device bimanual --teleop-arm-device leader_arm \
    --robot-teleop-action-mode joint_abs --teleop-event-device keyboard \
    --output-path demos/bimanual_20.pkl --num-successes 20 --max-episode-steps 300
```

`--teleop-device fake` declares both arms too: a dry run of the two-arm actor without an operator.

## Deploy

`examples/deploy_real_robot.py --env-backend real_robot_bimanual` with connection settings under
`--real_robot_bimanual.{left,right}.*`. The training run's two-arm config decides `arm_names`,
`max_episode_steps` and each arm's contract fields; a run trained on one backend cannot be
deployed with the other. `--algo rlpd_hybrid` (hil_serl) rebuilds the two-gripper agent from the
deploy env's contract, and `--algo real_world_bc` deploys a `real_world_bc` run:

```bash
python examples/deploy_real_robot.py --checkpoint runs/<bc run>/checkpoints/final.pt \
    --algo real_world_bc --env-backend real_robot_bimanual \
    --real_robot_bimanual.left.remote LEFT_PC:50051 --real_robot_bimanual.right.remote RIGHT_PC:50052
```

Add each arm's `token`, `cameras` and `image_size` (equal to the training run's) as for training.

## Verified hardware-free

`tests/test_bimanual_station_smoke.py` runs the whole two-arm workflow on fakes: a station yaml
with `robots: {left, right}` (fake arm, gripper, two cameras and a scripted `joint_abs` leader part
per arm, a token on one daemon) loaded with `robot_infra.discovery.autoconfig.load_station`, each
robot served by its own gRPC daemon (`robot_infra.server.daemon.build_daemon`, what
`python -m robot_infra.server` starts per robot) on loopback. Against those daemons it builds the
`real_robot_bimanual` env through the backend registry (`remote` per arm, `joint_delta`, four
`--obs.rgb` cameras) and runs, in order, what these commands run:

```bash
python examples/train_real_world.py hil_serl --role actor --config bimanual.yaml \
    --teleop-device bimanual --teleop-arm-device leader_arm --robot-teleop-action-mode joint_abs
python examples/record_real_demos.py --config bimanual.yaml \
    --teleop-device bimanual --teleop-arm-device leader_arm --robot-teleop-action-mode joint_abs \
    --output-path demos/bimanual.pkl --num-successes 1
python examples/train_real_world.py residual_hil_serl --role actor --config bimanual.yaml \
    --teleop-device bimanual --teleop-arm-device leader_arm --robot-teleop-action-mode joint_abs \
    --residual-variant hybrid --base-provider zero
python examples/train_real_world.py silri --role actor --config bimanual.yaml \
    --teleop-device bimanual --teleop-arm-device leader_arm --robot-teleop-action-mode joint_abs
python examples/train_real_world.py hil_serl --role learner --config bimanual.yaml \
    --demo-dataset-paths demos/bimanual.pkl            # likewise residual_hil_serl
                                                       # (--allow-base-less-demos) and silri
python examples/train_real_world.py real_world_bc --config bimanual.yaml \
    --demo-dataset-paths demos/bimanual.pkl
python examples/deploy_real_robot.py --checkpoint runs/<bc run>/checkpoints/final.pt \
    --algo real_world_bc --env-backend real_robot_bimanual ...
```

It checks the observation keys (`state` and one `rgb_<cam>` per camera), the 16-D action and
`num_arms=2`; that intervened rows follow the left leader while the right arm is held with its
gripper unchanged; that the demo pkl and its `.contract.json` sidecar (`num_arms`, `arm_names`,
per-arm `joint_limits`) load into the hil_serl, residual and SiLRI learners; that the agents get
a 9-way gripper head (`num_grippers=2`, from the env contract); and that the BC checkpoint deploys
on both daemons. The test is skipped when `grpc` is not installed.

## Limits

- No inter-arm collision check: each arm is clipped only by its own safety box, joint limits and
  device-layer limits. Keep the boxes disjoint.
- The single-arm teleop devices (`robot`, `leader_arm`, `gello`, `leader_cartesian`, `spacemouse`,
  `pico`) are rejected with `--env-backend real_robot_bimanual`; use `--teleop-device bimanual`.
