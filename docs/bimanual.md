# Two arms as one env (`real_robot_bimanual`)

`--env-backend real_robot_bimanual` drives two device-layer arms as one env: one
`RealRobotEnv` per arm (each a local robot or its own daemon), stepped concurrently, with one
action and one observation for both. `real_robot` (one arm) is unchanged.

The [tutorial](tutorial/00-overview.md) walks one arm from the setup wizard to deployment;
[Two arms: the tutorial steps](#two-arms-the-tutorial-steps) below lists what changes at each of
its steps for a two-arm station.

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

The action-contract layout (`real_world_rl/action_contract.py`) with `num_arms=2`:

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

Each arm is served by its own robot daemon with its own robot yaml and port (see the
device-layer guide, [How to Start](devices/README.md#how-to-start)), for example on two
robot PCs or on one PC with two ports:

```bash
rwrl daemon --station left.yaml --bind 0.0.0.0:50051 --watchdog-s 5
rwrl daemon --station right.yaml --bind 0.0.0.0:50052 --watchdog-s 5
```

A station yaml with a `robots:` list describes both arms of a station.
`rwrl setup` writes it (the [setup wizard](devices/README.md#setup-wizard))
and `rwrl daemon --watchdog-s 5` with no `--station` starts one daemon process per
robot on the ports in the file's `daemon.bind` entries, so the two commands above are only needed
for separate yaml files. Point `left.remote` / `right.remote` at the two addresses.

## Teleop and demo recording

`--teleop-device bimanual` drives both arms, with one leader per arm; `--teleop-arm-device` picks
each arm's device (`leader_arm`, the default: a GELLO served by that arm's daemon as its robot's
`teleop:` part; `robot`: any teleop part of that arm's robot; `gello`: a GELLO plugged into the
actor's machine). Default `--teleop-idle-behavior hold` holds the unselected arm and gripper
when only one operator intervenes; `policy` leaves unselected parts with the policy.
`--teleop-selected-parts` can restrict control to named arm/gripper parts. Human masks identify
actual expert dimensions, separately from hold overrides.
[teleop.md](teleop.md#two-arms-bimanual) has the details and the per-arm options yaml.

HIL actor over the two daemons, one keyboard for the toggle and the buttons:

```bash
rwrl train hil_serl --role actor --config bimanual.yaml \
    --teleop-device bimanual --teleop-arm-device leader_arm \
    --robot-teleop-action-mode joint_delta --teleop-event-device keyboard
```

Two GELLOs on the actor's machine instead, with their options taken from the station
yaml (`robots: {left, right}`, each with its `teleop:` section):

```bash
rwrl train hil_serl --role actor --config bimanual.yaml \
    --teleop-device bimanual --teleop-arm-device gello \
    --teleop-options-yaml local/station.yaml --teleop-event-device keyboard
```

Demo recording takes the same flags ([recording-demos.md](recording-demos.md)):

```bash
rwrl record --config bimanual.yaml \
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
rwrl deploy --checkpoint runs/<bc run>/checkpoints/final.pt \
    --algo real_world_bc --env-backend real_robot_bimanual \
    --real_robot_bimanual.left.remote LEFT_PC:50051 --real_robot_bimanual.right.remote RIGHT_PC:50052
```

Add each arm's `token`, `cameras` and `image_size` (equal to the training run's) as for training.

## Verified hardware-free

`tests/test_bimanual_station_smoke.py` runs the whole two-arm workflow on fakes: a station yaml
with `robots: {left, right}` (fake arm, gripper, two cameras and a scripted `joint_abs` leader part
per arm, a token on one daemon) loaded with `real_world_rl.discovery.autoconfig.load_station`, each
robot served by its own gRPC daemon (`real_world_rl.server.daemon.build_daemon`, what
`rwrl daemon` starts per robot) on loopback. Against those daemons it builds the
`real_robot_bimanual` env through the backend registry (`remote` per arm, `joint_delta`, four
`--obs.rgb` cameras) and runs, in order, what these commands run:

```bash
rwrl train hil_serl --role actor --config bimanual.yaml \
    --teleop-device bimanual --teleop-arm-device leader_arm --robot-teleop-action-mode joint_abs
rwrl record --config bimanual.yaml \
    --teleop-device bimanual --teleop-arm-device leader_arm --robot-teleop-action-mode joint_abs \
    --output-path demos/bimanual.pkl --num-successes 1
rwrl train residual_hil_serl --role actor --config bimanual.yaml \
    --teleop-device bimanual --teleop-arm-device leader_arm --robot-teleop-action-mode joint_abs \
    --residual-variant hybrid --base-provider zero
rwrl train silri --role actor --config bimanual.yaml \
    --teleop-device bimanual --teleop-arm-device leader_arm --robot-teleop-action-mode joint_abs
rwrl train hil_serl --role learner --config bimanual.yaml \
    --demo-dataset-paths demos/bimanual.pkl            # likewise residual_hil_serl
                                                       # (--allow-base-less-demos) and silri
rwrl train real_world_bc --config bimanual.yaml \
    --demo-dataset-paths demos/bimanual.pkl
rwrl deploy --checkpoint runs/<bc run>/checkpoints/final.pt \
    --algo real_world_bc --env-backend real_robot_bimanual ...
```

It checks the observation keys (`state` and one `rgb_<cam>` per camera), the 16-D action and
`num_arms=2`; that intervened rows follow the left leader while the right arm is held with its
gripper unchanged; that the demo pkl and its `.contract.json` sidecar (`num_arms`, `arm_names`,
per-arm `joint_limits`) load into the hil_serl, residual and SiLRI learners; that the agents get
a 9-way gripper head (`num_grippers=2`, from the env contract); and that the BC checkpoint deploys
on both daemons. The test is skipped when `grpc` is not installed.

## Two arms: the tutorial steps

What changes, step by step, when the [tutorial](tutorial/00-overview.md) runs on two Franka arms with
one GELLO each. Everything else (keys, onset, reward rule, metrics) is the same.

**Hardware ([01](tutorial/01-install.md)).** One Franka Panda with the Franka Hand, one GELLO with its
bus adapter and one wrist camera per arm. The second arm's FCI address must differ (the wizard also
probes `172.16.1.2`).

**Station ([02](tutorial/02-station.md)).** Answer `2` to `number of arms`; the default names are
`left right`. The wizard then asks every per-arm question twice (`-- arm 'left' (1/2) --`,
`-- arm 'right' (2/2) --`):

- the second arm needs a different robot IP, GELLO port and daemon port (it refuses a repeat, and
  tags earlier picks `(used by another arm)`); the daemon ports default to 50051 and 50052;
- the suggested cameras give each arm the next free camera as its wrist camera (`wrist_left`,
  `wrist_right`); the last arm also takes the rest (`front`, ...);
- every GELLO is calibrated in turn.

The result is one file with a `robots:` mapping, each robot with its own cameras, `teleop:` section
and `daemon:` block (abbreviated here; the real file is annotated and complete, see
[`station.bimanual.example.yaml`](../examples/stations/station.bimanual.example.yaml)):

```yaml
robots:
  left:
    type: franka
    robot_ip: 172.16.0.2
    cameras: [{name: wrist_left, serial: "..."}]
    teleop: {name: gello, backend: gello, options: {port: /dev/serial/by-id/...LEFT..., ...}}
    daemon: {bind: 0.0.0.0:50051, token: null}
  right:
    type: franka
    robot_ip: 172.16.1.2
    cameras: [{name: wrist_right, serial: "..."}, {name: front, serial: "..."}]
    teleop: {name: gello, backend: gello, options: {port: /dev/serial/by-id/...RIGHT..., ...}}
    daemon: {bind: 0.0.0.0:50052, token: null}
```

Check the safety limits on both robots; the two boxes should not overlap unless the task needs it
(there is no inter-arm collision check). The wizard's last lines name the env backend for two arms,
`real_robot_bimanual`. `--mock` also stands in for a two-arm station: its canned probe has two
Frankas, three cameras and two GELLO buses.

The env preset is `bimanual.yaml` instead of `hil.yaml`. `obs.rgb` lists every camera in arm order
(left's, then right's), each arm lists its own cameras, the daemon addresses come from the ports in
the station file, and the two arms must agree on `action_mode`, `state_fields` and the other shared
fields ([Config](#config)):

```yaml
# bimanual.yaml (two arms)
env_backend: real_robot_bimanual
env_id: my_bimanual_task
obs:
  rgb: [wrist_left, wrist_right, front]
  image_size: [128, 128]
real_robot_bimanual:
  max_episode_steps: 200
  left:
    remote: ROBOT_PC:50051
    cameras: [wrist_left]
    action_mode: joint_delta
    state_fields: [joint_pos, gripper_pos]
  right:
    remote: ROBOT_PC:50052
    cameras: [wrist_right, front]
    action_mode: joint_delta
    state_fields: [joint_pos, gripper_pos]
```

**Daemon ([03](tutorial/03-daemon.md)).** The doctor prints the device rows of both arms,
`left.*` and `right.*`, and one final `Result:` line; `--robot left` checks one.
The same `rwrl daemon --watchdog-s 5` starts both daemons: it prints
`starting 2 daemons from <path>: 'left' on 0.0.0.0:50051, 'right' on 0.0.0.0:50052`, then each daemon
prints its own preflight and `serving ...` line. The ports come from the file's `daemon.bind` entries.
`--robot left` serves just one; `--bind` and `--token` are errors for several robots (set them per
robot in the file). The dry run is `tests/test_bimanual_station_smoke.py`
([Verified hardware-free](#verified-hardware-free)).

**Record demos ([04](tutorial/04-record-demos.md)).** Replace the device flags with
`--config bimanual.yaml --teleop-device bimanual --teleop-arm-device leader_arm
--robot-teleop-action-mode joint_delta --teleop-event-device keyboard`; each arm reads its own GELLO
and a step is recorded when either operator intervenes (the other arm is held still).
`--max-episode-steps` replaces `--real_robot_bimanual.max_episode_steps`.

**Base policy ([05](tutorial/05-base-policy.md)).** BC: the same command with `--config bimanual.yaml`.

**HIL training ([07](tutorial/07-hil-training.md)).** Only `ENV_FLAGS` changes; `num_grippers=2` is
derived from the env's action contract for all three agents (you do not pass it):

```bash
ENV_FLAGS="--config bimanual.yaml --teleop-device bimanual --teleop-arm-device leader_arm \
    --robot-teleop-action-mode joint_delta --teleop-event-device keyboard --reward-from-teleop-events"
```

The BC base needs nothing extra (`checkpoint:...?algo=real_world_bc`). A two-arm `hil_serl`
(`rlpd_hybrid`) checkpoint as the base names its gripper count:
`checkpoint:<path>?algo=rlpd_hybrid&num_grippers=2`
([provider table](residual.md#base-action-providers)).

**Deploy ([08](tutorial/08-deploy.md)).** Add `--env-backend real_robot_bimanual` and give each arm's
connection and cameras (the cameras are not filled from the training run for two arms):

```bash
rwrl deploy --checkpoint runs/bc/checkpoints/final.pt \
    --algo real_world_bc --env-backend real_robot_bimanual \
    --real_robot_bimanual.left.remote ROBOT_PC:50051 \
    --real_robot_bimanual.right.remote ROBOT_PC:50052 \
    --real_robot_bimanual.left.cameras wrist_left \
    --real_robot_bimanual.right.cameras wrist_right front
```

For unattended deployment, use `--no-enable-teleop` and a station YAML without either robot's
`teleop:` block. For assisted deployment, keep those devices configured and select the bimanual
source; task runs may restore their saved setup. See [tutorial 08](tutorial/08-deploy.md).

## Limits

- No inter-arm collision check: each arm is clipped only by its own safety box, joint limits and
  device-layer limits. Keep the boxes disjoint.
- The single-arm teleop devices (`robot`, `leader_arm`, `gello`, `leader_cartesian`, `spacemouse`,
  `pico`) are rejected with `--env-backend real_robot_bimanual`; use `--teleop-device bimanual`.
