# 0. Overview and safety

This tutorial starts with a NUC and one Franka FR3, establishes a real doctor
handshake, then prepares a pick-and-place experiment with a GELLO leader.
The main workflow runs the recorder or HIL actor **in one hardware process on
the NUC**, connecting directly through franky with `real_robot.robot`. A GPU
learner can run separately. Use a robot daemon only when the actor runs on
another machine, with `real_robot.remote` instead.

The control contract is **cartesian GELLO at 50 Hz**. Use the same action
mode, frequency, cameras, state fields and gripper contract for demos, BC/ACT,
HIL and deployment. Run names such as `runs/bc-pick-and-place` and
`runs/hil-pick-and-place` identify separate experiments; reuse an identity and
its local storage only when resuming that experiment.

## The pipeline

```text
New cell: power --> Control network --> Desk control + FCI --> install
                                                               |
Already commissioned: take Desk control --> confirm FCI --------+
                                                               v
                     station.yaml --> real doctor handshake
                                                               |
                       gello_cartesian.yaml (NUC, direct) <-----+
                            |                    |
                   record GELLO demos     remote actor needed?
                            |                    |
                       BC or ACT          optional daemon + remote:
                            |                    |
                            +-----> HIL actor + learner --> deploy
```

| Page | What you do |
|---|---|
| [01 Install](01-install.md) | power, cabling, persistent networking, six Desk checks, install and doctor |
| [02 Station](02-station.md) | select per-arm hardware, calibrate GELLO, verify limits and prepare direct `hil.yaml` |
| [03 Process layout](03-daemon.md) | use one hardware process on the NUC; configure a daemon only for a remote actor |
| [04 Record demos](04-record-demos.md) | supervised GELLO or SpaceMouse recording |
| [05 Base policy](05-base-policy.md) | train BC or ACT from compatible demos |
| [06 Intervening](06-intervening.md) | operator keys, bounded following, gripper latch and recorded data |
| [07 HIL training](07-hil-training.md) | learner and actor for HIL-SERL, residual HIL-SERL or SiLRI |
| [08 Deploy](08-deploy.md) | deploy a checkpoint with its saved observation/action contract |

## Names used in the commands

| Name | Meaning |
|---|---|
| NUC / robot PC | wired to the Control port, cameras and GELLO; runs the direct recorder/actor by default |
| `ROBOT_PC` | the NUC's reachable address, needed only for remote daemon clients |
| `LEARNER_IP` | GPU learner's reachable address; may be the NUC |
| actor machine | NUC by default; another machine only for the optional remote route |

Every command states its machine, working directory and environment. The default is the
NUC, repository root (`real-world-rl/`), **project environment** active: the venv or conda
env where [01 step 4](01-install.md#4-install-the-project-environment) installed the
three local packages (`scripts/install.sh` creates `.venv` by default).
For services on the same machine use `127.0.0.1`; a remote client must use a
reachable address instead.

## Action mode

The main workflow uses `action_mode: cartesian_delta`, `control_hz: 50` and
`tcp_pose` feedback. GELLO's joint readings must pass through a cartesian mapping
(`leader_cartesian`), not be sent as joint commands to a cartesian env.
Calibrate signs and offsets and verify motion directions and scale before use.
50 Hz is the requested application update rate, not a guarantee about SDK
servo timing or measured USB sampling throughput.

Pages 04–07 use `examples/presets/gello_cartesian.yaml`: state-only, 50 Hz,
`xyz_scale_m: 0.05`, `rot_scale_rad: 0.3`, with a direct robot mapping. Recording
and hardware actor commands run under `robot_infra.supervise`. Joint-space
examples are separate modes requiring their own compatible demos and base policies. Two-arm setup starts with
[per-arm Desk and wizard checks](02-station.md#multiple-arms), then
[bimanual.md](../bimanual.md).

## Operator keys

The NUC's terminal (`stdin`) and desktop (`keyboard`) event sources share these keys:

| Key | Effect |
|---|---|
| `t` | GELLO takeover; anchor the relative mapping (not a toggle) |
| `h` | GELLO hand-back; recorder holds measured pose, HIL returns to the policy |
| `s` | label success and end the episode |
| `f` | label failure and end the episode |
| `r` | discard and rerecord the recorder episode |
| Esc | end the current episode without a success label |
| `q` | stop the run and close the devices |

SpaceMouse uses button 0 for takeover/hand-back and button 1 for the gripper;
its terminal `t`/`h` are ignored. Puck movement alone never takes over. Outcome
keys and quit still use the terminal. See [06](06-intervening.md).
Terminal stdin needs no evdev permissions; a desktop keyboard source needs the
actor's desktop session.

## Safety

Keep the workspace clear and the emergency stop accessible. Before any motion,
check the station's `home_joints`, `joint_limits`, `safety_box` and
`max_delta_per_step` against this physical arm and cell. Wizard defaults are
starting points, not validated FR3 limits or a commissioned home path. The device
layer enforces limits in both direct and daemon layouts; the env may tighten them.
Never copy a recorded home pose to another arm.

The direct process cannot issue an independent stop if it hangs as a whole.
A supervisor can kill it, after which stopping relies on the firmware's loss-of-
communication behavior. A separate live daemon can stop a silent actor, but its
watchdog does not cover a failed daemon or NUC. See the
[process comparison](03-daemon.md).

A robot configured with a teleop device requires that device to connect. For
unattended deployment use a matching station without `teleop`; for assisted
deployment keep the operator devices and their contract.

## Known limits

- Real hardware commissioning has established a left-arm FCI handshake and a
  single-arm GELLO recording session, plus separate right-Hand checks. This is
  not a full two-arm or online RL validation. Cartesian scale, following lag and
  process-wide watchdog limitations still need attention.
- Existing joint demos cannot be silently relabeled as cartesian demos. ACT training on physical recordings and
  the complete online training/deployment path remain to be validated.
- With `task.name: none`, the env returns zero reward and does not detect success;
  use explicit operator labels or a reward classifier. A positive reward alone
  does not establish an autonomous success.
- Actor uploads and learner ingestion use local SQLite storage. Keep their
  outbox, journal and checkpoint bundles on local storage; process-crash recovery
  and power-loss durability have different guarantees. See
  [Real-world experiments](../real-world-experiments.md) and
  [known issues](../known-issues.md).

Next: [01 Power-on through doctor](01-install.md), or the
[already commissioned shortcut](01-install.md#a-robot-someone-else-has-already-commissioned).
