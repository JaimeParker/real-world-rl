# 0. Overview and safety

This tutorial takes one Franka arm from an empty robot PC to a deployed policy that picks an object
and places it in a bin. It uses one GELLO leader arm, read through the robot daemon, and the
keyboard of the actor machine for the operator's keys. Every page says what to run, what you should
see, and what to do if you do not.

The worked example throughout is **pick and place**: one object, one bin. Runs are named
`*-pick-and-place` (`runs/bc-pick-and-place`, `runs/hil-pick-and-place`, ...).

## The pipeline

```text
01 install --> 02 station.yaml + hil.yaml --> 03 daemon + doctor
    --> 04 record demos (GELLO + keyboard)       demos/pick-and-place.pkl
    --> 05 base policy: BC or ACT                runs/bc-pick-and-place, runs/act-pick-and-place
    --> 06 how to intervene (read before 04 and 07)
    --> 07 HIL training: hil_serl | residual_hil_serl | silri
    --> 08 deploy a checkpoint
```

| Page | What you do |
|---|---|
| [01 Install](01-install.md) | hardware checklist, network, permissions, install per machine |
| [02 Station](02-station.md) | run the setup wizard (GELLO calibration included), check the safety limits, create `hil.yaml` |
| [03 Daemon](03-daemon.md) | discovery, doctor, serve the robot; a dry run without hardware |
| [04 Record demos](04-record-demos.md) | record successful pick-and-place demos into a pkl |
| [05 Base policy](05-base-policy.md) | train BC or ACT on the demos |
| [06 Intervening](06-intervening.md) | the operator page: keys, takeover, what gets recorded, when to intervene |
| [07 HIL training](07-hil-training.md) | learner and actor for `hil_serl`, `residual_hil_serl` (BC or ACT base) and `silri`; what to watch |
| [08 Deploy](08-deploy.md) | run a checkpoint with no learner and no operator |

## Names used in the commands

| Name | Meaning |
|---|---|
| `ROBOT_PC` | the machine wired to the arm, the cameras and the GELLO; runs the robot daemon |
| `LEARNER_IP` | the GPU machine that runs the learner and trains BC and ACT; may be the robot PC |
| actor machine | the machine with the keyboard that runs the actor and the recorder (often the robot PC) |

On a single machine use `127.0.0.1` wherever a command says `ROBOT_PC` or `LEARNER_IP`. Run every
command from the repository root (`real-world-rl/`).

## Action mode

The main line uses `action_mode: joint_delta` for every stage: the recorded demos, the base policy,
every HIL run and the deployment must share one action mode, because the learners reject a demo pkl
whose contract sidecar differs from their env and a residual base must act in the env's action
space. `joint_abs` is also supported; the pages where it differs carry a **joint_abs** callout
(recording, the base policy).

Two arms are not part of this tutorial: [bimanual.md](../bimanual.md#two-arms-the-tutorial-steps)
lists what changes at each step.

## Safety

Treat the first runs as commissioning:

- Keep a hand on the emergency stop, start with a clear workspace, and read each step's output before
  moving on.
- Check the `safety_box`, `joint_limits` and `max_delta_per_step` of `station.yaml` before the first
  motion ([02](02-station.md#check-the-safety-limits)); commission with low `max_delta_per_step`
  values. The daemon clips every servo target to them; the env can only be tighter.
- A configured teleop must be present for the robot to connect, so deployment uses a station yaml
  without the `teleop:` section ([08](08-deploy.md)).

## Known limits

Known bugs with their workarounds and planned fixes: [Known issues and deferred work](../known-issues.md).

- **Everything hardware-side was verified against fakes only.** In the robot_infra README's words:
  "Nothing below has been exercised on real hardware; the fakes cover the logic only"
  ([Known limitations](../../robot_infra/README.md#known-limitations-and-needs-hardware-verification)),
  and [gello.md](../gello.md#8-not-verified-yet): "no run on a physical GELLO or robot has been made
  yet". The GELLO hardware numbers (baud rate, read time, latency), the per-unit offsets and signs,
  and torque-on behaviour are unverified. Expect to debug on the first day.
- **ACT training from recorded pkls** has not been exercised on real demos yet.
- **`leader_cartesian`**: the Panda forward-kinematics DH table that turns a GELLO into a cartesian
  twist has not been compared with a real robot
  ([teleop.md](../teleop.md#verify-the-dh-table-on-the-robot)). This tutorial does not use it.
- **`franka_real`** (the SERL HTTP bridge) is single-arm and cartesian only; this tutorial uses
  `real_robot` (robot_infra).
- **Reward**: the `real_robot` env returns reward 0 and never terminates on its own. The reward comes
  from the operator's `s` key (`--reward-from-teleop-events`, [06](06-intervening.md#where-the-reward-comes-from))
  or from a reward classifier ([07](07-hil-training.md#optional-a-reward-classifier)).
- **Configured teleop is mandatory**: a robot yaml with a `teleop:` section fails to connect when the
  device is absent.
