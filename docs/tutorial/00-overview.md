# 0. Overview and safety

This tutorial takes one Franka FR3 with a Franka Hand, a NUC and a GELLO leader arm from an
empty NUC to a deployed pick-and-place policy. Every step is an `rwrl` command. This page has
no commands to run: it shows the system, the roles of the machines, the order of the pages
and the safety rules that hold on all of them.

## The system

```text
                 Ethernet (Control port, not the Arm port)
  +-----------+  172.16.0.1  <------------->  172.16.0.2  +-----------------+
  | NUC       |                                            | Franka control  |
  | (robot    |                                            | box: Desk + FCI |
  |  host)    |  USB: cameras, GELLO, SpaceMouse           +--------+--------+
  |           |                                                     |
  | rwrl      |                                              Franka arm + Hand
  | recorder, |
  | HIL actor |  network (optional): replay data out, policy weights in
  +-----+-----+  <--------------------------------------->  +-----------------+
                                                            | GPU host        |
                                                            | rwrl train      |
                                                            | --role learner  |
                                                            +-----------------+
```

The recorder and the HIL actor run **in one hardware process on the NUC** and open the arm,
Hand, cameras and GELLO directly through franky (`real_robot.robot`). The learner can run on
a GPU host or on the NUC itself. A robot daemon is only needed when the actor runs on another
machine; see [Remote actor over the robot daemon](../remote-actor.md).

The control contract is **cartesian GELLO at 50 Hz** (`action_mode: cartesian_delta`). Use the
same action mode, frequency, cameras, state fields and gripper contract for demos, BC/ACT, HIL
and deployment.

## Roles and names

| Name | Meaning |
|---|---|
| NUC / robot host | wired to the Control port, cameras and teleop devices; runs every hardware command |
| GPU host | runs the learner; may be the NUC |
| `ROBOT_PC` | the NUC's reachable address, needed only by remote daemon clients |
| `LEARNER_IP` | the learner's reachable address; `127.0.0.1` when it is the NUC |

Every command states its machine, working directory and environment. The default is the NUC,
the repository root (`real-world-rl/`), with the **project environment** active: the venv or
conda environment that [01 Install](01-install.md) creates (`.venv` by default). Commands for
services on the same machine use `127.0.0.1`; a remote client uses a reachable address.

## Reading order

| Page | What you do | Commands |
|---|---|---|
| [01 Install](01-install.md) | install the project environment, check it without hardware | `bash scripts/install.sh`, `rwrl doctor --mock` |
| [02 Set up the station](02-setup.md) | power, network and Desk; write `local/station.yaml` | `rwrl discover`, `rwrl setup`, `rwrl workspace show` |
| [03 Bring up the Franka](03-franka-bringup.md) | real handshake; read the doctor; read state | `rwrl doctor`, `rwrl status`, `rwrl q`, `rwrl tcp`, `rwrl recover` |
| [04 Home, gripper, cameras](04-first-motion.md) | first motion; check every device | `rwrl home`, `rwrl gripper`, `rwrl camera`, `rwrl stop` |
| [05 Teleop: GELLO](05-teleop-gello.md) | check the leader arm | `rwrl teleop read`, `rwrl teleop calibrate` |
| [06 Teleop: SpaceMouse](06-teleop-spacemouse.md) | check the SpaceMouse | `rwrl teleop diagnose`, `rwrl teleop profiles` |
| [07 Teleop: leader-follower](07-teleop-leader-follower.md) | joint-space leader arms | `rwrl teleop read --with-follower` |
| [08 Record demos](08-record-demos.md) | supervised GELLO or SpaceMouse recording | `rwrl supervise`, `rwrl record` |
| [09 Base policy](09-base-policy.md) | train BC or ACT from the demos | `rwrl train real_world_bc` |
| [10 HIL training](10-hil-training.md) | learner and actor for HIL-SERL, residual or SiLRI | `rwrl train` |
| [11 Deploy](11-deploy.md) | run a checkpoint with its saved contract | `rwrl deploy` |

The [operator page](../intervening.md) is the page to keep open beside the robot while
recording and training. Two arms: follow these pages with the changes in
[bimanual.md](../bimanual.md).

## Status words

`rwrl doctor`, `rwrl setup` and the daemon preflight report every check with one status word.
On a terminal the word is coloured; piped output and `NO_COLOR=1` stay plain.

| Status | Meaning | Colour | Fails the run |
|---|---|---|---|
| `PASS` | checked and fine | green | no |
| `WARN` | fine, but read the hint | yellow | no |
| `SKIPPED` | not checked; not evidence that it works | grey | no |
| `FAIL` | checked and broken | red | yes |
| `BLOCKED` | could not be checked or repaired (busy device, missing permission) | magenta | yes |
| `PENDING_ACTION` | a repair ran and needs a step from you (log in again, replug) | yellow | yes |

Details and the list of checks: [rwrl doctor](../doctor.md).

## Safety

- Keep the workspace clear and the emergency stop within reach whenever the arm is powered.
- Before the first motion, check the station's `home_joints`, `joint_limits`, `safety_box`
  and `max_delta_per_step` against this physical arm and cell
  ([02](02-setup.md#4-check-limits-and-home)). Wizard defaults are starting points, not
  validated limits. Never copy a home pose to another arm.
- Motion commands (`home go`, `gripper open/close/move`) print the target first and ask
  `proceed? [y/N]`; `--yes` answers for you. Read the target before you confirm.
- `rwrl stop` stops the arm without asking. On the direct route it needs the arm to be free:
  finish the recorder or actor first, or use the physical stop.
- The direct process has no independent watchdog if the whole process hangs: a supervisor can
  kill it, and the firmware then handles the lost connection. A daemon watchdog can stop a
  silent actor but does not cover a failed daemon or NUC.
- `BLOCKED` in the doctor means another process holds a device. The doctor never stops, kills
  or takes over that process; finish it yourself.
- A station with a teleop device needs that device to connect. For unattended deployment use a
  station without `teleop`.

<a id="operator-keys"></a>
Operator keys (takeover, hand-back, success, failure, rerecord, quit) are on the
[operator page](../intervening.md).

## Known limits

- Hardware commissioning covers a single-arm FCI handshake and a GELLO recording session; full
  two-arm and online RL runs, ACT training on physical recordings and the complete deployment
  path are not yet validated.
- Joint demos cannot be relabelled as cartesian demos.
- With `task.name: none` the env returns reward 0; success comes from operator labels or a
  reward classifier.
- Keep the actor outbox, learner journal and checkpoint bundles on local storage.

Details: [known issues](../known-issues.md) and [real-world experiments](../real-world-experiments.md).

Next: [01 Install](01-install.md).
