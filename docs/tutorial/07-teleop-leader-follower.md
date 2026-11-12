# 7. Teleop: leader-follower

**Goal:** choose a joint-space or cartesian leader mapping, check takeover behavior, and
understand how a GELLO or a second arm supplies the leader readings.

## Before you start

Unless stated otherwise, commands run on the **follower's NUC**, from the **repository root**,
with the **project environment active**, including learning dependencies for recording.

- Finish [03](03-franka-bringup.md), [04](04-first-motion.md) and
  [05 GELLO](05-teleop-gello.md). Keep the physical stop accessible and the workspace clear;
  recording resets and can move the follower. Read the [operator sequence](../intervening.md#the-sequence).
- `leader_arm` reads the follower robot's joint-space teleop part (usually a GELLO served
  by its daemon). It needs a joint-space env preset. `leader_cartesian` reads a separate
  leader part and uses forward kinematics to drive a `cartesian_delta` follower.
- The second-arm route below **needs hardware verification**. Manual guiding, FK/TCP
  agreement, gripper readings and physical stop behavior must be checked at the actual cell.
  The GELLO mapping also needs physical direction and kinematics checks; fake tests do not
  establish hardware behavior.

## 1. Check joint-space alignment

Use the calibrated GELLO station on a running follower daemon, with no recorder or actor
sending commands:

```bash
rwrl --station local/station.yaml teleop read --duration-s 30 --rate-hz 3 --with-follower
```

*You should see:* leader `q`, measured `follower_q` and per-joint `delta` (leader minus follower,
in radians) for a `joint_abs` leader, plus `gripper`, `intervening` and fired `events`.
Move only the leader while watching these readings; this command sends no follower motion.
Compare signs and joint order at several poses. If they disagree, repeat
[GELLO calibration](05-teleop-gello.md#2-calibrate-and-save-the-units-signs-and-offsets).

`--with-follower` requires the daemon route. A direct read opens only the teleop input and
cannot read the follower without a separate arm connection. If no daemon is running, complete
[03's daemon setup](03-franka-bringup.md) before this comparison, or omit `--with-follower`
for an input-only check. For connection errors, use [If it fails](#if-it-fails).

## 2. Record with joint-space `leader_arm`

Create a separate joint-space preset; keep it separate from the cartesian preset used in 05.
Copy on the NUC, repository root (no environment needed for the copy):

<!-- tutorial-test: skip -->
```bash
cp examples/presets/hil_single_arm.yaml hil_abs.yaml
```

Edit `hil_abs.yaml`: change `real_robot.action_mode` from `joint_delta` to `joint_abs`,
uncomment `joint_limits`, and replace those example bounds with the station's checked limits.
Set `remote` and any `token` to the running follower daemon; match `obs.rgb` to its cameras.
Keep `joint_pos` in `state_fields`, commission home/reset for the task, and ensure the daemon
station supplies `max_delta_per_step.joint_rad`. Absolute-joint control requires both joint
limits and that per-step cap. See [the robot YAML reference](../gello.md#3-the-robot-yaml-real_robot-franky).

From the NUC's repository root, project environment active:

```bash
rwrl supervise --log runs/record-leader-joint/supervise.jsonl -- \
    rwrl record --config hil_abs.yaml \
    --teleop-device leader_arm --robot-teleop-action-mode joint_abs \
    --teleop-onset arm --teleop-event-device stdin \
    --output-path demos/leader-joint-check.pkl --num-successes 1
```

*You should see:* reset, then takeover gated by alignment: `t` requests control, but `arm`
waits until every leader joint is within 0.1 rad of the follower. Move the leader to match the
follower, then guide it, match the trigger to the intended gripper state, and use `h` to hand
back. Label `s` while intervening to keep a successful demonstration, or `f`/`r`/Esc/`q` as on
the operator page. The ordinary recorder handles episodes automatically; unlike `teleop run`,
it does not default to label-hold and `n` sequencing. If startup or arming fails, use
[If it fails](#if-it-fails).

`--robot-teleop-action-mode joint_abs` declares the remote teleop part before connection;
the env's `real_robot.action_mode` determines the emitted action. `--teleop-device leader_arm`
selects the part; `--teleop-event-device stdin` supplies takeover and labels in this terminal
(over SSH too). Its toggle replaces the part's own intervention state. `--teleop-robot NAME`
selects a robot when an options YAML is a multi-robot station; it is unnecessary for this
single-arm setup. `--teleop-options-yaml PATH` supplies source options when needed.

Choose onset deliberately:

| `--teleop-onset` | Behavior |
|---|---|
| `arm` | Wait for max joint error ≤ `--teleop-arm-tolerance-rad` (default 0.1 rad); reset disarms. Before alignment, the step is not intervened. |
| `ramp` (default) | Take over immediately; initial `joint_abs` targets advance by at most `--teleop-ramp-step-rad` (default 0.05 rad) per step until aligned. |

To try `ramp`, replace `arm` in the command and choose a new output path. Initial catch-up is
marked `teleop_syncing`: the archive retains executed actions, while the recorder's human pkl
excludes syncing steps by default (`--keep-syncing-steps` retains them). For `joint_delta`,
initial catch-up is marked until the leader first comes within one `joint_scale_rad` step.
Use a separate compatible dataset for each action mode. See
[onset tradeoffs](../gello.md#onset-ramp-or-arm) and [leader sources](../teleop.md#leader-arms-gello).

## 3. Use GELLO as a cartesian leader

For the supervised GELLO workflow, reuse the commissioned local cartesian preset and the
[05 session command](05-teleop-gello.md#4-complete-one-supervised-session). It selects
`leader_cartesian`, not the joint-space onset policies above.

At takeover, forward kinematics gives the leader pose `(p_l0, R_l0)`. The current command pose
`(p_c, R_c)` is anchored to it. Moving the leader requests
`p* = p_c + (p_l − p_l0)` and `R* = (R_l R_l0^T) R_c`. The tracker bounds speed and lead over
the measured follower TCP; default caps are 0.05 m / 0.3 rad and speeds 0.05 m/s / 0.3 rad/s.
The env scales must be at least the caps. No joint-by-joint alignment is needed on this path.

The session copies station `workspace` and `task_frame` into the resolved env. On `real_robot`,
the source also reads these from env configuration, falling back to its local robot YAML;
explicit source options override them. Both the workspace (position and optional orientation
in the task frame) and base safety box apply. `WORKSPACE HOLD` hands back and holds measured
pose; returning inside alone does not resume following. Clear the request, press `h`, then `t`
to re-anchor. See [mapping and limits](../teleop.md#leader-arm-on-a-cartesian-robot-leader_cartesian).

*You should see:* small leader displacements produce correspondingly directed bounded
cartesian motion, and hand-back holds measured pose. If directions or FK/TCP alignment are
wrong, stop and verify the model before continuing; see [If it fails](#if-it-fails).

## 4. Use a second arm as the cartesian leader

**Needs hardware verification.** A second arm can supply `joint_abs` readings through the
`arm_leader` teleop backend. It reads the arm without issuing motion commands; connecting it
does not enable manual guiding. Arrange manual operation using the leader vendor's procedure.

On the **leader arm's host**, repository root, no environment needed for the copy:

<!-- tutorial-test: skip -->
```bash
cp examples/stations/leader.franka-as-leader.example.yaml local/leader.yaml
```

Edit the copy for the actual leader's address and backend. Its relevant structure is:

```yaml
type: franka
name: franka_leader_station
arm_backend: fake
gripper: null
cameras: []
teleop:
  name: leader
  backend: arm_leader
  options:
    arm: {type: franky, robot_ip: 172.16.0.2, realtime: ignore}
    gripper: false
    intervention: always
```

`arm.type` selects the backend; remaining `arm` keys go to its constructor. The outer fake
arm is a carrier: only the teleop part opens the physical leader, avoiding duplicate libfranka
sessions. `gripper` defaults to `false`; without a leader gripper, the follower keeps its last
commanded gripper action. `intervention: always` reports active; follower-side terminal events
will decide takeover.

On the **leader host**, repository root, with its **arm SDK and daemon environment active**:

```bash
rwrl daemon --station local/leader.yaml --bind 0.0.0.0:50052
```

*You should see:* a daemon listening on the chosen address. This serves the leader's teleop
part; it does not serve or control the follower. If the leader connection fails, run doctor
with `--station local/leader.yaml` on this host and follow [If it fails](#if-it-fails).

On the **follower NUC**, save `local/leader-cartesian.yaml` with this flat source-options
mapping, replacing `leader-host` with the reachable leader host (and adding `remote_token`
if the daemon uses a token):

```yaml
remote_address: leader-host:50052
mapping: anchored
kinematics: panda
```

`from_env` accepts exactly one leader route: `remote_address`, `robot_yaml` (a local carrier
YAML), or an in-process `leader` part/resolver. The remote route connects at source start.
`kinematics` defaults to `panda`, the only built-in model; the leader must report `joint_abs`
with exactly its seven joints. Other arm models require a registered matching kinematics
implementation. Model choice does not follow automatically from `arm.type`.

On the **follower NUC**, **repository root**, **learning environment active**, use the
commissioned follower cartesian preset and override its GELLO source options. This ordinary
recorder reads the follower settings from the preset, so include the reviewed `task_frame`
and `workspace` under `real_robot` there (or in its local robot YAML); it does not automatically
merge `local/station.yaml` as `teleop run` does. Finish any daemon directly owning the follower
if the preset opens that arm locally; the leader daemon remains running:


```bash
rwrl supervise --log runs/record-second-leader/supervise.jsonl -- \
    rwrl record --config local/gello_cartesian.yaml \
    --teleop-device leader_cartesian --teleop-options-yaml local/leader-cartesian.yaml \
    --teleop-event-device stdin \
    --output-path demos/second-leader-check.pkl --num-successes 1
```

*You should see:* follower reset, a fixed hold pose, then anchored motion on `t` and hand-back
on `h`. Check manual guiding, translation/rotation direction, leader FK versus physical TCP,
workspace holds and any gripper channel before keeping data. These expected behaviors are a
commissioning target, **not a hardware-verified result**. See
[a second arm as the leader](../teleop.md#a-second-arm-as-the-leader) for the full contract.

## If it fails

First run `rwrl doctor --station local/station.yaml` on the follower NUC (or the leader host's
own station command above) and find the red row in
[doctor's red lights](../doctor.md#red-lights-what-to-do).

- GELLO serial/input access: [serial, input groups and USB latency](../doctor.md#serial-input-groups-and-usb-latency).
- Another process owns an arm or serial port: [busy devices (BLOCKED)](../doctor.md#busy-devices-blocked).
- Daemon unreachable or FCI failure: [network and FCI](../doctor.md#network-and-fci).
- Alignment never arms: check joint order/signs and `delta`; correct calibration before
  changing tolerances. A joint-count/FK mismatch needs the correct kinematics model.
- `WORKSPACE HOLD`: clear the request and take over again to re-anchor; review the existing
  limits and physical frame rather than widening them to clear a hold.
- Contract mismatch when loading data: use a dataset recorded with the same action mode and
  tracker settings; see [08 compatibility](08-record-demos.md#files-and-compatibility).

Next: [08 Record demos](08-record-demos.md).
