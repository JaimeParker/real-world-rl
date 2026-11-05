# 6. Intervening: the operator page

Use this page beside the robot for [recording](04-record-demos.md) and
[HIL training](07-hil-training.md). The hardware process runs directly on the NUC,
under `robot_infra.supervise`, with the 50 Hz state-only cartesian preset.
GELLO uses `--teleop-device leader_cartesian --teleop-event-device stdin`.
SpaceMouse uses `--teleop-device spacemouse --teleop-event-device stdin`.

`stdin` reads the terminal running the supervised command, including over SSH;
`keyboard` uses the same keys through `pynput` and needs a desktop session.

| Input | Effect |
|---|---|
| `t` | GELLO: take over and anchor the relative mapping; repeated presses do not toggle it off |
| `h` | GELLO: hand back; recorder holds the measured pose at that moment, HIL returns control to the policy |
| `s` | success and episode end; recorder defaults to reward 1 and a terminal step; HIL requires `--reward-from-teleop-events` for key rewards |
| `f` | failure; discard the recorder attempt, or truncate the HIL episode |
| `r` | discard and rerecord in the recorder; no rerecord action in HIL |
| Esc | end the episode without a success label (`human_episode_end`) |
| `q` | end the current episode and stop the recorder/actor after processing it |
| GELLO trigger | close (`-1`) when squeezed, open (`+1`) when released, after the takeover latch is matched |
| SpaceMouse button 0 | take over; next press hands back; only this button controls takeover on this route |
| SpaceMouse button 1 | while driving, toggle gripper open/closed; ignored while handed back |
| SpaceMouse puck | velocity input while driving; movement alone does not count as takeover |

With SpaceMouse, terminal `t`/`h` are ignored; `s`/`f`/`r`/Esc/`q` still work.
Keys pressed during reset/home are dropped. Keep the physical stop accessible.

After a reflex stop, inspect the cell and release the activation device or
clear the fault in Desk as needed. On the **NUC**, **repository root**, with
the **project environment active**, use `rwrl recover` to request error recovery;
`rwrl stop` requests an arm stop without confirmation. Both follow the station's
connection selection. On this direct workflow, finish the recorder/actor and
release its connection first; a second FCI connection cannot stop a directly
owned arm. With a declared live daemon, the commands use that daemon. Keep the
physical stop accessible during a run. See [rwrl](../cli.md#connection-ownership).

## The sequence

### GELLO

1. Wait for reset and any home/gripper motion to finish.
2. Hold the leader comfortably and press **`t`**. The current leader pose anchors
   the relative mapping to the follower's command; no joint-by-joint matching is
   required on this cartesian path.
3. Move the leader to guide the tool. Squeeze/release the trigger for the gripper.
   If its position disagrees with the last gripper command at takeover, match
   that state first; the latch prevents an immediate unintended opening.
4. Press **`h`** to hand back. In the recorder the measured pose is held; in HIL
   the policy resumes. Press `t` again for a fresh anchor on the next takeover.
5. Use **`s`/`f`/`r`** for the outcome. In the recorder press `s` while intervening
   so the successful terminal step enters the human-only pkl export. After `h`,
   use `t` to re-anchor before recording that label.
6. Use **Esc** for an unlabeled episode end or **`q`** to end the run.

### SpaceMouse

1. Wait for reset. Release the puck and buttons before starting a new takeover.
2. Press **button 0** to take over. Nudging the puck alone never takes control.
3. Deflect the puck for tool velocity; release it to stop adding motion to the
   command. The arm can still settle by at most the lead cap. Use **button 1** to
   toggle the gripper; takeover itself keeps its last commanded state.
4. Press **button 0** again to hand back; a later press begins another takeover.
   Keyboard `t` and `h` do not change this device's takeover state.
5. Use keyboard **`s`/`f`/`r`/Esc/`q`** for outcomes and quit. In the recorder label
   success while button-0 takeover remains active.

During [evaluation](07-hil-training.md#evaluate), leave action control with the
policy to measure autonomous success. An assisted success counts in
`task_success_rate`, while the primary `success_rate` is
`autonomous_success_rate`. Label-only `s`/`f` events do not count as action
intervention; a failure label vetoes a simultaneous success label.

## How the robot follows you

The cartesian source keeps a fixed hold pose after reset and every hand-back.
The recorder's hold policy shares that tracker, so holding sends commands back
to that fixed pose rather than a zero twist that would drift with measured pose.
HIL hands control back to the policy; it is not a command to freeze policy motion.

At GELLO takeover, the source anchors the leader's current pose to its existing
command pose. Leader displacement becomes a requested translation and rotation.
The command advances at up to **0.05 m/s** and **0.3 rad/s** and leads measured TCP
by at most **0.05 m** and **0.3 rad**. Excess lead is discarded, never queued.
When the leader stops, the follower can still travel at most one lead cap toward
the retained command. Verify physical direction and scale during commissioning.

SpaceMouse uses the same tracker. Full puck deflection requests the configured
velocity; letting go stops further velocity requests. It does not remove the
bounded distance between the arm and its existing command.

A gap of more than **0.3 s** between intervened polls holds the follower and logs
the delay; the next poll re-anchors the GELLO, avoiding a jump across the pause.
This tracker behavior is separate from an env StepWatchdog trip, which stops the
arm and requires reset, and from the external supervisor's heartbeat escalation.

A requested position outside the safety box logs **`WORKSPACE HOLD: <axes>`**,
hands back and holds the measured pose. Clear the leader request and use `h`
then `t` to resume GELLO with a new anchor; on SpaceMouse press button 0 to take
over again. Do not widen the workspace simply to clear a HOLD.

The four `--teleop-lead-cap-m`, `--teleop-lead-cap-rad`,
`--teleop-target-speed-m-s`, `--teleop-target-speed-rad-s` values enter
`experiment.teleop_control`. Keep them and the selected teleop device consistent
through this tutorial's recording, training and deployment. Current HEAD
compares the shared tracker block rather than physical device identity; see
[04's compatibility note](04-record-demos.md#files-and-compatibility).

<details>
<summary>Other modes: Onset arm or ramp (joint-space leader_arm only)</summary>

This section concerns the joint-space leader path, not `leader_cartesian` or
SpaceMouse. `--teleop-onset` decides what happens when `t` requests takeover:

- `arm` waits until every leader joint is within `--teleop-arm-tolerance-rad`
  (0.1 rad) of the follower; reset disarms it. Keep the leader aligned first.
- Default `ramp` begins immediately. `joint_delta` moves toward the leader by at
  most `joint_scale_rad` per step; `joint_abs` initial targets are limited by
  `--teleop-ramp-step-rad` (0.05) per step.

Only these joint modes emit `teleop_syncing`: initial catch-up in `joint_delta`
and rate-limited initial targets in `joint_abs`. Executed actions and masks stay
in the archive; syncing supervision is excluded from archive-derived training
views. `--keep-syncing-steps` can retain these rows in recorder pkl exports.
Joint-space trigger takeover still needs the operator to match gripper state;
the cartesian gripper latch does not apply to that route.

</details>

## What the system records

The intervention wrapper writes these `info` keys on every step:

| `info` key | Value |
| --- | --- |
| `intervene_action` | the executed `(1, A)` action; **only while intervening** |
| `policy_action` | the action the policy proposed, as received |
| `intervened` | the operator's action was executed |
| `intervention_onset` | intervened now and not on the previous step of this episode |
| `human_episode_end` | the operator's episode-end key (Esc) |
| `teleop_events` | the `success`/`failure`/`rerecord`/`operator_quit` events that fired; `teleop_syncing` applies only to joint-space leaders; **only when some did** |
| `human_action_mask` | action dimensions actually controlled by the human |
| `override_mask` | overridden dimensions, including idle-hold supplements |
| `task_success`, `task_success_source` | explicit outcome and its source when available; a positive reward or terminal flag alone is insufficient |

Esc makes `step()` return `truncated=True` (never `terminated`); `q` also requests
shutdown after the episode. A success key can instead make its step terminal. The stored `action` of an
intervened step is always the executed one, the human's. What each method does with it:

| Method | Executed action and supervision | Replay or export |
|---|---|---|
| recorder | stores actual executed actions; human masks select supervised dimensions | complete SQLite archive; successful human pkl view, excluding joint-space syncing by default |
| `hil_serl` | learns from all executed actions; human, nonsyncing rows also populate demo replay | online replay plus growing demo replay, mixed at `--demo-data-ratio` |
| `residual_hil_serl` | human action is the total executed action; retains `policy_action`, `base_actions`, `next_base_actions` | residual BC intersects the actual human mask with nonzero residual scales |
| `silri` | retains proposal, actual masks and the predecessor's `intervention_onset` | behavior fitting and pretraining supervise human dimensions; onset rows are not relabeled as expert |

The gripper heads (`hil_serl`'s DQN, `--residual-variant hybrid`, `silri`'s discrete head) label each
stored gripper action with the nearest of `-1`/`0`/`+1`; with `gripper_binarize_threshold: 0.5` (the
wizard's default) the trigger gives exactly `+1` or `-1`. Details:
[residual.md](../residual.md#human-intervention), [silri.md](../silri.md#data-flow),
[teleop.md](../teleop.md#interventionwrapper).

## Where the reward comes from

With the tutorial's default `task.name: none`, `real_robot` returns reward 0 and has no task
success detector. Configure key rewards or a classifier. Versioned task presets can supply
explicit outcomes and reset/termination rules; a TCP-position proxy remains a separate metric.

- **Recorder**: the reward comes from your keys by default (`--no-reward-from-teleop-events` turns it
  off). `s` stores the step as a terminal success with reward 1.
- **HIL training** (`hil_serl`, `residual_hil_serl`, `silri`): without `--classifier-checkpoint`,
  pass `--reward-from-teleop-events` to the actor (the commands of [07](07-hil-training.md) do). The
  actor and learner log a warning when neither is set.

The rule for `s`: press it **while you still hold control** in the recorder, because only intervened
steps are recorded, and an `s` after the release leaves the demo with reward 0 everywhere. In HIL
training `s` counts on any step, yours or the policy's: press it on the step the object lands in the
bin. With a classifier the larger of the two rewards wins (the keys never lower the classifier's).

## When to intervene

Take over when the policy is about to fail the task, collide, or leave the safe region; release when
it is back on track. Do not intervene while it is doing fine: the actor stores every step, and only
your intervened steps enter the demo buffer. Expect to intervene a lot at first and less as the
policy improves.

Two kinds of takeover, and what they teach:

- **Recovery**: the policy has gone wrong (missed the object, drifting out of the workspace). Take
  over, bring the robot back to a state the task can continue from, and release before you finish
  the task yourself. These rows show the way out of states the policy reached on its own, which the
  demos never visited.
- **Correction**: the policy is roughly right but misses a detail (the grasp is off-centre, the
  object is released too early). Take over only for that moment, do it right, release. Short
  corrections at the decisive step give the most information per intervened row.

Keep each takeover purposeful and consistent: in `silri` your actions train a model of your behaviour
whose spread loosens the constraint where you are inconsistent; in `residual_hil_serl` your action is
the total action, and the BC term clips the residual it implies to the box `base +/- scale`, so a
correction far from the base teaches the residual only "as far as the box allows" (the critic still
learns the value of the action you executed). Esc ends an episode that cannot be recovered.


## Take over selected parts

`--teleop-selected-parts arm.arm` selects the tutorial's arm motion; `arm.gripper` selects its
gripper. On two arms use `left.arm`, `right.arm`, `left.gripper` or `right.gripper`, matching
`arm_names`. An empty list uses all parts available from the source. Default
`--teleop-idle-behavior hold` holds unselected parts during a takeover; `policy` leaves them
with the policy. Only `human_action_mask` dimensions supply expert supervision; `override_mask`
also covers held parts. Set the same part selection and idle behavior when recording and
training, because they enter the experiment contract. See [partial takeover](../real-world-experiments.md#part-takeover-and-imitation).
