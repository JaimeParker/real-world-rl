# 6. Intervening: the operator page

This page is for the person at the keyboard and the GELLO, in the recorder ([04](04-record-demos.md))
and in HIL training ([07](07-hil-training.md)). Keep it open next to the robot.

The setup: the GELLO is read through the robot daemon (`--teleop-device leader_arm`), the keys come
from the actor machine's keyboard (`--teleop-event-device keyboard`, read with a `pynput` listener,
which needs that machine's desktop session).

| Key | Effect |
|---|---|
| **Space** | toggle the intervention: take control, press again to give it back |
| **s** | success. Recorder: the episode is a success and ends. HIL with `--reward-from-teleop-events`: reward 1, the step is stored as terminal and the episode ends |
| **f** | failure. Recorder: the episode is discarded. HIL with `--reward-from-teleop-events`: the episode ends as truncated (not terminal) |
| **r** | rerecord. Recorder: the episode is discarded and the next one starts at once. HIL: no effect |
| **Esc** | end the episode (truncated, never terminal; `info["human_episode_end"]`) |
| trigger | the gripper: squeezed past half travel closes it (`-1`), released opens it (`+1`) |

Keys pressed while the robot homes during a reset are dropped.

## The sequence

1. **Reset.** The env resets: the arm homes, the gripper opens. Wait until it is still.
2. **Leader near follower.** Bring the GELLO into the follower's pose, joint by joint.
3. **Space.** You take control. With `--teleop-onset arm` control starts only once every leader
   joint is within `--teleop-arm-tolerance-rad` (0.1 rad) of the follower; until then the policy
   keeps acting and the actor logs the per-joint error once per second. With `ramp` it starts at once.
4. **Drive.** Move the GELLO; the follower follows at up to `joint_scale_rad` (0.05 rad) per joint
   per step in `joint_delta`, and the daemon clips every target to `max_delta_per_step`.
5. **Trigger.** Squeeze to grasp, release to drop into the bin.
6. **Space again** to give control back (HIL), once the robot is back on track.
7. **s / f / r** when the outcome is known. Press `s` **while you still hold control** when you are
   recording, or whenever the step that achieved the task should be the stored terminal one.
8. **Esc** to end an episode that has no outcome (the step cap ends it too).

During `--role eval` ([07](07-hil-training.md#evaluate)), keep Space released to measure an
unassisted episode. Takeover remains available when needed; a successful assisted episode counts
in `task_success_rate`, while the primary `success_rate` is `autonomous_success_rate`. Press `s`
to label a success when using key rewards. Label-only `s`/`f` presses do not count as action
intervention. A failure label vetoes a success label, even when both arrive on the same step.

## Onset: `arm` or `ramp`

`--teleop-onset` decides what happens at the moment you press Space:

- `arm` (used for recording in [04](04-record-demos.md)): the toggle counts only once the leader is
  within 0.1 rad of the follower; every env reset disarms it, and once armed it stays armed until you
  release the toggle. The takeover never moves the robot by itself, at the cost of matching the leader
  to the follower by hand before every takeover.
- `ramp` (the default; used in [07](07-hil-training.md)): control starts at once. In `joint_delta` the
  follower moves towards the leader at up to `joint_scale_rad` per step until they are aligned. In
  `joint_abs` the first targets are rate-limited to `--teleop-ramp-step-rad` (0.05) per step. In both
  modes catch-up is tagged `teleop_syncing`: `joint_delta` marks the initial steps until the
  leader first comes within `joint_scale_rad` of the follower; `joint_abs` marks each rate-limited
  ramp step. Executed actions and actual human masks remain in the archive, while the training
  projection excludes syncing supervision. After initial alignment, later saturated `joint_delta`
  teaching steps are supervised normally. Recorder pkl exports can keep syncing rows as ordinary
  supervision with `--keep-syncing-steps`; archive-derived training views still exclude them.

Prefer `arm` when every demonstration should start from a still robot, `ramp` for quick takeovers
while the policy is moving. Either way, hold the leader near the follower before you press Space.

## What the system records

The intervention wrapper writes these `info` keys on every step:

| `info` key | Value |
| --- | --- |
| `intervene_action` | the executed `(1, A)` action; **only while intervening** |
| `policy_action` | the action the policy proposed, as received |
| `intervened` | the operator's action was executed |
| `intervention_onset` | intervened now and not on the previous step of this episode |
| `human_episode_end` | the operator's episode-end key (Esc) |
| `teleop_events` | the `success`/`failure`/`rerecord` keys that fired (leader arms add `teleop_syncing` on catch-up steps); **only when some did** |
| `human_action_mask` | action dimensions actually controlled by the human |
| `override_mask` | overridden dimensions, including idle-hold supplements |
| `task_success`, `task_success_source` | explicit outcome and its source when available; a positive reward or terminal flag alone is insufficient |

An episode end makes `step()` return `truncated=True` (never `terminated`). The stored `action` of an
intervened step is always the executed one, the human's. What each method does with it:

| Method | Executed action and supervision | Replay or export |
|---|---|---|
| recorder | stores actual executed actions; human masks select supervised dimensions | complete SQLite archive; successful human pkl view, excluding syncing unless `--keep-syncing-steps` |
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
