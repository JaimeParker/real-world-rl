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

During `--role eval` ([07](07-hil-training.md#evaluate)) do not press Space: the episodes measure the
policy alone. Do press `s` when the task succeeded if the run takes its reward from the keys.

## Onset: `arm` or `ramp`

`--teleop-onset` decides what happens at the moment you press Space:

- `arm` (used for recording in [04](04-record-demos.md)): the toggle counts only once the leader is
  within 0.1 rad of the follower; every env reset disarms it, and once armed it stays armed until you
  release the toggle. The takeover never moves the robot by itself, at the cost of matching the leader
  to the follower by hand before every takeover.
- `ramp` (the default; used in [07](07-hil-training.md)): control starts at once. In `joint_delta` the
  follower moves towards the leader at up to `joint_scale_rad` per step until they are aligned. In
  `joint_abs` the first targets are rate-limited to `--teleop-ramp-step-rad` (0.05) per step. In both
  modes the catch-up steps, from the toggle until the leader and follower first agree, are tagged
  `teleop_syncing`: they are executed and stored, but they do not count as human supervision
  (the learners treat them as policy steps). Once aligned, the rest of the takeover is supervised
  normally even if you move faster than the follower can track.

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
| `teleop_events` | the `success`/`failure`/`rerecord` keys that fired (leader arms add `teleop_syncing` on rate-limited onset steps); **only when some did** |

An episode end makes `step()` return `truncated=True` (never `terminated`). The stored `action` of an
intervened step is always the executed one, the human's. What each method does with it:

| Method | Stored action | What `intervened` does | Buffers |
|---|---|---|---|
| recorder | the human's | only intervened steps are recorded (ramp-syncing steps left out unless `--keep-syncing-steps`) | the demo pkl |
| `hil_serl` | the executed action (the human's while intervening) | routing only: the row is also copied into the demo buffer; the flag itself is not stored | online buffer: every row; demo buffer: intervened rows and the `--demo-dataset-paths` demos, mixed at `--demo-data-ratio` |
| `residual_hil_serl` | the human's **total** action, not an offset from the base; plus `policy_action`, `base_actions`, `next_base_actions` | the row is copied into the demo buffer; with `--residual-bc-weight` > 0 the unit residual is pulled towards `clip((a_human - a_base) / scale, -1, 1)` on intervened rows | as `hil_serl`, both buffers store `intervened` |
| `silri` | the human's; plus `policy_action`, `intervened`, `intervention_onset` | intervened rows train the human-behaviour model and the gripper head; the policy row right before a takeover is tagged `intervention_onset` | online buffer: every row; demo buffer: intervened rows (onset rows too with `--onset-to-demo-buffer`, still not intervened) |

The gripper heads (`hil_serl`'s DQN, `--residual-variant hybrid`, `silri`'s discrete head) label each
stored gripper action with the nearest of `-1`/`0`/`+1`; with `gripper_binarize_threshold: 0.5` (the
wizard's default) the trigger gives exactly `+1` or `-1`. Details:
[residual.md](../residual.md#human-intervention), [silri.md](../silri.md#data-flow),
[teleop.md](../teleop.md#interventionwrapper).

## Where the reward comes from

The `real_robot` env returns reward 0 and never terminates by itself. Without a reward source every
transition carries reward 0, and nothing tells the agent what success is.

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
