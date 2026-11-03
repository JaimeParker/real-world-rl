# Known issues and deferred work

What a first real run on one Franka arm with a gripper (GELLO through the robot daemon,
`--teleop-device leader_arm`, keys from `--teleop-event-device keyboard`) can run into, where it is
in the code, and what to do about it today. Every `path:line` was checked against the code when the
entry was written; line numbers drift, so search for the named function if one no longer matches.
Numbers marked "static analysis, not measured" come from reading the code, not from a run.

Each entry gives the symptom, the impact, where it is, the workaround for today, the planned fix with
the test that will prove it, and its status. Fixed entries move to [Closed](#closed) with the commit
and the test; nothing is deleted.

## A. To fix after the first real run

In dispatch order.

### S4. The learner loses or double-loads buffer rows across a stop

- **Symptom**: after a learner restart with the same `--exp-name`, `learner/buffer_size` is smaller
  than `learner/received` was before the stop; or, with `--load-checkpoint ... --load-replay-buffer`,
  it is about twice as large.
- **Impact**: data loss (up to `--buffer-period` - 1 rows, 999 by default, about 100 s at 10 Hz) and
  duplicated rows that skew sampling.
- **Where**: `HilSerlLearnerLoop._on_transition` writes the crash-recovery snapshot only when a full
  `--buffer-period` of rows has arrived (`rl_garden_real_world/loops/hil_serl.py:376`, `_snapshot` at
  `:379`); `LearnerLoop.run` saves `final.pt` on the way out but never flushes the pending rows
  (`rl_garden_real_world/loops/learner.py:254`). Since the tutorial stops the actor first, no further
  row arrives to trigger a snapshot, so the tail is always lost. A restart loads the checkpoint's
  online buffer when `--load-replay-buffer` is set (`agent.load` in the builder,
  `rl-garden/rl_garden/training/online/entrypoints/rlpd_hybrid.py:121`, `rlpd.py:108`,
  `rl_garden_real_world/training/entrypoints/silri.py:168`) and then reloads every snapshot in the
  same directory (`rl_garden_real_world/loops/hil_serl.py:270`, `_reload_snapshots` at `:292`). The
  periodic checkpoint runs after the buffer lock is released (`rl_garden_real_world/loops/learner.py:150`,
  `_maybe_save_periodic_checkpoint` at `:153`; SiLRI `rl_garden_real_world/loops/silri.py:200`), so with
  `--save-replay-buffer` the network thread can add rows while the buffer is being written (static
  analysis, not measured).
- **Workaround today**: use a small `--buffer-period` (for example 100) for the first runs; stop the
  actor right after a new `checkpoints/buffer/transitions_<n>.pkl` appears; never combine
  `--load-replay-buffer` with a run directory that holds snapshots.
- **Planned fix**: flush the pending rows when the loop stops; skip the snapshot reload when the
  checkpoint carried the buffer; take the buffer lock around the periodic save. Tests: a stop with
  pending rows writes them and a restart reloads exactly `received` rows; a checkpoint with a buffer
  plus a snapshot directory loads each row once.
- **Status**: open (S4).

### S6. Actor shutdown: the arm is released by the watchdog, queued rows are lost

- **Symptom**: after Ctrl-C or a crash in the actor, the arm stops only when the daemon's watchdog
  fires; the last rows (and SiLRI's last held row) never reach the learner.
- **Impact**: safety (the arm keeps its last servo target for `--watchdog-s`, 0.5 s by default) and
  data loss at every stop.
- **Where**: the actor entrypoints never call `env.close()`
  (`rl_garden_real_world/training/entrypoints/hil_serl.py:36`, `residual_hil_serl.py:95`,
  `silri.py:187`; the eval loops, `hil_serl.py:185`, `residual_hil_serl.py:183`, `silri.py:263`,
  neither). `ActorLoop.run` only stops the sync client (`rl_garden_real_world/loops/actor.py:171`).
  `env.close()` would send `robot.disconnect` to the daemon
  (`rl_garden_real_world/envs/wrappers/intervention.py:182`,
  `rl_garden_real_world/envs/real_robot/env.py:418`, `robot_infra/robot_infra/server/remote.py:440`).
  The watchdog default is `robot_infra/robot_infra/server/daemon.py:51`. `ActorSyncClient.stop` sets
  the stop event before the push worker has drained the queue, and the worker checks that event before
  each item (`rl_garden_real_world/loops/sync.py:285`, `:323`), so the rows SiLRI flushes in
  `_on_run_end` (`rl_garden_real_world/loops/silri.py:76`) can be dropped.
- **Workaround today**: end the current episode with Esc and let the arm reach a safe pose before
  Ctrl-C; keep a hand on the emergency stop; do not raise the daemon's `--watchdog-s` above
  what the resets need.
- **Planned fix**: close the env in a `finally` in every actor and eval path; drain the push queue
  (with a deadline) before setting the stop event. Tests: an interrupted actor run closes a fake env;
  rows queued before `stop()` all reach a fake learner.
- **Status**: open (S6).

### S8. Small gaps: truncated rows untested, `joint_abs` start-up motion

- **Symptom**: none yet for the first; for the second, an untrained `joint_abs` policy moves the arm
  towards the middle of `joint_limits` as soon as the actor starts.
- **Impact**: a regression in the done flag would be silent (data corruption); unexpected motion
  (safety).
- **Where**: `ActorLoop._build_transition` stores `done = terminated` (`rl_garden_real_world/loops/actor.py:113`),
  but the only direct test passes `terminated=True, truncated=False`
  (`tests/test_real_world_actor_loop_hooks.py:86`); `tests/test_single_arm_station_smoke.py:461` runs a
  truncated episode without checking its `done`. `joint_abs` maps an action near 0 onto the midpoint
  of `joint_limits` (`rl_garden_real_world/envs/real_robot/env.py:337`), bounded per step only by
  `max_delta_per_step` (`:167`). `hil_serl` and `silri` start from an untrained policy;
  `residual_hil_serl` adds to its base policy.
- **Workaround today**: use `joint_delta` (the tutorial's main line); with `joint_abs`, commission with
  a low `max_delta_per_step` and take over with Space at the start.
- **Planned fix**: a test that `truncated=True, terminated=False` stores `done` False; a warm-up or
  hold for `joint_abs` without a base policy.
- **Status**: open (S8).

### S2. Takeover transients enter the demo buffer

- **Symptom**: right after Space in `joint_delta`, the follower jumps towards the leader at
  `joint_scale_rad` per step, and those saturated rows train the policy as if they were demonstrations.
- **Impact**: data corruption (the demo buffer holds catch-up motion, not the task).
- **Where**: only the `joint_abs` ramp tags rows `teleop_syncing`
  (`rl_garden_real_world/teleop/leader_arm.py:212`, set in `events` at `:167`); `joint_delta` returns
  untagged (`:208`). The actor does not forward the tag (`_extra_transition_fields`,
  `rl_garden_real_world/loops/hil_serl.py:145`) and the learners route on `intervened` alone (`:360`).
  `tests/test_leader_arm_hil_loops.py:216` pins that ramp rows reach the demo buffer. Only the recorder
  drops them (`rl_garden_real_world/demos/record.py:381`).
- **Workaround today**: `--teleop-onset arm` in HIL training too: control starts only once the leader
  is within `--teleop-arm-tolerance-rad` of the follower
  (`rl_garden_real_world/teleop/leader_arm.py:200`), at the cost of
  matching the GELLO by hand before each takeover.
- **Planned fix**: tag saturated onset rows in `joint_delta`, forward the tag, and add
  `--demo-skip-syncing` to the learners. Test: onset rows of both modes stay out of the demo buffer
  with the flag.
- **Status**: open (S2).

### S3. Taking over can open the gripper

- **Symptom**: the policy holds the object, you press Space with the trigger released, and the
  gripper opens at once.
- **Impact**: safety (a dropped object) and a bad intervened row.
- **Where**: the trigger maps straight to `2w - 1` (`rl_garden_real_world/teleop/part.py:273`); the
  onset policies act on the joints only (`rl_garden_real_world/teleop/leader_arm.py:161`, gripper added
  at `:164`).
- **Workaround today**: before Space, set the trigger to the gripper's current state (squeezed if it
  holds the object).
- **Planned fix**: hold the last commanded gripper action until the trigger agrees with it. Test: a
  takeover with a disagreeing trigger keeps the gripper until the trigger crosses it.
- **Status**: open (S3).

### S1. `--fwbw` drops interventions

- **Symptom**: `learner/demo_buffer_size` never grows and no `actor/*` metrics appear in a
  `hil_serl --fwbw` run.
- **Impact**: interventions are executed and stored as online rows but never reach the demo buffer.
- **Where**: `hil_serl --fwbw` drives the plain `FWBWActorLoop`
  (`rl_garden_real_world/training/entrypoints/hil_serl.py:82`), whose `_extra_transition_fields` is
  empty (`rl_garden_real_world/loops/actor.py:228`). `residual_hil_serl` and `silri` refuse `--fwbw`
  (`residual_hil_serl.py:64`, `silri.py:173`).
- **Workaround today**: do not use `--fwbw` on the first runs.
- **Planned fix**: an FWBW actor that tags `intervened` and posts episode statistics. Test: an
  intervened FWBW row lands in the demo buffer.
- **Status**: open (S1).

### S5. SiLRI pretraining stalls the actor's uploads

- **Symptom**: at the start of a `silri` run the actor logs `learner at ... unreachable` warnings for
  as long as the behaviour model pretrains; `actor/dropped_transitions` may then rise.
- **Impact**: data loss if the stall outlasts the actor's queue.
- **Where**: `SilriLearnerLoop._train_step` runs `pretrain()` (`--bc-pretrain-steps`, default 500,
  `rl_garden_real_world/training/entrypoints/silri.py:69`) and the behaviour refreshes under the buffer
  lock (`rl_garden_real_world/loops/silri.py:189`). Each request thread of the sync server waits for
  that lock in `_on_transition` (`rl_garden_real_world/loops/silri.py:155`,
  `rl_garden_real_world/loops/sync.py:133`); the actor's push times out after 5 s (`sync.py:336`), so it
  sends one row per 5 s while it produces 10 per second, and its 600-row queue drops the oldest rows
  after about a minute (`sync.py:204`, `:296`; static analysis, not measured). A row whose request
  timed out is still added once the lock frees. Pretraining cannot be finished before the actor
  starts: it waits for `--learning-starts` (10) online rows (`rl_garden_real_world/loops/silri.py:171`).
- **Workaround today**: start the learner, then the actor, and keep control (Space) until the
  warnings stop; if they last more than a minute, lower `--bc-pretrain-steps`. Check
  `actor/dropped_transitions` stays 0.
- **Planned fix**: pretrain outside the buffer lock, or before the server accepts transitions. Test:
  a transition posted during pretraining returns before the push timeout.
- **Status**: open (S5).

### S7. Eval counts your interventions as policy successes

- **Symptom**: `[eval] success_rate=...` is higher than the policy deserves when you touched Space
  during eval.
- **Impact**: a misleading evaluation.
- **Where**: `--role eval` builds the env with the teleop wrapper
  (`rl_garden_real_world/training/env_building.py:373`) and counts every terminated episode as a
  success (`rl_garden_real_world/training/entrypoints/hil_serl.py:224`, `residual_hil_serl.py:209`,
  `silri.py:286`).
- **Workaround today**: do not press Space during eval
  ([06](tutorial/06-intervening.md#the-sequence)); use Esc to end a hopeless episode (a failure).
- **Planned fix**: report intervened episodes separately and exclude them from `success_rate`. Test:
  an eval episode with an intervened step is not counted as a policy success.
- **Status**: open (S7).

### Docs: stale items in `hil-serl-roadmap.md`

Listed here, to be fixed in the docs restructure: item 8's "Also missing" says there is no demo
recorder (there is `examples/record_real_demos.py`); item 4 says the dual-arm gripper is not
implemented (`RLPDHybrid` has a `3 ** num_grippers` head and `real_robot_bimanual` exists); "Landed
since the initial migration" still lists a leader-arm teleop backend as missing; in "Behavioral
Divergences", `backup_entropy`, image augmentation and pooling are now overridden by `HilSerlArgs`.
Status: open (docs).

## B. Not planned

- **GELLO torque-follow** (the leader tracking the follower): needs Dynamixel current control in
  robot_infra and an operator-safety review; the `ramp` and `arm` onsets cover the takeover
  ([gello.md](gello.md#9-future-work-leader-tracking-wp8)). Re-evaluate after the GELLO latency has
  been measured on hardware.
- **Handing back by letting go of the leader**: there is no grip sensor, and a pause would hand back
  by mistake. Use Space, or a foot pedal through the GELLO's `event_device` option
  ([teleop.md](teleop.md#leader-arms-gello)).
- **The actor stops on a teleop read fault**: by design. A failed read propagates
  (`rl_garden_real_world/teleop/base.py:86`, polled before every step at
  `rl_garden_real_world/envs/wrappers/intervention.py:147`), so a dropped reading is never mistaken
  for a policy step. S6's `env.close()` is the fix for the arm.
- **`policy.load_state_dict` on every actor step** (`rl_garden_real_world/loops/actor.py:72`):
  `actor/param_refresh_ms_*` measures it; decide after hardware numbers.

## C. Unverified on hardware

Everything hardware-side has run against fakes only, `leader_cartesian`'s forward kinematics has not
been compared with a robot, ACT has not been trained on real pkls, and the reward classifier has not
seen real frames. Details: [Overview, Known limits](tutorial/00-overview.md#known-limits).

## Closed

None yet.
