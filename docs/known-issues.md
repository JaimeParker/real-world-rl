# Known issues and deferred work

What a first real run on one Franka arm with a gripper (GELLO through the robot daemon,
`--teleop-device leader_arm`, keys from `--teleop-event-device keyboard`) can run into, where it is
in the code, and what to do about it today. Every `path:line` was checked against the code when the
entry was written; line numbers drift, so search for the named function if one no longer matches.
Numbers marked "static analysis, not measured" come from reading the code, not from a run.

Each entry gives the symptom, the impact, where it is, the workaround for today, the planned fix with
the test that will prove it, and its status. Fixed entries move to [Closed](#closed) with their validation.

## A. To fix after the first real run

In dispatch order.


### S8. Untrained `joint_abs` startup motion

A normalized action near zero commands the midpoint of joint limits. An initial learner
snapshot can still contain an untrained policy; the readiness handshake does not replace
that policy with a measured-pose hold for nonresidual `joint_abs` runs. Commission this mode
with the configured per-step motion bounds. A hold/warm-up policy before autonomous control
remains open. Truncation/bootstrap boundaries now have regression coverage in
`tests/test_partial_imitation.py` and the trajectory recovery tests.

Status: open for the startup hold/warm-up behavior (S8).

### S3. Taking over can open the gripper

- **Symptom**: the policy holds the object, you press Space with the trigger released, and the
  gripper opens at once.
- **Impact**: safety (a dropped object) and a bad intervened row.
- **Where**: the trigger maps straight to `2w - 1` (`real_world_rl/teleop/part.py:273`); the
  onset policies act on the joints only (`real_world_rl/teleop/leader_arm.py:161`, gripper added
  at `:164`).
- **Workaround today**: before Space, set the trigger to the gripper's current state (squeezed if it
  holds the object).
- **Planned fix**: hold the last commanded gripper action until the trigger agrees with it. Test: a
  takeover with a disagreeing trigger keeps the gripper until the trigger crosses it.
- **Status**: open (S3).


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
  (`real_world_rl/teleop/base.py:86`, polled before every step at
  `real_world_rl/envs/wrappers/intervention.py:147`), so a dropped reading is never mistaken
  for a policy step. Actor cleanup now closes the environment; a forced kill still relies on the watchdog.


## C. Unverified on hardware

Everything hardware-side has run against fakes only, `leader_cartesian`'s forward kinematics has not
been compared with a robot, ACT has not been trained on real pkls, and the reward classifier has not
seen real frames. Details: [Overview, Known limits](tutorial/00-overview.md#known-limits).

## Closed

### S1. FWBW intervention tracing

Each FWBW direction now emits its own durable trajectory segment with executed/proposed
commands and actual human masks. Human rows reach online and demonstration replay through
the same training projection as other actor paths; segment boundaries prevent n-step
windows crossing a direction change.

Status: fixed. Regression coverage:
`tests/test_p1_ingestion.py` FWBW trace/partial-human regression.


### Actor parameter versions

Actor snapshots use learner session/version identity. The actor loads a new state only when
that identity changes, including a learner restart with a lower version in a new session.
Status: fixed. Regression coverage: P0 lifecycle tests.


### S2. Catch-up motion and partial demonstration labels

The catch-up after a takeover carries syncing metadata: in `joint_delta` the steps until the leader
is first within `joint_scale_rad` of the follower, in `joint_abs` the rate-limited ramp steps. Once
aligned, a takeover is supervised normally. Durable archives preserve the executed motion and
actual human mask; the training projection excludes syncing supervision. Human/override masks
also keep hold supplements out of partial imitation losses. Recorder pickle exports made with
`--keep-syncing-steps` keep those rows as ordinary supervised steps.

Status: fixed. Regression coverage:
`tests/test_partial_imitation.py`, `tests/test_leader_arm_source.py`, and recorder regressions.


### S4. Learner replay and checkpoint recovery

Durable journal admission, atomic checkpoint generations, replay-extension restoration and journal cursors now preserve committed rows and avoid bundle-plus-snapshot double loading. Legacy native replay remains readable with explicit provenance limits.

Status: fixed. Regression coverage: `tests/test_replay_state.py`, `tests/test_p0_process_recovery.py`.


### S6. Actor lifecycle and pending uploads

Actor/eval cleanup releases environment resources and drains uploads with a deadline. SQLite outboxes retain unacknowledged messages and SiLRI held rows. A forced process kill still relies on the daemon watchdog; the last action before its durable commit is outside the persistence guarantee.

Status: fixed. Regression coverage: `tests/test_p0_process_recovery.py` and `tests/test_p0_lifecycle.py`.


### S5. SiLRI pretraining and ingestion

HTTP admission commits to the learner journal independently of the training buffer lock. Stable transition IDs make retries idempotent; outbox rows survive a missed ACK or process restart.

Status: fixed. Regression coverage: `tests/test_p0_process_recovery.py` and `tests/test_p0_lifecycle.py`.


### S7. Autonomous and assisted evaluation

Shared explicit outcomes now drive actor/eval/deploy/recording. The primary success rate uses all completed episodes and credits only successes without action intervention; total task success and intervention rates are separate. Positive reward or termination alone does not imply success.

Status: fixed. Regression coverage: `tests/test_episode_outcomes.py`, `tests/test_p0_lifecycle.py` and `tests/test_real_demo_recorder.py`.


Operational details and remaining recovery limits: [Real-world experiments](real-world-experiments.md).
