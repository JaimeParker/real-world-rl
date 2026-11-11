# Running and recovering real-world experiments

This guide describes durable ingestion, explicit outcomes, part takeover and trajectory archives.
It has CPU/fake-environment regression coverage; hardware validation remains pending.
For device setup and commissioning, continue with the [tutorial](tutorial/00-overview.md).

## Actor identity and durable uploads

The actor commits transitions to a local SQLite outbox before acknowledging a successful
`push_transition` call to the rollout loop. The learner acknowledges a transition after
committing it to its SQLite journal. Retries retain their transition ID; the journal
admits an identical ID/payload once. Reusing an ID with different contents is an error.

The default actor outbox path is:

```text
<log_dir>/<run_id or exp_name or sync_host_sync_port>/<actor_id>/outbox.sqlite3
```

`actor_id` defaults to `actor`. `--actor-outbox-path` selects an explicit file.
Without `--run-id`, the outbox's persistent UUID supplies the message run identity.
Each process also generates a session ID and sequence number. FWBW clients add their
`_forward`/`_backward` direction to the filename and actor identity.

To continue delivery after a restart, use the same outbox path and actor identity.
Keep the matching learner journal/checkpoints. For an independent experiment, choose a
new `--run-id` or an independent explicit outbox path; changing only an experiment label
while forcing the old outbox path also retains its pending messages. Run only one actor
process per outbox. Keep SQLite databases and their WAL files on local storage.

Use a distinct `actor_id` and outbox path for each concurrent actor; restarting the same
actor reuses its existing identity and path.

FWBW learners also keep separate output paths. An explicit `--exp-name experiment`
becomes `experiment_forward`/`experiment_backward`; an explicit checkpoint root becomes
`<root>/forward`/`<root>/backward`. Passing an existing directory named `forward` or
`backward` uses that directory directly and rejects the other direction. Select the
matching direction's manifest when restoring a bundle.

An older flat checkpoint directory containing a journal or bundle has ambiguous direction
ownership and is rejected. Establish which direction owns its records before separating
it. A journal containing both directions needs record identification and separation; copying
it into a direction directory does not establish a valid recovery history. Preserve each
verified direction's complete matching journal, WAL/SHM, bundle generations and snapshots.
No existing directory is moved or rewritten automatically. Standalone legacy `.pt`
weights/replay loading remains available.

Outbox rows are removed after the learner ACK. Unacknowledged rows survive process exit
and are resent on restart. SiLRI also persists its held predecessor transition; recovery
flushes it without inventing a successor onset label. Persistence covers transitions
successfully committed by the upload/held-transition API. An abrupt process death between
robot execution and that commit can still lose the in-memory result of that action.

The outbox and the learner journal run SQLite in WAL mode with `synchronous=NORMAL`, so a
commit does not wait for an fsync and the robot control loop does not pay one per step. A
committed row survives a crash or kill of the actor or learner process. An operating-system
crash or power loss can roll back the most recent commits on that machine: on the actor,
the newest outbox rows; on the learner, the newest journal rows, which the actor may already
have removed after their acknowledgement.

The actor keeps running while the learner is unreachable (learner restart, network outage):
uploads retry with a back-off that grows from 0.1 s to 2 s and resets on the first
acknowledged batch, a warning is logged on the 1st and every 10th consecutive failure, and
rows accumulate in the outbox until the learner returns. Collection stops only when the
outbox reaches its record or byte capacity, a disk write fails, or the learner rejects
records with an HTTP 4xx error. `ActorSyncClient(max_consecutive_failures=N)` restores a
bounded tolerance; the default, `None`, never gives up.

## Startup and shutdown

`--sync-startup-timeout` defaults to 30 seconds. The actor waits for an initial learner
policy snapshot before resetting/stepping the environment. Parameter snapshots include
learner session/version, training phase and policy readiness. A learner restart can
therefore publish a lower version under a new session and still refresh the actor.
Residual warm-up respects policy readiness and uses its configured base/residual warm-up.

`--sync-drain-timeout` defaults to 30 seconds. On a normal stop or exception, rollout
cleanup releases the environment/provider before attempting the held-transition flush,
episode finalization and sync drain, so artifact writes do not keep arm control active.
A drain timeout leaves unacknowledged rows in the outbox and reports their count; restart
with the same path to deliver them. A forced process kill cannot execute cleanup; the
robot daemon's watchdog remains part of that failure path. In single-process mode (no
daemon) see [Watchdogs and the supervisor process](#watchdogs-and-the-supervisor-process).

## Watchdogs and the supervisor process

Three independent layers stop a robot whose control loop goes quiet. None of them replaces
the physical stop button or the robot's own protections.

| Layer | Where it runs | Detects | Action |
|---|---|---|---|
| Daemon watchdog, `robot-infra daemon --watchdog-s 0.5` | the daemon process on the robot PC (daemon route only) | no action for `watchdog_s` while servoing | `arm.stop()`, arm to `IDLE` |
| Step watchdog, env config `watchdog_s` (default off) | a thread inside the robot process | no `env.step()` for `watchdog_s` after a reset | `arm.stop()` once; the next step raises `StepWatchdogTripped` |
| Supervisor, `rwrl supervise` | a separate parent process | no heartbeat for `--heartbeat-timeout-s` | SIGINT, then SIGTERM, then SIGKILL to the robot process group |

### Step watchdog (in-process)

`RealRobotEnvConfig.watchdog_s` (YAML `real_robot.watchdog_s`) starts a `StepWatchdog`
when a reset finishes and pauses it at the start of every reset. The value must exceed one
control period. Steps check in on entry and after the command returns, so a slow caller
(policy, teleop source, logging) and a step blocked in a device call are both detected.
After a trip, the arm has been stopped once and every later step raises until the next
reset. A trip during silence after an episode's last step (finalization, saving) is logged
as a warning at the next reset and is otherwise harmless. The setting is not part of the
experiment contract.

It shares the process and the GIL with the control loop. It does not cover a frozen process
(SIGSTOP, a suspended machine, an extension call holding the GIL), a killed process, or a
`stop()` that itself hangs. Those are what the supervisor is for.

### Supervisor process

Run the recorder or the actor as the child of a supervisor:

```bash
rwrl supervise --log runs/<run>/supervise.jsonl -- \
    rwrl record --config hil.yaml ...
```

| Option | Default | Meaning |
|---|---|---|
| `--heartbeat-timeout-s` | `2.0` | silence after which the escalation starts |
| `--grace-s` | `3.0` | SIGINT to SIGTERM |
| `--kill-s` | `2.0` | SIGTERM to SIGKILL |
| `--startup-s` | `30.0` | no heartbeat required until the first message arrives or this elapses |
| `--long-op-s` | `60.0` | silence allowed after the child announces a long operation (an env reset) |
| `--log` | none | jsonl: child pid, every escalation with monotonic and wall timestamps, exit code/signal |

The child runs in its own session, so a terminal Ctrl-C (or SIGTERM, or the terminal closing)
reaches only the supervisor. The supervisor forwards SIGINT to the child's process group and
keeps supervising until the child exits; it never exits first. Its exit code is the child's
(128 + N for signal N). After an escalation it is non-zero even when the child exited 0, and
the reason is printed on stderr. When the child exits, anything left in its process group is
killed. If the supervisor dies, the kernel sends the child SIGINT, and the child's next
heartbeat raises `SupervisorLost`. Linux only.

The recorder and the actor heartbeat through `robot_infra.supervise.Heartbeat`: one beat per
step and after every reset, and a long-operation message before every reset. Without the
supervisor the calls do nothing. Choose `--startup-s` longer than everything before the first
reset, for the actor including `--sync-startup-timeout`.

### What the supervisor can and cannot do

In single-process mode the env drives the arm through the in-process SDK (franky). The
supervisor has no connection to the robot. Its only lever is to kill the robot process.
libfranka then loses its control connection and the robot stops at the firmware level: a
reflex stop, with the robot left in `Reflex` mode. This is accepted as the last-resort stop.
It is not a controlled stop: there is no deceleration profile chosen by us, and the gripper
keeps its last command.

It cannot:

- stop the robot more gently than a firmware reflex, or call `arm.stop()`: that exists out of
  process only on the daemon route (`--watchdog-s`);
- detect a loop that keeps beating while doing the wrong thing (bad actions, a wrong target);
- act when the whole machine is frozen or loses power, or when the network to the robot
  fails: those rely on the robot's own communication timeout;
- guarantee a bound on the stop time. The bound is roughly the heartbeat timeout plus
  `--grace-s` plus `--kill-s` (7 s with the defaults) when SIGINT and SIGTERM are not
  honoured, plus libfranka's own connection timeout.

### Recovering after a supervisor kill

1. Check that the arm is at rest and the work cell is safe; keep the physical stop in reach.
2. Read the reason in the supervisor's stderr and the `escalate` events of `--log`.
3. The robot is in `Reflex`: release the external activation device if it is pressed, clear
   the errors in Desk, or run `rwrl doctor --station robot.yaml --fix`, which
   recovers the arm and re-runs the mode check.
4. The killed run did not finalize. The SQLite outbox/journal keeps every committed row;
   the recorder's last episode and the action in flight are lost. Restart with the same
   outbox path to deliver pending rows.

## Learner checkpoints and recovery

The learner journal is `<checkpoint_dir>/journal.sqlite3`. Journal admission is independent
of training and replay-buffer mutation, so long SiLRI pretraining does not hold the HTTP
request behind the training buffer lock. The training thread drains journal rows in order.

A checkpoint bundle contains the model/optimizers, online and demo replay, ring cursors,
intervention extension columns, loop counters and journal cursor/identity. FWBW replay
boundary/Monte Carlo state and SiLRI refresh counters are retained. A generation becomes
current only when `bundle_manifest.json` is atomically published; an interrupted generation
write leaves the previous manifest current.

Resume with the matching checkpoint directory and manifest together:

```text
--load-checkpoint /path/to/checkpoints/bundle_manifest.json \
--checkpoint-dir /path/to/checkpoints
```

The matching `--exp-name`/`--log-dir` can also select the existing checkpoint directory.
A manifest path alone does not relocate the journal: a new run directory creates a new
journal and its identity will be rejected. The manifest is the recovery entry point and always restores the complete online/demo/
cursor consistency unit. `--load-replay-buffer` controls legacy `.pt` loading; it does not
turn a bundle restore into a weights-only restore. For weights-only initialization, select
the generation's `agent.pt` and start a new run with an empty journal.

Bundles include full replay buffers by default, with corresponding disk space, save-time
and temporary CPU-memory costs. Periodic saves respect `checkpoint_freq`; final saves
respect `save_final_checkpoint`. Stopping the learner with Ctrl-C (`KeyboardInterrupt`) or
`SystemExit` is a normal stop: it writes `final.pt` and a new bundle, including when the
interrupt lands inside an update. An exception raised by an update, or any interruption
while a journal record is being applied to the replay buffers, skips the final save so the
last complete bundle stays current; resuming from it replays the journal tail. Keep the referenced `bundle_*` directory and
matching journal together. A journal identity mismatch is rejected. After restore, only
rows after the saved journal cursor are applied; seed demos and old pickle snapshots are
not loaded again on top of bundle replay.

An explicit legacy `.pt` checkpoint remains supported. `--load-replay-buffer` requests its
associated native replay file; an existing replay is restored through the real-world
adapter rather than silently ignored. A native replay checkpoint has no journal cursor,
so it requires an empty journal. Do not combine an unrelated legacy replay with a current
journal. Legacy `.pkl` transition snapshots and seed-demo paths remain fallback sources
when there is no authoritative bundle/replay recovery. Keep the original action-contract
sidecars with the data.

Legacy core replay may lack intervention provenance. It remains readable, with a warning.
A checkpoint explicitly identified as the old `BC` algorithm retains its full-dimension
supervision intent when intervention labels and masks are absent. Explicit `intervened`
values and existing empty or partial masks take priority. Action source, policy proposal
and episode outcome remain unknown. Native replay without that algorithm identity,
including unknown online replay, does not acquire expert labels. Recorded actions remain
placeholders for unknown policy proposals. Missing algorithm-required base actions or configured penalty columns
cannot be reconstructed; use matching transition snapshots or a complete bundle. A legacy
five-field demo remains readable, and archive metadata is projected out before buffer
`add` calls. Versioned archives retain partial human-action masks separately from replay training views.

## Reading success and intervention metrics

Evaluation keeps human takeover available by default. The primary `success_rate` is an
alias for `autonomous_success_rate`:

```text
successful completed episodes with no action intervention / all completed episodes
```

`task_success_rate` includes successful assisted episodes. `episode_intervention_rate`
counts completed episodes with action intervention; `step_intervention_rate` counts their
intervened steps over completed-episode steps. Unknown outcomes stay in the completed
success-rate denominator and are also reported as `unknown_episodes`. Interrupted and
rerecord/discarded attempts have separate counts and stay outside that denominator.

Success comes from explicit environment/classifier/human labels or teleop success events.
Positive reward and `terminated=True` alone do not establish task success. Human failure
vetoes success, including simultaneous success/failure events. Pressing a success/failure
label button without taking action control does not make an episode assisted. Reports
also preserve outcome source and end reason, so operator judgment can be distinguished
from classifier judgment. `done` for replay remains `terminated`; timeout truncations
continue to bootstrap according to the existing algorithm contract.

## Part takeover and imitation

`--teleop-selected-parts` selects named parts, for example `left.arm`, `right.arm`,
`left.gripper` and `right.gripper` on a two-arm setup; a single arm uses `arm.arm` and
`arm.gripper`. Names follow the environment's `arm_names`. Leaving the list empty uses the
source's available parts. The action vector keeps its original order: all arm blocks,
then one gripper channel per arm when grippers are included.

`--teleop-idle-behavior hold` is the default. During a partial takeover, unselected arm
parts hold their current follower pose (`joint_abs`) or zero delta; grippers retain the
last commanded target. `--teleop-idle-behavior policy` leaves those parts with the policy.
`human_action_mask` identifies actual human control. `override_mask` also includes the
hold supplements. An empty human mask supplies no expert label, and hold supplements do
not train imitation losses. Legacy full-action interventions retain full-mask behavior.

Leader catch-up executes and remains in the archive. In `joint_abs` with the `ramp` onset,
every rate-limited step is catch-up. In `joint_delta`, only the steps from the takeover until
the leader is first within `joint_scale_rad` of the follower are catch-up; later saturated
steps of the same takeover (the operator moving faster than the follower) are ordinary human
teaching and supervise like any other intervened step. The catch-up rows' `syncing` marker
disables demonstration supervision. The recorder's `--keep-syncing-steps` keeps them in its
pickle export as ordinary supervised rows (the archive still marks them `syncing`);
archive-derived training views continue to exclude them. A leader with no gripper channel holds the
previous target instead of supplying an expert gripper label.

BC, SiLRI behavior fitting and actor pretraining supervise only human dimensions. For
continuous NLL, each supervised row uses `D / k * sum(mask * NLL)`, where `D` is the full
continuous width and `k` its supervised width. Empty rows contribute no gradient, and full
masks reproduce the original objective. Residual BC also intersects the human mask with
nonzero residual scales. SiLRI's joint gripper head uses the marginal probability of the
observed human grippers, summing over unobserved gripper choices. Its online behavior-prior
distance and multiplier constraint retain their existing full-continuous-action semantics.

No-gripper HIL uses the local continuous `ContinuousHilRLPD` branch. Residual continuous
and SiLRI continuous also accept no-gripper contracts. Hybrid/discrete-gripper branches
require at least one gripper and reject a zero-gripper configuration before rollout.

## Trajectory archives and training views

Recording creates `<output stem>.trajectory.sqlite3` alongside the successful-demo `.pkl`.
Episode start, every step and episode end are committed through the same SQLite durability
layer as ingestion. The archive includes policy proposals, executed actions, human/override
masks, syncing, termination/truncation, reward components, outcome source and experiment
identity. Failed and rerecorded attempts remain archived. Iteration is paginated, so reopening
an archive does not load the historical camera frames into an in-memory list.

After an abrupt stop, reopening the same archive marks an unfinished episode interrupted
and starts future attempts under new episode IDs. The recovery guarantee starts when a
record's durable append returns; an action executed before that append remains subject to
the execution-to-commit gap. `--export-trajectory-pt` explicitly creates a portable plain
record export; SQLite is the default archive and recovery source.

`--demo-dataset-paths` accepts legacy `.pkl`, `.trajectory.sqlite3` and explicit
`.trajectory.pt` exports. Archive training views select human, nonsyncing steps from complete
successful episodes. Original step indices remain in the archive, so filtering does not
invent temporal adjacency or move terminal rewards to earlier human steps. Legacy human
demo datasets without an intervention column retain full-dimension supervision; explicit
false labels and unknown online-checkpoint provenance remain nonexpert.

The versioned experiment contract has a compared part and a provenance part. The compared
part is what changes what the policy sees or how its actions are interpreted: the task,
control settings, `control_hz`, `joint_limits`, depth cameras, `image_size`, image
preprocessing, the final observation layout (keys, shapes, camera order, frame stack) and
the teleop part selection/idle behavior. Recording, training, deployment and evaluation must
agree on it; a mismatch is rejected rather than replayed under different action or
observation semantics, and only this part enters the experiment hash. The provenance part is
saved with the contract and shown in mismatch errors but never compared: the reward sources
(classifier paths, contents and threshold, `reward_from_teleop_events`,
`teleop_success_reward`, grasp penalty) and the station facts the command line owns
(`safety_box`, `home_joints`, `reset_settle_s`, `max_episode_steps`, and the `cameras`
list, whose names and order are compared through the observation layout). So demos recorded with the recorder's teleop-event reward load into a run
with a classifier, and a deploy may use another home pose or episode cap. Data recorded
before experiment metadata existed loads with a warning unless the run's task, control,
depth or preprocessing settings need that metadata.
Two-arm task outcomes retain per-arm labels. A one-arm success does not confirm an entire
parallel task; an explicit whole-task confirmation uses `task_success`/`task_failure` events.
Checkpoint and actor parameter loading also verify the constructed policy's action-bound,
residual-scale and discrete-action buffers. A model with the same action width but different
action interpretation fails before reset motion; continuous heads retain their own shapes
when gripper heads are separate.


## Task presets and observation settings

Run the complete local-fake task/reset path without algorithm training:

```bash
PYTHONPATH=.:rl-garden:robot_infra python examples/task_smoke.py --config examples/presets/pick_and_place_fake.yaml
PYTHONPATH=.:rl-garden:robot_infra python examples/task_smoke.py --config examples/presets/parallel_pick_and_place_fake.yaml
```

These presets show versioned tasks, reset waypoints/jitter, control settings, termination
and evaluation overrides. `success_source=tcp_proxy` reports a TCP-position proxy metric;
the main task success remains unknown. Physical pick/place success uses explicit manual
or classifier labels. Parallel classifier tasks need one `classifier_checkpoints` entry
per arm and may select per-arm `classifier_image_keys`; all required arms must succeed.

Per-camera `preprocess` supports pixel ROI `[top,left,height,width]`, source resolution
`[height,width]` and depth clipping in metres. Configure depth cameras in the environment
and matching observation depth keys. RGB uses bilinear resizing; depth uses nearest-neighbor
resizing, metre units, one channel and zero for invalid pixels. Requested and resolved
control/safety/SDK settings are recorded with run and generation metadata. Conflicting
nondefault global/environment control frequencies are rejected, and actor/deploy use the
resolved frequency.

Control frequencies, action scales, reset durations and configured station/environment
geometry must contain finite values. Invalid action values are rejected before any arm
command; a two-arm action is checked in full before either arm steps. Environment safety
limits may only tighten station limits.

When classifier files move, deployment accepts replacement `classifier_checkpoint` or
task `classifier_checkpoints` paths. A task path override can leave the other task fields
at their defaults; the saved task still supplies the task settings. Classifier paths and
contents are provenance: a retrained classifier deploys, and its content hash is recorded
next to the one the policy was trained with.
