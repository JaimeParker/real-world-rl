# 10. HIL training

**Goal:** start a learner and supervised robot actor, observe learning and intervention
metrics, then stop, evaluate and resume with the same experiment contract.

Three methods share the same actor, learner and eval roles, teleop flags, demo pkl and sync
protocol. Run one at a time, each with its own `--exp-name`:

| Method | What it adds | Concepts |
|---|---|---|
| `hil_serl` | RLPD with human interventions, a growing demo buffer and a DQN gripper head | [README](../../README.md#running-on-hardware), [hil-serl-roadmap.md](../hil-serl-roadmap.md) |
| `residual_hil_serl` | a bounded residual on top of a frozen base policy (BC or ACT from [09](09-base-policy.md)) | [residual.md](../residual.md) |
| `silri` | an actor pulled towards a learned model of your interventions | [silri.md](../silri.md) |

## Before you start

- Finish [08 Record demos](08-record-demos.md); for a residual run also finish
  [09 Base policy](09-base-policy.md). Read the [operator page](../intervening.md).
- The actor is the single hardware process on the **NUC**, connected directly through franky.
  Keep the workspace clear, the physical stop within reach and other device owners stopped.
- Run the learner on the **GPU host** (which may be the NUC); start it first. Commands use
  each machine’s **repository root** and **project environment**.
- Copy the full commissioned `local/gello_cartesian.yaml`, required leader options and
  demo pkl/sidecar to both hosts. Use the full preset on the learner too, not the offline BC
  copy of [09](09-base-policy.md). Keep base checkpoints and their metadata at the paths below.

For an actor on a different machine from the robot host, follow
[Remote actor over the robot daemon](../remote-actor.md) for the daemon, transport and
connection settings. The commands here use the direct NUC route.

When using a calibrated SpaceMouse station, provide matching effective
task-frame/workspace and profile settings to recording, actor and learner.
Profile file paths may differ between machines, but their numeric transforms
must agree; changing only a profile name does not change the numerical contract.
The learner resolves these settings without opening HID. Select
`episode_end_behavior: hold` explicitly for manually advanced actor episodes;
it does not insert neutral gates between interventions. See the
[SpaceMouse CLI guide](../spacemouse-commissioning.md#spacemouse-profiles-and-task-frame-workspace)
and [physical acceptance checks](../testing/spacemouse-control-session.md).

## 1. Set the shared flags

All commands run from the **repository root**, with the machine's
**project environment active**. Set the following variables in **both the learner and NUC
shells**. The example uses `127.0.0.1` for a learner on the same machine; for a
separate GPU learner replace it with that machine's reachable IP in both shells.
Ensure both checkouts have the same preset and experiment inputs.

```bash
LEARNER_IP=127.0.0.1
ENV_FLAGS="--config local/gello_cartesian.yaml --teleop-device leader_cartesian \
    --teleop-event-device stdin --reward-from-teleop-events"
COMMON="$ENV_FLAGS --sync-host $LEARNER_IP --sync-port 6000 --log-dir runs \
    --checkpoint-freq 1000 --buffer-size 20000 --demo-buffer-size 20000"
```

*You should see:* the variables defined in both shells (assignments print nothing). If the
learner is remote, both `--sync-host` values must name its reachable address.

Use the station-specific [08 setup](08-record-demos.md#before-you-start), including
`gello_leader.yaml`, on the NUC. The preset is state-only `cartesian_delta` at
50 Hz, with `xyz_scale_m: 0.05` and `rot_scale_rad: 0.3`.
Keep the recording device and the four tracker caps/speeds unchanged. The demo
sidecar's `experiment.teleop_control` records the tracker version and values;
changing or omitting it is rejected. Current HEAD compares tracker identity,
not physical input-device identity; the tutorial requires the same device as an
operator workflow rule ([compatibility note](08-record-demos.md#files-and-compatibility)).

To use SpaceMouse recordings instead, set `--teleop-device spacemouse` and
`--teleop-options-yaml None` in `ENV_FLAGS` in both shells, use the matching demo
path, and keep the four values unchanged. Button 0 handles takeover; stdin supplies
`s`/`f`/`r`/Esc/`q`. Moving the puck alone never takes over.

For native SpaceMouse intervention, add `--spacemouse-mode serl` to `ENV_FLAGS`
in **both the NUC and learner shells**, **repository root**, each machine's
**project environment active**. Use a station preset with xyz and RPY safety
boxes, and demos recorded in the same mode. Mode resolution saves
`real_robot.cartesian_semantics: serl` before training starts, including for
the learner with no device:
raw six-axis norm `> 0.001` intervenes, releasing the puck and buttons hands back,
button 0 held closes, button 1 held opens, and neither sends neutral gripper zero.
Tracker caps/speeds do not apply; the compared contract is `spacemouse_serl_v1`.
See [both modes](../teleop.md#spacemouse); hardware behavior is unverified.

Each hardware command below is wrapped by `rwrl supervise`. Its heartbeat
silence threshold defaults to **2.0 s**. `--startup-s 60` allows initial policy
loading and the actor's snapshot wait before its first reset heartbeat. Check that
this exceeds startup work on your machine; the default startup allowance is 30 s.
The current preset leaves the **env StepWatchdog off** (no `real_robot.watchdog_s`);
this is separate from the supervisor. Recording in [08](08-record-demos.md)
explicitly enables a 0.3-second StepWatchdog. No actor StepWatchdog value is
specified in the current [experiment guide](../real-world-experiments.md#step-watchdog-in-process).

- `--reward-from-teleop-events`: your `s` key is the reward (1, terminal) and `f` ends an episode as
  a failure. Training does not turn it on by default; without it, and without
  `--classifier-checkpoint`, every transition carries reward 0 and the actor and learner warn
  ([operator page](../intervening.md#where-the-reward-comes-from)).
- `--checkpoint-freq` (default 0: final saves only) writes
  `checkpoint_<update>.pt` plus a checkpoint bundle every that many learner updates.
- `--buffer-size` is the replay capacity (defaults: 200000 for `hil_serl` and `residual_hil_serl`,
  30000 for `silri`), held on `--buffer-device cuda`; with several 128 x 128 cameras size it to your
  GPU memory or use `--buffer-device cpu`. Every checkpoint bundle copies both replay buffers at
  full capacity to host memory and disk. Size both `--buffer-size` and `--demo-buffer-size` to
  available GPU/host memory and disk; 20000 rows is about 6 minutes 40 seconds at 50 Hz. Each periodic save
  creates another generation. Keep the generation referenced by the manifest with its journal.
- `--teleop-device` has no default. Actor and eval must select a source explicitly or in
  the preset. The learner does not open a teleop device, but still uses the tracker settings
  to validate the experiment contract.

## 2. Run hil_serl

On the GPU host, repository root, project environment active, start the learner:

```bash
rwrl train hil_serl --role learner $COMMON --exp-name hil-pick-and-place \
    --demo-dataset-paths demos/pick-and-place.pkl
```

*You should see:* the learner start and wait for actor data. If it exits, check the named
path or contract difference before starting the actor.

On the NUC, repository root, project environment active, start the actor:

```bash
rwrl supervise --startup-s 60 --log runs/hil-pick-and-place/supervise.jsonl -- \
    rwrl train hil_serl --role actor $COMMON --exp-name hil-pick-and-place
```

*You should see:* the actor receive a policy snapshot, reset and begin episodes. If startup
fails, use [If it fails](#if-it-fails) before another reset.

The actor waits up to `--sync-startup-timeout` (default 30 seconds) for an initial learner
snapshot before reset/step. That snapshot can still contain an untrained policy;
cartesian teleop does not make its autonomous actions safe. Be ready to take over
with `t` and use the physical stop when needed.
The learner preloads demos, waits for `--learning-starts` online transitions (default 4000;
preloaded demos do not count), then trains. It publishes the first ready policy and later
snapshots every `--publish-freq` updates (default 100). Snapshot session/version identifies
updates across a learner restart. Be ready to intervene while the policy is learning.

*You should see:* `learner/received` and buffer sizes growing as actor data arrives; after
learning starts, `learner/update` and SAC losses. Each episode supplies `actor/task_success`
and intervention metrics ([table below](#8-watch-the-metrics)). If ingestion stalls, check the
sync address and actor outbox; hardware failures go through [If it fails](#if-it-fails).

<a id="residual_hil_serl-bc-base"></a>

## 3. Alternative: residual_hil_serl, BC base

Set `RESIDUAL` and `BASE_BC` in both shells with the project environment active; run the learner first on
the learner machine, then the supervised actor on the NUC, from each repository root.

In both shells, repository root, project environment active, define:

```bash
RESIDUAL="--residual-variant hybrid --residual-action-scale 0.1 --residual-bc-weight 0.1 \
    --critic-only-updates 500"
```

*You should see:* no output; the variable is available to the commands below.

In both shells, repository root, project environment active, define:

```bash
BASE_BC='checkpoint:runs/bc-pick-and-place/checkpoints/final.pt?algo=real_world_bc'
```

*You should see:* no output; the variable is available to the commands below.

On the GPU host, repository root, project environment active, start the learner:

```bash
rwrl train residual_hil_serl --role learner $COMMON $RESIDUAL \
    --exp-name residual-bc-pick-and-place --base-provider "$BASE_BC"
```

*You should see:* the learner start and wait for actor data. If it exits, check the named
path or contract difference before starting the actor.

On the NUC, repository root, project environment active, start the actor:

```bash
rwrl supervise --startup-s 60 --log runs/residual-bc-pick-and-place/supervise.jsonl -- \
    rwrl train residual_hil_serl --role actor $COMMON $RESIDUAL \
    --exp-name residual-bc-pick-and-place --base-provider "$BASE_BC"
```

*You should see:* the actor receive a policy snapshot, reset and begin episodes. If startup
fails, use [If it fails](#if-it-fails) before another reset.

*You should see:* ingestion and actor episode metrics, then
`losses/actor/residual_bc` and `losses/actor/intervened_fraction` after critic-only updates.
If base loading or contract validation fails, check the [09 metadata](09-base-policy.md#keep-the-experiment-contract-with-the-base)
and [If it fails](#if-it-fails).

## 4. Alternative: residual_hil_serl, ACT base

Set `RESIDUAL` and `BASE_ACT` in both shells with the project environment active; run the learner first on
the learner machine, then the supervised actor on the NUC, from each repository root.

In both shells, repository root, project environment active, define:

```bash
RESIDUAL="--residual-variant hybrid --residual-action-scale 0.1 --residual-bc-weight 0.1 \
    --critic-only-updates 500"
```

*You should see:* no output; the variable is available to the commands below.

In both shells, repository root, project environment active, define:

```bash
BASE_ACT='act:runs/act-pick-and-place/checkpoints/final.pt?temporal_agg=1'
```

*You should see:* no output; the variable is available to the commands below.

On the GPU host, repository root, project environment active, start the learner:

```bash
rwrl train residual_hil_serl --role learner $COMMON $RESIDUAL \
    --exp-name residual-act-pick-and-place --base-provider "$BASE_ACT"
```

*You should see:* the learner start and wait for actor data. If it exits, check the named
path or contract difference before starting the actor.

On the NUC, repository root, project environment active, start the actor:

```bash
rwrl supervise --startup-s 60 --log runs/residual-act-pick-and-place/supervise.jsonl -- \
    rwrl train residual_hil_serl --role actor $COMMON $RESIDUAL \
    --exp-name residual-act-pick-and-place --base-provider "$BASE_ACT"
```

*You should see:* the actor receive a policy snapshot, reset and begin episodes. If startup
fails, use [If it fails](#if-it-fails) before another reset.

*You should see:* the same residual metrics as with BC, including
`losses/actor/residual_bc` after critic-only updates. If ACT cannot load, check its saved
observation/action widths and base path before [If it fails](#if-it-fails).

The actor executes `clamp(base + scale * residual, -1, 1)`; your interventions override the total
action by default; selected-part takeover uses the masks described on the [operator page](../intervening.md#take-over-selected-parts). The base must come from the same cartesian preset and tracker contract (same observation and action spaces, the
[contract of 09](09-base-policy.md#the-act-checkpoint-contract)). The base spec is quoted: `?` is a
glob character and `&` backgrounds. Only the actor and eval build the base; the learner trains on the
base actions the actor stores. With the ACT base the actor replans the chunk after every intervened
step ([resync](09-base-policy.md#resync-after-an-intervention)).

- `--residual-variant hybrid` puts the residual on the arm dims and gives the gripper `hil_serl`'s
  DQN head; `continuous` represents all dims, with zero residual scale on grippers by default, keeping the base's gripper.
- `--residual-action-scale` bounds the residual (0.1 of the normalised action per step).
- `--critic-only-updates 500`: the first 500 learner updates train the critic only.
- `--residual-bc-weight 0.1` pulls the residual towards your corrections on intervened rows.

Teleop recordings carry no base actions, so the demo pkl is not passed with `--demo-dataset-paths`
here (the learner rejects it unless you add `--allow-base-less-demos`, which learns its rows as
residuals from a zero base, see [residual.md](../residual.md#human-intervention)).

<a id="silri"></a>

## 5. Alternative: silri

On the GPU host, repository root, project environment active, start the learner:

```bash
rwrl train silri --role learner $COMMON --exp-name silri-pick-and-place \
    --demo-dataset-paths demos/pick-and-place.pkl
```

*You should see:* the learner start and wait for actor data. If it exits, check the named
path or contract difference before starting the actor.

On the NUC, repository root, project environment active, start the actor:

```bash
rwrl supervise --startup-s 60 --log runs/silri-pick-and-place/supervise.jsonl -- \
    rwrl train silri --role actor $COMMON --exp-name silri-pick-and-place
```

*You should see:* the actor receive a policy snapshot, reset and begin episodes. If startup
fails, use [If it fails](#if-it-fails) before another reset.

Before its first update the SiLRI learner pretrains its human-behaviour model on the demo buffer
(`--bc-pretrain-steps`, default 500); the actor's rows keep arriving in the learner's journal while it
does and are consumed afterwards. It then refreshes the model as your interventions arrive
([silri.md](../silri.md#data-flow)).

*You should see:* `behavior/pretrained`, `behavior/pretrain/*` and later
`behavior/refresh/*`, alongside ingestion, episode and SiLRI loss metrics in the table below.
If those remain absent, check demo loading and received rows before
[If it fails](#if-it-fails).

## 6. Stop and save

Finish the episode in a safe state, then press `q` to stop the NUC actor while the
learner is still running. Ctrl-C at the supervisor terminal also requests cleanup. Cleanup releases the environment/provider and attempts to drain uploads for
`--sync-drain-timeout` (default 30 seconds). A timeout leaves unacknowledged rows in the local
outbox for the next restart. If cleanup stalls, the supervisor escalates signals; a final kill breaks the
libfranka connection and relies on firmware Reflex stopping. Inspect the cell
and supervisor log before clearing faults and restarting. Then stop the learner:
under a normal stop it writes
`runs/<exp-name>/checkpoints/final.pt` and a checkpoint bundle (model, optimizers, both replay
buffers and the journal position, published through `bundle_manifest.json`) on the way out.
`runs/<exp-name>/config.json` is what eval, deploy and the residual base provider read. Rows the
actor could not deliver before it stopped stay in its outbox under `runs/<exp-name>/actor/` and are
sent when an actor with the same outbox path and identity starts again. The default path above
assumes `--run-id` and `--actor-outbox-path` were not overridden. Each concurrent actor needs
its own `--actor-id` and outbox.

*You should see:* the actor exit, then `final.pt`, `config.json` and a complete bundle
manifest after the learner stops normally. If cleanup times out, preserve the outbox and
journal and follow [resume](#9-resume-an-interrupted-run).

<a id="evaluate"></a>

## 7. Evaluate

Deterministic episodes on the robot, no learner, no data collection; a success is an explicit
success label (your `s` with `--reward-from-teleop-events`, or the classifier), and it only counts as
autonomous when you did not take over during the episode. Stop the actor first, and do not
intervene ([operator page](../intervening.md#the-sequence)). Run each eval command on the
NUC, repository root, project environment active, after defining `ENV_FLAGS` and the matching
base variables above. Choose the command for your run:

```bash
rwrl supervise --startup-s 60 --log runs/hil-pick-and-place/eval-supervise.jsonl -- \
    rwrl train hil_serl --role eval $ENV_FLAGS --exp-name hil-pick-and-place \
    --load-checkpoint runs/hil-pick-and-place/checkpoints/final.pt --eval-n-trajs 10
```

On the NUC, repository root, project environment active:

```bash
rwrl supervise --startup-s 60 --log runs/residual-bc-pick-and-place/eval-supervise.jsonl -- \
    rwrl train residual_hil_serl --role eval $ENV_FLAGS $RESIDUAL \
    --exp-name residual-bc-pick-and-place --base-provider "$BASE_BC" \
    --load-checkpoint runs/residual-bc-pick-and-place/checkpoints/final.pt --eval-n-trajs 10
```

On the NUC, repository root, project environment active:

```bash
rwrl supervise --startup-s 60 --log runs/residual-act-pick-and-place/eval-supervise.jsonl -- \
    rwrl train residual_hil_serl --role eval $ENV_FLAGS $RESIDUAL \
    --exp-name residual-act-pick-and-place --base-provider "$BASE_ACT" \
    --load-checkpoint runs/residual-act-pick-and-place/checkpoints/final.pt --eval-n-trajs 10
```

On the NUC, repository root, project environment active:

```bash
rwrl supervise --startup-s 60 --log runs/silri-pick-and-place/eval-supervise.jsonl -- \
    rwrl train silri --role eval $ENV_FLAGS --exp-name silri-pick-and-place \
    --load-checkpoint runs/silri-pick-and-place/checkpoints/final.pt --eval-n-trajs 10
```

Keep GELLO handed back (`h`) or SpaceMouse button-0 takeover off for autonomous
episodes. `s`/`f` label outcomes; `q` exits. A takeover makes the episode assisted.
A periodic `checkpoint_<update>.pt` evaluates the same way. *You should see*
`[eval] episode=... task_success=... autonomous_success=... return=...` per episode and a final
`[eval] ...` line with `autonomous_success_rate` (successes without a takeover over completed
episodes), `task_success_rate`, the intervention rates and `mean_return`.

<a id="what-to-watch"></a>

## 8. Watch the metrics

On the learner machine, repository root, project environment active, view the logs
from `--log-type` (TensorBoard by default) under `runs/`:

<!-- tutorial-test: skip -->
```bash
tensorboard --logdir runs
```

*You should see:* the metrics below as rows arrive and updates run. If there are no events,
check the run directory and `--log-type`; for device errors use [If it fails](#if-it-fails).

**The x axis of every metric is the number of transitions the learner has received**, not wall
time or updates. Learner metrics are logged on `--log-freq` boundaries (default 1000 transitions,
about 20 s at 50 Hz); the actor posts one set of `actor/*` scalars per episode.

| Metric | Methods | What to look for |
|---|---|---|
| `actor/return`, `actor/terminated`, `actor/task_success` | all | per episode: `terminated` is any terminal step (your `s`, or a classifier hit); `task_success` is the explicit success label when one exists |
| `actor/autonomous_success_rate`, `actor/task_success_rate`, `actor/episode_intervention_rate` | all | cumulative since the actor started: successes without any takeover over completed episodes should rise, the intervention rates fall |
| `actor/intervention_rate`, `actor/intervened_steps`, `actor/intervention_count` | all | should fall as the policy improves |
| `actor/length`, `actor/human_episode_end` | all | episodes you ended with Esc |
| `actor/safety_clips`, `actor/safety_clips_total` | all | servo commands the daemon clipped; a rising count means the policy keeps asking for targets outside the limits |
| `actor/rtt_ms_mean/max`, `actor/step_time_ms_mean/max`, `actor/param_refresh_ms_mean/max` | all | timing |
| `actor/outbox_depth`, `actor/upload_retries` | all | rows waiting on the actor's disk and failed upload attempts; a growing depth means uploads are pending; retries preserve IDs, while full capacity or disk failures stop collection |
| `learner/received`, `learner/update`, `learner/buffer_size`, `learner/train_time_s` | all | ingestion and training progress |
| `learner/demo_buffer_size`, `learner/intervened_received` | all | the demo buffer grows from human-supervised, nonsyncing rows |
| `losses/critic_loss`, `losses/qf_loss`, `losses/actor_loss`, `losses/alpha_loss`, `entropy/alpha`, `entropy/target_entropy`, `q/predicted` | `hil_serl`, `residual_hil_serl` | SAC-style losses; `q/predicted` should track the returns |
| `losses/actor/residual_bc`, `losses/actor/intervened_fraction` | `residual_hil_serl` | the BC term and the share of intervened rows in a batch (after the critic-only updates) |
| `losses/critic/loss`, `losses/actor/loss`, `losses/actor/q`, `losses/actor/distance`, `losses/lambda/loss`, `losses/lambda/mean`, `losses/behavior/sigma_sum`, `losses/batch/intervened_fraction`, `losses/discrete/ce` | `silri` | `losses/actor/distance` against `losses/behavior/sigma_sum` + margin: where the actor is held to your behaviour |
| `behavior/pretrained`, `behavior/pretrain/*`, `behavior/refresh/*`, `behavior/refresh_count` | `silri` | the behaviour model's pretraining and refreshes |

Also on disk: the actor's `outbox.sqlite3`; the learner's `checkpoints/journal.sqlite3`;
`bundle_manifest.json` pointing to one complete `bundle_<update>_<id>/` generation; and legacy
`checkpoint_<update>.pt` files. Bundles carry model/optimizers, both replay buffers and the journal
cursor. Legacy `buffer/` and `demo_buffer/` pkl snapshots remain fallback sources; they are not
loaded again on top of an authoritative bundle.

## Optional: a reward classifier

The main preset has no RGB cameras, so it cannot use a wrist-image classifier.
For a separate image experiment, configure cameras and image observations before
recording demos, training the base and running HIL. Keep that image contract
consistent through all stages. Frame collection and classifier training are in
[Real-world experiments](../real-world-experiments.md) and
[teleop.md](../teleop.md#recording-demos-on-the-real-robot), including
terminal outcome labels. Classifier collection on physical data remains to be
validated. With keys and a classifier, the larger reward wins.

## 9. Resume an interrupted run

Keep the learner checkpoint directory, its journal (including SQLite WAL/SHM files) and bundle
generations together. Restart the learner first with the same preset, experiment name, seed-demo
paths and capacities. For the `hil_serl` run above:

On the GPU host, repository root, project environment active, start the learner:

```bash
rwrl train hil_serl --role learner $COMMON --exp-name hil-pick-and-place \
    --demo-dataset-paths demos/pick-and-place.pkl --load-checkpoint \
    runs/hil-pick-and-place/checkpoints/bundle_manifest.json --checkpoint-dir \
    runs/hil-pick-and-place/checkpoints
```

*You should see:* the learner start and wait for actor data. If it exits, check the named
path or contract difference before starting the actor.

On the NUC, repository root, project environment active, start the actor:

```bash
rwrl supervise --startup-s 60 --log runs/hil-pick-and-place/supervise.jsonl -- \
    rwrl train hil_serl --role actor $COMMON --exp-name hil-pick-and-place
```

*You should see:* the actor receive a policy snapshot, reset and begin episodes. If startup
fails, use [If it fails](#if-it-fails) before another reset.

The actor reuses its existing outbox; acknowledged IDs are admitted once. A learner/network outage
retries with back-off while the outbox has capacity. For residual or SiLRI, use that method's
original arguments and its matching manifest/directory. A manifest alone does not relocate the
journal. Bundle restore always restores the complete replay/cursor unit, regardless of
`--load-replay-buffer`; a standalone generation `agent.pt` is the weights-only entry for a new run.
Use a new `--exp-name`/`--run-id` or independent outbox for an independent experiment.

*You should see:* the restored learner ingest the actor’s pending outbox rows and resume
updates. If restore fails, keep the complete bundle and journal together; do not merge
independent runs.

SQLite uses WAL with `synchronous=NORMAL`: these guarantees cover process crashes, while OS
crashes or power loss can roll back the latest commits. An update exception or interruption during
journal application preserves the last complete bundle, so recovery replays the journal tail.
Full paths, FWBW direction isolation and legacy recovery are in
[Real-world experiments](../real-world-experiments.md).

## Appendix: the env preset and observation settings

These settings derive the local direct-route preset and define the task/image contract.
Configure them before recording and keep them consistent through every stage.

### The env preset (`local/gello_cartesian.yaml`)

Use the copy made from `examples/presets/gello_cartesian.yaml` in
[04](04-first-motion.md#1-home) and [08](08-record-demos.md#1-prepare-the-local-preset-and-home).
It embeds `real_robot.robot` with `arm_backend: franky`; exactly one of `robot` and `remote`
must be set. Do not start a daemon for this direct route.

- `robot` is a mapping, not a path to `station.yaml`. Copy the station’s single-arm robot
  mapping, excluding its `daemon` block, and keep hardware fields synchronized.
- Use `real_robot.action_mode: cartesian_delta`, `control_hz: 50`, and include `tcp_pose` in
  `state_fields`. Raw GELLO joints need the `leader_cartesian` source to drive this mode.
- Keep `obs.rgb: []` for the state-only tutorial. For an image experiment, configure camera
  names (for example `[wrist]`) and `obs.image_size: [128, 128]` before recording new demos.
- With a Hand, use `gripper_in_action: true`; add `gripper_pos` only if you want it in every
  stage’s observation. Without a Hand, use `robot.gripper: null`, `gripper_in_action: false`
  and omit `gripper_pos`.

On either host, repository root, project environment active, check without opening hardware:

```bash
rwrl train hil_serl --config local/gello_cartesian.yaml --teleop-device leader_cartesian --print-config
```

*You should see:* the resolved config with `env_backend: real_robot`, a `real_robot.robot`
mapping, no remote address, `action_mode: cartesian_delta` and `control_hz: 50`. If not,
check YAML indentation and the copied settings; a config parse does not validate hardware.

On the NUC, repository root, project environment active, with no recorder or actor running:

```bash
rwrl --station local/station.yaml --preset local/gello_cartesian.yaml doctor
```

*You should see:* `arm.preset` PASS when copied hardware fields match. A differing field
instead gives `arm.preset:<field>` WARN with both values. Read WARN rows, fix the copy and
rerun. For red rows use [If it fails](#if-it-fails).

### Task, images and control profiles

The tutorial preset leaves `real_robot.task.name` at `none`: the operator or classifier supplies
success labels. The task/reset, depth and per-camera preprocessing options are described in
[Real-world experiments](../real-world-experiments.md#task-presets-and-observation-settings).
Set them before recording demos and keep them consistent through BC, HIL and deployment.
`preprocess.<camera>.roi` is `[top, left, height, width]` in pixels and requires the actual
`source_resolution: [height, width]`. RGB resizing is bilinear; depth is one-channel metres,
uses nearest-neighbor resizing and encodes invalid values as zero. Changing those settings
changes the policy's observation contract.

Read the actual resolution and pick the ROI from a real frame with `rwrl camera show` and
`rwrl camera grab` ([04 Home, gripper, cameras](04-first-motion.md#3-cameras)).

Station YAML accepts an optional per-arm `control:` impedance profile; `real_robot.control`
can override it for an experiment. Configure while IDLE. Overrides may only tighten declared
torque-slew and error-clip bounds, and the installed SDK must support the requested keywords.
See [Arm control profiles](../control-profiles.md) for fields, effective
settings and the wizard's `--control` option. The wizard preserves existing profiles and
uses `control: null` for a new station. Choose gains for the actual tool and payload.

Use `real_robot.control_hz` as the robot's control frequency. A nondefault global
`--control-hz` can fill an otherwise default robot frequency, but conflicting nondefault
global and environment frequencies are rejected. Record, train and deploy at the resolved
frequency; it is part of the experiment contract.

## If it fails

On the NUC, repository root, project environment active, finish the actor first:

```bash
rwrl --station local/station.yaml doctor
```

Start with the first red row and its hint. See
[Network and FCI](../doctor.md#network-and-fci),
[Serial, input groups and USB latency](../doctor.md#serial-input-groups-and-usb-latency),
[SpaceMouse and HID](../doctor.md#spacemouse-and-hid),
[Python environment and extras](../doctor.md#python-environment-and-extras), or
[Busy devices](../doctor.md#busy-devices-blocked).

- `Unknown env backend 'real_robot'`: install this package in the interpreter
  running the NUC actor.
- Missing leader configuration or failed reads: inspect `gello_leader.yaml`, the
  calibrated local leader-only YAML and serial permissions. A failed teleop read
  stops the actor rather than silently removing an intervention.
- `WORKSPACE HOLD`: use the recovery sequence on the [operator page](../intervening.md#how-the-robot-follows-you).
- A difference naming `experiment.teleop_control`: restore the recording's
  device, tracker version, lead caps and target speeds. Check other named
  action/observation fields too.
- The supervisor escalates before startup completes: inspect its log and startup
  allowance; a running loop that stops heartbeating is a separate failure.
- `--base-provider is required`: every residual role needs the base spec,
  including the learner.

Next: [11 Deploy](11-deploy.md).
