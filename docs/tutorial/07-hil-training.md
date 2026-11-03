# 7. HIL training

Three methods share the same actor, learner and eval roles, teleop flags, demo pkl and sync
protocol. Run one at a time, each with its own `--exp-name`:

| Method | What it adds | Concepts |
|---|---|---|
| `hil_serl` | RLPD with human interventions, a growing demo buffer and a DQN gripper head | [README](../../README.md#running-on-hardware), [hil-serl-roadmap.md](../hil-serl-roadmap.md) |
| `residual_hil_serl` | a bounded residual on top of a frozen base policy (BC or ACT from [05](05-base-policy.md)) | [residual.md](../residual.md) |
| `silri` | an actor pulled towards a learned model of your interventions | [silri.md](../silri.md) |

Processes and where they run: the daemon on the robot PC ([03](03-daemon.md), `--watchdog-s 5`), the
learner on the GPU machine, the actor on the actor machine. The learner binds `--sync-host` and the
actor connects to it, so both get the learner's address. Start the learner first. Read
[06 Intervening](06-intervening.md) before the first actor run.

```bash
ENV_FLAGS="--config hil.yaml --teleop-device leader_arm --robot-teleop-action-mode joint_delta \
    --teleop-event-device keyboard --reward-from-teleop-events"
COMMON="$ENV_FLAGS --sync-host LEARNER_IP --sync-port 6000 --log-dir runs \
    --checkpoint-freq 1000 --buffer-size 100000"
```

- `--reward-from-teleop-events`: your `s` key is the reward (1, terminal) and `f` ends an episode as
  a failure. Training does not turn it on by default; without it, and without
  `--classifier-checkpoint`, every transition carries reward 0 and the actor and learner warn
  ([06](06-intervening.md#where-the-reward-comes-from)).
- `--checkpoint-freq` (default 0: only `final.pt` when the learner stops) writes
  `checkpoint_<update>.pt` every that many learner updates.
- `--buffer-size` is the replay capacity (defaults: 200000 for `hil_serl` and `residual_hil_serl`,
  30000 for `silri`), held on `--buffer-device cuda`; with several 128 x 128 cameras size it to your
  GPU memory or use `--buffer-device cpu`.
- The learner ignores the teleop flags (it never opens a device).

## hil_serl

```bash
python examples/train_real_world.py hil_serl --role learner $COMMON --exp-name hil-pick-and-place \
    --demo-dataset-paths demos/pick-and-place.pkl
python examples/train_real_world.py hil_serl --role actor $COMMON --exp-name hil-pick-and-place
```

The learner loads the demos into its demo buffer, waits for `--learning-starts` online transitions
(default 4000; the preloaded demos do not count), then trains and publishes weights every
`--publish-freq` updates (default 100). Until the first weights arrive the actor acts with an
untrained policy: be ready to take over.

## residual_hil_serl, BC base

```bash
RESIDUAL="--residual-variant hybrid --residual-action-scale 0.1 --residual-bc-weight 0.1 \
    --critic-only-updates 500"
BASE_BC='checkpoint:runs/bc-pick-and-place/checkpoints/final.pt?algo=real_world_bc'
python examples/train_real_world.py residual_hil_serl --role learner $COMMON $RESIDUAL \
    --exp-name residual-bc-pick-and-place --base-provider "$BASE_BC"
python examples/train_real_world.py residual_hil_serl --role actor $COMMON $RESIDUAL \
    --exp-name residual-bc-pick-and-place --base-provider "$BASE_BC"
```

## residual_hil_serl, ACT base

```bash
RESIDUAL="--residual-variant hybrid --residual-action-scale 0.1 --residual-bc-weight 0.1 \
    --critic-only-updates 500"
BASE_ACT='act:runs/act-pick-and-place/checkpoints/final.pt?temporal_agg=1'
python examples/train_real_world.py residual_hil_serl --role learner $COMMON $RESIDUAL \
    --exp-name residual-act-pick-and-place --base-provider "$BASE_ACT"
python examples/train_real_world.py residual_hil_serl --role actor $COMMON $RESIDUAL \
    --exp-name residual-act-pick-and-place --base-provider "$BASE_ACT"
```

The actor executes `clamp(base + scale * residual, -1, 1)`; your interventions override the total
action. The base must come from the same `hil.yaml` (same observation and action spaces, the
[contract of 05](05-base-policy.md#the-act-checkpoint-contract)). The base spec is quoted: `?` is a
glob character and `&` backgrounds. Only the actor and eval build the base; the learner trains on the
base actions the actor stores. With the ACT base the actor replans the chunk after every intervened
step ([resync](05-base-policy.md#resync-after-an-intervention)).

- `--residual-variant hybrid` puts the residual on the arm dims and gives the gripper `hil_serl`'s
  DQN head; `continuous` puts it on every dim and keeps the base's gripper.
- `--residual-action-scale` bounds the residual (0.1 of the normalised action per step).
- `--critic-only-updates 500`: the first 500 learner updates train the critic only.
- `--residual-bc-weight 0.1` pulls the residual towards your corrections on intervened rows.

Teleop recordings carry no base actions, so the demo pkl is not passed with `--demo-dataset-paths`
here (the learner rejects it unless you add `--allow-base-less-demos`, which learns its rows as
residuals from a zero base, see [residual.md](../residual.md#human-intervention)).

## silri

```bash
python examples/train_real_world.py silri --role learner $COMMON --exp-name silri-pick-and-place \
    --demo-dataset-paths demos/pick-and-place.pkl
python examples/train_real_world.py silri --role actor $COMMON --exp-name silri-pick-and-place
```

Before its first update the SiLRI learner pretrains its human-behaviour model on the demo buffer
(`--bc-pretrain-steps`, default 500) and blocks ingestion while it does; it then refreshes the model
as your interventions arrive ([silri.md](../silri.md#data-flow)).

## Stopping

Stop the actor with Ctrl-C (the arm stays where it is), then the learner: it writes
`runs/<exp-name>/checkpoints/final.pt` on the way out. `runs/<exp-name>/config.json` is what eval,
deploy and the residual base provider read.

## Evaluate

Deterministic episodes on the robot, no learner, no data collection; success is a terminated episode
(your `s` with `--reward-from-teleop-events`, or the classifier). Stop the actor first, and do not
intervene ([06](06-intervening.md#the-sequence)):

```bash
python examples/train_real_world.py hil_serl --role eval $ENV_FLAGS --exp-name hil-pick-and-place \
    --load-checkpoint runs/hil-pick-and-place/checkpoints/final.pt --eval-n-trajs 10
python examples/train_real_world.py residual_hil_serl --role eval $ENV_FLAGS $RESIDUAL \
    --exp-name residual-bc-pick-and-place --base-provider "$BASE_BC" \
    --load-checkpoint runs/residual-bc-pick-and-place/checkpoints/final.pt --eval-n-trajs 10
python examples/train_real_world.py residual_hil_serl --role eval $ENV_FLAGS $RESIDUAL \
    --exp-name residual-act-pick-and-place --base-provider "$BASE_ACT" \
    --load-checkpoint runs/residual-act-pick-and-place/checkpoints/final.pt --eval-n-trajs 10
python examples/train_real_world.py silri --role eval $ENV_FLAGS --exp-name silri-pick-and-place \
    --load-checkpoint runs/silri-pick-and-place/checkpoints/final.pt --eval-n-trajs 10
```

A periodic `checkpoint_<update>.pt` evaluates the same way. *You should see*
`[eval] episode=... success=... return=...` per episode and a final `[eval] success_rate=...` line.

## What to watch

The learner logs to `--log-type` (TensorBoard by default) under `runs/`:

<!-- tutorial-test: skip -->
```bash
tensorboard --logdir runs
```

**The x axis of every metric is the number of transitions the learner has received**, not wall
time or updates. Learner metrics are logged on `--log-freq` boundaries (default 1000 transitions,
about 100 s at 10 Hz); the actor posts one set of `actor/*` scalars per episode.

| Metric | Methods | What to look for |
|---|---|---|
| `actor/return`, `actor/success` | all | rising success rate: the success is a terminated episode (your `s`) |
| `actor/intervention_rate`, `actor/intervened_steps`, `actor/intervention_count` | all | should fall as the policy improves |
| `actor/length`, `actor/human_episode_end` | all | episodes you ended with Esc |
| `actor/safety_clips`, `actor/safety_clips_total` | all | servo commands the daemon clipped; a rising count means the policy keeps asking for targets outside the limits |
| `actor/rtt_ms_mean/max`, `actor/step_time_ms_mean/max`, `actor/param_refresh_ms_mean/max`, `actor/dropped_transitions` | all | timing and network health |
| `learner/received`, `learner/update`, `learner/buffer_size`, `learner/train_time_s` | all | ingestion and training progress |
| `learner/demo_buffer_size`, `learner/intervened_received` | all | the demo buffer grows only while you intervene |
| `losses/critic_loss`, `losses/qf_loss`, `losses/actor_loss`, `losses/alpha_loss`, `entropy/alpha`, `entropy/target_entropy`, `q/predicted` | `hil_serl`, `residual_hil_serl` | SAC-style losses; `q/predicted` should track the returns |
| `losses/actor/residual_bc`, `losses/actor/intervened_fraction` | `residual_hil_serl` | the BC term and the share of intervened rows in a batch (after the critic-only updates) |
| `losses/critic/loss`, `losses/actor/loss`, `losses/actor/q`, `losses/actor/distance`, `losses/lambda/loss`, `losses/lambda/mean`, `losses/behavior/sigma_sum`, `losses/batch/intervened_fraction`, `losses/discrete/ce` | `silri` | `losses/actor/distance` against `losses/behavior/sigma_sum` + margin: where the actor is held to your behaviour |
| `behavior/pretrained`, `behavior/pretrain/*`, `behavior/refresh/*`, `behavior/refresh_count` | `silri` | the behaviour model's pretraining and refreshes |

Also on disk: `checkpoint_<update>.pt` in `runs/<exp-name>/checkpoints/` every `--checkpoint-freq`
updates, and the crash-recovery snapshots next to them, `buffer/transitions_<n>.pkl` (every
transition) and `demo_buffer/transitions_<n>.pkl` (your intervened steps only), one pair per
`--buffer-period` received transitions (default 1000). A restarted learner with the same
`--exp-name` reloads them.

## Optional: a reward classifier

Instead of (or as well as) the `s` key, a classifier on the wrist image can give the reward. Neither
step has been exercised on real data yet. Collect labelled frames with the daemon running: drive with
the GELLO as usual, press `s` when the object is in the bin and move around inside the success region,
`f` when leaving it (`s` latches success labels until `f` or the episode end):

```bash
python -m rl_garden_real_world.reward.collect_data --env-backend real_robot --config hil.yaml \
    --teleop-device leader_arm --robot-teleop-action-mode joint_delta --teleop-event-device keyboard \
    --output-dir classifier_data --exp-name pick-and-place --successes-needed 200
python -m rl_garden.models.reward.success.train --success_paths 'classifier_data/*success*.pkl' \
    --failure_paths 'classifier_data/*failure*.pkl' --output_dir runs/classifier-pick-and-place
```

The trainer's frozen ResNet needs its pretrained weights (`--pretrained_weights`, see
`rl-garden/rl_garden/encoders/resnet.py`). Then add to `ENV_FLAGS`
`--classifier-checkpoint runs/classifier-pick-and-place/success_classifier.pt --classifier-image-keys rgb_wrist`
(`--classifier-threshold`, default 0.5). With both reward sources the larger reward wins.

## If not

- `DeviceFault: the daemon watchdog stopped the arm ...` on the first step after a reset: the
  daemon's `--watchdog-s` is too small.
- `Unknown env backend 'real_robot'`: the `rlgarden-real-world` package is not installed.
- `TransportError: robot daemon unreachable at <addr>`: the daemon is not running or `remote` in
  `hil.yaml` is wrong.
- The actor stops with a `DeviceFault` from the teleop: the GELLO read failed (it is never silently
  dropped).
- `ValueError: ... recorded action contract does not match the env's`: the demo pkl was recorded with
  a different preset.
- `--base-provider is required`: every `residual_hil_serl` role needs it, the learner included.

Next: [08 Deploy](08-deploy.md).
