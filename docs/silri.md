# HIL-RL (SiLRI)

`silri` trains a real-robot policy from human interventions that may be suboptimal, after
"Real-world Reinforcement Learning from Suboptimal Interventions" (HIL-RL, arXiv 2512.24288)
and its reference implementation (github.com/nuomizai/HIL-RL, policy `silri`). The policy is an
off-policy actor-critic. Its actor is pulled towards a learned model of the human's actions,
but only as far as the human is consistent: a state-wise Lagrange multiplier enforces a
distance constraint whose tolerance grows with the spread of the human model. The hardware setup
and a first run are walked through in
[tutorial 07](tutorial/07-hil-training.md#silri) (two arms: [bimanual.md](bimanual.md#two-arms-the-tutorial-steps)).

Code: `real_world_rl/policies/silri_policy.py` (`SiLRIPolicy`),
`real_world_rl/algorithms/silri.py` (`SiLRI`), `real_world_rl/loops/silri.py`
(`SilriActorLoop`, `SilriLearnerLoop`), `real_world_rl/training/entrypoints/silri.py`
(`SilriArgs`, `build_silri`, `run_silri`).

## Method

Networks (`SiLRIPolicy`, built on rl-garden's `DDPGPolicy`):

| Module | What it is | Trained by |
|---|---|---|
| actor `mu_pi(s)` | own encoder, MLP, `tanh` mean over the continuous action dims | `actor_optimizer` |
| `actor_target` | Polyak copy of the actor head; reads the actor's encoder | Polyak, `tau` |
| critic `Q_1, Q_2` | own encoder, twin Q heads over the continuous dims | `q_optimizer` |
| `critic_target` | Polyak copy of the Q heads; reads the critic's encoder | Polyak, `tau` |
| `behavior_model` (beta) | own encoder, MLP, `mu_b = tanh(.)`, `sigma_b = clamp(exp(.), std_min, std_max)` | `behavior_optimizer` |
| `lambda_net` | own encoder, MLP with a zero-initialised output, `softplus` (starts at `log 2`) | `lambda_optimizer` |
| `discrete_head` (`discrete_gripper`) | MLP on the actor's features, logits over open/hold/close (-1/0/+1) | `discrete_optimizer` |

Acting: `a = clamp(mu_pi(s) + N(0, actor_noise_std^2), -1 + 1e-6, 1 - 1e-6)` (no noise with
`--deterministic-actor`), gripper from the `discrete_head` argmax appended last.

Losses, as implemented (`a_cont` = recorded action without the gripper dim, `done` =
`terminated` only, so timeouts bootstrap):

```
a'     = clamp(mu_target(s') + N(0, actor_noise_std^2), -1, 1)
y      = r + (1 - done) * gamma * min_k Q_target_k(s', a')
L_Q    = sum_k MSE(Q_k(s, a_cont), y)
d(s)   = || mu_pi(s) - mu_b(s) ||_2                          (unsquared; beta and lambda held fixed)
L_pi   = mean_s [ (-min_k Q_k(s, mu_pi(s)) + lambda(s) * d(s)) / (1 + lambda(s)) ]
L_lam  = -mean_s [ lambda(s) * (d(s) - sum_j sigma_b,j(s) - constraint_margin) ]
L_beta = -mean_{intervened} log N(a_cont; mu_b(s), diag(sigma_b(s)^2))
L_g    =  mean_{intervened} CE(gripper logits(s), gripper label)
L_bc   = -mean_{intervened} log N(a_cont; mu_pi(s), actor_noise_std^2 I)          (pretraining only)
```

`L_lam` is descended, which raises `lambda(s)` where the actor is farther from the human mean
than `sum_j sigma_b,j(s) + constraint_margin` and lowers it elsewhere. `L_pi` and `L_g` are
backpropagated together (the actor and gripper optimizers step on one backward; only the
actor's gradients are clipped, as in the reference).

Schedule of `SiLRI.train(n)`: for each of the `n` gradient steps, `utd - 1` critic updates on
fresh batches, each followed by a Polyak update of both targets; then one more fresh batch with
a critic update, the actor (+ gripper) update, the multiplier update and a Polyak update of both
targets. Every batch is `batch_size` rows, a `demo_data_ratio` share from the demo buffer.
`global_update` advances once per gradient step. The behaviour model is not trained by `train`.

## Data flow

```
actor (robot PC)                                    learner (GPU box)
SilriActorLoop                                      SilriLearnerLoop
  policy.predict + teleop wrapper  -- transitions -->  every row -> online buffer (buffer_size)
  holds each transition one step;                     intervened rows (+ onset rows with
  tags intervention_onset on the                      --onset-to-demo-buffer) -> demo buffer
  row before a takeover                               (demo_buffer_size)
                                   <-- policy params -- publish every publish_freq iterations
```

- Pushed fields: `obs`, `next_obs` (explicit, the pre-reset observation at an episode end),
  `action` (the executed one, the human's while intervening), `reward`, `done = terminated`,
  `intervened`, `policy_action` (what the policy proposed) and `intervention_onset`.
- `intervention_onset` is set on the last policy transition before an intervened one, within
  an episode. The actor flushes its held transition when the episode ends; one still held when
  the actor stops is pushed then, untagged.
- Both buffers are `InterventionReplayBuffer`s storing `intervened`, `policy_actions` and
  `intervention_onset`. Onset rows routed to the demo buffer keep `intervened=False`, so they
  never train the behaviour model or the gripper head.
- Training starts once `learning_starts` transitions were received and the demo buffer holds
  `demo_learning_starts` rows.
- Pretraining: before the first `train` of a learner process, if the agent is not yet
  `pretrained` and the demo buffer holds data, `bc_pretrain_steps` steps of `L_beta` then
  `L_bc + L_g` on demo-only batches, then `actor_target <- actor`. The `pretrained` flag is
  checkpointed, so a resumed run does not pretrain again.
- Behaviour refresh: every `behavior_refresh_every` demo-buffer additions, `behavior_refresh_steps`
  steps of `L_beta` on demo-only batches, run in the training thread right before the next
  `train`. Pretraining resets the addition counter.
- Pretraining and refreshes may hold the training buffer lock. HTTP admission commits to the
  durable journal independently; the training thread applies queued rows afterwards. Actor
  retries retain message IDs, so missed ACKs do not duplicate admitted rows.
- Demo pkls (`--demo-dataset-paths`) and crash-recovery snapshots load as in hil_serl. Demo
  pkls without SiLRI's fields load as intervened, non-onset rows whose policy action is the
  recorded action.
- One learner iteration calls `train(train_freq)`: `utd` is the number of critic updates inside
  each gradient step, not a multiplier of gradient steps (unlike `LearnerLoop`'s default).

## Hyperparameters

| Flag | Default | Reference (Franka) | Meaning |
|---|---|---|---|
| `--batch-size` | 256 | 256 (128 online + 128 demo) | rows per batch |
| `--demo-data-ratio` | 0.5 | 0.5 | demo share of each batch |
| `--gamma` | 0.97 | 0.97 | discount |
| `--tau` | 0.005 | 0.005 / 0.005 | Polyak rate, critic and actor targets |
| `--utd` | 2 | 2 | critic updates per gradient step |
| `--actor-lr` / `--critic-lr` | 3e-4 / 3e-4 | 3e-4 / 3e-4 | actor (and gripper head) / critic |
| `--behavior-lr` | 3e-4 | 3e-4 (`actor_lr`) | behaviour model |
| `--lambda-lr` | 3e-6 | 3e-6 (`0.01 * critic_lr`) | multiplier |
| `--grad-clip-norm` | 40.0 | 40.0 | every update |
| `--actor-noise-std` | 0.05 (std) | 0.05 used as a variance (std 0.224) | sampling noise, actor and TD target |
| `--constraint-margin` | 0.2 | 0.2 | tolerance offset |
| `--std-min` / `--std-max` | 1e-5 / 2.0 | -5 / 2 (bounds of a variance) | behaviour std clamp |
| `--bc-pretrain-steps` | 500 | 500 | pretraining steps |
| `--behavior-refresh-steps` / `--behavior-refresh-every` | 50 / 50 | 50 / 50 | refresh length / cadence (0 disables) |
| `--learning-starts` / `--demo-learning-starts` | 10 / 10 | 10 / 10 | buffer sizes before training |
| `--buffer-size` / `--demo-buffer-size` | 30000 / 10000 | 30000 / 10000 | capacities |
| `--onset-to-demo-buffer` | false | routed (see S4) | copy onset rows to the demo buffer |
| `--feature-dim` / `--hidden-dim` | 256 / 256 | MLP [256, 256] | actor/critic trunk and hidden widths |
| `--behavior-hidden-dims` / `--lambda-hidden-dims` | 256 256 / 256 256 | [256, 256] | MLP widths (Linear, LayerNorm, SiLU) |
| `--discrete-gripper` / `--discrete-hidden-dim` | true / 256 | `num_discrete_actions` | gripper head |
| `--encoder.*` | plain_conv, random shift | frozen ResNet10, random shift | image encoder (one per network) |

## Deviations from the reference

| # | Reference | Here | Why |
|---|---|---|---|
| S1 | The actor's fixed noise 0.05 is passed where a variance is expected, so it samples with std 0.224 | `actor_noise_std=0.05` is a standard deviation (the paper's value) | a bug in the reference; the paper's intent |
| S2 | The tolerance sums the behaviour model's variances: `sum_j v_b,j + 0.2` | `sum_j sigma_b,j + constraint_margin` with standard deviations, margin 0.2 | consistent with S1; the formula's structure (unsquared L2 distance, sum plus margin) follows the code, not the paper's `kappa * sigma + c` |
| S3 | The critic target reads the online critic's encoder; `actor_target` reads the actor's encoder | the same: rl-garden's critic target is a lagged copy of the Q heads over the online critic encoder, and `actor_target` shares the actor encoder | rl-garden's target convention, which coincides with the reference here |
| S4 | The transition before an intervention onset goes to the demo buffer flagged as human, so the behaviour model and BC train on a policy action | it is tagged `intervention_onset` and stays in the online buffer; `--onset-to-demo-buffer` copies it to the demo buffer, still as a policy row | onset rows are a policy action, not a human one |
| S5 | Pretrained weights are cached in a directory shared across runs and reused | no cache; each run pretrains (resumes keep the checkpointed `pretrained` flag) | a stale cache from another task or seed would be reused silently |
| S6 | `next_state` of a row is the next stored row (`optimize_memory`) | explicit `next_obs` per transition | wrong targets at intervention bursts and truncated episodes |
| S7 | The actor optimizer is built from `generator + list` (raises) | plain Adam per network; the gripper head has its own Adam at `actor_lr` stepped on the same backward as the actor | the reference cannot build its optimizers |
| S8 | Recorded demos pair each action with the next observation | demos come from the actor loop or pkls of the same transition format, aligned | |

Other differences:

- The multiplier loss uses the distance computed for the actor loss, before the actor update;
  the reference recomputes it after the actor update.
- The gripper head predicts open/hold/close (-1/0/+1, rl-garden's hybrid gripper mapping) instead
  of the reference's two classes, and its cross-entropy is averaged over the intervened rows of the
  batch (zero when there are none, where the reference yields NaN).
- A behaviour refresh runs once per training iteration when due, however many multiples of
  `behavior_refresh_every` arrived since the last one.
- MLP blocks are rl-garden's (`create_mlp` with LayerNorm and SiLU for the behaviour model and the
  multiplier; `DDPGPolicy`'s trunk and ReLU heads for the actor and critic). The image encoders are
  rl-garden's (`--encoder.*`), not a frozen pretrained ResNet10.
- No `torch.compile` of the critics.

## CLI

The full commands (learner, actor and eval, the env preset and the teleop flags) are in
[tutorial 07](tutorial/07-hil-training.md#silri); deployment is in
[tutorial 08](tutorial/08-deploy.md) (`--algo silri`: the `silri` entry of rl-garden's online
registry rebuilds the agent). The minimal shape, for a GELLO served by the robot daemon on a
`joint_delta` env:

```bash
python examples/train_real_world.py silri --role learner --config hil.yaml --demo-dataset-paths 'demos/*.pkl'
python examples/train_real_world.py silri --role actor --config hil.yaml --teleop-device leader_arm \
    --robot-teleop-action-mode joint_delta --teleop-event-device keyboard --reward-from-teleop-events
```

Evaluation (`--role eval --load-checkpoint ...`) uses deterministic actions and explicit task
outcomes. Takeover remains available; `autonomous_success_rate` excludes action-assisted
successes while retaining all completed episodes in its denominator.

## Limitations

- Real-world loops only: `SiLRI.learn` raises; there is no simulator training path.
- `--fwbw` and `--use-grasp-penalty` are rejected (no reset-free variant, no discrete critic).
- The learner publishes the full policy state: the actor receives the behaviour model, the
  multiplier and the targets as well, though it only needs the actor and the gripper head; a
  filtered publish is future work.
- Pretraining (500 steps by default) and refreshes defer replay application; durable HTTP journal admission continues.
- Grippers: the env action ends with one gripper dim per arm (the action contract's layout), and
  the gripper head classifies all of them jointly over `3 ** num_grippers` classes, labelled as
  rl-garden's `encode_gripper_labels` does; a single-arm env has one, the last dim.


## Human masks and recovery

Behavior fitting and actor pretraining supervise only actual human dimensions. Continuous NLL
uses `D / k * sum(mask * NLL)` per supervised row: `D` is the full continuous width and `k` the
observed human width. Empty rows contribute no gradient, and full masks recover the original
objective. The joint gripper head uses marginal probability over observed human grippers;
unobserved gripper choices are summed out. The online behavior-prior distance and multiplier
constraint keep their full-continuous-action meaning. Syncing and idle-hold supplements are
excluded from imitation. Continuous no-gripper configurations use `--no-discrete-gripper`.

The durable outbox preserves SiLRI's held predecessor, and bundle recovery restores both replay
buffers, ingestion cursor, pretraining state and refresh counters. A recovered held row is
flushed without inventing an onset successor. Paths and process-crash/power-loss limits are in
[Real-world experiments](real-world-experiments.md).
