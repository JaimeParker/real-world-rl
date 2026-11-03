# Residual RL with human intervention (`residual_hil_serl`)

`residual_hil_serl` trains a bounded correction on top of a frozen base policy (a BC
policy, an earlier RL checkpoint or an ACT model) on the real robot, with HIL-SERL's
actor / learner split, demo buffer and human intervention. The hardware setup, the BC and
ACT base policies and a first residual run are walked through in the tutorial:
[05 Base policy](tutorial/05-base-policy.md) and
[07 HIL training](tutorial/07-hil-training.md#residual_hil_serl-bc-base) (two arms:
[bimanual.md](bimanual.md#two-arms-the-tutorial-steps)).

## Method

The base policy proposes `a_base` (a normalized env action in `[-1, 1]`). The actor
network sees the observation features concatenated with `a_base` (its continuous part)
and outputs a unit residual `u` in `[-1, 1]`. The executed action is

```
a = clamp(a_base + scale * u, -1, 1)        (per dim)
```

The critic scores the composed action `a`, as it would score any env action. The SAC
actor loss uses `a` for the Q term and the log-density of `u` for the entropy term; the
`log(scale)` Jacobian of the composition is not added, so the entropy target applies to
the unit residual: `--target-entropy auto` (the default) resolves to `-A_cont`, minus the
residual actor's dimension (the full action width for `continuous`, the arm width for
`hybrid`, e.g. -6 for a 7-D action), not to `-A_env` as in plain SAC.

With `--residual-zero-init` (default) the mean head starts at zero: the deterministic
residual is exactly 0 and the first policy acts like the base. The log-std head starts
state-independent at `--residual-log-std-init` (default `-3.0`, std about 0.05 in unit
residual space).

The learner never loads the base policy. The actor stores the base action of `obs` and of
`next_obs` (the pre-reset observation at an episode end) with every transition; the actor
loss conditions on the stored `base_actions` and the TD target on `next_base_actions`.

## Variants

| `--residual-variant` | Algorithm | Residual on | Gripper |
|---|---|---|---|
| `continuous` (default) | `ResidualRLPD` | every action dim; scale `--residual-action-scale` on the arm dims, `--residual-gripper-scale` (default 0) on the gripper | base gripper plus its (default zero) residual |
| `hybrid` | `ResidualRLPDHybrid` | the arm dims | `RLPDHybrid`'s DQN head; the base's gripper is ignored |

When grippers are included, both variants treat the last `num_grippers` action dims as
one gripper per arm after the arm blocks. Continuous residual also accepts zero-gripper
contracts; hybrid requires at least one gripper and rejects zero before rollout.
`--use-grasp-penalty` needs the hybrid variant.
Both variants keep the growing demo buffer (intervened transitions, mixed at
`--demo-data-ratio`).

## Base action providers

`--base-provider` takes a spec string `kind[:path][?key=value&key=value]`. It is stored in
`config.json` and in the checkpoint metadata (`base_provider`), so eval and deploy rebuild
the same provider.

| Spec | Provider |
|---|---|
| `zero` | `ZeroBaseProvider`: base action 0 (plain RL with scale-bounded actions; tests, ablations) |
| `checkpoint:<path>?algo=<name>[&num_grippers=<k>]` | `CheckpointBaseProvider`: the deterministic action of an rl-garden checkpoint. `algo` is `real_world_bc` or an rl-garden online-registry name with an agent builder (`rlpd`, `rlpd_hybrid`, ...). The run's `config.json` next to `checkpoints/` is read when present; the checkpoint must match the env's observation and action spaces. `num_grippers` (default 1, only for `algo=rlpd_hybrid`) is the gripper count of a multi-arm checkpoint, whose discrete head covers the last `num_grippers` action dims: a two-arm `hil_serl` checkpoint is `checkpoint:<path>?algo=rlpd_hybrid&num_grippers=2` (a spaces-only rebuild cannot infer it; a `real_world_bc` base needs no such option) |
| `act:<path>[?temporal_agg=1&temporal_agg_k=0.01&state_dict_key=ema_agent]` | `ACTBaseProvider`: rl-garden's ACT checkpoint wrapper, de-normalised by the checkpoint statistics, clipped to `[-1, 1]`. Reads `obs["state"]`; `reset()` at episode end (and by default after every intervened step, see below) clears the chunk / ensemble state |

Options are joined with `&`; quote the spec on the shell (`?` globs, `&` backgrounds):

```bash
--base-provider 'act:/data/act/policy_best.ckpt?temporal_agg=1&temporal_agg_k=0.02'
```

Every provider returns `(B, A)` actions in `[-1, 1]` and is computed without gradients.
Providers are built only by the actor, eval and deploy, never by the learner.

`--residual-base-resync-on-intervention` (default on; `--no-residual-base-resync-on-intervention`
turns it off) makes the actor reset the provider after every intervened step, before it
computes the base of that step's next observation. While the human acts, the robot leaves
the trajectory an ACT chunk was planned for: without the resync an open-loop provider
(`temporal_agg=0`) keeps replaying the stale chunk until its next chunk boundary, and an
ensembling one (`temporal_agg=1`) keeps averaging chunks predicted before and during the
takeover, so the stored `base_actions` / `next_base_actions`, the BC target on intervened
rows and the base the policy acts on right after the release lag the robot by up to one
chunk. With the resync the base of every observation after an intervened step is the first
action of a chunk freshly planned at that observation. The step on which the human takes
over keeps the base computed before the takeover. The `zero` and `checkpoint` providers
have no state (`reset()` does nothing), so the flag changes nothing for them.

## Human intervention

The operator overrides the *total* action. Replay stores:

- `action`: the executed action, the human's while intervening. It is stored as is, even
  outside the box `base +/- scale`, and the critic learns its value.
- `policy_action`: what the policy proposed, `base_actions`, `next_base_actions`, and
  `intervened`.

Intervened transitions are also copied into the demo buffer, as in hil_serl.

Pre-collected demos (`--demo-dataset-paths`) are human actions: a transition without
`intervened` / `policy_action` gets `intervened=True` and `policy_action=action`. The
learner cannot recover base actions, so a demo pkl without `base_actions` /
`next_base_actions` (hil_serl snapshots, teleop recordings) is rejected with a
`ValueError` naming the file. `--allow-base-less-demos` loads such rows with zero base
actions instead (with one warning): the critic target and the BC term then learn them as
residuals from a zero base, which matches the run only when the base acts near 0 there.
Demo buffers written by a residual run's own learner carry the base actions.

Optional BC term (`--residual-bc-weight`, default 0): on nonsyncing human rows, the unit
residual is pulled towards the human-implied residual, restricted to `human_action_mask`
dimensions with nonzero scale. Hold supplements and empty masks supply no expert loss.
For a legacy full-action intervention, the objective is:

```
L_bc = mean over intervened rows of  sum_d ( u_d - clip((a_human_d - a_base_d) / scale_d, -1, 1) )^2
```

summed over the dims with a non-zero scale (the denominator is guarded when a batch has no
intervened row). On its `--log-freq` boundaries the learner logs, besides the SAC losses
(`losses/critic_loss`, `losses/actor_loss`, `entropy/alpha`, `q/predicted`, ...), this term as
`losses/actor/residual_bc` and the batch's share of intervened rows as
`losses/actor/intervened_fraction` (both only on updates that train the actor, so not during
`--critic-only-updates`). The actor's per-episode `actor/intervention_rate` is the share of
intervened steps on the robot ([tutorial 07](tutorial/07-hil-training.md#what-to-watch)).

## Warm-up

- Actor: until a learner snapshot reports `policy_ready`, the policy uses warm-up, so the
  unit residual is `--residual-warmup-unit * U(-1, 1)` (default 0: pure base actions) and
  the gripper is the base's (the hybrid DQN head is untrained too).
- Learner: `--critic-only-updates N` makes the first `N` `train()` calls update the
  critics only (through the update mask: `update_actor=False`); the actor and the
  temperature start afterwards. The count is saved with checkpoints.

## Commands

The full commands (learner, actor and eval with a BC and with an ACT base, the env preset and the
teleop flags) are in [tutorial 07](tutorial/07-hil-training.md#residual_hil_serl-bc-base); deployment
is in [tutorial 08](tutorial/08-deploy.md). The minimal shape, for a GELLO served by the robot daemon
on a `joint_delta` env (every role, the learner included, takes `--base-provider`):

```bash
python examples/train_real_world.py residual_hil_serl --role actor --config hil.yaml \
    --teleop-device leader_arm --robot-teleop-action-mode joint_delta --teleop-event-device keyboard \
    --reward-from-teleop-events --base-provider 'checkpoint:runs/bc/checkpoints/final.pt?algo=real_world_bc'
```

Eval (`--role eval --load-checkpoint ...`) runs the deterministic residual on top of the provider;
success uses explicit task labels, and the primary metric excludes action-assisted episodes. Deploy takes `--algo residual_hil_serl` for the continuous variant and
`residual_hil_serl_hybrid` for the hybrid one (both names rebuild the class the run's `config.json`
names, and both are registered on rl-garden's online registry, one name per algorithm class; the
real-world CLI has `residual_hil_serl` only). The provider comes from the checkpoint metadata;
`--base-provider` replaces it, for example when the base checkpoint moved.

## Limitations

- `nstep` 1 only, in-memory replay only (no `--mmap-dir`, no `--memory-efficient-buffer`),
  one env (one robot), no `--fwbw`.
- No simulator `learn()` path and no `rl-garden-eval`: both call `policy.predict` without a
  base action (`checkpoint_eval_unsupported_reason` says so).
- Demo pkls without base actions need `--allow-base-less-demos`, and their rows are then
  learned as residuals from a zero base (see "Human intervention").
- The checkpoint provider rebuilds an agent from the base run's Args; a base checkpoint
  needs the same observation and action spaces as the residual run.


## Durable operation and partial supervision

Actor outboxes, learner journals and complete replay bundles are shared with the other HIL
methods. Resume using the matching `bundle_manifest.json`, checkpoint directory and actor
identity; retain the base-provider spec and experiment contract. See
[Real-world experiments](real-world-experiments.md) for persistence boundaries, selected-part
interventions and mask-aware imitation. Episode outcomes remain separate from reward and
bootstrap terminal flags.
