# Residual RL with human intervention (`residual_hil_serl`)

`residual_hil_serl` trains a bounded correction on top of a frozen base policy (a BC
policy, an earlier RL checkpoint or an ACT model) on the real robot, with HIL-SERL's
actor / learner split, demo buffer and human intervention.

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

Both treat the last action dim as the gripper, as every HIL env does (the intervention
wrapper requires `gripper_in_action`). `--use-grasp-penalty` needs the hybrid variant.
Both variants keep the growing demo buffer (intervened transitions, mixed at
`--demo-data-ratio`).

## Base action providers

`--base-provider` takes a spec string `kind[:path][?key=value&key=value]`. It is stored in
`config.json` and in the checkpoint metadata (`base_provider`), so eval and deploy rebuild
the same provider.

| Spec | Provider |
|---|---|
| `zero` | `ZeroBaseProvider`: base action 0 (plain RL with scale-bounded actions; tests, ablations) |
| `checkpoint:<path>?algo=<name>` | `CheckpointBaseProvider`: the deterministic action of an rl-garden checkpoint. `algo` is `real_world_bc` or an rl-garden online-registry name with an agent builder (`rlpd`, `rlpd_hybrid`, ...). The run's `config.json` next to `checkpoints/` is read when present; the checkpoint must match the env's observation and action spaces |
| `act:<path>[?temporal_agg=1&temporal_agg_k=0.01&state_dict_key=ema_agent]` | `ACTBaseProvider`: rl-garden's ACT checkpoint wrapper, de-normalised by the checkpoint statistics, clipped to `[-1, 1]`. Reads `obs["state"]`; `reset()` at episode end clears the chunk / ensemble state |

Options are joined with `&`; quote the spec on the shell (`?` globs, `&` backgrounds):

```bash
--base-provider 'act:/data/act/policy_best.ckpt?temporal_agg=1&temporal_agg_k=0.02'
```

Every provider returns `(B, A)` actions in `[-1, 1]` and is computed without gradients.
Providers are built only by the actor, eval and deploy, never by the learner.

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

Optional BC term (`--residual-bc-weight`, default 0): on intervened rows the unit residual
is pulled towards the residual the human action implies,

```
L_bc = mean over intervened rows of  sum_d ( u_d - clip((a_human_d - a_base_d) / scale_d, -1, 1) )^2
```

summed over the dims with a non-zero scale (the denominator is guarded when a batch has no
intervened row). The learner logs `actor/residual_bc` and `actor/intervened_fraction`.

## Warm-up

- Actor: until the first weights arrive from the learner the policy is untrained, so the
  unit residual is `--residual-warmup-unit * U(-1, 1)` (default 0: pure base actions) and
  the gripper is the base's (the hybrid DQN head is untrained too).
- Learner: `--critic-only-updates N` makes the first `N` `train()` calls update the
  critics only (through the update mask: `update_actor=False`); the actor and the
  temperature start afterwards. The count is saved with checkpoints.

## Commands

Learner and actor (as for hil_serl; see the README for the env, teleop and robot flags):

```bash
BASE='checkpoint:runs/bc/checkpoints/final.pt?algo=real_world_bc'   # quoted: '?' is a glob character
COMMON="--config hil.yaml --teleop-device spacemouse --log-dir runs --exp-name residual \
    --residual-variant hybrid --residual-action-scale 0.1 --residual-bc-weight 0.1 \
    --critic-only-updates 500 --sync-host 127.0.0.1 --sync-port 6055"
python examples/train_real_world.py residual_hil_serl --role learner $COMMON --base-provider "$BASE"
python examples/train_real_world.py residual_hil_serl --role actor $COMMON --base-provider "$BASE"
```

Evaluation episodes on the robot (deterministic residual on top of the provider; success
is `terminated`):

```bash
python examples/train_real_world.py residual_hil_serl --role eval $COMMON --base-provider "$BASE" \
    --load-checkpoint runs/residual/checkpoints/checkpoint_<update>.pt --eval-n-trajs 10
```

Deploy (`--algo residual_hil_serl` for the continuous variant, `residual_hil_serl_hybrid`
for the hybrid one; both names rebuild the class the run's `config.json` names). The
provider comes from the checkpoint metadata; `--base-provider` replaces it, for example
when the base checkpoint moved:

```bash
python examples/deploy_real_robot.py --checkpoint runs/residual/checkpoints/checkpoint_<update>.pt \
    --algo residual_hil_serl_hybrid --real-robot.remote ROBOT_PC:50051 --real-robot.token T \
    --base-provider 'checkpoint:/data/bc/final.pt?algo=real_world_bc'
```

Both names are registered on rl-garden's online registry (one name per algorithm class);
the real-world CLI has `residual_hil_serl` only.

## Limitations

- `nstep` 1 only, in-memory replay only (no `--mmap-dir`, no `--memory-efficient-buffer`),
  one env (one robot), no `--fwbw`.
- No simulator `learn()` path and no `rl-garden-eval`: both call `policy.predict` without a
  base action (`checkpoint_eval_unsupported_reason` says so).
- Demo pkls without base actions need `--allow-base-less-demos`, and their rows are then
  learned as residuals from a zero base (see "Human intervention").
- The checkpoint provider rebuilds an agent from the base run's Args; a base checkpoint
  needs the same observation and action spaces as the residual run.
