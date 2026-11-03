# 5. Base policy: BC or ACT

A base policy trained on the demos is useful twice: deployed on its own ([08](08-deploy.md)) it is
the baseline, and in [07](07-hil-training.md) `residual_hil_serl` trains a bounded correction on top
of it. Train either or both, on the GPU machine, from the pkl of [04](04-record-demos.md). Neither
connects to the robot.

| | BC (`real_world_bc`) | ACT |
|---|---|---|
| Model | rl-garden's BC actor, one action per observation | action-chunking transformer (CVAE), predicts `--num-queries` actions per step |
| Trained by | `examples/train_real_world.py real_world_bc` | `python -m rl_garden.models.act.train_trajectories` (in rl-garden) |
| Residual base spec | `checkpoint:<path>?algo=real_world_bc` | `act:<path>?temporal_agg=1` |
| Deploy on its own | yes, `--algo real_world_bc` | no (only as a residual base) |

## BC

`real_world_bc` is offline behaviour cloning over the demo pkls. The `--config` preset only gives it
the env's observation and action spaces.

```bash
python examples/train_real_world.py real_world_bc --config hil.yaml \
    --demo-dataset-paths demos/pick-and-place.pkl \
    --log-dir runs --exp-name bc-pick-and-place --buffer-size 100000 \
    --num-offline-steps 20000 --no-tanh-squash
```

- `--no-tanh-squash`: recorded gripper actions sit exactly at +1 or -1, where the default
  tanh-squashed actor's log-probability is numerically degenerate (see the comment on `tanh_squash`
  in rl-garden's `OfflineBCArgs`).
- `--buffer-size` is the replay capacity the demos are loaded into (default 1000000, on
  `--buffer-device cuda`); keep it at least as large as the number of recorded transitions and size
  it to your GPU memory.
- `--num-offline-steps` (default 100000) is the number of gradient steps; 20000 is an example.
- `--log-freq` (default 1000) is how often the BC losses are logged to `--log-type` (TensorBoard by
  default).

*You should see* a metrics line every `--log-freq` steps; the run is done when the process exits and
`runs/bc-pick-and-place/checkpoints/final.pt` and `runs/bc-pick-and-place/config.json` exist. The
`config.json` next to `checkpoints/` is what deploy and the residual base provider read to rebuild
the policy.

*If not:* `ValueError: ... recorded action contract does not match the env's (differs in [...])`
means the demos were recorded with different settings than the preset (action mode, joint limits,
scales, state fields): record again, or fix the preset. `No demo transitions loaded` means the path
or glob matched nothing.

## ACT

rl-garden trains ACT from the same pkls, without a simulator. With rl-garden installed
(`pip install -e rl-garden`, [01](01-install.md#install)) run it from the repository root:

```bash
python -m rl_garden.models.act.train_trajectories --demo-paths demos/pick-and-place.pkl \
    --total-iters 50000 --batch-size 256 --num-queries 30 \
    --log-dir runs --exp-name act-pick-and-place
```

- Each `rgb_<camera>` key of the demos' observations becomes a camera of a visual policy (here
  `rgb_wrist`); `--no-use-cameras` trains on `state` alone. Images are resized to `--image-size`
  (224) and ImageNet-normalised.
- `--backbone-weights` initialises the ResNet: `imagenet` (default; downloads torchvision's
  `IMAGENET1K_V1`), `none`, or a torchvision state-dict path for an offline machine.
- `--num-queries` is the chunk length (30 steps = 3 s at 10 Hz). Episodes end at `done`, so chunks
  never cross the end of a demo.
- `--save-freq N` also writes `<iter>.pt` every `N` iterations. There is no evaluation during
  training.

*You should see* `runs/act-pick-and-place/checkpoints/final.pt` when the process exits. Details:
`rl-garden/rl_garden/models/act/README.md`, "Training from trajectory pkls". ACT training from real
demos has not been exercised yet ([00](00-overview.md#known-limits)).

### The ACT checkpoint contract

The residual actor loads the checkpoint with the `act:` provider:

- `obs["state"]` must have the checkpoint's width, and the action space its action width (both are
  checked when the provider is built): train on demos recorded with the same `state_fields` (here
  `[joint_pos, gripper_pos]`, 8 values).
- A visual checkpoint reads the env's `rgb_<camera>` keys in sorted order; record and run with the
  same `obs.rgb` in `hil.yaml`.
- The provider uses the `ema_agent` weights by default (`state_dict_key=ema_agent`) and rebuilds the
  model from the `act_config` stored in the checkpoint; the actions are de-normalised with the
  checkpoint's statistics and clipped to `[-1, 1]`, so they mean what the demos' actions meant (same
  `action_mode` and scales).

The spec for [07](07-hil-training.md), quoted on the shell (`?` globs, `&` backgrounds):

<!-- tutorial-test: skip -->
```bash
--base-provider 'act:runs/act-pick-and-place/checkpoints/final.pt?temporal_agg=1'
```

Options, joined with `&`: `temporal_agg=1` (default; ensembles the overlapping chunks with weights
`exp(-temporal_agg_k * i)`) or `0` (replays each chunk open loop), `temporal_agg_k=0.01`,
`state_dict_key=ema_agent`.

### Resync after an intervention

While the human acts, the robot leaves the trajectory an ACT chunk was planned for. Without a resync
an open-loop provider (`temporal_agg=0`) keeps replaying the stale chunk until its next chunk
boundary, and an ensembling one (`temporal_agg=1`) keeps averaging chunks predicted before and during
the takeover, so the stored `base_actions` / `next_base_actions`, the BC target on intervened rows and
the base the policy acts on right after the release lag the robot by up to one chunk.
`--residual-base-resync-on-intervention` (default on; `--no-residual-base-resync-on-intervention`
turns it off) makes the actor reset the provider after every intervened step, before it computes the
base of that step's next observation: the base of every observation after an intervened step is then
the first action of a chunk freshly planned at that observation. The step on which the human takes
over keeps the base computed before the takeover. The `zero` and `checkpoint` (BC) providers have no
state, so the flag changes nothing for them.

> **joint_abs.** A residual base must act in the residual run's action space. For a `joint_abs` HIL
> run, train BC or ACT on the `joint_abs` demos with the `joint_abs` preset. Both modes have the same
> action width, so the providers' width checks do not catch a `joint_delta` base in a `joint_abs` run:
> keep the modes equal yourself.

Next: [06 Intervening](06-intervening.md), then [07 HIL training](07-hil-training.md).
