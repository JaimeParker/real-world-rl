# 9. Base policy: BC or ACT

**Goal:** train an offline BC or ACT base from [08](08-record-demos.md), and keep the
checkpoint with the metadata needed by HIL and deployment. BC can deploy on its own;
[10](10-hil-training.md) can train a bounded residual over either base.

## Before you start

All commands run on the **GPU host**, from the **repository root**, with the **project
environment active** ([01](01-install.md)). Neither trainer connects to the robot.

- Copy the pkl and its contract sidecar from [08](08-record-demos.md) to the same relative
  paths on this host.
- Bring the commissioned `local/gello_cartesian.yaml` from the NUC, preserving its action,
  observation, task and control settings. Keep the NUC’s full preset for recording and HIL.

| | BC (`real_world_bc`) | ACT |
|---|---|---|
| Model | rl-garden's BC actor, one action per observation | action-chunking transformer (CVAE), predicts `--num-queries` actions per step |
| Trained by | `rwrl train real_world_bc` | `python -m rl_garden.models.act.train_trajectories` (in rl-garden) |
| Residual base spec | `checkpoint:<path>?algo=real_world_bc` | `act:<path>?temporal_agg=1` |
| Deploy on its own | yes, `--algo real_world_bc` | no (only as a residual base) |

## 1. Train BC

`real_world_bc` trains offline from demo pkls or trajectory archives, without connecting to the
robot. The `--config` preset defines the target action/observation and experiment contracts.
The loader validates the recorded contract before training. Partial demonstrations supervise
only `human_action_mask` dimensions; idle hold supplements are not expert labels. Old demos
without intervention metadata retain full-action supervision; explicit empty masks remain empty.

`real_world_bc` accepts the same `local/gello_cartesian.yaml` used for recording, HIL and
deployment. Its teleop settings are ignored during offline BC training.

On the GPU host, repository root, project environment active:

```bash
rwrl train real_world_bc --config local/gello_cartesian.yaml \
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

*You should see:* BC loss metrics logged every `--log-freq` steps; the run is done when the process exits and
`runs/bc-pick-and-place/checkpoints/final.pt` and `runs/bc-pick-and-place/config.json` exist. The
`config.json` next to `checkpoints/` is what deploy and the residual base provider read to rebuild
the policy.

*If not:* `ValueError: ... recorded action contract does not match the env's (differs in [...])`
means the demos were recorded with different settings than the preset (action mode, joint limits,
scales, state fields): record again, or fix the preset. `No demo transitions or restored replay available` means the path
or glob matched nothing. For environment failures, use [If it fails](#if-it-fails).

## 2. Alternative: train ACT

rl-garden trains ACT from the same pkls, without a simulator. On the GPU host, repository root, project environment active:

```bash
python -m rl_garden.models.act.train_trajectories --demo-paths demos/pick-and-place.pkl \
    --total-iters 50000 --batch-size 256 --num-queries 30 --no-use-cameras \
    --log-dir runs --exp-name act-pick-and-place
```

- Each `rgb_<camera>` key of the demos' observations becomes a camera of a visual policy (none in this preset); the command
  uses `--no-use-cameras` to train on `state` alone. Images are resized to `--image-size`
  (224) and ImageNet-normalised.
- `--backbone-weights` initialises the ResNet: `imagenet` (default; downloads torchvision's
  `IMAGENET1K_V1`), `none`, or a torchvision state-dict path for an offline machine.
- `--num-queries` is the chunk length (30 steps = 0.6 s at 50 Hz). Episodes end at `done`, so chunks
  never cross the end of a demo.
- `--save-freq N` also writes `<iter>.pt` every `N` iterations. There is no evaluation during
  training.

*You should see* `runs/act-pick-and-place/checkpoints/final.pt` when the process exits. Details:
`rl-garden/rl_garden/models/act/README.md`, "Training from trajectory pkls". If training fails, check the demo path and observation keys first, then
[If it fails](#if-it-fails). ACT training from real
demos has not been exercised yet ([00](00-overview.md#known-limits)).

### The ACT checkpoint contract

The residual actor loads the checkpoint with the `act:` provider:

- `obs["state"]` must have the checkpoint's width, and the action space its action width (both are
  checked when the provider is built): train on demos recorded with the same `state_fields` (here
  `[tcp_pose, tcp_vel, joint_pos, joint_vel]`, 27 values).
- A visual checkpoint reads the env's `rgb_<camera>` keys in sorted order; record and run with the
  same `obs.rgb` in `local/gello_cartesian.yaml`.
- The provider uses the `ema_agent` weights by default (`state_dict_key=ema_agent`) and rebuilds the
  model from the `act_config` stored in the checkpoint; the actions are de-normalised with the
  checkpoint's statistics and clipped to `[-1, 1]`, so they mean what the demos' actions meant (same
  `action_mode` and scales).

For [10](10-hil-training.md), use `--base-provider` with the quoted value
`'act:runs/act-pick-and-place/checkpoints/final.pt?temporal_agg=1'` (`?` globs, `&` backgrounds).

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

## Keep the experiment contract with the base

Keep the training `config.json`, `experiment.json` and checkpoint together. Task, control,
preprocessing, camera order and action interpretation are checked along with shapes. Configure
them before recording and preserve them through the residual run. See
[experiment contracts](../real-world-experiments.md#trajectory-archives-and-training-views).

## If it fails

On the GPU host, repository root, project environment active:

```bash
rwrl doctor
```

Start with the first red row and its hint; see
[Python environment and extras](../doctor.md#python-environment-and-extras). An absent
station on an offline GPU host is `SKIPPED`, not a training failure. For contract errors,
compare the named differences against the recording preset; keep the sidecar intact.

Next: the [operator page](../intervening.md), then [10 HIL training](10-hil-training.md).
