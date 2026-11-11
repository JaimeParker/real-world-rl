# 8. Deploy

Deployment runs a checkpoint without a learner or training-data uploads. It restores the training
`config.json` and `experiment.json` to rebuild the policy, wrappers and task settings. It accepts
standalone `.pt` checkpoints and learner `bundle_manifest.json` files; a manifest resolves to its
generation's model for inference, without restoring replay.

## The deploy daemon

For unattended deployment, copy `local/station.yaml` to `local/station.deploy.yaml` and remove the
robot's `teleop:` block. A configured device is mandatory when the daemon connects the robot.
Stop the training daemon first; a second daemon on the same robot gets `DeviceBusy`. Start with
the same `--jpeg-quality` as training (the tutorial used none):

```bash
python -m robot_infra.server --config local/station.deploy.yaml --watchdog-s 5
```

## Run a checkpoint

Explicitly disable takeover for an unattended run:

```bash
python examples/deploy_real_robot.py --checkpoint runs/bc-pick-and-place/checkpoints/final.pt --algo \
    real_world_bc --real_robot.remote ROBOT_PC:50051 --no-enable-teleop --episodes 5 \
    --max-steps-per-episode 200
```

| Run | `--algo` | Checkpoint |
|---|---|---|
| BC | `real_world_bc` | `runs/bc-pick-and-place/checkpoints/final.pt` |
| `hil_serl` with the tutorial's gripper | `rlpd_hybrid` | `runs/hil-pick-and-place/checkpoints/bundle_manifest.json`, `final.pt` or a `checkpoint_<update>.pt` |
| `residual_hil_serl` | `residual_hil_serl_hybrid` for `--residual-variant hybrid`, `residual_hil_serl` for `continuous` | the matching residual run's `bundle_manifest.json` or `.pt`; the base provider is rebuilt from metadata |
| `silri` | `silri` | `runs/silri-pick-and-place/checkpoints/bundle_manifest.json` or `.pt` |

For example, an ACT-base residual bundle:

```bash
python examples/deploy_real_robot.py --checkpoint \
    runs/residual-act-pick-and-place/checkpoints/bundle_manifest.json --algo \
    residual_hil_serl_hybrid --real_robot.remote ROBOT_PC:50051 --no-enable-teleop --episodes 5
```

`--base-provider` replaces the recorded base spec when the base checkpoint moved, for example
`--base-provider 'act:/data/act/final.pt?temporal_agg=1'`. Add `--real_robot.token T` when required
and `--device cuda` for GPU inference (default `cpu`). ACT is used through a residual checkpoint.
Keep the run configuration and experiment metadata with the checkpoint; `--config-path` can
locate a moved training configuration. Two-arm runs also select `--env-backend real_robot_bimanual`
and give each arm's connection/camera settings as in [bimanual.md](../bimanual.md#deploy).

## Keep operator takeover available

`enable_teleop` defaults to true. Deployment composes a takeover source when a versioned task is
active or `--teleop-device` is supplied. To keep the tutorial's GELLO and keyboard available,
use the station YAML with its `teleop:` block, omit `--no-enable-teleop`, and supply
`--teleop-device leader_arm --teleop-event-device stdin`. The saved settings supply the rest
of the intervention contract. With the terminal event source, `t` takes over,
`h` hands back, `s`/`f` label outcomes, Esc ends an episode and `q` quits.
Task runs can retain their saved operator setup. For an autonomous
measurement, leave action control with the policy; take over when needed and report that episode
as assisted. Only pressing success/failure labels does not count as action intervention.

A moved classifier can be supplied with `--classifier-checkpoint`. Per-arm task classifier paths
can be replaced in the task config; task, control, preprocessing, observation layout and part
selection still have to match. Reward sources are recorded provenance, so replacement classifier
contents are recorded without changing the action/observation contract.

## Read the deployment report

*You should see* `[deploy] episode=... steps=... return=... truncated=... rtt_ms=... health_ok=True`
and a final `[deploy] summary ...` with health/timing plus `success_rate`,
`autonomous_success_rate`, `task_success_rate`, intervention rates and `unknown_episodes`.
The primary success rate counts successful unassisted episodes over all completed episodes;
assisted successes count in task success. Unknown outcomes remain in the completed denominator.
Interrupted or rerecorded/discarded attempts are reported separately.

The tutorial's default task has no automatic success detector. With takeover disabled and no
classifier, it reports zero reward and unknown outcomes; watch the robot to judge the attempt.
A classifier or versioned task can supply reward and explicit outcomes. A positive reward or
`terminated=True` alone does not confirm task success, and a human failure label vetoes success.

## If not

A device fault prints `[deploy] device error: ...` and `[deploy] hint: ...` to stderr and exits
with code 1 (invalid arguments exit with code 2). A mismatched action/experiment contract raises `ValueError` before reset motion;
use the recorded settings and only override allowed connection/provenance fields. A daemon that
cannot connect after unplugging GELLO is still serving a YAML with `teleop:`. Normal or exception
cleanup closes the environment and base provider; a forced kill still depends on the watchdog.

Reference: [Deploy](../../README.md#deploy) and
[Real-world experiments](../real-world-experiments.md).
