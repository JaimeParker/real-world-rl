# 8. Deploy

Deployment runs a checkpoint on the robot with no learner, no teleop and no data collection. The
training run's `config.json` (next to `checkpoints/`) decides the action and observation contract,
so the command line gives only the connection.

## The deploy daemon

Deploy against a daemon started from a copy of the station yaml **without the `teleop:` section** (a
configured teleop must be present for a robot to connect, and a deployment has no operator): copy
`local/station.yaml` to `local/station.deploy.yaml` and delete the robot's `teleop:` block. Stop the
training daemon first (a second daemon on the same robot gets `DeviceBusy`), and start the deploy
daemon with the same `--jpeg-quality` as training (this tutorial used none):

```bash
python -m robot_infra.server --config local/station.deploy.yaml --watchdog-s 5
```

## Run a checkpoint

```bash
python examples/deploy_real_robot.py --checkpoint runs/bc-pick-and-place/checkpoints/final.pt \
    --algo real_world_bc --real_robot.remote ROBOT_PC:50051 \
    --episodes 5 --max-steps-per-episode 200
```

| Run | `--algo` | Checkpoint |
|---|---|---|
| BC | `real_world_bc` | `runs/bc-pick-and-place/checkpoints/final.pt` |
| `hil_serl` | `rlpd_hybrid` | `runs/hil-pick-and-place/checkpoints/final.pt` (or a `checkpoint_<update>.pt`) |
| `residual_hil_serl` | `residual_hil_serl_hybrid` for `--residual-variant hybrid`, `residual_hil_serl` for `continuous` | `runs/residual-bc-pick-and-place/checkpoints/final.pt` or `runs/residual-act-pick-and-place/...`; the base provider is rebuilt from the checkpoint metadata |
| `silri` | `silri` | `runs/silri-pick-and-place/checkpoints/final.pt` |

The residual run with its ACT base, for example:

```bash
python examples/deploy_real_robot.py \
    --checkpoint runs/residual-act-pick-and-place/checkpoints/final.pt \
    --algo residual_hil_serl_hybrid --real_robot.remote ROBOT_PC:50051 --episodes 5
```

`--base-provider` replaces the recorded base spec, for example when the base checkpoint moved
(`--base-provider 'act:/data/act/final.pt?temporal_agg=1'`). Add `--real_robot.token T` if the daemon
requires a token, and `--device cuda` for a GPU policy (default `cpu`). An ACT base is not deployed
on its own: it only runs under a residual checkpoint.

*You should see* one line per episode and a summary
(`[deploy] episode=0 steps=... return=... truncated=... rtt_ms=... health_ok=True`, then
`[deploy] summary episodes=... mean_steps=... mean_return=... rtt_ms=... all_healthy=True`). The
return is 0: there is no reward source in deployment; judge success by watching the robot.

*If not:* a device error (daemon unreachable, watchdog, safety) prints `[deploy] device error: ...`
and a `[deploy] hint: ...` line to stderr and exits with code 2. A contract field on the command line
that differs from the training run (`--real_robot.action_mode joint_abs` against a `joint_delta` run,
for example) is a `ValueError`; drop it, the run decides. A robot that fails to connect while the
GELLO is unplugged means the daemon still serves the yaml with the `teleop:` section.

Reference: the README's [Deploy](../../README.md#deploy) section (contract fields, limitations).
