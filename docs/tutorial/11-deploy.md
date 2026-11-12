# 11. Deploy

**Goal:** run a saved checkpoint without a learner or training-data uploads, then read its
outcome and intervention report. Deployment restores `config.json` and `experiment.json`
to rebuild the policy, wrappers and task settings. A standalone `.pt` loads directly;
`bundle_manifest.json` resolves to its generation’s model without restoring replay.

## Before you start

All commands run on the **NUC**, from the **repository root**, with the **project environment
active**, unless stated otherwise.

- Finish [09 Base policy](09-base-policy.md) or [10 HIL training](10-hil-training.md).
  Copy the checkpoint, `config.json` and `experiment.json` to the paths below; for a manifest,
  keep the referenced bundle generation too. A residual run also needs its base checkpoint.
- Stop the recorder or actor. This tutorial restores the saved direct `real_robot.robot`
  connection; no daemon is needed. Keep the workspace clear and the physical stop within reach.
- Complete [03](03-franka-bringup.md) and [04](04-first-motion.md) on this cell. Check the
  saved home, limits, camera settings and action/observation contract before reset motion.
- Read the [operator page](../intervening.md). For unattended deployment, remove `teleop:`
  from the robot mapping used for the connection: a configured device remains mandatory even
  when takeover is disabled. Preserve the robot’s motion and observation settings.

## 1. Choose the checkpoint and algorithm

| Run | `--algo` | Checkpoint |
|---|---|---|
| BC | `real_world_bc` | `runs/bc-pick-and-place/checkpoints/final.pt` |
| `hil_serl` with the tutorial’s gripper | `rlpd_hybrid` | `runs/hil-pick-and-place/checkpoints/bundle_manifest.json`, `final.pt` or `checkpoint_<update>.pt` |
| `residual_hil_serl` | `residual_hil_serl_hybrid` for `hybrid`, `residual_hil_serl` for `continuous` | the matching residual run’s manifest or `.pt`; metadata supplies the base provider |
| `silri` | `silri` | `runs/silri-pick-and-place/checkpoints/bundle_manifest.json` or `.pt` |

Deploy reads the saved training configuration; it does not take the training `--config` flag.
`--config-path` locates a moved `config.json`. Keep the effective settings from
`local/gello_cartesian.yaml` used in [08](08-record-demos.md) and [10](10-hil-training.md).
ACT runs through a residual checkpoint.

## 2. Run without operator takeover

On the NUC, repository root, project environment active:

```bash
rwrl supervise --startup-s 60 --log runs/bc-pick-and-place/deploy-supervise.jsonl -- \
    rwrl deploy --checkpoint runs/bc-pick-and-place/checkpoints/final.pt \
    --algo real_world_bc --no-enable-teleop --episodes 5 --max-steps-per-episode 200
```

*You should see:* one `[deploy] episode=... steps=... return=... truncated=... rtt_ms=...`
line per completed episode, then `[deploy] summary ...`. Check health and timing as well as
motion. The tutorial’s `task.name: none` has no automatic success detector: without takeover
or a classifier, reward is zero and outcomes are unknown. If startup or a device fails, use
[If it fails](#if-it-fails).

For an ACT-base residual bundle, on the same NUC, repository root, project environment active:

```bash
rwrl supervise --startup-s 60 --log runs/residual-act-pick-and-place/deploy-supervise.jsonl -- \
    rwrl deploy --checkpoint runs/residual-act-pick-and-place/checkpoints/bundle_manifest.json \
    --algo residual_hil_serl_hybrid --no-enable-teleop --episodes 5
```

*You should see:* the same episode and summary report. If the base moved, supply
`--base-provider 'act:/data/act/final.pt?temporal_agg=1'` with its actual path. A contract
mismatch needs the original action/observation settings, not an edited data sidecar.

`--device cuda` selects GPU inference (default `cpu`). The supervisor’s heartbeat timeout
is 2.0 s by default; `--startup-s 60` allows policy loading before the first heartbeat.
If cleanup stalls it escalates signals; a final kill relies on firmware handling the lost
connection ([supervision](../daemon.md#supervising-a-single-process-robot-run)).

## 3. Alternative: keep operator takeover available

Use the commissioned GELLO setup from [05](05-teleop-gello.md). `--teleop-device` has no
default, as in [10](10-hil-training.md#1-set-the-shared-flags): whenever deployment builds
an intervention source it needs an explicit device or a saved preset value. `enable_teleop`
defaults to true; a versioned task or an explicit `--teleop-device` activates source composition.
With `--no-enable-teleop`, no intervention source is built.

On the NUC, repository root, project environment active, with the same leader options and
tracker contract as recording:

```bash
rwrl supervise --startup-s 60 --log runs/hil-pick-and-place/deploy-supervise.jsonl -- \
    rwrl deploy --checkpoint runs/hil-pick-and-place/checkpoints/final.pt \
    --algo rlpd_hybrid --teleop-device leader_cartesian --teleop-event-device stdin --episodes 5
```

*You should see:* episode reports with intervention counts and the final assisted/autonomous
rates. `t` takes over, `h` hands back, `s`/`f` label outcomes, Esc ends an episode and `q` quits.
If the leader fails to read, finish deployment and follow [If it fails](#if-it-fails).
Keep the same tracker caps, speeds and device as recording. For matching SpaceMouse-trained
runs use `spacemouse`; preserve the saved tracker/SERL mode and its numeric settings from
[10](10-hil-training.md#1-set-the-shared-flags).

For an autonomous measurement leave action control with the policy; a takeover makes that
episode assisted. Success/failure labels alone do not count as action intervention. A moved
classifier can be supplied with `--classifier-checkpoint`; per-arm task classifiers can be
replaced in the task config. Replacement classifier contents are provenance; task, control,
preprocessing, observation layout and part selection must still match.

## 4. Read the report and stop

The final summary includes health/timing, `success_rate`, `autonomous_success_rate`,
`task_success_rate`, intervention rates and `unknown_episodes`. The primary rate counts
successful unassisted episodes over all completed episodes; assisted successes count in task
success. Unknown outcomes remain in the denominator. Interrupted or rerecorded/discarded
attempts are reported separately. Positive reward or `terminated=True` alone does not confirm
success; a human failure label vetoes success.

*You should see:* the summary after the episode limit or normal quit. Ctrl-C at the supervisor
terminal requests cleanup; normal and exception cleanup close the environment and base
provider. If cleanup stalls, inspect the supervisor log and physical cell before restarting.

## Optional: deploy through a daemon

For a client on another machine use [Remote actor over the robot daemon](../remote-actor.md)
for the station, daemon and transport setup, then supply deploy’s `--real_robot.remote
ROBOT_PC:50051` and, when required, `--real_robot.token T`. An unattended daemon needs a
station without `teleop:`; stop any direct hardware owner first. Use the same JPEG setting
as training (the tutorial uses raw frames). Two-arm runs select
`--env-backend real_robot_bimanual` with each arm’s settings as in
[bimanual.md](../bimanual.md#deploy).

## If it fails

On the NUC, repository root, project environment active, finish deployment first:

```bash
rwrl --station local/station.yaml doctor
```

Start with the first red row and its hint. See
[Network and FCI](../doctor.md#network-and-fci),
[SDK and libfranka version](../doctor.md#sdk-and-libfranka-version),
[Serial, input groups and USB latency](../doctor.md#serial-input-groups-and-usb-latency),
[Cameras](../doctor.md#cameras), or
[Busy devices](../doctor.md#busy-devices-blocked).

A device fault prints `[deploy] device error: ...` and `[deploy] hint: ...` to stderr and exits
with code 1; invalid arguments exit with code 2. A mismatched contract raises `ValueError`
before reset: restore the recorded settings and override only supported connection/provenance
fields. A daemon that still requires an unplugged GELLO is serving a station with `teleop:`.

Next: [Real-world experiments](../real-world-experiments.md) for contracts and recovery,
or [00 Overview](00-overview.md) to review the complete workflow.
