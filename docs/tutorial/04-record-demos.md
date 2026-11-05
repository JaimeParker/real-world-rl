# 4. Record demos

`examples/record_real_demos.py` builds the same env as the HIL actor. It archives
all attempts in SQLite and exports human-supervised steps of successful episodes
to a demo pkl for BC, ACT and HIL. Read [06 Intervening](06-intervening.md) first.

## Prepare the NUC

Run every command on this page on the **NUC**, from the **repository root**, with the
**project environment active**. Finish the station and real doctor checks first. Keep a clear
workspace and the physical stop accessible. Inspect the physical arm's FCI IP,
Hand presence, home, joint limits and workspace before reset, which may move the
arm and open the gripper.

The main preset is
[`examples/presets/gello_cartesian.yaml`](../../examples/presets/gello_cartesian.yaml):
state-only, `cartesian_delta`, 50 Hz, `xyz_scale_m: 0.05`, `rot_scale_rad: 0.3`.
Its robot mapping connects directly through franky on this NUC. Supply the
station-specific values before running: the example workspace is not a cell
approval. Without a physical Hand, set `gripper: null` in the robot mapping and
`gripper_in_action: false` in the env.

The preset's `teleop_options_yaml` points to `gello_leader.yaml` in the repository
root. Create that local options file with `robot_yaml: gello_station.yaml`. Make
`gello_station.yaml` a local leader-only robot YAML: `type: franka`, a distinct
`name`, `arm_backend: fake`, `gripper: null`, `cameras: []`, and the commissioned
GELLO `teleop:` section with its serial port and calibration. Do not point this
source at a physical-follower station: it connects the whole leader robot YAML,
which would open a second connection to the follower. Calibration and alternative local
leader configurations are in [gello.md](../gello.md).

Before recording, on this NUC, repository root, project environment active, finish
any process directly holding the arm. Run `rwrl status`, manually guide the arm
to the desired home, then leave guiding mode. Save and verify home in the same
preset used by the recorder below:

<!-- tutorial-test: skip -->
```bash
rwrl --preset examples/presets/gello_cartesian.yaml home set
rwrl --preset examples/presets/gello_cartesian.yaml --yes home go
```

This is the same home that `env.reset()` resolves. The verification moves the
arm and applies `task.reset.gripper`, while skipping other reset phases.
`home set` rewrites this preset with PyYAML: comments are lost and the previous
file is saved as `examples/presets/gello_cartesian.yaml.bak`. See
[rwrl home](../cli.md#home-resolve-save-and-move).

## Record with GELLO

On the NUC, repository root, project environment active:

```bash
python -m robot_infra.supervise --log runs/record-pick-and-place/supervise.jsonl -- \
    python examples/record_real_demos.py --config examples/presets/gello_cartesian.yaml \
    --teleop-device leader_cartesian --teleop-event-device stdin \
    --real_robot.watchdog_s 0.3 \
    --output-path demos/pick-and-place.pkl --num-successes 20
```

The supervisor runs the hardware process as its child. Terminal keys go to that
child; no desktop session or evdev keyboard permission is needed. The explicit
StepWatchdog setting is **0.3 s**: current HEAD's preset does not yet contain it.
It checks for missing steps inside the process. The separate supervisor detects
missing heartbeats (default **2.0 s**) and escalates signals if cleanup stalls.
Its final kill loses the libfranka connection and relies on a firmware reflex
stop. It cannot issue a separate robot stop command. See
[supervision](../../robot_infra/README.md#supervising-a-single-process-robot-run).

The preset already sets these tracker flags; keep them unchanged through the
pipeline:

| Flag | Value | Meaning |
|---|---|---|
| `--teleop-lead-cap-m` | `0.05` | maximum translation lead over measured TCP, metres |
| `--teleop-lead-cap-rad` | `0.3` | maximum rotation lead, radians |
| `--teleop-target-speed-m-s` | `0.05` | maximum command translation speed, m/s |
| `--teleop-target-speed-rad-s` | `0.3` | maximum command rotation speed, rad/s |

The env scales must be at least the lead caps. The preset's 3000-step episode cap
is 60 seconds at 50 Hz; `--max-episode-steps` overrides it for recording.

## An episode

1. **Reset:** wait for any home motion and gripper opening to finish. The
   recorder holds a fixed pose while nobody drives.
2. **`t`:** take over. The current leader pose becomes an anchor; displacement
   from it maps onto the follower's command pose. Joint-by-joint pose matching
   is not required for this cartesian route.
3. **Drive:** move the GELLO to reach, transport and place the object. The command
   advances at the configured speeds and stays within the lead caps of measured
   TCP; excess lead is dropped rather than queued.
4. **Trigger:** squeeze to close and release to open. At takeover, a trigger that
   disagrees with the gripper's last commanded state is ignored until it crosses
   to that state, preventing an immediate unintended opening.
5. **`h`:** hand back when appropriate. The recorder samples the measured pose
   and holds it fixed. Press `t` to re-anchor and resume.
6. **`s`:** label success and end the episode. For the successful terminal step
   to enter the human demo pkl, press `s` while intervening. If you already used
   `h`, take over again before labeling; this anchors without a jump.

`f` discards the attempt, `r` discards and starts another, and Esc ends the episode
without a success label. `q` ends the current episode and stops after its outcome
is processed; Ctrl-C also stops and saves kept episodes. The default recorder key
reward makes `s` terminal with reward 1. A label after hand-back still ends the
episode, but its terminal step is absent from the human-only pkl view.

*You should see:* episode outcomes and a final save line, for example:

```text
[record] episode=0 steps=87 transitions=87 kept successes=1/20
[record] episode=1 steps=40 transitions=0 discarded (<reason>) successes=1/20
[record] saved 20 episodes (1734 transitions) to demos/pick-and-place.pkl
```

`WORKSPACE HOLD: <axes>` means the requested target left the safety box. The
source hands back and holds measured pose; clear the request, press `h`, then `t`
to re-anchor. It is not a successful step or permission to widen the box.

## Record with a SpaceMouse

Use the same NUC, repository root and project environment. Install the SpaceMouse extra as
covered in [teleop.md](../teleop.md#spacemouse). Override the preset's GELLO
options file so that no leader options are passed to the SpaceMouse source:

```bash
python -m robot_infra.supervise --log runs/record-spacemouse/supervise.jsonl -- \
    python examples/record_real_demos.py --config examples/presets/gello_cartesian.yaml \
    --teleop-device spacemouse --teleop-event-device stdin --teleop-options-yaml None \
    --real_robot.watchdog_s 0.3 \
    --output-path demos/pick-and-place-spacemouse.pkl --num-successes 20
```

Button **0** takes over; the next press hands back. Pushing the puck alone never
counts as takeover. While driving, the puck requests velocity, at up to 0.05 m/s
and 0.3 rad/s at full deflection. Releasing it removes the velocity request; the
arm settles at the existing command, at most one lead cap away. Button **1**
toggles the gripper only while driving. Takeover retains the last gripper command.
Use terminal `s`/`f`/`r`/Esc/`q` for outcomes and quit; terminal `t`/`h` are ignored
on this route. A workspace HOLD requires a new button-0 takeover.

## Files and compatibility

A successful run produces `demos/pick-and-place.pkl`,
`demos/pick-and-place.contract.json` and `demos/pick-and-place.trajectory.sqlite3`.
The archive contains every proposed/executed action, mask and outcome, including
failed, rerecorded and interrupted attempts. The pkl selects successful human
steps; no kept episode means no pkl. Use a new `.pkl` output path for a new set.
`--export-trajectory-pt` additionally writes portable `.trajectory.pt` data.

Keep the sidecar with the pkl. It includes the versioned experiment contract,
including `teleop_control` (`cartesian_tracker_v1`, lead caps and target speeds).
For this tutorial, use the **same teleop device and all four tracker values** for
recording, training and deployment. Missing or changed tracker settings are
rejected with a difference naming `experiment.teleop_control`.

**Current HEAD difference:** the tracker contract names the tracker, not the
physical input device. GELLO and SpaceMouse with equal tracker values can share
data in the implementation. Requiring the same device is this tutorial's workflow
rule; device-only mismatches are not currently rejected by that contract.

*If not:*

- The task times out: review the 60-second cap; increase `--max-episode-steps`
  only for an appropriate task duration.
- `t` does nothing: check terminal focus, the GELLO connection/calibration and
  whether a workspace HOLD first needs `h` followed by `t`. On SpaceMouse use
  button 0 instead.
- Device reads fail: stop, inspect the serial/HID permissions and hardware,
  then rerun doctor with no hardware process holding the station.
- A contract mismatch appears: use the recorded preset, device and tracker
  values rather than editing the data sidecar.

<details>
<summary>Other modes: joint-space leader_arm and joint_abs (not the main workflow)</summary>

These modes need separate joint-space presets and separately recorded demos.
For `leader_arm`, `--teleop-onset arm` waits until the leader is within
`--teleop-arm-tolerance-rad` (0.1 rad) before takeover. Default `ramp` begins
immediately: `joint_abs` rate-limits the initial target by
`--teleop-ramp-step-rad` (0.05) per step; `joint_delta` marks the initial catch-up
until the leader is within one `joint_scale_rad` step. `teleop_syncing` applies
only to these joint-space leader routes, not the cartesian main workflow.

On the NUC, repository root, project environment active, with the separately commissioned
`hil_abs.yaml` (`joint_abs`, joint limits, local robot plus GELLO teleop part):

```bash
python -m robot_infra.supervise --log runs/record-joint-abs/supervise.jsonl -- \
    python examples/record_real_demos.py --config hil_abs.yaml \
    --teleop-device leader_arm --robot-teleop-action-mode joint_abs --teleop-event-device stdin \
    --output-path demos/pick-and-place-abs.pkl --num-successes 20
```

For `joint_abs`, the recorder refuses episode start when any leader joint is
more than `--start-tolerance-rad` (0.8) from its follower. Move the leader into
alignment first. The pkl normally omits syncing rows; `--keep-syncing-steps`
retains them as ordinary supervised steps. Archive-derived training views still
exclude syncing supervision. Joint-space gripper takeover does not have the
cartesian latch; match the trigger to the current gripper state before takeover.

</details>

## Continue after an interruption

The archive commits during recording. Reopening the output stem while its pkl
does not yet exist retains the archive and marks unfinished attempts interrupted;
new attempts receive new IDs. The pkl export covers successes kept by the current
invocation; use the archive for earlier complete successes. Durability begins
when append returns, so an action before its append may be lost. SQLite's
process-crash and power-loss boundaries are in
[Real-world experiments](../real-world-experiments.md#trajectory-archives-and-training-views).
Read the supervisor log before restarting a stopped process; verify the cell and
clear any robot fault before another reset.

More detail: [recording-demos.md](../recording-demos.md).
Next: [05 Base policy](05-base-policy.md).
