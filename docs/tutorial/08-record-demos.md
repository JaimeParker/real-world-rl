# 8. Record demos

**Goal:** record successful human demonstrations and inspect their archive before training a
base policy or HIL learner. The recorder builds the same env as the HIL actor, archives every
attempt in SQLite and exports successful human-supervised steps to a demo pkl.

<a id="prepare-the-nuc"></a>

## Before you start

All commands run on the **NUC**, from the **repository root**, with the **project environment
active**, unless stated otherwise.

- Finish [03 Bring up the Franka](03-franka-bringup.md) and
  [04 Home, gripper, cameras](04-first-motion.md). Nothing else may hold the arm.
- Commission the leader and its options file using [05 Teleop: GELLO](05-teleop-gello.md).
- Read the [operator page](../intervening.md). Keep the workspace clear and the physical stop
  accessible. Reset may move the arm and open the gripper.
- Check the physical arm’s FCI IP, Hand, home, joint limits and safety box in the local preset.
  Without a Hand, set `real_robot.robot.gripper: null` and `real_robot.gripper_in_action: false`.

`rwrl record` is the hardware process; `rwrl supervise` starts it as a child and watches its
heartbeats. Wrap each recording command below so a silent or stalled child triggers
SIGINT → SIGTERM → SIGKILL. The heartbeat timeout defaults to **2.0 s**; a final kill loses
the libfranka connection and relies on firmware stopping. The supervisor cannot issue an
independent robot stop. The recorder commands also enable an in-process StepWatchdog at
**0.3 s**, which detects missing steps while that process can still run. Terminal keys reach
the child; stdin needs neither a desktop session nor evdev keyboard permissions. See
[supervision](../daemon.md#supervising-a-single-process-robot-run).

| Flag | Value here | Meaning |
|---|---|---|
| `--config` | `local/gello_cartesian.yaml` | commissioned env and teleop preset |
| `--teleop-device` | `leader_cartesian` | explicit motion source; no default |
| `--teleop-event-device` | `stdin` | operator keys in this terminal |
| `--real_robot.watchdog_s` | `0.3` | in-process step timeout, seconds |
| `--output-path` | `demos/pick-and-place.pkl` | new pkl path; existing files are refused |
| `--num-successes` | `20` (also the default) | stop after this many kept episodes |
| `--teleop-lead-cap-m` / `--teleop-lead-cap-rad` | `0.05` / `0.3` | maximum lead over measured TCP |
| `--teleop-target-speed-m-s` / `--teleop-target-speed-rad-s` | `0.05` / `0.3` | maximum target speed |

## 1. Prepare the local preset and home

If [04](04-first-motion.md#1-home) already created and commissioned this copy, keep it.
Otherwise, on the NUC, repository root, no environment needed, copy the example once:

<!-- tutorial-test: skip -->
```bash
cp examples/presets/gello_cartesian.yaml local/gello_cartesian.yaml
```

*You should see:* `local/gello_cartesian.yaml`. It starts with state-only observations,
`cartesian_delta`, 50 Hz, `xyz_scale_m: 0.05` and `rot_scale_rad: 0.3`. Complete the local
hardware/leader settings and save and verify home as in [04](04-first-motion.md#1-home).
Copying again would replace that saved home. If home or device checks fail, use
[If it fails](#if-it-fails) before recording.

## 2. Start the GELLO recorder

On the NUC, repository root, project environment active:

```bash
rwrl supervise --log runs/record-pick-and-place/supervise.jsonl -- \
    rwrl record --config local/gello_cartesian.yaml \
    --teleop-device leader_cartesian --teleop-event-device stdin \
    --real_robot.watchdog_s 0.3 \
    --output-path demos/pick-and-place.pkl --num-successes 20
```

*You should see:* reset followed by a fixed hold pose, ready for `t`. If startup fails,
finish the recorder and follow [If it fails](#if-it-fails).

The env scales must be at least the lead caps. The preset's 3000-step episode cap
is 60 seconds at 50 Hz; `--max-episode-steps` overrides it for recording.

## 3. Record one episode

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

## 4. Alternative: record with a SpaceMouse

Use the same NUC, repository root and project environment. Install the SpaceMouse extra as
covered in [teleop.md](../teleop.md#spacemouse). Override the preset's GELLO
options file so that no leader options are passed to the SpaceMouse source:

```bash
rwrl supervise --log runs/record-spacemouse/supervise.jsonl -- \
    rwrl record --config local/gello_cartesian.yaml \
    --teleop-device spacemouse --teleop-event-device stdin --teleop-options-yaml None \
    --real_robot.watchdog_s 0.3 \
    --output-path demos/pick-and-place-spacemouse.pkl --num-successes 20
```

*You should see:* the same episode outcomes and save line as with GELLO, at the SpaceMouse
output path. If HID reads fail, follow [If it fails](#if-it-fails).

Button **0** takes over; the next press hands back. Pushing the puck alone never
counts as takeover. While driving, the puck requests velocity, at up to 0.05 m/s
and 0.3 rad/s at full deflection. Releasing it removes the velocity request; the
arm settles at the existing command, at most one lead cap away. Button **1**
toggles the gripper only while driving. Takeover retains the last gripper command.
Use terminal `s`/`f`/`r`/Esc/`q` for outcomes and quit; terminal `t`/`h` are ignored
on this route. A workspace HOLD requires a new button-0 takeover.

For native HIL-SERL SpaceMouse behavior, on the **NUC**, **repository root**,
**project environment active**, add `--spacemouse-mode serl` to the SpaceMouse
recorder command. Configure the station's xyz **and RPY** safety boxes; the mode
selects and saves `real_robot.cartesian_semantics: serl`. Puck motion with six-axis
norm `> 0.001` takes over immediately; release the puck and buttons to hand back.
Hold button 0 to close, button 1 to open (left wins); neither sends neutral zero.
`t`/`h` are ignored. Press `s` while intervening to include the successful terminal
step in the human demo pkl; recorder idle steps still use its hold policy.
Keep the same SERL mode in training/deploy: its contract is `spacemouse_serl_v1`
and tracker caps/speeds do not apply. See [both modes](../teleop.md#spacemouse).

For an inclined base or operator-specific puck direction, commission a station
task frame and named profile with the [SpaceMouse CLI workflow](../spacemouse-commissioning.md#spacemouse-profiles-and-task-frame-workspace).
The supervised `rwrl teleop run` workflow uses those settings and holds the last
accepted target after a label; n requests the next trajectory and q releases
control. The ordinary recorder commands above retain their default automatic
episode handling unless `--episode-end-behavior hold` is selected. Use the
[acceptance checklist](../testing/spacemouse-control-session.md) before treating
direction mapping or normal-stop behavior as verified.

<a id="files-and-compatibility"></a>

## 5. Inspect the files and compatibility

A successful run produces `demos/pick-and-place.pkl`,
`demos/pick-and-place.contract.json` and `demos/pick-and-place.trajectory.sqlite3`.
The archive contains every proposed/executed action, mask and outcome, including
failed, rerecorded and interrupted attempts. The pkl selects successful human
steps; no kept episode means no pkl. Use a new `.pkl` output path for a new set.
`--export-trajectory-pt` additionally writes portable `.trajectory.pt` data.

Keep the sidecar with the pkl. It includes the versioned experiment contract,
including `teleop_control` (`cartesian_tracker_v1`, lead caps and target speeds
for the default tracker route; `spacemouse_serl_v1` for SERL mode).
For this tutorial, use the **same teleop device and all four tracker values** for
recording, training and deployment. Missing or changed tracker settings are
rejected with a difference naming `experiment.teleop_control`.

**Current HEAD difference:** the tracker contract names the tracker, not the
physical input device. GELLO and SpaceMouse with equal tracker values can share
data in the implementation. Requiring the same device is this tutorial's workflow
rule; device-only mismatches are not currently rejected by that contract.

On the NUC, repository root, project environment active (this command opens no hardware):

```bash
rwrl archive inspect demos/pick-and-place.trajectory.sqlite3
```

*You should see:* JSON with `records`, `episode_starts`, `episode_ends`, `transitions`,
`human_mask_counts`, `outcomes` and per-episode measurements. The source is read-only.
If the file is missing, check the recorder’s output stem and whether recording reached reset.

## 6. Optional: joint-space recording

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
rwrl supervise --log runs/record-joint-abs/supervise.jsonl -- \
    rwrl record --config hil_abs.yaml \
    --teleop-device leader_arm --robot-teleop-action-mode joint_abs --teleop-event-device stdin \
    --output-path demos/pick-and-place-abs.pkl --num-successes 20
```

For `joint_abs`, the recorder refuses episode start when any leader joint is
more than `--start-tolerance-rad` (0.8) from its follower. Move the leader into
alignment first. The pkl normally omits syncing rows; `--keep-syncing-steps`
retains them as ordinary supervised steps. Archive-derived training views still
exclude syncing supervision. Joint-space gripper takeover does not have the
cartesian latch; match the trigger to the current gripper state before takeover.

*You should see:* episode outcomes and a save line for `demos/pick-and-place-abs.pkl`.
If the initial gap is rejected, align the leader before trying again; do not raise the tolerance
to bypass the check.

</details>

## 7. Continue after an interruption

The archive commits during recording. Reopening the output stem while its pkl
does not yet exist retains the archive and marks unfinished attempts interrupted;
new attempts receive new IDs. The pkl export covers successes kept by the current
invocation; use the archive for earlier complete successes. Durability begins
when append returns, so an action before its append may be lost. SQLite's
process-crash and power-loss boundaries are in
[Real-world experiments](../real-world-experiments.md#trajectory-archives-and-training-views).
Read the supervisor log before restarting a stopped process; verify the cell and
clear any robot fault before another reset.

Rerun the matching recorder command above only when its pkl does not exist. If a pkl was
already saved, choose a new output path; there is no pkl append flag.

*You should see:* new attempt IDs in the existing archive and, after keeping successes, a new
save line. If restart fails, inspect the supervisor log and use the checks below.

## If it fails

On the NUC, repository root, project environment active, finish the hardware process first:

```bash
rwrl --station local/station.yaml doctor
```

Start with the first red row and its hint. See
[Network and FCI](../doctor.md#network-and-fci),
[Serial, input groups and USB latency](../doctor.md#serial-input-groups-and-usb-latency),
[SpaceMouse and HID](../doctor.md#spacemouse-and-hid),
[Cameras](../doctor.md#cameras), or
[Busy devices](../doctor.md#busy-devices-blocked).

- The task times out: review the 60-second cap; increase `--max-episode-steps`
  only for an appropriate task duration.
- `t` does nothing: check terminal focus, the GELLO connection/calibration and
  whether a workspace HOLD first needs `h` followed by `t`. On SpaceMouse use
  button 0 instead.
- Device reads fail: stop, inspect the serial/HID permissions and hardware,
  then rerun doctor with no hardware process holding the station.
- A contract mismatch appears: use the recorded preset, device and tracker
  values rather than editing the data sidecar.

More detail: [recording-demos.md](../recording-demos.md).
Next: [09 Base policy](09-base-policy.md).
