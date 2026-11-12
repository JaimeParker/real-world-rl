# 6. Teleop: SpaceMouse

**Goal:** check HID input, commission a direction profile and task frame, and complete one
supervised SpaceMouse session.

## Before you start

All commands run on the **NUC with the SpaceMouse attached**, from the **repository root**,
with the **project environment active**, unless stated otherwise.

- Finish [03](03-franka-bringup.md) and [04](04-first-motion.md), including the reviewed local
  cartesian preset, home and safety limits. Read the [operator page](../intervening.md).
- Use a station whose `teleop.backend` is `spacemouse`. If switching from GELLO, preserve its
  calibrated station separately before changing the local station. Nothing else may hold the
  HID device or follower arm. Keep the physical stop accessible during motion.
- These steps commission the input; physical directions, timing and disconnect behavior still
  need hardware verification. The defaults are described in [SpaceMouse reference](../teleop.md#spacemouse).

## 1. Install and check USB access

Install the input extra in the existing project environment:

<!-- tutorial-test: skip -->
```bash
pip install -e '.[spacemouse]'
```

Then check and, where proposed, repair udev/group access:

```bash
rwrl doctor --station local/station.yaml
rwrl doctor --station local/station.yaml --fix
```

*You should see:* `usb.*` and `spacemouse.*` checks passing after any requested replug or new
login. The extra supplies `hidapi` (import `hid`); follow
[SpaceMouse and HID](../doctor.md#spacemouse-and-hid) for SDK or permission failures.

For an unknown 3Dconnexion device, capture its descriptor and reports without moving a robot:

```bash
rwrl --station local/station.yaml teleop probe-hid --duration-s 10
```

Move each axis and press every button during capture. With multiple candidates, repeat with
`--device VID:PID`, using the reported ID. `--output DIR` chooses a new evidence directory.

*You should see:* evidence under `local/teleop/probes/<vid>-<pid>/`, including `descriptor.hex`,
`reports.jsonl` and `candidate.yaml`. Review the draft and resolve any TODO decoding fields;
merge the verified entry into `local/spacemouse-devices.yaml` (next to the station). The probe
does not install it. See [site device table](../spacemouse-commissioning.md#site-device-table)
for format, evidence and failures; rerun doctor before continuing.

## 2. Diagnose the input without motion

```bash
rwrl --station local/station.yaml teleop diagnose --duration-s 5
```

*You should see:* guided input capture and a diagnostic report describing axes, buttons and
the device identity. Check that each requested motion has the expected sign and returns to
rest. `--profile NAME` applies an existing profile; `--report-dir DIR` chooses the evidence
location. `diagnose` is SpaceMouse-only and takes no `--backend` flag. If the device cannot
open, use [If it fails](#if-it-fails); do not proceed to motion with unverified axes.

## 3. Set the task frame and inspect the workspace

As introduced in [04](04-first-motion.md#5-optional-task-frame-and-workspace), position the arm
by the vendor's manual guiding procedure at three distinct points: the origin, a point along
+X, then a point on the +Y side of X. Leave guiding mode before each read. Execute each capture
at its corresponding point; these commands read the pose and do not move the arm:

```bash
rwrl --station local/station.yaml workspace capture --name frame-origin --output frame-points.yaml --save
rwrl --station local/station.yaml workspace capture --name frame-x --output frame-points.yaml --save
rwrl --station local/station.yaml workspace capture --name frame-y --output frame-points.yaml --save
rwrl --station local/station.yaml workspace frame set --name table \
    --from-points frame-points.yaml --origin frame-origin --x-axis frame-x --y-axis frame-y --save
rwrl --station local/station.yaml workspace frame show
rwrl --station local/station.yaml workspace show
```

*You should see:* a reviewed task-frame diff, its saved origin and axes, then the configured
workspace and base safety limits. Point files are relative to the station directory. Skip
recapture if the existing frame is already commissioned; changing it rebases a workspace with
that frame name, so review its bounds again. Collinear or too-close points must be recaptured.

The task frame gives the tracker profile's directions a task-relative meaning. Workspace
bounds constrain requests in that frame, intersected with the base safety box. A frame alone
does not create a workspace or prove its interior collision-free. Review bounds using the
[profile and workspace commissioning guide](../spacemouse-commissioning.md#spacemouse-profiles-and-task-frame-workspace)
before motion; see [If it fails](#if-it-fails) for device errors during capture.

## 4. Calibrate and select a direction profile

With `table` saved as above, run the interactive direction capture:

```bash
rwrl --station local/station.yaml teleop calibrate --backend spacemouse \
    --profile rear --frame table --output teleop-profiles.yaml --use
rwrl --station local/station.yaml teleop profiles list
rwrl --station local/station.yaml teleop profiles show --profile rear
rwrl --station local/station.yaml teleop profiles use --profile rear
```

*You should see:* prompted direction trials, a validated candidate, then the saved `rear`
profile and its selection in station `teleop_profiles`. Review both file and station diffs.
The output profile path is relative to the station directory. `profiles use` explicitly
selects an existing profile; it is redundant immediately after successful `--use` calibration.

A direction profile fits puck yaw relative to `table`, records direction/rotation validation
and device identity, and stores the neutral threshold. It does not change the robot's home.
Calibration defaults are 3 seconds per capture, 25 degrees maximum error and neutral threshold
0.05. Use `--frame base` for a base-frame profile. `--save` saves without selecting; `--use`
saves and selects. `--samples`, `--capture-label`, `--append-trial` and `--report-dir` support
repeatable trials; see [live calibration](../spacemouse-commissioning.md#live-input-calibration-and-diagnosis).
If validation fails, repeat the indicated trials instead of selecting the candidate.

## 5. Complete a supervised tracker session

```bash
rwrl --station local/station.yaml --preset local/gello_cartesian.yaml teleop run --backend spacemouse \
    --mode tracker --profile rear --record demos/spacemouse-check.pkl --num-successes 1
```

Review the motion confirmation; reset can home the arm and operate the gripper. The command
resolves the station profile and starts its own supervisor. After reset, press button **0**
to take over, deflect the puck gently, and press button **1** to toggle the gripper. A second
button-0 press hands back. Puck motion alone does not take over. Releasing the puck stops
adding velocity; the follower can still settle toward its bounded command target.

Label with terminal `s` while driving, or `f`/`r`/Esc as described on the
[operator page](../intervening.md#spacemouse). Terminal `t`/`h` are ignored. Labels hold the last
accepted target. `n` requests another trajectory when the quota allows; release puck/buttons
until neutral (profile threshold, stable for 0.2 seconds). At the quota, use `q` to exit.

*You should see:* the selected profile, supervised reset/hold behavior and a report directory
`demos/spacemouse-check.session-<id>/` containing resolved configuration and provenance. A kept
success produces the pkl and contract/archive companions. A workspace HOLD hands back; clear
the request and press button 0 for a fresh takeover. For startup or input failures, use
[If it fails](#if-it-fails).

`tracker` is the default: button takeover, persistent velocity-limited target, last gripper
command retained at takeover. Opt-in `--mode serl` uses raw motion/button takeover: six-axis
norm above 0.001 takes over, release returns control to the policy (recorder idle holds), and
held button 0 closes / button 1 opens, with button 0 priority and neutral gripper when neither
is held. Configure both xyz and RPY safety boxes first. SERL has a separate action contract;
keep the same mode through recording, training and deployment. See [both modes](../teleop.md#spacemouse).

Common session flags are `--record` (required new `.pkl` path), `--num-successes` (default 20),
`--max-episodes` (optional attempt cap) and `--events-file` (JSONL events instead of terminal
keys). Formal dataset collection and archive inspection continue in [08](08-record-demos.md).

## If it fails

Run `rwrl doctor --station local/station.yaml` first and find the red row in
[doctor's red lights](../doctor.md#red-lights-what-to-do).

- USB/HID access or the wrong Python SDK: [SpaceMouse and HID](../doctor.md#spacemouse-and-hid).
- Device held by another session: [busy devices (BLOCKED)](../doctor.md#busy-devices-blocked).
- Unknown VID/PID: review the [site device table](../spacemouse-commissioning.md#site-device-table)
  evidence before installing a draft entry.
- Missing/mismatched frame, invalid profile or reversed directions: repeat frame/profile
  commissioning and input diagnosis before moving. Profile selection cannot repair HID access.
- Missing preset or existing output: pass the commissioned global `--preset` and a new pkl path.

Next: [07 Teleop: leader-follower](07-teleop-leader-follower.md).
