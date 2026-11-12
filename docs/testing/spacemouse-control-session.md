# SpaceMouse tabletop control and normal-stop acceptance

Run on the robot's NUC, from the repository root, in Bash with its installed
project environment active (`source .venv/bin/activate` for a venv installation).
This checklist is **NOT_RUN** until evidence is collected. It covers one
commissioned Franka using `real_robot`; run motion checks separately for tracker
and SERL. It does not certify other backends or unattended robot operation.

A supervising operator must confirm arm identity, clear workspace, physical stop,
home/reset path, attachment mass/COM, TCP and installation settings before motion.
Use reviewed `local/station.yaml` and `local/task.yaml`; no example IP, home,
workspace or model value is an accepted physical calibration. SERL also requires
its existing XYZ/RPY safety boxes. Do not alter NIC routes, payload or firmware
as a shortcut when diagnostics fail. Input yaw calibration does not calibrate gravity.

The `table` task frame used below is the station's `task_frame`; `teleop run`
passes it to the `local/task.yaml` session, and profile calibration requires it.
Create it before A1 from three captured TCP points: an operator positions the arm
at the frame origin, at a point on +X, and at a point on the +Y side (the commands
read the TCP and never move the arm). `workspace frame set` previews the frame,
its axis lengths and the angle between the measured and final Y axes; `--save`
writes only `task_frame` after the reviewed diff:

```bash
rwrl --station local/station.yaml workspace capture --name frame-origin --output frame-points.yaml --save
rwrl --station local/station.yaml workspace capture --name frame-x --output frame-points.yaml --save
rwrl --station local/station.yaml workspace capture --name frame-y --output frame-points.yaml --save
rwrl --station local/station.yaml workspace frame set --name table --from-points frame-points.yaml --origin frame-origin --x-axis frame-x --y-axis frame-y --save
rwrl --station local/station.yaml workspace frame show
```

Points paths are relative to the station directory. Use a fresh points file for
each frame calibration: a points file records the frame it was captured in, and
later captures into it must match that frame. If `local/task.yaml` carries its own robot
copy, `rwrl --station local/station.yaml --preset local/task.yaml doctor` warns
on every field, including `task_frame`, that differs from the station.

Prepare a fresh evidence directory and copy the record template. The following
setup is shell bookkeeping, not a device check:

<!-- tutorial-test: skip -->
```bash
RUN="local/spacemouse-acceptance-$(date -u +%Y%m%dT%H%M%SZ)"
mkdir "$RUN"
cp docs/testing/spacemouse-control-session-record.csv "$RUN/record.csv"
git rev-parse HEAD > "$RUN/commit.txt"
git diff --binary > "$RUN/working-tree.patch"
cp local/station.yaml local/task.yaml "$RUN/"
python -m pip freeze > "$RUN/dependencies.txt"
```

Keep original profile files and all untracked source changes with this record;
the commit and tracked patch alone do not identify an uncommitted installation.
Each CLI input acquisition creates a unique report directory. Recording creates
an archive plus a sibling `*.session-*` directory. Copy these directories into
RUN after writers close, preserving SQLite WAL/SHM files if present. Record paths
in the CSV. Record timestamped video of the arm and operator's inputs for motion
items. A task-success label is not a hardware PASS.

## A: Input-only commissioning

A1 opens only SpaceMouse. Stand behind the arm to define fixed tabletop +X
forward, +Y left, +Z up. Stand in the actual operating position while making
natural labelled pushes. Include +X/+Y, independent -X/-Y, both Z directions and
signed rotations about all three axes. Repeat for front operation and at least
one non-right-angle device placement; save separate profiles.

```bash
rwrl --station local/station.yaml teleop calibrate --profile rear --frame table --save --output profiles.yaml --use
rwrl --station local/station.yaml teleop diagnose --profile rear --duration-s 5
```

A1 passes when all labelled directions agree with intent and residuals meet the
chosen threshold (default 25 degrees), with raw evidence retained. Natural
cross-axis input is acceptable; direction and usability matter more than precise
endpoint placement. Wrong signs, reflected mappings or weak evidence require
recapture, not a second guessed mounting-roll correction. For single-group
recapture see [CLI calibration](../spacemouse-commissioning.md#spacemouse-profiles-and-task-frame-workspace).

## B: Static feedback and workspace

B1 inspects the host and configured device health. B2 captures a read-only TCP
sample and reviews the current constraints; it does not drive to a boundary.

```bash
rwrl --station local/station.yaml doctor
rwrl --station local/station.yaml workspace show
rwrl --station local/station.yaml workspace frame show
rwrl --station local/station.yaml workspace capture --name reference --output workspace-points.yaml --save
```

B1/B2 pass only with independently verified arm identity, fresh feedback, correct
frame origin/orientation (`workspace frame show` gives the origin and unit axes
in the base frame; compare them with the table and the `frame-points.yaml` points
used to create the frame), home inside the joint limits (`home_within_limits` in
`workspace show`), reviewed base/task constraint intersection and actual
installation/payload/TCP evidence. Doctor or an SDK-reported mass alone cannot
verify the physical model. Missing measurements mean BLOCKED. Named captured
points can propose bounds, but do not prove the whole enclosed volume is safe.

## C: Supervised direction checks

C1 translation, C2 rotation, C3 all axes use low diagnostic gain. Each command
launches the recorder under `robot_infra.supervise`; do not wrap it in a second
supervisor. Reset can home the arm. At the nominal step budget it holds until q;
`--duration-s` is not a wall-clock termination deadline.

```bash
rwrl --station local/station.yaml --preset local/task.yaml teleop run --mode tracker --diagnose translation --gain 0.2 --duration-s 10 --record local/check-translation.pkl
rwrl --station local/station.yaml --preset local/task.yaml teleop run --mode tracker --diagnose rotation --gain 0.2 --duration-s 10 --record local/check-rotation.pkl
rwrl --station local/station.yaml --preset local/task.yaml teleop run --mode tracker --diagnose all --gain 0.2 --duration-s 10 --record local/check-all.pkl
```

Repeat with `--mode serl` and new output paths. PASS requires intended positive
and negative directions, fixed tabletop axes at different TCP orientations,
usable combined control, and no measured workspace breach or fault. Compare
raw input, requested/limited/accepted targets and measured base/task pose in
`diagnostics-<timestamp>-<id>.jsonl`; report clipping separately from mapping errors. Small hand
coupling and tracking lag do not by themselves fail direction acceptance. Stop
if control is unexpected; do not continue merely to complete a table row.

## D: Trajectory ownership and end holding

```bash
rwrl --station local/station.yaml --preset local/task.yaml teleop run --mode tracker --record local/check-session.pkl --num-successes 2 --max-episodes 3
```

D1: inside one trajectory perform human → hand-back → human → hand-back at least
twice. Tracker uses button 0; SERL uses motion/buttons and returns on idle.
Recording hand-back uses its hold policy. To verify **learned policy** restoration,
repeat D1 using the reviewed [training workflow](../tutorial/07-hil-training.md)
with `episode_end_behavior: hold`; a recorder-only run cannot pass that subcheck.
Record policy version, human masks and at least two ownership transitions in the
same episode; no reset or neutral gate may occur at hand-back.

D2: label success/failure, then move the puck. The last accepted target must stay
fixed, no extra trajectory transitions may be added, and a new trajectory starts
only after n and stable neutral input. Defaults are six-axis max magnitude ≤0.05,
buttons released for 0.2 seconds; record any override. At the episode quota, n
must remain disabled while q exits. Repeat in SERL mode with a fresh output.

D3: verify a candidate tracker boundary rejection hands back without sending the
out-of-bounds target; button 0 can retake. Do not intentionally push the measured
TCP outside the safe volume to test a fault. Device enforcement must remain
active for policy and teleoperation alike. Fault/disconnect/watchdog checks need
a separately reviewed supervised protocol or hardware-free tests; never unplug
or kill a live controller as an unplanned experiment.

## E: Normal-stop evidence and F: remediation

E1 uses the session's `stop-trace-<timestamp>-<id>.json` and assessment. Once the writer has
closed, copy the chosen trace to the following path and evaluate offline:

```bash
rwrl --json teleop inspect-stop --trace local/stop-trace.json
```

Require stationary pre-stop coverage, the entire stop interval, at least 2 seconds
of post-stop observation, advancing device timestamps and no coverage gap over
0.1 seconds. Maximum sampled 3D displacement from the last pre-stop pose must be
≤10 mm, SO(3) angle ≤5 degrees, and every joint |dq| ≤0.02 rad/s throughout the
final 0.5 seconds. The arm must be in active tracker control when stop starts; an already
IDLE arm is BLOCKED, even with stationary samples. No control fault or measured workspace
breach beyond `measured_margin_m` / `measured_margin_rad` (defaults `0.003 m` / `0.03 rad`)
is allowed. Target limits remain strict.
Report task-frame downward displacement separately; it does not replace 3D error.
Keep raw samples, SDK fingerprints/model fields, assessment and video. Missing
coverage is BLOCKED, known threshold violation FAIL. Test both modes; do not
claim a sparse endpoint comparison passes the full interval requirement.

F1: if E1 fails, first distinguish unstable fixed-target holding, incorrect
installation/payload/TCP configuration and the SDK's stop transition. Do not
blindly tune gains, add gravity compensation or extend timeout. Any model/SDK
change needs measured justification and operator review, a reproducible version,
and repeated affected C–E checks. A required undelivered SDK change is BLOCKED.
Emergency stops and fault cleanup do not inherit the normal-stop displacement
promise. This page does not claim the previous observed stop drop has been fixed.
