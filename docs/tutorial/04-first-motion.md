# 4. Home, gripper, cameras

**Goal:** check each device with one `rwrl` command: save a home pose and move the arm to it,
open and close the gripper, see the cameras' frames, and know how to stop and recover.

All commands run on the NUC, from the repository root, with the project environment active.

## Before you start

- The doctor of [03](03-franka-bringup.md) is green and nothing else holds the arm.
- The workspace is clear and the emergency stop is within reach: this page moves the arm.
- **How motion is confirmed.** `home go` and `gripper open/close/move` first print the target
  and the largest change, then ask `proceed? [y/N] `. Only `y` (any case) moves; any other answer
  or end of input cancels with exit code 2. Without a terminal (piped input, an agent) they
  refuse to move unless you give `--yes`, which answers the question for you; the target is
  still printed. Global options such as `--station`, `--preset` and `--yes` go **before** the
  command.

## 1. Home

The station's home is the `home_joints` the wizard wrote. Compare it with the arm's current
position (this reads the arm and moves nothing):

```bash
rwrl --station local/station.yaml home show
```

*You should see:* `source: station home_joints`, then `home (rad)`, `q (rad)`,
`delta (home - q, rad)` and `max delta`.

The home used by recording and training lives in the env preset, where `env.reset()` reads it.
Make a local copy of the tutorial's preset so the checked-in file stays unchanged
(no environment needed for `cp`):

<!-- tutorial-test: skip -->
```bash
cp examples/presets/gello_cartesian.yaml local/gello_cartesian.yaml
```

Guide the arm by hand to the home you want (Desk's guiding mode or the arm's guiding button),
leave guiding mode, then save the measured joints into the copy and check them:

```bash
rwrl --preset local/gello_cartesian.yaml home set
rwrl --preset local/gello_cartesian.yaml home show
```

`home set` writes `task.reset.home_joints` (rounded to four decimals) after checking them
against the station's and preset's joint limits, keeps comments and key order, and saves the
previous file beside it as `local/gello_cartesian.yaml.<id>.bak`. It asks no question: it does
not move the arm. Page 08 switches to this same `--preset local/gello_cartesian.yaml` form in a
later update; until then, use this copy wherever it names `examples/presets/gello_cartesian.yaml`.

Now move the arm away a little and send it home. The first command asks before it moves; the
second answers for you:

```bash
rwrl --preset local/gello_cartesian.yaml home go
rwrl --preset local/gello_cartesian.yaml --yes home go
```

*You should see:* `target home [...]; max delta ... rad` before the motion, then the home, the
new `q` and a `max delta` close to zero. `home go` runs only the home phase of a reset: a
blocking joint move, then the preset's `task.reset.gripper` (`open` or `close`; `keep` leaves
it alone). It skips lift, waypoints and settle. The resolution order of home is in
[cli.md](../cli.md#home-resolve-save-and-move).

## 2. Gripper

```bash
rwrl gripper show
rwrl --yes gripper close
rwrl --yes gripper open
rwrl --yes gripper move 0.5
```

*You should see:* for each command a line such as
`gripper width 0.0800 m (1.00 of max_width_m 0.080), grasped=False, source FrankyGripper`, and
before each motion `target gripper close 0.0000; max delta 1.0000 norm`. Widths are normalized:
`0` closed, `1` open (`max_width_m`). Check that the measured width matches what you see; a
width reading alone does not prove a Hand is fitted.

## 3. Cameras

RealSense cameras need the `realsense` extra in the project environment:

<!-- tutorial-test: skip -->
```bash
python -m pip install -e '.[realsense]'
```

Then, with no recorder or actor holding the devices:

```bash
rwrl --station local/station.yaml camera show
rwrl --station local/station.yaml camera show --name wrist
rwrl --station local/station.yaml camera grab --name wrist --output local/camera-wrist
```

*You should see:* for each camera its configuration, `color (H, W, 3)` for the frames read, the
frame rate measured over 30 reads (`--frames N` changes that) and its health; exit code 1 when a
camera delivers no frames. `camera grab` saves `wrist-color.png` (plus `wrist-depth.png` with
depth) and `wrist.json` under `local/camera-wrist/`, and never overwrites an earlier grab, so
choose a new `--output` directory for the next one.

The image's `[H, W]` is the camera's `source_resolution`. A region of interest,
`preprocess.<camera>.roi: [top, left, height, width]` in pixels, is read off the saved colour
image; ROI, depth and resize settings are part of the policy's observation contract
([observation settings](10-hil-training.md#task-images-and-control-profiles)).

## 4. Stop and recover

```bash
rwrl stop
rwrl recover
```

*You should see:* `stop: ok` and `recover: ok`. `stop` stops the arm without asking; `recover`
clears arm errors after a reflex (see [03](03-franka-bringup.md#5-after-a-reflex-stop)). Both
use the same connection as `status`: on the direct route a second FCI connection cannot reach
an arm that a recorder or actor holds, so finish that process first, or use the physical stop.

## 5. Optional: task frame and workspace

A task frame and a workspace box in it are only needed for SpaceMouse control and workspace
holds. With the arm positioned by an operator (these commands never move it), capture three
points and save the frame:

```bash
rwrl --station local/station.yaml workspace capture --name frame-origin --output frame-points.yaml --save
rwrl --station local/station.yaml workspace capture --name frame-x --output frame-points.yaml --save
rwrl --station local/station.yaml workspace capture --name frame-y --output frame-points.yaml --save
rwrl --station local/station.yaml workspace frame set --name table \
    --from-points frame-points.yaml --origin frame-origin --x-axis frame-x --y-axis frame-y --save
rwrl --station local/station.yaml workspace frame show
```

Point files are relative to the station directory. Proposing workspace bounds from captured
points (`rwrl workspace set --frame table --from-points ...`) and the review steps are in the
[SpaceMouse commissioning guide](../spacemouse-commissioning.md#spacemouse-profiles-and-task-frame-workspace).

<details>
<summary>Rehearse these commands with a mock station</summary>

Write `local/mock-station.yaml` with this content:

```yaml
type: franka
name: mock_station
arm_backend: fake
claim: false
gripper:
  backend: fake
home_joints: [0.1, 0.0, 0.0, -1.5, 0.0, 1.5, 0.7]
```

Then, on the NUC (or any machine with the project environment), repository root:

```bash
rwrl --station local/mock-station.yaml --mock status
rwrl --station local/mock-station.yaml --mock home show
rwrl --station local/mock-station.yaml --mock --yes gripper close
```

```text
connection: direct fake
robot: mock_station (franka)
arm: FakeArm, ok=True, mode=idle, errors=[]
gripper width 0.0800 m (1.00 of max_width_m 0.080), grasped=unknown, source FakeGripper
cameras: none
teleop: none
```

```text
source: station home_joints
home (rad): [0.1000, 0.0000, 0.0000, -1.5000, 0.0000, 1.5000, 0.7000]
q (rad): [0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000]
delta (home - q, rad): [0.1000, 0.0000, 0.0000, -1.5000, 0.0000, 1.5000, 0.7000]
max delta: 1.500000 rad
```

```text
target gripper close 0.0000; max delta 1.0000 norm
gripper width 0.0000 m (0.00 of max_width_m 0.080), grasped=unknown, source FakeGripper
```

Without `--yes` and without a terminal, `home go` prints its target on stdout and
`error: refusing to move without --yes` on stderr, and exits 2.

</details>

## If it fails

- **Exit code 2 and `refusing to move without --yes`:** no terminal to ask on; add `--yes`
  after reading the printed target.
- **`home set requires --preset`:** give the preset before the command.
- **Bounds failure on `home set`:** the measured joints lie outside a joint limit; move the arm
  inside the limits, or fix the limits ([02](02-setup.md#4-check-limits-and-home)).
- **An error naming the arm, gripper or a camera:** run `rwrl doctor --station local/station.yaml`
  and look at which row is red, then the matching section of
  [rwrl doctor: red lights](../doctor.md#red-lights-what-to-do).

Next: [05 Teleop: GELLO](05-teleop-gello.md).
