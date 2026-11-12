# 5. Teleop: GELLO

**Goal:** calibrate the GELLO, check its readings without moving the follower, then complete
one supervised cartesian session before collecting a dataset.

## Before you start

All commands run on the **NUC with the GELLO attached**, from the **repository root**, with
the **project environment active**. If the GELLO is on a separate host, run calibration and
input checks there; the motion session runs on the follower's NUC.

- Finish [03 Bring up the Franka](03-franka-bringup.md) and
  [04 Home, gripper, cameras](04-first-motion.md). Keep the physical stop accessible.
- Check [GELLO hardware and serial wiring](../gello.md#1-hardware). The station's
  `teleop.backend` must be `gello`; `teleop.options.port` must name the stable
  `/dev/serial/by-id/...` adapter, with the correct joint IDs and baud rate.
- Keep the commissioned `local/station.yaml` and `local/gello_cartesian.yaml` from 02–04.
  The session needs the learning dependencies, a reviewed home and limits, and no competing
  recorder or actor. Calibration needs exclusive access to the GELLO serial port.

## 1. Check access first

```bash
rwrl doctor --station local/station.yaml
```

*You should see:* the GELLO checks (`gello.*`) and the serial/group checks pass. If access or
latency is flagged, run the repair on this same machine, follow any replug/login instructions,
and rerun doctor:

```bash
rwrl doctor --station local/station.yaml --fix
```

For a red row, follow [serial, input groups and USB latency](../doctor.md#serial-input-groups-and-usb-latency)
before calibrating. Stop a process holding the serial port rather than opening it twice.

## 2. Calibrate and save the unit's signs and offsets

With any daemon using the GELLO stopped:

```bash
rwrl --station local/station.yaml teleop calibrate --backend gello --use
```

Check that the configured joint IDs correspond to the physical joints. For each sign check,
move only the requested joint in the follower's positive direction and press Enter. Then hold
the leader in the calibration pose and confirm; finish by holding the trigger fully open and
fully closed as prompted. Review the proposed station diff before accepting it.

*You should see:* calibrated signs, offsets, wrap centres and (when fitted) trigger endpoints;
`--use` writes these to the station's `teleop.options`. Without `--use`, the command prints the
YAML for review. A large pose residual means the pose or a sign needs checking; repeat this
step before motion. Offsets are solved on a quarter-turn grid; a residual above pi/8 is
reported. Wrap centres come from the station's joint-limit midpoints (or the alternate limits
YAML). Without limits they default to zero with a warning, which is wrong for Panda joint 6;
check the limits before accepting calibration. See [If it fails](#if-it-fails) for access failures.

| Calibration flag | Default / purpose |
|---|---|
| `--port` | station `teleop.options.port` |
| `--joint-ids` | station joint IDs, in leader joint order |
| `--gripper-id` | station gripper ID; absent means no trigger channel |
| `--baudrate` | station value, otherwise 57600; must match the servos |
| `--joint-signs` | station signs, otherwise all `+1` |
| `--calibration-pose` | seven-joint Franka pose `0 0 0 -pi/2 0 pi/2 0`, in radians |
| `--joint-limits-yaml` | alternate robot YAML for wrap centres; otherwise station `joint_limits` |
| `--skip-sign-check` | keep the initial signs; off by default |
| `--use` | save into the station after review; off by default |

The stored YAML fields and gripper options are in [the calibration reference](../gello.md#2-calibration).

## 3. Read without commanding the follower

If the station declares a daemon, restart it after calibration in a separate terminal on the
NUC, repository root, project environment active; keep that terminal running:

```bash
rwrl daemon --station local/station.yaml
```

*You should see:* the daemon listening at the station's configured address. For the direct
route, leave it stopped and use global `--direct` on the read command instead. Do not run a
daemon alongside a process that directly owns the same devices.

In the operator terminal:

```bash
rwrl --station local/station.yaml teleop read --duration-s 30
```

*You should see:* lines with `q=` (leader joints in radians), `gripper=` (normalized width,
1 open, 0 closed, or `none` without a channel), `intervening=` and fired `events=`. Check the
calibration pose and several other poses, one joint at a time. `--rate-hz` controls printed
samples (default 3 Hz); omitting `--duration-s` reads until Ctrl-C.

With `intervention: always`, `intervening` stays true. With a configured evdev
`event_device` and `intervention: toggle`, check that its toggle changes the state and its
label keys appear in `events`. This read command does not listen for the session's terminal
`t`/`h` keys; those come from the event source in the next step.

Without a daemon, the direct route opens only the GELLO, leaving the follower arm, gripper
and cameras unopened. If the station declares a daemon that is stopped, put global `--direct`
before `teleop` to read locally. `--with-follower` is available only through a running daemon:
it adds `follower_q` and joint-absolute `delta` without opening a second arm connection.
See [07](07-teleop-leader-follower.md#1-check-joint-space-alignment) for that comparison.
If readings fail or disagree with the physical joints, use [If it fails](#if-it-fails).

## 4. Complete one supervised session

On the follower's NUC, repository root, project environment active, with the arm clear and
the local preset's home and limits checked:

```bash
rwrl --station local/station.yaml --preset local/gello_cartesian.yaml teleop run --backend gello \
    --record demos/gello-check.pkl --num-successes 1
```

This starts its own supervisor and recorder. Review the motion confirmation: reset can home
the arm and operate the gripper. It selects `leader_cartesian`, an anchored cartesian mapping;
you do not need to match leader joints to follower joints. See [the operator sequence](../intervening.md#the-sequence).

1. Wait for reset to finish, with takeover released.
2. Press `t` to anchor the current leader pose to the follower's command pose; move gently.
3. Squeeze/release the trigger to close/open. If it disagrees with the last gripper command,
   match that state first to release the takeover latch.
4. Press `h` to hand back and hold measured pose; `t` takes over with a fresh anchor.
5. While intervening, press `s` to label success and include the successful terminal step in
   the human pkl. `f` labels failure, `r` discards and rerecords, Esc ends without success.
6. Labels enter session hold, freezing the last accepted target. `n` requests the next episode
   when the quota allows; `q` releases control and exits, including after the success quota.

*You should see:* the reset, takeover and hold behavior above, followed by a report directory
beside the output named `gello-check.session-<id>/`. It contains `resolved-config.yaml`,
`leader-robot.yaml` (a carrier with a fake arm and the calibrated GELLO teleop),
`leader-options.yaml`, `provenance.json` and the session reports. A kept successful episode
produces the pkl and its contract/archive companions; no kept success means no pkl.
If startup, hold or saving fails, follow [If it fails](#if-it-fails).

The between-episode neutral gate means **release takeover**, not align the joints or centre
the trigger. `--neutral-stable-s` defaults to 0.2 seconds. Common run options are `--record`
(required, new `.pkl` path), `--num-successes` (default 20), `--max-episodes` (optional attempt
cap) and `--events-file` (append-only JSONL, or `-` for piped stdin; otherwise terminal keys).
Use [08 Record demos](08-record-demos.md) for the full dataset and archive workflow.

## If it fails

Run `rwrl doctor --station local/station.yaml` first and find the red row in
[doctor's red lights](../doctor.md#red-lights-what-to-do).

- Serial access, event-device groups or slow FTDI reads:
  [serial, input groups and USB latency](../doctor.md#serial-input-groups-and-usb-latency).
- A daemon or another process owns the input/arm:
  [busy devices (BLOCKED)](../doctor.md#busy-devices-blocked). Finish the competing process.
- Wrong signs or pose residual: repeat calibration and the read check before moving.
- `WORKSPACE HOLD`: clear the request, press `h`, then `t` to re-anchor;
  see [how the robot follows you](../intervening.md#how-the-robot-follows-you).
- Missing preset or reused output: supply the commissioned `--preset` before `teleop` and a
  new `.pkl` path. Neither is repaired by changing hardware permissions.

Next: [06 Teleop: SpaceMouse](06-teleop-spacemouse.md).
