# Recording demos on the real robot

`examples/record_real_demos.py` archives complete teleoperated episodes on the real robot to
SQLite and exports successful human demonstrations to the pkl format accepted by
`--demo-dataset-paths` (`hil_serl`, `residual_hil_serl`, `silri`, `real_world_bc`). BC/HIL loaders
also accept `.trajectory.sqlite3` and `.trajectory.pt`. ACT's trajectory trainer uses pkl.
The logic lives in `real_world_rl/demos/record.py`.

The recorder builds the env exactly as the HIL actor does (`build_hil_env(args, role="actor")`),
so it takes the same env, teleop and observation flags, and the same `--config` preset, as
`train_real_world.py hil_serl`. Pass the flags of the run that will load the demos: the recorded
observations and actions must match its env (the pkl's contract sidecar is checked on load).
[Tutorial 04](tutorial/04-record-demos.md) walks through the first recording session, and
[tutorial 06](tutorial/06-intervening.md) is the operator's page (keys, onset, what is recorded).

For SpaceMouse on a commissioned `real_robot` station, the supervised
[`rwrl teleop run` workflow](spacemouse-commissioning.md#supervised-recording-sessions) adds explicit
episode-end holding. A label seals the trajectory; puck motion then has no
effect until `n` requests a new episode and the puck is stably neutral. `q`
releases control, including after the episode budget is reached. Repeated
takeover/hand-back inside a trajectory remains supported and does not invoke
this neutral gate. After the gate, recording and actor loops acquire a fresh observation;
the first archived frame and policy input are from after neutral confirmation, with image
stack histories refilled from that fresh frame. The Python recorder's default remains `episode_end_behavior:
auto`; select `hold` explicitly when using it directly.

Use the [SpaceMouse acceptance checklist](testing/spacemouse-control-session.md)
and its record template for direction, workspace, holding and normal-stop
verification in both modes. Successful demonstrations alone do not establish
hardware acceptance or normal-stop stability.

## Commands

GELLO served by the robot daemon, `joint_delta` follower (the tutorial's setting for every stage),
keyboard for the buttons:

```bash
python examples/record_real_demos.py --config hil.yaml \
    --teleop-device leader_arm --robot-teleop-action-mode joint_delta \
    --teleop-event-device keyboard --teleop-onset arm \
    --output-path demos/task_20.pkl --num-successes 20
```

A `joint_abs` set (for a `joint_abs` pipeline) adds `--real_robot.action_mode joint_abs
--robot-teleop-action-mode joint_abs` and needs `joint_limits` in the preset.

`hil.yaml` is a copy of the training preset
[`examples/presets/hil_single_arm.yaml`](../examples/presets/hil_single_arm.yaml); `joint_limits`
has to come from it because the command line cannot express a list of pairs. Keys of the preset that are
not recorder fields (the algorithm's hyperparameters) are skipped.

Other motion devices work the same way: `--teleop-device gello` with `--teleop-options-yaml
gello.yaml` for a GELLO on this machine, `--teleop-device leader_cartesian` for a GELLO driving a
`cartesian_delta` arm (`real_robot` or
`franka_real`; [`examples/presets/gello_cartesian.yaml`](../examples/presets/gello_cartesian.yaml)
is a single-arm 50 Hz preset, its tracker flags `--teleop-lead-cap-m`, `--teleop-lead-cap-rad`,
`--teleop-target-speed-m-s`, `--teleop-target-speed-rad-s` are in
[teleop.md](teleop.md#leader-arm-on-a-cartesian-robot-leader_cartesian)), and
`--teleop-device spacemouse --teleop-event-device stdin` for a SpaceMouse driving a
`cartesian_delta` arm. Demos recorded with `leader_cartesian` or `spacemouse` carry the tracker in
the experiment contract (`teleop_control`), so a policy trained on them is rejected by a deploy
built with a `--teleop-device` without it. The GELLO setup itself is in [gello.md](gello.md), the
devices in [teleop.md](teleop.md).

SpaceMouse flags (details in [teleop.md](teleop.md#spacemouse)):

| Flag | Default | What |
| --- | --- | --- |
| `--teleop-device spacemouse` | | the SpaceMouse on this machine; button 0 takes over and hands back, button 1 toggles the gripper (starting from its last command), the puck is a velocity |
| `--teleop-event-device stdin` (or `keyboard`) | | the outcome keys `s`/`f`/`r`, Esc, `q`; its `t`/`h` are ignored (button 0 is the toggle) |
| `--teleop-target-speed-m-s`, `--teleop-target-speed-rad-s` | 0.05, 0.3 | the speed at full puck deflection |
| `--teleop-lead-cap-m`, `--teleop-lead-cap-rad` | 0.05, 0.3 | how far the command may lead the arm; releasing the puck stops it within this |
| `--real-robot.xyz-scale-m`, `--real-robot.rot-scale-rad` | 0.02, 0.1 | must be at least the lead caps: pass 0.05 and 0.3 |
| `--teleop-options-yaml` with `device_index`, `deadzone`, `invert_axes` | 0, 0.05, `[]` | which SpaceMouse, the per-axis rest-noise deadzone (of full scale), the output axes (0-5) to flip |

The operator keys of `--teleop-event-device keyboard` (this machine's desktop session) and
`--teleop-event-device stdin` (the terminal running the recorder, also over SSH): `t` takes over,
`h` hands back, `s` success, `f` failure, `r` rerecord, Esc ends the episode, `q` ends the episode
and stops the recorder after it.

Two arms (`--env-backend real_robot_bimanual`): `--teleop-device bimanual --teleop-arm-device
leader_arm` (or `gello`, or `robot`) reads one leader per arm, and the recorder works per arm.
The hold action is each arm's own (in `joint_abs`, every follower's joints on its own limits),
the episode-start check compares every leader with its own follower and names the arm furthest
apart, and `--max-episode-steps` replaces `--real_robot_bimanual.max_episode_steps`. A step is
recorded when either operator intervenes; the other arm is then held still. The commands are in
[bimanual.md](bimanual.md#teleop-and-demo-recording).

Then train with the demos:

```bash
python examples/train_real_world.py hil_serl --role learner --config hil.yaml \
    --demo-dataset-paths demos/task_20.pkl ...
```

## Recorder flags

| Flag | Default | Meaning |
| --- | --- | --- |
| `--output-path` | required | the demo pkl (must end in `.pkl` and not exist); `<stem>.contract.json` is written next to it |
| `--num-successes` | 20 | stop after this many kept episodes |
| `--max-episodes` | unset | stop after this many completed attempts, including failure, rerecord and timeout; the first of this limit and `--num-successes` stops recording |
| `--max-episode-steps` | unset | episode step cap; replaces `--real_robot.max_episode_steps`, `--real_robot_bimanual.max_episode_steps` or `--franka_real.max_episode_steps` when set |
| `--start-tolerance-rad` | 0.8 | `joint_abs`: refuse to start an episode while the leader is further than this from the follower in any joint |
| `--keep-syncing-steps` | off | include leader catch-up rows as ordinary supervised pkl steps; SQLite retains their syncing marker |
| `--export-trajectory-pt` | off | additionally export the complete archive as portable `.trajectory.pt`; SQLite remains the recovery source |
| `--reward-from-teleop-events` | on (recorder only; training defaults to off) | the success key stores its step as a terminal success (reward `--teleop-success-reward`, 1.0; `done` True) and ends the episode; `--no-reward-from-teleop-events` keeps the env's own reward (0 on `real_robot`), so a kept success is stored with reward 0 and `done` False |

`--control-hz` paces the loop as it paces the actor. `--fwbw` is not supported.

## What happens in an episode

1. The env resets (the robot homes, the gripper opens).
2. `joint_abs` only: the recorder reads the leader once and compares it with the follower's
   `joint_pos`. When any joint is further apart than `--start-tolerance-rad`, it stops with the
   per-joint error: move the leader onto the follower's pose and restart. Below the tolerance the
   onset policy (`--teleop-onset`) handles the takeover: `ramp` moves the follower to the leader
   at `--teleop-ramp-step-rad` per step, `arm` waits until the leader is within
   `--teleop-arm-tolerance-rad`. The read goes through the motion device with the toggle
   released, so the onset policy does not hide the gap and no event-device button is consumed.
3. Every step the recorder sends a hold action; while the operator intervenes the intervention
   wrapper executes the operator's action instead. The hold action keeps the robot where it is:
   a zero arm action in `joint_delta` (and on two `cartesian_delta` arms), the follower's current
   joint positions in `joint_abs`, and on one `cartesian_delta` arm the action back onto a fixed
   hold pose (sampled after the reset and at every hand-back; with `leader_cartesian` the source's
   own tracker, see [teleop.md](teleop.md#mapping)), so the arm does not drift. An arm further
   from the hold pose than one step (`--real-robot.xyz-scale-m`/`--real-robot.rot-scale-rad`)
   gets a saturated action and returns one step at a time. The gripper dims hold the last
   executed gripper action (open after a reset), one per arm: the grippers are the last
   `num_arms` action dims (the action contract's layout). `run_recorder(args,
   hold_action_fn_factory=...)` injects another hold policy (called with the built env).
4. Every step is archived. The pkl view selects intervened steps; leader catch-up carries
   `teleop_syncing` in both joint modes and is excluded unless `--keep-syncing-steps` is set.
   Archive-derived training views still exclude syncing. Human masks select expert dimensions,
   while idle-hold supplements have no expert label.
5. The episode ends when the env terminates or truncates: the operator's episode-end button
   (Esc on the keyboard) truncates, the step cap truncates, the success key terminates (with the
   default `--reward-from-teleop-events`), a reward classifier terminates. Press the success key
   while still intervening: only intervened steps are recorded, so the terminal step must be one.

Episode outcome, from the operator's events (any event path: keyboard, terminal, the GELLO's
evdev device; the SpaceMouse buttons raise none):

| Event | Effect |
| --- | --- |
| `success` (`s`) or an explicit environment/classifier success label | the episode counts as successful unless a failure vetoes it; positive reward alone is not a success label |
| `failure` (`f`) | the episode is discarded, even after a success |
| `rerecord` (`r`) | the episode is discarded at once and the next one starts |

Only successful episodes with at least one recorded step are kept; each finished episode is
printed with its outcome. Ctrl-C stops the session and still saves the episodes kept so far; the
`q` key (`operator_quit`) ends the current episode, which is judged as usual, and then stops the
session the same way.

In `joint_delta` the start check is skipped: the leader reading saturates at one
`joint_scale_rad` per step, so it cannot measure a larger gap, and the same saturation already
limits how fast the follower moves towards the leader.

## Output

The pkl contains successful human transition dicts with `obs`, `next_obs`, executed `action`,
`reward`, `done` (`terminated`), optional `grasp_penalty`, policy proposals and human/override
masks. Versioned records also retain episode/step identity and training-segment boundaries, so
filtering cannot invent adjacency or transfer terminal rewards to earlier human steps.
`<stem>.contract.json` records action and experiment compatibility; retain it with the data.
The compared experiment fields cover task, control, observation layout, preprocessing and
teleop part selection/idle behavior. Reward sources and station-owned reset facts are provenance.

`<stem>.trajectory.sqlite3` is the complete durable archive, including failed, rerecorded,
syncing and interrupted attempts. Only the pkl view drops unsuccessful episodes. If nothing is
kept, no pkl is written. Reopening an existing archive, while the pkl path is still new, marks
unfinished episodes interrupted and assigns new IDs to future attempts. `--export-trajectory-pt`
also writes `<stem>.trajectory.pt`. Durability begins when append returns and uses SQLite WAL
`synchronous=NORMAL`; the newest commits can be lost on OS crash or power failure.
Full recovery, masks and training-view rules: [Real-world experiments](real-world-experiments.md).

## Single-attempt commissioning

Use `--max-episodes 1` to end after one attempt, even when no successful demo is kept.
The recorder closes device control before scanning the archive or exporting pkl/PT files.
It preserves the full SQLite archive on success, failure, rerecord, timeout and interruption;
only successful human demonstrations produce a pkl. Restarting the command begins another
reset, including home motion and gripper opening. Verify the station and workspace before
each restart. The default, with no attempt limit, continues until `--num-successes` is met.

`examples/inspect_teleop_archive.py <archive>` reads the archive read-only and, for a single-arm
state with `tcp_pose`, reports per human_action_mask segment the hold drift (also over the first
`--drift-window-s`), the residual and the take-over jump in mm, plus the settle time after each
`--stop-time-s`; `--measurements PATH` writes them as the test pages' `measurements.csv` rows.

See the [hardware session checklist index](testing/README.md) for the English primary
test pages, record CSV conventions and evidence requirements. Fill hardware results on site.
