# Recording demos on the real robot

`examples/record_real_demos.py` records teleoperated demonstrations on the real robot into the
pkl format that `--demo-dataset-paths` loads (`hil_serl`, `residual_hil_serl`, `silri` and
`real_world_bc`). The logic lives in `rl_garden_real_world/demos/record.py`.

The recorder builds the env exactly as the HIL actor does (`build_hil_env(args, role="actor")`),
so it takes the same env, teleop and observation flags, and the same `--config` preset, as
`train_real_world.py hil_serl`. Pass the flags of the run that will load the demos: the recorded
observations and actions must match its env (the pkl's contract sidecar is checked on load).

## Commands

GELLO served by the robot daemon, `joint_abs` follower (the BC setting), keyboard for the buttons:

```bash
python examples/record_real_demos.py --config hil.yaml \
    --env-backend real_robot --real-robot.action-mode joint_abs \
    --teleop-device leader_arm --robot-teleop-action-mode joint_abs \
    --teleop-event-device keyboard \
    --output-path demos/task_20.pkl --num-successes 20
```

`hil.yaml` is the training preset (see the README's "Configuring the env"); `joint_limits` has to
come from it because the command line cannot express a list of pairs. Keys of the preset that are
not recorder fields (the algorithm's hyperparameters) are skipped.

Other motion devices work the same way: `--teleop-device gello` with `--teleop-options-yaml
gello.yaml` for a GELLO on this machine, `--teleop-device spacemouse` for a cartesian env,
`--teleop-device leader_cartesian` on `franka_real`. The GELLO setup itself is in
[gello.md](gello.md), the devices in [teleop.md](teleop.md).

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
| `--max-episode-steps` | unset | episode step cap; replaces `--real-robot.max-episode-steps` or `--franka-real.max-episode-steps` when set |
| `--start-tolerance-rad` | 0.8 | `joint_abs`: refuse to start an episode while the leader is further than this from the follower in any joint |
| `--keep-syncing-steps` | off | also record the ramp-onset steps (see below) |

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
   a zero arm action in `joint_delta` and `cartesian_delta`, the follower's current joint
   positions in `joint_abs`, and the last executed gripper action (open after a reset).
4. A step is recorded only when it was intervened. Steps the ramp onset rate-limited carry
   `teleop_syncing` and are left out unless `--keep-syncing-steps` is given.
5. The episode ends when the env terminates or truncates: the operator's episode-end button
   (Esc on the keyboard) truncates, the step cap truncates, a reward classifier terminates.

Episode outcome, from the operator's events (any event path: keyboard, SpaceMouse buttons, the
GELLO's evdev device):

| Event | Effect |
| --- | --- |
| `success` (`s`) or a positive reward | the episode counts as a success |
| `failure` (`f`) | the episode is discarded, even after a success |
| `rerecord` (`r`) | the episode is discarded at once and the next one starts |

Only successful episodes with at least one recorded step are kept; each finished episode is
printed with its outcome. Ctrl-C stops the session and still saves the episodes kept so far.

In `joint_delta` the start check is skipped: the leader reading saturates at one
`joint_scale_rad` per step, so it cannot measure a larger gap, and the same saturation already
limits how fast the follower moves towards the leader.

## Output

The pkl is a list of transition dicts, the format the hil_serl learner's demo snapshots use:
`obs` and `next_obs` as the env returned them, `action` the executed action
(`info["intervene_action"]`, `(1, A)`), `reward`, `done` (`terminated`), and `grasp_penalty`
when `--use-grasp-penalty` is on. The sidecar `<stem>.contract.json` records the env's action
contract (action mode, widths, joint limits, scales, state fields); loading the pkl into an env
with a different contract raises a `ValueError` naming the differing keys.
