# 4. Record demos

`examples/record_real_demos.py` builds the env exactly as the HIL actor does and records only the
steps where you intervene, into the demo pkl that BC, ACT and the HIL learners load. Read
[06 Intervening](06-intervening.md) first: it is the operator page for this step and for
[07](07-hil-training.md).

Start the daemon ([03](03-daemon.md)), then on the actor machine:

```bash
ENV_FLAGS="--config hil.yaml --teleop-device leader_arm --robot-teleop-action-mode joint_delta \
    --teleop-event-device keyboard"
python examples/record_real_demos.py $ENV_FLAGS --teleop-onset arm \
    --output-path demos/pick-and-place.pkl --num-successes 20
```

- `--teleop-device leader_arm` reads the GELLO that the daemon serves as the robot's `teleop:` part.
  `--robot-teleop-action-mode` declares that part before the remote robot connects; for a leader arm
  it only has to name a joint space, the env's `action_mode` decides what is emitted.
- `--teleop-event-device keyboard` gives the intervention toggle and the episode keys.
- `--teleop-onset arm` makes the toggle take effect only once the leader is within
  `--teleop-arm-tolerance-rad` (0.1) of the follower, so every demo starts from a still, aligned
  robot ([06](06-intervening.md#onset-arm-or-ramp)).
- The recorder takes its reward from your keys by default (`reward_from_teleop_events` is on for the
  recorder only; `--no-reward-from-teleop-events` turns it off): `s` stores the step as a terminal
  success (reward 1, `done` True) and ends the episode.
- `--max-episode-steps N` overrides the preset's `max_episode_steps` (200 steps = 20 s at 10 Hz) for
  the recording session.

## An episode

1. The env resets: the arm homes and the gripper opens.
2. Move the GELLO onto the follower's pose and press **Space**. Control starts once the leader is
   within 0.1 rad of the follower in every joint.
3. Drive the GELLO: reach the object, squeeze the trigger to close the gripper, carry the object to
   the bin, release the trigger.
4. While you still hold control, press **s**: the episode ends as a success and the arm homes.

**f** discards the episode, **r** discards it and starts the next at once, **Esc** ends it without a
success (discarded). Only successful episodes with at least one recorded step are kept; Ctrl-C stops
the session and still saves what was kept. Press `s` while you still hold control: the recorder
stores only intervened steps, so an `s` after you released the toggle ends the episode but its
terminal step is not in the demo, and the kept demo carries reward 0 throughout.

*You should see* one line per finished episode and a final save line (numbers are examples):

```text
[record] episode=0 steps=87 transitions=87 kept successes=1/20
[record] episode=1 steps=40 transitions=0 discarded (<reason>) successes=1/20
...
[record] saved 20 episodes (1734 transitions) to demos/pick-and-place.pkl
```

and two files, `demos/pick-and-place.pkl` and its contract sidecar `demos/pick-and-place.contract.json`
(action mode, widths, joint limits, scales, state fields). Every later loader reads the pkl together
with the sidecar and raises a `ValueError` naming the differing keys if its env does not match.
`--output-path` must be a new `.pkl` file. Vary the object's start pose across the demos.

*If not:*

- Episodes end before the task is done: raise `max_episode_steps` in `hil.yaml` (or pass
  `--max-episode-steps`).
- Space does nothing: the leader is further than 0.1 rad from the follower; the actor logs the
  per-joint error once per second. Move the leader onto the follower.
- A GELLO that is not read at all shows up as the daemon's `feedback:teleop` doctor check, or as a
  `DeviceFault` from the recorder.
- `--keep-syncing-steps` and `--start-tolerance-rad` are explained in
  [recording-demos.md](../recording-demos.md#recorder-flags).

> **joint_abs.** Record a separate set in the `joint_abs` mode with the `hil_abs.yaml` copy of the
> preset (`action_mode: joint_abs` and `joint_limits` set, see
> [02](02-station.md#the-env-preset-hilyaml)): it only loads into `joint_abs` runs.
>
> ```bash
> python examples/record_real_demos.py --config hil_abs.yaml \
>     --teleop-device leader_arm --robot-teleop-action-mode joint_abs --teleop-event-device keyboard \
>     --output-path demos/pick-and-place-abs.pkl --num-successes 20
> ```
>
> Before each episode the recorder reads the leader once and stops with the per-joint error when any
> joint is further than `--start-tolerance-rad` (0.8) from the follower: move the leader onto the
> follower and restart. With the default `ramp` onset the follower slews to the leader at
> `--teleop-ramp-step-rad` (0.05) per step; those steps are left out of the demos unless
> `--keep-syncing-steps`. In `joint_delta` the start check is skipped: the leader reading saturates at
> one `joint_scale_rad` per step.

More detail: [recording-demos.md](../recording-demos.md).

Next: [05 Base policy](05-base-policy.md).
