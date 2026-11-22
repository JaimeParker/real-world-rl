.. _en-recording-demos-recording-demos-on-the-real-robot:

Recording demos on the real robot
================================================================================

``examples/record_real_demos.py`` archives complete teleoperated episodes on the real robot to
SQLite and exports successful human demonstrations to the pkl format accepted by
``--demo-dataset-paths`` (``hil_serl``, ``residual_hil_serl``, ``silri``, ``real_world_bc``). BC/HIL loaders
also accept ``.trajectory.sqlite3`` and ``.trajectory.pt``. ACT's trajectory trainer uses pkl. Residual training also needs
``--allow-base-less-demos`` for these recordings, because the recorder does not
compute ``base_actions``/``next_base_actions``; see :doc:`Residual RL <residual>`.
The logic lives in ``real_world_rl/demos/record.py``.

The recorder builds the env exactly as the HIL actor does (``build_hil_env(args, role="actor")``),
so it takes the same env, teleop and observation flags, and the same ``--config`` preset, as
``train_real_world.py hil_serl``. Pass the flags of the run that will load the demos: the recorded
observations and actions must match its env (the pkl's contract sidecar is checked on load).
\ :doc:`Tutorial 08 <../tutorials/record>`\  walks through the first recording session, and
the \ :doc:`operator page <intervening>`\  lists the keys, the onset and what is recorded.

For SpaceMouse on a commissioned ``real_robot`` station, the supervised
\ :ref:`rwrl teleop run workflow <en-spacemouse-commissioning-supervised-recording-sessions>`\  adds explicit
episode-end holding. A label seals the trajectory; puck motion then has no
effect until ``n`` requests a new episode and the puck is stably neutral. ``q``
releases control, including after the episode budget is reached. Repeated
takeover/hand-back inside a trajectory remains supported and does not invoke
this neutral gate. After the gate, recording and actor loops acquire a fresh observation;
the first archived frame and policy input are from after neutral confirmation, with image
stack histories refilled from that fresh frame. The Python recorder's default remains ``episode_end_behavior: auto``; select ``hold`` explicitly when using it directly.

Use the \ :doc:`SpaceMouse acceptance checklist <testing/spacemouse-control-session>`\ 
and its record template for direction, workspace, holding and normal-stop
verification in both modes. Successful demonstrations alone do not establish
hardware acceptance or normal-stop stability.

.. _en-recording-demos-commands:

Commands
--------------------------------------------------------------------------------

GELLO served by the robot daemon, ``joint_delta`` follower (the daemon-served joint-space route),
keyboard for the buttons:

.. code-block:: bash

   rwrl record --config hil.yaml \
       --teleop-device leader_arm --robot-teleop-action-mode joint_delta \
       --teleop-event-device keyboard --teleop-onset arm \
       --output-path demos/task_20.pkl --num-successes 20

A ``joint_abs`` set (for a ``joint_abs`` pipeline) adds ``--real_robot.action_mode joint_abs --robot-teleop-action-mode joint_abs`` and needs ``joint_limits`` in the preset.

``hil.yaml`` is a copy of the training preset
\ |reference-link-1|_\ ; ``joint_limits``
has to come from it because the command line cannot express a list of pairs. Keys of the preset that are
not recorder fields (the algorithm's hyperparameters) are skipped.

Other motion devices work the same way: ``--teleop-device gello`` with ``--teleop-options-yaml gello.yaml`` for a GELLO on this machine, ``--teleop-device leader_cartesian`` for a GELLO driving a
``cartesian_delta`` arm (``real_robot`` or
``franka_real``; \ |reference-link-2|_\ 
is a single-arm 50 Hz preset, its tracker flags ``--teleop-lead-cap-m``, ``--teleop-lead-cap-rad``,
``--teleop-target-speed-m-s``, ``--teleop-target-speed-rad-s`` are in
\ :ref:`teleop.md <en-teleop-leader-arm-on-a-cartesian-robot-leader_cartesian>`\ ), and
``--teleop-device spacemouse --teleop-event-device stdin`` for a SpaceMouse driving a
``cartesian_delta`` arm. Demos recorded with ``leader_cartesian`` or ``spacemouse`` carry the tracker in
the experiment contract (``teleop_control``), so a policy trained on them is rejected by a deploy
built with a ``--teleop-device`` without it. The GELLO setup itself is in \ :doc:`gello.md <gello>`\ , the
devices in \ :doc:`teleop.md <teleop>`\ .

SpaceMouse tracker-mode flags (``--spacemouse-mode tracker``, the default; details in \ :ref:`teleop.md <en-teleop-spacemouse>`\ ):

.. list-table::
   :header-rows: 1

   * - Flag
     - Default
     - What
   * - ``--teleop-device spacemouse``
     - 
     - the SpaceMouse on this machine; button 0 takes over and hands back, button 1 toggles the gripper (starting from its last command), the puck is a velocity
   * - ``--teleop-event-device stdin`` (or ``keyboard``)
     - 
     - the outcome keys ``s``/``f``/``r``, Esc, ``q``; its ``t``/``h`` are ignored (button 0 is the toggle)
   * - ``--teleop-target-speed-m-s``, ``--teleop-target-speed-rad-s``
     - 0.05, 0.3
     - the speed at full puck deflection
   * - ``--teleop-lead-cap-m``, ``--teleop-lead-cap-rad``
     - 0.05, 0.3
     - how far the command may lead the arm; releasing the puck stops it within this
   * - ``--real-robot.xyz-scale-m``, ``--real-robot.rot-scale-rad``
     - 0.02, 0.1
     - must be at least the lead caps: pass 0.05 and 0.3
   * - ``--teleop-options-yaml`` with ``device_index``, ``deadzone``, ``invert_axes``
     - 0, 0.05, ``[]``
     - which SpaceMouse, the per-axis rest-noise deadzone (of full scale), the output axes (0-5) to flip

The operator keys of ``--teleop-event-device keyboard`` (this machine's desktop session) and
``--teleop-event-device stdin`` (the terminal running the recorder, also over SSH): ``t`` takes over,
``h`` hands back, ``s`` success, ``f`` failure, ``r`` rerecord, Esc ends the episode, ``q`` ends the episode
and stops the recorder after it.

Two arms (``--env-backend real_robot_bimanual``): ``--teleop-device bimanual --teleop-arm-device leader_arm`` (or ``gello``, or ``robot``) reads one leader per arm, and the recorder works per arm.
The hold action is each arm's own (in ``joint_abs``, every follower's joints on its own limits),
the episode-start check compares every leader with its own follower and names the arm furthest
apart, and ``--max-episode-steps`` replaces ``--real_robot_bimanual.max_episode_steps``. A step is
recorded when either operator intervenes; the other arm is then held still. The commands are in
\ :ref:`bimanual.md <en-bimanual-teleop-and-demo-recording>`\ .

Then train with the demos:

.. code-block:: bash

   rwrl train hil_serl --role learner --config hil.yaml \
       --demo-dataset-paths demos/task_20.pkl ...

.. _en-recording-demos-recorder-flags:

Recorder flags
--------------------------------------------------------------------------------

.. list-table::
   :header-rows: 1

   * - Flag
     - Default
     - Meaning
   * - ``--output-path``
     - required
     - the demo pkl (must end in ``.pkl`` and not exist); ``<stem>.contract.json`` is written next to it
   * - ``--num-successes``
     - 20
     - stop after this many kept episodes
   * - ``--max-episodes``
     - unset
     - stop after this many completed attempts, including failure, rerecord and timeout; the first of this limit and ``--num-successes`` stops recording
   * - ``--max-episode-steps``
     - unset
     - episode step cap; replaces ``--real_robot.max_episode_steps``, ``--real_robot_bimanual.max_episode_steps`` or ``--franka_real.max_episode_steps`` when set
   * - ``--start-tolerance-rad``
     - 0.8
     - ``joint_abs``: refuse to start an episode while the leader is further than this from the follower in any joint
   * - ``--keep-syncing-steps``
     - off
     - include leader catch-up rows as ordinary supervised pkl steps; SQLite retains their syncing marker
   * - ``--export-trajectory-pt``
     - off
     - additionally export the complete archive as portable ``.trajectory.pt``; SQLite remains the recovery source
   * - ``--reward-from-teleop-events``
     - on (recorder only; training defaults to off)
     - the success key stores its step as a terminal success (reward ``--teleop-success-reward``, 1.0; ``done`` True) and ends the episode; ``--no-reward-from-teleop-events`` keeps the env/task/classifier reward and termination; on a bare ``real_robot`` env these are reward 0 and ``done`` False

``--spacemouse-mode serl`` instead uses raw motion/button takeover, release to return
control, and held-button gripper commands; it requires the configured SERL xyz/RPY safety boxes.
The tracker lead-cap and target-speed rules above do not apply to this mode.

``--control-hz`` paces the loop as it paces the actor. ``--fwbw`` is not supported.

.. _en-recording-demos-what-happens-in-an-episode:

What happens in an episode
--------------------------------------------------------------------------------

1. The env resets (the robot homes, the gripper opens).
2. ``joint_abs`` only: the recorder reads the leader once and compares it with the follower's
   ``joint_pos``. When any joint is further apart than ``--start-tolerance-rad``, it stops with the
   per-joint error: move the leader onto the follower's pose and restart. Below the tolerance the
   onset policy (``--teleop-onset``) handles the takeover: ``ramp`` moves the follower to the leader
   at ``--teleop-ramp-step-rad`` per step, ``arm`` waits until the leader is within
   ``--teleop-arm-tolerance-rad``. The read goes through the motion device with the toggle
   released, so the onset policy does not hide the gap and no event-device button is consumed.
3. Every step the recorder sends a hold action; while the operator intervenes the intervention
   wrapper executes the operator's action instead. The hold action keeps the robot where it is:
   a zero arm action in ``joint_delta`` (and on two ``cartesian_delta`` arms), the follower's current
   joint positions in ``joint_abs``, and on one ``cartesian_delta`` arm the action back onto a fixed
   hold pose (sampled after the reset and at every hand-back; with ``leader_cartesian`` the source's
   own tracker, see \ :ref:`teleop.md <en-teleop-mapping>`\ ), so the arm does not drift. An arm further
   from the hold pose than one step (``--real-robot.xyz-scale-m``/``--real-robot.rot-scale-rad``)
   gets a saturated action and returns one step at a time. The gripper dims hold the last
   executed gripper action (open after a reset), one per arm: the grippers are the last
   ``num_arms`` action dims (the action contract's layout). ``run_recorder(args, hold_action_fn_factory=...)`` injects another hold policy (called with the built env).
4. Every step is archived. The pkl view selects intervened steps; leader catch-up carries
   ``teleop_syncing`` in both joint modes and is excluded unless ``--keep-syncing-steps`` is set.
   Archive-derived training views still exclude syncing. Human masks select expert dimensions,
   while idle-hold supplements have no expert label.
5. The episode ends when the env terminates or truncates: the operator's episode-end button
   (Esc on the keyboard) truncates, the step cap truncates, the success key terminates (with the
   default ``--reward-from-teleop-events``), a reward classifier terminates. Press the success key
   while still intervening: only intervened steps are recorded, so the terminal step must be one.

Episode outcome, from the operator's events (any event path: keyboard, terminal, the GELLO's
evdev device; the SpaceMouse buttons raise none):

.. list-table::
   :header-rows: 1

   * - Event
     - Effect
   * - ``success`` (``s``) or an explicit environment/classifier success label
     - the episode counts as successful unless a failure vetoes it; positive reward alone is not a success label
   * - ``failure`` (``f``)
     - the episode is discarded, even after a success
   * - ``rerecord`` (``r``)
     - the episode is discarded at once and the next one starts

Only successful episodes with at least one recorded step are kept; each finished episode is
printed with its outcome. Ctrl-C stops the session and still saves the episodes kept so far; the
``q`` key (``operator_quit``) ends the current episode, which is judged as usual, and then stops the
session the same way.

In ``joint_delta`` the start check is skipped: the leader reading saturates at one
``joint_scale_rad`` per step, so it cannot measure a larger gap, and the same saturation already
limits how fast the follower moves towards the leader.

.. _en-recording-demos-output:

Output
--------------------------------------------------------------------------------

The pkl contains successful human transition dicts with ``obs``, ``next_obs``, executed ``action``,
``reward``, ``done`` (``terminated``), optional ``grasp_penalty``, policy proposals and human/override
masks. Versioned records also retain episode/step identity and training-segment boundaries, so
filtering cannot invent adjacency or transfer terminal rewards to earlier human steps.
``<stem>.contract.json`` records action and experiment compatibility; retain it with the data.
The compared experiment fields cover task, control, observation layout, preprocessing and
teleop part selection/idle behavior. Reward sources and station-owned reset facts are provenance.

``<stem>.trajectory.sqlite3`` is the complete durable archive, including failed, rerecorded,
syncing and interrupted attempts. Only the pkl view drops unsuccessful episodes. If nothing is
kept, no pkl is written. Reopening an existing archive, while the pkl path is still new, marks
unfinished episodes interrupted and assigns new IDs to future attempts. ``--export-trajectory-pt``
also writes ``<stem>.trajectory.pt``. Durability begins when append returns and uses SQLite WAL
``synchronous=NORMAL``; the newest commits can be lost on OS crash or power failure.
Full recovery, masks and training-view rules: \ :doc:`Real-world experiments <real-world-experiments>`\ .

.. _en-recording-demos-single-attempt-commissioning:

Single-attempt commissioning
--------------------------------------------------------------------------------

Use ``--max-episodes 1`` to end after one attempt, even when no successful demo is kept.
The recorder closes device control before scanning the archive or exporting pkl/PT files.
It preserves the full SQLite archive on success, failure, rerecord, timeout and interruption;
only successful human demonstrations produce a pkl. Restarting the command begins another
reset, including home motion and gripper opening. Verify the station and workspace before
each restart. The default, with no attempt limit, continues until ``--num-successes`` is met.

``rwrl archive inspect <archive>`` (offline, no station or robot) reads the archive read-only and,
for a single-arm state with ``tcp_pose``, reports per human_action_mask segment the hold drift (also
over the first ``--drift-window-s``), the residual and the take-over jump in mm, plus the settle time
after each ``--stop-time-s``; ``--measurements PATH`` writes them as the test pages' ``measurements.csv``
rows and ``--csv PATH`` exports the steps. With ``--stop-time-s`` it exits 0 when every stop settled,
3 when one did not and 4 when a stop time has no spanning TCP episode
(\ :ref:`cli.md <en-cli-commands>`\ ). ``python examples/inspect_teleop_archive.py`` is an alias.

See the \ :doc:`hardware session checklist index <testing/index>`\  for the English primary
test pages, record CSV conventions and evidence requirements. Fill hardware results on site.

.. |reference-link-1| replace:: ``examples/presets/hil_single_arm.yaml``
.. _reference-link-1: https://github.com/JaimeParker/real-world-rl/blob/main/examples/presets/hil_single_arm.yaml

.. |reference-link-2| replace:: ``examples/presets/gello_cartesian.yaml``
.. _reference-link-2: https://github.com/JaimeParker/real-world-rl/blob/main/examples/presets/gello_cartesian.yaml
