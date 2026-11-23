.. _en-recording-demos-recording-demos-on-the-real-robot:

Recording demos on the real robot
================================================================================

``rwrl record`` (``examples/record_real_demos.py`` is the same entry point) writes every teleoperated episode on the real robot into a demo dataset directory (``--output-dir``):
one subdirectory per episode, written when the episode ends, so memory only ever holds the episode being recorded. Failed, re-recorded and interrupted episodes are stored too, flagged with a reason.
The training commands' ``--demo-dataset-paths`` read this directory directly (``hil_serl``, ``residual_hil_serl``, ``silri``, ``real_world_bc``) and take the
human-controlled steps of its kept episodes, rebuilding the image history to the training run's ``obs.frame_stack``; old pkls, ``.trajectory.sqlite3`` and ``.trajectory.pt`` still load.
rl-garden's ACT trajectory trainer and ``deploy-act`` still read a pkl plus sidecar, so export one first with ``rwrl dataset export --format pkl`` (see \ :ref:`Export <en-recording-demos-export>`\ ). Residual training also needs
``--allow-base-less-demos`` for these recordings, because the recorder does not
compute ``base_actions``/``next_base_actions``; see :doc:`Residual RL <residual>`.
The logic lives in ``real_world_rl/demos/record.py``.

The recorder builds the env exactly as the HIL actor does (``build_hil_env(args, role="actor")``),
so it takes the same env, teleop and observation flags, and the same ``--config`` preset, as
``train_real_world.py hil_serl``. Pass the flags of the run that will load the demos: the recorded
observations and actions must match its env (the dataset header's ``action_contract`` in ``dataset.json`` is checked on load; for an old pkl, its contract sidecar).
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
       --output-dir demos/task_20 --num-successes 20

A ``joint_abs`` set (for a ``joint_abs`` pipeline) adds ``--real_robot.action_mode joint_abs --robot-teleop-action-mode joint_abs`` and needs ``joint_limits`` in the preset.

``hil.yaml`` is a copy of the training preset
\ |reference-link-1|_\ ; ``joint_limits``
has to come from it because the command line cannot express a list of pairs. Keys of the preset that are
not recorder fields (the algorithm's hyperparameters) are skipped.

Other motion devices work the same way: ``--teleop-device gello`` with ``--teleop-options-yaml gello.yaml`` for a GELLO on this machine, ``--teleop-device leader_cartesian`` for a GELLO driving a
``cartesian_delta`` arm (``real_robot``; \ |reference-link-2|_\
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
       --demo-dataset-paths demos/task_20 ...

.. _en-recording-demos-recorder-flags:

Recorder flags
--------------------------------------------------------------------------------

.. list-table::
   :header-rows: 1

   * - Flag
     - Default
     - Meaning
   * - ``--output-dir``
     - required
     - the demo dataset directory. A new directory is created; an existing dataset is appended to, provided its header matches this run's env, otherwise the recorder fails and asks for a new directory. A name ending in ``.pkl`` or pointing at a file is refused
   * - ``--num-successes``
     - 20
     - stop once the dataset holds this many kept episodes; episodes already kept when appending count
   * - ``--max-episodes``
     - unset
     - stop after this many completed attempts, including failure, rerecord and timeout; the first of this limit and ``--num-successes`` stops recording
   * - ``--max-episode-steps``
     - unset
     - episode step cap; replaces ``--real_robot.max_episode_steps``, ``--real_robot_bimanual.max_episode_steps`` when set
   * - ``--start-tolerance-rad``
     - 0.8
     - ``joint_abs``: refuse to start an episode while the leader is further than this from the follower in any joint
   * - ``--keep-syncing-steps``
     - off
     - include leader catch-up rows as ordinary supervised training rows; the dataset always retains their syncing marker, and the choice is written to the dataset header and followed when training loads it
   * - ``--record-video-dir``
     - not available
     - the recorder refuses it (training keeps it): the dataset already writes a ``video/<rgb key>.mp4`` review video in each episode directory
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
4. Every step is written to the dataset (idle hold steps too). The training view selects intervened steps; leader catch-up carries
   ``teleop_syncing`` in both joint modes and is excluded unless ``--keep-syncing-steps`` is set.
   Dataset-derived training views still exclude syncing. Human masks select expert dimensions,
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
     - the episode is flagged discarded (``failure event``), even after a success
   * - ``rerecord`` (``r``)
     - the episode ends at once, flagged discarded (``rerecord``), and the next one starts

Every episode is stored in the dataset; only successful episodes with at least one human-controlled step are flagged kept, and the rest are flagged discarded with a reason (table below). Each finished episode is
printed with its outcome. Ctrl-C stops the session: episodes already written are unaffected, and the interrupted episode, if it has at least one step, is stored as discarded (``interrupted``) after the environment has released the robot; the
``q`` key (``operator_quit``) ends the current episode, which is judged as usual, and then stops the
session the same way.

.. list-table::
   :header-rows: 1

   * - discard_reason
     - Meaning
   * - ``rerecord``
     - the operator pressed the rerecord key
   * - ``failure event``
     - a failure event fired in the episode
   * - ``no success``
     - the episode ended without a success outcome (for example Esc or the step cap)
   * - ``no intervened steps``
     - successful, but with no human-controlled step, so it cannot be a demonstration
   * - ``interrupted``
     - Ctrl-C or an error cut the episode short
   * - ``manual``
     - discarded afterwards with ``rwrl dataset drop``

In ``joint_delta`` the start check is skipped: the leader reading saturates at one
``joint_scale_rad`` per step, so it cannot measure a larger gap, and the same saturation already
limits how fast the follower moves towards the leader.

.. _en-recording-demos-output:

Output
--------------------------------------------------------------------------------

``--output-dir`` is a dataset directory with one subdirectory per episode:

.. code-block:: text

   demos/peg/
     dataset.json                 the dataset header
     episode_000000/
       meta.json                  this episode's metadata
       data.npz                   per-step arrays
       frames/rgb_wrist.npz       one lossless image array per camera key
       video/rgb_wrist.mp4        review video, rgb_* keys only, at control_hz
     episode_000001/

- ``dataset.json`` records the format version, ``control_hz``, the task description, the action contract (``action_contract``, the content of the former ``.contract.json``), ``obs_spec`` (the single-frame shape of each observation key), ``named_state_spec``, the action dimension, the ``recorded_frame_stack`` and a snapshot of the recording configuration (token fields are not written). Appending to an existing dataset requires an equal action contract, ``obs_spec``, ``named_state_spec``, action dimension and ``control_hz``; otherwise the recorder fails and names the differing keys.
- ``meta.json`` records ``status`` (``kept`` or ``discarded``), ``discard_reason``, ``success``, the step count, the number of intervened steps, the intervene / hold ``segments``, the events, the start time and the duration; it also holds operator notes and the drop / undrop history.
- ``data.npz`` has observation arrays with T+1 rows (row t is the observation before step t) and step arrays with T rows: the executed ``action``, the proposed ``policy_action``, ``human_action_mask``, ``override_mask``, ``intervened``, ``reward``, ``terminated``, ``truncated``, ``syncing`` and one array per operator event seen. ``stack_reset`` marks the rows where the frame-stack history was refilled.
- Episode indices only grow: ``next_index`` in ``dataset.json`` holds the next one, and ``prune`` never renumbers.

Each episode is first written into a ``.tmp-episode_...`` directory and renamed to ``episode_NNNNNN`` only once complete and synced, so readers only ever see complete episodes; a temporary directory left by a crash is removed, with a log line, the next time the dataset is opened. The training view (``real_world_rl.dataset.iter_transitions``) takes the intervened steps of kept episodes (no syncing steps by default) and yields the rows the old pkl held: ``obs``, ``next_obs``, executed ``action``, ``reward``, ``done`` (``terminated``), optional ``grasp_penalty``, policy proposals, human/override masks, and episode/step identity and training-segment boundaries, so filtering cannot invent adjacency or transfer terminal rewards to earlier human steps. The compared experiment fields cover task, control, observation layout, preprocessing and teleop part selection/idle behavior; reward sources and station-owned reset facts are provenance. Full recovery, masks and training-view rules: \ :doc:`Real-world experiments <real-world-experiments>`\ .

.. warning::

   Images are stored losslessly, so the disk use is much larger than the old pkl. One 224×224 RGB frame is about 150 KB (one per camera per control step, before compression), about 1.5 MB per second per camera at 10 Hz; a larger ``image_size`` costs more. The tutorials and example tasks recommend 224×224; a smaller size is fine when resources are tight, as long as recording, training and deployment agree.

.. _en-recording-demos-review-and-maintenance:

Review and maintenance
--------------------------------------------------------------------------------

``rwrl dataset`` works offline, with no station or robot (``export --format pkl`` needs the ``learning`` extra, see below):

.. list-table::
   :header-rows: 1

   * - Command
     - Effect
   * - ``rwrl dataset list DIR``
     - the episode table: status, reason, success, steps, human share, duration and note count, with the kept transitions totalled at the end
   * - ``rwrl dataset view DIR [-e N ...] [--web] [--save x.rrd]``
     - replay in the Rerun viewer; needs the ``viewer`` extra (``pip install -e '.[viewer]'``). Each episode is a separate recording; pick it in the viewer's recording list. The camera images share a timeline with the joint, TCP, gripper and action curves, and intervention segments, events and status are annotated. ``-e`` is repeatable to view only some episodes; ``--web`` serves a web viewer, to be reached on a remote machine through an ssh forward of the web port (default 9090) and the gRPC port (default 9876), changeable with ``--web-port`` and ``--grpc-port``; ``--save`` only writes an ``.rrd`` file and opens no viewer
   * - ``rwrl dataset drop DIR N [N ...] [--note TEXT]``
     - flag kept episodes discarded (reason ``manual``); only ``meta.json`` changes and the history is recorded, so it is reversible
   * - ``rwrl dataset undrop DIR N [N ...]``
     - restore dropped episodes; an episode already discarded at record time cannot be restored and is an error
   * - ``rwrl dataset prune DIR --yes``
     - delete the directories of all discarded episodes, irreversibly; without ``--yes`` it lists the episodes it would delete and exits with a usage error. Indices are not renumbered

``archive inspect`` also reads a dataset directory directly; see \ :ref:`Single-attempt commissioning <en-recording-demos-single-attempt-commissioning>`\  below.

.. _en-recording-demos-export:

Export
--------------------------------------------------------------------------------

.. code-block:: bash

   rwrl dataset export demos/peg --format pkl --out demos/peg.pkl
   rwrl dataset export demos/peg --format lerobot --out demos/peg-lerobot

- ``--format pkl`` writes the training view as a pkl with a matching ``.contract.json``. rl-garden's ACT trajectory trainer and ``deploy-act`` still need a pkl plus sidecar, and the tutorials' helper scripts that need a pkl must run this step first. ``--frame-stack N`` stacks frames as the training ``obs.frame_stack`` does, and ``--keep-syncing-steps`` keeps the catch-up steps (the export does not follow the choice made at recording time). The export reads all training rows into memory. It needs the ``learning`` extra.
- ``--format lerobot`` writes a LeRobot v3.0 dataset (needs the ``lerobot`` extra: ``pip install -e '.[lerobot]'``) with the kept episodes only, one frame per control step: ``observation.state``, the named state fields, ``observation.images.<camera>`` (video), ``action``, ``intervened`` and ``human_action_mask``. Depth keys are not exported and the skipped keys are listed. RGB frames must have an even height and width, and the output directory must not exist. ``--fps`` defaults to ``control_hz`` (which must be a whole number), ``--repo-id`` to the output directory name, and ``--vcodec`` to ``h264`` (``hevc`` and ``libsvtav1`` are available; AV1 crashes on very small frames).

.. _en-recording-demos-single-attempt-commissioning:

Single-attempt commissioning
--------------------------------------------------------------------------------

Use ``--max-episodes 1`` to end after one attempt, even when no successful demo is kept.
The recorder closes device control before writing an interrupted episode.
Success, failure, rerecord, timeout and interruption all store their episode in the dataset;
only successful human demonstrations are flagged kept. Restarting the command begins another
reset, including home motion and gripper opening. Verify the station and workspace before
each restart. The default, with no attempt limit, continues until the dataset holds ``--num-successes`` kept episodes.

``rwrl archive inspect <DIR>`` (offline, no station or robot; it also accepts an old SQLite archive) reads the dataset read-only and,
for a single-arm state with ``tcp_pose``, reports per human_action_mask segment the hold drift (also
over the first ``--drift-window-s``), the residual and the take-over jump in mm, plus the settle time
after each ``--stop-time-s``; ``--measurements PATH`` writes them as the test pages' ``measurements.csv``
rows and ``--csv PATH`` exports the steps. With ``--stop-time-s`` it exits 0 when every stop settled,
3 when one did not and 4 when a stop time has no spanning TCP episode
(\ :ref:`cli.md <en-cli-commands>`\ ). ``python examples/inspect_teleop_archive.py`` is an alias.

.. note::

   Earlier test records (the session pages under the \ :doc:`hardware session checklist index <testing/index>`\ ) were recorded with the old recorder, so their commands show ``--output-path`` and ``.pkl``; the current form is the ``--output-dir`` above.

See the \ :doc:`hardware session checklist index <testing/index>`\  for the English primary
test pages, record CSV conventions and evidence requirements. Fill hardware results on site.

.. |reference-link-1| replace:: ``examples/presets/hil_single_arm.yaml``
.. _reference-link-1: https://github.com/JaimeParker/real-world-rl/blob/main/examples/presets/hil_single_arm.yaml

.. |reference-link-2| replace:: ``examples/presets/gello_cartesian.yaml``
.. _reference-link-2: https://github.com/JaimeParker/real-world-rl/blob/main/examples/presets/gello_cartesian.yaml
