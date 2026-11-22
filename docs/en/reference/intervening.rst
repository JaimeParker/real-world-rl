.. _en-intervening-operator-page-intervening:

Operator page: intervening
================================================================================

Use this page beside the robot for \ :doc:`recording <../tutorials/record>`\  and
\ :doc:`HIL training <../tutorials/rl-train>`\ . The hardware process runs directly on the NUC,
under ``rwrl supervise``, with the 50 Hz state-only cartesian preset.
GELLO uses ``--teleop-device leader_cartesian --teleop-event-device stdin``.
SpaceMouse uses ``--teleop-device spacemouse --teleop-event-device stdin``.

The SpaceMouse instructions below describe default ``--spacemouse-mode tracker``.
For ``--spacemouse-mode serl``, configure xyz and RPY safety boxes in the station
preset first: raw six-axis norm ``> 0.001`` takes over, and the next idle step
returns to the policy (or the recorder's hold). Hold button 0 to close or button 1
to open, with button 0 priority and neutral gripper ``0`` when neither is held.
There is no takeover toggle. Terminal ``t``/``h`` remain ignored; labels/end/quit work.
Select this mode on the NUC, repository root, project environment active, using
the same mode and preset as recording and training. See \ :ref:`mode details <en-teleop-spacemouse>`\ .

``stdin`` reads the terminal running the supervised command, including over SSH;
``keyboard`` uses the same keys through ``pynput`` and needs a desktop session.

.. list-table::
   :header-rows: 1

   * - Input
     - Effect
   * - ``t``
     - GELLO: take over and anchor the relative mapping; repeated presses do not toggle it off
   * - ``h``
     - GELLO: hand back; recorder holds the measured pose at that moment, HIL returns control to the policy
   * - ``s``
     - success and episode end; recorder defaults to reward 1 and a terminal step; HIL requires ``--reward-from-teleop-events`` for key rewards
   * - ``f``
     - failure; discard the recorder pkl attempt, or truncate HIL with ``--reward-from-teleop-events`` (task presets may define their own outcome rules)
   * - ``r``
     - discard and rerecord in the recorder; no rerecord action in HIL
   * - Esc
     - end the episode without a success label (``human_episode_end``)
   * - ``q``
     - end the current episode and stop the recorder/actor after processing it
   * - GELLO trigger
     - close (``-1``) when squeezed, open (``+1``) when released, after the takeover latch is matched
   * - SpaceMouse button 0
     - take over; next press hands back; only this button controls takeover on this route
   * - SpaceMouse button 1
     - while driving, toggle gripper open/closed; ignored while handed back
   * - SpaceMouse puck
     - velocity input while driving; movement alone does not count as takeover

With SpaceMouse, terminal ``t``/``h`` are ignored; ``s``/``f``/``r``/Esc/``q`` still work.
Keys pressed during reset/home are dropped. Keep the physical stop accessible.

After a reflex stop, inspect the cell and release the activation device or
clear the fault in Desk as needed. On the **NUC**, **repository root**, with
the **project environment active**, use ``rwrl recover`` to request error recovery;
``rwrl stop`` requests an arm stop without confirmation. Both follow the station's
connection selection. On this direct workflow, finish the recorder/actor and
release its connection first; a second FCI connection cannot stop a directly
owned arm. With a declared live daemon, the commands use that daemon. Keep the
physical stop accessible during a run. See \ :ref:`rwrl <en-cli-connection-ownership>`\ .

.. _en-intervening-the-sequence:

The sequence
--------------------------------------------------------------------------------

.. _en-intervening-gello:

GELLO
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

1. Wait for reset and any home/gripper motion to finish.
2. Hold the leader comfortably and press ``t``. The current leader pose anchors
   the relative mapping to the follower's command; no joint-by-joint matching is
   required on this cartesian path.
3. Move the leader to guide the tool. Squeeze/release the trigger for the gripper.
   If its position disagrees with the last gripper command at takeover, match
   that state first; the latch prevents an immediate unintended opening.
4. Press ``h`` to hand back. In the recorder the measured pose is held; in HIL
   the policy resumes. Press ``t`` again for a fresh anchor on the next takeover.
5. Use ``s``\ **/**\ ``f``\ **/**\ ``r`` for the outcome. In the recorder press ``s`` while intervening
   so the successful terminal step enters the human-only pkl export (the archive
   retains policy and human steps). After ``h``,
   use ``t`` to re-anchor before recording that label.
6. Use **Esc** for an unlabeled episode end or ``q`` to end the run.

.. _en-intervening-spacemouse:

SpaceMouse
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

1. Wait for reset. Release the puck and buttons before starting a new takeover.
2. Press **button 0** to take over. Nudging the puck alone never takes control.
3. Deflect the puck for tool velocity; release it to stop adding motion to the
   command. The arm can still settle by at most the lead cap. Use **button 1** to
   toggle the gripper; takeover itself keeps its last commanded state.
4. Press **button 0** again to hand back; a later press begins another takeover.
   Keyboard ``t`` and ``h`` do not change this device's takeover state.
5. Use keyboard ``s``\ **/**\ ``f``\ **/**\ ``r``\ **/Esc/**\ ``q`` for outcomes and quit. In the recorder label
   success while button-0 takeover remains active.

With the opt-in ``episode_end_behavior: hold`` workflow, a trajectory label freezes
the last accepted target and ignores puck motion. Press n to request a new
trajectory, then release all axes/buttons until neutral (default max absolute
axis 0.05 for 0.2 seconds). This gate applies between trajectories, not at each
takeover. At the episode quota, n is disabled and q still exits. Within a
trajectory, repeated human/policy handovers continue normally. SERL keeps its
motion-based takeover threshold; the neutral threshold does not replace it.

During \ :doc:`evaluation <../tutorials/rl-train>`\ , leave action control with the
policy to measure autonomous success. An assisted success counts in
``task_success_rate``, while the primary ``success_rate`` is
``autonomous_success_rate``. Label-only ``s``/``f`` events do not count as action
intervention; a failure label vetoes a simultaneous success label.

.. _en-intervening-how-the-robot-follows-you:

How the robot follows you
--------------------------------------------------------------------------------

The cartesian source keeps a fixed hold pose after reset and every hand-back.
The recorder's hold policy shares that tracker, so holding sends commands back
to that fixed pose rather than a zero twist that would drift with measured pose.
HIL hands control back to the policy; it is not a command to freeze policy motion.

At GELLO takeover, the source anchors the leader's current pose to its existing
command pose. Leader displacement becomes a requested translation and rotation.
The command advances at up to **0.05 m/s** and **0.3 rad/s** and leads measured TCP
by at most **0.05 m** and **0.3 rad**. Excess lead is discarded, never queued.
When the leader stops, the follower can still travel at most one lead cap toward
the retained command. Verify physical direction and scale during commissioning.

SpaceMouse uses the same tracker. Full puck deflection requests the configured
velocity; letting go stops further velocity requests. It does not remove the
bounded distance between the arm and its existing command.

A gap of more than **0.3 s** between intervened polls holds the follower and logs
the delay; the next poll re-anchors the GELLO, avoiding a jump across the pause.
This tracker behavior is separate from an env StepWatchdog trip, which stops the
arm and requires reset, and from the external supervisor's heartbeat escalation.

A requested position outside the safety box logs ``WORKSPACE HOLD: <axes>``,
hands back and holds the measured pose. Clear the leader request and use ``h``
then ``t`` to resume GELLO with a new anchor; on SpaceMouse press button 0 to take
over again. Do not widen the workspace simply to clear a HOLD.

The four ``--teleop-lead-cap-m``, ``--teleop-lead-cap-rad``,
``--teleop-target-speed-m-s``, ``--teleop-target-speed-rad-s`` values enter
``experiment.teleop_control``. Keep them and the selected teleop device consistent
through this tutorial's recording, training and deployment. Current HEAD
compares the tracker values plus SpaceMouse-specific ``axis_lock`` and applicable
input-frame/diagnostic settings, rather than physical device identity; see
\ :doc:`08's compatibility note <../tutorials/record>`\ .

.. dropdown:: Other modes: Onset arm or ramp (joint-space leader_arm only)

   This section concerns the joint-space leader path, not ``leader_cartesian`` or
   SpaceMouse. ``--teleop-onset`` decides what happens when ``t`` requests takeover:

   -  ``arm`` waits until every leader joint is within ``--teleop-arm-tolerance-rad``
      (0.1 rad) of the follower; reset disarms it. Keep the leader aligned first.
   -  Default ``ramp`` begins immediately. ``joint_delta`` moves toward the leader by at
      most ``joint_scale_rad`` per step; ``joint_abs`` targets are limited by
      ``--teleop-ramp-step-rad`` (0.05) on every intervened step.

   Only these joint modes emit ``teleop_syncing``: initial catch-up in ``joint_delta``
   and any ``joint_abs`` step whose leader/follower gap exceeds the ramp limit,
   including after initial alignment. Executed actions and masks stay
   in the archive; syncing supervision is excluded from archive-derived training
   views. ``--keep-syncing-steps`` can retain these rows in recorder pkl exports.
   Joint-space trigger takeover still needs the operator to match gripper state;
   the cartesian gripper latch does not apply to that route.

.. _en-intervening-what-the-system-records:

What the system records
--------------------------------------------------------------------------------

The intervention wrapper supplies the intervention keys below. Task/reward
wrappers supply ``task_success`` and ``task_success_source`` when an explicit
outcome is available; event-only keys are conditional as noted.

.. list-table::
   :header-rows: 1

   * - ``info`` key
     - Value
   * - ``intervene_action``
     - the executed ``(1, A)`` action; **only while intervening**
   * - ``policy_action``
     - the action the policy proposed, as received
   * - ``intervened``
     - the operator's action was executed
   * - ``intervention_onset``
     - intervened now and not on the previous step of this episode
   * - ``human_episode_end``
     - the operator's episode-end key (Esc)
   * - ``teleop_events``
     - the ``success``/``failure``/``rerecord``/``operator_quit`` events that fired; ``teleop_syncing`` applies only to joint-space leaders; **only when some did**
   * - ``human_action_mask``
     - action dimensions actually controlled by the human
   * - ``override_mask``
     - overridden dimensions, including idle-hold supplements
   * - ``task_success``, ``task_success_source``
     - explicit outcome and its source when available; a positive reward or terminal flag alone is insufficient

Esc sets ``truncated=True`` and leaves any existing ``terminated`` flag unchanged; ``q`` also requests
shutdown after the episode. A success key can instead make its step terminal. The stored ``action`` of an
intervened step is always the executed one, the human's. What each method does with it:

.. list-table::
   :header-rows: 1

   * - Method
     - Executed action and supervision
     - Replay or export
   * - recorder
     - stores actual executed actions; human masks select supervised dimensions
     - complete SQLite archive; successful human pkl view, excluding joint-space syncing by default
   * - ``hil_serl``
     - learns from all executed actions; human, nonsyncing rows also populate demo replay
     - online replay plus growing demo replay, mixed at ``--demo-data-ratio``
   * - ``residual_hil_serl``
     - human action is the total executed action; retains ``policy_action``, ``base_actions``, ``next_base_actions``
     - residual BC intersects the actual human mask with nonzero residual scales
   * - ``silri``
     - retains proposal, actual masks and the predecessor's ``intervention_onset``
     - behavior fitting and pretraining supervise human dimensions; onset rows are not relabeled as expert

The gripper heads (``hil_serl``'s DQN, ``--residual-variant hybrid``, ``silri``'s discrete head) label each
stored gripper action with the nearest of ``-1``/``0``/``+1``; with ``gripper_binarize_threshold: 0.5`` (the
wizard's default) the trigger gives exactly ``+1`` or ``-1``. Details:
\ :ref:`residual.md <en-residual-human-intervention>`\ , \ :ref:`silri.md <en-silri-data-flow>`\ ,
\ :ref:`teleop.md <en-teleop-interventionwrapper>`\ .

.. _en-intervening-where-the-reward-comes-from:

Where the reward comes from
--------------------------------------------------------------------------------

With the tutorial's default ``task.name: none``, ``real_robot`` returns reward 0 and has no task
success detector. Configure key rewards or a classifier. Versioned task presets can supply
explicit outcomes and reset/termination rules; a TCP-position proxy remains a separate metric.

-  **Recorder**: the reward comes from your keys by default (``--no-reward-from-teleop-events`` turns it
   off). ``s`` stores the step as a terminal success with reward 1.
-  **HIL training** (``hil_serl``, ``residual_hil_serl``, ``silri``): without ``--classifier-checkpoint``,
   pass ``--reward-from-teleop-events`` to the actor (the commands of \ :doc:`10 <../tutorials/rl-train>`\  do). The
   actor and learner log a warning when neither is set.

The rule for ``s``: press it **while you still hold control** in the recorder, because only intervened
steps enter the human pkl export (all steps remain in the archive), and an ``s`` after the release leaves the demo with reward 0 everywhere. In HIL
training ``s`` counts on any step, yours or the policy's: press it on the step the object lands in the
bin. With a classifier the larger of the two rewards wins (the keys never lower the classifier's).

.. _en-intervening-when-to-intervene:

When to intervene
--------------------------------------------------------------------------------

Take over when the policy is about to fail the task, collide, or leave the safe region; release when
it is back on track. Do not intervene while it is doing fine: the actor stores every step, and only
your intervened steps enter the demo buffer. Expect to intervene a lot at first and less as the
policy improves.

Two kinds of takeover, and what they teach:

-  **Recovery**: the policy has gone wrong (missed the object, drifting out of the workspace). Take
   over, bring the robot back to a state the task can continue from, and release before you finish
   the task yourself. These rows show the way out of states the policy reached on its own, which the
   demos never visited.
-  **Correction**: the policy is roughly right but misses a detail (the grasp is off-centre, the
   object is released too early). Take over only for that moment, do it right, release. Short
   corrections at the decisive step give the most information per intervened row.

Keep each takeover purposeful and consistent: in ``silri`` your actions train a model of your behaviour
whose spread loosens the constraint where you are inconsistent; in ``residual_hil_serl`` your action is
the total action, and the BC term clips the residual it implies to the box ``base +/- scale``, so a
correction far from the base teaches the residual only "as far as the box allows" (the critic still
learns the value of the action you executed). Esc ends an episode that cannot be recovered.

.. _en-intervening-take-over-selected-parts:

Take over selected parts
--------------------------------------------------------------------------------

``--teleop-selected-parts arm.arm`` selects the tutorial's arm motion; ``arm.gripper`` selects its
gripper. On two arms use ``left.arm``, ``right.arm``, ``left.gripper`` or ``right.gripper``, matching
``arm_names``. An empty list uses all parts available from the source. Default
``--teleop-idle-behavior hold`` holds unselected parts during a takeover; ``policy`` leaves them
with the policy. Only ``human_action_mask`` dimensions supply expert supervision; ``override_mask``
also covers held parts. Set the same part selection and idle behavior when recording and
training, because they enter the experiment contract. See \ :ref:`partial takeover <en-real-world-experiments-part-takeover-and-imitation>`\ .


