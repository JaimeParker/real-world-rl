Record and label demonstrations
================================================================================

After :doc:`spacemouse` or :doc:`gello`, both devices use the same recording workflow.
Continue with ``local/station.yaml`` and ``local/task.yaml``. This chapter uses a single-arm Cartesian session,
not the joint-space or bimanual extensions with automatic reset at the end of the GELLO chapter.
An episode is one attempt, from after reset until success, failure, rerecord, timeout or quit.

``teleop run`` defaults to ``episode_end_behavior: confirm``: the start of every episode, the gripper handling after it and the next reset
are all confirmed by the operator at the terminal. Nothing starts or resets automatically. The smoothest setup uses two people:
**operator A** presses keys in the terminal that runs ``rwrl``, and **operator B** holds the SpaceMouse or GELLO and gives the demonstration.
One person can do it too, switching between teleoperating and pressing keys.

.. tip::

   The commands in this chapter omit ``--station`` and ``--preset``. They use the two files remembered by ``rwrl use`` in step 10 of :doc:`station`.
   If you have not done that step, run ``rwrl use preset local/task.yaml`` first. To use another file temporarily, pass the parameter explicitly in the command.

1. Start recording with quotas
--------------------------------------------------------------------------------

Give the session a new output name (a dataset directory, ``local/first-demo``, with no suffix). Limit both successes and total attempts to avoid continuing to another round unintentionally.
Running with the same directory again appends to it, so use a new name for each new experiment.

.. warning::

   The first reset after startup moves home and opens the gripper. Check home, the path, object placement and how to stop first.
   Pressing n in POST_EPISODE (once the gripper check passes) also moves home and opens the gripper. Do not enter the robot's motion range while control remains active.

.. literalinclude:: ../../examples/record.sh
   :language: bash
   :start-after: # start record
   :end-before: # end record

``teleop run`` selects the device from the station and combines its connection information with the preset into a session configuration.
``--num-successes 2`` keeps two successful demonstrations. ``--max-episodes 5`` allows at most five completed attempts;
failure, rerecord and timeout each count as an attempt. Set both to 1 for a single practice round.
If omitted, the success quota is 20 and total attempts are unlimited. The task's max_episode_steps still limits each episode to 200 steps in this tutorial.
``--record`` points at a dataset directory: each episode is written as an ``episode_000000/`` subdirectory when it ends,
and memory only ever holds the episode being recorded. The success quota counts the episodes already kept in the dataset, including those from earlier runs.

**What you should see**: A confirmation prompt, then RESETTING (the arm moves home and the gripper opens) and READY (the arm holds at home with motion ignored; B arranges the objects and A presses space to start).
Episode statistics appear after a successful episode. The following example follows the source output format. Its numbers are not measurements:

.. literalinclude:: ../../examples/en/record-output.txt
   :language: text
   :start-after: # start episode
   :end-before: # end episode

An episode moves through phases. Each time one starts, the terminal prints a ``<PHASE>: ...`` line, and A presses the keys that line names:

.. literalinclude:: ../../examples/en/record-output.txt
   :language: text
   :start-after: # start diagram
   :end-before: # end diagram

.. list-table:: Keys operator A can press in each phase
   :header-rows: 1
   :widths: 18 32 50

   * - Phase
     - Arm
     - Operator A's keys
   * - RESETTING
     - Moves home, then the gripper opens
     - None. Keys pressed now are discarded
   * - READY
     - Holds at home, teleop motion ignored
     - space: start the episode; q: quit
   * - ACTIVE
     - B teleoperates; recording runs
     - s success, f failure, r rerecord, Esc end without a verdict, q quit after this episode (step 2)
   * - POST_EPISODE
     - Holds where it is, gripper unchanged, teleop motion ignored
     - o open the gripper, c close it, n ask for the reset, q quit
   * - GRIPPER_CHECK
     - Same
     - Enter resets now; o opens in place first and returns to POST_EPISODE; any other key returns to POST_EPISODE; q quits
   * - STOPPING
     - Control released
     - None

**If it doesn't**:

* ``--record must be a demo dataset directory (new or existing), not a file``: Give ``--record`` a directory name without ``.pkl``, and do not point it at an existing file.
* ``teleop run requires --preset with home and task limits``: No preset has been remembered. Run ``rwrl use preset local/task.yaml``, or pass ``--preset local/task.yaml`` in the command.
* Stuck at READY: A has to press space to start. Press keys in the terminal that runs ``rwrl``, not in another shell. Keys pressed during reset are discarded, so wait for READY.
* The quota never increases: Check the discarded reason. A successful episode with no human-controlled steps is not kept as a demonstration either.

2. Complete and label episodes
--------------------------------------------------------------------------------

Complete the task with the selected device, then label the outcome with the event keys so that successful demonstrations can be distinguished from failed attempts.
The default event source in this chapter is ``stdin`` in the terminal running the command. It works in a local terminal or an interactive SSH terminal. The first five keys below act only in ACTIVE.

.. list-table:: Keys in the current code
   :header-rows: 1
   :widths: 15 25 60

   * - Key
     - Event / state
     - Use in this chapter
   * - s
     - success
     - Press after confirming task success. Event rewards are enabled by default; success gives a reward of 1 and ends the episode.
   * - f
     - failure
     - Mark failure and end the episode. An attempt containing a failure is not kept as a successful demonstration.
   * - r
     - rerecord
     - End the attempt. The episode is still stored in the dataset, flagged discarded (rerecord), and is not used for training.
   * - Esc
     - episode_end
     - End the episode without a verdict; it is flagged discarded (no success).
   * - q
     - operator_quit + episode_end
     - Judge this episode as usual, then end the session; in READY, POST_EPISODE and GRIPPER_CHECK it ends the session at once. Finished episodes were already written to the dataset.
   * - space
     - start_episode
     - Acts only in READY: start the episode (step 3).
   * - n, o, c, Enter
     - next_episode, gripper_open, gripper_close, confirm
     - Act only in POST_EPISODE and GRIPPER_CHECK (step 3). They do nothing in ACTIVE.
   * - t / h
     - Takeover / handback state
     - Used during active GELLO episodes in this chapter. SpaceMouse tracker uses button_0; SERL uses motion / button levels.

**What you should see**: After s, the statistics say kept and successes increases by one. f, r or an end without success shows discarded.
Reaching the step cap does not count as success either; it is ``discarded (no success)``.
A wrong verdict needs no re-recording: afterwards ``rwrl dataset drop`` discards a kept episode and ``rwrl dataset undrop`` restores it, see :ref:`Review and maintenance <en-recording-demos-review-and-maintenance>`.
Human candidate steps exist only during intervention. Gripper keys control only the gripper; they do not label success.
This chapter has no reward classifier. The default ``reward_from_teleop_events: true`` makes labels produce rewards.

**If it doesn't**:

* Keys have no effect: Return focus to the recording terminal and check that it is still connected. Do not type s in another shell.
* ``discarded (no success)``: Neither Esc nor reaching the step limit automatically counts as success. Explicitly press s when the task is complete.
* ``discarded (no intervened steps)``: Pressing only s without operating the robot cannot produce a demonstration. Complete a real intervention first.

3. Between episodes: hold, gripper check and reset
--------------------------------------------------------------------------------

After an episode ends the arm stays where it is, and operator A decides when to reset. B should not touch the leader or the cap yet; A reads the phase line in the terminal and presses a key once the scene is ready.

.. literalinclude:: ../../examples/en/record-output.txt
   :language: text
   :start-after: # start confirm
   :end-before: # end confirm

**What you should see**: After an episode ends with success, failure, rerecord, Esc or the step cap, the session enters POST_EPISODE: the arm holds where it is,
the gripper stays as it is, teleop motion is ignored, the environment's step is not called, and waiting time is not added to the trajectory. A can now press:

* ``o`` to open the gripper and ``c`` to close it; the terminal prints ``GRIPPER: open sent to arm.`` and the phase stays, so you can repeat. For example, open the gripper to put an object down before resetting.
* ``n`` to ask for the reset.
* ``q`` to quit without a reset.

On n the session first looks at the gripper width. Fully open (at least 95% of the maximum width) or fully closed (at most 5%) resets directly.
Anything in between may still hold an object, so GRIPPER_CHECK asks once more: Enter resets now (the gripper opens only after the arm reaches home, so the object drops at home);
``o`` opens it in place first and returns to POST_EPISODE, where you press n again; any other key returns to POST_EPISODE; q quits.
A gripper that reports no width is always asked about. The two thresholds are ``gripper_open_fraction`` and ``gripper_closed_fraction``; see :doc:`../reference/recording-demos`.

During reset the arm moves home and then the gripper opens; keys pressed in that time (RESETTING) are discarded. After the reset the session enters READY: the arm holds at home, teleop motion is still ignored,
A and B arrange the objects, and A presses space to start the next episode. Only steps in ACTIVE are recorded: motion and waiting in POST_EPISODE, GRIPPER_CHECK, RESETTING and READY never reach the dataset,
and row 0 of an episode is the observation freshly read at the moment space is pressed.

When the budget is used up (the success quota or ``--max-episodes`` is met), POST_EPISODE says ``n or q = end the session (no reset)``:
o and c still work, and n or q ends the session without another reset. q works in every phase: in READY, POST_EPISODE and GRIPPER_CHECK it ends the session at once,
and in ACTIVE it first ends the current episode and judges it as usual, then ends the session. After q the devices close; each episode was already written to the dataset when it ended, and evidence of a normal exit is saved in the session directory.

GELLO joint mode (joint_abs, see step 5 of :doc:`gello`) adds one lock in READY: while any leader joint is further from the follower than the start tolerance
(0.8 rad by default, ``start_tolerance_rad``), space does nothing and the terminal lists the joints that are too far apart, until the leader is aligned. The session does not exit with an error:

.. literalinclude:: ../../examples/en/record-output.txt
   :language: text
   :start-after: # start lock
   :end-before: # end lock

Cartesian sessions have no such lock. In confirm mode ``--neutral-stable-s`` and the neutral threshold no longer apply; only ``episode_end_behavior: hold`` uses them (see step 6).

.. warning::

   Pressing n (and Enter) permits reset and gripper opening. Confirm that objects are stable and the path is clear.
   Before entering the workspace, stop control using the site's procedure. Do not treat POST_EPISODE as a powered-off state.

**If it doesn't**:

* Nothing happens after n: Check that the terminal has focus and the phase is POST_EPISODE (n does nothing in READY and ACTIVE); when the budget is used up, n ends the session instead of resetting.
* Nothing happens after space: The session may still be in RESETTING (keys are discarded; wait for READY to appear); in GELLO joint mode, check whether the terminal says ``start locked`` and align the leader with the follower.
* The event device is a GELLO evdev on the robot PC, no key does anything and the session stays in READY: an older daemon does not forward the new keys such as start_episode; upgrade the daemon on the robot PC.
* The robot moves home immediately after the episode: Check whether you bypassed teleop run and passed an explicit ``--episode-end-behavior auto`` (the default of ``rwrl record`` is already confirm).
* No successful demonstration visible: Exit normally with q first, then use ``rwrl dataset list`` from the next chapter to see each episode's status and reason. Episodes that were not kept are still in the dataset, flagged discarded.

4. Use scripted events or other event sources when needed
--------------------------------------------------------------------------------

The event source carries only intervention state and episode labels. Motion still comes from SpaceMouse or GELLO.
teleop run currently defaults to stdin; providing events-file selects jsonl. The confirm keys have names in jsonl too, listed below.
First ensure that the event file below does not exist, then create an empty file and start a separate new session:

.. tutorial-test: skip
.. literalinclude:: ../../examples/record.sh
   :language: bash
   :lines: 6-7

.. literalinclude:: ../../examples/record.sh
   :language: bash
   :lines: 8-8

After completing the actual operation, write an event from a second terminal:

.. tutorial-test: skip
.. literalinclude:: ../../examples/record.sh
   :language: bash
   :start-after: # start success
   :end-before: # end success

Write the reset request only after POST_EPISODE appears and the scene is ready; once the reset is done and READY appears, write the start event:

.. tutorial-test: skip
.. literalinclude:: ../../examples/record.sh
   :language: bash
   :start-after: # start next
   :end-before: # end next

.. tutorial-test: skip
.. literalinclude:: ../../examples/record.sh
   :language: bash
   :start-after: # start start-event
   :end-before: # end start-event

Finally, exit normally:

.. tutorial-test: skip
.. literalinclude:: ../../examples/record.sh
   :language: bash
   :start-after: # start quit
   :end-before: # end quit

**What you should see**: The same state transitions as with the keyboard. Valid event names are takeover, handback, success,
failure, rerecord, end, next and quit, and for the confirm flow also start (space), enter (Enter), open (o) and close (c).
Each poll consumes only one line. Do not prefill events for the entire session; write each event only after the matching phase line appears.
A regular file waits for appended data at EOF. ``--events-file -`` reads a pipe and raises an error if the stream closes before quit.

For a desktop keyboard, select ``teleop_event_device: keyboard`` in a standalone recorder configuration.
It requires pynput and a desktop session; it is not teleop run's default source. It also reads space and Enter (including the keypad Enter) as start and confirm.
GELLO's evdev event device defaults to Space for toggling (which doubles as start), S/F/R for labels and Esc for ending an episode, and binds n, o, c, Enter and q where those keys are free;
see :doc:`../reference/gello`. With a pedal that has only a few keys, rebind with ``event_keys`` and let the terminal cover the rest.

**If it doesn't**:

* ``invalid JSONL operator event``: Each line must contain only one event field. Use a name listed above and end the line with a newline.
* ``JSONL event stream closed before quit``: Have the producer write quit before closing the pipe.
* Old events replay: Use a new empty event file. touch does not empty an existing file.

5. Find the recording artifacts
--------------------------------------------------------------------------------

After a normal exit, use the output name to find the dataset directory and the session evidence.
The paths below use first-demo as an example; the program generates uuid.

.. literalinclude:: ../../examples/en/record-output.txt
   :language: text
   :start-after: # start files
   :end-before: # end files

**What you should see**: The dataset has one directory per episode, and every episode is there, including those without intervention, failures, rerecords and interruptions.
``meta.json`` records the status (kept or discarded), the reason, the step count, the intervention segments and the events; ``data.npz`` holds the per-step data;
``frames/<key>.npz`` holds the lossless images; ``video/<key>.mp4`` is a review video at the control rate, for color cameras only.
Training uses only the human-controlled steps of kept episodes, excluding ``teleop_syncing`` alignment steps by default.
The action / observation convention lives in ``dataset.json``; there is no pkl, ``.contract.json``, SQLite or ``.trajectory.pt`` any more.
An episode appears only once it is complete: the one being written is a ``.tmp-episode_...`` directory, which is removed automatically the next time the dataset is opened after a crash and is never read as an episode.

.. note::

   Images are stored losslessly, so the disk use is larger than the old pkl. One 224×224 RGB frame is about 150 KB, one frame per camera per control step, before compression;
   at 10 Hz and 200 steps, one episode with one camera is about 30 MB. Check that the target disk has room before recording.

In the session directory, resolved-config is the final configuration, provenance records its sources, and summary records the session result.
diagnostics records control diagnostics; session-events records phases such as RESETTING, READY, ACTIVE, POST_EPISODE, GRIPPER_CHECK and STOPPING.
The diagnostics, session-events and stop-trace filenames all contain a timestamp and random suffix.
A normal stop generates stop-trace and an assessment with the same basename; these may be absent after a fault or if that phase was not reached.
GELLO also saves the extracted leader-robot and leader-options.

**If it doesn't**:

* No kept episode in the dataset: Read the discarded reason on the terminal's ``[record] episode=...`` lines, then check with ``rwrl dataset list`` in the next chapter.
* The summary's stop_assessment is BLOCKED: This session lacks evidence of a normal stop. Do not treat the session return code as verification that stopping worked.
* You cannot find the output: Check output and report_directory in summary, and the working directory used for the run.

6. Use the ordinary recorder directly when needed
--------------------------------------------------------------------------------

Use this extension only when you need to manage source and environment options independently. Copy a verified session's resolved-config.yaml
to ``local/record-cartesian.yaml``, keeping the robot connection, home, limits, observations and source option files.
Confirm that paths such as leader-options remain accessible. Before changing input sources, remove the ``teleop_input`` cache from the copied configuration;
otherwise the old source option snapshot may remain effective. Use a GELLO session configuration as the starting point for the two commands below. Ordinary ``rwrl record`` reads this configuration;
it does not automatically merge the current station as teleop run does. Check task_frame and workspace in real_robot.

.. warning::

   The following commands explicitly select auto: an episode may automatically reset, move home and open the gripper after ending, without waiting for n.
   Check the reset path before startup and keep an operator on site. For a direct connection, first stop any daemon using the same follower arm.

.. literalinclude:: ../../examples/record.sh
   :language: bash
   :start-after: # start ordinary
   :end-before: # end ordinary

supervise monitors the recorder's heartbeat in a separate process. The normal heartbeat timeout defaults to 2.0 seconds, with escalation through
SIGINT → SIGTERM → SIGKILL after timeout. It cannot independently send a stop command to the arm; after forced termination, stopping depends on disconnection and firmware.
The command also explicitly sets ``--real_robot.watchdog_s 0.3`` so that the in-process StepWatchdog detects missing steps.
If the entire hardware process hangs, this in-process check cannot be relied on to run independently.

**What you should see**: Hold after reset, t/h for GELLO takeover and handback, and episode results and save lines after labeling.
The ordinary recorder does not generate teleop run's full session reports.

``--episode-end-behavior`` takes three values. ``confirm`` is the default of ``rwrl record`` and what teleop run uses: the two-person flow of the previous three steps.
It needs an event device (``--teleop-event-device keyboard|stdin|jsonl``, or the evdev ``event_device`` of a GELLO) and works with Cartesian and joint arms.
``hold`` is the single-operator version: after an episode the arm holds until a neutral-confirmed n resets it, and it supports Cartesian arms only.
``auto`` resets as soon as an episode ends; training commands default to it, see :doc:`rl-task`.

**If it doesn't**: If startup fails, read supervise.jsonl and the terminal error first, then check configuration files and device ownership.
Do not assume that the robot has stopped normally because the supervisor is still running.

When switching a GELLO configuration to SpaceMouse, clear the leader source options so they are not passed to SpaceMouse:

.. literalinclude:: ../../examples/record.sh
   :language: bash
   :start-after: # start ordinary-spacemouse
   :end-before: # end ordinary-spacemouse

**What you should see**: In tracker mode, button 0 takes over / hands back, and the terminal handles labels. t/h do not control this input source.

**If it doesn't**: If the source reports unknown options, confirm that you passed ``--teleop-options-yaml None``.
Also verify the direction settings as in :doc:`spacemouse`. Do not begin collecting data after changing only the device name.

7. Resume ordinary recording after an interruption
--------------------------------------------------------------------------------

Read the supervisor log first. Check the station, robot faults and stop state before deciding whether to restart.
Rerun the step 6 command with the same output directory to append: existing episodes are kept, new episodes continue the numbering, and indices are never reused.
Appending requires this run's environment to match the dataset header (action contract, observation shapes, control rate); otherwise the recorder fails and asks for a new ``--output-dir``.
An episode cut short by Ctrl-C or an error is stored as discarded (interrupted) once it has a step.
Each episode is written to a temporary directory and renamed only when complete, so a crash loses only the episode being recorded.
See :doc:`../reference/real-world-experiments` for process-crash and power-loss boundaries.

.. warning::

   Restarting also resets and operates the gripper. Recheck the scene and reset path first. Reusing a filename does not remove the need to check motion.

**What you should see**: A start line such as ``[record] dataset local/ordinary-demo: 3 episodes (2 kept); recording until 5 kept``.
``--num-successes`` counts the episodes already kept in the dataset (old ones included), so only the difference is recorded. When the quota is already met, no episode is recorded.

**If it doesn't**: If the error says the dataset was recorded with a different ``action_contract``, ``obs_spec`` or similar key, use a new ``--output-dir``
or rerun with the settings the dataset was recorded with. Do not edit ``dataset.json`` to get around it, and do not use a teleop run session report directory as an entry point for resuming the ordinary recorder.

.. note::

   **What needs real-hardware confirmation in this chapter**

   Task success / failure criteria, label timing, the first and next-round reset paths, the effect of opening the gripper,
   actual motion during hold, terminal disconnection and normal stopping must be confirmed on site.
   The confirm flow adds a few more: in joint modes the hold uses the measured joints (the controller stops at where the arm is; there is no earlier target to reuse);
   the Franka Hand opens and closes asynchronously, so pressing n right after o may still read as partly closed and ask again;
   an older daemon does not forward the new keys, so no key works and the session stays in READY.
   JSONL format validation and offline archive inspection can confirm only the software paths.

Checklist: Complete at least one successful episode and one failed attempt; observe POST_EPISODE and READY;
use n to reset, space to start the next round and q to exit normally; locate each episode's meta.json in the dataset directory.

Next, use :doc:`inspect` to identify the segment you controlled.
