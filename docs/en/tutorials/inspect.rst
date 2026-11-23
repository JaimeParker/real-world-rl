Inspect recording results
================================================================================

This chapter continues with the ``local/first-demo`` output from :doc:`record`. All operations are offline and do not connect to the robot.
List and replay the episodes in the dataset first, then measure the intervention segments with the inspector, and finally check the training data.
The goal is to explain which steps of which episode you controlled, and which dimensions you controlled.
The commands in this chapter do not modify the dataset; ``drop``, ``undrop`` and ``prune``, which do, are described in :doc:`../reference/recording-demos`.

1. List and replay the episodes
--------------------------------------------------------------------------------

List every episode in the dataset first, including those that were not kept:

.. literalinclude:: ../../examples/inspect.sh
   :language: bash
   :start-after: # start list
   :end-before: # end list

**What you should see**: One line per episode with its status (kept or discarded), the reason, whether it succeeded, the step count, the share of steps the operator drove, the duration and the number of notes.
The last line totals the episodes, the kept and discarded counts, and the kept transitions (the rows training will use). For example:

.. literalinclude:: ../../examples/inspect-list-output.txt
   :language: text
   :start-after: # start output
   :end-before: # end output

Then replay them in the Rerun viewer. This needs the viewer extra: ``pip install -e '.[viewer]'``.

.. literalinclude:: ../../examples/inspect.sh
   :language: bash
   :start-after: # start view
   :end-before: # end view

**What you should see**: Each episode is a separate recording in the viewer; pick one in the recording list. Names look like
``episode_000007 · kept`` or ``episode_000003 · discarded (rerecord)``.
Each color or depth camera has its own view, sharing a timeline with the joint, TCP position and orientation, gripper, and ``action`` versus ``policy_action`` curves.
Intervention segments, success / failure events and the status are annotated, and a panel shows the episode summary (status, notes, history). Drag the timeline and the camera images and curves move together.

To view only some episodes, view on a remote machine, or save a file to take with you:

.. literalinclude:: ../../examples/inspect.sh
   :language: bash
   :start-after: # start view-options
   :end-before: # end view-options

``--web`` starts a web viewer on the robot computer and prints its address (web port 9090 and gRPC port 9876 by default); stop it with ``Ctrl-C``.
When the robot computer has no display, forward the ports from your own computer and open the printed address. Forward both ports, because the address carries the gRPC port:

.. tutorial-test: skip
.. literalinclude:: ../../examples/inspect.sh
   :language: bash
   :start-after: # start forward
   :end-before: # end forward

``--save x.rrd`` opens no viewer and only writes an ``.rrd`` file; open it with ``rerun x.rrd`` and pick an episode in the recording list in the same way.

**If it doesn't**:

* A message says rerun-sdk is needed: Install the viewer extra (``pip install -e '.[viewer]'``).
* No camera image: An episode without camera keys only has curves; when ``video/`` is missing, the viewer falls back to the lossless frames.
* You want to abandon an episode after watching it: ``rwrl dataset drop`` is reversible; see :doc:`../reference/recording-demos`.

2. Measure the intervention segments with the inspector
--------------------------------------------------------------------------------

Use the inspector to list episodes, human-control markers and measured displacement, and export CSV files for checking each row. ``archive inspect`` accepts a dataset directory directly (and also an old SQLite archive).

.. literalinclude:: ../../examples/inspect.sh
   :language: bash
   :start-after: # start inspect
   :end-before: # end inspect

**What you should see**: records, episode_starts, episode_ends, transitions,
human_mask_counts, outcomes and episodes in the JSON summary. Each steps.csv row corresponds to one environment step;
each measurements.csv row corresponds to one interval metric. The dataset is opened read-only.
Every episode (kept or discarded) counts as one episode_start and one episode_end; ``episode_id`` comes from each episode's ``meta.json``.

``human_action_mask`` is a boolean mask as long as the action. Each bit indicates whether a human controlled that action dimension.
The inspector prints it as a 0/1 string and segments consecutive identical masks: all zeros means hold; any 1 means intervene.
Here, hold is an interval within an episode without intervention. EPISODE_HOLD after an episode in the previous chapter is not written to the trajectory.
``override_mask`` marks dimensions whose actions were replaced, not necessarily all by a human. Use human_action_mask to find human-controlled intervals.

**If it doesn't**:

* Directory not found or not a dataset: Check the actual --record name; the directory should contain ``dataset.json``.
* CSV already exists: Choose a new export path. The inspector does not overwrite existing evidence.
* More episode_starts than episode_ends: There may be an unfinished episode. The inspector does not repair or end it for you.
* ``tcp_metrics: false``: Locatable single-arm TCP state / contract is missing, or the archive is bimanual. You can still inspect mask segments.

3. Run an offline example with a known answer
--------------------------------------------------------------------------------

Reuse the synthetic trajectory from the existing tests to learn the output format before inspecting your own recording.
The test writes this trajectory's TCP and mask values. They are neither simulated dynamics nor real-hardware measurements.
Install the test extra. Run once from the repository root, using an output directory that has not been used before.

.. tutorial-test: skip
.. literalinclude:: ../../examples/inspect.sh
   :language: bash
   :lines: 20-27

.. literalinclude:: ../../examples/inspect.sh
   :language: bash
   :lines: 28-28

**What you should see**: The actual run produces 48 records (start, 46 transitions, end),
21 steps with an all-zero mask and 25 steps with a human mask. The full output follows; temporary verification directories and local paths will differ.

.. literalinclude:: ../../examples/inspect-synthetic-output.txt
   :language: json
   :start-after: # start output
   :end-before: # end output

The answer for this example is: in ``episode-0``, steps 16–40, timestamps 1001.6–1004.0 seconds,
a human controls the six arm dimensions but not the gripper. The mask is ``1111110``.
step_index starts at 0. The interval includes both endpoints, for 25 steps.
The endpoint samples are 2.4 seconds apart. Do not interpret that as the total duration of 25 control cycles.

**If it doesn't**:

* Test files not found: Return to the repository root and confirm that the checked-out test files exist.
* Missing pytest or torch: Use an environment with learning and test installed.
* The export path already exists on a rerun: Choose a new directory. Do not accumulate repeatedly generated data in the same directory.

4. Compare human actions with measured pose changes
--------------------------------------------------------------------------------

Find the first human action in the step CSV and calculate the measured TCP change across that step.

.. tutorial-test: skip
.. literalinclude:: ../../examples/inspect.sh
   :language: bash
   :start-after: # start rows
   :end-before: # end rows

**What you should see**: The following is the actual output of this command.

.. literalinclude:: ../../examples/inspect-rows-output.txt
   :language: text
   :start-after: # start output
   :end-before: # end output

``action`` is the action actually passed to the environment; ``policy_action`` is the candidate action before intervention.
Without intervention, the recorder uses a hold policy, not a trained policy.
This test deliberately writes an all-zero action while the TCP changes along a predefined path. It verifies only the inspector's readings and segmentation;
it cannot establish that the action caused this motion.

For real recordings, decode action using the same episode's contract, then compare state and next_state.
With the earlier preset and current semantics, multiply the first three dimensions by 0.05 m and the three rotation dimensions by 0.3 rad. These are environment requests;
actual arm motion also depends on the controller, boundaries and tracking error.
The tutorial task's state_fields begins with tcp_pose, so TCP xyz occupies the first 3 entries of state.
This synthetic example places seven-dimensional joint_pos first, so the script takes entries 7–9. Check the contract before adapting the script; do not copy the slice blindly.
Quaternions use xyzw. Compare relative rotations for orientation changes; do not treat a quaternion difference directly as an angle.

The actual contents of the corresponding measurements.csv are:

.. literalinclude:: ../../examples/inspect-measurements.txt
   :language: text
   :start-after: # start output
   :end-before: # end output

``max_drift_mm`` is the maximum distance from the interval's first observation; residual_mm is the distance between its start and end.
The first takeover jump_mm in this example is 20 mm, comparing the previous segment's last obs with the first intervention step's next_obs.
The test specifies the operator stop time as 1002.4 seconds, yielding settle_s=0.8 and 30 mm residual displacement after the operator stops.
Real archives do not save teleoperation targets. action=0 does not automatically identify when the operator stopped moving the input.
timestamp_s is recorded after step, while xyz comes from obs before step, so absolute time has an offset of about one cycle.

``--stop-time-s`` uses Unix wall-clock seconds from the recording machine. Repeat it to specify multiple operator stop times.
The defaults are ``--settle-speed-mm-s 2.0`` and ``--settle-hold-s 0.5``: speed must remain below the threshold for the specified duration.
``--drift-window-s`` defaults to 10 seconds; this example explicitly changes it to 0.85 seconds. All three parameters must be greater than zero.
Without operator stop times, the exit code is 0. With them, it is 0 if all settle, or 3 if any do not.
If any time lacks a TCP episode covering it, the code is 4 (taking precedence over 3). Global ``--json`` output also has a
``status`` field: REPORT, PASS, FAIL or BLOCKED, respectively. The ordinary text JSON summary lacks this field.

**If it doesn't**:

* All-zero masks: Confirm that you actually took over during the episode. Do not count hold time outside an episode as human operation.
* Very different numerical scales: Check units, state_fields, action mode and current / serl semantics.
* Stop time outside an episode with TCP: Check the time source. Omit --stop-time-s if you only need intervals.
* No TCP metrics for bimanual data: Still check masks dimension by dimension in the left and right action blocks. Do not apply the single-arm slices here.

5. Check the training data and the action contract
--------------------------------------------------------------------------------

Training reads the dataset directory directly and uses the human-controlled steps of kept episodes. The snippet below fetches those rows with the same function training uses
and prints the action contract from the dataset header, so you can confirm that the filtered steps and the conventions needed for training are present.

.. tutorial-test: skip
.. literalinclude:: ../../examples/inspect.sh
   :language: bash
   :start-after: # start training-view
   :end-before: # end training-view

**What you should see**: transitions is the number of retained training rows, not the dataset's total step count.
Each row contains obs, next_obs, action, reward and done, and retains fields such as episode_id, step_index and
human_action_mask. The last step before a gap is marked training_segment_end to prevent training from joining trajectories across the gap.
With ``frame_stack=1`` images are not stacked; during training the image history is rebuilt to the run's ``obs.frame_stack``.
The action contract (formerly ``.contract.json``) is now ``action_contract`` in ``dataset.json``,
and ``obs_spec`` is the single-frame shape of each observation key. Check action_mode, arm_action_dim, gripper_in_action, state_fields,
the observation keys and shapes in experiment, action scales and experiment.sources.teleop_control. Keep these consistent with later training and deployment.
The earlier configuration should produce state and rgb_wrist with seven-dimensional actions; the GELLO joint-space extension has eight dimensions.

In ``experiment.sources.teleop_control``, the default Cartesian tracker is identified as ``cartesian_tracker_v1``.
It also records translation / rotation lead caps and target speeds; the SERL path is identified as ``spacemouse_serl_v1``.
These identifiers distinguish control rules, not physical input device models.
SpaceMouse currently also records ``axis_lock`` (default dominant), which GELLO's tracker block lacks.
Thus their default control conventions differ even with identical speeds and lead caps. Compare the full control block and the other action and
observation conventions. Do not merge data based only on a matching version identifier.

Retention requires a successful episode with human-controlled steps; teleop_syncing alignment steps are excluded by default.
Intervals without human operation in the same successful episode also stay out of the training rows. If success is labeled after handback,
the last human-controlled row may not contain a success event. Also check success and events in the episode's ``meta.json``.
Failures, rerecords, timeouts and interruptions remain in the dataset, flagged discarded with a reason (visible in ``rwrl dataset list``).
An absence of failures in the training rows does not mean there were no failed attempts on site.

**If it doesn't**:

* No training rows: No successful human demonstration was retained in the previous chapter. Check status and reason with ``rwrl dataset list`` first, then the masks.
* Fewer training rows than dataset steps: Account for the success, human-control and syncing filters in turn. This is usually normal filtering.
* The action contract does not match the task: Select the configuration and data from the same session. Do not guess from filenames or action dimensions alone.

If you have no dataset of your own, continue from the synthetic example and read the dataset it wrote:

.. tutorial-test: skip
.. literalinclude:: ../../examples/inspect-offline.sh
   :language: bash
   :start-after: # start training-view
   :end-before: # end training-view

**What you should see**: The actual output below shows one kept episode and 25 training rows (the 25 human-controlled steps),
with the last human step ``step_index=40`` marked as a segment end; the action contract is a six-dimensional Cartesian delta plus the gripper.
The values of this synthetic data were written by the test. Do not copy them into a robot configuration.

.. literalinclude:: ../../examples/inspect-offline-output.txt
   :language: text
   :start-after: # start output
   :end-before: # end output

**If it doesn't**: If test dependencies are missing, return to the installation chapter and install the test extra.

.. note::

   **What needs real-hardware confirmation in this chapter**

   This chapter's synthetic dataset and output were executed offline. Real intervention timing, the correspondence between sent actions and motion,
   camera contents, pose feedback, operator stop times and stopping behavior must be checked against real-hardware archives.
   Masks identify the control source; they alone cannot prove that the robot executed the requested actions.

Checklist: State the episode_id, start and end step_index / timestamps, and mask; locate action and the states before and after it;
explain which steps are missing from the training rows and where failed attempts are stored (in the dataset, flagged discarded).

The station and recording section is complete. Next, prepare training data in :doc:`bc-data`.
