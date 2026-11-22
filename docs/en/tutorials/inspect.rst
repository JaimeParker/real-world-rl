Inspect recording results
================================================================================

This chapter continues with the ``local/first-demo`` output from :doc:`record`. All operations are offline and do not connect to the robot.
Open the full archive first, then check the training data. The goal is to explain which steps of which episode you controlled, and which dimensions you controlled.

1. Open the full archive
--------------------------------------------------------------------------------

Use the inspector to list episodes, human-control markers and measured displacement, and export CSV files for checking each row.

.. literalinclude:: ../../examples/inspect.sh
   :language: bash
   :start-after: # start inspect
   :end-before: # end inspect

**What you should see**: records, episode_starts, episode_ends, transitions,
human_mask_counts, outcomes and episodes in the JSON summary. Each steps.csv row corresponds to one environment step;
each measurements.csv row corresponds to one interval metric. The source SQLite is opened read-only.

``human_action_mask`` is a boolean mask as long as the action. Each bit indicates whether a human controlled that action dimension.
The inspector prints it as a 0/1 string and segments consecutive identical masks: all zeros means hold; any 1 means intervene.
Here, hold is an interval within an episode without intervention. EPISODE_HOLD after an episode in the previous chapter is not written to the trajectory.
``override_mask`` marks dimensions whose actions were replaced, not necessarily all by a human. Use human_action_mask to find human-controlled intervals.

**If it doesn't**:

* File not found: Check the actual --record basename. The archive suffix is .trajectory.sqlite3.
* CSV already exists: Choose a new export path. The inspector does not overwrite existing evidence.
* More episode_starts than episode_ends: There may be an unfinished episode. The inspector does not repair or end it for you.
* ``tcp_metrics: false``: Locatable single-arm TCP state / contract is missing, or the archive is bimanual. You can still inspect mask segments.

2. Run an offline example with a known answer
--------------------------------------------------------------------------------

Reuse the synthetic trajectory from the existing tests to learn the output format before inspecting your own recording.
The test writes this trajectory's TCP and mask values. They are neither simulated dynamics nor real-hardware measurements.
Install the test extra. Run once from the repository root, using an output directory that has not been used before.

.. tutorial-test: skip
.. literalinclude:: ../../examples/inspect.sh
   :language: bash
   :lines: 5-13

.. literalinclude:: ../../examples/inspect.sh
   :language: bash
   :lines: 14-14

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
* The export path already exists on a rerun: Choose a new directory. Do not accumulate repeatedly generated archives in the same file.

3. Compare human actions with measured pose changes
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

4. Check the training pkl and contract
--------------------------------------------------------------------------------

Open the pkl you just recorded. Confirm that the filtered steps and the conventions needed for training are present.
Load only pickle files you generated yourself or trust.

.. tutorial-test: skip
.. literalinclude:: ../../examples/inspect.sh
   :language: bash
   :start-after: # start pkl
   :end-before: # end pkl

**What you should see**: transitions is the number of retained training rows, not the full archive's step count.
Each row contains obs, next_obs, action, reward and done, and retains archive fields such as episode_id, step_index and
human_action_mask. The last step before a gap is marked training_segment_end to prevent training from joining trajectories across the gap.
In the contract, check action_mode, arm_action_dim, gripper_in_action, state_fields,
the observation keys and shapes in experiment, action scales and experiment.sources.teleop_control. Keep these consistent with later training and deployment.
The earlier configuration should produce state and rgb_wrist with seven-dimensional actions; the GELLO joint-space extension has eight dimensions.

In ``experiment.sources.teleop_control``, the default Cartesian tracker is identified as ``cartesian_tracker_v1``.
It also records translation / rotation lead caps and target speeds; the SERL path is identified as ``spacemouse_serl_v1``.
These identifiers distinguish control rules, not physical input device models.
SpaceMouse currently also records ``axis_lock`` (default dominant), which GELLO's tracker block lacks.
Thus their default control conventions differ even with identical speeds and lead caps. Compare the full control block and the other action and
observation conventions. Do not merge data based only on a matching version identifier.

Retention requires a successful episode with human-controlled steps; teleop_syncing alignment steps are excluded by default.
Intervals without human operation in the same successful episode also stay out of the pkl. If success is labeled after handback,
the last human-controlled row may not contain a success event. Also check episode_end.outcome in the archive.
Failures, rerecords, timeouts and interruptions remain in the full SQLite, with their results in outcomes.
An absence of failure rows in the pkl does not mean there were no failed attempts on site.

**If it doesn't**:

* No pkl: No successful human demonstration was retained in the previous chapter. Check SQLite outcomes and masks first.
* Fewer pkl rows than archive steps: Account for the success, human-control and syncing filters in turn. This is usually normal filtering.
* The contract does not match the task: Select the configuration and data from the same session. Do not guess from filenames or action dimensions alone.

If you have no pkl of your own, continue from the synthetic example and run the fake environment in the existing recorder test:

.. tutorial-test: skip
.. literalinclude:: ../../examples/inspect-offline.sh
   :language: bash
   :start-after: # start record
   :end-before: # end record

Then read the files it generated:

.. tutorial-test: skip
.. literalinclude:: ../../examples/inspect-offline.sh
   :language: bash
   :start-after: # start pkl
   :end-before: # end pkl

**What you should see**: The actual output below shows one retained pkl row with its mask and an eight-dimensional joint_abs contract.
This test specifically uses a joint-space fake environment. It is not a training export of the earlier Cartesian trajectory.
The ±2 limits and joint_scale_rad=0.05 are test data. Do not copy them into a robot configuration.

.. literalinclude:: ../../examples/inspect-offline-pkl-output.txt
   :language: text
   :start-after: # start output
   :end-before: # end output

**If it doesn't**: If demos.pkl already exists, choose a new offline directory. If test dependencies are missing, return to the installation chapter and install the test extra.

.. note::

   **What needs real-hardware confirmation in this chapter**

   This chapter's synthetic archive and output were executed offline. Real intervention timing, the correspondence between sent actions and motion,
   camera contents, pose feedback, operator stop times and stopping behavior must be checked against real-hardware archives.
   Masks identify the control source; they alone cannot prove that the robot executed the requested actions.

Checklist: State the episode_id, start and end step_index / timestamps, and mask; locate action and the states before and after it;
explain which steps are missing from the pkl and where failed attempts are stored.

The station and recording section is complete. Next, prepare training data in :doc:`bc-data`.
