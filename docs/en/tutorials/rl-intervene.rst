Make a human correction during training
================================================================================

This chapter follows just one takeover: what the policy proposed, what the human replaced, and which record the learner received.
Keep the learner and device service from :doc:`rl-train` running. Both input paths share the record checks below.
The observations and seven-dimensional cartesian_delta actions must match each path's recording session.

1. Take over and hand back with your input device
--------------------------------------------------------------------------------

Choose only the device calibrated during recording. The commands below are alternatives to the Chapter 3 actor command.
If an actor is already running, follow the operating steps directly; do not start a second actor.
Keep the Chapter 3 ``LEARNER_HOST`` setting. Both machines use the configuration generated in Chapter 2.

.. warning::

   The actor will reset, home, open the gripper, and execute the policy. Handback restores policy control, and the arm may immediately resume motion.
   Confirm the stop method, workspace, home path, and on-site supervision first; handback is not a stop command.

.. tab-set::

   .. tab-item:: SpaceMouse
      :selected:

      Use the direction profile and tracker configuration from :doc:`spacemouse`. First confirm that the cap and all buttons are released.

      .. literalinclude:: ../../examples/rl-intervene.sh
         :language: bash
         :start-after: # start spacemouse
         :end-before: # end spacemouse

      #. Wait for reset and START_NEUTRAL to finish; press button_0 to take over when a correction is needed.
      #. Push the cap gently to correct position or orientation; during takeover, press button_1 to toggle the gripper open/closed.
         Releasing the cap stops advancing the target, but the arm may still travel the remaining lead distance, and takeover remains active.
      #. Press button_0 again to hand back to the policy. Terminal t/h does not switch control in this mode.
      #. Press s or f to label the actual outcome. At EPISODE_HOLD, choose n for the next episode or q to exit.

      **What you should see**: human_action_mask is nonzero during human control; after handback, subsequent human masks return to zero,
      while policy_action continues to come from the policy. The gripper has its own actual-control marker; use the records as the source of truth.

      **If it doesn't**: if pushing the cap immediately takes over, check whether serl was selected by mistake. If buttons do nothing, recheck HID reads and button numbering.
      At WORKSPACE HOLD, check directions and workspace before taking over again; do not enlarge the boundaries to keep trying.
      If you choose SERL, use the SERL configuration and data independently verified in :doc:`spacemouse`: motion/button levels trigger takeover,
      release hands back, button_0 closes, and button_1 opens. Do not change only the mode on the fly and combine training data.

   .. tab-item:: GELLO

      Use the leader_cartesian session from :doc:`gello`, preserving the leader-options.yaml path and calibration.
      This controls relative end-effector changes, not joint_abs/joint_delta from the joint-space extension.

      .. literalinclude:: ../../examples/rl-intervene.sh
         :language: bash
         :start-after: # start gello
         :end-before: # end gello

      #. Wait for reset; keep the leader arm steady and its own Space toggle off so the neutral check passes.
      #. Press t in the actor terminal. The current leader pose becomes the relative-mapping anchor; pressing t again does not hand back.
      #. Move the leader gently to correct the motion; during takeover, first match the gripper trigger to the last commanded state, then open or close it.
      #. Press h to hand back to the policy; the next t re-anchors. Use s/f to label outcomes and n/q during hold.

      **What you should see**: actions remain six-dimensional Cartesian deltas plus the gripper; the human mask is nonzero during takeover and zero after h.
      The leader's absolute joint pose need not match the follower joint by joint. Alignment here means relative mapping and trigger matching.

      **If it doesn't**: if START_NEUTRAL stalls, check the physical toggle and intervention: always.
      If the serial port cannot open, follow :doc:`gello` to check franka.os:serial, device permissions, and whether another process owns it.
      If toggling Space does not take over, use t/h from the current event source. If joint directions are wrong, return to calibration; do not hide the problem with software labels.

With residual ACT, ``--residual-base-resync-on-intervention`` is enabled by default.
After every intervention step, the provider resets before computing the base action for the next observation, so it replans a chunk from the actual new state;
the takeover step itself keeps the base action computed before takeover. BC checkpoint and zero providers are stateless and unaffected.

``--no-residual-base-resync-on-intervention`` disables this behavior: temporal_agg=0 continues replaying the old chunk,
while temporal_agg=1 keeps mixing chunks from before/during takeover. base_actions, next_base_actions, and handback actions may lag by one chunk.
**What you should see**: after intervention, the next observation's base action comes from the first action of a new chunk. When checking records, do not mistake the current step's base action for a recomputed one.
**If it doesn't**: if handback follows the old plan, check this switch and ACT aggregation options first; do not immediately increase the residual scale.

2. Find this correction on the learner
--------------------------------------------------------------------------------

In the Chapter 3 journal inspection output, find a transition with nonzero human_action_mask.
Note its complete record_id, episode_id, and step_index, then read it separately.

.. tutorial-test: skip
.. literalinclude:: ../../examples/rl-intervene.sh
   :language: bash
   :start-after: # start match
   :end-before: # end match

**What you should see**: the same row contains the following evidence.

.. list-table::
   :header-rows: 1
   :widths: 35 65

   * - Field
     - How to check it here
   * - ``policy_action``
     - The action the actor proposed at that time. Do not predict again with the current checkpoint and present that as the historical proposal.
   * - ``action``/``executed_action``
     - The request submitted to the environment after human replacement; both should match in this online record. Read obs/next_obs to check actual TCP tracking.
   * - ``human_action_mask``/``override_mask``
     - The first marks actual human control per dimension; the second may also include non-human filling. Use the first for intervention statistics.
   * - ``record_id``/``episode_id``/``step_index``
     - The record's identity across machines, its episode, and its step order. journal_or_outbox_seq is a database-local sequence number and cannot match records across machines.
   * - ``policy_session_id``/``policy_version``/``global_update``
     - Which policy produced this action. A later version with a positive update count proves parameters returned to the actor, not that any particular sample was necessarily selected.

Finding the same record_id in the journal means the learner received it; received/intervened_received in update logs
show ingestion progressing. Standard HIL-SERL replay retains only fields needed for training and does not log “which SGD update sampled this record” per sample.
This chapter therefore traces journal receipt, the replay-entry rules, and new policy publication. It does not claim a measurable causal contribution from one correction.

**If it doesn't**: if the ID cannot be found, first confirm you are inspecting the same learner run. If the outbox still contains the ID, wait for upload and check
upload_retries. ACKed outbox rows are deleted; the learner journal holds persistent evidence in production runs.
Do not treat an empty outbox as lost data. Rows with syncing=True are still archived, but their training projection clears human supervision, so they do not count as human demos.

3. Check the same ID and training fields with the CPU example
--------------------------------------------------------------------------------

Complete the Chapter 3 exercise first, then compare its extra actor audit copy with the learner journal.
That audit copy is added specifically for this tutorial; you do not need to modify the production actor to read learner records.

.. tutorial-test: skip
.. literalinclude:: ../../examples/rl-intervene.sh
   :language: bash
   :start-after: # start rehearsal
   :end-before: # end rehearsal

**What you should see**: the inspector compares policy_action, action, human_action_mask, and reward by ID,
and prints matching_id_and_action_mask_reward=True only when they match. Below is an actual CPU record and comparison result:

.. literalinclude:: ../../examples/rl-intervene-output.txt
   :language: text
   :start-after: # start correction
   :end-before: # end correction

The fake environment has only three action dimensions; do not copy them to the seven-dimensional real robot. The injected action is [0.25,-0.25,1], with an all-true mask.
Chapter 3's learner/received=6, intervened_received=2, and demo_buffer_size=2 agree with the two correction steps.
All six records enter online replay, and the two human corrections also enter demo replay; demo is not a partition moved out of online.
Real training also has seed demonstrations, so demo_buffer_size need not equal intervened_received, and it stops growing once capacity is reached.

**If it doesn't**: on matching record missing, check that the two databases being compared came from the same exercise run.
If a tensor assertion fails, preserve the original files and IDs for investigation; do not manually change action to make it pass.
If policy and human actions happen to match, that does not prove there was no takeover; use human_action_mask to decide.

.. note::

   **What needs hardware confirmation in this chapter**

   Confirm physical takeover timing, directions, trigger latching, policy motion after handback, and requested actions versus measured TCP on site.
   The CPU example verified matching action/mask/reward for the same ID; it did not verify USB, serial, network timing, or actual operator actions.

Checklist: identify one takeover and handback; find the proposal, execution request, and mask;
find the same ID in the learner journal and a new policy version in later actor records.

Next: :doc:`rl-resume`.
