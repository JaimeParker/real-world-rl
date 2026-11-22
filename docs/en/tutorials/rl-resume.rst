Evaluate a policy and resume training
================================================================================

This chapter saves a recoverable training state, then evaluates a fixed policy independently.
Do not count a task completed with human help as autonomous success; do not claim to have restored the complete online experiment by copying only model weights.

1. Save the model, replay, and receipt progress
--------------------------------------------------------------------------------

This example uses checkpoint_freq=100 to save checkpoints by the learner's global_update;
buffer_period=100 writes legacy pkl snapshots by the number of transitions received. These are two different counters.
For a normal stop, first let the actor exit with q during hold, confirm that the arm has stopped, then press Ctrl-C in the learner terminal.
Letting the actor drain uploads first reduces records left in the outbox after the learner stops.

During cleanup, the actor attempts to drain uploads within ``--sync-drain-timeout``, which defaults to 30 seconds.
After timeout, unACKed rows remain in the local outbox and are resent on the next restart with the same identity. Do not forcibly delete records just to empty the outbox.
Concurrent actors must each have a separate ``--actor-id`` and ``--actor-outbox-path``
and control separate devices; this does not allow two actors to control this tutorial's single arm simultaneously.

**What you should see**: final.pt and bundle_manifest.json under checkpoints; periodic saves may also include
checkpoint_<update step>.pt. The manifest points to a bundle_<update step>_<uuid> directory containing
agent.pt, replay.pt, and experiment.json. A complete bundle saves the model/optimizer, online and demo replay,
ring-buffer positions, loop counters, and journal cursor/identity.

A generation becomes a recovery point only after the manifest is atomically published. Normal Ctrl-C attempts a final save; training errors or interrupted ingestion
may leave the previous generation in place. A new final.pt is not guaranteed on every exit. Do not select the newest half-written directory if the manifest does not reference it.

**If it doesn't**: if periodic files are missing, check whether global_update reached 100. save_final_checkpoint=false means no final save.
If disk space runs out or an update fails, preserve the logs and use the last complete manifest. The buffer/demo_buffer pkl files are compatibility snapshots,
not complete recovery points with optimizer state and a consistent cursor.

2. Check local evidence after an interruption
--------------------------------------------------------------------------------

Check the outbox on the actor machine and the journal and manifest on the learner machine to confirm they still belong to the same experiment.
With two machines, run the corresponding lines of the following file on each machine separately.

.. tutorial-test: skip
.. literalinclude:: ../../examples/rl-resume.sh
   :language: bash
   :start-after: # start local-data
   :end-before: # end local-data

**What you should see**: unACKed records in the outbox, received records in the journal, and the last complete episode-end information.
These are complementary evidence: an empty outbox can mean everything was delivered. An unfinished episode from a killed process cannot count as success;
on restart, the actor adds a process_restart interruption boundary for its persisted active episode.

Keep the complete run directory, original demonstrations and sidecars, generated configuration, teleop reference files, and station version.
SQLite uses local WAL; keep the database and any -wal/-shm files. To migrate, stop writers and copy the complete directory;
do not copy only the main .sqlite3 file while it is live. Do not mix a journal with another experiment's bundle.
Read only checkpoints/pickles you generated or trust.

This implementation's synchronous=NORMAL lets committed records recover across process crashes, but does not guarantee that the last few commits survive a power loss.
A step can also be lost if the actor is forcibly killed after the arm executes but before the actor commits. During a network interruption, the actor keeps collecting with its existing policy,
stopping only when the outbox reaches capacity or persistence fails; this differs from waiting for the first snapshot at startup.

**If it doesn't**: without a manifest, you can only reconstruct usable data from the existing journal/old snapshots; you cannot claim to restore lost optimizer state.
If the database is empty but actions were unfinished, preserve on-site records for investigation. If directories belong to different experiments, select the matching ones; do not modify identity to bypass checks.

3. Resume the learner, then the actor
--------------------------------------------------------------------------------

On the learner machine, resume with the original directory and manifest so the journal identity matches the bundle cursor.

.. literalinclude:: ../../examples/rl-resume.sh
   :language: bash
   :start-after: # start resume
   :end-before: # end resume

**What you should see**: the online/demo buffers and update count restore first, only the journal tail after the saved cursor is replayed,
and then the initial policy is published and actors are accepted. A complete bundle's demo buffer already contains retained seed demonstrations, so they are not loaded again.
The Chapter 3 CPU run restored received=6, update=6, demo=2, with a replayed tail of 0.
The old hil-serl-roadmap statement that “seed demos reload on every startup” describes the migration period; current complete-bundle recovery does not add them again.

``--load-replay-buffer`` controls the companion replay for legacy standalone .pt files. Selecting a bundle always restores the complete consistent unit;
--no-load-replay-buffer cannot turn it into a weights-only load. Passing the manifest while changing checkpoint-dir creates a mismatched journal.

After confirming on-site safety and daemon recovery, rerun the original Chapter 3 actor command, keeping run_id, actor_id, and the outbox path.
Only one writer process may use a given outbox; unACKed records are resent, and the learner deduplicates by stable ID.
policy_session_id changes after a learner restart; do not require policy_version to exceed the old session's maximum.

.. warning::

   Restarting the actor runs reset again and may home and open the gripper. It does not continue from the middle of the interrupted trajectory.
   Clear object positions and check fault recovery and stop devices before running.

.. literalinclude:: ../../examples/rl-train.sh
   :language: bash
   :start-after: # start actor
   :end-before: # end actor

**If it doesn't**: on ``checkpoint journal identity differs``, retrieve the original journal and WAL;
on ``checkpoint seed dataset contents differ``, restore the original demonstration contents;
on ``durable store belongs to a different experiment``, check the configuration. Use a new directory for an independent experiment, not the old outbox.

4. Deploy the fixed new policy for independent evaluation
--------------------------------------------------------------------------------

Stop the online actor first, then copy the complete saved run to the robot PC (no copy is needed if the learner is on that machine).
This chapter deploys a fixed snapshot through hil_serl's eval role, keeping the same env and event wrapper to obtain both success rates directly.
See :doc:`deploy` for general deployment and observation/action checks.

.. warning::

   eval resets and executes the new policy. It uses the same control_hz as the actor and waits out the remainder when inference and step take less than one period.
   This example keeps episode_end_behavior=hold: confirm neutral input after reset, hold after each episode, and press n to start the next;
   after the budget is exhausted, only q can exit. auto mode resets automatically. Keep human takeover and physical stop methods available.

.. literalinclude:: ../../examples/rl-resume.sh
   :language: bash
   :start-after: # start eval
   :end-before: # end eval

Residual BC/ACT and SiLRI use their respective eval entry points. For residual evaluation, first reset BASE and RUN from
Step 6 of :doc:`rl-train` in the current shell (choose BC or ACT), and keep the training hybrid and residual parameters. Select only the eval command for your method.
These also reset and execute the policy; follow this section's motion warning and stop the corresponding actor first.

.. literalinclude:: ../../examples/rl-resume.sh
   :language: bash
   :start-after: # start residual-eval
   :end-before: # end residual-eval

.. literalinclude:: ../../examples/rl-resume.sh
   :language: bash
   :start-after: # start silri-eval
   :end-before: # end silri-eval

**What you should see**: the same per-episode [eval] output and autonomous/task success-rate summary, with deterministic policy prediction.
**If it doesn't**: check whether you passed another method's checkpoint to this entry point; residual eval still requires the base specification.
To resume training, also use the original method, configuration/parameters, and that run's manifest and checkpoint-dir; do not reuse the first-hil directory.

This entry point does not connect to a learner, train, or run the actor's outbox archiving loop.
Policy prediction is deterministic; checkpoint resolution and contract validation finish before reset. This chapter still requires a matching rl-task.yaml on the command line.
Save eval output through redirection or the terminal. If you need a step-by-step action audit, arrange collection separately; do not claim that this command automatically generates a journal.
Avoid takeover during normal evaluation where possible, and press s/f only when the task actually succeeds/fails. If takeover is necessary, retain the intervention statistics honestly.

**What you should see**: per-episode [eval] lines and a final summary. Below is actual CPU output from the manually labeled fake environment described in Chapter 2,
using the same evaluate_policy function; it is not a measured result from the real-robot eval command above:

.. literalinclude:: ../../examples/rl-resume-output.txt
   :language: text
   :start-after: # start eval
   :end-before: # end eval

.. list-table::
   :header-rows: 1
   :widths: 36 64

   * - Actual metric name
     - How it is counted
   * - ``task_success_rate``
     - Completed episodes with explicit task_success=True / all completed episodes; success with human help counts too.
   * - ``autonomous_success_rate``, ``success_rate``
     - Explicitly successful episodes with no human-control steps / all completed episodes. success_rate is an alias for autonomous success rate.
   * - ``episode_intervention_rate``, ``step_intervention_rate``
     - The fraction of completed episodes with intervention, and the fraction of human steps among all steps in completed episodes.
       Pressing only s/f to label outcomes, without a nonzero human_action_mask, does not count as action intervention.
   * - ``unknown_episodes``, ``completed_episodes``
     - Completed episodes without an explicit label, and all completed episodes. unknown remains in the success-rate denominator; do not hide it.
   * - ``interrupted_episodes``, ``discarded_episodes``
     - Separately listed interrupted/discarded episodes; they do not enter the completed-episode denominator.
   * - ``tcp_proxy_success_rate``
     - A TCP-position proxy result. This tutorial does not configure that proxy; it cannot replace object-task success.

During online training, the same summaries appear as ``episodes/task_success_rate``, ``episodes/autonomous_success_rate``,
and episode statistics such as this example's ``actor/actor/task_success_rate``. Online statistics reflect training with exploration and human corrections;
independent eval uses a frozen snapshot and deterministic actions. Do not combine them into one acceptance number.

**If it doesn't**: high task_success_rate but low autonomous_success_rate means completion depends on human help.
If unknown_episodes is high, check the label source. An error saying eval needs a checkpoint means --load-checkpoint was not supplied.
If reset happens automatically when unexpected, stop evaluation and check that episode_end_behavior is hold in the configuration.

.. note::

   **What needs hardware confirmation in this chapter**

   Arm stopping after normal exit and forced termination, device fault recovery, another reset, network retransmission, independent evaluation timing, and success criteria
   need on-site validation. The CPU run measured bundle save/restore and both success rates; it did not verify power loss, a two-machine disconnection, or real task success.

Checklist: keep the matching journal, manifest, and generation; seed demonstrations are not added twice after recovery;
report task success and autonomous success separately; connect one correction's ID, the learner receipt record, and a later actor version.

This completes the real-robot reinforcement learning section. Continue with :doc:`deploy` to check independent deployment of the selected policy.
