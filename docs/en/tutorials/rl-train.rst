Start HIL-SERL training
================================================================================

This chapter starts the device service, learner, and actor in that order, then uses records to verify data and parameter flow in both directions.
Run all commands from the repository root, using each machine's environment with learning installed; the device service needs only device-side dependencies.
Use ``local/rl-task.yaml`` from :doc:`rl-task`. The hardware commands below were not run locally.

.. tip::

   Device commands in this chapter omit ``--station``: when run from the repository root, they use the station remembered by ``rwrl use``
   in Step 10 of :doc:`station`; without a remembered station, they also find ``local/station.yaml`` directly. To use another file, pass ``--station`` explicitly.

1. Know where the first policy comes from
--------------------------------------------------------------------------------

Confirm the initialization method before starting so you can decide when the operator should take over.
A single-gripper task constructs ``RLPDHybrid``: six continuous arm-action dimensions and one discrete gripper head.
A contract without a gripper uses ``ContinuousHilRLPD``. This chapter does not automatically initialize from the earlier BC final.pt.

The learner starts with a new network and loads ``--demo-dataset-paths`` data into the demo buffer.
Demonstrations do not count toward ``learning_starts``; updates begin only after the 100 online transitions in this example have arrived.
Training batches mix online and demonstration data according to demo_data_ratio, and online human corrections keep expanding the demo buffer.
This entry point does not “train BC on demonstrations first, then go online.”

The actor also constructs a policy of the same shape with small buffers, then waits for the learner's initial snapshot to overwrite its weights.
The initial snapshot can have ``phase=warmup`` and ``policy_ready=False``; the HIL-SERL actor
still calls policy predict. This flag does not automatically hold the arm or wait for BC to finish.
A ready policy is published after the first training update, then at publish_freq intervals.

**What you should see**: the earliest records in a new experiment have global_update=0; positive values appear later.
``--load-actor-checkpoint`` supports only continuous policies; the hybrid arm actor and DQN gripper
are incompatible with a full continuous-action BC head, so passing this argument raises an explicit error. Use the complete bundle from Chapter 5 to resume HIL-SERL training.
**If it doesn't**: if old versions/replay appear at startup, check whether you reused the first-hil checkpoint directory.
If passing a BC file to --load-checkpoint causes an algorithm or weight incompatibility, select the HIL-SERL bundle instead.

2. Start the device service on the robot PC
--------------------------------------------------------------------------------

Stop recording, deployment, and processes directly connected to devices first. Use the reviewed station to provide the sole hardware connection.
The 5 seconds here follows this repository's daemon example; review it on site against the longest action interval measured in the deployment chapter and the acceptable stop delay.

.. warning::

   The device service will own real hardware; the actor's later reset will home and open the gripper, and the policy may move immediately afterward.
   Confirm home, workspace, limits, stop methods, and on-site supervision. Do not run two actors at once.

.. literalinclude:: ../../examples/rl-train.sh
   :language: bash
   :start-after: # start daemon
   :end-before: # end daemon

**What you should see**: the daemon keeps serving, with a listening port matching remote in rl-task.yaml.
The learner should not control the arm at this point; do not open the same arm with a direct FCI client.
This chapter does not fabricate successful daemon startup logs; use the connection acceptance checks from :doc:`deploy`.

**If it doesn't**: if devices are busy, close old recording/deployment processes. If the declared daemon is unreachable, check the station's
listening address, port, and token. If the watchdog repeatedly triggers, revisit the deployment chapter's timing budget and control frequency.

**Optional direct connection**: if you decide to let the actor own hardware directly, stop the daemon first, then copy
local/rl-task.yaml to local/rl-direct.yaml. Remove real_robot.remote
and put the commissioned station's single-arm robot configuration mapping into real_robot.robot; do not use a station path or copy the daemon block.
Choose exactly one of robot and remote. Both machines must use the same observation, action, home, workspace, and teleop configuration.
The commands below first print configuration, then compare the station with doctor. doctor initializes devices, so ensure no other process owns them.

.. literalinclude:: ../../examples/rl-train-alternatives.sh
   :language: bash
   :start-after: # start direct-check
   :end-before: # end direct-check

**What you should see**: print-config contains a robot mapping and no remote; doctor's <arm name>.preset is PASS.
If hardware fields differ, it lists <arm name>.preset:<field> WARN with both station/preset values,
including IP, backends, gripper, cameras, home, joint limits, frames, and workspace.

**If it doesn't**: fix each item in the copy; do not remove warnings. Successful parsing does not mean hardware is available.
Then change the configuration in both Step 3 commands to local/rl-direct.yaml and start the actor through the supervisor:

.. warning::

   The command below resets and runs the policy. The direct actor is the sole hardware owner; confirm on-site supervision and stop methods first.

.. literalinclude:: ../../examples/rl-train-alternatives.sh
   :language: bash
   :start-after: # start direct-actor
   :end-before: # end direct-actor

The supervisor's heartbeat timeout defaults to 2 seconds, and its startup grace before the first heartbeat defaults to 30 seconds; here 60 seconds is explicit to allow loading.
This differs from the actor's sync_startup_timeout and from the environment's StepWatchdog.
real_robot.watchdog_s defaults to None and was not enabled in the original direct-connection example; a session-generated configuration may retain a recording-time value, so inspect the actual configuration.
StepWatchdog runs in the same process and cannot cover a hang of the entire process; the daemon watchdog is a separate path.
**What you should see**: the actor enters reset/neutral checking after receiving the initial snapshot.
**If it doesn't**: if it exits before the first heartbeat, check loading and startup grace; if heartbeats disappear during operation, investigate the blockage rather than only extending startup time.

3. Start the learner on the training machine, then the actor on the robot PC
--------------------------------------------------------------------------------

Start the learner first so the actor can connect within the initial-snapshot deadline.
The training machine must be able to read the pkl, contract, rl-task.yaml, and referenced teleop configuration. remote points to the service
on the robot PC, but the learner only constructs spaces; it does not require the training machine to open that connection.

.. literalinclude:: ../../examples/rl-train.sh
   :language: bash
   :start-after: # start learner
   :end-before: # end learner

**What you should see**: config.json, experiment.json,
and checkpoints/journal.sqlite3 appear under local/rl-runs/first-hil. Without actor data, the learner waits; do not expect continuous loss output.
This entry point writes metrics to TensorBoard and does not provide a fixed “learner ready” terminal line.

**If it doesn't**: on Unknown env backend 'real_robot', complete the editable installation/backend registration.
A demonstration path with no matches may yield an empty demo buffer; check the actual files first. For contract errors, compare the original session.
In particular, do not overwrite the previous chapter's cartesian_delta with the joint_delta default from examples/presets/hil_single_arm.yaml.

Next, set the training machine's actual LAN address on the robot PC. Use 127.0.0.1 if both run on the same machine.
This actor uses the SpaceMouse or GELLO configuration selected in Chapter 2; there is no need to run teleop run alongside it.

.. warning::

   The next command resets and starts policy control. The initial policy is untrained; the operator should be ready to take over immediately,
   with a physical stop device available. n enters the next episode and resets again.

.. literalinclude:: ../../examples/rl-train.sh
   :language: bash
   :start-after: # start actor
   :end-before: # end actor

**What you should see**: the actor first waits for the initial snapshot; START_NEUTRAL appears after reset. Release the input so the neutral check passes.
Below is a hardware-run format example reproduced from session.py, not a local hardware measurement:

.. literalinclude:: ../../examples/rl-train-format.txt
   :language: text
   :start-after: # start hold
   :end-before: # end hold

Make a few human corrections and label outcomes during active episodes; after an episode ends, the actor waits at EPISODE_HOLD.
Press n to request the next episode and release the input, or q to exit. The learner runs independently; stop it with Ctrl-C in its own terminal.

**If it doesn't**: on ``initial learner policy was not available before startup deadline``, check the learner
and port 6000; sync_startup_timeout defaults to 30 seconds. If START_NEUTRAL does not finish, return to the input-device chapter and check resting values/toggle state.
On ``learner policy experiment hash differs from actor``, align the configurations and reference files; do not remove validation to bypass it.

4. Check data, ingestion, then parameters
--------------------------------------------------------------------------------

Read the learner journal first, then metrics; finally check the version used by the actor to prove an update returned to the control side.

.. tutorial-test: skip
.. literalinclude:: ../../examples/rl-train.sh
   :language: bash
   :start-after: # start inspect
   :end-before: # end inspect

.. tutorial-test: skip
.. literalinclude:: ../../examples/rl-train.sh
   :language: bash
   :start-after: # start metrics
   :end-before: # end metrics

**What you should see**:

* Collection: transition step_index increases, with obs/next_obs, action, and reward. The inspector prints only key fields;
  raw tensors remain in the SQLite payload. episode_end contains this episode's outcome.
* Ingestion: ``learner/received`` and ``learner/buffer_size`` grow; human corrections also increase
  ``learner/intervened_received`` and ``learner/demo_buffer_size``.
  ``learner/admitted_records`` includes episode-boundary records and is not the transition count.
* Updates: ``learner/update`` is greater than 0; ``learner/journal_backlog`` is the backlog received but not yet applied.
  Metrics are recorded when received samples cross log_freq boundaries; without new samples, new scalars may not be written even if gradient updates continue.
* Back at the actor: later transitions show changes in ``policy_session_id``, ``policy_version``, and
  ``global_update``, with policy_phase changing from warmup to online. These fields are written after the actor loads a snapshot,
  providing stronger evidence that it used new parameters than learner/update alone.

``actor/actor/outbox_depth`` and ``actor/actor/upload_retries`` come from episode statistics with actor_id=actor in this example;
``transport/actor/outbox_depth`` comes from episode_end. Outbox rows are deleted after ACK, so it cannot serve as a permanent history query.
Compare parameter-version ordering only within the same learner session; restarting the learner changes the session, and version may decrease.

**If it doesn't**: if received stays flat while admitted_records grows, check journal_backlog and ingestion errors.
If received has not reached 100, collect more data; adding demonstration files does not bypass the threshold. If only learner/update changes, check the synchronization link
and wait for the actor to execute another step. If TensorBoard has no scalars, confirm log_type, the directory, and that log_freq has been crossed at least once.

Loss names differ by method. Learner metrics use received transition count as their horizontal axis
and are recorded at log_freq boundaries (default 1000), not by seconds or global_update.
Actor episode statistics gain an ``actor/<actor_id>/`` prefix on the learner; this chapter defaults to actor/actor/.
Only old statistics without actor_id use actor/ directly.

.. list-table:: Metric directory (availability depends on the method and data)
   :header-rows: 1
   :widths: 55 45

   * - Metric
     - How to read it
   * - return, length, terminated, task_success, human_episode_end under actor/<actor_id>/
     - Episode return/length, ending, and explicit outcome; terminated does not mean task success.
   * - autonomous_success_rate, task_success_rate, episode_intervention_rate, intervention_rate, intervened_steps, intervention_count under the same prefix
     - Distinguish autonomous outcomes, assisted outcomes, human steps, and takeover counts.
   * - safety_clips, safety_clips_total under the same prefix
     - When the device provides them, the episode's clipping increment and cumulative count respectively; continued growth calls for checking out-of-bounds requests.
   * - rtt_ms_mean, rtt_ms_max, step_time_ms_mean, step_time_ms_max, param_refresh_ms_mean, param_refresh_ms_max under the same prefix
     - Network, step, and parameter-refresh timing; RTT appears only if the environment provides it.
   * - outbox_depth, upload_retries under the same prefix
     - Upload backlog and retries; retry count is not the number of lost rows.
   * - learner/received, learner/update, learner/buffer_size, learner/train_time_s, learner/demo_buffer_size, learner/intervened_received
     - Ingestion, updates, capacity, and human demonstration growth.
   * - losses/critic_loss, losses/qf_loss, losses/actor_loss, losses/alpha_loss, entropy/alpha, entropy/target_entropy, q/predicted
     - SAC-style HIL-SERL/residual metrics; exact fields depend on the continuous/hybrid variant and update phase.
   * - losses/actor/residual_bc, losses/actor/intervened_fraction
     - The residual human-fitting term and batch intervention fraction; the actor is not updated during the critic-only phase.
   * - losses/critic/loss, losses/actor/loss, losses/actor/q, losses/actor/distance
     - SiLRI's critic, actor, and distance from the human behavior model.
   * - losses/lambda/loss, losses/lambda/mean, losses/behavior/sigma_sum, losses/batch/intervened_fraction, losses/discrete/ce
     - SiLRI's constraint multiplier, human behavior uncertainty, and human fraction; discrete/ce appears only with a discrete gripper head.
   * - behavior/pretrained, behavior/pretrain/*, behavior/refresh/*, behavior/refresh_count
     - SiLRI pretraining and later refreshes; compare actor/distance with sigma_sum plus constraint_margin to assess the constraint state.

**What you should see**: metrics for this run's method advance as records arrive and updates progress.
**If it doesn't**: check the method, actor_id, log_freq, and current phase first; do not expect every variant to produce the entire table at once.

5. Exercise the data chain on CPU first
--------------------------------------------------------------------------------

Run this six-step exercise from the source environment to become familiar with the records and recovery results; the output directory must not already exist.

.. tutorial-test: skip
.. literalinclude:: ../../examples/rl-train.sh
   :language: bash
   :start-after: # start smoke
   :end-before: # end smoke

The script reuses the fake environment and small CPU RLPDHybrid from tests/test_real_world_hil_serl_loops.py,
running the real HilSerlActorLoop, journal ingestion, gradient updates, and bundle recovery.
Each of two episodes has three steps, with a human action injected at step 2; success and failure events are injected at their final steps respectively and handled by the real reward wrapper.
The script wraps observation in the state dictionary required by the policy; it replaces the HTTP service and transport scheduling.
The learner updates once per received record; do not use this speed to estimate on-site throughput or network latency.

A local attempt to create a loopback socket produced the following measured error, so the two-process HTTP acceptance test was not run:

.. literalinclude:: ../../examples/rl-train-format.txt
   :language: text
   :start-after: # start socket
   :end-before: # end socket

**What you should see**: below is the script's actual stdout. Summaries such as received are printed by the exercise script; learner/* names and values
come from real loop metrics. This does not claim that the production CLI prints these lines step by step.

.. literalinclude:: ../../examples/rl-train-output.txt
   :language: text
   :start-after: # start smoke
   :end-before: # end smoke

At the end, the learner has updated 6 times, and the actor's last action used update 5; update 6 was produced only after the final action,
so this difference of one is not lost parameters. The complete code is in ``docs/examples/rl-train-smoke.py``.
The script additionally saves actor-audit.sqlite3 for comparison in Chapter 4; the production actor does not keep this permanent audit copy by default.

**If it doesn't**: if the directory already exists, choose a new one. If tests modules are missing, run from the full repository root.
If torch/gymnasium is missing, use the rlg environment with learning and test installed. For ordinary source CLI help, use:

.. literalinclude:: ../../examples/rl-train.sh
   :language: bash
   :start-after: # start source-help
   :end-before: # end source-help

6. Alternative: train a residual over BC or ACT
--------------------------------------------------------------------------------

This workflow replaces HIL-SERL in Step 3. Both sides still use the verified local/rl-task.yaml.
Run only one method; stop the old actor/learner first. This example isolates records with separate exp_name and run_id values.
First choose the same base specification on both machines:

.. literalinclude:: ../../examples/rl-train-alternatives.sh
   :language: bash
   :start-after: # start base-bc
   :end-before: # end base-bc

For ACT, use the following group instead; do not execute both groups consecutively as one run:

.. literalinclude:: ../../examples/rl-train-alternatives.sh
   :language: bash
   :start-after: # start base-act
   :end-before: # end base-act

Both sides must receive the base specification; only actor/eval actually construct the base model. The learner trains from uploaded
base_actions and next_base_actions. Base files and metadata must be readable on the actor machine.
Run this on the training machine:

.. literalinclude:: ../../examples/rl-train-alternatives.sh
   :language: bash
   :start-after: # start residual-learner
   :end-before: # end residual-learner

**What you should see**: a journal is created in the separate run directory, waiting for online data.
**If it doesn't**: check the base specification, configuration, and experiment identity first; do not mix in the old first-hil journal.

.. warning::

   The actor below resets and executes the base policy plus residual. A loadable base model is not necessarily safe to execute.
   Complete input/action contract checks first, and keep takeover, on-site supervision, and a physical stop device available.

.. literalinclude:: ../../examples/rl-train-alternatives.sh
   :language: bash
   :start-after: # start residual-actor
   :end-before: # end residual-actor

Continuous dimensions execute clamp(base + scale * residual, -1, 1); human takeover overrides the total action.
hybrid adds residuals only to arm dimensions and uses a DQN head for the gripper; continuous uses the residual formula for all dimensions,
but residual_gripper_scale defaults to 0, preserving the base gripper. This example explicitly selects hybrid; the code defaults to continuous.
residual_action_scale=0.1 is a normalized-action scale; critic_only_updates=500 trains only the critic for the first 500 updates,
and residual_bc_weight=0.1 fits human corrections on human-supervised dimensions. The latter two settings both default to 0 in code.

Ordinary teleop pkl files have no base actions, so this example does not pass demo_dataset_paths.
Only ``--allow-base-less-demos`` permits loading such demonstrations with zero base actions; this does not recompute their base actions with the frozen model.
It changes the residual learning target, so do not enable it merely to remove an error.

**What you should see**: ingestion and updates progress; residual actor metrics appear only after the critic-only phase ends.
**If it doesn't**: for mismatched base weights, first follow :doc:`bc-check`; see :doc:`rl-intervene` for ACT replanning after intervention.

7. Alternative: SiLRI's human behavior model
--------------------------------------------------------------------------------

SiLRI also replaces Step 3. Copy local/rl-task.yaml to local/rl-silri.yaml
and change utd to integer 1 (SiLRI requires int; the original template's 1.0 is float). Keep the remaining task/observation/teleop contract.
The commands below name a separate new experiment; run the learner first on its machine, then the actor on the robot PC.

.. literalinclude:: ../../examples/rl-train-alternatives.sh
   :language: bash
   :start-after: # start silri-learner
   :end-before: # end silri-learner

Before the first training update, the learner pretrains the human behavior model from the demo buffer; bc_pretrain_steps defaults to 500.
During pretraining, received rows can still be written to the journal and consumed afterward; as online human demonstrations grow, refresh conditions trigger behavior-model retraining.
By default, every 50 new demos trigger 50 refresh steps, controlled by behavior_refresh_every and behavior_refresh_steps.

**What you should see**: behavior/pretrained and behavior/pretrain/*, followed by behavior/refresh/* and refresh_count.
**If it doesn't**: check that seed demonstrations loaded and that demo_learning_starts (default 10) and online learning_starts are satisfied.
For a configuration type error, first change utd to an integer; do not remove task fields to bypass validation.

.. warning::

   The actor resets and executes the policy. SiLRI's behavior constraint is not a safety guarantee; keep verified takeover and physical stop methods available.

.. literalinclude:: ../../examples/rl-train-alternatives.sh
   :language: bash
   :start-after: # start silri-actor
   :end-before: # end silri-actor

**What you should see**: initial policy receipt, episode records, and new parameter versions, checked as in Steps 3 and 4.
**If it doesn't**: check the separate run_id, synchronization address, and outbox first; do not start a second actor on the same robot.


.. note::

   **What needs hardware confirmation in this chapter**

   Hardware commands, two-machine HTTP, daemon gRPC, input devices, image throughput, control periods, the initial random policy's behavior,
   and stopping timing still need on-site confirmation. The CPU exercise measured six samples, two human demos, six updates, parameter refresh, and bundle recovery;
   it used neither GPU nor external network, and did not measure socket synchronization.

Checklist: know that initialization is not BC; samples precede updates; find a positive global_update at the actor;
distinguish measured script output from hardware format examples.

Next: :doc:`rl-intervene`.
