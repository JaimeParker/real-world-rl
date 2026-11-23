Deploy a BC policy trained in this repository
================================================================================

This chapter uses ``local/bc-runs/first-bc/checkpoints/final.pt`` from :doc:`bc-check`.
Restore the configuration and check saved observations first, then arrange a supervised episode.
Keep config.json, experiment.json and offline-check.json in the same directory.
For existing openpi / RLinf weights, use entry B: :doc:`deploy-openpi`.

The BC entry point in this chapter is ``real_world_rl/deploy/checkpoint.py``, corresponding to
``rwrl deploy --algo real_world_bc`` and ``real-world-rl-deploy``.
``examples/deploy_real_robot.py`` forwards to the same entry point; ``deploy-act`` loads a different kind of ACT weights.
This entry point has no dry-run parameter. The offline part of this chapter only restores configuration or predicts from saved observations, without calling reset / step.

Run commands from the repository root with a Python environment that has the learning dependencies installed. In a source environment, first set:

.. literalinclude:: ../../examples/deploy-bc.sh
   :language: bash
   :start-after: # start environment
   :end-before: # end environment

The measured output below comes from CPU execution in the rlg environment, with synthetic data and four-step training regenerated as in :doc:`bc-train`.
The synthetic checkpoint verifies loading only; it does not enter a real-hardware episode.

Choose the entry point by training method first. Steps 1–4 below still use BC as the example.

.. list-table:: Checkpoints and deployment algorithms
   :header-rows: 1

   * - Training method
     - deploy --algo / standalone entry point
     - Weights
   * - BC
     - real_world_bc
     - final.pt from the corresponding run
   * - HIL-SERL (with the tutorial gripper)
     - rlpd_hybrid
     - The corresponding run's manifest or .pt
   * - residual_hil_serl
     - residual_hil_serl for continuous; residual_hil_serl_hybrid for hybrid
     - The residual run's manifest or .pt, plus the base model
   * - SiLRI
     - silri
     - The corresponding run's manifest or .pt
   * - ACT
     - rwrl deploy-act (step 6)
     - The ACT trainer's final.pt and the demonstration contract

A manifest resolves to the model generation it references. Deployment neither restores replay nor connects to the learner.

1. Restore the configuration and compare with the current station
--------------------------------------------------------------------------------

Print the configuration the model actually uses, then compare it item by item with the previous chapter's input / output record and the on-site configuration.
``deploy-bc.py`` is this chapter's helper script. By default, it only calls restore_training_args and
resolve_deploy_robot_config from the deployment module, without establishing device connections.

.. tutorial-test: skip
.. literalinclude:: ../../examples/deploy-bc.sh
   :language: bash
   :start-after: # start inspect
   :end-before: # end inspect

**What you should see**: The following is actual output from the synthetic checkpoint. obs determines RGB cameras and image sizes.
An empty real_robot.cameras here does not mean that the model uses no cameras; obs is also applied when constructing the environment.

.. literalinclude:: ../../examples/deploy-bc-output.txt
   :language: text
   :start-after: # start config
   :end-before: # end config

Check the following against offline-check.json. Do not compare action length alone:

.. list-table:: Pre-deployment comparison
   :header-rows: 1
   :widths: 30 70

   * - Model record
     - What to check at the current station
   * - cameras, observation_shapes, observation_dtypes
     - obs in local/task.yaml matches the restored result; the station has wrist, and its actual view, serial number and mounting position match the demonstrations.
       Here, rgb_wrist is [1,224,224,3] uint8. Also check depth, preprocess and frame_stack.
   * - state_fields
     - Here, tcp_pose, tcp_vel, joint_pos and joint_vel concatenate to 27 dimensions. Preserve their order, units, xyzw quaternion convention and task frame.
       Check experiment for rotation conversions.
   * - action_conventions, action_contract
     - Here, current / cartesian_delta, seven dimensions, position scale 0.05 m, rotation scale 0.3 rad, 10 Hz.
       The gripper becomes an opening request through (a+1)/2. Also check the station's binary threshold, joint_limits, workspace, control and teleop provenance.
   * - station and the original session resolved-config
     - Devices, home, workspace, per-step limits and control settings must match the reviewed configuration. Connection addresses can change; changing cameras or action semantics is not merely changing an address.

Deployment restores training arguments from config.json and checkpoint metadata. Training observation / action conventions take precedence;
the CLI supplies connection addresses. It does not automatically read the current local/task.yaml or local/station.yaml.
For the remote deployment below, the daemon loads the station. Configure the service and tunnel as in :doc:`deploy-openpi`;
this chapter reuses its GPU-side 127.0.0.1:50061.

After moving directories, use ``--config-path`` to specify the training config.json; deploy does not accept the training entry point's --config.
Keep experiment.json and the generation files referenced by the bundle. If the residual base model moves separately, use
``--base-provider`` to specify its actual specification; moving only the residual weights is insufficient.

After moving a classifier, use ``--classifier-checkpoint`` for its new location. Replace per-arm task classifier mappings in the corresponding task configuration.
The replacement classifier's content is provenance, but task, control, preprocessing, observation layout and intervention parts must still match the original experiment.
Changing a model path does not remove the other contract checks.

Verify the rejection path with a deliberately conflicting scale. This command is still offline:

.. tutorial-test: skip
.. literalinclude:: ../../examples/deploy-bc.sh
   :language: bash
   :start-after: # start mismatch
   :end-before: # end mismatch

**What you should see**: The process fails, ending with the following exception from the deployment module's actual check:

.. literalinclude:: ../../examples/deploy-bc-output.txt
   :language: text
   :start-after: # start mismatch
   :end-before: # end mismatch

**If it doesn't**:

* ``contradicts the training run's real_robot config``: Check the listed fields against the training record. Do not change model scales to values you want on site.
  After checking them, remove the conflicting overrides so deployment restores the training values.
* No error does not mean you can skip comparison: The CLI check treats only fields differing from dataclass defaults as explicit overrides.
  It also cannot verify actual camera views, units or mounting changes.
* Missing config.json / experiment.json, or records belonging to another checkpoint: Recover the full run and demonstration provenance first, then return to :doc:`bc-check`.
* The current task, state order or camera differs from the record: Fix the configuration or prepare data and the model again before running an episode.

2. Check a saved observation first
--------------------------------------------------------------------------------

Reuse the previous chapter's offline checking script and retain a separate prediction record for this deployment.
Load only checkpoints and pickle files you generated yourself or trust.

.. tutorial-test: skip
.. literalinclude:: ../../examples/deploy-bc.sh
   :language: bash
   :start-after: # start offline
   :end-before: # end offline

For the synthetic exercise, use this instead. Do not rename synthetic data as real demonstrations:

.. tutorial-test: skip
.. literalinclude:: ../../examples/deploy-bc.sh
   :language: bash
   :start-after: # start synthetic
   :end-before: # end synthetic

**What you should see**: Actual CPU output follows. deterministic=true, and the model is in eval mode.

.. literalinclude:: ../../examples/deploy-bc-output.txt
   :language: text
   :start-after: # start offline
   :end-before: # end offline

The predicted x is about -0.1588, requesting about -0.0079 m at this example's scale. This is a requested increment, not measured displacement.
The model also outputs motion in other directions absent from the demonstrations. This four-step synthetic model cannot be used for the on-site task.
For your own model, check observations from several independent episodes before deciding whether to continue.

**If it doesn't**:

* ``recorded action contract does not match`` or ``saved experiment contract/hash differs``:
  Check the data and training directory using the error fields. Do not delete contracts to bypass validation.
* ``prediction/action/mask shape mismatch`` or ``non-finite prediction or no supervised dimensions``:
  Return to :doc:`bc-check` to inspect inputs, actions and masks.
* ``FileExistsError``: Keep the old record and give --record a new filename.

3. Run one supervised episode
--------------------------------------------------------------------------------

After passing the on-site checks, use the same deployment loop for one episode of at most ten steps, saving a per-step record.
Inspect the actual CLI parameters first:

.. literalinclude:: ../../examples/deploy-bc.sh
   :language: bash
   :start-after: # start help
   :end-before: # end help

The equivalent standard entry point command follows. It prints only an episode summary and does not save per-step trajectories.
Choose either this or the recording command below; this chapter uses the latter.

.. warning::

   Both commands below connect to devices, reset to home and open the gripper, then send policy actions.
   Check the real model, home path, workspace, gripper and object placement first. Keep on-site supervision and a physical stop device available.
   This example disables teleoperation intervention. SpaceMouse is not a stop button.
   Interrupting the current process with Ctrl-C enters environment cleanup and disconnects devices. Confirm on site that the arm actually stops;
   use the physical stop device immediately for abnormal motion. A ten-step limit is no motion-safety guarantee and does not include reset time.

.. literalinclude:: ../../examples/deploy-bc.sh
   :language: bash
   :start-after: # start cli
   :end-before: # end cli

This chapter's script calls build_deploy_env, restore_policy and run_deploy, adding a recording wrapper
without changing policy outputs. It restricts execution to the BC chapters' task.name=none, CPU, deterministic inference, one episode and a ten-step limit,
and disables teleoperation. Check task configuration and evaluation logic separately for custom tasks.
Run the following and save the terminal output as well:

.. tutorial-test: skip
.. literalinclude:: ../../examples/deploy-bc.sh
   :language: bash
   :start-after: # start episode
   :end-before: # end episode

**What you should see**: The terminal first prints the restored configuration. After normal completion, ``[deploy] episode=0`` and
``[deploy] summary`` appear, including steps, return, truncated, RTT and health status.
No real-hardware run was performed here, so no invented episode output is provided. The BC chapters have no success criterion by default;
reaching ten steps is usually truncation, and a zero return cannot be interpreted as task success or failure.

**If it doesn't**:

* ``[deploy] device error`` (standard CLI) or a device exception (helper script): Follow the hint; see :doc:`deploy-openpi` for link and daemon diagnostics.
* ``FileExistsError``: The helper script refuses to reuse an output directory. Choose a new --output and update the tee log path accordingly.
* Convention / weight mismatch while constructing the environment: Stop and recheck the full training configuration, experiment and section 1 record.
* Abnormal action direction, speed or health status: Stop the episode, save logs and recheck conventions. Do not increase limits to keep trying.

After catching a device error, the standard deploy CLI prints ``[deploy] device error: ...`` to stderr,
then ``[deploy] hint: ...`` if a hint exists, and exits with code 1. Command-line parsing errors exit with code 2.
A contract mismatch raises ValueError before reset and is not part of the device-error summary above.
These exit conventions apply to standard deploy, not this chapter's helper script or deploy-act.

4. Read policy actions, measured motion and results
--------------------------------------------------------------------------------

Compare requests and feedback using the same step number to distinguish what the policy intended from what the arm actually did.

.. tutorial-test: skip
.. literalinclude:: ../../examples/deploy-bc.sh
   :language: bash
   :start-after: # start records
   :end-before: # end records

**What you should see**: The helper script writes the files below. The standard CLI itself does not generate these JSON files.

.. list-table:: Execution records for this chapter
   :header-rows: 1
   :widths: 40 60

   * - Path
     - Contents and interpretation
   * - local/deploy-bc-episode/steps.jsonl
     - The reset row saves the initial state. Each step's proposal row saves policy_action and time, followed by a feedback row saving
       state, tcp_pose, reward, terminated and truncated. Each row is flushed immediately.
   * - local/deploy-bc-episode/result.json
     - EpisodeResult after normal completion: steps, episode_return, truncated, rtt_ms_mean, health_ok and outcome.
       task_success=null means unknown. health_ok=true cannot replace a success criterion.
   * - local/deploy-bc-episode.log
     - tee saves the configuration, deployment summary and exceptions. Per-step records contain no images; also archive the original checkpoint, offline-check and station/task together.

The first seven state entries in this example are TCP XYZ and xyzw. Compare the reset state with the first feedback,
then adjacent feedback states, to measure position changes in m. Joint feedback occupies state entries 13–19
(numbered from 0), in rad. tcp_pose comes from device feedback in the environment step.
These are poses at sample times, not a full motion trajectory or proof that the robot has settled at the target.
Do not subtract quaternion components and treat the result as a rotation angle, or treat normalized policy_action as measured displacement.

This example has no human intervention, so proposal is the action passed to env.step. Environment mapping, clipping and device limits may still change the target.
Separately record actual task success / failure, the basis for that judgment, object state and whether stopping was normal on site. Save these in
``operator-notes.txt`` in the same directory. Do not rewrite unknown labels in machine records.

**If it doesn't**:

* A proposal has no feedback with the same number: step did not return normally, but the action may already have been sent. Do not conclude that no motion occurred.
* An interrupted row exists but no result.json: This is an unfinished attempt. Keep steps and the terminal exception; do not fabricate a completed episode.
* Missing state / tcp_pose or an unexpected layout: Stop interpreting displacement and recheck observation fields and the feedback path.
* Measured changes differ greatly from requests: Check clipping, workspace, control cycle and device feedback, not just cumulative reward.

5. Alternative: deploy residual weights or retain human intervention
--------------------------------------------------------------------------------

After completing ACT residual training in :doc:`rl-train`, keep the full run and base model on the robot PC
and replace step 3 with the command below. For a BC-based residual run, substitute your own first-residual-bc directory and checkpoint: base specification.
The example below uses the connection configuration saved during training. If addresses change, override only connection fields you have checked.

.. warning::

   This section's commands reset, open the gripper and execute the policy. End the online actor first and check the base model's and residual's action conventions.
   Keep on-site supervision and physical stopping means. The absence of intervention does not mean unattended operation has been validated.

.. literalinclude:: ../../examples/deploy-alternatives.sh
   :language: bash
   :start-after: # start residual
   :end-before: # end residual

**What you should see**: The same [deploy] episode and summary lines as in step 3.
**If it doesn't**: If the base path moved, change its path in --base-provider while preserving original options such as temporal_agg.
For the wrong residual variant, select the algorithm used during training; do not adapt it by changing the action width.

To retain GELLO intervention, use the leader_cartesian configuration, referenced files and stdin event source verified in that run.
The following is for HIL-SERL weights trained with GELLO. A SpaceMouse run cannot simply switch to this device name.

.. literalinclude:: ../../examples/deploy-alternatives.sh
   :language: bash
   :start-after: # start assisted
   :end-before: # end assisted

enable_teleop defaults to true, but with task.name=none, an explicit teleop-device is also needed to enable source composition;
a versioned task also triggers composition. There is no default device name; it comes from explicit arguments or saved configuration.
``--no-enable-teleop`` skips construction of intervention sources; devices already declared in the station may still open during device initialization.

**What you should see**: t takes over, h hands back, s/f label outcomes, and Esc or q ends the current episode. Reports include intervention and autonomous / task success rates.
This example has one episode and exits after it ends. General deploy does not check operator_quit to exit a multi-episode loop,
so multiple episodes still continue to reset. Use Ctrl-C to interrupt the entire deployment process.
Labeling s/f alone does not count as action intervention. A successful episode with actual takeover counts only as task success, not autonomous success.
**If it doesn't**: A leader-arm read failure stops deployment. Check referenced files and device ownership first.
For SpaceMouse, explicitly select spacemouse and preserve the original tracker / SERL contract; see :doc:`rl-intervene`.

Bimanual models require ``--env-backend real_robot_bimanual`` and restoration of both arms' observation, action and connection settings.
Check per-arm configuration against :doc:`../reference/bimanual`. A single-arm checkpoint cannot become bimanual by changing one flag.
A bimanual reset moves both arms, so check each home and their mutual collision-clearance space first.

6. Alternative: run ACT independently
--------------------------------------------------------------------------------

Use the ACT weights from step 4 of :doc:`bc-train`. Explicitly provide the recording task configuration and demonstration contract here;
do not pass an ACT file to real_world_bc. local/rl-task.yaml must be generated from the same recording session.
Preserve the original model's state, camera and action settings. Deployment checks the demonstration contract against model widths / camera order before reset.

.. warning::

   The command below connects to devices, resets and executes ACT actions, with human intervention disabled.
   Run under on-site supervision only after completing the convention checks in :doc:`bc-check`. Keep the physical stop device available.

.. literalinclude:: ../../examples/deploy-alternatives.sh
   :language: bash
   :start-after: # start act
   :end-before: # end act

ACT defaults to temporal_agg=true and clears chunk / aggregation state at the start of each episode.
``--no-temporal-agg`` switches to chunk-by-chunk replay. Do not mix the two settings partway through evaluation.
``--demo-contract`` takes the ``.contract.json`` that ``rwrl dataset export --format pkl`` (step 4 of :doc:`bc-train`) writes next to the pkl.

General deploy and deploy-act share run_deploy. They do not perform the training eval role's START_NEUTRAL /
EPISODE_HOLD / n workflow, nor wait in the loop to fill out the control_hz period. Saved hold configuration does not change this.
If you need that neutral check, episode hold and loop pacing, use the corresponding training method's eval entry point in :doc:`rl-resume`.

**What you should see**: [deploy] episode and summary lines after loading / convention checks pass.
No ACT real-hardware episode was run here, and the local environment lacking torchvision did not complete ACT model loading either.
**If it doesn't**: Missing --checkpoint or --demo-contract is rejected. For camera or width mismatches, recover the matching configuration and demonstration contract (the dataset header's action_contract, or the sidecar written when exporting the pkl);
do not edit the contract to make checks pass. Deployment performs no training updates or data uploads. Archive evaluation results separately.

.. note::

   **What needs real-hardware confirmation in this chapter**

   Synthetic CPU training, configuration restoration, scale-conflict rejection and saved-observation inference were actually run.
   Camera views, state units, home / reset paths, action mapping, measured motion, control timing,
   supervised episodes, per-step on-site records, stopping behavior and task outcomes all need real-hardware confirmation.

Checklist: before real-hardware reinforcement learning
--------------------------------------------------------------------------------

* Verify the actor's observation and action configuration against offline-check, and explain how the policy produces actions.
* Know how to stop the current episode, locate policy requests, measured feedback and episode results, and distinguish unknown, truncated and successful outcomes.
* Save the station and experiment configuration. Real-hardware reinforcement learning (starting at :doc:`rl-overview`) reuses them; HIL-SERL policy initialization is explained separately.

See :doc:`deploy-openpi` for remote services, tunnels and more complete deployment checks.
Next, follow one time step in :doc:`rl-overview` to see how an online experiment runs.
