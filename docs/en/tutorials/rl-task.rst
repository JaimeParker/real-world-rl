Prepare a trainable task
================================================================================

This chapter gives the same task from :doc:`record` an explicit reset, end conditions, success labels, and rewards.
Use the supported manual-event path: the operator presses s after seeing the task succeed, and f after seeing it fail.
``env_id: pick_and_place`` is only the experiment name; it does not detect whether the object has been placed correctly.

1. Generate the training configuration from a verified session
--------------------------------------------------------------------------------

Read the configuration template first, then merge observation, action, and teleop parameters from the actual ``first-demo`` session.
Do not treat the template below as an already calibrated on-site configuration.

.. literalinclude:: ../../examples/rl-task.yaml
   :language: yaml
   :start-after: # start config
   :end-before: # end config

.. list-table::
   :header-rows: 1
   :widths: 35 65

   * - Fields you must change
     - How to obtain the value
   * - ``obs`` and the action, state, scale, and limits (``limits``) fields in ``real_robot``
     - Copy from the session/resolved-config.yaml actually used in the previous chapter; the script below preserves these blocks in full.
       If inspect reports rgb_wrist as [1,224,224,3], the corresponding settings are rgb: [wrist] and image_size: [224,224].
   * - ``real_robot.remote`` and, if needed, ``token``
     - Use the station's daemon listening address/port. If the actor and daemon share a machine and the port is 50051,
       use 127.0.0.1:50051; copy the matching token for stations that enable one. The script removes the original direct-connection robot block.
   * - ``teleop_device``, ``teleop_options_yaml``, ``teleop_input``, and related fields
     - Copy from the session. SpaceMouse should use spacemouse; the previous chapter's GELLO Cartesian path should use leader_cartesian.
       Keep leader-options.yaml, leader-robot.yaml, and their references in the session directory.
   * - ``real_robot.max_episode_steps``, ``reset_settle_s``
     - Earlier chapters use 200 steps and 0.5 seconds respectively; the script uses the actual session. Compute the task budget from the verified frequency.
       Change it only after confirming on site that the wait is sufficient. Do not set episode length from the learner's speed.
   * - ``reward_from_teleop_events``, ``teleop_success_reward``
     - This chapter selects true and 1.0. The operator confirms success against the task definition, such as the object resting stably in the target area.
       See Section 3 for the key test; do not infer real success from episode return.
   * - ``--demo-dataset-paths`` (passed through the CLI)
     - Use the dataset directory local/first-demo verified in :doc:`inspect`;
       you can also set ``demo_dataset_paths: [local/first-demo]`` in YAML. CLI arguments take precedence.
   * - ``buffer_size``, ``demo_buffer_size``
     - The template allocates 1000 rows each, for short sessions only. Adjust demo capacity using the training-row (transition) count from :doc:`inspect`.
       It must hold all seed demonstrations and leave room for online corrections; estimate image memory before extending the session.
   * - ``log_dir``, ``exp_name``, ``run_id``, ``actor_id``
     - Here these are local/rl-runs, first-hil, first-hil, and actor respectively. Use new names and a new outbox for an independent experiment.
       Keep them when resuming the same experiment; see :doc:`rl-resume`.

Keep the remaining template settings: buffer_device=cpu; the commands hide the GPU with CUDA_VISIBLE_DEVICES; batch_size=32, learning_starts=100,
demo_data_ratio=0.5, train_freq=1, utd=1, publish_freq=10, log_freq=10,
checkpoint_freq/buffer_period=100, save_final_checkpoint=true.
These are tutorial values for a short session, not all HIL-SERL defaults or recommended convergence settings. Adjust them after data volume and collection speed stabilize.
classifier_checkpoint=null and use_grasp_penalty=false keep a single reward source.
The template explicitly sets ``encoder.plain_conv_pooling: gap``: plain_conv's default flatten pooling only fits 128×128 and 64×64 images, so 224×224 needs gap (the reason is in :doc:`bc-train`).
HIL-SERL's Args has no device field like the BC entry point. Policy construction selects a device automatically, so these commands explicitly hide the GPU.

The default HIL-SERL/residual_hil_serl online capacity is 200000 and demo capacity is 100000;
SiLRI uses 30000 and 10000 respectively. All default to buffer_device=cuda.
HIL-SERL defaults to learning_starts=4000 and publish_freq=100; demonstrations do not count toward the online startup threshold.
checkpoint_freq defaults to 0, saving only at the end; this chapter's 100 explicitly enables periodic saves.

A bundle copies the allocated storage of both replays to the host and writes it to disk. Do not estimate its cost from the current valid row count alone.
Multiple image streams take substantial space; each periodic save also creates a new generation directory. Leave headroom in GPU memory, host memory, and disk.
Keep the generation referenced by the manifest and its matching journal; see :doc:`rl-resume` for recovery.

real_robot.control_hz determines the control frequency. A non-default global ``--control-hz``
only fills in an environment frequency still at the default 10 Hz; if both are non-default and differ, configuration is rejected before device connection.
Use the resolved frequency to check recording, training, and deployment. Do not silently change the experiment contract through a global parameter.
See :doc:`controller` for on-site limits on impedance overrides.

List the previous chapter's artifacts first, then replace ``<uuid>`` in the command with the actual directory name:

.. tutorial-test: skip
.. literalinclude:: ../../examples/rl-task.sh
   :language: bash
   :start-after: # start files
   :end-before: # end files

.. tutorial-test: skip
.. literalinclude:: ../../examples/rl-task.sh
   :language: bash
   :start-after: # start prepare
   :end-before: # end prepare

The generation script preserves the session's model inputs, actions, and teleop calibration. It changes only the connection to remote and applies this chapter's training settings.
``local/station.yaml`` and ``local/task.yaml`` stay unchanged. Copy the generated configuration, the dataset directory, and
referenced teleop files to the learner machine (the lossless frames in a dataset are large; check ``du -sh`` first); the paths must resolve there too. The learner reads the calibration configuration but does not open devices.

**What you should see**: the script prints the config path, remote, action_mode, and teleop_device.
The action mode should be cartesian_delta and the device should match your chosen input device. In a text editor, check that real_robot no longer contains a direct robot configuration,
and that home/limits/control settings still come from the original session or the same daemon station.

**If it doesn't**: on FileExistsError, inspect the existing local/rl-task.yaml; do not overwrite a running experiment.
If the script rejects joint actions, select the previous chapter's Cartesian session. If GELLO reference files are missing, copy the complete session.
If no daemon is configured, first establish the connection using the device-service workflow in :doc:`deploy`; do not guess the remote address.

2. Check reset, ending, and reward separately
--------------------------------------------------------------------------------

Keep ``task.name: none`` here. Reuse the previous chapter's demonstration contract and supply task labels through the event wrapper.
reset uses the original real_robot.home_joints, or the station's home if unset; it then opens the gripper and waits reset_settle_s.
The seven home values come from the q readings and reviewed starting pose in :doc:`station`; no generic joint angles are supplied here.

``episode_end_behavior: hold`` makes the actor hold after an episode ends and wait for n, then pass a neutral check before the next reset.
It does not change handback between policy and human control within an episode. ``max_episode_steps`` is the truncation limit.
Training commands default to ``auto``: the env resets right after an episode. When a person has to rearrange the scene (an object fell on the table, or must be swapped), use ``episode_end_behavior: confirm`` instead:
the actor waits in POST_EPISODE and READY for a person, who presses ``o``/``c`` for the gripper, ``n`` to reset (a partly closed gripper is asked about once) and space to start the next episode;
the actor pushes no trajectory in those phases, and the learner keeps training on what it has. confirm needs an event device (``--teleop-event-device stdin``, ``keyboard`` or ``jsonl``)
and supports Cartesian and joint arms; this chapter is a single-operator Cartesian session and keeps ``hold``. The phases and keys are in :doc:`../reference/recording-demos`.
A success event makes reward at least 1 and sets terminated=True. A failure event takes priority over success in the same step and sets
task_success=False and truncated=True. The underlying reward is 0 in this configuration, so a failure row has reward=0;
the failure wrapper itself does not guarantee zeroing an arbitrary classifier reward. Esc only ends the episode; it does not add a success label.

Run a parse-only preflight to confirm the fields and path sources:

.. literalinclude:: ../../examples/rl-task.sh
   :language: bash
   :start-after: # start preflight
   :end-before: # end preflight

**What you should see**: the resolved real_robot, teleop, reward, and demo_dataset_paths.
This example passes the dataset path through the CLI; the current field type is ``tuple[str, ...]``, and YAML lists are also accepted.
print-config does not confirm demonstration contents or hardware availability; the Chapter 3 learner checks the contract when it actually loads the data.

For automatic success detection, the code supports classifier_checkpoint, classifier_threshold (default 0.5), and
classifier_image_keys. First collect success/failure images separately and validate the classifier.
This chapter does not run a classifier: its construction path uses get_device("auto"), so buffer_device=cpu alone does not restrict the classifier's device.
The versioned ``real_robot.task.name: pick_and_place`` also supports staged reset, task rewards, and end configuration,
but changing task changes the experiment contract; it cannot directly reuse the old task=none demonstrations here.
``tcp_proxy`` only checks whether the TCP is near a position; it cannot prove a successful grasp or placement.

**If it doesn't**: demo_dataset_paths must be a list of path strings, not a single string.
On ``recorded action contract does not match``, compare each field with the session;
do not edit the dataset header's action_contract. If labels are missing, check that stdin is actually connected to the actor terminal and reward_from_teleop_events=true.

3. Confirm one success and one failure
--------------------------------------------------------------------------------

Inspect the existing dataset first. If the previous chapter has no failure episode, return to :doc:`record` and record one explicit failure and one success.
Use a new output name and replace the paths below accordingly. Failed episodes remain in the dataset (flagged discarded); by default they do not enter the successful human demonstrations used for training.

.. tutorial-test: skip
.. literalinclude:: ../../examples/rl-task.sh
   :language: bash
   :start-after: # start labels
   :end-before: # end labels

**What you should see**: One line for each of two different episodes, with task_success true and false respectively,
both with outcome_source=manual; the failure has end_reason=manual_failure, status discarded and reason failure event.
Read the last step of each episode too, with ``rwrl dataset view`` or ``archive inspect --csv``, and check reward, terminated, and truncated.
The script only reads each episode's meta.json.

Chapter 3 provides a hardware-free CPU exercise. Below are two episode-end records actually produced by that exercise;
the UUIDs and timestamps came from that run. These are not real task labels:

.. literalinclude:: ../../examples/rl-intervene-output.txt
   :language: json
   :start-after: # start labels
   :end-before: # end labels

**If it doesn't**: not finding failures in the training rows alone is normal filtering. unknown/timeout means no explicit outcome,
not a failure label. If outcome_source is not manual, check whether a classifier or versioned task was additionally enabled.

.. note::

   **What needs hardware confirmation in this chapter**

   Confirm the home path, gripper opening, object placement, workspace, reset wait, episode budget, and manual criteria on site.
   The CPU example only proves that success and failure fields can be distinguished. A new classifier needs independent validation; this chapter neither trains nor deploys one.

Checklist: the configuration comes from the same session; both machines have the data and reference files; success/failure can be distinguished in the dataset's meta.json;
know when reset, hold, and truncation each occur.

Next: :doc:`rl-train`.
