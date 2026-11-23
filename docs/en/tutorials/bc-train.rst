Train your first BC policy
================================================================================

This chapter passes the demonstrations selected in :doc:`bc-data` to ``real_world_bc`` to produce a loadable checkpoint.
Everything runs on CPU, without starting a daemon or connecting to an arm or teleoperation device.
The training environment provides only observation / action spaces. This entry point has neither separate actor / learner processes nor online evaluation episodes.

1. Write the training configuration
--------------------------------------------------------------------------------

Read the minimal configuration first, then generate your own version using the recording session's observation and action settings.
The template is at ``docs/examples/bc-train.yaml``:

.. literalinclude:: ../../examples/bc-train.yaml
   :language: yaml
   :start-after: # start config
   :end-before: # end config

.. list-table::
   :header-rows: 1
   :widths: 36 64

   * - Fields to change
     - How to obtain the value
   * - ``env_backend``, ``env_id``, ``obs``, ``real_robot``
     - Copy these blocks in full from the session resolved-config located in the previous chapter. The preparation script below does this for you.
       In particular, preserve robot / remote, task_frame, workspace, control, preprocess and action semantics.
       The template remote is only an address needed to construct the spaces offline; no service needs to run there.
   * - ``obs.rgb``, ``obs.image_size``, ``real_robot.state_fields``
     - Compare with the previous chapter's checking output and contract. The example rgb_wrist shape is [1,224,224,3],
       corresponding to rgb: [wrist] and image_size: [224,224]. state has 27 dimensions, corresponding to the template's four state_fields.
   * - ``buffer_size``
     - Use the previous chapter's transitions count. The preparation script sets max(64, row count), so insufficient capacity does not retain only the last demonstrations.
       For multiple dataset directories, allow for all rows and estimate the images' CPU memory use (one 224×224 RGB obs plus next_obs row is about 0.3 MB).
   * - ``log_dir``, ``exp_name``
     - This example writes to local/bc-runs/first-bc. Use a new name for a new experiment to avoid overwriting checkpoints or mixing logs.
   * - ``num_offline_steps``, ``batch_size``
     - This example makes four updates with four rows each, solely to verify the workflow. For real training, adjust to the dataset size and reserve complete episodes as a checking set.

Keep the template's other training fields: device and buffer_device are cpu; log_type is tensorboard,
log_freq is 1, checkpoint_freq is 2, save_final_checkpoint is true, and seed is 1.
Reduce logging and saving frequency for longer training. Unlisted model settings retain the code defaults:
actor_lr=0.0003, backbone_type=mlp, encoder.backbone=plain_conv,
encoder.features_dim=256, tanh_squash=true and offline_sampling=with_replace.
Change these only when comparing models or optimization settings. This chapter requires no pretrained weight downloads.
The one exception is that the template explicitly sets ``encoder.plain_conv_pooling: gap``: plain_conv's default flatten pooling only fits 128×128 and 64×64 images,
and 224×224 input fails with a matrix shape mismatch on the first forward pass; gap pooling fits any size.

The code defaults differ from this small exercise: buffer_size=1000000, buffer_device=cuda,
num_offline_steps=100000, log_freq=1000 and log_type=tensorboard.
Capacity must cover all selected demonstration transitions and fit GPU memory; also estimate host memory when switching to CPU.
num_offline_steps counts gradient steps, not episodes. This chapter explicitly overrides these values to verify the workflow first.

For real data, replace uuid in the command with the actual directory name printed in the previous chapter:

.. tutorial-test: skip
.. literalinclude:: ../../examples/bc-train.sh
   :language: bash
   :start-after: # start prepare
   :end-before: # end prepare

For the synthetic-data exercise, use this instead. Choose only one of the two:

.. tutorial-test: skip
.. literalinclude:: ../../examples/bc-train.sh
   :language: bash
   :start-after: # start synthetic-prepare
   :end-before: # end synthetic-prepare

**What you should see**: The configuration path, input dataset path, row count and buffer_size.
The synthetic example's actual run has 8 rows and buffer_size 64; the output file is ``local/bc-train.yaml``.
The script copies only env_backend, env_id, obs and real_robot from the session; training parameters come from the template.
The trainer restores the teleop source conventions from the dataset header's action_contract without opening devices.

**If it doesn't**:

* ``FileExistsError``: The preparation script does not overwrite an existing configuration. Inspect the old file first or change the output name.
* ``empty demonstration``: Return to the previous chapter and confirm that the demonstration contains rows.
* The session points to another task: Select the directory matching the dataset. Do not edit the action_contract in ``dataset.json`` to bypass checks.

2. Start training
--------------------------------------------------------------------------------

Load this configuration with the repository's training entry point, reading demonstration paths from the YAML ``demo_dataset_paths`` list:

.. literalinclude:: ../../examples/bc-train.sh
   :language: bash
   :start-after: # start train
   :end-before: # end train

``rwrl train`` forwards to this repository's training registry. ``examples/train_real_world.py``
calls the same entry point; it is not another BC implementation. The input is the dataset directory ``local/first-demo``: training reads the human-controlled steps of its kept episodes directly,
checks the dataset header's action_contract automatically, and rebuilds the image history to this run's ``obs.frame_stack`` (old pkls still load). The preparation script writes demonstration paths into the YAML ``demo_dataset_paths`` list.
You can still use the ``--demo-dataset-paths`` CLI parameter for a temporary override.

The rlg environment used for this verification lacked the rwrl console script and backend entry point.
Setting only PYTHONPATH and launching directly still produced ``Unknown env backend 'real_robot'`` during preflight.
The following imports this repository's backend first, then calls the same training entry point. This is the command actually used for this chapter's synthetic data:

.. tutorial-test: skip
.. literalinclude:: ../../examples/bc-train.sh
   :language: bash
   :start-after: # start synthetic-train
   :end-before: # end synthetic-train

An environment with the full package installed can use the rwrl command above. Both commands read demonstration paths from local/bc-train.yaml.
Use the configuration generated by synthetic-prepare for the synthetic exercise, or by prepare for real data.
To switch input temporarily, append ``--demo-dataset-paths local/first-demo`` to the training command.
These commands are alternatives, not successive repeated steps. Run only once in a given experiment directory.

**What you should see**: The process exits normally and generates the files listed in the next section. The current BC loop writes values to
TensorBoard without printing training progress at each step. A quiet terminal does not tell you whether updates occurred.

**If it doesn't**:

* ``No demo transitions or restored replay available``: Check for a typo in the CLI path and whether the glob matches files.
* ``recorded action contract does not match``: Compare the fields listed after differs in with the session configuration.
  If observations, action modes, scales or processing differ, select the matching data.
* ``Unknown env backend 'real_robot'`` or rwrl not found: Use the explicit registration above in a source environment.
* Out of memory: First reduce the selected data / buffer capacity and check that all rows still fit. Do not silently drop cameras or change image sizes.

3. Understand supervision and logs
--------------------------------------------------------------------------------

Confirm in the logs that updates occurred, then explain the target the model is fitting.

.. tutorial-test: skip
.. literalinclude:: ../../examples/bc-train.sh
   :language: bash
   :start-after: # start logs
   :end-before: # end logs

**What you should see**: The following scalars were actually read after four CPU training steps on eight synthetic rows:

.. literalinclude:: ../../examples/bc-train-output.txt
   :language: text
   :start-after: # start metrics
   :end-before: # end metrics

Logs are at ``local/bc-runs/pick_and_place/first-bc/events.out.tfevents.*``.
pick_and_place comes from env_id (no separate log_group was set).
The checkpoint and configuration directory has no such grouping level. Its actual contents are:

.. literalinclude:: ../../examples/bc-train-output.txt
   :language: text
   :start-after: # start files
   :end-before: # end files

``config.json`` saves the parsed training configuration; ``experiment.json`` saves the experiment contract and effective control metadata.
Checkpoints save policy weights, optimizer state and other training state. With the default save_replay_buffer=false,
they do not include the original demonstrations or replay files. When moving them, preserve the relative locations of config.json, experiment.json and checkpoints.

Supervision occurs in ``real_world_rl/algorithms/bc.py``, in ``MaskedBC._compute_losses``:
the model receives obs from row t, and the target is action from that same row—the action actually passed to the environment after human takeover.
It is not the next step's action corresponding to next_obs, the pre-intervention policy_action, or measured TCP displacement.
next_obs, reward and done enter the buffer but not this BC loss.

The default with_replace sampler samples single-step transitions from the buffer with replacement. Four steps means four gradient updates,
not four epochs. The Gaussian policy computes a per-dimension negative log likelihood for the target action. With the default tanh_squash, it first maps actions
back to Gaussian space and includes the Jacobian correction. Targets on the boundary are first clipped to ±(1−1e-6).

``human_action_mask`` is preserved from demonstrations through demo_loading and DemoReplayBuffer.
``masked_sum_mean`` sums only human-controlled dimensions: each row is first scaled by total action dimensions / human-controlled dimensions,
then averaged over rows with human-controlled dimensions. In the seven-dimensional example, 1111110 supervises only the first six entries, with a 7/6 scale;
1111111 supervises all seven. All-zero rows are excluded from the average. An all-zero batch skips the optimizer update and returns zero metrics.
The loader treats old data with neither mask nor intervened as fully human demonstrations. This chapter requires explicit masks to be retained.

The logged ``losses/behavior_mse`` is the mean squared error between deterministic actions and targets on human-controlled dimensions.
It averages over the total number of human-controlled dimensions, unlike NLL's per-row scaling.
Here, training MSE falls from about 0.214 to 0.068, showing that parameter updates can fit these synthetic samples.
The initial NLL is large. Do not interpret it as position error; inverse tanh at boundary actions and the policy standard deviation both affect its value.
Four updates cannot establish convergence, and random images cannot establish task success. The next chapter checks a specific output row.

**If it doesn't**:

* No scalars: Confirm that the process has exited and log_type is tensorboard, then read the directory grouped by env_id.
* Zero loss and MSE at every step: Check for all-zero masks. Do not immediately conclude that the fit is perfect.
* NaN / Inf in loss or predictions: Check finite data values and action scales first. For numerical problems at boundary actions, separately try
  tanh_squash=false, retrain and record the configuration. Do not reuse the old checkpoint.
* Training error falls but independent episodes perform poorly: Split training and checking data by episode, inspect demonstration coverage, and add representative demonstrations.

4. Alternative: train ACT on the same demonstrations
--------------------------------------------------------------------------------

This step replaces BC training. It is still offline and does not connect to the robot. ACT predicts a chunk of actions at once.
The following keeps the previous chapter's rgb_wrist visual input and saves to a separate first-act directory to avoid overwriting BC.
rl-garden's ACT trainer and ``deploy-act`` still read a pkl plus its contract sidecar, not a dataset directory,
so export one from the dataset first (the export reads all training rows into memory):

.. tutorial-test: skip
.. literalinclude:: ../../examples/bc-train-act.sh
   :language: bash
   :start-after: # start export
   :end-before: # end export

Then train:

.. literalinclude:: ../../examples/bc-train-act.sh
   :language: bash
   :start-after: # start train
   :end-before: # end train

state and action are normalized using the training data's mean / standard deviation, and the statistics are saved in the checkpoint.
Each rgb_<camera> key, sorted by name, represents a camera. RGB is resized to image_size (default 224),
then normalized using the ImageNet mean / standard deviation. For a state-only experiment, add ``--no-use-cameras``,
but preserve that model's input choice afterward; do not claim that it used images.

``--backbone-weights`` defaults to imagenet and downloads torchvision's IMAGENET1K_V1.
This example uses none to start from random weights. An offline machine can also supply a local torchvision state-dict path.
``--num-queries 30`` means 30 steps per chunk: 3 seconds at this tutorial's 10 Hz, not 0.6 seconds at the older example's 50 Hz.
Chunks do not cross done, truncated or demonstration file boundaries. By default, episodes also split where next_obs and the next row's obs are discontinuous.
``--save-freq 10000`` saves periodic weights separately. Training does not run robot evaluations.

**What you should see**: After normal completion, local/bc-runs/first-act/checkpoints/final.pt exists,
and periodic files are named <iter>.pt. This is not the BC checkpoint format. See step 4 of :doc:`bc-check` for convention checks
and step 6 of :doc:`deploy` for standalone deployment.

**If it doesn't**: Check the exported pkl path and sidecar, state and RGB keys, and full episode boundaries first. If torchvision is missing, install
ACT's required learning dependencies. The rlg environment used here lacked torchvision when loading the training module, so ACT training was not run.
The parameters and preprocessing above were checked against the current source. Expected artifacts are not evidence that training passed verification.

Artifacts from this chapter
--------------------------------------------------------------------------------

* **Used in the next chapter and the deployment chapter:** ``local/bc-runs/first-bc/checkpoints/final.pt``.
* Periodic checkpoints: ``local/bc-runs/first-bc/checkpoints/checkpoint_2.pt`` and ``checkpoint_4.pt``.
  The general naming pattern is ``<log_dir>/<exp_name>/checkpoints/checkpoint_<update step>.pt``, with final.pt as the final file.
* ``local/bc-runs/first-bc/config.json``, ``experiment.json``, and TensorBoard logs in the grouped directory.
  This example does not set checkpoint_dir. Setting it manually changes the save directory, so the layout above no longer applies directly.

.. note::

   **What needs real-hardware confirmation in this chapter**

   Training, log reading and saving were verified with synthetic data on CPU. Whether demonstrations succeeded, cameras see the task,
   and action conventions correspond to on-site motion still depends on inspecting real-hardware recordings. Falling loss cannot replace these checks.

Checklist: The configuration matches the data contract; you know how the mask enters the loss; logs contain four steps with finite values;
you can locate final.pt and its accompanying configuration and experiment record.

Next, use a saved observation in :doc:`bc-check` to explain one checkpoint prediction.
