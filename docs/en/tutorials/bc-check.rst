Check a BC checkpoint offline
================================================================================

This chapter loads ``local/bc-runs/first-bc/checkpoints/final.pt`` from :doc:`bc-train`,
feeds it a saved observation, and compares the output with the demonstration action. Everything is offline, with no reset, step or device connection calls.
The goal is to obtain a loadable checkpoint and explain one input and one output.

1. Load the weights and run one prediction
--------------------------------------------------------------------------------

Use this chapter's script to restore the training configuration and policy, check the data conventions, and predict from the saved obs in row 0.
Load only checkpoints and pickle files you generated yourself or trust.

.. tutorial-test: skip
.. literalinclude:: ../../examples/bc-check.sh
   :language: bash
   :start-after: # start check
   :end-before: # end check

If you trained on synthetic data in the previous chapter, replace the prediction command with this one. The output record path stays the same:

.. tutorial-test: skip
.. literalinclude:: ../../examples/bc-check.sh
   :language: bash
   :start-after: # start synthetic
   :end-before: # end synthetic

The script requires config.json and experiment.json to remain in the run directory.
It calls ``restore_training_args`` and ``restore_policy`` from this repository's deployment module,
explicitly sets the algorithm to real_world_bc, and places both the model and data on CPU.
The environment uses the same offline space construction as training. The script also compares the demo contract with the training experiment record;
it does not enter a deployment episode. The core inference code is:

.. literalinclude:: ../../examples/bc-check.py
   :language: python
   :start-after: # start inference
   :end-before: # end inference

``policy.eval()`` is set before this code. deterministic=true selects a deterministic action:
with the default tanh Gaussian, this is the transformed mean, not a random sample.
Inputs use the saved tensors directly, retaining uint8 RGB and float32 state. Do not manually reorder channels or rescale images again.

**What you should see**: Observation shapes, demonstration action, predicted action, mask, masked MSE and the new record path.
This is the actual output from the previous chapter's four-step CPU checkpoint:

.. literalinclude:: ../../examples/bc-check-output.txt
   :language: text
   :start-after: # start inference
   :end-before: # end inference

**If it doesn't**:

* Missing config.json / experiment.json: Recover the training directory. Copying final.pt alone is insufficient for this chapter's convention checks.
* ``recorded action contract does not match`` or ``saved experiment contract/hash differs``:
  Check that the checkpoint, data and training configuration come from the same experiment. Do not delete metadata to bypass checks.
* Parameter size mismatch or observation key error: Confirm that the algorithm is real_world_bc and that you have not used ACT weights by mistake. Preserve the original camera and state layout.
* ``FileExistsError``: The script does not overwrite offline-check.json. Give record a new path when checking other rows.

2. Explain this input and output
--------------------------------------------------------------------------------

Interpret each printed value using the recording conventions, distinguishing numerical error from action meaning.

.. list-table:: Row 0 after four synthetic training steps
   :header-rows: 1
   :widths: 25 75

   * - Item
     - Explanation
   * - Input state
     - [1,27]: tcp_pose has 7 entries, tcp_vel 6, joint_pos 7 and joint_vel 7.
       This example does not convert quaternions or stack multiple frames.
   * - Input rgb_wrist
     - [1,224,224,3], uint8, RGB. The contents are synthetic random pixels, not a real scene.
   * - Demonstration action
     - [0.2,0,0,0,0,0,1]: under the current Cartesian convention, this requests an x increase of 0.01 m, no rotation and an open gripper.
   * - Predicted action
     - The x component is about -0.1588, corresponding to about -0.0079 m; y is about 0.0467, corresponding to about 0.0023 m.
       The model still outputs y, z and rotation requests absent from the demonstration.
   * - Predicted gripper
     - About 0.0207, corresponding to a normalized opening request of about 0.5104. With the binary Hand used for the earlier recording, this just exceeds the 0.5 opening threshold.
       The training loss compares continuous action values, not whether the binary device ultimately opens.
   * - Mask and error
     - All seven dimensions in row 0 are human-controlled; masked MSE is about 0.194124. This is error in normalized action space, not m or rad.

The final training-log MSE comes from a sampled batch before the update; this value comes from row 0 after the update. They need not match.
If you inspect a row with mask=1111110, the gripper still outputs a value, but it does not enter that row's supervised error.
This does not establish that the gripper dimension is well learned. Also check demonstrations with gripper supervision.

**What you should see**: You can identify the action mode, scale and human-control marker for any prediction component.
This example has only four updates and still has clear output errors. Loading and inference have been verified; task suitability has not.
For real data, inspect multiple observations from independent complete episodes. Do not claim success from a single selected training sample.

**If it doesn't**:

* You interpret the prediction as an absolute TCP target: Recheck action_mode. This example uses increments relative to the current state.
* Low error but an obviously wrong dimension: Check the mask first; that dimension may not have received supervision.
* You interpret GELLO joint_abs results in meters: Decode the seven joint targets with that recording's ``limits.joints``. Do not apply this table.

3. Save the record to check before deployment
--------------------------------------------------------------------------------

Inspect the script's offline-check.json so the deployment chapter can compare the current station with the model's requirements item by item.

.. tutorial-test: skip
.. literalinclude:: ../../examples/bc-check.sh
   :language: bash
   :start-after: # start record
   :end-before: # end record

**What you should see**: checkpoint, demo and row identify this check; observation_shapes,
observation_dtypes, cameras and state_fields record the input; action_conventions records the output conventions;
action_contract and training_experiment preserve the full source contracts.
demo_action, predicted_action, human_action_mask and masked_mse save the actual values from this comparison.
Here, cameras is wrist, state_fields contains the four fields in the table above, and in action_conventions,
cartesian_semantics is current, xyz_scale_m is 0.05, rot_scale_rad is 0.3 and control_hz is 10.

Before deployment, check at least these groups of items:

* Camera names, actual views, RGB / depth, image sizes, preprocess such as crop, frame stacking and rotation conversions.
  The same wrist name does not imply the same image distribution after moving the camera mount.
* state_fields order, field dimensions, the xyzw quaternion convention, task frame and units.
* action_mode, cartesian_semantics, action dimensions, scales, ``limits`` (joint ranges, TCP box), gripper mapping and control frequency.
  Action conventions also include spatial constraints and teleop_control provenance in experiment.
* Devices, home, limits (``limits``) and control settings in the current ``local/station.yaml``, and
  differences between ``local/task.yaml`` and the training configuration. Keep the original session resolved-config as the recording reference.

**If it doesn't**:

* The record differs from the expected data path: Confirm whether the previous chapter actually trained on first-demo or synthetic demos. Do not label a synthetic checkpoint as real-hardware training.
* Only final.pt remains, with no input record or conventions: Archive the entire run directory with the demonstration dataset before continuing to deployment.
* The camera, coordinate frame or action mode changed: Recheck / prepare data and the model first. Do not merely change deployment parameters to make dimensions match.

4. Check input and action conventions separately for ACT or a residual base policy
----------------------------------------------------------------------------------

The ACT provider checks state and action widths. This chapter's four state_fields total 27 dimensions,
and their order must also match the demonstrations. Visual inputs are sorted by rgb_<camera> name. Preserve the original obs.rgb,
camera mounting and preprocessing. Standalone deploy-act also compares the checkpoint's camera_keys with the environment's camera order.

The provider reconstructs the model from the checkpoint's act_config and selects ema_agent weights by default.
It normalizes state and denormalizes actions using the saved statistics, then clips actions to [-1,1].
A residual base specification can be ``act:local/bc-runs/first-act/checkpoints/final.pt?temporal_agg=1``.
Quote the entire specification so the shell does not treat ? as a wildcard or & as a background operator.

Join multiple options with &: temporal_agg=1 is the default temporal aggregation of overlapping chunks, with weights
computed as exp(-temporal_agg_k * i); temporal_agg=0 replays chunks open-loop.
temporal_agg_k defaults to 0.01 and state_dict_key to ema_agent. See :doc:`rl-intervene` for chunk resets after intervention.

**What you should see**: The training data, run configuration and model use the same state layout, camera order and action interpretation.
Passing width checks is only one part of this. ACT does not use this chapter's BC offline script.

**If it doesn't**: joint_abs and joint_delta can have the same width, so the provider's width check cannot detect mixing them.
Check action_mode, ``limits.joints`` and scales separately. A residual base policy must also output the action space required by the current run.
Matching widths or finite values after clipping do not establish matching action semantics.

Artifacts from this chapter
--------------------------------------------------------------------------------

* **Checkpoint for the deployment chapter:** ``local/bc-runs/first-bc/checkpoints/final.pt``.
* ``config.json``, ``experiment.json`` and this chapter's new ``offline-check.json`` in the same run directory.
* Original demonstrations and their contract, session resolved-config and station/task. Together, they explain one model input and output.
  The synthetic exercise artifacts verify only the offline workflow; they do not establish readiness to deploy on a real-hardware task.

.. note::

   **What needs real-hardware confirmation in this chapter**

   Checkpoint restoration, single-observation inference and numerical comparison were run offline. Camera views, feedback units,
   actual control cycles, the correspondence between actions and motion, workspace, stopping behavior and task success rate need real-hardware confirmation.
   This chapter sends no actions, so it cannot establish supervised deployment performance.

Checklist: Load final.pt; identify the source and shape of an input tensor;
decode an output component with the correct convention; save the configuration, contract and offline check record needed for deployment.

The BC training section is complete. Next, use this record in :doc:`deploy` to prepare supervised deployment and inference.
