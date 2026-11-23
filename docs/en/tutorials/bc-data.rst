Prepare BC training data
================================================================================

This chapter starts with the artifacts from :doc:`inspect`: the dataset directory ``local/first-demo``,
``resolved-config.yaml`` in the session directory, and
``local/station.yaml`` and ``local/task.yaml``. All work is offline, from the repository root,
in an environment with learning dependencies installed.

These chapters use ``real_world_bc`` because it is the BC entry point in this repository's training registry that reads demonstration dataset directories directly
and preserves per-dimension human supervision. The CLI requires an explicit algorithm name; there is no default that selects one automatically.
It builds ``MaskedBC``, which defaults to a Gaussian actor, an MLP backbone and a plain_conv image encoder.
Another path is rl-garden's ACT trajectory trainer, paired with this repository's deploy-act.
It predicts action chunks, uses a different checkpoint format, and still reads a pkl plus sidecar,
so export one from the dataset first with ``rwrl dataset export --format pkl`` (see the end of :doc:`bc-train`).
rl-garden also provides bc, diffusion_bc, flow_bc, mean_flow_bc, a2a_bc and consistency_distill_bc.
These are not part of this repository's real_world_bc entry point for loading single-step demonstrations. These chapters do not switch algorithms.

1. Select data and configuration from the same session
--------------------------------------------------------------------------------

Match the successful demonstrations to the episodes in the dataset first, so that data with different action conventions do not become one training set.

.. literalinclude:: ../../examples/bc-data.sh
   :language: bash
   :lines: 2-2

.. tutorial-test: skip
.. literalinclude:: ../../examples/bc-data.sh
   :language: bash
   :lines: 3-3

The first command is the repository's inspector, which reads the dataset directory directly; the second is the offline checking script provided for this chapter.
The script fetches the training rows with the same function training uses, reads obs, next_obs, action and human_action_mask from every row and checks shapes, finite values,
action ranges and masks. It compares the first row's obs with the observation keys, shapes and dtype recorded in the dataset header's ``action_contract``.
For the remaining obs / next_obs, it checks keys, shapes and finite values, but does not check dtype row by row.

**What you should see**: The episodes and human-controlled intervals in the dataset, and the transitions count of the training rows (after filtering kept episodes).
With the earlier configuration, observations should include ``state`` and ``rgb_wrist``, with seven-dimensional actions.
``human masks`` shows the number of rows for each combination of human-controlled dimensions.
The training rows exclude discarded episodes and, by default, alignment steps; see :doc:`inspect` for the filtering rules.
Start with one checked dataset. When merging several later, perform the same checks on each and compare their full contracts.
Do not use discarded episodes as successful demonstrations.

**If it doesn't**:

* Not a dataset or no training rows: Check that the directory has ``dataset.json`` and that the previous chapter retained successful human-controlled steps (``rwrl dataset list``).
* ``observation shape/dtype differs from contract``: Check for the wrong dataset or independently modified images / state.
* ``action/mask shape mismatch``: A seven-dimensional mask cannot accompany an eight-dimensional action. Return to the original dataset to check where it came from.
* All-zero masks or no valid actions: Check the dataset's human-controlled intervals first. All-zero rows provide no BC supervision.

Next, locate the final configuration used during recording and compare it with the current local files:

.. tutorial-test: skip
.. literalinclude:: ../../examples/bc-data.sh
   :language: bash
   :start-after: # start session
   :end-before: # end session

**What you should see**: One or more session configuration paths containing a uuid.
Use summary and provenance in the same directory to identify which session generated first-demo. Record the full path.
resolved-config is the merged station and task configuration; training uses it and the dataset header's action_contract as its basis.
The current station/task may have changed. Do not use them to overwrite the cameras, scales or coordinate conventions used during recording.
Keep these two local files to check device connections during deployment.

**If it doesn't**: If the session is missing, recover the recording directory first. If several candidates exist, check output in summary.
Do not pass multiple YAML files to training at once with a wildcard.

2. Explain one observation and action
--------------------------------------------------------------------------------

Check the state concatenation order and the physical meaning of actions before deciding which demonstrations can be used together.
The recording task selects ``[tcp_pose, tcp_vel, joint_pos, joint_vel]``, so state has 27 entries:
the first 7 are XYZ and an xyzw quaternion, followed by 6 TCP velocity entries, 7 joint positions and 7 joint velocities.
``rgb_wrist`` is the image from the camera named wrist, processed according to the task settings.
The contract's ``experiment.observation`` gives the final shapes, dtype, frame stacking and rotation conversions;
``experiment.preprocess`` gives processing such as cropping. Do not reprocess saved observations.

The SpaceMouse tracker example uses ``cartesian_delta`` with ``current`` semantics.
The action order is three position increments, a three-dimensional rotation vector, then the gripper.
Multiply the first three dimensions by ``xyz_scale_m=0.05`` and the rotation dimensions by ``rot_scale_rad=0.3``.
The rotation increment left-multiplies the current orientation. Also check task_frame for position and rotation coordinate conventions.
With current semantics, the gripper maps from [-1, 1] to [0, 1]. For the binary Hand configured earlier, +1 opens and -1 closes.
These are requests passed to the environment; limits and the controller further process the actual targets.

The default GELLO exercise also uses a Cartesian mapping and can produce seven-dimensional actions.
If you completed the joint-space extension in :doc:`gello`, inspect that separate recording:

.. tutorial-test: skip
.. literalinclude:: ../../examples/bc-data.sh
   :language: bash
   :start-after: # start gello
   :end-before: # end gello

**What you should see**: The joint-space extension has ``action_mode: joint_abs``, seven arm dimensions plus one gripper dimension,
and seven pairs of ``limits.joints``. A normalized action a corresponds to the joint target
``low + (a + 1) / 2 * (high - low)``, not directly to a value in radians.
With joint_delta instead, the target is the current joint angle plus ``a * joint_scale_rad``.

For example, to move the gripper right in space, SpaceMouse may record a Cartesian x increment,
while GELLO joint_abs records seven normalized joint targets. The same statement of operator intent does not determine the action's meaning.
The joint-space fake example in :doc:`inspect` uses ±2 rad limits, where a=0.2 corresponds to 0.4 rad.
In the Cartesian synthetic example below, a=0.2 corresponds to a 0.01 m x request. ±2 is a test value, not an on-site limit.

**If it doesn't**:

* GELLO still has seven dimensions: Check action_mode first. Cartesian GELLO has seven dimensions; do not force it to eight.
* Equal dimensions but different meanings: Check cartesian_semantics, task_frame, scales, limits, gripper conventions and
  teleop_control in experiment.sources. Train different conventions separately.
* An unknown state entry: Reconstruct the order from the recording's state_fields and rotation conversions. Do not reuse slices from another example.

3. Generate a small reproducible example without real-hardware data
--------------------------------------------------------------------------------

Generate one eight-step episode, written as a dataset in the same format as ``rwrl record``,
to verify the later training and loading workflow first. This uses the real environment class's space definitions, without ever calling reset or step.
Images are random uint8, states are synthetic values, and actions are fixed. This is not a robot simulation.

For source execution, set the import path first. This chapter's CPU verification disables CUDA:

.. literalinclude:: ../../examples/bc-data.sh
   :language: bash
   :start-after: # start source-env
   :end-before: # end source-env

Then run the following. The output directory must not already exist:

.. tutorial-test: skip
.. literalinclude:: ../../examples/bc-data.sh
   :language: bash
   :start-after: # start synthetic
   :end-before: # end synthetic

**What you should see**: The dataset ``demos/`` (with ``dataset.json`` and one episode) and resolved-config.yaml are generated.
The following is an excerpt of the checking script's actual output; the full contract is printed afterward:

.. literalinclude:: ../../examples/bc-data-output.txt
   :language: text
   :start-after: # start summary
   :end-before: # end summary

``[1, 224, 224, 3]`` means batch, H, W and RGB channels; ``[1, 27]`` means batch and state dimensions.
Four rows have mask 1111111 and the other four have 1111110. In the latter rows, the gripper receives no human supervision,
even though the last action entry is still +1.

**If it doesn't**:

* Directory already exists: Choose a new directory and update subsequent commands to avoid mixing runs.
* Missing torch or rl_garden: Return to :doc:`install` and use a learning environment. Keep the PYTHONPATH above when running from source.
* ``offline demos has no action-contract metadata`` during generation: This message appears while the synthetic spaces are being built;
  the action contract is written to ``dataset.json`` when the dataset is generated. If it appears when training on real data, the path you loaded is not a dataset directory; recover the dataset.

Artifacts from this chapter
--------------------------------------------------------------------------------

* Real-hardware data: The checked dataset ``local/first-demo``,
  plus the matching session resolved-config and station/task.
* Exercise without hardware: The dataset ``local/bc-synthetic/demos`` and its resolved-config.
  The next two chapters use these as alternative inputs, not as real-hardware demonstrations.

.. note::

   **What needs real-hardware confirmation in this chapter**

   The actual wrist view and colors, state feedback order, SpaceMouse directions and coordinate frame, GELLO calibration,
   action scales, gripper opening / closing and the correspondence between masks and operator control must be confirmed against real-hardware recordings.
   Synthetic data checks only the format and software workflow.

Checklist: Identify one observation row's keys and shapes, action mode and dimensions, and human-controlled dimensions;
locate the configuration and dataset from the same session, and explain whether the SpaceMouse and GELLO actions can be mixed.

Next, train your first checkpoint in :doc:`bc-train`.
