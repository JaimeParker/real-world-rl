Teleoperate with a SpaceMouse
================================================================================

This chapter uses ``local/station.yaml`` and ``local/task.yaml`` from :doc:`station`.
Run commands from the repository root on the recording machine with the SpaceMouse attached. Read input before moving the arm.
Keep the ``base`` workspace from :doc:`station`. SpaceMouse directions follow the table frame from step 11 of :doc:`station`.

.. tip::

   Commands in this chapter omit ``--station`` and ``--preset``, using the two files remembered by ``rwrl use`` in step 10 of :doc:`station`.
   If you have not done that step, run ``rwrl use preset local/task.yaml`` first. To temporarily use another file, pass the option explicitly.

1. Find the device and add the teleop block
--------------------------------------------------------------------------------

Confirm that the USB node is accessible, then tell the station which input backend to use.
``teleop`` is the station configuration block describing the teleoperation device.

.. tutorial-test: skip
.. literalinclude:: ../../examples/spacemouse.sh
   :language: bash
   :start-after: # start hid
   :end-before: # end hid

Add the following block at the top level of ``local/station.yaml``. Replace any existing teleop block.

.. literalinclude:: ../../examples/spacemouse-config.txt
   :language: yaml
   :start-after: # start teleop
   :end-before: # end teleop

.. list-table::
   :header-rows: 1

   * - Field to change
     - How to obtain the value
   * - ``teleop.backend``
     - Use ``spacemouse`` in this chapter.
   * - ``options.device_index``
     - Keep 0 if there is only one supported device. For several devices, unplug and reconnect them individually, then use the next step's readings to confirm which device each index selects.

Keep the other fields at the example defaults: ``name``, ``deadzone: 0.001``, ``invert_axes: []``,
``poll_interval_s: 0.005``. Adjust deadzone only if idle noise actually affects input;
calibrate direction with the profile below first. Do not also change invert_axes arbitrarily. Measure input timing before changing the polling interval.

**What you should see**: the HID list contains the device's vendor/product ID, and node permissions allow the current user to read and write.
The repository's udev rules set ``MODE="0664"`` and ``GROUP="plugdev"`` on hidraw nodes for ``256f`` and ``046d``.

**If it doesn't**:

* ``franka.sdk:teleop:spacemouse`` fails or the list is empty: check USB and the ``spacemouse`` extra. The library used is ``hidapi``, imported as ``hid``.
* Doctor's ``franka.os:hid`` / ``franka.os:teleop:spacemouse`` fails: compare the node group with ``id``. Review permission-repair suggestions using the installation chapter's doctor workflow, then log in again, replug and recheck.
* Unknown model: check the HID decoding table. An arbitrary ``/dev/hidraw*`` may be another device; broader permissions do not add decoding support.

For an unknown model, collect raw evidence first without starting robot control:

.. literalinclude:: ../../examples/spacemouse.sh
   :language: bash
   :start-after: # start probe
   :end-before: # end probe

If several devices are candidates, use ``--device VID:PID`` with the hexadecimal ID from the HID list.
During sampling, push the puck along each axis and press the buttons. Review the report, then merge the device entry from candidate.yaml into
the devices mapping in ``local/spacemouse-devices.yaml``. Continue with the next step's reading checks.

**What you should see**: reports.jsonl and candidate.yaml in the report directory, plus descriptor.hex if the descriptor was read.
The candidate is only a draft decoder. Check axis layout, signs, buttons and TODOs before merging; probe does not install it automatically.

**If it doesn't**: if no reports appear, sample again while moving the puck. If the descriptor cannot be read, keep the reports you have;
missing evidence does not mean the model is verified. Fix path or permission errors before sampling again; do not try unrelated hidraw devices arbitrarily.

2. Read six axes and buttons
--------------------------------------------------------------------------------

Push, pull and twist the puck one direction at a time, and press both buttons. Confirm which axis and button the software reads.
These two commands only read input; they do not start the robot controller.

.. literalinclude:: ../../examples/spacemouse.sh
   :language: bash
   :start-after: # start read
   :end-before: # end read

**What you should see**: each ``teleop read`` row contains six ``twist`` values, ``gripper``,
``intervening`` and ``events``. Short presses produce ``button_0`` and ``button_1`` respectively.
The six axes are ordered x, y, z, rx, ry, rz. The device layer maps decoded values to
``[-y, x, z, -roll, -pitch, -yaw]``, then applies invert_axes.
Here, gripper and intervening are device-layer readings. The take-over exercise uses the tracker rules described in the next section.

``diagnose`` reports ``report_directory``. Its ``samples.jsonl`` saves each sample's
``decoded``, ``mapped``, ``task_input``, ``base_input``, ``buttons`` and ``fresh_report``.
Use it to check axis signs and button levels. Cached polling readings do not count as fresh HID reports.

The device layer reads at most 64 queued HID reports per poll, keeping the newest axis values read during that poll
and latching button-press events. Reports beyond that cap wait for later polls, so input is not guaranteed never to lag.
In session input diagnostics, ``hid_reports_this_poll`` is the number read by the device layer in this poll. Check for backlog if it keeps reaching the cap.
It is usually 0–2 while pushing the puck; persistently large values mean polling is too slow.
``teleop diagnose`` reads reports individually for samples.jsonl, recording ``report_count`` and ``fresh_report`` without this field.

**If it doesn't**:

* A single-axis action activates other axes: reduce coupling from your hand and retry each axis, then inspect raw decoded and mapped values.
* Idle values do not return to zero: check whether the puck is under force. Confirm noise in the diagnostic report before adjusting thresholds.
* ``exclusive input diagnostic blocked``: stop the recording / diagnostic program holding the input, then rerun.

3. Create an orientation profile and check frames
--------------------------------------------------------------------------------

Confirm that step 11 of :doc:`station` is complete. If the arm is not upright, the station needs a measured ``task_frame``
(for example ``name: table``). SpaceMouse readings are first interpreted in table directions (forward = table +x, left = +y, up = vertical +z),
then rotated into the robot base frame by ``task_frame``'s ``base_R_table``, and only then turned into end-effector targets for the controller.
The orientation profile adds only a yaw on top of this: the direction the SpaceMouse faces on the table.

An orientation profile saves the rotation between the device's current placement and task directions, so you do not have to guess directions by feel each time.
For an upright robot without ``task_frame``, use the following command (``--frame base``):

.. literalinclude:: ../../examples/spacemouse.sh
   :language: bash
   :start-after: # start calibrate
   :end-before: # end calibrate

Follow the prompts: leave the puck still, then move in each specified positive and negative direction. Each group samples for 3 s by default.
Prompts describe directions in operator terms, with data labels in brackets, for example
``roll + (about forward X: tilt the cap so its right side goes down) [+rx]`` :
roll / pitch / yaw rotate about forward X, left Y and upward Z respectively (right-hand rule).
``+ry`` tilts the puck forward with its front edge down; ``+rz`` twists it counterclockwise when viewed from above.
``--frame base`` interprets direction labels in the robot base frame. ``--profile operator-base`` is a profile name you choose.
``--output`` is relative to the station directory, so it writes ``local/spacemouse-profiles.yaml``.
After confirmation, ``--use`` saves the profile and writes the station's ``teleop_profiles`` reference.

**What you should see**: results include ``candidate``, ``frame``, ``yaw_rad``,
``horizontal_validated``, ``rotation_validated`` and the save path.
After sampling both signs of all six axes, check that both validated fields are true.
``profiles show`` should select operator-base. ``workspace show`` should still display the base boundaries reviewed in the previous chapter.
Sampling only the four horizontal directions can fit yaw, but does not prove that rotation directions were verified.

For a robot with ``task_frame`` (such as a 45° side mounting), calibrate in the table frame. Set ``--frame`` to the ``name`` of ``task_frame``:

.. literalinclude:: ../../examples/spacemouse.sh
   :language: bash
   :start-after: # start calibrate-table
   :end-before: # end calibrate-table

Prompt directions now refer to the table (forward, left and up are all table directions). Profile yaw combines with ``base_R_table`` into the final rotation.
Establish ``task_frame`` before calibrating the profile. If you change ``task_frame`` later, recalibrate the profile. Orientation profiles do not change workspace geometry.

**If it doesn't**:

* ``calibration frame must match station task_frame``: the frame name must be base or the configured task_frame.name.
* ``direction error``: check whether you moved in the opposite direction and resample it. Do not treat a failed result as calibrated.
* ``connected SpaceMouse does not match profile``: the device identity differs from the profile. Select the matching profile or recalibrate.

Reproducible offline checks
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Without an input device, use fake readings to check the output format, then use direction samples from tests to verify the fitting entry point.

.. literalinclude:: ../../examples/spacemouse-offline.sh
   :language: bash
   :start-after: # start read
   :end-before: # end read

**What you should see**: the following is actual mock output; timing varies slightly. It does not verify HID decoding.

.. literalinclude:: ../../examples/spacemouse-mock-output.txt
   :language: text
   :start-after: # start output
   :end-before: # end output

.. tutorial-test: skip
.. literalinclude:: ../../examples/spacemouse-offline.sh
   :language: bash
   :lines: 5-12

.. literalinclude:: ../../examples/spacemouse-offline.sh
   :language: bash
   :lines: 13-13

**What you should see**: the actual fit result below. Only horizontal samples were supplied, so rotation_validated is false;
it was not saved and must not be treated as your device profile.

.. literalinclude:: ../../examples/spacemouse-calibration-output.txt
   :language: json
   :start-after: # start output
   :end-before: # end output

**If it doesn't**: return to the repository root if the test file is not found. Install the test extra if pytest is missing.

To recheck a direction, add ``--samples PATH`` to calibrate to read existing JSON samples,
and use ``--capture-label +x`` to resample only that direction. It replaces the group by default; add ``--append-trial`` to keep old trials.
The merged-samples.json path is printed after capture. Use it as the samples input next time.
If any of the four horizontal directions is missing, output says NEEDS_SAMPLES. Complete them before fitting; one direction is not a full calibration.

Saving and selecting are separate steps: after confirmation, ``--save`` only saves the profile; ``--use`` also writes the station reference.
You can list and switch profiles already saved in the file referenced by the station without sampling again:

.. literalinclude:: ../../examples/spacemouse.sh
   :language: bash
   :start-after: # start profiles-select
   :end-before: # end profiles-select

**What you should see**: list shows saved profiles, use updates the default selection after confirmation, and show displays that profile.

**If it doesn't**: if the profile is not found, check the station's teleop_profiles.file and the profile name;
using only --save to write another file does not automatically make the station reference it.

4. Practice take-over, gripper control and hand-back with the default tracker
--------------------------------------------------------------------------------

Run a short session first to confirm that hand input, measured motion and control transitions agree.
``tracker`` maintains an end-effector target and limits its speed and lead over the measured pose.
Default target speeds are 0.05 m/s and 0.3 rad/s; lead caps are 0.05 m and 0.3 rad.

.. warning::

   The following starts the arm. The first reset homes it and opens the gripper. Clear the homing path and check home,
   workspace and the stopping method first. Keep hands away from the gripper and maintain on-site supervision.

.. literalinclude:: ../../examples/spacemouse.sh
   :language: bash
   :start-after: # start practice
   :end-before: # end practice

After confirming the run, release the puck and both buttons and wait for the neutral check to pass. Press button_0 to take over,
push gently in one direction, then release the puck. The target stops advancing, but the robot may finish moving through the remaining lead distance.
While controlling, press button_1 to toggle the gripper open / closed. Press button_0 again to hand back.
After hand-back, the recorder holds the current pose. Terminal ``t`` / ``h`` do not control this mode.
Finally press ``s`` to mark the practice successful, then ``q`` during the hold phase to quit.

**What you should see**: pressing button_0 to take over prints ``INTERVENTION ON: the operator drives the arm``;
about 0.3 s after hand-back it prints ``INTERVENTION OFF: the operator handed back``. Pressing ``s`` or ``f`` prints
``EPISODE MARK: success`` or ``failure``. Puck motion moves the robot only after button_0 is pressed. Take-over retains the last gripper command;
it does not open automatically. Releasing the puck is not hand-back. EPISODE_HOLD appears after the episode ends; see :doc:`record`.

Your wrist may tilt the puck while pushing it. Tracker mode defaults to ``axis_lock: dominant``: each reading retains only
the translation or rotation group with the larger peak and zeros the other, so a pure x push does not introduce pitch.
For simultaneous translation and rotation, set ``axis_lock: "off"`` in the station's ``teleop.options`` (serl mode rejects this option).

.. note::

   Which controller executes these targets depends on the station's ``cartesian_controller`` (``impedance`` by default, ``internal`` or ``serl``),
   independently of tracker / serl input mode here. Any combination is possible; see :doc:`controller`.
   The controller receives base-frame targets and does not rotate them again. Switching controllers changes one station field; the orientation profile and preset need no changes.

**Single-axis check.** Push in just one direction (or twist about one axis) at a time. Check that the TCP moves in the same **table**\ direction (or rotates about the same axis).
For translation, read with ``workspace capture``: once before the session, then push only one direction about 3 cm, hand back, press ``s`` and ``q`` to exit, and read again:

.. literalinclude:: ../../examples/spacemouse.sh
   :language: bash
   :start-after: # start axis-check
   :end-before: # end axis-check

.. literalinclude:: ../../examples/en/spacemouse-axis-output.txt
   :language: text
   :start-after: # start capture
   :end-before: # end capture

``task_position`` is the TCP position in the table frame (base frame without ``task_frame``). After pushing only “up”, only z should increase noticeably:

.. literalinclude:: ../../examples/en/spacemouse-axis-output.txt
   :language: text
   :start-after: # start check
   :end-before: # end check

Repeat for forward, left and up. Inspect the three rotations visually: twist about “forward” and the gripper should rotate about the table's forward axis, and likewise for the others.
If pushing “up” makes the TCP move diagonally, ``task_frame`` is missing or its angle is wrong. Return to step 11 of :doc:`station`.

To restrict diagnostic input, add ``--diagnose translation``,
``rotation`` or ``all`` to the ``teleop run`` above. This session moves the robot and requires an already reviewed workspace in the station.
Here ``--gain`` defaults to 0.2, range (0, 1]; ``--duration-s`` defaults to 5 s, range (0, 300],
and is converted to an episode step cap. Unless explicitly specified, both the success quota and episode budget are 1. The episode still enters hold when it ends.
Without ``--diagnose``, gain and duration-s have no effect. Input-filter settings are written into the control contract.

Tracker lead caps cannot exceed the preset's per-step action scales. Otherwise session startup reports
``needs xyz_scale_m >= lead_cap_m and rot_scale_rad >= lead_cap_rad``。
For example, if you follow SERL conventions and set ``real_robot.xyz_scale_m`` to 0.02 and ``rot_scale_rad`` to 0.1,
also set ``teleop_lead_cap_m: 0.02`` at the preset's top level (the default angular cap of 0.3 rad exceeds 0.1, so also set ``teleop_lead_cap_rad: 0.1``).

**If it doesn't**:

* ``START_NEUTRAL`` keeps waiting: release the puck and buttons, then check idle noise and the profile's neutral_threshold (default 0.05).
* ``WORKSPACE HOLD: <axes>``: the request is out of bounds. Recheck workspace and safety_box, hand back and take over again. Do not enlarge boundaries to bypass the check.
* Direction or magnitude differs: exit the session, recheck the orientation profile, task frame and preset action scales, then repeat single-axis practice.

5. Decide whether to use SERL mode
--------------------------------------------------------------------------------

Start a separate SERL session and recheck the action convention only if you want immediate take-over through puck motion.
First remove the station's ``teleop.options.deadzone`` and any explicitly set ``axis_lock`` (the device layer still uses its default deadzone);
the SERL source rejects tracker-only options. Then copy the preset. SERL requires both XYZ and RPY bounds at the environment layer;
the previous chapter's task template lacks these fields.

.. tutorial-test: skip
.. literalinclude:: ../../examples/spacemouse.sh
   :language: bash
   :lines: 45-45

.. literalinclude:: ../../examples/spacemouse.sh
   :language: bash
   :lines: 46-47

.. tutorial-test: skip
.. literalinclude:: ../../examples/spacemouse.sh
   :language: bash
   :lines: 48-49

Read the TCP at a pose independently confirmed safe, and use the same conversion program to convert xyzw to radians.
If manual guiding is needed, follow the station chapter's stop-control and on-site supervision procedure. Reading here does not itself move the robot.
Merge the following snippet into the new file's real_robot block and replace every placeholder with reviewed values:

.. literalinclude:: ../../examples/spacemouse-config.txt
   :language: yaml
   :start-after: # start serl
   :end-before: # end serl

.. list-table::
   :header-rows: 1

   * - Field to change
     - How to obtain the value
   * - ``real_robot.safety_box``
     - Copy the reviewed base_safety_box from workspace show, preserving the order xmin, ymin, zmin, xmax, ymax, zmax.
   * - ``safety_box_rpy_low`` / ``safety_box_rpy_high``
     - Derive lower and upper bounds from this step's converted TCP readings and allowed orientations. Use absolute roll; keep pitch / yaw signs. Units are rad.

Keep the other defaults: gripper_action_scale is 1 and gripper_sleep_s is 0.6 s;
adjust only after checking gripper feedback and actual opening/closing intervals. SERL's roll limit preserves sign and clips magnitude.
For example, the station chapter's quaternion ``[1, 0, 0, 0]`` converts to ``[pi, 0, 0]``.
This is a single orientation, not evidence for an allowed range. If crossing an Euler branch or singularity, first choose a representable orientation interval.


.. warning::

   In SERL, puck motion itself takes over, and both buttons change meaning. Leave the puck still and confirm site clearance before starting.

.. literalinclude:: ../../examples/spacemouse.sh
   :language: bash
   :start-after: # start serl
   :end-before: # end serl

This command explicitly passes ``--preset local/task-serl.yaml``, so this run uses it rather than the remembered ``local/task.yaml``;
stderr also no longer prints ``using preset``.

**What you should see**: motion input above 0.001 or either held gripper button takes over;
releasing the puck and buttons hands back. Holding button_0 closes, holding button_1 opens; close wins when both are held.
It outputs raw normalized input without the tracker's target-speed or lead limits.
Direction: by default, serl mode rotates input into the base frame using only the station ``task_frame`` rotation ``base_R_table``. It **does not add**\ the profile yaw from step 3.
Place the SpaceMouse \ **squarely**\ as in HIL-SERL: puck “forward” faces table +x (away from the robot base), “left” faces table +y, and “up” is vertical.
Viewed from above:

.. code-block:: text

                 Table +x (forward, away from base)
                        ▲
                        │
          +y (left) ◄── [SpaceMouse]       Cable side faces the operator
                        │
                 ┌──────┴──────┐
                 │ Franka base │          +z: vertically up (out of the page)
                 └─────────────┘

For an upright robot (without ``task_frame``), the table frame is the base frame. This is exactly HIL-SERL's approach: it assumes upright mounting and uses the raw base frame directly.
For a side-mounted robot with ``task_frame``, pushing “up” still moves vertically upward, not along the tilted base z axis.
Once aligned, SERL needs no yaw correction, but current ``teleop run`` still requires the station to reference an orientation profile
for device identity, neutral threshold and other information. Keep the profile from step 3. Perform these six single-axis checks, moving only one axis at a time and checking that the TCP moves along (or about) the same table-frame axis:

.. list-table::
   :header-rows: 1

   * - Action
     - Expected TCP motion
   * - Push the puck forward
     - Move along table +x (away from the base)
   * - Push the puck left
     - Move along table +y
   * - Lift the puck upward
     - Move vertically upward
   * - Rotate the puck about the forward–back axis (right side down)
     - Rotate about table x, with the gripper's right side down
   * - Tilt the puck forward (front edge down)
     - Rotate about table y, with the gripper's front end down
   * - Twist the puck counterclockwise from above
     - Rotate counterclockwise from above

Confirm translation direction with ``workspace capture``'s ``task_position`` readings (as in step 4: read before and after each session, pushing only one direction per session):

.. literalinclude:: ../../examples/spacemouse.sh
   :language: bash
   :start-after: # start serl-axis-check
   :end-before: # end serl-axis-check

If that placement is impossible (for example the operator can only stand to the side), use step 3's profile. Set
``use_profile_rotation: true`` in the station's ``teleop.options`` so serl mode uses the full profile rotation (``base_R_table`` × profile yaw).
It accepts only true / false. Tracker mode always uses the profile (or ``task_frame`` without a profile).
This choice is saved in the recording's teleop_control record. An RL run recorded by an older version that implicitly applied profile rotation
will be rejected on resume with the new version because the records differ, rather than silently changing direction.
The session also switches environment ``cartesian_semantics`` to ``serl``, adding RPY clipping and changing gripper execution conventions.
Do not merge data from the two modes for training under the same contract. Terminal t/h have no effect here either.

**If it doesn't**:

* ``serl requires complete safety_box`` or ``safety_box_rpy``: complete the new preset's environment-layer bounds. Position bounds only in the station are insufficient.
* You expected a short press to keep control: SERL uses current input and button levels. Switch back to tracker for latched take-over.
* Changing tracker speed has no effect: SERL does not use those options. Review environment scales and position / orientation bounds instead.
* Direction differs from tracker mode: serl mode does not add profile yaw by default. If the SpaceMouse is not aligned as shown above, directions rotate by an angle; this is expected. Align it or set ``use_profile_rotation: true``.
* Pushing “up” moves the TCP diagonally: ``task_frame`` is missing or inaccurate. Return to step 11 of :doc:`station`.

.. note::

   **What needs confirmation on hardware in this chapter**

   HID permissions, model decoding, six-axis directions, button indices, idle noise, task-frame transformation, workspace,
   reset path, speed and lead distance, gripper opening/closing, and motion after hand-back in tracker / SERL all need on-site confirmation.
   Offline tests do not replace these checks.

Checklist: six axes and buttons can be read; the orientation profile is saved and selected; single-axis motion matches expectations;
you can take over, open and close the gripper, hand back and exit.

GELLO users should continue to :doc:`gello`. Users of the ``serl`` controller should return to step 8 of :doc:`controller` to check automatic recovery after a reflex;
otherwise, go directly to :doc:`record` after completing SpaceMouse practice.
