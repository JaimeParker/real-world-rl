Teleoperate with GELLO
================================================================================

This chapter uses the single-arm Franka configuration from :doc:`station`. Run it on the recording machine with the GELLO serial port.
The leader is the GELLO you move by hand; the follower is the Franka doing the work.
This chapter's ``local/task.yaml`` outputs seven-dimensional Cartesian delta actions, not seven joint angles.

.. tip::

   Commands in this chapter omit ``--station`` and ``--preset``, using the two files remembered by ``rwrl use`` in step 10 of :doc:`station`.
   If you have not done that step, run ``rwrl use preset local/task.yaml`` first. To temporarily use another file, pass the option explicitly.

1. Find the serial port, joint IDs and event device
--------------------------------------------------------------------------------

Match each leader joint to its bus ID before starting calibration.

.. tutorial-test: skip
.. literalinclude:: ../../examples/gello.sh
   :language: bash
   :start-after: # start serial
   :end-before: # end serial

Use a stable ``/dev/serial/by-id/`` path. Follow the symlink and check that the actual serial node is readable and writable.
Use Dynamixel Wizard to scan serial ports, baud rates and IDs. Record IDs in leader joint order from base to end effector;
record the gripper trigger ID separately. This repository's calibrate does not scan IDs or change EEPROM baud rates.
For example, use the next section's command arguments only if the scan finds joints 1–7, trigger 8 and 57600 baud.

**What you should see**: a serial-port path, seven distinct IDs in mechanical joint order and a separate gripper ID.
You should have installed ``gello`` in the installation chapter. The node list and ``gello-events`` are needed only for an evdev keyboard / pedal.

**If it doesn't**:

* Doctor's ``franka.os:serial`` fails: check the U2D2 / OpenRB-150 USB connection and enumerate again.
* The node belongs to dialout but the user is not in that group: run the commands below, log in again and recheck id.
* The serial port opens but servos cannot be read: check bus power, actual IDs and baud rate. Do not treat example IDs as scan results.
* ``franka.os:groups`` fails or evdev will not open: check read permissions for the selected ``/dev/input`` node. Serial dialout permissions do not cover it.

.. tutorial-test: skip
.. literalinclude:: ../../examples/gello.sh
   :language: bash
   :start-after: # start dialout
   :end-before: # end dialout

2. Calibrate signs, offsets and gripper
--------------------------------------------------------------------------------

Calibration converts raw servo angles into joint angles consistent with the follower, and measures both ends of the gripper trigger.
If the station previously used a SpaceMouse, remove its teleop block first so ``--use`` does not refuse to replace a different backend.

.. warning::

   This step releases leader servo torque and moves the leader by hand. Support the leader and stop all following programs.
   Do not record on the follower at the same time. The calibration pose requires positioning only the leader; it does not command follower motion.

.. literalinclude:: ../../examples/gello.sh
   :language: bash
   :start-after: # start calibrate
   :end-before: # end calibrate

Replace the example port, IDs and baud rate with your own. Follow the prompts to rotate each joint in the follower's defined positive direction,
confirm the suggested joint_signs, then place the leader in the prompted calibration pose.
The default seven-joint pose is ``[0, 0, 0, -pi/2, 0, pi/2, 0]``;
if your mechanism has a different known pose, pass the corresponding seven radian values through ``--calibration-pose``.
Finally, fully open and close the trigger in turn, pressing Enter for each.

To override calibration inputs, ``--joint-signs`` accepts signs in joint order;
if omitted, station signs are used, or all +1 if none exist.
``--joint-limits-yaml`` computes wrap centers from another robot YAML's ``limits.joints``;
if omitted, it uses the current station. ``--skip-sign-check`` skips the per-joint direction check and uses the initial signs directly.
Use it only when those signs have already been verified on this machine.

**What you should see**: seven residuals after ``offset residuals (rad):`` and teleop YAML output,
saved to the station after confirmation. The relationship is ``q = sign * (raw - offset)``, followed by angle wrapping around the wrap center.
Offsets are selected on a quarter-turn grid. Residuals are not installation errors you may ignore arbitrarily.

After calibration, add the following options to the generated teleop block, retaining every measured calibration field:

.. literalinclude:: ../../examples/gello-config.txt
   :language: yaml
   :start-after: # start events
   :end-before: # end events

.. list-table::
   :header-rows: 1

   * - Field to change
     - How to obtain the value
   * - ``options.port``, ``baudrate``, ``joint_ids``, ``gripper_id``
     - Use the previous step's port and Wizard list; for example trigger ID 8 corresponds to ``gripper_id: 8``.
   * - ``joint_signs``, ``joint_offsets_rad``
     - Use the output from the per-joint check and calibration pose. Do not copy generic zeros by hand.
   * - ``gripper_open_rad``, ``gripper_closed_rad``
     - Use raw radian values from calibration with the trigger fully open / closed.
   * - ``wrap_centers_rad``
     - Midpoints of each follower ``limits.joints`` row. Fill the station's ``limits.joints`` from the current robot model's specification first, then calibrate.
   * - ``intervention``, ``event_device``, ``event_keys``
     - New configurations default to always. This example uses always with terminal t/h during active episodes; evdev toggle is unnecessary.
       ``--use`` on an existing GELLO block preserves intervention, event_device and event_keys. Check and edit them as needed.

Keep other generated fields: ``name`` and ``backend``. Unset ``hold_s`` and ``poll_interval_s``
default to 0; device-layer ``gripper_binarize_threshold`` defaults to null. Adjust the latter only if device-layer binarization is needed;
this chapter's leader_cartesian already uses 0.5 to separate open from closed by default.

**If it doesn't**:

* ``moved less than 0.1 rad``: the old sign is retained this time. Repeat the single-joint check with enough motion.
* Residual exceeds pi/8: check pose and signs, then recalibrate. Do not start following immediately.
* ``gripper trigger moved only ...``: check the trigger ID. Open and closed endpoints must differ by at least 0.05 rad.
* ``no limits.joints given``: the previous chapter's template has no ``limits.joints``. A zero wrap center is unsuitable for some Panda joints.
  Add seven limit pairs from the current model's specification and recalibrate, checking joints that cross pi especially carefully.

3. Align leader and follower: read before moving
--------------------------------------------------------------------------------

Compare joint readings and gripper state before practicing motion with a potentially incorrect mapping.

.. literalinclude:: ../../examples/gello.sh
   :language: bash
   :start-after: # start read
   :end-before: # end read

**What you should see**: read's ``q`` contains calibrated leader angles; the other command's ``q (rad)`` contains follower angles.
With the follower safely stopped, manually align the leader to it and compare each value. A fully open trigger gives gripper=1; closed gives 0.
In default always mode, intervening is always True. Space switches device-layer state only if you separately configure evdev toggle mode.
For direct connections, read leader and follower separately. ``teleop read --with-follower`` supports only the daemon route.

**If it doesn't**:

* A value is reversed or differs by about pi/2: recheck joint_ids order, sign and offset. Do not compensate by moving the leader backward.
* Readings jump across pi: check wrap_centers_rad against actual ``limits.joints``.
* Open and close are reversed: capture gripper_open_rad and gripper_closed_rad again. Do not continue by merely swapping labels.

read prints 3 times per second by default. Without ``--duration-s``, it reads until Ctrl-C.
It does not consume terminal t/h; those keys cannot switch device-layer intervening.
If the station declares a daemon but it is stopped, put global ``--direct`` before teleop:

.. literalinclude:: ../../examples/gello.sh
   :language: bash
   :start-after: # start read-routes
   :end-before: # end read-routes

**What you should see**: the daemon route adds ``follower_q`` and per-joint ``delta``,
the latter being leader minus follower in radians. Move only the leader and compare direction, order and differences at several poses;
this read command sends no follower motion. Direct mode opens only the input, not the follower, gripper or cameras.

**If it doesn't**: if the daemon is unreachable, restore the service or remove --with-follower and check input directly;
do not let the daemon and a direct process hold the same device at once.

4. Take over, operate and hand back
--------------------------------------------------------------------------------

Use the previous chapter's Cartesian preset for a short exercise to learn how leader displacement becomes follower action.
The default ``anchored`` mapping remembers leader and follower poses at take-over, then applies relative leader end-effector translation and rotation.
It uses leader forward kinematics, so still check the mechanism, joint order and tool-frame conventions.

.. warning::

   The following resets, homes and opens the gripper. Confirm home, path, workspace and stopping method, with someone supervising.
   Keep the leader steady and begin with very small motions in one direction.

.. literalinclude:: ../../examples/gello.sh
   :language: bash
   :start-after: # start practice
   :end-before: # end practice

In current ``teleop run`` active episodes, terminal ``t`` takes over and ``h`` hands back.
Default always mode has no physical toggle. After the reset the session waits in READY; press space to start the episode (ACTIVE).
Once ACTIVE appears, press t, move the leader, try opening and closing with the trigger, then press h.
Finally press s to mark success; the episode enters POST_EPISODE, where q exits. One person can switch between the leader and the terminal; the full two-person workflow is in :doc:`record`. The next chapter explains labeling.

**What you should see**: the follower follows relative changes in the leader end effector and holds after hand-back;
if the trigger differs from the last gripper state at take-over, first move the trigger to that state before controlling opening and closing.
The session report's ``resolved-config.yaml`` contains ``teleop_device: leader_cartesian`` and
``action_mode: cartesian_delta``. Training actions are six Cartesian deltas plus the gripper.

**If it doesn't**:

* Stuck at READY: the episode starts only when you press space, in the terminal that runs ``rwrl``.
* Space switches device state but does not take over the active episode: this session uses the terminal event source. Press t/h; do not mix the two take-over states.
* The follower stops advancing at the box edge and only slides along the wall: the station's ``limits.tcp`` clips the target into the box; this is not a fault. Hand back, check the request direction and the box, then take over again. Do not enlarge the box to hide a mapping error.
* ``measured TCP outside TCP box``: the measured TCP is beyond the box plus ``measured_margin_m``; the environment stopped the arm and latched a ``SafetyViolation``. Inspect feedback and robot state and restart only after a manual check.

To check the mapping, denote the leader end effector at take-over by ``(p_l0, R_l0)`` and the follower command by ``(p_c, R_c)``.
Subsequent requested position is ``p* = p_c + (p_l - p_l0)``, and rotation is ``R* = (R_l R_l0^T) R_c``;
the leader end effector here comes from forward kinematics. The tracker then limits request speed and lead.
``teleop run`` merges the station's task_frame into the environment; limits come with the robot, and the environment inherits the station's ``limits`` on the first reset.
Explicit source options override environment configuration; local robot YAML is used only if the environment has no setting.
The TCP box (axes from ``task_frame``) clips Cartesian targets into the box, and the robot's joint ranges limit joint targets as well.

5. Switch when joint following is needed
--------------------------------------------------------------------------------

Create a separate joint-action configuration and output file only if you want the follower to follow the leader joint by joint.
Copy resolved-config.yaml from this session directory to ``local/gello-joint.yaml``,
retain its merged robot connection and leader-options paths, and edit these fields.
This extension uses ``python -m real_world_rl.demos``, the same recording entry point called by ``rwrl record``:

.. literalinclude:: ../../examples/gello-config.txt
   :language: yaml
   :start-after: # start joint
   :end-before: # end joint

.. list-table::
   :header-rows: 1

   * - Field to change
     - How to obtain the value
   * - ``teleop_device``, ``robot_teleop_action_mode``
     - The leader reads absolute joint angles; choose leader_arm and joint_abs.
   * - ``real_robot.action_mode``, ``joint_scale_rad``
     - This extension follows the repository's bimanual smoke-test preset: joint_delta and 0.01 rad. Confirm that step size on site.
   * - ``teleop_onset``, ``episode_end_behavior``
     - arm requires alignment at take-over. This example uses auto: the env resets right after an episode. hold supports only Cartesian environments; a joint environment can use confirm instead (it needs an event device such as ``--teleop-event-device stdin``), which gives the two-person flow of :doc:`record`, with the hold using the measured joints; confirm this on site.

Keep the rest of the copied configuration. ``teleop_arm_tolerance_rad`` defaults to 0.1 rad; change it only after reviewing the alignment tolerance again.
Each arm now has seven joint deltas plus the gripper, eight dimensions total. Do not mix this with the earlier seven-dimensional data.

If you switch to ``real_robot.action_mode: joint_abs``, you must also fill in
seven locally verified ``limits.joints`` pairs (``real_robot.limits.joints``, which can only be narrower than the station's range). Absolute actions are mapped using these lower and upper limits.
The robot configuration must also provide ``limits.max_delta_per_step.joint_rad``; for remote connections, set it in the station used by the daemon.
Keep ``joint_pos`` in ``state_fields`` and check remote, token, cameras and home/reset. Changing only the action mode is insufficient.
If the source options file contains multiple robots, select one with ``--teleop-robot NAME``.

**Make the follower track more smoothly.** The station's default joint controller ``impedance`` receives a step target every ``control_hz`` period.
For joint following you can switch to ``joint_controller: trajectory_impedance``: each target is turned by a planner into a continuous trajectory and tracked with GELLO's joint impedance law.
It is written in the **robot-side station**, needs franky, ``serl_controller`` and ``joint_controller`` built together, and a realtime kernel is recommended;
choose, install and check it with doctor as in step 10 of :doc:`controller`; no recalibration is needed. Also raise ``real_robot.control_hz`` in the configuration to 15–30 Hz:

.. literalinclude:: ../../examples/en/controller-joint.yaml
   :language: yaml
   :start-after: # start controller
   :end-before: # end controller

.. code-block:: yaml

   real_robot:
     control_hz: 20     # default 10; GELLO joint collection can use 15 to 30

After raising ``control_hz``, the speeds implied by ``joint_scale_rad`` and ``max_delta_per_step.joint_rad``, which are counted **per step**, rise with it; review them again.
Collection, training and deployment must use the same ``control_hz``, so settle it **before** you start collecting.
While the leader moves, the follower trails by about one braking distance (about 0.06 rad by default); tune it with ``joint_trajectory.max_acceleration``, see :doc:`../reference/joint-controller`.

``--teleop-onset arm`` waits until the maximum joint difference is no greater than ``--teleop-arm-tolerance-rad`` before taking over;
the default tolerance is 0.1 rad. Steps before alignment are not human steps, and alignment is required again after reset.
The default onset policy ``ramp`` takes over immediately: each joint_abs target advances relative to the follower by at most
``--teleop-ramp-step-rad`` (default 0.05 rad) per step. Limited steps are marked teleop_syncing.
joint_delta is already capped by joint_scale_rad; the initial catch-up flag persists until the leader first comes within one step.

The dataset retains these executed actions. By default, the recorder excludes syncing steps from the training rows;
``--keep-syncing-steps`` retains them, but dataset-derived training views still exclude syncing supervision.
The joint route lacks the Cartesian gripper take-over latch. Match the trigger to the current gripper state before taking over.
See :doc:`record` for recording artifacts and recovery. Use a separate dataset for each action mode.

.. warning::

   This joint extension automatically resets after an episode ends, without waiting for n. Confirm leader and follower joint ranges and the homing path first.

.. literalinclude:: ../../examples/gello.sh
   :language: bash
   :start-after: # start joint
   :end-before: # end joint

**What you should see**: the new dataset header's action_contract has action_mode joint_delta and arm_action_dim 7.
For a joint_abs environment, the recorder also checks the maximum leader–follower difference at episode start (default 0.8 rad); with confirm this becomes a start lock in READY: while the difference exceeds the tolerance, space does nothing, the terminal lists the joints that are too far apart, and the session starts once they are aligned, without exiting with an error.

**If it doesn't**: stop if action dimensions or joint limits do not match. Check leader and follower joint order and corresponding ranges;
if ``leader arm too far`` appears, align first. Do not merely increase start_tolerance_rad.

6. Use a second robot arm as the Cartesian leader
--------------------------------------------------------------------------------

This extension uses ``arm_leader`` only to read another arm's joint angles. It sends no motion commands to the leader.
Copy the configuration on the leader host first, then enter the actual leader address and installed backend:

.. tutorial-test: skip
.. literalinclude:: ../../examples/gello.sh
   :language: bash
   :start-after: # start second-copy
   :end-before: # end second-copy

.. literalinclude:: ../../examples/gello-config.txt
   :language: yaml
   :start-after: # start second-station
   :end-before: # end second-station

The outer fake arm is only a carrier. Only teleop opens the real leader, avoiding duplicate connections.
``arm.type`` selects the backend; other arm fields are passed to that backend's constructor.
``gripper`` defaults to false. Without leader gripper readings, the follower retains its last gripper command.
``intervention: always`` provides continuously active readings; actual take-over is controlled by t/h in the follower's recording terminal.

.. warning::

   Connecting does not enable manual guiding. Arrange manual operation according to the leader manufacturer's procedure, and confirm stopping methods on both sides.
   Recording below resets and homes the follower and operates its gripper. Someone must confirm the motion range on each side.

On the leader host, activate an environment containing the arm SDK and daemon dependencies, then start the service:

.. literalinclude:: ../../examples/gello.sh
   :language: bash
   :start-after: # start second-daemon
   :end-before: # end second-daemon

**What you should see**: the service listens on 50052 and supplies leader readings. It does not control the follower.

**If it doesn't**: run ``rwrl doctor --station local/leader.yaml`` on the leader host,
then check the real leader connection and SDK. A healthy outer fake arm does not prove the real leader can be read.

On the follower's recording machine, save the following as ``local/leader-cartesian.yaml``.
Replace leader-host with a reachable host. If the service has a token, add the matching remote_token.

.. literalinclude:: ../../examples/gello-config.txt
   :language: yaml
   :start-after: # start second-source
   :end-before: # end second-source

Choose exactly one leader route: remote_address, robot_yaml pointing to a local carrier configuration,
or an in-process leader part / resolver. The remote route connects when the source starts.
The only built-in kinematics model is the default panda, requiring the leader to report joint_abs for exactly seven joints;
other mechanisms require a matching registered model. kinematics does not change automatically with arm.type.

Prepare ``local/record-cartesian.yaml`` as in step 6 of :doc:`record`, retaining the verified follower configuration.
The ordinary recorder does not automatically merge the station's task_frame. Confirm it is already in real_robot
or its local robot configuration; limits (``limits``) are in the robot configuration or come with the daemon's station. If the preset connects directly to the follower, stop the daemon holding the follower first, but keep the leader daemon running.
Start in the learning environment on the follower machine:

.. literalinclude:: ../../examples/gello.sh
   :language: bash
   :start-after: # start second-record
   :end-before: # end second-record

**What you should see**: the follower holds after reset, anchors when t takes over, and hands back on h.
Before saving demonstrations, verify leader FK against the actual TCP, translation and rotation directions, gripper behavior, sliding at the box edge and stopping effects.
These are on-site acceptance targets, not yet hardware validation results for a second physical leader arm.

**If it doesn't**: fix kinematics first if the joint count or model is wrong. Stop recording if direction or stopping differs from expectations;
do not enlarge boundaries or skip checks to continue collecting.

Extension: bimanual GELLO
--------------------------------------------------------------------------------

Use a separate bimanual preset, with left / right each mapped to one follower and one leader.

.. warning::

   Both arms move during reset. Confirm clearance between the arms and their tools. Verify left-only take-over first, then right-only.
   This preset uses auto episode behavior and may reset automatically after an episode ends.

.. tutorial-test: skip
.. literalinclude:: ../../examples/gello.sh
   :language: bash
   :lines: 27-27

.. literalinclude:: ../../examples/gello.sh
   :language: bash
   :lines: 28-29

Run only the copy line first. Check ``arm_names: [left, right]`` and both remote values in the copied file:
the example uses local 50051 for left and local 50052 for right. Start the configured, calibrated station daemons before recording.
Each daemon's teleop.options.port, IDs, calibration and event node must correspond to that side.
When reading a leader through a service, remote_address names the service HOST:PORT and remote_token matches the service;
for a local leader, robot_yaml points to the configuration containing its teleop block.

This preset uses ``bimanual`` with ``leader_arm``, with joint_delta and 0.01 rad on both sides,
and no shared terminal event source. Bind a separate key in each GELLO's event_keys.toggle,
for example left KEY_F1 and right KEY_F2. Set both intervention values to toggle so one key cannot trigger both sides.

**What you should see**: the right holds during left take-over, and the left holds during right take-over.
The action layout is left seven joints, right seven joints, left gripper, right gripper. An example mask
for left-only take-over is ``1111111000000010``. Success / failure events from both sides merge into labels for the whole episode.

**If it doesn't**: if left and right are reversed, check arm_names, remote and physical serial-port mappings;
if one key controls both sides, check event_device and key bindings. Bimanual archives lack single-arm TCP metrics, but the next chapter still lets you inspect the mask per dimension.

.. note::

   **What needs confirmation on hardware in this chapter**

   Serial permissions, power, IDs, baud rates, positive directions, offset residuals, joint wrap centers, trigger endpoints,
   leader–follower alignment and kinematics, physical key states, neutral checks, gripper take-over, the joint extension and independent bimanual take-over need on-site confirmation.
   Hardware-free tests have verified always mode's neutral-check logic. Alignment thresholds and motion safety still need on-site verification.

Checklist: leader and follower readings match item by item; calibration parameters come from measurements; the neutral gate passes; t takes over and h hands back;
you know whether current output is Cartesian or joint actions.

Continue to the shared workflow in :doc:`record`.
