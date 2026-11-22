Set up a station
================================================================================

You will connect a Franka, a Franka Hand and a RealSense, then write your first experiment configuration.
Complete :doc:`install` first. Run every command in this chapter from the repository root on the NUC, with the environment active.

A ``station`` is a YAML file describing which devices this machine has and how to connect to them.
A ``preset`` is another YAML file describing the observations given to the policy and the actions it outputs for this experiment.
This chapter connects directly to devices. The ``daemon`` lets other programs use devices remotely; you do not need to start it here.

1. Copy the templates
--------------------------------------------------------------------------------

Copy the two templates first. Edit only the local copies afterwards.

.. tutorial-test: skip
.. literalinclude:: ../../examples/station.sh
   :language: bash
   :start-after: # start copy
   :end-before: # end copy

The templates come from the repository's ``robot.franka.example.yaml``, ``gello_cartesian.yaml`` and
``hil_single_arm.yaml``, combined into an example with one arm and one camera.
“Keep the defaults” below means keeping the values in these templates.
Edit ``local/station.yaml`` first. Leave ``local/task.yaml`` until after the device checks.

**What you should see**: ``station.yaml`` and ``task.yaml`` under ``local/``.
Replace ``REPLACE_WITH_...`` in the former with values from your site. You will read and fill in home and boundary values later.

**If it doesn't**:

* Templates are not found: confirm that the terminal is at the repository root.
* ``local/`` already contains your configuration: back it up or choose another filename, and update the paths in subsequent commands too.

**If the station is already commissioned.** Reuse its station file and project environment. Do not overwrite it by copying the templates again.
Before each use, request control in the corresponding arm's Desk and refresh. Confirm that FCI is active,
stop other processes using the devices, then run doctor in step 3. A change in browser control ownership does not mean GELLO needs recalibration.

.. _en-station-setup-wizard:

**Use the wizard instead of filling in the station by hand.** For a new station, you can also run ``rwrl setup``. It probes devices first,
then asks each question, calibrates GELLO if you choose, writes an annotated station and runs the device doctor.
It generates only the station. Keep using ``local/task.yaml`` above for the experiment preset.
Complete the Control network checks below before running the real wizard. Without hardware, rehearse with mock first.

.. literalinclude:: ../../examples/station.sh
   :language: bash
   :start-after: # start setup
   :end-before: # end setup

Every question can be answered with an option. Per-arm options accept one value for all arms or one value per arm in arm order.
For addresses, cameras and GELLO serial ports, enter either the index listed by the wizard or the actual value.

.. list-table:: Common wizard options
   :header-rows: 1

   * - Option
     - What to enter
   * - ``--arms``, ``--arm-names``, ``--robot-ip``
     - 1 or 2 arms, names and each arm's FCI address. Names prefix doctor rows.
   * - ``--arm-backend``, ``--gripper``
     - ``franky``, ``serl_http`` or ``fake`` for the arm; the gripper also accepts ``none``.
   * - ``--robot-model``
     - ``panda`` or ``fr3``, which determines default joint limits. Required when creating a new arm non-interactively.
   * - ``--cartesian-controller``, ``--realtime``
     - The controller defaults to ``impedance``; ``internal`` / ``serl`` are also available. Realtime is ``ignore`` / ``enforce``. internal, serl or a PREEMPT_RT kernel default to enforce; otherwise ignore. See :doc:`controller` for SERL installation.
   * - ``--home-joints``, ``--joint-limits``, ``--safety-box``
     - Comma-separated joint angles, lo/hi pairs for each joint and a base-frame position box. Set home to ``none`` to measure it in step 4. Do not treat wizard suggestions as station acceptance results.
   * - ``--max-delta-xyz-m``, ``--max-delta-joint-rad``, ``--control``
     - Per-step position / joint caps and an impedance-parameter mapping. See ``setup --help`` in this step for the full format.
   * - ``--cameras``
     - ``SERIAL=wrist``, comma-separated for multiple cameras; ``none`` for no cameras.
   * - ``--camera-resolution``, ``--camera-fps``, ``--camera-depth``
     - For example ``1280x720``, ``15``, ``yes``. If omitted, new cameras use 640×480, 30 fps and no depth; existing cameras keep their values.
   * - ``--teleop``, ``--gello-port``
     - ``gello`` / ``spacemouse`` / ``fake`` / ``none``. For GELLO, use a stable ``/dev/serial/by-id/`` path.
   * - ``--gello-baudrate``, ``--gello-joint-ids``, ``--gello-gripper-id``
     - Baud rate, comma-separated joint-servo IDs and trigger ID (``none`` if absent).
   * - ``--intervention``, ``--event-device``
     - ``always`` or ``toggle``; an event-device path or ``none``. See :doc:`gello` for pedal and take-over settings.
   * - ``--daemon-port``
     - A device-service port for each arm. Use different ports for two arms.
   * - ``--calibrate`` / ``--no-calibrate``
     - Calibrate GELLO now / later. Interactive mode asks; non-interactive mode defaults to no calibration.
   * - ``--scope local`` / ``--scope user`` / ``--output PATH``
     - Write to ``./local/station.yaml`` / ``~/.config/rwrl/station.yaml`` / the specified path.
   * - ``--allow-unprobed``
     - Accept an IP, serial number or serial port not found by probing. Does not replace the real checks in step 3.
   * - ``--scan-subnet``, ``--port``, ``--probe-timeout-s``
     - Additional subnet and serial-port probes, and TCP probe timeout. Read the help before setting these if needed.

Use ``--non-interactive`` to make a script fail immediately when an answer is missing.
The following command takes answers from an existing station and updates only cameras. For a new station, you can also supply every answer as an option.
If a question has neither an option nor an old value, it fails instead of silently accepting an interactive suggestion.

.. literalinclude:: ../../examples/station.sh
   :language: bash
   :start-after: # start setup-noninteractive
   :end-before: # end setup-noninteractive

An existing output file is first backed up as ``station.yaml.bak.<UTC timestamp>`` (other filenames receive the same suffix).
A new, uncalibrated GELLO is written with zero offsets, all ``+1`` signs and a ``TODO calibrate`` comment.
Do not use these placeholders to drive a follower. Calibrate with :doc:`gello` afterwards, or re-enter the wizard:

.. warning::

   Calibration disables GELLO servo torque. An operator must stand beside the leader and manually move its joints and trigger.
   Keep the follower stopped. Do not start teleoperation with unverified offsets, signs or trigger ranges.

.. literalinclude:: ../../examples/station.sh
   :language: bash
   :start-after: # start setup-calibrate
   :end-before: # end setup-calibrate

Non-interactive mode cannot also request ``--calibrate``. To rehearse file writing and doctor only, use a separate output file:

.. literalinclude:: ../../examples/station.sh
   :language: bash
   :start-after: # start setup-mock
   :end-before: # end setup-mock

``--mock`` uses preset probe results, a simulated GELLO and a mock doctor. Do not use the rehearsal file as the site configuration.

**What you should see**: the written path, device-check ``Result:`` and follow-up commands.
**If it doesn't**: supply options or an old station for missing answers; fix connections if probing fails; handle doctor blockers as in step 3.
A written file does not mean the devices passed.

.. _en-station-multi-arm:

**Two arms and partial reconfiguration.** Use different IPs, names, serial ports and daemon ports for the two arms.
Complete this chapter's checks for each arm first, then do the bimanual exercise in :doc:`gello`. The first command below starts the bimanual wizard; the next two check each arm:

.. literalinclude:: ../../examples/station.sh
   :language: bash
   :start-after: # start setup-bimanual
   :end-before: # end setup-bimanual

To reconfigure only part of a station, pass ``--station`` with ``--only cameras``, ``--only teleop`` or
``--only daemon``. The other parts retain their old values. Do not copy home or calibration values between arms.

**Connect the Control network.** Connect the NUC Ethernet cable to the controller box's Control port, not the Arm port on the robot base.
The following assumes the controller box is ``172.16.0.2`` and the NUC's dedicated interface is ``172.16.0.1/24``.
Use the site values if these addresses have changed. Check the route first, then replace ``ROBOT_IF`` with the actual interface name:

.. tutorial-test: skip
.. literalinclude:: ../../examples/station.sh
   :language: bash
   :start-after: # start network-check
   :end-before: # end network-check

If the interface lacks this address, add it temporarily. Do not repeat this if it already exists. The temporary address disappears after reboot.

.. tutorial-test: skip
.. literalinclude:: ../../examples/station.sh
   :language: bash
   :start-after: # start network-temporary
   :end-before: # end network-temporary

If NetworkManager manages this dedicated interface, reuse its connection profile; create one if none exists.
Before changing it, confirm that this interface is not carrying your current SSH or Internet connection:

.. tutorial-test: skip
.. literalinclude:: ../../examples/station.sh
   :language: bash
   :start-after: # start network-persistent
   :end-before: # end network-persistent

Do not set a gateway on the robot link or replace the Internet default route. Check whether a VPN is taking over the robot subnet, then check the route and ping again after reboot.
If the interface is unmanaged, use the network configuration tool that actually manages it. Do not give it to two managers at once.

**What you should see**: the route leaves through the Control interface with source address ``172.16.0.1``, and ping receives replies.
**If it doesn't**: check the interface, site subnet, Ethernet cable and controller-box power. Do not proceed to arm motion.

2. Fill in connection addresses and the camera serial number
--------------------------------------------------------------------------------

Open the target arm's Desk on the NUC first. Desk runs on the controller box; you do not install it on the NUC.
Confirm that the page belongs to the controller box on site, then check each item:

#. The page is reachable and status lights are normal. Resolve faults first.
#. You are logged in and this machine holds control. Use Request control if needed, then refresh to confirm.
#. Unlock joints has been completed, and End Effector Power On is enabled if a Hand is attached.
#. Franka Control Interface appears under System's Installed Features.
#. Activate FCI from the Desk menu. Running a Desk task does not replace activating FCI.

Record the Control and System image versions from the Dashboard.
An Installed Features entry such as ``FCI 1.0.0`` is a feature version, not the firmware or robot server version,
nor the libfranka version to install. See step 2 of :doc:`controller` for version mappings and how to read handshake errors.

Next fill in the two values that let the program find devices: the Franka IP and RealSense serial number.

Find this block in ``local/station.yaml`` and edit ``robot_ip``:

.. literalinclude:: ../../examples/en/station.yaml
   :language: yaml
   :start-after: # start arm
   :end-before: # end arm

.. list-table::
   :header-rows: 1

   * - Field to change
     - How to obtain the value
   * - ``robot_ip``
     - Open the site's Franka Desk in the NUC browser. Take the IP from the currently reachable ``https://<actual IP>/desk/``.
   * - ``robot_model``
     - Check the system version in the same Desk's system settings: 4.x is a Franka Emika Robot; enter ``panda``.
       5.x is a Franka Research 3; enter ``fr3``.

Keep the other fields at their defaults: ``type``, ``name``, ``arm_backend``, ``realtime``,
``relative_dynamics_factor``, ``cartesian_controller``, ``claim``。
``cartesian_controller: impedance`` means franky's impedance tracker executes end-effector targets. It also works on a regular kernel.
Keep it for now. After step 3, :doc:`controller` will help you choose ``impedance``, ``internal`` or ``serl``,
install its software and set ``realtime``.
``robot_model`` selects the joint-limit table for SERL's joint reset and supplies the default ``joint_limits`` in ``rwrl setup``.
Review ``relative_dynamics_factor`` when you need to adjust homing speed. It scales velocity, acceleration and jerk for ``move_to_*``.
``claim: true`` keeps the host's exclusive device lock.

Plug in the RealSense and run discovery:

.. literalinclude:: ../../examples/station.sh
   :language: bash
   :start-after: # start discover
   :end-before: # end discover

Run ``rwrl --json discover`` for a machine-readable report. The result is a ``schema_version: 1`` object
with ``cameras``, ``teleops``, ``can_interfaces``, ``hosts``, ``hid_devices`` and ``unknown_hid`` lists;
empty lists mean nothing was found. ``hosts`` comes from probing station IPs, not from an FCI control session.

Enter the listed serial number in the camera block:

.. literalinclude:: ../../examples/en/station.yaml
   :language: yaml
   :start-after: # start camera
   :end-before: # end camera

.. list-table::
   :header-rows: 1

   * - Field to change
     - How to obtain the value
   * - ``cameras[0].serial``
     - Copy it from the ``realsense`` row of the ``rwrl discover`` command you just ran.

Keep the other fields at their defaults: ``name: wrist``, ``backend: realsense``, 640 × 480, 30 fps,
``enable_depth: false``. Check supported camera modes before changing the capture mode. Enable depth only when needed.
With only one camera, an empty serial number also selects it automatically. Fill it in here to make adding a second camera easier later.

**What you should see**: the discovery report lists the camera in the format below, and both text placeholders have been replaced.

.. literalinclude:: ../../examples/en/station-output.txt
   :language: text
   :start-after: # start discover
   :end-before: # end discover

**If it doesn't**:

* ``cameras: none``: check the ``realsense`` extra, USB connection and cable, then run discovery again.
* Several serial numbers appear: unplug and reconnect cameras one at a time to identify ``wrist``, then fill it in.
* Desk will not open: fix the network connection from the NUC to the controller box before continuing.

3. Have doctor check the devices
--------------------------------------------------------------------------------

Connect to the devices and read feedback now. Confirm that addresses and drivers work before measuring home.

``doctor`` generates a check report, with one check per row. Acquire control in Desk and activate FCI first,
stop other programs using the devices, then run:

.. literalinclude:: ../../examples/station.sh
   :language: bash
   :start-after: # start doctor
   :end-before: # end doctor

Doctor connects and disconnects without homing or moving the arm. Device initialization can still change device state, such as disabling GELLO torque.

.. warning::

   You must be able to stop motion on site. With a direct connection, ``rwrl stop`` also needs an FCI connection;
   a second connection cannot stop an arm held by a recorder or actor. End the program holding the connection first; use the physical stop device in an emergency.
   ``BLOCKED`` does not stop or preempt another process for you.
   An in-process watchdog does not cover a frozen process. An independent supervisor can terminate its child; firmware then handles connection loss.
   The daemon watchdog covers silent servo clients, not failure of the daemon or NUC itself.

The report lists host checks first, then device checks for each arm. Find the first failure by layer; do not treat later skipped checks as passes:

.. list-table:: Six layers of device checks
   :header-rows: 1

   * - Layer
     - Row name (prefixed by the robot name)
     - What it checks
   * - 0 deps
     - ``franka.deps``
     - Required extras
   * - 1 sdk
     - ``franka.sdk:*``
     - Whether each device SDK can be imported
   * - 2 os
     - ``franka.os:*``
     - Network, device nodes and permissions
   * - 3 handshake
     - ``franka.handshake``
     - Exclusive claim and connection; feedback and mode checks are skipped on failure
   * - 4 feedback
     - ``franka.feedback:*``
     - Whether the arm, gripper, cameras and teleop devices return data
   * - 5 mode
     - ``franka.mode:*``
     - Robot mode, versions and realtime requirements

``Report:`` at the end gives the report directory. If blockers remain, ``Needs attention:`` repeats the issues and hints.
The command exits 0 when results contain only PASS, WARN and SKIPPED; it exits 1 if FAIL, BLOCKED or PENDING_ACTION remains.

**What you should see**: the following checks pass, with gripper feedback in ``franka.feedback:gripper`` as well.
This example follows the source code's output format. Use your own report for site results.

.. literalinclude:: ../../examples/en/station-output.txt
   :language: text
   :start-after: # start doctor
   :end-before: # end doctor

Rehearse the complete report without hardware (machine-dependent values are placeholders):

.. code-block:: bash

   rwrl doctor --mock --station examples/stations/robot.franka.example.yaml

.. doctor-example: mock
.. literalinclude:: ../../examples/en/station-output.txt
   :language: text
   :start-after: # start mock-doctor
   :end-before: # end mock-doctor

**If it doesn't**:

* ``franka.os:ping`` / ``franka.os:tcp`` fails: check the IP, NUC interface and controller-box state.
* ``franka.handshake`` fails: check FCI and device ownership. If it reports ``libfranka mismatch: robot server N, ...``,
  record N and read step 2 of :doc:`controller` before deciding how to install franky. If you intend to use ``serl``, \ **do not**\ run
  ``rwrl doctor --install-sdk``. Build franky and serl_controller together on this machine as in step 4B of :doc:`controller`.
  Only other controllers should use ``rwrl doctor --station local/station.yaml --install-sdk`` to switch to a matching franky wheel.
  ``--install-sdk`` handles only version mismatches confirmed by the handshake.
* ``franka.feedback:arm`` fails: check power, emergency stop and Desk state. Fix them and rerun.
* ``franka.feedback:camera:wrist`` reports ``no frames within 2s``: check USB 3 bandwidth, change ports and reconnect.
  For abnormal gripper feedback, check travel and units using the gripper checks below.

Use guided repair for serial-port, input-group or USB-latency issues. It previews and asks first, then applies changes and rechecks:

.. literalinclude:: ../../examples/station.sh
   :language: bash
   :start-after: # start doctor-fix
   :end-before: # end doctor-fix

**What you should see**: checks after repair, or a PENDING_ACTION for you to complete.
**If it doesn't**: after group membership changes, log out and back in, then rerun ordinary doctor. Opening a new terminal is not enough.

4. Read home and write it back to the file
--------------------------------------------------------------------------------

Place the arm at the starting pose for the next experiment and read its joint angles as the home used by this repository's reset.
``reset`` is the reset procedure at the start of a new episode. This chapter's configuration homes the arm and opens the gripper.

.. warning::

   You will manually guide the arm next. Confirm the payload, stopping method, surrounding clearance and on-site supervision first.
   Do not run ``home go``, recording or reset during measurement. Before later resets,
   also confirm that homing and opening the gripper will not cause a collision or drop an object.

Use the site's manual-guiding procedure to reach a safe starting pose. Exit guiding, then read:

.. literalinclude:: ../../examples/station.sh
   :language: bash
   :start-after: # start arm-state
   :end-before: # end arm-state

**What you should see**: ``status`` shows ``robot: franka (franka)`` and ``cameras: wrist``;
``q`` and ``tcp`` have the format below. These numbers are illustrative only.

.. literalinclude:: ../../examples/en/station-output.txt
   :language: text
   :start-after: # start home
   :end-before: # end home

Copy the seven numbers after ``q (rad):`` unchanged into ``home_joints``. For the example above, the result is:

.. literalinclude:: ../../examples/en/station.yaml
   :language: yaml
   :start-after: # start home
   :end-before: # end home

.. list-table::
   :header-rows: 1

   * - Field to change
     - How to obtain the value
   * - ``home_joints``
     - Use the seven numbers from ``rwrl --station local/station.yaml q`` in this step. Also save the ``tcp`` output for review.

Keep the other fields at their defaults. When changing the starting pose, read it again and update this line.
After saving, this line should match your own ``q`` output.

**If it doesn't**:

* The read command cannot connect: confirm that manual guiding has ended, then recheck Desk control ownership and FCI.
* ``q`` is not seven finite numbers: stop filling in values and rerun doctor to check arm feedback.
* YAML will not load after editing: preserve brackets, commas and the original indentation, then check again.

**Optional: write the measured home to the experiment preset.** Still save the station home as described above.
If this task needs its own home, exit manual guiding and run:

.. literalinclude:: ../../examples/station.sh
   :language: bash
   :start-after: # start home-preset
   :end-before: # end home-preset

``home set`` does not move or ask for confirmation. After checking the station, arm and preset joint limits,
it writes the measured values to four decimal places into ``real_robot.task.reset.home_joints``, preserving comments and key order.
The old file remains in the same directory as ``task.yaml.<id>.bak``.

**What you should see**: the old value, new value and backup path. ``home show`` reports preset task.reset.home_joints as the source.
This value takes precedence over station home. “Use the station value” in step 9 applies only to tasks without this override.
**If it doesn't**: pass an explicit path if the preset is missing. For a limit violation, check the measured pose and limits first; do not relax limits just to write the value.

5. Measure the workspace and check the saved result
--------------------------------------------------------------------------------

Read the TCP at the corners of the allowed region to obtain the boundaries this repository uses to limit end-effector targets.

Follow the manual-guiding procedure from the previous step. First choose a region with clearance for tools and obstacles,
then guide to each of its eight corners. At each point, exit guiding and run:

.. literalinclude:: ../../examples/station.sh
   :language: bash
   :start-after: # start corner
   :end-before: # end corner

**What you should see**: one TCP reading each time. The following illustrates eight corners of a base-frame position box.

.. literalinclude:: ../../examples/en/station-output.txt
   :language: text
   :start-after: # start corners
   :end-before: # end corners

Take the minimum and maximum x, y and z values from these readings separately.
The example gives ``low = [0.3, -0.3, 0.05]`` and ``high = [0.7, 0.3, 0.6]``.
Tighten the bounds inward to account for measurement error on site, then recheck the tool, arm and paths within the region.
Reachable corners do not guarantee that the entire box is collision-free.

Write the result to ``local/station.yaml``:

.. literalinclude:: ../../examples/en/station.yaml
   :language: yaml
   :start-after: # start bounds
   :end-before: # end bounds

.. list-table::
   :header-rows: 1

   * - Field to change
     - How to obtain the value
   * - ``workspace.low`` / ``workspace.high``
     - Take lower / upper XYZ bounds from every ``rwrl --station local/station.yaml tcp`` reading in this step, in m.
   * - ``safety_box``
     - Concatenate the reviewed low and high values in the order xmin, ymin, zmin, xmax, ymax, zmax.

Keep the other fields at their defaults: ``workspace.frame: base`` and ``max_delta_per_step``.
The latter limits each step from the measured state to the servo target to the example's 0.05 m / 0.1 rad.
Adjust it only after checking the control period and motion on site. Both position ranges apply; their intersection is used.

After saving, check the values in the file:

.. literalinclude:: ../../examples/station.sh
   :language: bash
   :start-after: # start workspace
   :end-before: # end workspace

**What you should see**: ``workspace``, ``base_safety_box`` and ``home_joints`` match what you just entered.
``home_within_limits`` being true means station home is within the configured joint limits. False means a violation;
check ``home_violations``. Null means home or joint limits are missing and must not be treated as a pass.
Below is an excerpt showing the output fields. The output also labels units as m, rad and xyzw.

.. literalinclude:: ../../examples/en/station-output.txt
   :language: text
   :start-after: # start workspace
   :end-before: # end workspace

**If it doesn't**:

* low is greater than or equal to high: reorganize lower and upper bounds separately for x, y and z.
* Output still shows old values: save the file and confirm that the command uses ``local/station.yaml``.
* A corner or path is unsafe: shrink or reselect the region. Do not force the arm to reach a point just to complete the box.

6. Read the gripper opening
--------------------------------------------------------------------------------

Read gripper feedback once to confirm that the software opening matches the physical opening.

.. literalinclude:: ../../examples/station.sh
   :language: bash
   :start-after: # start gripper-state
   :end-before: # end gripper-state

**What you should see**: width, normalized ratio, ``grasped`` and feedback source. An example format follows:

.. literalinclude:: ../../examples/en/station-output.txt
   :language: text
   :start-after: # start gripper
   :end-before: # end gripper

The Franka Hand in this chapter uses the template's gripper block:

.. literalinclude:: ../../examples/en/station.yaml
   :language: yaml
   :start-after: # start gripper
   :end-before: # end gripper

.. list-table::
   :header-rows: 1

   * - Field to change
     - How to obtain the value
   * - None; check ``max_width_m``
     - Check 0.08 m against the gripper specification and the width from ``gripper show``. Change it only if actual travel differs.

Keep the other fields at their defaults: ``backend: franky``, ``mode: binary``, ``binary_threshold: 0.5``,
``shares_arm_session: false``. The Hand connects independently. In binary mode, a device action greater than 0.5 opens it.
The last four keys specify HIL-SERL's gripper commands: close grasps at ``speed_m_s`` 0.3 m/s with ``grasp_force_n`` 130 N to
``close_width_m`` 0.01 m (an empty grasp also counts as success); open moves to ``open_width_m`` (``null`` means fully open).
Commands run in the background without blocking control steps. Repeated button presses execute only the latest request. Omitting these keys uses the same values,
so old stations also use them without edits. Reduce ``grasp_force_n`` first for fragile objects (maximum 130).
Switch to ``continuous`` only if the task needs continuous width, and check the action convention at the same time.

**If it doesn't**:

* ``robot has no gripper``: check that the station still contains the gripper block.
* Doctor's ``franka.feedback:gripper`` says readings are unreliable: check ``max_width_m`` and the SDK's width units.
* ``grasped=unknown``: current feedback cannot determine grasp state. Confirm using site feedback; do not record it as a successful grasp.

7. Read camera status
--------------------------------------------------------------------------------

Read a few image frames to confirm that ``wrist`` is the correct device and keeps producing frames.

.. literalinclude:: ../../examples/station.sh
   :language: bash
   :start-after: # start camera-state
   :end-before: # end camera-state

**What you should see**: the serial number, 640 × 480 image shape, frame age and health status. An example format follows:

.. literalinclude:: ../../examples/en/station-output.txt
   :language: text
   :start-after: # start camera
   :end-before: # end camera

30 fps is the requested rate; ``measured`` is the measured rate. They need not be exactly equal.
``camera show`` reads 30 times by default; change this with ``--frames N``. It exits 1 when checks fail, such as no frames.
Also save a frame and visually confirm that the image matches the camera name:

.. literalinclude:: ../../examples/station.sh
   :language: bash
   :start-after: # start camera-grab
   :end-before: # end camera-grab

**What you should see**: ``wrist-color.png`` and ``wrist.json`` in the directory;
``wrist-depth.png`` also appears if depth is enabled and a depth frame is returned. The JSON saves camera configuration, frame times, image shapes and other information.
The command does not overwrite capture files with the same name. Use a new output directory for another capture.

**If it doesn't**:

* ``wrist`` is not found: check the camera ``name`` in the station. Use the same name in the command and configuration.
* ``failed cameras: wrist`` or no frames: check USB 3 bandwidth, cable and serial number, then reconnect.
* Startup reports an unsupported mode: set width, height and fps to a combination this camera supports.

8. Choose observations for the policy
--------------------------------------------------------------------------------

Once all devices can be read, edit ``local/task.yaml`` to select this experiment's cameras and image size.

.. literalinclude:: ../../examples/en/task.yaml
   :language: yaml
   :start-after: # start observations
   :end-before: # end observations

.. list-table::
   :header-rows: 1

   * - Field to change
     - How to obtain the value
   * - ``obs.rgb``
     - Enter the camera ``name`` from the station you just checked; this chapter uses ``[wrist]``.
   * - ``obs.image_size``
     - Enter height and width for the model. Start with ``[128, 128]`` in this example, and keep recording, training and deployment consistent.

Keep the other fields at their defaults: ``env_backend: real_robot``, ``env_id: pick_and_place``,
``obs.state: true``, ``obs.depth: []``. You can change ``env_id`` to identify another experiment;
it does not automatically provide success detection. Change both the station's depth switch and ``obs.depth`` only when depth is needed.

**What you should see**: the configuration uses ``wrist``. Capture remains 640 × 480, while model input is 128 × 128.
The environment resizes images. The policy will receive the keys ``state`` and ``rgb_wrist``;
the RGB tensor is uint8 with shape ``(1, H, W, 3)``.
To crop images, measure pixel coordinates from the saved color image first. Under ``real_robot.preprocess.wrist``,
``source_resolution`` is the original image's ``[height, width]``, and ``roi`` is ``[top, left, height, width]``.
The ROI must lie inside the original image. Setting an ROI also requires source_resolution; mismatched actual frame dimensions cause an error.
Processing crops first, then resizes to ``obs.image_size``. You can instead use ``crop_region: center_square``
or normalized ``[top, left, bottom, right]`` (0–1, increasing bounds) in place of a pixel ROI. Do not set both;
``crop_region`` is resolved against the camera's original dimensions and requires ``image_size``.
``resize`` defaults to ``pil_bilinear``; ``cv2_linear`` is also available. RGB uses the corresponding bilinear interpolation; depth uses nearest-neighbor.
Do not use the resized 128×128 as the original image size. ROI, depth selection and resize dimensions form the observation contract and must match across recording, training and deployment.
This section only checks the file; it has not started a policy.

**If it doesn't**:

* Names differ from the camera check: set ``obs.rgb`` to the actual station names.
* Adding a second camera: give it a unique name and serial number, pass the camera checks, then add its name here.
* Training and deployment image shapes differ: align ``image_size`` and preprocessing before using the model.

9. Choose actions and episode length
--------------------------------------------------------------------------------

Finally, check the preset's action scales and timing so later recording and policies use the same conventions.
Keep the following block under ``real_robot:``.

.. literalinclude:: ../../examples/en/task.yaml
   :language: yaml
   :start-after: # start action
   :end-before: # end action

.. list-table::
   :header-rows: 1

   * - Field to change
     - How to obtain the value
   * - ``action_mode`` / ``xyz_scale_m`` / ``rot_scale_rad``
     - Keep ``cartesian_delta``, 0.05 m and 0.3 rad initially. Check them against the teleop target limits in the following chapter.
   * - ``control_hz``
     - Start with 10 Hz in this example. After connecting, adjust to the measured loop capacity, and keep it consistent during collection and deployment.
   * - ``max_episode_steps``
     - Set the episode length allowed by the task. This example uses 200 steps, excluding reset time.

Keep the other fields at their defaults: ``num_joints: 7``, ``gripper_in_action: true`` and ``state_fields``.
State is concatenated in this field order: TCP pose, velocity, joint positions and velocities. Change the order or fields only when changing the policy's input convention.

**What you should see**: this chapter's configuration produces seven-dimensional actions. The table below describes how each part executes.

.. list-table:: Seven-dimensional actions
   :header-rows: 1

   * - Dimensions
     - Meaning
     - Input range and execution units
   * - 0–2
     - Position delta
     - Each dimension [-1, 1] × ``xyz_scale_m``, in m, relative to the measured TCP, in the base frame
   * - 3–5
     - Rotation delta
     - Each dimension [-1, 1] × ``rot_scale_rad``, a rotation vector in rad
   * - 6
     - Gripper
     - [-1, 1] becomes device width [0, 1] through ``(a + 1) / 2``; -1 closes, +1 opens

The default ``current`` action semantics use the mapping above. Use station home and keep the reset wait:

.. literalinclude:: ../../examples/en/task.yaml
   :language: yaml
   :start-after: # start reset
   :end-before: # end reset

.. list-table::
   :header-rows: 1

   * - Field to change
     - How to obtain the value
   * - None
     - ``home_joints`` was filled in during step 4. Use the station value here.

Keep the other field at its default: ``reset_settle_s: 0.5``. Adjust it if the arm needs longer to settle after homing on site.
Without ``real_robot.task``, the task name defaults to ``none``. Reset homes the arm, opens the gripper and waits.

**If it doesn't**:

* Existing demonstrations use joint actions: match their action convention before choosing a preset. Matching only the seven-dimensional length is not enough.
* A smaller motion range is needed: review action scales, device per-step caps and the next chapter's teleop target limits together.
* Episodes always end early: check ``max_episode_steps``. It is not the reset timeout.
* Reset reaches the wrong pose: recheck the station's measured home. Do not keep trying motion with template numbers.

10. Remember the station and preset
--------------------------------------------------------------------------------

Every subsequent command needs these two files. Use ``rwrl use`` to remember them so you no longer need to write
``--station`` and ``--preset`` every time.
``rwrl use`` saves the choice in ``local/.rwrl-use.json``. It applies only to commands run from the current directory (the repository root).

.. literalinclude:: ../../examples/station.sh
   :language: bash
   :start-after: # start use
   :end-before: # end use

**What you should see**: each command checks that the file loads, remembers it and prints the current choice:

.. literalinclude:: ../../examples/en/station-use-output.txt
   :language: text
   :start-after: # start remember
   :end-before: # end remember

Afterwards, run ``rwrl use`` without arguments to view the current choices. Commands with omitted arguments print to stderr
which file they used:

.. literalinclude:: ../../examples/station.sh
   :language: bash
   :start-after: # start use-show
   :end-before: # end use-show

.. literalinclude:: ../../examples/en/station-use-output.txt
   :language: text
   :start-after: # start fallback
   :end-before: # end fallback

.. tip::

   Device commands in later chapters omit ``--station local/station.yaml`` and ``--preset local/task.yaml``.
   An explicit ``--station`` or ``--preset`` always takes precedence. If set, ``$RWRL_STATION`` also takes precedence over the remembered station.
   ``train``, ``record`` and ``deploy`` do not read the remembered preset; still pass ``--config`` explicitly.

**If it doesn't**:

* ``station local/station.yaml does not exist``: the command is not running from the repository root, or the template was not copied in step 1.
* ``... does not load: ...``: the file has YAML syntax errors or invalid fields. Fix the reported error before remembering it.
* Output says ``station: ... ($RWRL_STATION)``: the environment variable overrides the remembered choice. Run ``unset RWRL_STATION`` if it is not needed.
* A remembered file was moved or renamed: the command reports ``chosen with `rwrl use station` does not exist``;
  rerun ``rwrl use station <yaml>``, or clear the choice with ``rwrl use --clear station``.

11. Check the arm mounting and measure the table frame
--------------------------------------------------------------------------------

Do this before the first teleoperation. It determines how SpaceMouse input (and any command expressed in table directions) becomes arm motion.

First inspect the mounting: **upright**\ (base flat on the table), \ **side-mounted**\ (for example rotated about its own x axis by roughly 45°, common in this lab),
or otherwise tilted or rotated. The SpaceMouse sits on the table, so its directions are table directions: forward = table +x, left = +y, up = vertical +z.
The Franka base frame follows the base. If the base is not upright, base “up” is not vertically upward.
SpaceMouse commands must therefore be \ **rotated into the base frame**\ before becoming end-effector targets:

.. code-block:: text

   SpaceMouse readings (table frame: forward +x, left +y, up +z; three translation xyz and three rotation rpy dimensions)
        │   × orientation profile yaw (only describes how the SpaceMouse faces on the table; see step 3 of the SpaceMouse chapter)
        ▼
   base_R_table  = upper-left 3×3 of station task_frame.base_T_frame
        │   Δp_base = base_R_table · Δp_table ;Δω_base = base_R_table · Δω_table
        ▼   (the same matrix for translation and rotation)
   Deltas in the base frame
        │   position: target = current pose + scaled Δp_base; orientation: left-multiply the current orientation by the rotation delta
        ▼
   End-effector target (base frame)
        ▼
   Controller cartesian_controller: impedance | internal | serl (receives a base-frame target; does not rotate it again)

``base_R_table`` is identity only when the arm is upright and the table x / y axes align with the base x / y axes.
You can omit ``task_frame`` in that case. An upright base alone does not rule out a horizontal yaw offset.
For other mountings, set ``task_frame`` so both SpaceMouse modes, tracker and serl, and all three controllers use it:

.. list-table::
   :header-rows: 1

   * - Case
     - How SpaceMouse commands rotate into the base frame
   * - tracker mode, with an orientation profile
     - Profile rotation: ``base_R_table`` × profile yaw (profile calibrated with ``--frame table``)
   * - tracker mode, no profile, with ``task_frame``
     - ``base_R_table`` only
   * - serl mode (default)
     - ``base_R_table`` only, without profile yaw
   * - serl mode, ``use_profile_rotation: true``
     - Profile rotation (``base_R_table`` × profile yaw)
   * - No ``task_frame`` and no profile rotation in use
     - No rotation; readings are treated directly as base-frame input. Confirm that input and base axes align (tracker still uses profile rotation when a profile exists)

Policy actions are already base-frame deltas and do not pass through this chain.

**11.1 Write nominal values and determine the sign.** Start with a nominal ``task_frame`` for the mounting. The matrix below represents table axes relative to base axes as ``Rx(+45°)``;
it transforms table to base and is the inverse of the mounting angle “base rotated +45° relative to table”:

.. literalinclude:: ../../examples/en/station-frame.yaml
   :language: yaml
   :start-after: # start nominal
   :end-before: # end nominal

``base_T_frame`` is a 4×4 matrix. The three columns of its upper-left 3×3 are the table +x, +y and +z directions in the base frame;
the final column is the table-frame origin's position in the base frame (start with nominal 0).
Determine +45° versus −45° with one vertical lift. Place the TCP in the middle of the workspace and read ``tcp`` once,
then manually guide it \ **vertically**\ upward (toward the room ceiling, not along an arm axis) by about 10 cm and read it again:

.. literalinclude:: ../../examples/station.sh
   :language: bash
   :start-after: # start frame-sign
   :end-before: # end frame-sign

.. literalinclude:: ../../examples/en/station-frame-output.txt
   :language: text
   :start-after: # start frame-sign
   :end-before: # end frame-sign

The difference between the two xyz readings is “table up” in the base frame: the matrix's third column (multiplied by 0.1 m):

.. list-table::
   :header-rows: 1

   * - Approximate xyz change
     - Mounting
     - Nominal ``base_R_table``
   * - ``[0, 0, +0.10]``
     - z axes aligned; still check horizontal x / y axes
     - Identity only if x / y also align (omit ``task_frame``)
   * - ``[0, -0.07, +0.07]``
     - Table axes relative to base axes: ``Rx(+45°)``
     - ``Rx(+45°)`` above
   * - ``[0, +0.07, +0.07]``
     - Table axes relative to base axes: ``Rx(-45°)``
     - ``Rx(-45°)``: change row 2, column 3 of the matrix above to ``+0.7071…``, and row 3, column 2 to ``-0.7071…``

**11.2 Measure the actual frame.** Nominal values are only a starting point; real mountings always have deviations. For example, one nominally 45° arm in this lab
measured roll 41.8°, pitch 2.3°, yaw −6.7° through hand–eye calibration; another measured −46.1°, −0.4°, 3.9°.
Measure the full roll / pitch / yaw using three points:

1. Origin ``frame-origin``: an easily located point on the table, for example near a corner;
2. ``frame-x``: 20–30 cm from the origin along table “forward” (the direction the SpaceMouse front should face);
3. ``frame-y``: 20–30 cm from the origin along table “left”.

Keep all three points at the same height above the table (for example, let the fingertip just touch marks taped to the table). Follow step 4's manual-guiding procedure:
guide to one point, exit guiding and capture, then move to the next:

.. literalinclude:: ../../examples/station.sh
   :language: bash
   :start-after: # start frame-capture
   :end-before: # end frame-capture

``--output`` is relative to the station directory, so this writes ``local/frame-points.yaml``. Preview the candidate without ``--save`` first; when satisfied, add ``--save`` to write it to the station:

.. literalinclude:: ../../examples/station.sh
   :language: bash
   :start-after: # start frame-set
   :end-before: # end frame-set

**What you should see**: the candidate ``task_frame`` and the following readings (actual output from synthetic data here):

.. literalinclude:: ../../examples/en/station-frame-output.txt
   :language: text
   :start-after: # start frame-set
   :end-before: # end frame-set

.. list-table::
   :header-rows: 1

   * - Reading
     - How to interpret it
   * - ``rpy_deg``
     - Table-frame roll / pitch / yaw relative to base (``base_R_table = Rz(yaw)·Ry(pitch)·Rx(roll)``). It should be near the nominal value: roll near ±45° for a 45° side mounting. A few degrees is normal mounting error; if it differs by over ten degrees, check for incorrectly chosen points first.
   * - ``axis_lengths_m``
     - Distances from the origin to the x and y points. Longer is more accurate; both should preferably exceed 0.2 m.
   * - ``y_axis_angle_deg``
     - How far the y point deviates from perpendicular to x (the y axis is automatically made perpendicular to x). For perpendicular table edges, this should be only a few degrees; a large value means the y point was misplaced.

With ``--save``, changes are shown first and written only after confirmation. ``workspace frame show`` displays the saved result and ``rpy_deg`` again.

**11.3 Frame first, profile second.** Once the frame is established, go to :doc:`spacemouse` to calibrate the orientation profile (``--frame table``):
the profile yaw then describes only how the SpaceMouse sits on the table, without mixing in the arm's mounting angle.
After changing ``task_frame``, redo profiles calibrated and points captured in the old frame.
If ``workspace.frame`` names this task frame (for example ``table``), the existing low / high bounds change
position and orientation in the base frame with the new frame. After saving, run ``rwrl workspace show`` and use
``rwrl workspace validate --pose X Y Z QX QY QZ QW`` to check the base-frame poses you intend to use
(position in m, quaternion xyzw; replace placeholders with actual readings). Recheck boundaries and the on-site motion range. ``workspace frame set`` reminds you through ``workspace_frame_rebased`` and
``workspace_note``. This chapter's ``workspace.frame: base`` and ``safety_box`` remain defined in the base frame;
they do not move with ``task_frame``, but you must still check the available range under the new teleop directions.
Finally, verify with single-axis pushes in step 4 of :doc:`spacemouse`: lifting the puck should move the TCP vertically upward; pushing forward should move it forward along the table.

**If it doesn't**:

* ``origin, x-axis and y-axis points are collinear``: move the y point off the line along x.
* ``x-axis point is ... mm from the origin``: the x point is too close to the origin. Separate them by more than 20 cm.
* ``rpy_deg`` differs greatly from the nominal value: check whether x and y points were reversed (forward / left), and whether all three points have the same height.
* ``the station workspace is defined in frame ...``: the workspace uses a different frame. This chapter's workspace is ``base`` and is unaffected.
* An upright robot measures only a fraction of a degree of roll / pitch: still check yaw. Omit ``task_frame`` only when all input and base axes align.

The two configurations and table frame are now ready. In the next chapter, :doc:`controller`, choose and install the Franka controller;
then :doc:`spacemouse` uses ``teleop run`` to combine station and preset into a session configuration and begin take-over practice.

.. note::

   **What needs confirmation on hardware in this chapter**

   Hardware output in this chapter is illustrative and follows the CLI format. FCI and SDK handshakes, real device feedback,
   gripper travel and opening/closing, camera modes and images, home and homing paths, workspace and per-step caps,
   action directions and magnitudes, 10 Hz control timing and 0.5 s settling time all need on-site confirmation.
   ``workspace show`` only displays configuration. It does not validate site geometry, collisions or a live daemon.
   Nominal and measured table-frame angles and single-axis directions also need on-site confirmation.
