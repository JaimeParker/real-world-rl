Choose and configure a Franka controller
================================================================================

This chapter chooses a Cartesian controller for a \ **newly connected**\ Franka, installs its software on the robot host (NUC),
writes the corresponding station fields, then checks it with doctor and one homing motion.
Complete steps 1–3 of :doc:`station` first (the station loads and doctor can connect to the arm).
Run every command in this chapter from the repository root on the NUC, with the project environment active.

``cartesian_controller`` is a station field that determines \ **what turns end-effector targets into motor torques**\ .
At each step, the policy or teleop supplies an end-effector target (position + orientation). The controller makes the arm follow it.

.. tip::

   Two independent switches are easy to confuse:

   * The station's ``cartesian_controller``: how the **robot side**\ executes targets (this chapter).
   * ``teleop run --mode tracker|serl``: how the **SpaceMouse side**\ turns puck input into targets (see :doc:`spacemouse`).

   SpaceMouse readings start in the table frame, rotate into the robot base frame, and only then reach this chapter's controller.
   Step 11 of :doc:`station` explains this chain.

1. Learn the three controllers and choose one
--------------------------------------------------------------------------------

.. list-table::
   :header-rows: 1

   * - ``cartesian_controller``
     - What executes targets
     - What to install on the robot host
     - Suitable for
   * - ``impedance`` (default)
     - Franky's torque-level impedance tracker pulls the TCP toward the target like a spring; gains come from the station's ``control:``
     - An official franky wheel matching the robot; also runs on a regular kernel
     - Connecting a robot for the first time and getting the other parts working
   * - ``internal``
     - The robot's built-in controller. Each target becomes a base-frame velocity motion, with velocity calculated as current error divided by 0.2 s,
       capped at 0.25 m/s and 1.0 rad/s. With no new target for 300 ms, it decelerates to a stop
     - Official franky wheel + PREEMPT_RT kernel
     - Letting the robot handle IK and payload
   * - ``serl``
     - SERL's original Cartesian impedance controller (ported line by line from ``serl_franka_controllers``, with bit-identical results);
       ``move_to_joints`` also becomes SERL's 10 s joint reset; gains come from the station's ``serl:``
     - Franky + ``serl_controller`` \ **built together from source**\ on this machine; PREEMPT_RT recommended
     - Reproducing the SERL / HIL-SERL operating feel

In all three modes, joint servo uses franky's joint impedance tracker, and ``move_to_pose`` uses a Ruckig trajectory.

Combinations with the two SpaceMouse modes:

.. list-table::
   :header-rows: 1

   * - Controller \\ SpaceMouse
     - ``--mode tracker`` (default)
     - ``--mode serl``
   * - ``impedance``
     - The default combination in :doc:`spacemouse`; press button_0 to take over
     - Supported; puck motion takes over, but execution does not use the SERL controller
   * - ``internal``
     - Supported
     - Supported
   * - ``serl``
     - Supported; recommended for the first motion
     - Closest to HIL-SERL: puck motion takes over, and the SERL controller executes

If unsure, start with ``impedance``. It requires no extra build, and switching controllers later changes only one station field.

**What you should see**: you have chosen a controller for this robot.

**If it doesn't**: if you are unsure whether the kernel is realtime or whether you need ``serl``, complete this chapter and :doc:`spacemouse` with ``impedance`` first.
Return to steps 3 and 4 of this chapter to switch later.

2. Record three facts about this robot
--------------------------------------------------------------------------------

These three facts determine the installation and configuration below. Write them down first.

**Model** ``robot_model``. Open Settings → System in Desk and read the system version:

.. list-table::
   :header-rows: 1

   * - Desk system version
     - Robot
     - Enter
   * - 4.x
     - Franka Emika Robot (FER, often called Panda)
     - ``robot_model: panda``
   * - 5.x
     - Franka Research 3
     - ``robot_model: fr3``

This determines the default ``joint_limits`` from ``rwrl setup`` (the manual's Panda and FR3 limits differ greatly at joints 6 and 7),
and selects the limit table for SERL's joint reset.

**robot server version**: the communication-protocol version on the controller-box side. It determines which libfranka versions work.
The most reliable way to read it is to let doctor perform a handshake:

.. literalinclude:: ../../examples/controller.sh
   :language: bash
   :start-after: # start server-version
   :end-before: # end server-version

If the libfranka bundled with the installed franky does not match the robot, the ``handshake`` row reports both versions:

.. literalinclude:: ../../examples/en/controller-output.txt
   :language: text
   :start-after: # start mismatch
   :end-before: # end mismatch

The 9 in ``robot server 9`` is the number to record. If the handshake \ **passes**\ , the installed franky's libfranka is compatible.
Check its version:

.. tutorial-test: skip
.. literalinclude:: ../../examples/controller.sh
   :language: bash
   :start-after: # start franky-version
   :end-before: # end franky-version

``2.0.0+libfranka.0.17.0`` means libfranka 0.17.0. The default official v2.0.0 wheel has version ``2.0.0`` and uses libfranka 0.21.2.
For custom builds or other franky versions, the absence of a suffix alone does not identify the libfranka version.
Use the libfranka version to look up the server version in the table below.

**libfranka version**: look it up by server version below. The table comes from franky v2.0.0's
`doc/compatibility.json <https://github.com/TimSchneider42/franky/blob/v2.0.0/doc/compatibility.json>`_
(the minimum libfranka required for each server version, from Franka's
`compatibility matrix <https://frankarobotics.github.io/docs/compatibility.html>`_ and libfranka-common's version definitions).
The “recommended version” is the libfranka used by franky's published wheel for that server version:

.. list-table::
   :header-rows: 1

   * - robot server version
     - Desk system version (reference)
     - Compatible libfranka
     - Recommended ``LIBFRANKA_VERSION``
   * - 10
     - 5.9.0 onward (FR3)
     - ≥ 0.18.0
     - 0.21.2
   * - 9
     - 5.7.2, among others (FR3)
     - 0.15.0 – 0.17.x
     - 0.17.0 (fully verified for this repository on a clean Ubuntu 22.04)
   * - 8
     - 5.x(FR3)
     - 0.14.1 – 0.14.x
     - 0.14.2
   * - 7
     - 5.x(FR3)
     - 0.13.3 – 0.14.0
     - 0.13.3
   * - 6
     - 5.2.0 onward (FR3)
     - 0.10.0 – 0.13.2
     - 0.12.1
   * - 5
     - 4.2.x (the final FER release)
     - 0.9.1 – 0.9.x
     - 0.9.2
   * - 4
     - 4.x(FER)
     - 0.8.0 – 0.9.0
     - 0.8.0
   * - ≤ 3
     - Earlier FER
     - < 0.8
     - ``serl`` unsupported (requires libfranka ≥ 0.8)

Use the handshake report as the authority for server version. The “Desk system version” column is only a cross-check; see the compatibility matrix above for exact mappings.
Server version ≥ 6 means a 5.x system, hence FR3; ≤ 5 means FER. If this disagrees with the model read in Desk, use Desk as the authority and recheck.

**Whether the kernel is realtime**:

.. tutorial-test: skip
.. literalinclude:: ../../examples/controller.sh
   :language: bash
   :start-after: # start rt-check
   :end-before: # end rt-check

**What you should see**: three recorded facts, for example “fr3; server 9 → libfranka 0.17.0; kernel is not PREEMPT_RT”.

**If it doesn't**:

* Doctor's ``handshake`` cannot connect (rather than reporting a version mismatch): return to step 3 of :doc:`station` to fix networking, FCI and control ownership.
* The model inferred from the handshake's server version disagrees with Desk: confirm in Desk first. Do not guess the model from appearance.

3. Install a realtime kernel (needed by internal and serl)
--------------------------------------------------------------------------------

Both ``internal``'s velocity interface and ``serl``'s 1 kHz torque loop are sensitive to delays. A regular kernel occasionally delays a control cycle,
causing a communication reflex that stops the robot. The original SERL ran with ``realtime_config: enforce``.
Skip this step for ``impedance``.

If the previous check did not show PREEMPT_RT, follow Franka's official documentation,
`Setting up the real-time kernel <https://frankarobotics.github.io/docs/installation_linux.html#setting-up-the-real-time-kernel>`_
to install a PREEMPT_RT kernel (Ubuntu 22.04 / 24.04 can also use Ubuntu Pro's ``sudo pro enable realtime-kernel``). Reboot into the new kernel.
Then grant the current user realtime priority, log in again and rerun the checks from step 2:

.. tutorial-test: skip
.. literalinclude:: ../../examples/controller.sh
   :language: bash
   :start-after: # start rt-limits
   :end-before: # end rt-limits

**What you should see**: ``uname -v`` contains ``PREEMPT_RT``, ``/sys/kernel/realtime`` is 1, and ``ulimit -r`` is 99.

**If it doesn't**:

* ``ulimit -r`` is still 0: confirm that you logged in again and that ``id`` includes the ``realtime`` group.
* NVIDIA drivers will not install on the realtime kernel: see franky's FAQ (``IGNORE_PREEMPT_RT_PRESENCE=1``), or move GPU programs to another machine.

4. Install the controller's software
--------------------------------------------------------------------------------

.. important::

   **Installation order.** The script in this step builds the franky required by ``serl`` from source. It and ``serl_controller`` form \ **a pair**\ .
   When using ``serl``, \ **do not**\ run ``rwrl doctor --install-sdk`` or ``pip install franky-control``:
   they replace franky with an official wheel and break the pair (when the station already has ``cartesian_controller: serl``,
   doctor refuses ``--install-sdk`` and supplies the rebuild command).
   It is fine to have installed an official wheel before running the script; the script uninstalls it first.

4A. impedance and internal: official franky wheel
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The ``franky`` extra in :doc:`install` does not pin a version. The default PyPI build of franky v2.0.0 uses
libfranka 0.21.2 (server 10); check the compatibility table for other versions first. If step 2's handshake passed, nothing is needed.
If the handshake reports a version mismatch, let doctor replace the wheel based on the server version it read:

.. literalinclude:: ../../examples/controller.sh
   :language: bash
   :start-after: # start install-wheel
   :end-before: # end install-wheel

**What you should see**: doctor lists ``PLAN sdk:franky`` and a command such as ``pip install franky-control --extra-index-url .../by-robot-server-version/9/``,
then installs after confirmation. ``handshake`` passes when you rerun ``rwrl doctor``.

**If it doesn't**:

* ``sdk.franky.serl`` is BLOCKED: the station already selects ``serl``. Build as in 4B; do not use ``--install-sdk``.
* The plan includes ``Advice: serl_controller is installed ...``: this machine had the serl pair installed. Replacing it with an official wheel makes ``serl`` unavailable;
  redo 4B when you want to use ``serl`` again.

4B. serl: build franky and serl_controller together on the robot host
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

``serl_controller`` is a C++ extension. Its motion classes directly inherit franky's C++ classes and link to franky's ``libfranky.so`` and libfranka.
It therefore works only with franky from \ **the same build and the same libfranka**\ , not the official franky wheels on PyPI / GitHub.
The repository's script performs the entire build on the NUC's own Ubuntu installation, without Docker:

1. Install build tools and libraries with apt (``build-essential``, ``cmake``, ``libboost-all-dev``, etc.; requires sudo);
2. Install build tools in the current environment (CMake ≥ 3.24, scikit-build-core, pybind11 3.0.4, matching franky v2.0.0);
3. Download franky v2.0.0 source, check the commit and confirm it is unmodified;
4. Use franky's own dependency script to build and install Eigen, pybind11, ruckig and the specified libfranka version (also pinocchio from 0.14 onward) into ``/usr/local``;
   skip libfranka if the same version is already installed;
5. Install unmodified franky from source into the current environment, with ``+libfranka.X`` in its version;
6. Build and install ``serl_controller`` with the same compiler and franky;
7. Run C++ tests (including golden tests comparing each tick against original SERL), Python binding tests and import checks, then doctor's pairing check.

Set ``LIBFRANKA_VERSION`` to the version found in step 2 and run (the example is server 9 → 0.17.0):

.. tutorial-test: skip
.. literalinclude:: ../../examples/controller.sh
   :language: bash
   :start-after: # start install-native
   :end-before: # end install-native

.. list-table::
   :header-rows: 1

   * - Environment variable
     - What to enter
   * - ``LIBFRANKA_VERSION``
     - Required; the “recommended version” in step 2's table.
   * - ``WORK``
     - Location for source, build directories and logs. Defaults to ``~/.cache/rwrl-native-build``.
   * - ``REPLACE_LIBFRANKA=1``
     - The script stops by default if ``/usr/local`` already contains \ **another**\ libfranka version (replacing it affects all programs using libfranka on this computer). Set only after confirming replacement.
   * - ``SKIP_APT=1``  /  ``SKIP_DEPS=1``
     - Skip the corresponding step if you already installed the apt packages or C++ dependencies yourself.
   * - ``PIP_INDEX_URL``
     - Switch to a mirror if PyPI is slow, for example ``https://mirrors.aliyun.com/pypi/simple/``.

A full installation from scratch takes about 11 minutes on a 64-core machine. Expect 30–60 minutes on a typical NUC (4–8 cores), mostly compiling pinocchio (needed only for libfranka ≥ 0.14).
Logs are in ``$WORK/logs/``, or ``~/.cache/rwrl-native-build/logs/`` when ``WORK`` is unset.
The script targets Ubuntu (20.04 / 22.04 / 24.04). On other distributions it prints a warning, with no guarantee that the build works.
It requires an active project environment. The project requires Python ≥ 3.10;
Ubuntu 20.04 ships with 3.8, so install 3.10 separately first and create the environment with it.

**What you should see**: the script ends with the following lines (versions, paths and build id depend on your machine):

.. literalinclude:: ../../examples/en/controller-output.txt
   :language: text
   :start-after: # start native
   :end-before: # end native

Passing ``golden_equivalence`` and ``joint_reset_golden`` means the ported control laws match original SERL at every tick on this machine with this compiler;
the final ``PASS  sdk:serl_controller`` is doctor's pairing check. You can confirm again at any time:

.. tutorial-test: skip
.. literalinclude:: ../../examples/controller.sh
   :language: bash
   :start-after: # start install-native-check
   :end-before: # end install-native-check

In ``build_info``, ``franky_layout`` is ``source``, and ``libfranka_path`` points to the system libfranka installed by the script.

**If it doesn't**:

* ``attempt 1/3 failed (network?)``: the GitHub connection is unstable. The script retries automatically; if all attempts fail, set ``RETRIES=6`` and rerun. Completed steps will be quick.
* ``activate the project venv / conda env first``: run ``source .venv/bin/activate`` first. Do not install into system Python.
* ``libfranka 0.x is installed; installing ... replaces it``: another libfranka is already installed (for example for ROS). Confirm it can be replaced before setting ``REPLACE_LIBFRANKA=1``.
* A step fails: the script prints the last few dozen lines of that step's log. The full log is in the corresponding file under ``logs/``.
* A later ``pip install franky-control`` or ``--install-sdk`` replaces franky: ``sdk:serl_controller`` will FAIL. Rerun this step's script.
* libfranka is reinstalled or upgraded: ``sdk:serl_controller`` reports ``changed since serl_controller was built``. Rerun this step's script.

The franky in this pair is unmodified v2.0.0. ``impedance`` and ``internal`` still work normally, so switching controllers later needs no reinstall.

.. note::

   **Optional: build wheels elsewhere.** For several identical NUCs (same Ubuntu, Python and libfranka), use the repository's Docker recipe on a
   build machine with Docker to build a pair of manylinux wheels. Copy the entire directory to each NUC and install them together. The NUC itself does not need Docker:

   .. tutorial-test: skip
   .. literalinclude:: ../../examples/controller.sh
      :language: bash
      :start-after: # start wheels-docker
      :end-before: # end wheels-docker

   .. tutorial-test: skip
   .. literalinclude:: ../../examples/controller.sh
      :language: bash
      :start-after: # start wheels-install
      :end-before: # end wheels-install

   This wheel pair bundles libfranka and must be installed together. The Python version must match the wheels' cp tag (default cp310).

5. Edit the station: controller, model and realtime
--------------------------------------------------------------------------------

Back up before editing so rollback is a simple copy:

.. tutorial-test: skip
.. literalinclude:: ../../examples/controller.sh
   :language: bash
   :start-after: # start backup
   :end-before: # end backup

Edit the top level of ``local/station.yaml`` (alongside ``robot_ip``) for your chosen controller. The three common fields are:

.. list-table::
   :header-rows: 1

   * - Field
     - How to obtain the value
   * - ``cartesian_controller``
     - Your choice from step 1: ``impedance`` | ``internal`` | ``serl``.
   * - ``robot_model``
     - The Desk result from step 2: ``panda`` for 4.x, ``fr3`` for 5.x.
   * - ``realtime``
     - Set ``enforce`` only after step 3 passes. On a kernel without PREEMPT_RT, ``enforce`` makes the connection fail.
       Use ``enforce`` for ``internal`` and ``serl``; use ``ignore`` for ``impedance`` on a regular kernel.

You can also let the wizard rewrite these fields. It preserves other station values and backs up the old file as ``station.yaml.bak.<UTC>``:

.. literalinclude:: ../../examples/controller.sh
   :language: bash
   :start-after: # start setup-switch
   :end-before: # end setup-switch

The wizard asks for the model first and uses it for default ``joint_limits``. It suggests ``realtime: enforce`` for ``internal`` / ``serl``,
warns if the kernel is not PREEMPT_RT, and removes impedance-only Cartesian gains from ``control:`` when switching to ``serl`` (keeping joint fields).
Existing ``serl:`` parameters are preserved; if unset, SERL defaults apply. For manual editing, follow one of the three subsections below.

The preset's ``real_robot.control`` can override station ``control`` per key. Omitted keys retain station values.
If the station sets ``max_delta_tau``, ``translational_error_clip`` or ``rotational_error_clip``,
the preset may only keep or tighten these caps (elementwise for vectors), never loosen them. Violations raise ``SafetyViolation``.
Stiffness and damping must still meet the backend's supported fields, dimensions and value constraints. The environment applies configuration during reset's IDLE phase;
it cannot switch controllers during motion. ``cartesian_controller`` is still selected in the station.
SERL Cartesian parameters use ``serl`` / ``serl_profiles``, not these Cartesian tracker gains.

5A. impedance
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. literalinclude:: ../../examples/en/controller-impedance.yaml
   :language: yaml
   :start-after: # start controller
   :end-before: # end controller

Write the following Cartesian gains under ``control:``. They apply only to ``impedance`` Cartesian servo. Use this set for a new arm (it is what ``rwrl setup`` writes):

.. literalinclude:: ../../examples/en/controller-impedance.yaml
   :language: yaml
   :start-after: # start control
   :end-before: # end control

.. list-table::
   :header-rows: 1

   * - Field
     - Meaning
   * - ``translational_stiffness``  /  ``rotational_stiffness``
     - Translational (N/m) / rotational (Nm/rad) stiffness. Franky's built-in 500 N/m is too soft to overcome joint static friction, causing lag and drift.
   * - ``cartesian_damping``
     - Six damping values: translation first, then rotation.
   * - ``translational_error_clip``  /  ``rotational_error_clip``
     - Error caps. Each translational spring-axis term is bounded by stiffness × cap: 2000 × 0.01 = 20 N;
       this excludes damping and other terms, so it is not a safety bound on total contact force.

These values approximate SERL's default impedance (rotation-error definitions differ and agree only at small angles). They are not the SERL controller itself.

5B. internal
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. literalinclude:: ../../examples/en/controller-internal.yaml
   :language: yaml
   :start-after: # start controller
   :end-before: # end controller

``internal``'s Cartesian velocity interface has no adjustable gains. Error divided by a fixed 0.2 s gives the requested velocity,
then it is capped at 0.25 m/s / 1.0 rad/s. The 0.2 s is not a guaranteed arrival time. With no new target for 300 ms, it decelerates to a stop.
Joint fields in ``control:`` still apply to joint servo; ``relative_dynamics_factor`` scales ``move_to_*`` speed.
``internal`` has not yet passed hardware acceptance. Keep your hand on the emergency stop on first use.

5C. serl
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. literalinclude:: ../../examples/en/controller-serl.yaml
   :language: yaml
   :start-after: # start controller
   :end-before: # end controller

``serl:`` contains the SERL controller's gains and limits. Keys, defaults and ranges exactly match SERL's ``compliance_param.cfg``.
Use ``null`` for the first hardware run:

.. literalinclude:: ../../examples/en/controller-serl.yaml
   :language: yaml
   :start-after: # start serl-null
   :end-before: # end serl-null

To change individual values, write only those keys. Written out in full, they look like this (all are defaults):

.. literalinclude:: ../../examples/en/controller-serl-params.yaml
   :language: yaml
   :start-after: # start serl
   :end-before: # end serl

.. list-table::
   :header-rows: 1

   * - Field
     - Meaning (range)
   * - ``translational_stiffness`` / ``translational_damping``
     - Translational stiffness N/m (0–6000) / damping (0–400)
   * - ``rotational_stiffness`` / ``rotational_damping``
     - Rotational stiffness (0–300) / damping (0–30)
   * - ``nullspace_stiffness`` / ``joint1_nullspace_stiffness``
     - Null-space stiffness holding the starting pose (0–100), with a separate term for joint 1
   * - ``translational_clip_{x,y,z}`` / ``translational_clip_neg_{x,y,z}``
     - Positive / negative translational-error caps per axis, m (0–0.1). Stiffness × cap bounds that axis's spring term, default 2000 × 0.01 = 20 N; it does not bound total contact force
   * - ``rotational_clip_*``
     - Positive / negative rotation-error caps (quaternion vector part), (0–0.1)
   * - ``translational_Ki`` / ``rotational_Ki``
     - Integral gains (0–100), default 0

Out-of-range values are \ **clipped into range**\ with a warning (as in SERL's dynamic_reconfigure). Unknown keys or nonnumeric values fail immediately.
The impedance Cartesian gains in ``control:`` (``translational_stiffness``, ``rotational_stiffness``, ``cartesian_damping``,
``translational_error_clip``, ``rotational_error_clip``) have no effect under ``serl``. Doctor reports ``config:serl_control`` WARN;
remove them and keep the joint fields.

**Automatic recovery.** Pushing into an obstacle, reaching a collision threshold and similar events trigger a robot reflex, stopping the SERL controller.
HIL-SERL calls ``/clearerr`` before every ``/pose``, so the operator does not feel the interruption.
``serl_auto_recover`` does the same: when the next servo target arrives, it clears the error, creates a \ **new**\ SERL controller at the measured pose, then executes that target:

.. literalinclude:: ../../examples/en/controller-serl.yaml
   :language: yaml
   :start-after: # start recover
   :end-before: # end recover

``enabled`` defaults to ``true``. Set ``false`` to stop for manual confirmation after every reflex. ``max_per_episode`` limits automatic recoveries per episode (default 5; 0 disables them).
Beyond that count, servo reports a recoverable error requiring ``rwrl recover``. A new controller ramps stiffness and damping from 0 over about 0.2 s, so it briefly feels softer at the start. This is SERL's original behavior on every controller restart.

**Payload**\ (applies to all three controllers). ``load`` is sent to the robot after every connection, before any motion (libfranka's ``setLoad``):

.. literalinclude:: ../../examples/en/controller-serl.yaml
   :language: yaml
   :start-after: # start load
   :end-before: # end load

Set it only for items attached to the gripper that Desk has not registered, using the format below. A robot rejection makes ``connect`` fail:

.. literalinclude:: ../../examples/en/controller-serl-params.yaml
   :language: yaml
   :start-after: # start load
   :end-before: # end load

**What you should see**: the station loads (doctor checks this next), and the three common fields match your notes.

**If it doesn't**:

* ``cartesian_controller: serl needs arm_backend franky``: real control uses ``arm_backend: franky``. The configuration also permits ``fake`` for mock checks.
* ``robot_model 'x' invalid``: only ``panda`` or ``fr3`` is allowed.
* Existing ``joint_limits`` do not change automatically when you change ``robot_model``. For a new station, remove this field and rewrite with ``rwrl setup --station local/station.yaml``; the wizard supplies model-specific defaults.

6. Run doctor
--------------------------------------------------------------------------------

Acquire control in Desk, activate FCI, then run:

.. literalinclude:: ../../examples/controller.sh
   :language: bash
   :start-after: # start doctor
   :end-before: # end doctor

**What you should see**: in addition to the rows in :doc:`station`, the following rows appear and all show PASS.
This example follows the source code's output format. Use your own report on site:

.. literalinclude:: ../../examples/en/controller-output.txt
   :language: text
   :start-after: # start doctor
   :end-before: # end doctor

.. list-table::
   :header-rows: 1

   * - Row
     - What it checks
   * - ``sdk:serl_controller`` (serl)
     - Whether franky and serl_controller come from the same build: without importing the extension, compare ``build_info.json`` against installed franky's ``libfranky.so`` fingerprint and libfranka
   * - ``config:serl_control`` (serl)
     - Whether ``control:`` contains Cartesian gains ignored in serl mode
   * - ``mode:serl_realtime``  /  ``mode:internal_realtime``
     - Whether ``realtime: enforce`` is set for ``serl`` / ``internal``
   * - ``mode:realtime_kernel``
     - Whether the kernel is PREEMPT_RT; FAIL for ``enforce`` on a non-realtime kernel; kernel status is still reported under ``ignore``
   * - ``mode:robot_model``
     - Whether station ``robot_model`` matches the model reported by the robot (joint names in the robot-supplied URDF)

**If it doesn't**:

.. literalinclude:: ../../examples/en/controller-output.txt
   :language: text
   :start-after: # start doctor-warn
   :end-before: # end doctor-warn

* ``sdk:serl_controller`` FAIL: the pair does not come from the same build. Return to step 4B and rerun the script.
* ``mode:serl_realtime`` / ``mode:internal_realtime`` WARN: return to step 3. You may ignore it for a temporary test, but expect occasional communication reflexes.
* ``mode:robot_model`` WARN: confirm in Desk as in step 2, then change ``robot_model``.
* ``handshake`` reports libfranka mismatch: the server version in step 2 was wrong. For ``serl``, rerun 4B with the correct version; for other controllers, use 4A.

7. First motion: home
--------------------------------------------------------------------------------

Start with joint motion only. ``rwrl home go`` moves the arm to the station's ``home_joints``.
``impedance`` / ``internal`` use Ruckig trajectories; ``serl`` uses SERL's joint reset: 10 s of linear interpolation to home, slower than Ruckig and along a different path.

.. warning::

   This step moves the arm. Clear the path, check home (step 4 of :doc:`station`), keep your hand on the emergency stop and maintain on-site supervision.

.. literalinclude:: ../../examples/controller.sh
   :language: bash
   :start-after: # start home
   :end-before: # end home

``home show`` is read-only and prints home and current joint differences. ``home go`` shows the target and maximum joint difference first, and moves only after confirmation.
It performs only reset's home phase: blocking joint homing, followed by the preset's
``task.reset.gripper`` action, ``open`` / ``close``. ``keep`` or an unset value leaves the gripper unchanged.
It skips lift, waypoints and settling waits, so this check does not replace full reset acceptance.
If the preset has its own home, it overrides the station. Check the source in ``home show`` first.

.. warning::

   Homing may also open or close the gripper. Clear the homing path and secure objects that could fall.
   ``home go`` and ``gripper open/close/move`` move only after ``y`` (case-insensitive);
   other answers or end of input cancel and exit 2. Without a terminal, motion is refused with ``refusing to move without --yes``, also exiting 2.
   Global ``--yes`` before the command skips the prompt. Use it only after reviewing the target on site.
   With a direct connection, ``rwrl stop`` does not prompt but still needs an FCI connection. A second connection cannot stop an arm held by a recorder or actor.
   End the process holding the connection first; use the physical stop device in an emergency.

**What you should see**: under ``serl``, the arm stops at home after about 10–12 s, with ``max delta`` below 0.01 rad. Other controllers arrive within a few seconds:

.. literalinclude:: ../../examples/en/controller-output.txt
   :language: text
   :start-after: # start home
   :end-before: # end home

**If it doesn't**:

* The log contains ``joint reset TIMEOUT``: the joints did not reach near home (about 0.01 rad per joint) within about 30 s. Reset still ends; check for obstructions or home near a limit.
* A reflex such as ``communication_constraints_violation`` appears immediately: return to step 3 and confirm the realtime kernel.
* ``outside joint_limits``: home lies outside the station's joint limits. Recheck the model and ``joint_limits`` in step 5.

8. Next: first Cartesian motion and reflex recovery
--------------------------------------------------------------------------------

Cartesian motion needs someone to supply targets, so do it in :doc:`spacemouse`. First establish the table frame in step 11 of :doc:`station`,
then use tracker mode in step 4 of :doc:`spacemouse` to push gently about 1 cm in one direction. The controller does not change direction; check the frame and orientation profile if the direction is wrong.

For ``serl``, return here after that step to check automatic recovery. While controlling the arm, gently push the end effector toward the table or a soft object until the robot triggers a reflex.

**What you should see**: one WARNING line in the log, and the robot resumes from its current pose on the next puck push:

.. literalinclude:: ../../examples/en/controller-output.txt
   :language: text
   :start-after: # start recover-log
   :end-before: # end recover-log

**If it doesn't**: if ``max_per_episode`` is exceeded or the error cannot be cleared, address the Desk and site conditions, then run:

.. literalinclude:: ../../examples/controller.sh
   :language: bash
   :start-after: # start recover
   :end-before: # end recover

After ``rwrl recover`` succeeds, the next servo target starts a new SERL controller.

If the task needs different stiffness or limits, write named profiles under the preset's ``real_robot:`` (see ``docs/examples/controller-task.yaml`` for the complete file).
Switching profiles makes a \ **partial update**\ to current parameters. Keys omitted from the profile keep their previous values, as with HIL-SERL's ``update_param``.
The environment does not switch them automatically. Task code calls ``arm.update_serl_params`` when needed:

.. literalinclude:: ../../examples/en/controller-task.yaml
   :language: yaml
   :start-after: # start profiles
   :end-before: # end profiles
   :dedent: 2

9. Switching and rollback
--------------------------------------------------------------------------------

Switch controllers by editing only the station (restart the daemon if it is running). To restore the previous behavior, copy back the backup and check again:

.. tutorial-test: skip
.. literalinclude:: ../../examples/controller.sh
   :language: bash
   :lines: 47-47

.. literalinclude:: ../../examples/controller.sh
   :language: bash
   :lines: 48-48

**What you should see**: doctor's results return to their state before the change.

**If it doesn't**: resuming a saved experiment recorded with another controller is intentionally rejected because control metadata differs;
start a new experiment. Do not mix data from the two controllers.

You can keep the source-built franky: it is unmodified v2.0.0 and supports all three controllers. Only if you want to return to an official wheel,
first change the station to ``impedance`` or ``internal``, then run ``rwrl doctor --install-sdk``. Afterwards ``serl`` is unavailable until you rerun 4B.

.. note::

   **What needs confirmation on hardware in this chapter**

   The realtime kernel and jitter, Desk model, SERL joint-reset path, first Cartesian motion's direction and lag, and automatic recovery after a reflex all need on-site confirmation.
   The installation script has run completely on a clean Ubuntu 22.04 (Python 3.10, libfranka 0.17.0), but without a real robot connected.
   For ordered hardware acceptance steps and pass criteria, see
   :doc:`../reference/serl-robot-validation`;
   For deeper reference, see :doc:`../reference/serl-controller`.

Checklist: know which controller you chose and why; server version matches libfranka; relevant doctor rows pass; homing works; know how to roll back.

Continue to :doc:`spacemouse`; GELLO users should see :doc:`gello`.
