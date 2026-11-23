Joint controller (``joint_controller: trajectory_impedance``)
================================================================================

rwrl can execute joint servo targets with a Franka joint controller written for GELLO teleoperation. ``native/joint_controller/`` is a C++/pybind11 package
whose motion class derives from franky's torque motion class, and franky itself is unmodified, as with ``native/serl_controller``.
franky's own joint motions step to each target; this motion turns a stream of sparse targets into a continuous reference trajectory (continuous position, velocity and acceleration, within velocity and acceleration limits)
and tracks it with GELLO's joint impedance law. See `native/joint_controller/README.md <https://github.com/JaimeParker/real-world-rl/blob/main/native/joint_controller/README.md>`__\ .

Reference versions: the impedance law, its velocity filter and the default gains come from the public GELLO controller (``wuphilipp/gello_software`` @ ``9fc8c26``, Apache-2.0, Copyright (c) 2025 Franka Robotics GmbH);
the joint position limits are franka_description's (``franka_ros`` @ ``ddd2fff``, Apache-2.0); the motion runs inside unmodified `franky <https://github.com/TimSchneider42/franky>`__ (MIT).
The reference generator (the quintic planner) is this repository's own. See \ :ref:`Provenance and licences <en-joint-controller-provenance-and-licences>`\ .

This page is a reference. Choosing, installing and first-checking it on a new robot is written as steps in step 10 of :doc:`../tutorials/controller`; for GELLO itself see :doc:`gello` and :doc:`../tutorials/gello`.

Choosing the controller
--------------------------------------------------------------------------------

Joint servo is decided by the station's ``joint_controller``. It is independent of ``cartesian_controller``, and the two can be combined freely:

.. list-table::
   :header-rows: 1

   * - ``joint_controller``
     - What executes joint servo
     - What to install
   * - ``impedance`` (default)
     - franky's ``JointImpedanceTracker``, which tracks each target directly as it arrives at ``control_hz``; gains come from ``control:``
     - The official franky wheel is enough
   * - ``trajectory_impedance``
     - The controller of this page: each new target becomes a 1 kHz reference trajectory through a quintic planner, tracked with GELLO's joint impedance law
     - franky, ``serl_controller`` and ``joint_controller`` from one build; needs ``arm_backend: franky``

``move_to_joints`` (homing) is unaffected: it is still SERL's 10 s joint reset with ``cartesian_controller: serl`` and franky's ``JointMotion`` otherwise.

.. code-block:: yaml

   # station (robot PC), e.g. local/station.yaml; restart `rwrl daemon` afterwards
   joint_controller: trajectory_impedance   # impedance | trajectory_impedance
   realtime: enforce                        # PREEMPT_RT kernel recommended
   joint_trajectory: {}                     # optional overrides, see Parameters

Why use it for GELLO: the default ``impedance`` receives a step target every ``control_hz`` period; ``trajectory_impedance`` fills in continuous position, velocity and acceleration between targets, so the follower moves more smoothly.
For GELLO collection, raise ``real_robot.control_hz`` to 15–30 Hz: the shorter the interval (100 ms at 10 Hz), the smaller each replan's displacement and the shorter the delay from leader reading to execution.
This advice follows from the planner's behaviour and is not a measured result. Per-step limits (``max_delta_per_step.joint_rad``, ``joint_scale_rad``) are per step, so after raising ``control_hz`` the same per-step values mean a higher speed;
the actual speed is also limited by ``max_velocity`` in ``joint_trajectory``. ``control_hz`` is part of the experiment contract: collection, training and deployment must match.

.. _en-joint-controller-installing:

Installation
--------------------------------------------------------------------------------

``joint_controller`` derives from franky's C++ classes and the pybind11 class ``franky.BaseTorqueMotion``, with the same coupling rule as ``serl_controller``:
**franky, joint_controller and serl_controller must come from one build** (same machine or container, compiler, pybind11 3.0.4, libfranka and franky headers) and be deployed together.
Never install it next to a franky from elsewhere (including the official PyPI wheel).

* **Build directly on the robot host** (no Docker needed), with the project environment active:

  .. tutorial-test: skip
  .. code-block:: bash

     LIBFRANKA_VERSION=0.17.0 bash native/joint_controller/scripts/install-on-ubuntu.sh

  If the environment already has franky v2.0.0 built from source against the same libfranka with the same ``WORK`` (default ``~/.cache/rwrl-native-build``), the script reuses it;
  otherwise it first runs ``native/serl_controller/scripts/install-on-ubuntu.sh`` (which builds libfranka and franky and rebuilds ``serl_controller``), then pip-installs ``joint_controller`` with ``--no-build-isolation``,
  and runs CTest (including the franky motion test), the Python tests, an import check and a pairing check of ``build_info.json``'s ``libfranky.so`` sha256 against the installed file.
  ``REBUILD_FRANKY=1`` forces the franky step again; logs are in ``$WORK/logs/jc_*``.

* **Build the three wheels once with Docker** (for identical robot hosts):

  .. tutorial-test: skip
  .. code-block:: bash

     WORK=$HOME/rwrl-franky-build native/joint_controller/docker/build-on-host.sh
     # $WORK/out/wheels: franky_control-*.whl, serl_controller-*.whl, joint_controller-*.whl, SHA256SUMS
     pip uninstall -y franky-control serl-controller joint-controller
     pip install franky_control-*.whl serl_controller-*.whl joint_controller-*.whl

  ``WITH_SERL=0`` builds only franky and joint_controller.

In both ways ``build_info.json`` has ``joint_controller_version`` and ``gello_commit``. After installing ``joint_controller``, do not run ``rwrl doctor --install-sdk`` or ``pip install franky-control``:
they would replace the pair. When the station already says ``joint_controller: trajectory_impedance``, doctor refuses ``--install-sdk`` and reports BLOCKED.

Realtime kernel
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The control loop is a 1 kHz torque loop; as with ``serl``, a PREEMPT_RT kernel with ``realtime: enforce`` is recommended (installation: step 3 of :doc:`../tutorials/controller`).
``rwrl setup --joint-controller trajectory_impedance`` suggests ``enforce``; doctor gives a WARN, not a failure, when it is not ``enforce``.

Doctor checks
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

With ``joint_controller: trajectory_impedance``, ``rwrl doctor`` adds these rows for the robot (see the \ :ref:`doctor reference <en-doctor-joint-controller-checks>`\ ):

.. list-table::
   :header-rows: 1

   * - Row
     - Layer
     - Result
     - Meaning and action
   * - ``sdk:joint_controller``
     - 1 sdk
     - PASS ``joint_controller franky build X; installed franky Y`` / FAIL
     - ``joint_controller`` imports and has ``TrajectoryImpedanceMotion``, and the franky version recorded in ``build_info`` (``franky_dist_version``) equals the installed franky's version. On FAIL, rerun the install script or reinstall the three wheels from one build directory.
   * - ``mode:joint_trajectory_realtime``
     - 5 mode
     - PASS / WARN ``joint_controller: trajectory_impedance with realtime: ...``
     - WARN when ``realtime`` is not ``enforce``. Install a PREEMPT_RT kernel and set ``realtime: enforce``.

How it runs
--------------------------------------------------------------------------------

.. list-table::
   :header-rows: 1

   * - rwrl call
     - Implementation
   * - ``set_control_mode(JOINT_SERVO)``
     - Builds a ``TrajectoryImpedanceMotion`` from the current ``joint_trajectory`` and calls ``robot.move(motion, asynchronous=True)``; the reference starts at the measured ``q``, at rest, so the spring term is 0 at tick 0
   * - ``servo_joints(q)``
     - Passes the arm's safety layer first (at most ``max_delta_per_step.joint_rad`` from the measured joints, then clipped into ``joint_limits``), then ``motion.set_target(q)``; a target with a non-finite value is rejected and the arm raises ``joint target rejected (non-finite)``
   * - ``update_joint_trajectory_params(partial)``
     - Merged and validated; with the controller running it takes effect on the next tick (see \ :ref:`live update <en-joint-controller-live-update>`\ )
   * - ``stop()``, leaving the mode
     - franky's torque stop; the next servo target restarts the controller (from the measured joints)
   * - Reflex (any motion error)
     - The next servo target clears the errors per ``serl_auto_recover`` and restarts the controller at the measured joints, at most ``max_per_episode`` times per episode; past that, or if the errors cannot be cleared, servo raises a recoverable error and ``rwrl recover`` is needed. An end caused by ``stop()`` is not a reflex and is not auto-recovered
   * - ``move_to_joints`` (IDLE)
     - Unchanged (franky ``JointMotion``, or SERL's joint reset with ``cartesian_controller: serl``)

The control law
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Per tick (``dt`` is the measured period: franky's ``time_step``, 1 ms normally, 2–5 ms after lost packets):

1. **Reference.** ``(q_ref, dq_ref, ddq_ref)`` advances by ``dt``. A new target that is not finite is rejected; otherwise it is clipped into ``[lower_limit + limit_margin, upper_limit - limit_margin]``.
   Every joint is replanned from the current reference state (so position, velocity and acceleration stay continuous) to ``(target, 0, 0)``, with one quintic per joint and one common duration ``T``, then holds the target exactly. If several targets arrive within one tick, only the newest is used.
2. **Timeout.** No target for ``command_timeout_s``: bounded stop and hold; ``request_stop()`` does the same at once. The next target resumes tracking.
3. **Velocity filter (GELLO).** ``dq_f = (1 - alpha) dq_f + alpha dq``, with ``dq_f = 0`` at start.
4. **Torque (GELLO).** ``tau = K (q_ref - q) - D dq_f``, plus the model's Coriolis torque if ``compensate_coriolis`` (never gravity: libfranka adds it), then each joint's ``|tau - tau_prev|`` is limited to ``max_delta_tau``.
5. **Start.** The reference starts at the measured ``q``, at rest. ``tau_prev`` is franky's previous torque command when a running torque motion is replaced, otherwise 0.

Upstream GELLO feeds the raw leader joint state into ``q_goal``; here ``q_goal`` is ``q_ref``. franky's ``Robot.move`` defaults (``limit_rate=False``, ``cutoff_frequency=100``) can stay: the torque rate is already limited to ``max_delta_tau`` per tick inside the law.

Planner and bounded stop
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Duration selection.** For each candidate ``T`` the quintic of every joint is fixed. A candidate is feasible when every joint has ``|v| <= max_velocity``, ``|a| <= max_acceleration`` and its position within the limits;
these are checked at the polynomial's exact extrema (the roots of the jerk, of the acceleration and of the velocity). ``T`` starts at the largest of ``min_segment_duration_s`` and two per-joint lower bounds (``|h| / v_max``, and the bang-bang minimum time of a double integrator with ``|a| <= a_max`` from ``(p0, v0)`` to rest at the target),
grows by ×1.1 per step until every joint is feasible, and the last step is bisected 20 times, using the feasible end. From rest the result is the exact minimum.

**When planning fails.** From a moving state, no common ``T`` may exist: a joint still accelerating is feasible only at a short ``T``, while another joint with a long way to go needs a long ``T``.
Then the target stays pending, the reference brakes with the bounded stop, and the plan is retried every tick (at rest, every target inside the limits is feasible).
Simulated results recorded in the README: in a 300 000-tick test with jittery targets and large jumps this affected 2.7 % of the ticks and the longest wait was 0.46 s; with a smooth leader motion at 10, 20 or 30 Hz it never happened.

**Bounded stop.** Per joint, from ``(p0, v0, a0)`` to rest at ``p0 + v0 T / 2 + a0 T² / 12``, with a unimodal velocity: ``|v|`` rises only while the reference is still accelerating, then falls monotonically to 0 without reversing.
``T`` is the smallest grid point within the limits (velocity, acceleration, no reversal, position limits); joints stop with their own durations. A joint with no feasible stop yet (at the velocity limit and still accelerating) keeps its current feasible, rest-ending segment and is retried every tick.
The stop point is ahead of the reference, not the reference position at the timeout: stopping exactly there would make every moving joint reverse.

Tracking lag
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Every replan ends at rest at the newest target, so a moving leader arm is followed with a lag of about one braking distance, which shrinks as ``max_acceleration`` grows. For a smooth leader moving at about 0.5 rad/s:

.. list-table::
   :header-rows: 1

   * - ``max_acceleration``
     - Lag (about)
   * - 2 rad/s²
     - 0.20 rad
   * - 4 rad/s² (default)
     - 0.06 rad
   * - 8 rad/s²
     - 0.04 rad

At 0.19 rad/s the lag stays within 0.02 rad. These numbers come from the README's simulation, not from robot measurements. Raise ``joint_trajectory.max_acceleration`` for faster following, lower it for gentler motion.

.. _en-joint-controller-parameters:

Parameters
--------------------------------------------------------------------------------

The station's ``joint_trajectory:`` block holds this robot's overrides; omitted keys use the native defaults (``joint_controller.default_params(robot_model)``, with the limits of Panda or FR3 chosen by ``robot_model``, not the station's ``joint_limits``).
The 12 keys follow; those marked (7) are vectors of length 7 (units per joint):

.. list-table::
   :header-rows: 1

   * - Key
     - Default
     - Unit
     - Rule
     - Meaning and source
   * - ``stiffness`` (7)
     - 240, 240, 240, 240, 100, 60, 20
     - Nm/rad
     - ≥ 0
     - The impedance law's K. GELLO ``controllers.yaml`` ``k_gains``
   * - ``damping`` (7)
     - 20, 20, 20, 10, 10, 10, 5
     - Nms/rad
     - ≥ 0
     - The impedance law's D. GELLO ``d_gains``
   * - ``velocity_filter_alpha``
     - 0.99
     - none
     - [0, 1]
     - Velocity filter coefficient. GELLO ``k_alpha``
   * - ``max_velocity`` (7)
     - 0.6 (joints 1–4), 0.75 (5–7)
     - rad/s
     - > 0
     - Planner's per-joint velocity limit. Specification value
   * - ``max_acceleration`` (7)
     - 4.0
     - rad/s²
     - > 0
     - Planner's per-joint acceleration limit; sets the tracking lag (table above). Chosen by this repository
   * - ``min_segment_duration_s``
     - 0.1
     - s
     - > 0
     - Shortest duration of one trajectory segment. Specification value
   * - ``lower_limit`` (7), ``upper_limit`` (7)
     - franka_description limits of ``robot_model``
     - rad
     - lower + 2·margin < upper
     - Joint position limits. franka_ros ``joint_limits.yaml``
   * - ``limit_margin``
     - 0.05
     - rad
     - ≥ 0
     - Margin by which targets are clipped inside the limits. Specification value
   * - ``command_timeout_s``
     - 0.5
     - s
     - > 0
     - With no new target this long, bounded stop and hold. GELLO ``kJointStateTimeout``
   * - ``max_delta_tau``
     - 1.0
     - Nm per tick
     - > 0
     - Per-joint limit on the torque change per tick. Specification value (as SERL's ``delta_tau_max``)
   * - ``compensate_coriolis``
     - ``true``
     - none
     - boolean
     - Whether to add the Coriolis torque (never gravity). Specification value

All values must be finite. An invalid value is **rejected**; unlike SERL's parameters it is never clamped into range. Loading the station checks only the key names (an unknown key fails immediately and lists the valid ones);
the values are validated by the native package on connect and on live updates. Changing a reference limit (position limits, velocity, acceleration) replans the current target (re-clipped) from the current state.

.. _en-joint-controller-live-update:

Live update
--------------------------------------------------------------------------------

``arm.joint_trajectory_params`` returns the full effective parameters (the station overrides plus the native defaults; ``None`` when the arm is not ``trajectory_impedance``),
and ``arm.update_joint_trajectory_params({...})`` updates part of them:

.. code-block:: python

   arm.joint_trajectory_params                              # the full effective set
   effective = arm.update_joint_trajectory_params(
       {"max_acceleration": [6.0] * 7})                     # merged into the current values

* With the controller running, the native ``update_params`` merges and validates the update, which takes effect on the **next tick**; either all of it applies, or an error is raised and nothing changes.
* With the controller not running (not connected, or not in joint servo mode), a native motion is first built from the complete merged parameters to validate them, then they are stored for the next start.
* An update persists until changed again and is reused by auto-recovery after a reflex and by the restart after ``stop()``; it does not rewrite the station file.
* Calling the update on an arm that is not ``trajectory_impedance`` raises ``has no joint trajectory parameters`` with the hint to use franky with ``joint_controller: trajectory_impedance``.

Through the daemon (:doc:`daemon`), the methods are ``arm.joint_trajectory_params`` (read-only, safe to retry after a reconnect) and ``arm.update_joint_trajectory_params`` (a command, run under the daemon's lock);
the remote arm object ``RemoteArm`` offers the same names and checks the key names on the client first. There is no matching ``rwrl`` command.

Telemetry
--------------------------------------------------------------------------------

The native motion has ``telemetry()``, a per-tick snapshot: ``ticks``, ``rel_time``, ``q``, ``dq``, ``dq_filtered``, ``coriolis``, ``q_ref``, ``dq_ref``, ``ddq_ref``,
``target`` (the last accepted, clipped target), ``goal`` (where the reference comes to rest: the target or the stop point), ``tau_cmd``, ``segment_t``, ``segment_T``, ``time_since_target``, ``timed_out``, ``stopped``,
``target_pending``, ``plan_retries``, ``applied_command_seq``, ``applied_params_seq``.
rwrl does not currently expose it through the arm, the daemon or the command line (the backend reads only ``ticks`` to wait for the first tick after an auto-recovery restart); when needed, use the native Python API directly:

.. code-block:: python

   import joint_controller as jc

   params = jc.default_params("fr3")               # "panda" | "fr3"
   motion = jc.TrajectoryImpedanceMotion(params)   # a franky.BaseTorqueMotion
   jc.validate_params(params)                      # validate a full dict without a robot
   jc.build_info()                                 # build_info.json

``jc.TrajectoryImpedanceLaw(params)`` runs the law offline (``step(q, dq, coriolis, dt)``) without franky or a robot.

Cost
--------------------------------------------------------------------------------

``update()`` allocates nothing and does bounded work per tick (at most 50 grid plus 20 bisection feasibility checks per plan). Measurements in the README on a WSL2 laptop (``-O2``): about 1 µs per tick on average in the simulation, 35 µs for an average replan,
and 104 µs for the worst tick (a failed plan plus the start of the brake, minimum of 5 repetitions); single-run maxima reach about 0.5 ms, which is that laptop's jitter. Realtime jitter on the robot has to be measured on site.

.. _en-joint-controller-provenance-and-licences:

Provenance and licences
--------------------------------------------------------------------------------

The whole ``joint_controller`` package is licensed ``MIT AND Apache-2.0`` (see ``native/joint_controller/LICENSE``, which lists every part and its licence):

.. list-table::
   :header-rows: 1

   * - Part
     - Source
     - Licence
   * - The joint impedance law, velocity filter, default gains (stiffness, damping, filter coefficient) and the command timeout
     - The public GELLO controller ``wuphilipp/gello_software`` @ ``9fc8c26`` (``joint_impedance_controller.cpp`` and ``config/controllers.yaml`` under ``ros2/src/franka_fr3_arm_controllers/``), Copyright (c) 2025 Franka Robotics GmbH
     - Apache-2.0 (``LICENSE.Apache-2.0`` is that directory's licence text)
   * - The Panda / FR3 joint position limit tables
     - franka_description (``frankaemika/franka_ros`` @ ``ddd2fff``)
     - Apache-2.0
   * - The reference generator (quintic planner, duration selection, bounded stop), the rest of the control law (reference, timeout stop, Coriolis, torque-rate limit), the parameter table and validation, the franky motion class, the Python binding, tests and scripts
     - This repository's own code
     - MIT
   * - franky (the linked, unmodified v2.0.0)
     - `TimSchneider42/franky <https://github.com/TimSchneider42/franky>`__
     - MIT

The velocity and acceleration limits, ``min_segment_duration_s``, ``limit_margin`` and ``max_delta_tau`` do not come from GELLO; they are this repository's specification values (``max_delta_tau`` is as SERL's ``delta_tau_max``).
The README's Acknowledgements section also lists upstream GELLO and franky.

Tests
--------------------------------------------------------------------------------

The native package has its own tests (the repository's tests do not collect ``native/joint_controller/tests``): C++ tests of the planner, the law and the parameters; a soak test (2 × 300 000 ticks against a simulated arm with GELLO-like retargeting at 10–30 Hz, jumps beyond the limits, non-finite targets, pauses past the timeout, lost packets and live gain changes, checking every invariant every tick); a franky motion test; and Python binding tests.
The repository's Python tests use a fake ``joint_controller`` module and cover station validation, the mode matrix, live updates, the doctor rows and the wizard flag, with no hardware.
