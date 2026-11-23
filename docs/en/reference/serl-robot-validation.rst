SERL controller: robot validation
================================================================================

This is the hardware checklist for ``cartesian_controller: serl`` and the SpaceMouse modes. Run it in
order on the robot PC (for example an FR3 behind a NUC, with the SpaceMouse on the NUC), from the repository
root with the project environment active. Each step says what to look at and when it passes. Stop
at the first step that fails, and keep its logs. Background: \ :doc:`SERL controller <serl-controller>`\ .

Safety for every motion step: one person holds the external activation device and nobody else is
in the cell. Use a free workspace well inside ``safety_box``, and start with the arm at home.

0. Bring the robot PC's working tree to this code
--------------------------------------------------------------------------------------------------

If the checkout carries local changes, save them, then update to the commits that contain this
work:

.. tutorial-test: skip
.. code-block:: bash

   git status --short
   git diff > ~/rwrl-lab-patch-$(date +%Y%m%d).patch    # keep the old patch, just in case
   git stash push --include-untracked -m "lab patch before SERL validation"   # or: git checkout -- . after saving
   git fetch origin && git checkout main && git pull --ff-only
   git submodule update --init rl-garden
   git log --oneline -3                                  # inspect the checked-out revision; it must include SERL support
   python -m pytest tests -q -x                          # optional: the suite needs no hardware

``local/`` is git-ignored and is not touched by any of this. Back up the station before editing it:

.. tutorial-test: skip
.. code-block:: bash

   cp local/station.yaml local/station.yaml.before-serl-validation.bak
   ls local/station.yaml*.bak*      # wizard copies: station.yaml.bak.<UTC>; CLI edits: station.yaml.<id>.bak

1. PREEMPT_RT kernel and realtime limits
--------------------------------------------------------------------------------

.. tutorial-test: skip
.. code-block:: bash

   uname -v                          # must contain PREEMPT_RT after the reboot
   cat /sys/kernel/realtime          # 1

.. code-block:: bash

   rwrl doctor --fix                 # writes rtprio 99 / memlock unlimited for this user; log in again

Install the kernel by hand: on Ubuntu with Ubuntu Pro, ``sudo pro enable realtime-kernel``;
otherwise use Franka's "Setting up the real-time kernel" guide. Then reboot. Optionally measure the
kernel's latency with ``sudo cyclictest -m -Sp90 -i200 -h400 -D 60s`` (from ``rt-tests``). The max
latency should stay well below 100 µs.

Pass: ``realtime.kernel`` and ``realtime.limits`` PASS in ``rwrl doctor``.

2. Build and install franky + serl_controller
------------------------------------------------------------------------------------------

Read the robot server version from ``rwrl doctor`` (``libfranka mismatch: robot server N, ...``, or,
when the handshake passes, the installed franky's ``+libfranka.X`` label) and pick
``LIBFRANKA_VERSION`` from the table in
\ :ref:`SERL controller <en-serl-controller-installing-franky-and-serl_controller>`\ . Do not run
``rwrl doctor --install-sdk`` for this station. Then, on the robot PC (no Docker):

.. tutorial-test: skip
.. code-block:: bash

   LIBFRANKA_VERSION=0.17.0 bash native/serl_controller/scripts/install-on-ubuntu.sh
   python -c "import franky, serl_controller as sc; print(sc.BUILD_INFO)"
   cat ~/.cache/rwrl-native-build/BUILD_INFO.txt

Pass: the script ends with ``PASS  sdk:serl_controller``, CTest passes (including
``golden_equivalence`` and ``joint_reset_golden``), and ``BUILD_INFO`` names the build id of
``BUILD_INFO.txt`` with ``franky_layout: source``. (A wheel pair built elsewhere with the Docker recipe
works as well; install both wheels of one build directory together.)

3. robot_model: confirm in Desk
--------------------------------------------------------------------------------

In Desk → Settings → System, read the system version (4.x = Franka Emika Robot / ``panda``, 5.x =
Franka Research 3 / ``fr3``) and the robot type. A server version ≥ 6 (system 5.x) means ``fr3`` is
expected. Set it in the station:

.. code-block:: yaml

   robot_model: fr3

4. Doctor, first with the current controller, then with serl
------------------------------------------------------------------------------------------------------------------------

First run ``rwrl doctor`` on the unchanged station (``impedance``). The source-built franky must not
break anything: ``sdk:arm``, ``handshake``, ``feedback:*`` and ``mode:robot_mode`` PASS, and
``mode:robot_model`` reads ``robot_model: fr3 (robot URDF: fr3)`` (or the server version).

Then switch the station:

.. code-block:: yaml

   cartesian_controller: serl
   realtime: enforce
   serl: null                 # SERL's .cfg defaults
   serl_auto_recover: null    # {enabled: true, max_per_episode: 5}

Stop ``rwrl daemon`` and any recorder/actor, then run ``rwrl doctor`` again
on the idle station. Restart the daemon after the checks pass. Expected rows:

.. list-table::
   :header-rows: 1

   * - Row
     - Expected
   * - ``sdk:serl_controller``
     - PASS ``serl_controller 0.1.0 and franky_control 2.0.0+libfranka.0.17.0 from the same build (...)``
   * - ``config:serl_control``
     - PASS, or WARN listing ``control:`` Cartesian keys; harmless, they only apply in ``impedance``
   * - ``mode:realtime_kernel``
     - PASS ``PREEMPT_RT kernel; realtime: enforce``
   * - ``mode:serl_realtime``
     - PASS ``cartesian_controller: serl with realtime: enforce``
   * - ``mode:robot_model``
     - PASS ``robot_model: fr3 (...)``; a WARN means step 3 is not done

5. First serl servo: +x 1 cm
--------------------------------------------------------------------------------

Use ``spacemouse_mode: tracker`` first (it has the lead cap). Start a translation diagnostic at
low gain:

.. code-block:: bash

   rwrl --station local/station.yaml --preset local/task.yaml teleop run \
     --mode tracker --diagnose translation --gain 0.2 --duration-s 10 \
     --record local/serl-first-01

``--record`` is a dataset directory; this diagnosis's episode is also stored in ``local/serl-first-01/`` and can be replayed with ``rwrl dataset view``. Take over with button 0 and push the cap gently forward (+x) for about 1 cm, then release. Watch
and log:

-  the stderr lines ``INTERVENTION ON: the operator drives the arm`` and, 0.3 s after you hand back,
   ``INTERVENTION OFF: the operator handed back``;
-  that the arm moves forward without a sideways or vertical drift, that it starts within a few
   tenths of a second (stiffness ramps from 0 over ~0.2 s after every controller start), and that
   it stops when you release;
-  the stop trace the session writes (``local/serl-first-01.session-*/stop-trace-*.assessment.json``)
   and the daemon log (no ``SERL impedance controller stopped``).

Then analyse the session:

.. tutorial-test: skip
.. code-block:: bash

   python scripts/analyze_teleop_direction.py local/serl-first-01.session-*/

It prints one line per translation-isolated segment: ``angle`` between the commanded and the
measured displacement, ``speed``, ``lead at release`` (target minus TCP when you let go) and ``coast``
(how far the TCP still travels in the 1 s after release), with PASS/FAIL against angle < 10° and
coast < 3 mm.

**Control loop jitter.** SERL allocates Eigen temporaries on every 1 kHz tick, as the original does
(kept for bit-identical results). On a PREEMPT_RT kernel this has not shown up as a problem, but
check it. Over the session, the daemon log must show no ``communication_constraints_violation``
reflex. For a direct measurement, stop the daemon and hold the pose for 30 s while sampling the
controller's tick counter against robot time. A lost tick shows up as ``tick`` falling behind
``rel_time``:

.. code-block:: python

   import time
   from real_world_rl.devices import Robot
   from real_world_rl.devices.arm import ControlMode
   from real_world_rl.station import load_station
   robot = Robot.from_config(load_station("local/station.yaml").select().config); robot.connect()
   arm = robot.arm
   arm.set_control_mode(ControlMode.CARTESIAN_SERVO)
   pose = arm.read_state()["tcp_pose"]
   for _ in range(300):                      # 30 s at 10 Hz, holding the pose
       arm.servo_cartesian(pose); time.sleep(0.1)
   snap = arm._handle.serl_motion.snapshot()  # internal handle: the running SerlCartesianImpedanceMotion
   print(snap["tick"], snap["rel_time"], snap["tick"] / max(snap["rel_time"] * 1000, 1))
   arm.stop(); robot.disconnect()

Pass: the ratio is > 0.99 (fewer than 1 % lost ticks) and there are no reflexes.

6. Compare tracker, serl and internal on the same test
------------------------------------------------------------------------------------------------------------

Repeat the same translation-isolated test with the same operator, pushes and site. Push ±x, ±y
and ±z, one axis at a time, each for 1-2 s, at about half deflection. Repeat each row twice and
keep the ``--record`` names distinct:

.. list-table::
   :header-rows: 1

   * - Run
     - Station ``cartesian_controller``
     - ``--mode``
   * - A
     - ``impedance``
     - ``tracker``
   * - B
     - ``serl``
     - ``tracker``
   * - C
     - ``serl``
     - ``serl``
   * - D
     - ``internal``
     - ``tracker``

.. code-block:: bash

   rwrl --station local/station.yaml --preset local/task.yaml teleop run \
     --mode serl --diagnose translation --gain 0.5 --duration-s 20 --record local/cmp-C-01

.. tutorial-test: skip
.. code-block:: bash

   python scripts/analyze_teleop_direction.py local/cmp-*.session-*/

Restart the daemon after each station change, and keep ``spacemouse_mode`` / ``--mode`` aligned with
the table. For run C, place the SpaceMouse as HIL-SERL does (front = task-frame +x, left = +y,
up = vertical +z) and check the six single axes first; a rolled base needs the measured
``task_frame`` (see the station tutorial). ``--mode serl`` needs the preset's xyz and RPY safety boxes. Pass, per run: every segment
has angle < 10° and coast < 3 mm (post-release displacement, distinct from lead at release). Record the median angle, the speed and the lead at release
per run. The serl runs should be at least as good as run A.

7. Gripper
--------------------------------------------------------------------------------

.. code-block:: bash

   rwrl gripper close     # HIL-SERL grasp: 0.01 m, 0.3 m/s, 130 N; prints the width when done
   rwrl gripper open      # fully open at 0.3 m/s
   rwrl gripper move 0.5  # half of max_width_m

In a teleop session the gripper button must not stall the arm: commands run on a worker thread,
and a newer command replaces a queued one. Pass: the widths are as expected, and an arm motion
during a close does not stutter.

8. Joint reset
--------------------------------------------------------------------------------

.. code-block:: bash

   rwrl home go           # serl mode: SERL's joint reset (10 s linear blend, then hold)

The arm blends to home in about 10 s and then holds. The daemon log reports ``SERL joint reset TIMEOUT`` only if it does not get there within ~33 s. Read the report while no session runs:

.. code-block:: python

   from real_world_rl.server.remote import RemoteRobot, RemoteRobotConfig
   robot = RemoteRobot(RemoteRobotConfig(address="127.0.0.1:50051")); robot.connect()
   print(robot.arm.last_joint_reset)   # reached, polls, max_abs_error_rad, soft_limited_ticks, robot_model
   robot.disconnect()

Pass: ``reached: true``, ``max_abs_error_rad`` < 0.01, ``robot_model: fr3``. ``soft_limited_ticks`` is 0
for a home well inside the limits.

9. Auto-recovery
--------------------------------------------------------------------------------

In a serl-mode session (station ``serl``, ``--mode tracker`` or ``serl``), drive slowly towards a soft
obstacle (a foam block), or have the second person push the arm gently, until a reflex triggers
(for example ``cartesian_reflex``). Keep the speed low. Expected:

-  the daemon log prints ``SERL impedance controller stopped (<error>); auto-recovery 1/5 this episode: clearing errors and restarting the controller at the measured pose``;
-  the arm stays where it stopped, is briefly soft (~0.2 s ramp), and then follows the next input;
-  ``robot.arm.serl_auto_recovery`` (as in step 8) shows ``episode_count: 1`` and ``last.error``.

A sixth reflex in the same episode must instead raise ``... (serl_auto_recover: max_per_episode 5 reached, ...)``. Servoing is then refused until ``rwrl recover``. Pass: both behaviours, and no
motion after a recovery other than the one you command.

10. Roll back
--------------------------------------------------------------------------------

To undo the station changes, restore the backup and restart the daemon:

.. tutorial-test: skip
.. code-block:: bash

   cp local/station.yaml.before-serl-validation.bak local/station.yaml

Older copies are ``local/station.yaml.bak.<UTC>`` (setup wizard) and ``local/station.yaml.<id>.bak``
(CLI edits). To return to the official franky wheel, switch the station to
``cartesian_controller: impedance`` or ``internal``, then run ``rwrl doctor --install-sdk``. To return
to the old code, check out the previous commit, or apply the patch saved in step 0.

Hand back the session directories (``local/*.session-*/``), the daemon log, the analysis output and
the doctor report (``rwrl --json doctor > local/doctor-serl.json``).
