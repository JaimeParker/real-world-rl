.. _en-devices-roadmap-robot-infra-roadmap:

Robot Infra Roadmap
================================================================================

This document records the legacy V1 ManiSkill Franka impedance-controller roadmap.
The old ``robot_infra`` package is now the device layer in ``real_world_rl``; its
legacy simulation/ROS controllers live in ``real_world_rl/legacy/controller/``.
The training layer also belongs to this repository, under ``real_world_rl/loops/``
and ``real_world_rl/training/``. Paths beginning ``rl_garden/`` or ``3rd_party/``
are relative to the ``rl-garden/`` submodule.

**Legacy bridge:** ``real_world_rl/legacy/controller/real/franka_bridge.py``
forwards HTTP requests to ROS ``serl_franka_controllers``. ROS dependencies also
live in ``gripper_server.py`` in that directory. ``FrankaRealEnv`` accesses this
route over HTTP. The current ``real_robot`` route uses device backends such as
``real_world_rl/devices/backends/arms/franky.py`` locally or through the daemon;
it does not require this ROS bridge. Hardware control remains in the native SDK/
controller, not the torch simulation controller.

.. _en-devices-roadmap-v1-boundary:

V1 Boundary
--------------------------------------------------------------------------------

-  Keep controller math in pure torch modules under ``real_world_rl/legacy/controller/core``
   for GPU-batched simulation backends, where the point is avoiding GPU<->CPU
   handoffs across many parallel ``env.step()`` calls. This does not extend to
   real hardware: ``num_envs`` is always 1 on a physical robot, so there is no
   batching benefit to recoup, and ``libfranka``'s control loop has a hard
   real-time deadline (1kHz) that a Python/torch process cannot reliably meet
   regardless of how fast the tensor ops are.
-  Keep simulator-specific object access and command application in backend wrappers.
-  Treat ManiSkill as the first backend, not the owner of the control algorithm.
-  Keep GPU training paths batched and tensor-native.

.. _en-devices-roadmap-near-term-extensions:

Near-Term Extensions
--------------------------------------------------------------------------------

-  Add a dynamics provider interface for optional Coriolis, gravity, mass matrix,
   and inverse dynamics terms.
-  Implement a CPU Pinocchio provider for smoke tests and a GPU-compatible provider
   only when the required tensors are available without CPU transfer.
-  Add an integral impedance term (``Ki``) with explicit reset, clamp, and anti-windup
   behavior in ``controller/core``, mirrored (not physically shared -- see V1
   Boundary) in the real-time controller for behavioral parity.
-  Add a joint-delta adapter that maps joint commands into nullspace targets or a
   dedicated joint impedance command.

.. _en-devices-roadmap-backend-roadmap:

Backend Roadmap
--------------------------------------------------------------------------------

-  ``maniskill``: current wrapper, qf torque application, pytorch_kinematics Jacobian.
-  ``mujoco``: future wrapper for state/Jacobian/torque command extraction.
-  ``real_franka``: future real-time controller providing hardware safety gates,
   torque-rate limits, fault handling, and calibrated model/dynamics providers.
   Runs as its own real-time C++ process (today: ``serl_franka_controllers``;
   potentially a project-owned successor later) -- never a torch module under
   ``controller/core``. ``real_world_rl/legacy/controller/real/`` owns only the non-real-time
   interface into it.

.. _en-devices-roadmap-safety-and-observability:

Safety And Observability
--------------------------------------------------------------------------------

-  Add backend-level torque, velocity, workspace, and pose target guards before any
   real robot deployment.
-  Keep controller diagnostics opt-in so training observation spaces do not change
   accidentally.
-  Standardize logging for target pose, pose error, commanded torque, torque clamp,
   and optional measured wrench.

.. _en-devices-roadmap-real-world-rl-training-layer-todos-rl_garden-side:

Real-World RL Training-Layer TODOs (``rl_garden`` side)
--------------------------------------------------------------------------------


These current training components live in this repository; HG-DAgger's growing
on-disk correction dataset remains outside the implemented methods.

-  **Buffer layer**: HIL-SERL grows an in-memory demonstration buffer from human
   interventions, seeded by ``--demo-dataset-paths``. Online rows and human rows
   share the online replay, with human rows additionally entering demonstration
   replay. SQLite outboxes/journals and checkpoint bundles provide recovery.
   ``LearnerLoop._refresh_offline_data()`` remains a no-op for SERL/HIL-SERL;
   that hook is separate from the implemented live demonstration buffer.
-  **Runner/launcher layer**: ``real_world_rl/loops/`` contains base and concrete
   method loops; ``real_world_rl/training/args.py``, ``registry.py`` and
   ``entrypoints/`` provide the shared arguments, registry and method entrypoints.
   ``rwrl train`` and ``examples/train_real_world.py`` dispatch by subcommand.
   ``real_world_rl/training/env_building.py`` composes the HIL wrappers.
   HIL-SERL uses ``RLPDHybrid`` (or local ``ContinuousHilRLPD`` without grippers);
   standalone BC is implemented as ``real_world_bc``. Reward classifier model,
   inference and training live under ``rl_garden/models/reward/success/``;
   collection lives in ``real_world_rl/reward/``. See :doc:`HIL-SERL roadmap <hil-serl-roadmap>`.
