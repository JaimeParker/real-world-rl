.. _devices-roadmap-robot-infra-roadmap:

机器人基础设施路线图
================================================================================

本文记录旧 V1 ManiSkill Franka 阻抗控制器路线图。旧 ``robot_infra`` 包现为
``real_world_rl`` 设备层，旧仿真/ROS 控制器位于 ``real_world_rl/legacy/controller/``。
训练层也属于本仓库，位于 ``real_world_rl/loops/`` 和 ``real_world_rl/training/``。
``rl_garden/`` 或 ``3rd_party/`` 开头的路径相对于 ``rl-garden/`` 子模块。

**旧桥接：**\ ``real_world_rl/legacy/controller/real/franka_bridge.py`` 将 HTTP
请求转发到 ROS ``serl_franka_controllers``；同目录 ``gripper_server.py`` 也依赖 ROS。
``SerlHttpArm`` 经 HTTP 使用此路线。当前 ``real_robot`` 路线在本地或经 daemon
使用 ``real_world_rl/devices/backends/arms/franky.py`` 等设备后端，无需此 ROS 桥接。
真机控制仍由原生 SDK/控制器执行，不使用 torch 仿真控制器。

franka_real 环境已移除，由 real_robot + serl_http 替代。

.. _devices-roadmap-v1-boundary:

V1 边界
--------------------------------------------------------------------------------

-  将控制器的数学计算保留在 ``real_world_rl/legacy/controller/core`` 下的纯 torch
   模块中，用于 GPU 批量仿真后端，目的是避免大量并行 ``env.step()`` 调用中的
   GPU<->CPU 数据交接。这不适用于真机：物理机器人的 ``num_envs`` 始终为 1，
   无法获得批处理收益，而且 ``libfranka`` 的控制循环有硬实时截止要求（1kHz），
   无论张量运算多快，Python/torch 进程都无法可靠满足这一要求。
-  将模拟器特有的对象访问和命令执行保留在后端包装器中。
-  将 ManiSkill 作为第一个后端，而非控制算法的归属方。
-  保持 GPU 训练路径的批量处理和原生张量运算。

.. _devices-roadmap-near-term-extensions:

近期扩展
--------------------------------------------------------------------------------

-  添加动力学提供器接口，以支持可选的科里奥利项、重力项、质量矩阵和逆动力学项。
-  实现用于冒烟测试的 CPU Pinocchio 提供器；只有在无需 CPU 传输即可获得所需张量时，
   才实现兼容 GPU 的提供器。
-  在 ``controller/core`` 中添加阻抗积分项（``Ki``），明确其重置、限幅和抗积分饱和行为；
   在实时控制器中实现对应逻辑以保持行为一致（不直接共享实现，见 V1 边界）。
-  添加关节增量适配器，将关节命令映射为零空间目标或专用的关节阻抗命令。

.. _devices-roadmap-backend-roadmap:

后端路线图
--------------------------------------------------------------------------------

-  ``maniskill``：现有包装器，qf 力矩施加，pytorch_kinematics 雅可比矩阵。
-  ``mujoco``：未来用于提取状态、雅可比矩阵和力矩命令的包装器。
-  ``real_franka``：未来的实时控制器，提供硬件安全门控、力矩变化率限制、故障处理，
   以及经过标定的模型／动力学提供器。它作为独立的实时 C++ 进程运行
   （目前为 ``serl_franka_controllers``，以后可能由项目自有实现接替），
   绝不作为 ``controller/core`` 下的 torch 模块。
   ``real_world_rl/legacy/controller/real/`` 只负责通向该进程的非实时接口。

.. _devices-roadmap-safety-and-observability:

安全与可观测性
--------------------------------------------------------------------------------

-  在任何真机部署之前，添加后端层的力矩、速度、工作空间和位姿目标保护。
-  控制器诊断保持为显式启用，避免意外改变训练的观测空间。
-  统一目标位姿、位姿误差、指令力矩、力矩限幅以及可选实测力／力矩的日志格式。

.. _devices-roadmap-real-world-rl-training-layer-todos-rl_garden-side:

真机 RL 训练层待办事项（``rl_garden`` 侧）
--------------------------------------------------------------------------------


当前训练组件属于本仓库；HG-DAgger 持续增长的磁盘纠正数据集仍不在已实现方法中。

-  **缓冲区层**：HIL-SERL 从人工介入增长内存示范缓冲区，以 ``--demo-dataset-paths``
   初始化。在线与人工行均进入在线回放，人工行另进入示范回放。SQLite outbox/journal
   和 checkpoint bundle 提供恢复。SERL/HIL-SERL 的
   ``LearnerLoop._refresh_offline_data()`` 仍为空操作，与已实现的实时示范缓冲区不同。
-  **运行器／启动器层**：``real_world_rl/loops/`` 包含基类与各方法循环；
   ``real_world_rl/training/args.py``、``registry.py`` 与 ``entrypoints/`` 提供共享参数、
   注册表和方法入口。``rwrl train`` 与 ``examples/train_real_world.py`` 按子命令分派。
   ``real_world_rl/training/env_building.py`` 组合 HIL 包装器。HIL-SERL 使用
   ``RLPDHybrid``（无夹爪时使用本地 ``ContinuousHilRLPD``）；独立 BC 已实现为
   ``real_world_bc``。奖励分类器模型、推理与训练位于
   ``rl_garden/models/reward/success/``，采集位于 ``real_world_rl/reward/``。
   见 :doc:`HIL-SERL 路线图 <hil-serl-roadmap>`。
