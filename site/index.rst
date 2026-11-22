real-world-rl 中文教程
================================================================================

.. container:: expnote-home-intro

   按顺序读：先安装初验，再搭建工作站并录制示范，然后训练和部署策略，最后在真机上做强化学习。

.. rubric:: 开始：安装与初验

.. grid:: 1 1 2 2
   :gutter: 3

   .. grid-item-card:: 安装与初验
      :link: tutorials/install
      :link-type: doc
      :shadow: none

      确定机器角色，安装设备依赖，在没有硬件时验证软件。

.. rubric:: 搭建工作站与遥操作录制

.. grid:: 1 1 2 2
   :gutter: 3

   .. grid-item-card:: 建立工作站
      :link: tutorials/station
      :link-type: doc
      :shadow: none

      配置 station 和任务 preset，读取 Franka、夹爪和相机状态；看清机械臂的安装方式，测出桌面坐标系 task_frame。

   .. grid-item-card:: 选择并配置 Franka 控制器
      :link: tutorials/controller
      :link-type: doc
      :shadow: none

      对比 impedance、internal 与 serl；记下型号、server version 和内核，在 NUC 上装好对应软件（serl 从源码构建 franky + serl_controller），写 station 字段，用 doctor 和归位验收。

   .. grid-item-card:: 用 SpaceMouse 遥操作
      :link: tutorials/spacemouse
      :link-type: doc
      :shadow: none

      读取六轴和按钮，在桌面坐标系里建立方向档案；练习 tracker 接管、夹爪和交还，再按 HIL-SERL 摆放 SpaceMouse 用 serl 模式。

   .. grid-item-card:: 用 GELLO 遥操作
      :link: tutorials/gello
      :link-type: doc
      :shadow: none

      找到串口和关节 ID，检查方向、偏移与夹爪标定；对齐主从姿态后接管、操作和交还，区分关节与笛卡尔动作。

   .. grid-item-card:: 录制与标注示范
      :link: tutorials/record
      :link-type: doc
      :shadow: none

      完成一个回合，标记成功或失败，观察结束后的保持与下一轮复位。

   .. grid-item-card:: 检查录制结果
      :link: tutorials/inspect
      :link-type: doc
      :shadow: none

      打开完整归档，找出接管区间、环境动作和实测位姿变化，检查训练 pkl 与失败尝试。

.. rubric:: 用示范训练行为克隆（BC）

.. grid:: 1 1 2 2
   :gutter: 3

   .. grid-item-card:: 准备 BC 训练数据
      :link: tutorials/bc-data
      :link-type: doc
      :shadow: none

      选取示范，检查观测字段、相机名称、动作维度、动作含义和人工控制标记；比较 SpaceMouse 和 GELLO。

   .. grid-item-card:: 训练第一份 BC 策略
      :link: tutorials/bc-train
      :link-type: doc
      :shadow: none

      配置训练入口、输入输出目录，解释观测到示范动作的监督关系及 human_action_mask。

   .. grid-item-card:: 离线检查 BC checkpoint
      :link: tutorials/bc-check
      :link-type: doc
      :shadow: none

      用保存的观测运行推理，对照预测和示范动作，记录相机、状态字段与动作约定。

.. rubric:: 部署与推理

.. grid:: 1 1 2 2
   :gutter: 3

   .. grid-item-card:: 部署与推理
      :link: tutorials/deploy
      :link-type: doc
      :shadow: none

      A：恢复本仓 BC checkpoint 并完成受监护部署。B：核对 openpi / RLinf 权重、归一化统计与任务，检查 GPU 服务、dry-run 和工作站动作映射；Pick_Red WebSocket 推理待验证，双臂 Clean Desk 为扩展。

.. rubric:: 在真机上做强化学习（HIL-SERL）

.. grid:: 1 1 2 2
   :gutter: 3

   .. grid-item-card:: 看懂一次在线强化学习实验
      :link: tutorials/rl-overview
      :link-type: doc
      :shadow: none

      标明设备服务、actor 和 learner 所在机器，沿时间步追踪观测、实际动作、奖励和下一条观测。

   .. grid-item-card:: 准备可训练的任务
      :link: tutorials/rl-task
      :link-type: doc
      :shadow: none

      配置复位、回合结束、成功判定、奖励来源和示范，验证成功与失败标注。

   .. grid-item-card:: 启动 HIL-SERL 训练
      :link: tutorials/rl-train
      :link-type: doc
      :shadow: none

      按机器启动，观察 actor 产生数据、learner 收到样本，确认参数更新回到 actor。

   .. grid-item-card:: 训练中做一次人工纠正
      :link: tutorials/rl-intervene
      :link-type: doc
      :shadow: none

      分别用 SpaceMouse 与 GELLO 接管、纠正和交还，追踪策略动作、执行动作、人工标记与 learner 样本。

   .. grid-item-card:: 评估策略与恢复训练
      :link: tutorials/rl-resume
      :link-type: doc
      :shadow: none

      保存 checkpoint，区分任务成功和自主成功，确认中断后的本地数据，恢复 learner 并独立评估新策略。

.. toctree::
   :maxdepth: 2
   :caption: 开始：安装与初验
   :hidden:

   tutorials/install

.. toctree::
   :maxdepth: 2
   :caption: 搭建工作站与遥操作录制
   :hidden:

   tutorials/station
   tutorials/controller
   tutorials/spacemouse
   tutorials/gello
   tutorials/record
   tutorials/inspect

.. toctree::
   :maxdepth: 2
   :caption: 用示范训练行为克隆（BC）
   :hidden:

   tutorials/bc-data
   tutorials/bc-train
   tutorials/bc-check

.. toctree::
   :maxdepth: 2
   :caption: 部署与推理
   :hidden:

   tutorials/deploy
   tutorials/deploy-openpi

.. toctree::
   :maxdepth: 2
   :caption: 在真机上做强化学习（HIL-SERL）
   :hidden:

   tutorials/rl-overview
   tutorials/rl-task
   tutorials/rl-train
   tutorials/rl-intervene
   tutorials/rl-resume

.. toctree::
   :maxdepth: 2
   :caption: English
   :hidden:

   en/index
