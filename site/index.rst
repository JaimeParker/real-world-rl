real-world-rl 中文教程
======================

.. container:: expnote-home-intro

   从手动完成一次任务，到让策略在同一台机器上尝试，再到用人工纠正开展强化学习。
   已有 checkpoint 的读者可以直接进入部署与推理。

.. grid:: 1 1 2 2
   :gutter: 3

   .. grid-item-card:: 安装与初验
      :link: install
      :link-type: doc
      :shadow: none

      按机器用途和遥操作设备选择依赖，在没有机械臂时完成软件初验。

   .. grid-item-card:: 路线一 · 遥操作和真机数据采集
      :link: collect
      :link-type: doc
      :shadow: none

      用 SpaceMouse 或 GELLO 完成一次任务，检查哪些动作进入了示范。

   .. grid-item-card:: 路线二 · 行为克隆（BC）
      :link: bc
      :link-type: doc
      :shadow: none

      用 ACT 学习一段连续动作，保存一份可独立推理的策略。

   .. grid-item-card:: 部署与推理
      :link: deploy
      :link-type: doc
      :shadow: none

      在工作站运行 ACT；已有 π0.5 权重可以走远程策略服务路径。

   .. grid-item-card:: 路线三 · 真机强化学习
      :link: rl
      :link-type: doc
      :shadow: none

      在已完成部署初验的工作站上，用一次人工纠正追踪 actor 与 learner 的数据流。

.. toctree::
   :caption: 中文教程
   :hidden:
   :maxdepth: 2

   install
   collect
   bc
   deploy
   openpi
   rl

.. toctree::
   :maxdepth: 2
   :caption: English
   :hidden:

   en/index
