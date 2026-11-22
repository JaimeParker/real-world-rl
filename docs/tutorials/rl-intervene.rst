训练中做一次人工纠正
================================================================================

本章只追踪一段接管：策略提出了什么、人替换了什么、learner 收到了哪一条。
保持 :doc:`rl-train` 的 learner 与设备服务运行。两种输入路径共用后面的记录检查。
使用的观测与七维 cartesian_delta 动作必须与各自录制 session 一致。

1. 按自己的输入设备接管与交还
--------------------------------------------------------------------------------

只选择录制时已标定的那种设备。下面命令是第三章 actor 命令的替代形式，
已有 actor 运行时直接按操作步骤，不要另起第二个 actor。
``LEARNER_HOST`` 沿用第三章设置，两台机器都使用第二章生成的配置。

.. warning::

   actor 会 reset、归位、开爪并执行策略。人工交还是恢复策略控制，机械臂可能立即继续运动。
   先确认停止方式、工作区、home 路径和现场监护；交还不是停止命令。

.. tab-set::

   .. tab-item:: SpaceMouse
      :selected:

      使用 :doc:`spacemouse` 的方向档案与 tracker 配置，先确认帽和按钮都已松开。

      .. literalinclude:: ../examples/rl-intervene.sh
         :language: bash
         :start-after: # start spacemouse
         :end-before: # end spacemouse

      #. 等待 reset 与 START_NEUTRAL 完成；需要纠正时按 button_0 接管。
      #. 小幅推帽修正位置或姿态；接管时按 button_1 切换夹爪开／闭。
         松帽停止推进目标，但仍可能走完领先距离，且接管仍然有效。
      #. 再按 button_0 交还策略。终端 t/h 在这个模式不切换控制权。
      #. 按 s 或 f 标注实际结果，停在 EPISODE_HOLD 后决定 n 开始下一轮或 q 退出。

      **你应该看到**：人工段有非零 human_action_mask；交还后后续人工 mask 回到零，
      policy_action 仍随策略产生。夹爪有独立的实际控制标记，以记录为准。

      **如果不对**：推帽即接管时检查是否误用了 serl；按钮没反应时回查 HID 读取与编号；
      WORKSPACE HOLD 时检查方向和工作区再重新接管，不扩大边界继续试。
      若选 SERL，必须使用 :doc:`spacemouse` 单独验证过的 SERL 配置和数据：它靠运动／按钮电平接管，
      松开交还，button_0 关爪、button_1 开爪；不能临时只改模式合并训练。

   .. tab-item:: GELLO

      使用 :doc:`gello` 的 leader_cartesian session，保留 leader-options.yaml 的路径与标定。
      这里按末端相对变化控制，不是关节扩展中的 joint_abs／joint_delta。

      .. literalinclude:: ../examples/rl-intervene.sh
         :language: bash
         :start-after: # start gello
         :end-before: # end gello

      #. 等待 reset；主臂稳定，设备自身 Space toggle 保持关闭，使中立检查通过。
      #. 在 actor 终端按 t。当前主臂姿态成为相对映射锚点；重复 t 不会交还。
      #. 小幅移动主臂纠正；接管时先把夹爪触发器匹配上次命令状态，再操作开闭。
      #. 按 h 交还策略；下一次 t 会重新锚定。按 s/f 标注，保持阶段用 n/q。

      **你应该看到**：动作仍为六维笛卡尔增量加夹爪；人工段 mask 非零，h 后为零。
      主臂绝对关节姿态不要求逐关节等于从臂，这里的对齐是相对映射与触发器匹配。

      **如果不对**：START_NEUTRAL 卡住时检查实体 toggle 与 intervention: always；
      串口无法打开时按 :doc:`gello` 检查 franka.os:serial、设备权限及是否被另一进程占用；
      用 Space 切换却不接管时改用当前事件源的 t/h；关节方向不符则回到标定，不用软件标签掩盖。

使用残差 ACT 时，``--residual-base-resync-on-intervention`` 默认开启。
每个介入步后、计算下一观测的基础动作前都会重置提供器，让它从实际新状态重新规划块；
接管当步保留接管前已计算的基础动作。BC 的 checkpoint 提供器及 zero 提供器无状态，不受影响。

``--no-residual-base-resync-on-intervention`` 会关闭这个行为：temporal_agg=0 将继续回放旧块，
temporal_agg=1 则继续混合接管前／期间的块，base_actions、next_base_actions 和交还动作可能滞后一个块。
**你应该看到**：介入后下一观测的基础动作来自新块首动作；核对记录时不要把当步基础动作误认为已重算。
**如果不对**：交还后沿旧计划走时先核对这个开关及 ACT 聚合选项，不直接加大残差尺度。

2. 在 learner 上找出这一次纠正
--------------------------------------------------------------------------------

在第三章的 journal 检查输出中找一条 human_action_mask 非零的 transition，
记下完整 record_id、episode_id 和 step_index，再单独读取它。

.. tutorial-test: skip
.. literalinclude:: ../examples/rl-intervene.sh
   :language: bash
   :start-after: # start match
   :end-before: # end match

**你应该看到**：同一行包含以下证据。

.. list-table::
   :header-rows: 1
   :widths: 35 65

   * - 字段
     - 本次核对方式
   * - ``policy_action``
     - actor 当时提出的动作。不要用当前 checkpoint 重新预测来冒充历史提案。
   * - ``action``／``executed_action``
     - 人工替换后提交环境的请求；两者在这条在线记录里应一致。真实 TCP 跟踪仍要读 obs／next_obs。
   * - ``human_action_mask``／``override_mask``
     - 前者是实际人工控制的逐维标记，后者还可能包含非人工补齐；统计介入用前者。
   * - ``record_id``／``episode_id``／``step_index``
     - 同一条数据跨机器的身份、所属回合和步序。journal_or_outbox_seq 是数据库本地序号，不能跨机器匹配。
   * - ``policy_session_id``／``policy_version``／``global_update``
     - 这次动作使用哪一份策略。后续正更新版本证明参数已返回 actor，不能证明单个样本必然被抽中。

journal 中找到同一 record_id 说明 learner 已接收；更新日志中的 received／intervened_received
说明摄入在推进。标准 HIL-SERL replay 只保留训练需要的字段，并不提供逐样本“第几次 SGD 抽到了它”的日志。
因此本章追踪到 journal 接收、进入 replay 的规则和新策略发布，不声称某条纠正已经产生可量化因果贡献。

**如果不对**：ID 查不到时先确认在同一个 learner run 上查；outbox 尚有该 ID 则等待上传并检查
upload_retries。已经 ACK 的 outbox 行会删除，生产运行的持久证据在 learner journal；
不要把空 outbox 当成丢数据。syncing=True 的行仍归档，但训练投影清掉人工监督，不计入人工 demo。

3. 用 CPU 例子核对同一 ID 和训练字段
--------------------------------------------------------------------------------

先完成第三章练习，再比较它额外保存的 actor 审计副本和 learner journal。
该审计副本专为教程增加；不需要改生产 actor 才能读取 learner 的记录。

.. tutorial-test: skip
.. literalinclude:: ../examples/rl-intervene.sh
   :language: bash
   :start-after: # start rehearsal
   :end-before: # end rehearsal

**你应该看到**：检查器按 ID 比较 policy_action、action、human_action_mask 和 reward，
相同才输出 matching_id_and_action_mask_reward=True。以下是一条实际 CPU 记录及比较结果：

.. literalinclude:: ../examples/rl-intervene-output.txt
   :language: text
   :start-after: # start correction
   :end-before: # end correction

假环境动作只有三维，不能照搬到七维真机；注入动作是 [0.25,-0.25,1]，mask 全真。
第三章的 learner/received=6、intervened_received=2、demo_buffer_size=2 与两个纠正步骤吻合。
所有六条进 online replay，两条人工纠正额外进 demo replay；demo 不是从 online 中搬走的分区。
真实训练还有种子示范，所以 demo_buffer_size 不一定等于 intervened_received，满容量后也不再增长。

**如果不对**：matching record missing 时核对比较的两份数据库来自同一次练习；
张量断言失败时保留原文件和 ID 排查，不能手动改 action 使结果通过；
观察到策略和人工动作恰好相同并不证明没接管，应以 human_action_mask 判定。

.. note::

   **本章哪些内容需要真机确认**

   实体接管时刻、方向、触发器锁存、交还后的策略运动以及动作请求与实测 TCP 的对应需现场确认。
   CPU 例子验证了同一 ID 的动作／mask／reward 一致，未验证 USB、串口、网络时序或真实操作员动作。

检查清单：能指出一次接管与交还；找到提案、执行请求和 mask；
在 learner journal 找到相同 ID，并在后续 actor 记录中找到新策略版本。

下一章 :doc:`rl-resume`。
