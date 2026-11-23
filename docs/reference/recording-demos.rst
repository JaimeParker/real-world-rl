.. _recording-demos-recording-demos-on-the-real-robot:

在真实机器人上录制示范
================================================================================

``rwrl record``\ （``examples/record_real_demos.py`` 是同一入口）把真机遥操作的每个回合写进一个示范数据集目录（``--output-dir``\ ）：每个回合一个子目录，回合结束时写入，内存里始终只有正在录的那一个。失败、重录和中断的回合也存下来，标上原因。训练命令的 ``--demo-dataset-paths`` 直接读这个目录（\ ``hil_serl``\ 、\ ``residual_hil_serl``\ 、\ ``silri``\ 、\ ``real_world_bc``\ ），取其中保留回合的人工步骤，图像历史按训练运行的 ``obs.frame_stack`` 重建；旧 pkl、``.trajectory.sqlite3`` 和 ``.trajectory.pt`` 仍可加载。ACT 轨迹训练器和 ``deploy-act`` 仍读 pkl 加 sidecar，需要先用 ``rwrl dataset export --format pkl`` 导出（见下文\ :ref:`导出 <recording-demos-export>`\ ）。残差训练加载这些录制时还需 ``--allow-base-less-demos``，因为录制器不计算 ``base_actions``/``next_base_actions``；见 :doc:`残差 RL <residual>`。实现位于 ``real_world_rl/demos/record.py``\ 。

录制器与 HIL actor 使用完全相同的环境构建方式（\ ``build_hil_env(args, role="actor")``\ ），因此接受与 ``train_real_world.py hil_serl`` 相同的环境、遥操作、观测参数和 ``--config`` 预设。请采用后续加载示范的运行参数：录制的观测和动作必须与其环境匹配（加载时检查数据集头 ``dataset.json`` 的 ``action_contract``\ ；对旧 pkl 则检查其契约附属文件）。\ :doc:`教程 08 <../tutorials/record>`\  讲解首次录制，\ :doc:`操作员指南 <intervening>`\  是操作员指南（按键、接管起始和记录内容）。

录制器默认 ``episode_end_behavior: confirm``\ ：每个回合在 reset 后由操作员按空格开始，结束后手臂停在原处，夹爪由操作员检查，再按 ``n`` 才复位（\ :ref:`回合边界 <recording-demos-episode-boundaries>`\ ）。受监督的 \ :ref:`rwrl teleop run 流程 <spacemouse-commissioning-supervised-recording-sessions>`\  也用 confirm；仍支持同一轨迹内反复接管/交还。只有 ACTIVE 里的步骤被记录，首个归档帧来自按下空格那一刻的新观测，图像堆叠历史也用该新帧重新填充。训练命令默认仍为 ``auto``\ ，单人版 ``hold`` 需显式选择。

使用 \ :doc:`SpaceMouse 验收清单 <testing/spacemouse-control-session>`\  及其记录模板，验证两种模式的方向、工作区、保持和正常停止。仅成功示范不能证明硬件通过验收或正常停止稳定。

.. _recording-demos-commands:

命令
--------------------------------------------------------------------------------

GELLO 由机器人 daemon 提供，follower 为 ``joint_delta``\ （守护进程提供的关节空间路线），按键使用键盘：

.. code-block:: bash

   rwrl record --config hil.yaml \
       --teleop-device leader_arm --robot-teleop-action-mode joint_delta \
       --teleop-event-device keyboard --teleop-onset arm \
       --output-dir demos/task_20 --num-successes 20

``joint_abs`` 数据集（用于 ``joint_abs`` 流程）需增加 ``--real_robot.action_mode joint_abs --robot-teleop-action-mode joint_abs``\ ，并在预设中提供 ``joint_limits``\ 。

``hil.yaml`` 是训练预设 \ |reference-link-1|_\  的副本；命令行不能表达数值对列表，因此 ``joint_limits`` 必须来自预设。不是录制器字段的预设键（算法超参数）会被跳过。

其它运动设备使用方式相同：本机 GELLO 用 ``--teleop-device gello`` 和 ``--teleop-options-yaml gello.yaml``\ ；GELLO 驱动 ``cartesian_delta`` 臂（\ ``real_robot``\ ）用 ``--teleop-device leader_cartesian``\ （\ |reference-link-2|_\  是单臂 50 Hz 预设，tracker 参数 ``--teleop-lead-cap-m``\ 、\ ``--teleop-lead-cap-rad``\ 、\ ``--teleop-target-speed-m-s``\ 、\ ``--teleop-target-speed-rad-s`` 见 \ :ref:`teleop.md <teleop-leader-arm-on-a-cartesian-robot-leader_cartesian>`\ ）；SpaceMouse 驱动 ``cartesian_delta`` 臂用 ``--teleop-device spacemouse --teleop-event-device stdin``\ 。\ ``leader_cartesian`` 或 ``spacemouse`` 录制的示范在实验契约（\ ``teleop_control``\ ）中包含 tracker，因此部署时若 ``--teleop-device`` 不带 tracker，会拒绝使用这些示范训练的策略。GELLO 设置见 \ :doc:`gello.md <gello>`\ ，设备说明见 \ :doc:`teleop.md <teleop>`\ 。

SpaceMouse tracker 模式参数（默认 ``--spacemouse-mode tracker``；详情见 \ :ref:`teleop.md <teleop-spacemouse>`\ ）：

.. list-table::
   :header-rows: 1

   * - 参数
     - 默认值
     - 含义
   * - ``--teleop-device spacemouse``
     - 
     - 本机 SpaceMouse；按钮 0 接管/交还，按钮 1 切换夹爪（从最后命令开始），手柄输入速度
   * - ``--teleop-event-device stdin``\ （或 ``keyboard``\ ）
     - 
     - 结果键 ``s``/``f``/``r``\ 、Esc、\ ``q``\ ；\ ``t``/``h`` 被忽略（按钮 0 才是切换键）
   * - ``--teleop-target-speed-m-s``, ``--teleop-target-speed-rad-s``
     - 0.05, 0.3
     - 手柄满幅偏转时的速度
   * - ``--teleop-lead-cap-m``, ``--teleop-lead-cap-rad``
     - 0.05, 0.3
     - 命令领先机械臂的上限；释放手柄后在此范围内停止
   * - ``--real-robot.xyz-scale-m``, ``--real-robot.rot-scale-rad``
     - 0.02, 0.1
     - 必须至少等于领先上限：传入 0.05 和 0.3
   * - ``--teleop-options-yaml`` 中的 ``device_index``, ``deadzone``, ``invert_axes``
     - 0, 0.05, ``[]``
     - SpaceMouse 序号、每轴静止噪声死区（满幅比例）、要翻转的输出轴（0–5）

``--teleop-event-device keyboard``\ （本机桌面会话）和 ``--teleop-event-device stdin``\ （运行录制器的终端，也支持 SSH）的操作键：\ ``t`` 接管，\ ``h`` 交还，\ ``s`` 成功，\ ``f`` 失败，\ ``r`` 重录，Esc 结束回合，\ ``q`` 结束回合并在处理后停止录制器；confirm（默认）下回合之间还用空格（开始）、\ ``n``\ （reset）、\ ``o``/``c``\ （夹爪）和 Enter（夹爪检查确认），见\ :ref:`回合边界 <recording-demos-episode-boundaries>`\ 。

双臂（\ ``--env-backend real_robot_bimanual``\ ）：\ ``--teleop-device bimanual --teleop-arm-device leader_arm``\ （或 ``gello``\ 、\ ``robot``\ ）每臂读取一个 leader，录制器逐臂处理。保持动作属于各臂自身（\ ``joint_abs`` 中为各 follower 关节在自身限位下的值），回合开始检查逐臂比较 leader 与 follower 并指出差距最大的臂，\ ``--max-episode-steps`` 替换 ``--real_robot_bimanual.max_episode_steps``\ 。任一操作员介入都会记录该步；另一臂保持不动。命令见 \ :ref:`bimanual.md <bimanual-teleop-and-demo-recording>`\ 。

然后用示范训练：

.. code-block:: bash

   rwrl train hil_serl --role learner --config hil.yaml \
       --demo-dataset-paths demos/task_20 ...

.. _recording-demos-recorder-flags:

录制器参数
--------------------------------------------------------------------------------

.. list-table::
   :header-rows: 1

   * - 参数
     - 默认值
     - 含义
   * - ``--output-dir``
     - 必填
     - 示范数据集目录。目录不存在时新建；已是数据集时追加，要求数据集头与本次环境一致，否则报错并要求换新的目录。以 ``.pkl`` 结尾或指向文件会被拒绝
   * - ``--num-successes``
     - 20
     - 数据集中保留的回合达到指定数量后停止；追加时已保留的回合也计入
   * - ``--max-episodes``
     - 未设置
     - 完成指定次数尝试后停止，包括失败、重录和超时；该上限与 ``--num-successes`` 中先满足者结束录制
   * - ``--max-episode-steps``
     - 未设置
     - 回合步数上限；设置时替换 ``--real_robot.max_episode_steps``\ 、\ ``--real_robot_bimanual.max_episode_steps``
   * - ``--episode-end-behavior``
     - ``confirm``
     - 回合结束后：\ ``confirm`` 操作员按键复位并开始，\ ``hold`` 单人、等中立确认的 ``n``\ ，\ ``auto`` 立刻 reset；见\ :ref:`回合边界 <recording-demos-episode-boundaries>`
   * - ``--gripper-open-fraction``\ 、\ ``--gripper-closed-fraction``
     - 0.95、0.05
     - confirm：\ ``n`` 时夹爪宽度占最大宽度的比例至少 open 算全开、至多 closed 算全合，直接 reset；介于两者之间进入夹爪检查
   * - ``--start-tolerance-rad``
     - 0.8
     - ``joint_abs``\ ：任一关节的 leader 与 follower 差值超过此值时拒绝开始回合；confirm 下改为锁住空格，终端列出差得多的关节，对齐后才能开始
   * - ``--keep-syncing-steps``
     - 关闭
     - 将 leader 追赶行作为普通监督步纳入训练行；数据集始终保留 syncing 标记，这一选择写入数据集头，训练加载时沿用
   * - ``--record-video-dir``
     - 不可用
     - 录制器拒绝此参数（训练仍可用）：数据集已在每个回合目录里写 ``video/<rgb 键>.mp4`` 回看视频
   * - ``--reward-from-teleop-events``
     - 开启（仅录制器；训练默认关闭）
     - 成功键将该步保存为终止成功（奖励 ``--teleop-success-reward``\ ，1.0；\ ``done`` 为 True）并结束回合；\ ``--no-reward-from-teleop-events`` 保留环境/任务/分类器自身的奖励与终止；裸 ``real_robot`` 环境为奖励 0、``done`` 为 False

``--spacemouse-mode serl`` 改用原始运动/按钮接管，释放后交还控制，按住按钮控制夹爪；
需要配置 SERL xyz/RPY 安全盒。上述 tracker 领先上限和目标速度规则不适用于此模式。

``--control-hz`` 像对 actor 一样控制循环频率。不支持 ``--fwbw``\ 。

.. _recording-demos-episode-boundaries:

回合边界：confirm、hold 与 auto
--------------------------------------------------------------------------------

``--episode-end-behavior``\ （配置键 ``episode_end_behavior``\ ）决定一个回合结束后发生什么：

.. list-table::
   :header-rows: 1
   :widths: 12 40 24 24

   * - 值
     - 回合结束后
     - 默认用在
     - 要求
   * - ``confirm``
     - 手臂停在原处；操作员按键处理夹爪、请求 reset、开始下一回合（本节余下部分）
     - ``rwrl record``、\ ``rwrl teleop run``\ ；actor 和评估也支持
     - 一个事件设备；单臂或双臂，笛卡尔或关节动作
   * - ``hold``
     - 手臂停在原处，等一个经过中立检查的 ``n`` 再 reset；一个人操作
     - 需要显式选择
     - 单臂笛卡尔 ``real_robot``，运动设备能报告中立读数（SpaceMouse、\ ``leader_cartesian``\ ）
   * - ``auto``
     - 立刻 reset，不等任何按键
     - 训练命令（\ ``hil_serl`` 等）
     - 无

``confirm`` 适合两个人：操作员 A 在运行命令的终端按键，操作员 B 遥操作。回合分成六个阶段，进入每个阶段时打印一行 ``<阶段>: ...``\ （stderr），同时写一条会话事件（\ ``session-events`` 文件）。下面是一次完整运行的原样输出，夹爪宽度和关节名随现场不同：

.. literalinclude:: ../examples/record-output.txt
   :language: text
   :start-after: # start confirm
   :end-before: # end confirm

.. list-table::
   :header-rows: 1
   :widths: 20 40 40

   * - 阶段
     - 手臂与记录
     - 按键
   * - ``RESETTING``
     - 手臂回 home，之后夹爪张开。不记录；reset 期间按的键被丢弃
     - 无
   * - ``READY``
     - 手臂停在 reset 后实测的位姿，遥操作运动被忽略。不记录
     - ``空格`` 开始回合；\ ``q`` 退出
   * - ``ACTIVE``
     - 遥操作，逐步记录；回合第 0 行是按下空格时重新读取的观测
     - ``s``/``f``/``r``/``Esc`` 判定并结束回合（下表）；\ ``q`` 判定后退出；\ ``t``/``h`` 照常接管／交还
   * - ``POST_EPISODE``
     - 笛卡尔臂重复发送最后一个被接受的目标，关节臂保持实测关节；夹爪不变，遥操作运动被忽略。不记录
     - ``o`` 张开、\ ``c`` 合上夹爪（阶段不变）；\ ``n`` 请求 reset；\ ``q`` 退出
   * - ``GRIPPER_CHECK``
     - 同 ``POST_EPISODE``
     - ``Enter`` 马上 reset；\ ``o`` 在原地张开，回到 ``POST_EPISODE``\ ；其它键回到 ``POST_EPISODE``\ ；\ ``q`` 退出
   * - ``STOPPING``
     - 释放控制，已结束的回合已在磁盘上
     - 无

空格、\ ``n``\ 、\ ``o``\ 、\ ``c``\ 、Enter 只在上表所列阶段起作用，ACTIVE 里按下没有效果。每个等待周期都重发保持目标、检查机械臂健康和步骤看门狗，再喂监督心跳；这段时间不调用 ``env.step``\ ，所以等待不进轨迹，也不写数据集。

**夹爪检查。** ``n`` 时读夹爪宽度占最大宽度的比例：至少 ``gripper_open_fraction``\ （全开）或至多 ``gripper_closed_fraction``\ （全合）就直接 reset；介于两者之间，夹爪可能夹着物体，进入 ``GRIPPER_CHECK``\ 。双臂时列出每个半合的夹爪，\ ``o`` 只张开这些。读不到宽度的夹爪（没有宽度反馈）总是询问，提示里写 ``no width feedback``\ 。reset 先回 home，再张开夹爪，所以 Enter 后物体在 home 处掉下；要在原地放下，先按 ``o``\ 。

**预算用完。** 成功配额或 ``--max-episodes`` 满足后，最后一个回合仍先进入 ``POST_EPISODE``\ （o/c 可用），提示 ``n or q = end the session (no reset)``\ ；\ ``n`` 或 ``q`` 结束会话，不 reset。\ ``q`` 在任何阶段都有效。

**起始锁（\ ``joint_abs``\ ）。** 在 ``READY``\ ，任一主臂关节离从臂超过 ``start_tolerance_rad`` 时空格被锁住，终端列出差得多的臂和关节（只在列表变化或按了空格时刷新，至多每 0.5 秒一行），对齐后才能开始，不会抛错退出：

.. literalinclude:: ../examples/record-output.txt
   :language: text
   :start-after: # start lock
   :end-before: # end lock

**事件设备。** confirm 需要一个事件设备，否则启动时（任何 reset 之前）报错：``--teleop-event-device keyboard|stdin|jsonl``\ ，或 GELLO 的 evdev ``event_device``\ 。机械臂主臂部件读取的设备（\ ``leader_arm``\ 、\ ``leader_cartesian``\ ，经机器人 daemon）要等机器人连上才知道有没有事件设备，此时只打印一行 ``CONFIRM: ...`` 提示：空格和 ``n`` 没反应时改传 ``--teleop-event-device stdin``\ 。各设备的键名：

.. list-table::
   :header-rows: 1
   :widths: 20 80

   * - 设备
     - 空格 / Enter / 其它
   * - ``keyboard``\ （pynput，本机桌面）
     - 空格键和 Enter（含小键盘 Enter）；字母按小写；Esc
   * - ``stdin``\ （运行命令的终端，可经 SSH）
     - 空格和 Return（CR 或 LF，CRLF 算一次）；字母按小写；单独的 Esc，方向键、功能键序列被跳过。终端进入 cbreak 模式，按住键会重复
   * - ``jsonl``\ （\ ``--teleop-events-file``\ ，\ ``teleop run --events-file``\ ）
     - 事件名 ``start`` = 空格、\ ``enter``\ 、\ ``open`` = o、\ ``close`` = c、\ ``next`` = n、\ ``quit`` = q，以及原有的 ``takeover``\ 、\ ``handback``\ 、\ ``success``\ 、\ ``failure``\ 、\ ``rerecord``\ 、\ ``end``\ ；每次 poll 读一行
   * - GELLO ``event_device``\ （evdev）
     - 事件 ``start_episode``/``next_episode``/``gripper_open``/``gripper_close``/``confirm``/``operator_quit``\ ，默认键见 :doc:`gello`

其它设备不产生这些键：只有运动、没有事件设备的 ``spacemouse``\ 、\ ``leader_arm`` 等，要另配上面之一。

**回合判定键（ACTIVE）。**

.. list-table::
   :header-rows: 1
   :widths: 20 80

   * - 键
     - 结果
   * - ``s``
     - 成功：保留（至少有一个人工步）；之后进入 ``POST_EPISODE``
   * - ``f``
     - 失败：标为 discarded（\ ``failure event``\ ），不保留
   * - ``r``
     - 重录：标为 discarded（\ ``rerecord``\ ）。confirm 下之后也进入 ``POST_EPISODE``\ ，下一回合要 ``n`` 和空格才开始
   * - ``Esc``
     - 不判定就结束：discarded（\ ``no success``\ ）
   * - 步数上限
     - 截断，不算成功：discarded（\ ``no success``\ ）
   * - ``q``
     - 该回合照常判定，随后会话进入 ``STOPPING``

标错了不用重录：\ ``rwrl dataset drop`` 把保留的回合标为 discarded（\ ``manual``\ ），\ ``rwrl dataset undrop`` 恢复，见\ :ref:`复查与维护 <recording-demos-review-and-maintenance>`\ 。

**选项。**

.. list-table::
   :header-rows: 1
   :widths: 30 12 58

   * - 参数
     - 默认值
     - 含义
   * - ``--episode-end-behavior``
     - 录制 ``confirm``\ ，训练 ``auto``
     - 见上表；\ ``rwrl record`` 默认 confirm，训练命令默认 auto
   * - ``--gripper-open-fraction``
     - 0.95
     - confirm：夹爪宽度占最大宽度的比例至少为此值算全开，\ ``n`` 时不问
   * - ``--gripper-closed-fraction``
     - 0.05
     - confirm：比例至多为此值算全合，\ ``n`` 时不问。须满足 0 ≤ closed < open ≤ 1，否则启动报错
   * - ``--start-tolerance-rad``
     - 0.8
     - 录制器的 ``joint_abs``\ ：confirm 下它是起始锁的阈值（空格锁住，不报错）；auto/hold 下仍是拒绝开始的差值。actor 和评估的 confirm 同样使用该值，默认 0.8
   * - ``--neutral-threshold``\ 、\ ``--neutral-stable-s``
     - 0.05、0.2
     - 只用于 ``hold``\ ；confirm 不看中立

**什么时候用哪一个。** 录示范用 confirm（默认）。强化学习训练默认 auto，回合结束立刻 reset，learner 照常训练；任务需要人把场景摆回去（放回物体、换物体）时，改成 confirm：actor 在 ``POST_EPISODE``/``READY`` 等人，期间不推送轨迹，learner 继续用已有数据训练。actor 和评估把 ``episode_end_behavior: confirm`` 当作任何 HIL 配置字段使用；起止键与本节相同，评估跑完最后一个回合后停在 ``POST_EPISODE``\ （不 reset）。hold 只在只有一个人、手上有 SpaceMouse 且是笛卡尔臂时有意义。

**还没在真机上验证的地方。**

- 关节模式的保持用实测关节：机械臂上没有被接受的关节目标可重发，控制器在当时的位置继续跟踪，每步关节限幅约束任何差值。
- Franka Hand 的开合是异步的：按 ``o`` 后立刻 ``n`` 可能仍读到半合而再问一次。
- 事件设备在机器人 PC 的 daemon 上时，旧 daemon 不转发新键，A 会一直停在 ``READY``\ ；升级机器人 PC 上的 daemon。
- 夹爪宽度反馈依赖夹爪后端；读不到时按「总是询问」处理，不会自动开爪。

.. _recording-demos-what-happens-in-an-episode:

回合内的处理流程
--------------------------------------------------------------------------------

1. 环境 reset（机器人回 home，夹爪打开）。confirm 下 reset 之后还要操作员按空格（\ ``READY``\ ）才开始记录，见\ :ref:`回合边界 <recording-demos-episode-boundaries>`\ 。
2. 仅 ``joint_abs``\ （auto/hold；confirm 用起始锁，不会报错停止）：录制器读取一次 leader，与 follower 的 ``joint_pos`` 比较。任一关节差值超过 ``--start-tolerance-rad`` 时停止并给出逐关节误差：先将 leader 对齐 follower 位姿再重启。低于容差时由起始策略（\ ``--teleop-onset``\ ）处理接管：\ ``ramp`` 每步以 ``--teleop-ramp-step-rad`` 将 follower 移向 leader，\ ``arm`` 等待 leader 进入 ``--teleop-arm-tolerance-rad``\ 。读取通过未激活接管开关的运动设备进行，因此起始策略不会隐藏差距，也不会消费事件设备按钮。
3. 每步录制器发送保持动作；操作员介入时，介入包装器改为执行操作员动作。保持动作将机器人留在原处：\ ``joint_delta``\ （以及双臂 ``cartesian_delta``\ ）为零机械臂动作，\ ``joint_abs`` 为 follower 当前关节位置，单臂 ``cartesian_delta`` 则为返回固定保持位姿的动作（reset 后及每次交还时采样；\ ``leader_cartesian`` 使用输入源自身 tracker，见 \ :ref:`teleop.md <teleop-mapping>`\ ），避免机械臂漂移。距保持位姿超过一步（\ ``--real-robot.xyz-scale-m``/``--real-robot.rot-scale-rad``\ ）时发送饱和动作，逐步返回。夹爪维度保持最后执行的夹爪动作（reset 后为打开），每臂一个：夹爪是最后 ``num_arms`` 个动作维度（动作契约布局）。\ ``run_recorder(args, hold_action_fn_factory=...)`` 可注入其它保持策略（以构建后的环境调用）。
4. 每步都写入数据集（无人介入的保持步也在）。训练视图选择介入步；两种关节模式的 leader 追赶均带 ``teleop_syncing``\ ，除非设置 ``--keep-syncing-steps``\ ，否则排除。数据集派生训练视图仍排除 syncing。人工掩码选择专家维度，空闲保持补充动作没有专家标签。
5. 环境终止或截断时回合结束：操作员的回合结束按钮（键盘 Esc）截断，步数上限截断，成功键终止（默认 ``--reward-from-teleop-events``\ ），奖励分类器终止。仍在介入时按成功键：仅介入步进入示范记录，因此终止步也必须是介入步。

回合结果来自操作员事件（任意事件路径：键盘、终端、GELLO evdev 设备；SpaceMouse 按钮不产生这些事件）：

.. list-table::
   :header-rows: 1

   * - 事件
     - 作用
   * - ``success``\ （\ ``s``\ ）或显式环境/分类器成功标签
     - 回合计为成功，除非失败标签否决；仅正奖励不是成功标签
   * - ``failure``\ （\ ``f``\ ）
     - 回合标为 discarded（``failure event``\ ），即使先前已有成功标签
   * - ``rerecord``\ （\ ``r``\ ）
     - 立即结束回合并标为 discarded（``rerecord``\ ），开始下一回合（confirm 下进入 ``POST_EPISODE``\ ，下一回合要 ``n`` 和空格）

每个回合都存入数据集；只有至少有一个人工步的成功回合标为 kept，其余标为 discarded 并带原因（下表）。每个完成回合打印其结果。Ctrl-C 停止会话：已写入的回合不受影响，被打断的那个回合只要已有一步，就存为 discarded（``interrupted``\ ），在环境释放机器人之后写入；\ ``q`` 键（\ ``operator_quit``\ ）结束当前回合，照常判定结果，然后以同样方式停止会话。

.. list-table::
   :header-rows: 1

   * - discard_reason
     - 含义
   * - ``rerecord``
     - 操作员按了重录键
   * - ``failure event``
     - 回合内出现失败事件
   * - ``no success``
     - 回合结束但没有成功结果（例如 Esc 结束或步数上限）
   * - ``no intervened steps``
     - 成功，但没有任何人工介入步，不能作示范
   * - ``interrupted``
     - Ctrl-C 或错误打断了回合
   * - ``manual``
     - 事后用 ``rwrl dataset drop`` 丢弃

``joint_delta`` 跳过开始检查：leader 读取每步在一个 ``joint_scale_rad`` 处饱和，无法测量更大的差距；相同饱和限制已经约束 follower 向 leader 运动的速度。

.. _recording-demos-output:

输出
--------------------------------------------------------------------------------

``--output-dir`` 是一个数据集目录，每个回合一个子目录：

.. code-block:: text

   demos/peg/
     dataset.json                 数据集头
     episode_000000/
       meta.json                  这个回合的元数据
       data.npz                   逐步数组
       frames/rgb_wrist.npz       每个相机键一个无损图像数组
       video/rgb_wrist.mp4        回看视频，只给 rgb_* 键，帧率为 control_hz
     episode_000001/

- ``dataset.json`` 记录格式版本、``control_hz``\ 、任务描述、动作约定（``action_contract``\ ，即原先 ``.contract.json`` 的内容）、每个观测键的单帧形状 ``obs_spec``\ 、具名状态 ``named_state_spec``\ 、动作维数、录制时的 ``recorded_frame_stack`` 和录制配置快照（令牌类字段不写入）。追加到已有数据集时，动作约定、``obs_spec``\ 、``named_state_spec``\ 、动作维数和 ``control_hz`` 必须一致；不一致则报错并列出不同的键。
- ``meta.json`` 记录 ``status``\ （``kept`` 或 ``discarded``\ ）、``discard_reason``\ 、``success``\ 、步数、人工介入步数、介入／保持区间 ``segments``\ 、事件、起始时间和时长；还有操作员备注和 drop／undrop 历史。
- ``data.npz`` 的观测类数组有 T+1 行（第 t 行是第 t 步之前的观测），步骤类数组有 T 行：实际执行的 ``action``\ 、策略提议 ``policy_action``\ 、``human_action_mask``\ 、``override_mask``\ 、``intervened``\ 、``reward``\ 、``terminated``\ 、``truncated``\ 、``syncing``\ ，以及每个出现过的操作员事件。``stack_reset`` 标出帧堆叠历史被重新填充的行。
- 回合序号只增不减：``dataset.json`` 的 ``next_index`` 记录下一个序号，``prune`` 也不会重新编号。

每个回合先写进 ``.tmp-episode_...`` 临时目录，写完并落盘后整体改名为 ``episode_NNNNNN``\ ，所以读取方只会看到完整的回合；崩溃留下的临时目录在下次打开数据集时清掉并记一行日志。训练视图（``real_world_rl.dataset.iter_transitions``\ ）取保留回合里的介入步（默认不含 syncing），行内容与旧 pkl 相同：``obs``\ 、``next_obs``\ 、实际执行的 ``action``\ 、``reward``\ 、``done``\ （\ ``terminated``\ ）、可选 ``grasp_penalty``\ 、策略提议、人工／覆盖掩码，以及回合／步身份和训练片段边界，因此过滤不会虚构相邻关系或将终止奖励转移到更早的人工步。参与比较的实验字段覆盖任务、控制、观测布局、预处理和遥操作部件选择／空闲行为；奖励来源和工作站所属 reset 事实属于溯源。完整恢复、掩码和训练视图规则见\ :doc:`真实机器人实验 <real-world-experiments>`\ 。

.. warning::

   图像按无损保存，磁盘占用远大于旧 pkl。224×224 的 RGB 一帧约 150 KB（每个相机每个控制步一帧，压缩前），10 Hz 下每相机约 1.5 MB／秒；``image_size`` 越大占用越多。教程和示例任务推荐 224×224，资源紧张时可用较小尺寸，录制、训练和部署须一致。

.. _recording-demos-review-and-maintenance:

复查与维护
--------------------------------------------------------------------------------

``rwrl dataset`` 离线工作，不需要站点或机器人（``export --format pkl`` 需要 ``learning`` extra，见下）：

.. list-table::
   :header-rows: 1

   * - 命令
     - 作用
   * - ``rwrl dataset list DIR``
     - 回合表：状态、原因、是否成功、步数、人工占比、时长、备注数，末尾汇总保留的 transitions
   * - ``rwrl dataset view DIR [-e N ...] [--web] [--save x.rrd]``
     - 用 Rerun 查看器回放，需要 ``viewer`` extra（``pip install -e '.[viewer]'``\ ）。每个回合是一条独立 recording，在查看器的 recording 列表里选；相机画面与关节、TCP、夹爪和动作曲线共用时间轴，并标注介入区间、事件和状态。``-e`` 可重复，只看部分回合；``--web`` 起网页查看器，远程机器上通过 ssh 转发网页端口（默认 9090）和 gRPC 端口（默认 9876）后访问，端口可用 ``--web-port``\ 、``--grpc-port`` 改；``--save`` 只写 ``.rrd`` 文件，不开查看器
   * - ``rwrl dataset drop DIR N [N ...] [--note TEXT]``
     - 把保留的回合标为 discarded（原因 ``manual``\ ），只改 ``meta.json`` 并记入历史，可逆
   * - ``rwrl dataset undrop DIR N [N ...]``
     - 恢复被 drop 的回合；录制时就已 discarded 的回合不能恢复，会报错
   * - ``rwrl dataset prune DIR --yes``
     - 删除所有 discarded 回合的目录，不可逆；不带 ``--yes`` 会列出将删除的回合并以用法错误退出。回合序号不重排

``archive inspect`` 同样直接读数据集目录，见下文\ :ref:`单次尝试上机验收 <recording-demos-single-attempt-commissioning>`\ 。

.. _recording-demos-export:

导出
--------------------------------------------------------------------------------

.. code-block:: bash

   rwrl dataset export demos/peg --format pkl --out demos/peg.pkl
   rwrl dataset export demos/peg --format lerobot --out demos/peg-lerobot

- ``--format pkl`` 写出训练视图的 pkl 和同名 ``.contract.json``\ 。rl-garden 的 ACT 轨迹训练器和 ``deploy-act`` 仍需要 pkl 加 sidecar，教程里需要 pkl 的辅助脚本也要先走这一步。可用 ``--frame-stack N`` 按训练的 ``obs.frame_stack`` 叠帧，``--keep-syncing-steps`` 保留追赶步（导出不会自动沿用录制时的选择）。导出会把全部训练行读入内存。需要 ``learning`` extra。
- ``--format lerobot`` 写出 LeRobot v3.0 数据集（需要 ``lerobot`` extra：``pip install -e '.[lerobot]'``\ ），只含保留回合，每个控制步一帧：``observation.state``\ 、具名状态字段、``observation.images.<相机>``\ （视频）、``action``\ 、``intervened`` 和 ``human_action_mask``\ 。深度键不导出，会列出被跳过的键。RGB 帧的高和宽必须为偶数；输出目录必须不存在。``--fps`` 默认取 ``control_hz``\ （须为整数），``--repo-id`` 默认取输出目录名，``--vcodec`` 默认 ``h264``\ （``hevc``\ 、``libsvtav1`` 可选；AV1 在很小的画面上会崩溃）。

.. _recording-demos-single-attempt-commissioning:

单次尝试上机验收
--------------------------------------------------------------------------------

使用 ``--max-episodes 1`` 在一次尝试后结束，即使未保留成功示范。录制器在写入被打断的回合之前先关闭设备控制。成功、失败、重录、超时和中断的回合都存入数据集，只有成功人工示范标为 kept。重启命令会开始新的 reset，包括回 home 和打开夹爪。每次重启前核实工作站与工作区。默认没有尝试上限，会继续到数据集中保留的回合达到 ``--num-successes``\ 。

``rwrl archive inspect <DIR>``\ （离线，无需工作站或机器人；也接受旧的 SQLite 归档）只读访问数据集；对于带 ``tcp_pose`` 的单臂状态，按 human_action_mask 片段报告保持漂移（也报告前 ``--drift-window-s`` 内的漂移）、残差、接管跳变（毫米），以及每个 ``--stop-time-s`` 后的稳定时间；\ ``--measurements PATH`` 将这些结果写为测试页的 ``measurements.csv`` 行，\ ``--csv PATH`` 导出逐步数据。带 ``--stop-time-s`` 时，所有停止都已稳定则退出 0，有一次未稳定退出 3，某个停止时间没有覆盖它的 TCP 回合则退出 4（\ :ref:`cli.md <cli-commands>`\ ）。\ ``python examples/inspect_teleop_archive.py`` 是其别名。

.. note::

   较早的测试记录（\ :doc:`硬件测试清单索引 <testing/index>`\ 下的会话页）用旧录制器录成，命令里是 ``--output-path`` 和 ``.pkl``\ ；现在的写法是上面的 ``--output-dir``\ 。

英文主版本测试页、记录 CSV 约定和证据要求见\ :doc:`硬件测试清单索引 <testing/index>`\ 。硬件结果需在现场填写。

.. |reference-link-1| replace:: ``examples/presets/hil_single_arm.yaml``
.. _reference-link-1: https://github.com/JaimeParker/real-world-rl/blob/main/examples/presets/hil_single_arm.yaml

.. |reference-link-2| replace:: ``examples/presets/gello_cartesian.yaml``
.. _reference-link-2: https://github.com/JaimeParker/real-world-rl/blob/main/examples/presets/gello_cartesian.yaml
