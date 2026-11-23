启动 HIL-SERL 训练
================================================================================

本章按设备服务、learner、actor 的顺序启动，再用记录验证数据与参数的两个方向。
所有命令从仓库根目录运行，使用各机器已安装 learning 的环境；设备服务只需设备侧依赖。
配置沿用 :doc:`rl-task` 的 ``local/rl-task.yaml``。下面的真机命令没有在本地执行。

.. tip::

   本章设备命令省略了 ``--station``：从仓库根目录运行时，会用 :doc:`station` 第 10 步 ``rwrl use`` 记住的
   station；没记住时也会直接找到 ``local/station.yaml``。要换文件，在命令里显式写 ``--station``。

1. 先知道第一次执行的策略来自哪里
--------------------------------------------------------------------------------

开始前先确认初始化方式，决定操作员何时接管。
单夹爪任务构造 ``RLPDHybrid``：六维连续机械臂动作和一个离散夹爪头。
无夹爪契约使用 ``ContinuousHilRLPD``。本章不使用前面 BC 的 final.pt 自动初始化。

learner 从新网络开始，把 ``--demo-dataset-paths`` 的数据放入 demo buffer。
示范不会计入 ``learning_starts``；要收到本例 100 条在线 transition 才开始更新。
训练批次按 demo_data_ratio 混合在线和示范数据，在线人工纠正继续扩充 demo buffer。
这不是“先对示范训练一次 BC，再上线”的入口。

actor 也先构造同形状的小缓冲策略，再等待 learner 的初始快照覆盖权重。
初始快照可以是 ``phase=warmup``、``policy_ready=False``；HIL-SERL 的 actor
仍然调用策略 predict，不会因这个标记自动保持机械臂或等待 BC 完成。
首个训练更新后发布 ready 策略，后续按 publish_freq 发布。

**你应该看到**：新实验最早记录的 global_update=0，之后才有正值。
``--load-actor-checkpoint`` 只支持连续策略；hybrid 的机械臂 actor 与 DQN 夹爪
不兼容完整连续动作 BC 头，传入该参数会明确报错。恢复 HIL-SERL 训练使用第五章的完整 bundle。
**如果不对**：启动即有旧版本／旧 replay 时检查是否复用了 first-hil 的 checkpoint 目录；
把 BC 文件传给 --load-checkpoint 出现算法或权重不兼容时，选回 HIL-SERL bundle。

2. 在机器人 PC 启动设备服务
--------------------------------------------------------------------------------

先结束录制、部署和直接连接设备的进程，使用已审阅的 station 提供唯一硬件连接。
这里的 5 秒沿用本仓 daemon 示例；现场须按部署章测得的最长动作间隔与可接受停止延迟审阅。

.. warning::

   设备服务将持有真实硬件；稍后 actor reset 会归位并开爪，策略随后可能立即运动。
   确认 home、工作区、限位、停止方式及现场监护。不要同时运行两个 actor。

.. literalinclude:: ../examples/rl-train.sh
   :language: bash
   :start-after: # start daemon
   :end-before: # end daemon

**你应该看到**：daemon 持续服务，监听端口与 rl-task.yaml 的 remote 一致。
此时不应有 learner 控制机械臂；不要再用直接 FCI 客户端打开同一机械臂。
本章没有伪造 daemon 启动成功日志；连接验收沿用 :doc:`deploy`。

**如果不对**：设备占用时关闭旧录制／部署进程；声明了 daemon 却不可达时核对 station 的
监听地址、端口与 token；watchdog 反复触发时回查部署章的时序预算及控制频率。

**可选直连路径**：若决定让 actor 直接持有硬件，先停止 daemon，再复制
local/rl-task.yaml 为 local/rl-direct.yaml；删除 real_robot.remote，
将已调试 station 的单臂机器人配置映射放入 real_robot.robot，不放站点路径，也不复制 daemon 块。
robot 与 remote 必须且只能选一个；两机使用同一份观测、动作、home、限位及 teleop 配置。
以下先只打印配置，再用 doctor 比较工位；doctor 会初始化设备，须保证没有其他占用者。

.. literalinclude:: ../examples/rl-train-alternatives.sh
   :language: bash
   :start-after: # start direct-check
   :end-before: # end direct-check

**你应该看到**：print-config 中有 robot 映射、没有 remote；doctor 的 <臂名>.preset 为 PASS。
硬件字段不同时会列出 <臂名>.preset:<字段> WARN 及 station／preset 两个值，
包括 IP、后端、夹爪、相机、home、限位（``limits``）和坐标系。

**如果不对**：逐项修复副本，不删除告警；解析成功不代表硬件可用。
随后把第 3 步两端命令的配置都换成 local/rl-direct.yaml，actor 用监督器启动：

.. warning::

   下面命令会 reset 并运行策略。直连 actor 是唯一硬件拥有者；先确认现场监护和停止手段。

.. literalinclude:: ../examples/rl-train-alternatives.sh
   :language: bash
   :start-after: # start direct-actor
   :end-before: # end direct-actor

监督器心跳超时默认 2 秒，首次心跳启动宽限默认 30 秒；这里显式用 60 秒容纳加载。
这不同于 actor 的 sync_startup_timeout，也不同于环境内的 StepWatchdog。
real_robot.watchdog_s 默认 None，原直连示例未开启；会话生成的配置可能保留录制时的值，须检查实际配置。
StepWatchdog 在同一进程内，不能覆盖整个进程挂死；daemon 的 watchdog 是另一条路径。
**你应该看到**：actor 收到初始快照后进入 reset／中立检查。
**如果不对**：首次心跳前退出就查加载和启动宽限；运行中丢心跳则检查阻塞原因，不能只延长启动时间。

3. 在训练机启动 learner，再在机器人 PC 启动 actor
--------------------------------------------------------------------------------

先启动 learner，让 actor 能在初始快照等待期限内连上。
训练机上的示范数据集目录、rl-task.yaml 及所引用 teleop 配置必须可读；remote 指向机器人 PC
上的服务，但 learner 只构造空间，不要求训练机打开这个连接。

.. literalinclude:: ../examples/rl-train.sh
   :language: bash
   :start-after: # start learner
   :end-before: # end learner

**你应该看到**：生成 local/rl-runs/first-hil 下的 config.json、experiment.json，
以及 checkpoints/journal.sqlite3；没有 actor 数据时 learner 等待，不应期待 loss 连续输出。
本入口把指标写入 TensorBoard，不提供固定的“learner ready”终端行。

**如果不对**：Unknown env backend 'real_robot' 时补完整 editable 安装／后端注册；
示范路径无匹配时可能得到空 demo buffer，先核对真实文件；契约错误时比较原会话，
尤其不要把 examples/presets/hil_single_arm.yaml 的 joint_delta 默认值覆盖前章 cartesian_delta。

接着在机器人 PC 设置训练机实际 LAN 地址。两者同机时设 127.0.0.1。
这里运行的 actor 使用第二章已选好的 SpaceMouse 或 GELLO 配置，不需要同时再运行 teleop run。

.. warning::

   下一条命令会 reset 并开始策略控制。初始策略未训练；操作者应准备立即接管，
   保持物理停止装置可用。n 会进入下一回合并再次 reset。

.. literalinclude:: ../examples/rl-train.sh
   :language: bash
   :start-after: # start actor
   :end-before: # end actor

**你应该看到**：先等待初始快照，reset 后出现 START_NEUTRAL；松开输入使中立检查通过。
以下是按 session.py 原样列出的硬件运行格式示例，并非本地真机实测：

.. literalinclude:: ../examples/rl-train-format.txt
   :language: text
   :start-after: # start hold
   :end-before: # end hold

活动回合里做少量人工纠正并标注；回合结束后停在 EPISODE_HOLD。
按 n 请求下一轮并松开输入，按 q 退出。learner 独立运行，需要在自己的终端 Ctrl-C 停止。

**如果不对**：``initial learner policy was not available before startup deadline`` 时检查 learner
及 6000 端口；默认 sync_startup_timeout 为 30 秒。START_NEUTRAL 不结束时回到输入设备章检查静置值／toggle。
``learner policy experiment hash differs from actor`` 时统一配置与引用文件，不删除校验绕过。

4. 按数据、摄入、参数的顺序检查
--------------------------------------------------------------------------------

先读 learner journal，再读指标；最后检查 actor 所用版本，才能证明一次更新回到控制端。

.. tutorial-test: skip
.. literalinclude:: ../examples/rl-train.sh
   :language: bash
   :start-after: # start inspect
   :end-before: # end inspect

.. tutorial-test: skip
.. literalinclude:: ../examples/rl-train.sh
   :language: bash
   :start-after: # start metrics
   :end-before: # end metrics

**你应该看到**：

* 采集：transition 的 step_index 增加，有 obs／next_obs、action、reward。检查器只打印关键字段，
  原始张量仍在 SQLite payload 中。episode_end 有本轮结果。
* 摄入：``learner/received``、``learner/buffer_size`` 增长；人工纠正还增加
  ``learner/intervened_received``、``learner/demo_buffer_size``。
  ``learner/admitted_records`` 包括回合边界记录，不等于 transition 数。
* 更新：``learner/update`` 大于 0；``learner/journal_backlog`` 表示已接收但未应用的积压。
  指标按收到样本数跨 log_freq 边界记录；没有新样本时，即使梯度更新继续，也未必写新标量。
* 回到 actor：后续 transition 的 ``policy_session_id``、``policy_version`` 和
  ``global_update`` 变化，policy_phase 从 warmup 变 online。这些字段在 actor 加载快照后写入，
  比只看 learner/update 更能证明 actor 已用新参数。

``actor/actor/outbox_depth``、``actor/actor/upload_retries`` 来自本例 actor_id=actor 的回合统计；
``transport/actor/outbox_depth`` 来自 episode_end。outbox ACK 后删除，不能长期用它查询全部历史。
参数版本只在同一个 learner session 内比较大小；重启 learner 后 session 变化，version 可以变小。

**如果不对**：received 不涨但 admitted_records 涨，检查 journal_backlog 和摄入错误；
received 未达 100 时先采集，不靠增加示范文件绕过门槛；只有 learner/update 变化时检查同步链路，
并等 actor 再执行一步；TensorBoard 没有标量时确认 log_type、目录和 log_freq 已跨过一次。

方法不同，损失名也不同。learner 指标按收到的 transition 数作横轴，
按 log_freq 边界记录（默认 1000），不是按秒或 global_update。
actor 回合统计在 learner 上加 ``actor/<actor_id>/`` 前缀，本章默认是 actor/actor/；
只有未提供 actor_id 的旧统计才直接用 actor/。

.. list-table:: 指标目录（按实际方法和数据条件出现）
   :header-rows: 1
   :widths: 55 45

   * - 指标
     - 怎样读
   * - actor/<actor_id>/ 下的 return、length、terminated、task_success、human_episode_end
     - 回合回报／长度、结束及明确结果；terminated 不等于任务成功。
   * - 同前缀的 autonomous_success_rate、task_success_rate、episode_intervention_rate、intervention_rate、intervened_steps、intervention_count
     - 区分自主结果、辅助结果、人工步骤及接管次数。
   * - 同前缀的 safety_clips、safety_clips_total
     - 设备有提供时，分别为本回合裁剪增量和累计计数；持续增长需查越界请求。
   * - 同前缀的 rtt_ms_mean、rtt_ms_max、step_time_ms_mean、step_time_ms_max、param_refresh_ms_mean、param_refresh_ms_max
     - 网络、step 和参数刷新时序；RTT 只在环境提供时出现。
   * - 同前缀的 outbox_depth、upload_retries
     - 上传积压与重试，不能拿重试次数当丢失行数。
   * - learner/received、learner/update、learner/buffer_size、learner/train_time_s、learner/demo_buffer_size、learner/intervened_received
     - 摄入、更新、容量及人工示范增长。
   * - losses/critic_loss、losses/qf_loss、losses/actor_loss、losses/alpha_loss、entropy/alpha、entropy/target_entropy、q/predicted
     - HIL-SERL／残差的 SAC 类指标，具体字段依连续／混合变体和更新阶段而异。
   * - losses/actor/residual_bc、losses/actor/intervened_fraction
     - 残差人工拟合项和 batch 介入比例；critic-only 阶段不会更新 actor。
   * - losses/critic/loss、losses/actor/loss、losses/actor/q、losses/actor/distance
     - SiLRI 的 critic、actor 及其与人工行为模型的距离。
   * - losses/lambda/loss、losses/lambda/mean、losses/behavior/sigma_sum、losses/batch/intervened_fraction、losses/discrete/ce
     - SiLRI 约束乘子、人工行为不确定度与人工比例；discrete/ce 只在有离散夹爪头时出现。
   * - behavior/pretrained、behavior/pretrain/*、behavior/refresh/*、behavior/refresh_count
     - SiLRI 预训练与后续刷新，比较 actor/distance 与 sigma_sum 加 constraint_margin 判断约束状态。

**你应该看到**：本次运行的方法对应的指标随接收／更新推进。
**如果不对**：先核对方法、actor_id、log_freq 和当前阶段，不要求所有变体同时出现整张表。

5. 在 CPU 上先走通数据链
--------------------------------------------------------------------------------

从源码环境运行这个六步练习，熟悉记录和恢复结果；输出目录必须尚不存在。

.. tutorial-test: skip
.. literalinclude:: ../examples/rl-train.sh
   :language: bash
   :start-after: # start smoke
   :end-before: # end smoke

脚本复用 tests/test_real_world_hil_serl_loops.py 的假环境和小型 CPU RLPDHybrid，
执行真实 HilSerlActorLoop、journal 摄入、梯度更新和 bundle 恢复。
两回合各三步，第 2 步注入人工动作；末步分别注入成功和失败事件，由真实奖励 wrapper 处理。
脚本将 observation 包成策略需要的 state 字典，替换的是 HTTP 服务和传输调度。
学习器每收到一条就更新一次，不能用这个速度估计现场吞吐或网络延迟。

本地尝试创建回环 socket 得到以下实测错误，因此没有执行 HTTP 双进程验收：

.. literalinclude:: ../examples/rl-train-format.txt
   :language: text
   :start-after: # start socket
   :end-before: # end socket

**你应该看到**：以下是脚本实际 stdout。received 等摘要由练习脚本打印；learner/* 名称和值
取自真实 loop 指标，并非声称生产 CLI 会逐步打印这些行。

.. literalinclude:: ../examples/rl-train-output.txt
   :language: text
   :start-after: # start smoke
   :end-before: # end smoke

最后 learner 已更新 6 次，actor 最后一条动作使用第 5 次更新；第 6 次在最后动作之后才产生，
所以这个差一不是丢参数。完整代码位于 ``docs/examples/rl-train-smoke.py``。
脚本额外保存 actor-audit.sqlite3 供第四章比较；生产 actor 默认没有这个永久审计副本。

**如果不对**：目录已存在就换新目录；缺 tests 模块时从完整仓库根运行；
缺 torch／gymnasium 时使用已安装 learning、test 的 rlg 环境。普通源码 CLI 的帮助可用：

.. literalinclude:: ../examples/rl-train.sh
   :language: bash
   :start-after: # start source-help
   :end-before: # end source-help

6. 替代：在 BC 或 ACT 上训练残差
--------------------------------------------------------------------------------

这条路线替代第 3 步的 HIL-SERL，两端仍用已核对的 local/rl-task.yaml。
只运行一种方法，先结束旧 actor／learner；本例通过独立 exp_name 和 run_id 隔离记录。
两台机器先各选同一个基础规格：

.. literalinclude:: ../examples/rl-train-alternatives.sh
   :language: bash
   :start-after: # start base-bc
   :end-before: # end base-bc

ACT 改用下面一组，不能把两组连续执行成同一次运行：

.. literalinclude:: ../examples/rl-train-alternatives.sh
   :language: bash
   :start-after: # start base-act
   :end-before: # end base-act

两端均须传基础规格；只有 actor／eval 实际构造基础模型，learner 用上传的
base_actions 和 next_base_actions 学习。基础文件及其元数据须可在 actor 机器读取。
在训练机运行：

.. literalinclude:: ../examples/rl-train-alternatives.sh
   :language: bash
   :start-after: # start residual-learner
   :end-before: # end residual-learner

**你应该看到**：独立运行目录的 journal 已建立，等待在线数据。
**如果不对**：先查基础规格、配置与实验身份，不把旧 first-hil 的 journal 混入。

.. warning::

   下面 actor 会 reset 并执行基础策略加残差。基础模型能加载不代表可安全执行；
   先完成输入／动作约定检查，保留接管、现场监护和物理停止装置。

.. literalinclude:: ../examples/rl-train-alternatives.sh
   :language: bash
   :start-after: # start residual-actor
   :end-before: # end residual-actor

连续维度执行 clamp(base + scale * residual, -1, 1)，人工接管覆盖总动作。
hybrid 只对机械臂维度加残差，夹爪用 DQN 头；continuous 对全部维度使用残差公式，
但 residual_gripper_scale 默认 0，保留基础夹爪。本例显式选 hybrid，代码默认是 continuous。
residual_action_scale=0.1 是归一化动作尺度；critic_only_updates=500 只在前 500 次更新训练 critic，
residual_bc_weight=0.1 在人工监督维度上拟合人工修正；后两项代码默认均为 0。

普通遥操作示范没有基础动作，因此本例不传 demo_dataset_paths。
``--allow-base-less-demos`` 才允许把这种示范按零基础动作加载；这并非用冻结模型给示范补算基础动作，
会改变残差学习目标，不能为了消除报错随意打开。

**你应该看到**：摄入和更新推进，critic-only 阶段结束后才有残差 actor 指标。
**如果不对**：基础权重不匹配先按 :doc:`bc-check` 检查；介入后 ACT 的重规划见 :doc:`rl-intervene`。

7. 替代：SiLRI 的人工行为模型
--------------------------------------------------------------------------------

SiLRI 也替代第 3 步。复制 local/rl-task.yaml 为 local/rl-silri.yaml，
把 utd 改为整数 1（SiLRI 要求 int，原模板的 1.0 是 float），其余任务／观测／遥操作约定保留。
下面命令给新实验单独命名；先在 learner 机器运行，再在机器人 PC 运行 actor。

.. literalinclude:: ../examples/rl-train-alternatives.sh
   :language: bash
   :start-after: # start silri-learner
   :end-before: # end silri-learner

learner 在首次训练更新前用 demo buffer 预训练人工行为模型，bc_pretrain_steps 默认 500。
预训练期间接收行仍可写入 journal，之后再消费；在线人工示范增加后按刷新条件重训行为模型。
默认每新增 50 条 demo 刷新 50 步，对应 behavior_refresh_every 和 behavior_refresh_steps。

**你应该看到**：behavior/pretrained、behavior/pretrain/*，随后出现 behavior/refresh/* 与 refresh_count。
**如果不对**：检查种子示范是否加载、demo_learning_starts（默认 10）和在线 learning_starts 是否满足。
配置类型报错先把 utd 改为整数，不删除任务字段绕过验证。

.. warning::

   actor 会 reset 并执行策略。SiLRI 的行为约束不是安全保证，保留已验证的接管和物理停止手段。

.. literalinclude:: ../examples/rl-train-alternatives.sh
   :language: bash
   :start-after: # start silri-actor
   :end-before: # end silri-actor

**你应该看到**：初始策略接收、回合记录和新参数版本，检查方式与第 3、4 步相同。
**如果不对**：先检查独立 run_id、同步地址和 outbox，不启动第二个 actor 占用同一机器人。


.. note::

   **本章哪些内容需要真机确认**

   真机命令、双机 HTTP、daemon gRPC、输入设备、图像吞吐、控制周期、初始随机策略的行为
   及停止时序尚需现场确认。CPU 练习实测了六条样本、两条人工 demo、六次更新、参数刷新与 bundle 恢复；
   未使用 GPU 或外部网络，也未实测 socket 同步。

检查清单：知道初始化不是 BC；先有样本再有更新；找到 actor 上的正 global_update；
能区分实测脚本输出与硬件格式示例。

下一章 :doc:`rl-intervene`。
