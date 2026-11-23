看懂一次在线强化学习实验
================================================================================

真机强化学习这几章把 :doc:`station` 的工作站、:doc:`record` 的示范和 :doc:`deploy` 的执行检查
接成一次在线实验。先完成部署章的观测、动作和停止检查，再让策略参与真机控制。
使用 openpi 的读者也先完成 :doc:`deploy-openpi`；这里训练的是本仓 HIL-SERL 策略，
不会把 openpi 服务自动变成 HIL-SERL 的初始策略。

这几章选用前面验证过的单臂、七维笛卡尔动作和人工成功／失败标注。
SpaceMouse 使用 tracker；GELLO 使用 leader_cartesian。复用 ``local/station.yaml``、
``local/task.yaml`` 和 数据集目录 ``local/first-demo``，另写 ``local/rl-task.yaml`` 保存训练设置。
先不启用分类器、抓取惩罚或免复位的 FWBW，便于核对一次奖励到底从哪里来。

1. 先分清三个进程
--------------------------------------------------------------------------------

按下面的部署位置安排进程，避免 learner 和 actor 同时打开硬件。
设备服务是持有机械臂、夹爪和相机连接的进程；actor 是执行控制循环的进程；
learner 是更新策略参数的进程。这几章让 actor 与设备服务同在机器人 PC，learner 在训练机。
两者也可同机运行，连接 learner 时改用回环地址。

.. literalinclude:: ../examples/rl-overview-diagram.txt
   :language: text
   :start-after: # start topology
   :end-before: # end topology

actor 通过 ``real_robot.remote`` 找设备服务，通过 ``sync_host:sync_port`` 找 learner。
这两个地址不是同一端口。训练机只构造观测／动作空间，不 reset 或 step 真实机械臂，
不打开遥操作设备。相机大小必须在配置里提供，不能等 learner 连上相机后再推断。

.. warning::

   看门狗有故障边界。直连控制进程中的看门狗线程无法覆盖整个进程冻结；
   独立的 ``rwrl supervise`` 可以终止子进程，直连路径随后依靠固件处理控制连接丢失。
   daemon 的看门狗可在 servo 客户端不再发动作时停止机械臂，但它仍在 daemon 进程内，
   不能覆盖 daemon 或 NUC 故障。保持现场监护和可用的物理停止装置。

在各自已安装环境中查看入口，不连接设备：

.. literalinclude:: ../examples/rl-train.sh
   :language: bash
   :start-after: # start help
   :end-before: # end help

**你应该看到**：train 列出 ``hil_serl``，其帮助含 ``--role {actor,learner,eval}``、
``--sync-host``、``--sync-port``、``--actor-outbox-path`` 和 ``--load-checkpoint``。
三个角色共用一个方法入口；``examples/train_real_world.py`` 也调用相同注册表。

**如果不对**：找不到 rwrl 时回到 :doc:`install` 激活安装环境；源码环境可用
:doc:`rl-train` 给出的模块入口。把 50051 填成 sync_port 会连错服务；把 learner 的
0.0.0.0 绑定地址填给 actor，应改为训练机实际可达地址。

2. 沿一个时间步找数据
--------------------------------------------------------------------------------

选下一章产生的一条记录，按以下顺序核对它的输入和输出。

#. actor 等到 learner 的初始快照后才 reset。reset 从设备服务读状态和相机，得到第一条 obs。
   以后每步的 obs 来自上一步的 next_obs。
#. actor 在步首检查新快照，随后用当前策略提出 ``policy_action``。
   HIL-SERL 默认随机采样动作，eval 则用确定性预测。
#. 遥操作 wrapper 按接管状态和逐维 mask 选择执行动作。无人接管时策略控制；
   人工接管时 ``intervene_action`` 替换相应动作。设备层仍会实施限位和安全约束。
#. 环境把动作送往设备服务，再读取 next_obs。人工 ``s``／``f`` 事件形成任务结果；
   这里成功奖励为 1，失败奖励为 0，并分别终止／截断回合。
#. actor 写入 obs、action、reward、next_obs、done，同时归档 policy_action、
   human_action_mask、回合标记和策略版本。``action`` 是交给环境执行的请求，
   不是编码器测得的实际位移；实际运动要对照前后观测。
#. 本地 outbox 提交后异步上传。outbox 是待发送的 SQLite 队列；learner 的 journal
   是收到记录后先写入的 SQLite 日志。HTTP ACK 表示 journal 已接收，不表示已经训练。
#. learner 按 journal 顺序加入 online replay；非 syncing 的人工样本也加入 demo replay。
   从两种 replay 混合采样做更新，再发布参数。下一步 actor 可以加载新版本。

读取记录用同一个检查脚本；在第三章启动后，在 learner 机器执行：

.. tutorial-test: skip
.. literalinclude:: ../examples/rl-train.sh
   :language: bash
   :start-after: # start inspect
   :end-before: # end inspect

**你应该看到**：``episode_start``、``transition``、``episode_end`` 三类记录。
transition 有 ``record_id``、``episode_id``、``step_index`` 和动作／策略版本字段。
第四章会按同一个 record_id 找出人工纠正。回合记录可能尚未结束；别把当前最后一行当成完整回合。

**如果不对**：文件尚不存在就先完成第二、三章；outbox 空不代表未采集，ACK 后其行会删除，
应读 learner journal。仅看到 episode_start 时检查 actor 的首步执行、错误日志和上传状态；
START_NEUTRAL 等待发生在 episode_start 写入之前。
没有 terminated 但有 truncated 是截断，不能据此推断成功。

3. 明确这几章的验收证据
--------------------------------------------------------------------------------

每完成一章，留下能对应到下一章的文件或 ID。

* 任务：成功和失败的 episode_end.outcome 能区分，示范 contract 与在线配置一致。
* 训练：journal 有 transition，learner/received 与 learner/update 增加，actor 后续记录的
  global_update 大于 0。
* 纠正：一条 record_id 同时解释 policy_action、action 和 human_action_mask，
  learner 已接收该条记录，人工数据计数增加。
* 恢复：matching journal 与 bundle 一起恢复，独立 eval 报告 task_success_rate 和
  autonomous_success_rate。

**你应该看到**：一条连续的证据链；不是只凭 loss 下降判断真机学会了任务。
**如果不对**：缺逐步数据就回查 journal；缺成功标签就回查事件源；缺策略版本就确认读取的是
当前 actor 的在线归档，而非 :doc:`record` 的录制器文件。

验收状态要区分“有实现／软件检查”和“当前工位真机通过”。仓库中的测试清单本身不是通过记录。
目前可确认的证据范围如下；这里不把旧工位会话或假环境测试推广为所有方法的真机验收。

.. list-table:: 各路线的证据边界
   :header-rows: 1

   * - 路线
     - 已有证据或检查材料
     - 仍需现场完成
   * - 单臂握手、GELLO／SpaceMouse 录制
     - 旧会话记录过硬件问题及修复；有后续验收清单
     - 用当前软件、控制器和工位重验方向、夹爪、停止与归档
   * - 双臂
     - 有双臂流程和参考测试清单；清单明确不代表硬件通过
     - 双臂同时复位、接管、停止及各自工作区验收
   * - BC／ACT
     - 有离线训练和策略契约检查流程；旧记录尚未验证 ACT 在真实 pkl 上训练
     - 真实数据训练结果和完整真机部署验收
   * - HIL-SERL／residual／SiLRI 在线学习
     - 有方法实现；本教程的 CPU 假环境检查只覆盖数据与更新链
     - 完整真机在线闭环、人工接管、恢复和独立评估

现场检查与证据保存方法见 :doc:`../reference/testing/index`；
双臂材料见 :doc:`../reference/testing/bimanual-gello-session`。
没有对应配置、日志和实测记录时，不标记真机通过。

.. note::

   **本章哪些内容需要真机确认**

   设备服务连接、观测新鲜度、动作与实测运动、接管及交还、停止效果和控制时序需现场确认。
   这里的 CPU 假环境只验证数据与更新链；本地 socket 被沙箱禁止，HTTP 同步仍需现场验证。

检查清单：能指出两个服务地址；知道谁发动作、谁写奖励、谁做梯度更新；
能区分请求动作、实测运动、journal 接收和 replay 训练。

下一章 :doc:`rl-task`。
