路线三 3. 启动 HIL-SERL
================================================================================

本章按设备服务、learner、actor 的顺序启动，再用记录验证数据与参数的两个方向。
所有命令从仓库根目录运行，使用各机器已安装 learning 的环境；设备服务只需设备侧依赖。
配置沿用 :doc:`rl-task` 的 ``local/rl-task.yaml``。下面的真机命令没有在本地执行。

1. 先知道第一次执行的策略来自哪里
--------------------------------------------------------------------------------

开始前先确认初始化方式，决定操作员何时接管。
单夹爪任务构造 ``RLPDHybrid``：六维连续机械臂动作和一个离散夹爪头。
无夹爪契约使用 ``ContinuousHilRLPD``。本路线不使用前章 BC 的 final.pt 自动初始化。

learner 从新网络开始，把 ``--demo-dataset-paths`` 的数据放入 demo buffer。
示范不会计入 ``learning_starts``；要收到本例 100 条在线 transition 才开始更新。
训练批次按 demo_data_ratio 混合在线和示范数据，在线人工纠正继续扩充 demo buffer。
这不是“先对示范训练一次 BC，再上线”的入口。

actor 也先构造同形状的小缓冲策略，再等待 learner 的初始快照覆盖权重。
初始快照可以是 ``phase=warmup``、``policy_ready=False``；HIL-SERL 的 actor
仍然调用策略 predict，不会因这个标记自动保持机械臂或等待 BC 完成。
首个训练更新后发布 ready 策略，后续按 publish_freq 发布。

**你应该看到**：新实验最早记录的 global_update=0，之后才有正值。
帮助虽列出 ``--load-actor-checkpoint``，当前 hybrid builder 没有使用它来加载 BC；
不要据此宣称已有 BC 热启动。恢复 HIL-SERL 训练使用第五章的完整 bundle。
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

3. 在训练机启动 learner，再在机器人 PC 启动 actor
--------------------------------------------------------------------------------

先启动 learner，让 actor 能在初始快照等待期限内连上。
训练机上的 pkl、contract、rl-task.yaml 及所引用 teleop 配置必须可读；remote 指向机器人 PC
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

.. literalinclude:: ../examples/rl-train.sh
   :language: bash
   :start-after: # start inspect
   :end-before: # end inspect

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

5. 在 CPU 上先走通数据链
--------------------------------------------------------------------------------

从源码环境运行这个六步练习，熟悉记录和恢复结果；输出目录必须尚不存在。

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
所以这个差一不是丢参数。完整代码位于 ``site/examples/rl-train-smoke.py``。
脚本额外保存 actor-audit.sqlite3 供第四章比较；生产 actor 默认没有这个永久审计副本。

**如果不对**：目录已存在就换新目录；缺 tests 模块时从完整仓库根运行；
缺 torch／gymnasium 时使用已安装 learning、test 的 rlg 环境。普通源码 CLI 的帮助可用：

.. literalinclude:: ../examples/rl-train.sh
   :language: bash
   :start-after: # start source-help
   :end-before: # end source-help

.. note::

   **本章哪些内容需要真机确认**

   真机命令、双机 HTTP、daemon gRPC、输入设备、图像吞吐、控制周期、初始随机策略的行为
   及停止时序尚需现场确认。CPU 练习实测了六条样本、两条人工 demo、六次更新、参数刷新与 bundle 恢复；
   未使用 GPU 或外部网络，也未实测 socket 同步。

检查清单：知道初始化不是 BC；先有样本再有更新；找到 actor 上的正 global_update；
能区分实测脚本输出与硬件格式示例。

下一章 :doc:`rl-intervene`。
