评估策略与恢复训练
================================================================================

本章保存一次可恢复的训练状态，再用固定策略独立评估。
不要把人工帮助完成的任务计作自主成功；也不要只复制模型权重就声称恢复了完整在线实验。

1. 保存模型、replay 和接收进度
--------------------------------------------------------------------------------

本例 checkpoint_freq=100，按 learner 的 global_update 保存 checkpoint；
buffer_period=100 则按收到的 transition 数输出传统 pkl 快照。这是两个不同计数。
正常停止时先让 actor 在保持阶段 q 退出，确认机械臂停止，再在 learner 终端 Ctrl-C。
让 actor 先排空上传，能减少 learner 停止后留在 outbox 的记录。

actor 清理时会在 ``--sync-drain-timeout`` 内尝试排空上传，默认 30 秒。
超时后未 ACK 行保留在本地 outbox，下次以相同身份重启后重发，不必为了空 outbox 强行删记录。
多个并发 actor 必须各用独立 ``--actor-id`` 和 ``--actor-outbox-path``，
且各自控制独立设备；这不是允许两个 actor 同时控制本教程的一台机械臂。

**你应该看到**：checkpoints 下有 final.pt 和 bundle_manifest.json；周期保存还可能有
checkpoint_<更新步数>.pt。manifest 指向一个 bundle_<更新步数>_<uuid> 目录，包含
agent.pt、replay.pt、experiment.json。完整 bundle 保存模型／优化器、online 与 demo replay、
环形缓冲位置、loop 计数和 journal cursor／identity。

manifest 原子发布后该代才成为恢复点。正常 Ctrl-C 会尝试 final 保存；训练异常或摄入中断时
可能保留上一代，不保证每次退出都有新 final.pt。不要选时间最新但尚未被 manifest 引用的半写目录。

**如果不对**：没有周期文件时看 global_update 是否达到 100；save_final_checkpoint=false 时无 final；
磁盘不足或更新异常时保存日志，使用最后完整 manifest。buffer／demo_buffer 的 pkl 是兼容快照，
不是包含优化器和一致游标的完整恢复点。

2. 中断后先确认本地证据
--------------------------------------------------------------------------------

在 actor 机器检查 outbox，在 learner 机器检查 journal 与 manifest，确认仍是同一次实验。
以下文件在双机情况下分开执行相应行。

.. tutorial-test: skip
.. literalinclude:: ../examples/rl-resume.sh
   :language: bash
   :start-after: # start local-data
   :end-before: # end local-data

**你应该看到**：outbox 中未 ACK 的记录，以及 journal 已接收记录和最后完整的回合结束信息。
它们是互补证据：空 outbox 可以表示已全部交付。被杀进程的未结束回合不能算成功；
actor 重启会给其已持久化的活动回合补 process_restart 中断边界。

保留完整 run 目录、原示范和 sidecar、生成配置、teleop 引用文件及 station 版本。
SQLite 使用本地 WAL，保留数据库及存在的 -wal／-shm 文件；迁移时停止写入者后复制完整目录，
不要在线只拷主 .sqlite3 文件。journal 不要与其他实验的 bundle 混用。
只读取自己生成或可信的 checkpoint／pickle。

本实现的 synchronous=NORMAL 保证已提交记录可跨进程崩溃恢复，不能保证断电不丢最后几条提交。
机械臂执行后、actor 尚未提交之前被强杀，也可能丢失该步。网络中断时 actor 在已有策略上继续采集，
outbox 达容量或持久化失败才停止；这和启动时等待第一份快照是两种情况。

**如果不对**：没有 manifest 时只能以现有 journal／旧快照重建可用数据，不能宣称恢复了丢失优化器状态；
数据库为空却有未结束动作时保留现场记录排查；目录属于不同实验时选回匹配目录，不修改 identity 绕过检查。

3. 恢复 learner，再恢复 actor
--------------------------------------------------------------------------------

在 learner 机器用原目录和 manifest 恢复，保证 journal identity 与 bundle 的游标匹配。

.. literalinclude:: ../examples/rl-resume.sh
   :language: bash
   :start-after: # start resume
   :end-before: # end resume

**你应该看到**：先恢复 online／demo buffer 及更新计数，只重放保存游标之后的 journal 尾部，
再发布初始策略并接受 actor。完整 bundle 的 demo buffer 已含保留的种子示范，因此不会再加载一遍。
第三章 CPU 实测恢复值为 received=6、update=6、demo=2，重放尾部为 0。
旧 hil-serl-roadmap 的“每次启动都重载 seed demo”是迁移时期的描述；当前完整 bundle 恢复不会再次叠加。

``--load-replay-buffer`` 控制旧式独立 .pt 的伴随 replay；选择 bundle 时始终恢复完整一致单元，
不能靠 --no-load-replay-buffer 把它变成只读权重。只传 manifest、却换 checkpoint-dir，会创建不匹配 journal。

确认现场安全及 daemon 恢复后，再执行第三章原 actor 命令，保留 run_id、actor_id 和 outbox 路径。
同一 outbox 只允许一个写入进程；未 ACK 的记录重发，learner 按稳定 ID 去重。
重启 learner 后 policy_session_id 会改变；不要要求 policy_version 必须大于旧 session 的最大值。

.. warning::

   重启 actor 会重新 reset，可能归位并开爪，不是从中断的轨迹中间续走。
   清理物体位置、核对故障恢复与停止装置后才运行。

.. literalinclude:: ../examples/rl-train.sh
   :language: bash
   :start-after: # start actor
   :end-before: # end actor

**如果不对**：``checkpoint journal identity differs`` 时找回原 journal 及 WAL；
``checkpoint seed dataset contents differ`` 时恢复原始示范内容；
``durable store belongs to a different experiment`` 时核对配置，独立新实验用新目录，不复用旧 outbox。

4. 部署固定的新策略做独立评估
--------------------------------------------------------------------------------

先结束在线 actor，复制完整已保存 run 到机器人 PC（若 learner 就在本机则无需复制）。
本章用 hil_serl 的 eval 角色部署固定快照，保持同一个 env 与事件 wrapper，便于直接得到两种成功率。
通用部署与观测／动作检查流程见 :doc:`deploy`。

.. warning::

   eval 会 reset 并执行新策略；使用与 actor 相同的 control_hz，推理和 step 耗时不足一个周期时补足等待。
   本例保留 episode_end_behavior=hold：reset 后先确认中立，每回合结束保持，按 n 才开始下一回合；
   预算耗尽后只能按 q 退出。auto 模式则自动 reset。保留人工接管与物理停止手段。

.. literalinclude:: ../examples/rl-resume.sh
   :language: bash
   :start-after: # start eval
   :end-before: # end eval

残差 BC／ACT 或 SiLRI 使用各自的 eval 入口。残差先在当前 shell 重设 :doc:`rl-train`
第 6 步的 BASE、RUN（选 BC 或 ACT），保留训练时的 hybrid 及残差参数；两条 eval 命令只选对应方法。
同样会 reset 并执行策略，遵守本节运动警告，先停止对应 actor。

.. literalinclude:: ../examples/rl-resume.sh
   :language: bash
   :start-after: # start residual-eval
   :end-before: # end residual-eval

.. literalinclude:: ../examples/rl-resume.sh
   :language: bash
   :start-after: # start silri-eval
   :end-before: # end silri-eval

**你应该看到**：同样的逐回合 [eval] 和自主／任务成功率摘要，策略确定性预测。
**如果不对**：检查是否把另一方法的 checkpoint 传给当前入口；残差 eval 仍必须提供基础规格。
恢复训练也使用原方法、原配置／参数和该 run 的 manifest、checkpoint-dir，不复用 first-hil 的目录。

此入口不连接 learner，不训练，也不会运行 actor 的 outbox 归档循环。
策略确定性预测，checkpoint 解析和契约验证在 reset 前完成；本章仍需命令行提供匹配的 rl-task.yaml。
通过重定向或终端保存 eval 输出；如需逐步动作审计，需另外安排采集，不能声称该命令自动生成 journal。
正常评估尽量不接管，只在实际任务完成／失败时按 s/f 标注；必须接管时如实保留介入统计。

**你应该看到**：逐回合 [eval] 与末尾汇总。下面是第二章所述人工标签假环境的实际 CPU 输出，
使用同一个 evaluate_policy 函数；不是上面真机 eval 命令的实测结果：

.. literalinclude:: ../examples/rl-resume-output.txt
   :language: text
   :start-after: # start eval
   :end-before: # end eval

.. list-table::
   :header-rows: 1
   :widths: 36 64

   * - 真实指标名
     - 怎么计数
   * - ``task_success_rate``
     - 明确 task_success=True 的完成回合／全部完成回合；人工帮助成功也计入。
   * - ``autonomous_success_rate``、``success_rate``
     - 无任何人工控制步骤且明确成功的回合／全部完成回合。success_rate 是自主成功率的别名。
   * - ``episode_intervention_rate``、``step_intervention_rate``
     - 完成回合中有介入的比例，以及完成回合全部步骤中人工步骤的比例。
       只按 s/f 标注、没有非零 human_action_mask，不算动作介入。
   * - ``unknown_episodes``、``completed_episodes``
     - 未明确标注的完成回合与所有完成回合。unknown 仍在成功率分母里，不能隐去。
   * - ``interrupted_episodes``、``discarded_episodes``
     - 单列的中断／丢弃回合，不进入完成回合分母。
   * - ``tcp_proxy_success_rate``
     - TCP 位置代理结果；本教程没有配置该代理，不能拿它替代物体任务成功。

在线训练中同样的汇总可见 ``episodes/task_success_rate``、``episodes/autonomous_success_rate``，
以及本例 ``actor/actor/task_success_rate`` 等回合统计。在线统计反映带探索和人工纠正的训练，
独立 eval 用冻结快照与确定性动作，二者不要混成一个验收数字。

**如果不对**：task_success_rate 高而 autonomous_success_rate 低，说明依赖人工完成；
unknown_episodes 多就检查标注来源；eval 需要 checkpoint 的错误说明未传 --load-checkpoint；
意外自动 reset 时停止评估并检查配置中的 episode_end_behavior 是否为 hold。

.. note::

   **本章哪些内容需要真机确认**

   正常退出与强杀后的机械臂停止、设备故障恢复、再次 reset、网络重传、独立评估的节奏与成功判定
   需要现场验证。CPU 实测了 bundle 保存／恢复和两种成功率；没有验证断电、双机断链或真实任务成功。

检查清单：保留匹配的 journal、manifest 和 generation；恢复后没有重复叠加种子示范；
能分别报告任务成功与自主成功；能把一条纠正的 ID、learner 接收记录和后续 actor 新版本连起来。

真机强化学习这一部分完成。继续用 :doc:`deploy` 检查选定策略的独立部署。
