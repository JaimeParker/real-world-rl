训练第一份 BC 策略
================================================================================

本章把 :doc:`bc-data` 选好的示范交给 ``real_world_bc``，得到可加载的 checkpoint。
全部在 CPU 上运行，不启动 daemon，不连接机械臂或遥操作设备。
训练环境只提供观测／动作空间；本入口没有 actor／learner 双进程，也没有在线评估回合。

1. 写训练配置
--------------------------------------------------------------------------------

先看最小配置，再用录制会话中的观测和动作设置生成自己的版本。
模板位于 ``site/examples/bc-train.yaml``：

.. literalinclude:: ../examples/bc-train.yaml
   :language: yaml
   :start-after: # start config
   :end-before: # end config

.. list-table::
   :header-rows: 1
   :widths: 36 64

   * - 必须改的字段
     - 怎么得到这个值
   * - ``env_backend``、``env_id``、``obs``、``real_robot``
     - 用上一章找到的 session resolved-config 整块复制，下面的准备脚本代做。
       特别保留 robot／remote、task_frame、workspace、control、preprocess 和动作语义。
       模板 remote 只是离线构造空间所需的地址，不要求那里运行服务。
   * - ``obs.rgb``、``obs.image_size``、``real_robot.state_fields``
     - 对照上一章检查输出和 contract。例中 rgb_wrist 的形状是 [1,128,128,3]，
       对应 rgb: [wrist]、image_size: [128,128]；state 为 27 维，对应模板中的四个 state_fields。
   * - ``buffer_size``
     - 用上一章 transitions 数；准备脚本设为 max(64, 行数)，避免容量不足时只留下末尾示范。
       扩展为多个 pkl 时要容纳总行数，并估计图像占用的 CPU 内存。
   * - ``log_dir``、``exp_name``
     - 本例写入 local/bc-runs/first-bc。开始新实验时换尚未使用的名字，避免覆盖 checkpoint 或混合日志。
   * - ``num_offline_steps``、``batch_size``
     - 本例四次更新、每次四行，只用于验证流程。真实训练按数据量调整，另留完整回合作检查集。

其余训练字段保留模板值：device、buffer_device 都为 cpu；log_type 为 tensorboard，
log_freq 为 1，checkpoint_freq 为 2，save_final_checkpoint 为 true，seed 为 1。
更长训练时再降低记录和保存频率。未列出的模型设置保持代码默认：
actor_lr=0.0003、backbone_type=mlp、encoder.backbone=plain_conv、
encoder.features_dim=256、tanh_squash=true、offline_sampling=with_replace。
只有比较模型或优化设置时才调整它们；本章不需要下载预训练权重。

对真实数据，把命令里的 uuid 替换成上一章打印的实际目录名：

.. literalinclude:: ../examples/bc-train.sh
   :language: bash
   :start-after: # start prepare
   :end-before: # end prepare

如果走合成数据练习，改用这一条；两条只选一条：

.. literalinclude:: ../examples/bc-train.sh
   :language: bash
   :start-after: # start synthetic-prepare
   :end-before: # end synthetic-prepare

**你应该看到**：配置路径、输入 pkl 路径、行数与 buffer_size。
合成例子实跑为 8 行、buffer_size 64；输出文件是 ``local/bc-train.yaml``。
脚本只从 session 复制 env_backend、env_id、obs、real_robot，训练参数来自模板；
teleop 来源约定由训练器从 demo contract 恢复，不会打开设备。

**如果不对**：

* ``FileExistsError``：准备脚本不覆盖已有配置；先检查旧文件或改输出名。
* ``empty demonstration``：返回上一章确认示范确实有行。
* session 指向别的任务：重新选择匹配 pkl 的目录，不手动删掉 contract 来绕过检查。

2. 启动训练
--------------------------------------------------------------------------------

用本仓训练入口加载这份配置，从 YAML 的 ``demo_dataset_paths`` 列表读取示范路径：

.. literalinclude:: ../examples/bc-train.sh
   :language: bash
   :start-after: # start train
   :end-before: # end train

``rwrl train`` 转发到本仓 training 注册表；``examples/train_real_world.py``
也调用同一个入口，不是另一个 BC 实现。输入为 ``local/first-demo.pkl``，
其同名 contract 会自动读取。准备脚本把示范路径写入 YAML 的 ``demo_dataset_paths`` 列表；
需要临时覆盖时仍可使用 ``--demo-dataset-paths`` CLI 参数。

本次验证所用 rlg 环境未安装 rwrl console script 与后端 entry point，
只设置 PYTHONPATH 后直接启动还会在预检时报 ``Unknown env backend 'real_robot'``。
下面先导入本仓后端，再调用同一个训练入口；这是本章合成数据实际运行的命令：

.. literalinclude:: ../examples/bc-train.sh
   :language: bash
   :start-after: # start synthetic-train
   :end-before: # end synthetic-train

已安装完整包的环境可用上面的 rwrl 命令；合成练习要把输入路径改为 demos.pkl。
源码方式训练自己的数据时，只替换上面命令最后的路径为 first-demo.pkl。
这些命令不是先后重复执行的步骤，同一个实验目录只跑一次。

**你应该看到**：进程正常退出，生成下一节列出的文件。当前 BC 循环把数值写到
TensorBoard，不逐步打印训练进度；终端安静不能用来判断有没有更新。

**如果不对**：

* ``No demo transitions or restored replay available``：检查 CLI 路径是否拼错、glob 是否匹配文件。
* ``recorded action contract does not match``：按错误列出的 differs in 字段对照 session 配置；
  观测、动作模式、尺度或处理流程不同就选回匹配的数据。
* ``Unknown env backend 'real_robot'`` 或找不到 rwrl：源码环境使用上面的显式注册方式。
* 内存不足：先减少选入的数据量／缓冲容量并核对能容纳全部行；不要悄悄丢弃相机或改变图像尺寸。

3. 看懂监督和日志
--------------------------------------------------------------------------------

从日志确认更新确实发生，再解释模型在拟合哪个目标。

.. literalinclude:: ../examples/bc-train.sh
   :language: bash
   :start-after: # start logs
   :end-before: # end logs

**你应该看到**：以下是八行合成数据、四步 CPU 训练的实际标量读取结果：

.. literalinclude:: ../examples/bc-train-output.txt
   :language: text
   :start-after: # start metrics
   :end-before: # end metrics

日志位于 ``local/bc-runs/pick_and_place/first-bc/events.out.tfevents.*``，
其中 pick_and_place 来自 env_id（未另设 log_group）。
checkpoint 与配置目录没有这层分组。实际目录内容为：

.. literalinclude:: ../examples/bc-train-output.txt
   :language: text
   :start-after: # start files
   :end-before: # end files

``config.json`` 保存解析后的训练配置；``experiment.json`` 保存实验约定与有效控制元数据。
checkpoint 保存策略权重、优化器等训练状态；默认 save_replay_buffer=false，
不会附带原始示范或 replay 文件。迁移时保留 config.json、experiment.json、checkpoints 的相对位置。

监督发生在 ``real_world_rl/algorithms/bc.py`` 的 ``MaskedBC._compute_losses``：
给模型的是第 t 行 obs，目标是同一行的 action，即人接管后实际交给环境的动作。
不是 next_obs 对应的下一步动作，也不是接管前的 policy_action，更不是实测 TCP 位移。
next_obs、reward、done 会进入缓冲，但不进入这个 BC 损失。

默认采样器 with_replace 从缓冲中有放回抽取单步 transition；四步是四次梯度更新，
不是四个 epoch。高斯策略对目标动作计算逐维负对数似然；默认 tanh_squash 下先将动作
映回高斯空间，并包含 Jacobian 修正。目标在边界时会先裁剪到 ±(1−1e-6)。

``human_action_mask`` 从示范经 demo_loading 和 DemoReplayBuffer 保留下来。
``masked_sum_mean`` 只加人工维度：每行先乘“总动作维数／人工维数”，
再对有人工维度的行取平均。七维例子的 1111110 只监督前六项，比例为 7/6；
1111111 监督全部七项。全零行不参与平均，整个 batch 全零则跳过优化器更新并返回零指标。
旧数据若既无 mask 又无 intervened，会被加载器视为全人工示范；本章要求保留显式 mask。

日志的 ``losses/behavior_mse`` 是确定性动作与目标在人工维度上的均方误差；
它按人工维度总数平均，与 NLL 的逐行缩放不同。
本例训练 MSE 从约 0.225 降到 0.063，说明参数更新能拟合这些合成样本。
NLL 初值很大，不能把它当作位置误差；边界动作的逆 tanh 和策略标准差都会影响其数值。
四次更新不足以证明收敛，随机图像更不能证明任务成功。下一章检查具体一行输出。

**如果不对**：

* 找不到标量：先确认进程已经退出、log_type 是 tensorboard，并读取带 env_id 分组的目录。
* 每步 loss 和 MSE 都为零：检查是否所有 mask 都是零；不要直接判断“完美拟合”。
* loss 或预测出现 NaN／Inf：先检查数据有限值和动作尺度；边界动作数值问题可单独实验
  tanh_squash=false，并重新训练、记录配置，不沿用旧 checkpoint。
* 训练误差下降而独立回合表现差：按回合分开训练与检查数据，检查示范覆盖范围，增加有代表性的示范。

本章产物
--------------------------------------------------------------------------------

* **下一章和部署章使用：** ``local/bc-runs/first-bc/checkpoints/final.pt``。
* 周期 checkpoint：``local/bc-runs/first-bc/checkpoints/checkpoint_2.pt``、``checkpoint_4.pt``。
  一般命名为 ``<log_dir>/<exp_name>/checkpoints/checkpoint_<更新步数>.pt``，最终文件为 final.pt。
* ``local/bc-runs/first-bc/config.json``、``experiment.json``，以及分组目录中的 TensorBoard 日志。
  本例未设置 checkpoint_dir；若手动设置它，保存目录会改变，不再直接套用上述布局。

.. note::

   **本章哪些内容需要真机确认**

   训练、日志读取与保存已用 CPU 合成数据验证。示范是否成功、相机是否看到任务、
   动作约定是否对应现场运动，仍依赖真机录制检查；loss 下降不能替代这些确认。

检查清单：配置与数据 contract 一致；知道 mask 怎样进入损失；日志有四步有限值；
找到 final.pt 及旁边的配置与实验记录。

接下来 :doc:`bc-check`，用保存的观测解释 checkpoint 的一次预测。
