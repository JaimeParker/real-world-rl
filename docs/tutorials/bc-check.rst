离线检查 BC checkpoint
================================================================================

本章加载 :doc:`bc-train` 的 ``local/bc-runs/first-bc/checkpoints/final.pt``，
给它一条保存的观测，比较输出与示范动作。全部离线，不调用 reset、step 或设备连接。
目标是拿到可加载的 checkpoint，并能解释它的一条输入和一条输出。

1. 加载权重，运行一次预测
--------------------------------------------------------------------------------

用本章脚本恢复训练配置和策略，检查数据约定，再预测第 0 行保存的 obs。
只加载自己生成或可信的 checkpoint 与 pickle。

.. tutorial-test: skip
.. literalinclude:: ../examples/bc-check.sh
   :language: bash
   :start-after: # start check
   :end-before: # end check

如果上一章训练的是合成数据，用下面这一条替换预测命令；输出记录路径仍相同：

.. tutorial-test: skip
.. literalinclude:: ../examples/bc-check.sh
   :language: bash
   :start-after: # start synthetic
   :end-before: # end synthetic

脚本要求保留 run 目录中的 config.json 和 experiment.json。
它调用本仓部署模块的 ``restore_training_args`` 与 ``restore_policy``，
算法显式设为 real_world_bc，模型和数据都放到 CPU。
环境用与训练相同的离线空间构造方式，还会比较 demo contract 与训练实验记录；
不会进入部署回合。核心推理代码如下：

.. literalinclude:: ../examples/bc-check.py
   :language: python
   :start-after: # start inference
   :end-before: # end inference

``policy.eval()`` 在这段代码之前设置；deterministic=true 选择确定性动作，
默认 tanh Gaussian 下是变换后的均值，不是随机抽样。
输入直接使用保存的张量，保持 uint8 RGB 和 float32 state，不手动换通道或再缩放图像。

**你应该看到**：观测形状、示范动作、预测动作、mask、masked MSE，以及新建记录路径。
这是前章四步 CPU checkpoint 的实际输出：

.. literalinclude:: ../examples/bc-check-output.txt
   :language: text
   :start-after: # start inference
   :end-before: # end inference

**如果不对**：

* 缺少 config.json／experiment.json：找回训练目录；单独复制一个 final.pt 不足以完成本章的约定检查。
* ``recorded action contract does not match`` 或 ``saved experiment contract/hash differs``：
  检查 checkpoint、数据与训练配置是否来自同一次实验，不删除元数据绕过检查。
* 参数大小不匹配或观测键报错：确认算法是 real_world_bc，没误用 ACT 权重，保留原来的相机和状态布局。
* ``FileExistsError``：脚本不会覆盖 offline-check.json；检查别的行时给 record 一个新路径。

2. 解释这次输入和输出
--------------------------------------------------------------------------------

把上面的打印结果逐项译回录制约定，区分数值误差与动作含义。

.. list-table:: 四步合成训练的第 0 行
   :header-rows: 1
   :widths: 25 75

   * - 项目
     - 解释
   * - 输入 state
     - [1,27]；tcp_pose 7 项、tcp_vel 6 项、joint_pos 7 项、joint_vel 7 项。
       本例未转换四元数，未堆叠多帧。
   * - 输入 rgb_wrist
     - [1,224,224,3]，uint8，RGB；内容为合成随机像素，不能解释成真实场景。
   * - 示范动作
     - [0.2,0,0,0,0,0,1]；current 笛卡尔约定下请求 x 增加 0.01 m，不旋转，打开夹爪。
   * - 预测动作
     - x 分量约 -0.1588，对应约 -0.0079 m；y 分量约 0.0467，对应约 0.0023 m。
       模型仍输出了示范中没有的 y、z 和旋转请求。
   * - 预测夹爪
     - 约 0.0207，对应归一化开口请求约 0.5104；若使用前面录制时的二值 Hand，刚好超过 0.5 的开爪阈值。
       训练损失比较的是连续动作数值，不是二值设备最终是否开爪。
   * - mask 与误差
     - 第 0 行全七维人工，masked MSE 约 0.194124。它是归一化动作空间里的误差，不是 m 或 rad。

训练日志最后一步的 MSE 来自更新前抽样 batch，这里来自更新后的第 0 行，数值不必一致。
如果检查的是 mask=1111110 的行，夹爪仍会输出一个值，但它不进入该行监督误差。
这不能据此认定夹爪维度已学好；还要检查有夹爪监督的示范。

**你应该看到**：能够从任一预测分量说出动作模式、尺度和人工标记。
本例只有四次更新，输出偏差仍明显；“可以加载和推理”已经验证，任务可用性尚未验证。
真实数据应再检查来自独立完整回合的多条观测，不能只挑一条训练样本给出成功结论。

**如果不对**：

* 把预测理解成绝对 TCP 目标：重新查 action_mode，本例是相对当前状态的增量。
* 误差很小但某个维度明显不对：先看 mask，该维度可能没有参与监督。
* GELLO joint_abs 结果按米解释：用那次录制的 ``limits.joints`` 解码七个关节目标；不要套用本表。

3. 保存部署要核对的记录
--------------------------------------------------------------------------------

查看脚本生成的 offline-check.json，让部署章能把当前工作站与模型依赖逐项比较。

.. tutorial-test: skip
.. literalinclude:: ../examples/bc-check.sh
   :language: bash
   :start-after: # start record
   :end-before: # end record

**你应该看到**：checkpoint、demo、row 定位本次检查；observation_shapes、
observation_dtypes、cameras、state_fields 记录输入；action_conventions 记录输出约定；
action_contract 与 training_experiment 保留完整来源约定。
demo_action、predicted_action、human_action_mask、masked_mse 保存这一次比较的具体值。
本例 cameras 为 wrist，state_fields 是上一表的四项，action_conventions 中
cartesian_semantics 为 current、xyz_scale_m 为 0.05、rot_scale_rad 为 0.3、control_hz 为 10。

部署前至少核对以下几组内容：

* 相机名称、实际视角、RGB／深度、图像尺寸、crop 等 preprocess、帧堆叠和旋转转换。
  同名 wrist 不代表换了安装位置以后图像分布仍相同。
* state_fields 的顺序、字段维数、四元数 xyzw 约定、任务坐标系和单位。
* action_mode、cartesian_semantics、动作维数、尺度、``limits``（关节范围、TCP 盒）、夹爪映射和控制频率。
  动作约定还包括 experiment 中记录的 ``limits`` 与 teleop_control 来源。
* 当前 ``local/station.yaml`` 的设备、home、限位（``limits``）与控制设置，以及
  ``local/task.yaml`` 与训练时配置的差异。保留原 session resolved-config 作为录制依据。

**如果不对**：

* 记录与预期数据路径不同：确认上一章实际训练的是 first-demo 还是合成 demos，不给合成 checkpoint 贴真机标签。
* 只有 final.pt，没有输入记录或约定：把整个 run 目录与示范数据集一起归档后，再进入部署章。
* 换了相机、坐标系或动作模式：先重新核对／准备数据与模型；不能只改部署参数让维数凑齐。

4. ACT 或残差基础策略另查输入和动作约定
--------------------------------------------------------------------------------

ACT 提供器检查 state 宽度和动作宽度；本章四个 state_fields 合计 27 维，
字段顺序也必须与示范相同。视觉输入按 rgb_<camera> 名称排序，保留原 obs.rgb、
相机安装和预处理。独立 deploy-act 还比较 checkpoint 的 camera_keys 与环境相机顺序。

提供器从 checkpoint 的 act_config 重建模型，默认选 ema_agent 权重；
用保存的统计量归一化 state、反归一化动作，再裁剪到 [-1,1]。
残差的基础规格可写成 ``act:local/bc-runs/first-act/checkpoints/final.pt?temporal_agg=1``。
整个规格必须加引号，避免 shell 把 ? 当通配符或把 & 当后台执行符。

多个选项用 & 连接：temporal_agg=1 为默认的重叠块时间集成，权重按
exp(-temporal_agg_k * i) 计算；temporal_agg=0 为逐块开环回放。
temporal_agg_k 默认 0.01，state_dict_key 默认 ema_agent。介入后的块重置见 :doc:`rl-intervene`。

**你应该看到**：训练数据、运行配置与模型使用同一状态布局、相机顺序和动作解释。
宽度检查通过只是其中一项；ACT 不使用本章 BC 离线脚本。

**如果不对**：joint_abs 与 joint_delta 可以同宽，提供器的宽度检查发现不了混用。
必须另查 action_mode、``limits.joints`` 和尺度；残差基础策略也必须输出当前运行所需的动作空间。
不能把同宽或裁剪后的有限值当成动作语义匹配。

本章产物
--------------------------------------------------------------------------------

* **部署章的 checkpoint：** ``local/bc-runs/first-bc/checkpoints/final.pt``。
* 同一个 run 目录的 ``config.json``、``experiment.json`` 和本章新建的 ``offline-check.json``。
* 原始示范及其 contract、session resolved-config、station/task；共同解释模型的一条输入和输出。
  合成练习产物仅用于离线流程验证，不作为真机任务已可部署的证据。

.. note::

   **本章哪些内容需要真机确认**

   checkpoint 恢复、单观测推理和数值比较已离线执行。相机视角、反馈单位、
   实际控制周期、动作到运动的对应、工作区、停止效果与任务成功率，需要真机确认。
   本章没有发送任何动作，不能据此判断受监护部署的表现。

检查清单：能加载 final.pt；能指出一个输入张量的来源和形状；
能把一个输出分量按正确约定解码；已保存部署需要的配置、contract 与离线检查记录。

BC 训练这一部分完成。接下来 :doc:`deploy`，对照这份记录准备受监护的部署与推理。
