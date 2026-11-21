路线二 3. 离线检查 checkpoint
================================================================================

本章加载 :doc:`bc-train` 的 ``local/bc-runs/first-bc/checkpoints/final.pt``，
给它一条保存的观测，比较输出与示范动作。全部离线，不调用 reset、step 或设备连接。
目标是拿到可加载的 checkpoint，并能解释它的一条输入和一条输出。

1. 加载权重，运行一次预测
--------------------------------------------------------------------------------

用本章脚本恢复训练配置和策略，检查数据约定，再预测第 0 行保存的 obs。
只加载自己生成或可信的 checkpoint 与 pickle。

.. literalinclude:: ../examples/bc-check.sh
   :language: bash
   :start-after: # start check
   :end-before: # end check

如果上一章训练的是合成数据，用下面这一条替换预测命令；输出记录路径仍相同：

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

* 缺少 config.json／experiment.json：找回训练目录；单独复制一个 final.pt 不足以完成本路线的约定检查。
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
     - [1,128,128,3]，uint8，RGB；内容为合成随机像素，不能解释成真实场景。
   * - 示范动作
     - [0.2,0,0,0,0,0,1]；current 笛卡尔约定下请求 x 增加 0.01 m，不旋转，打开夹爪。
   * - 预测动作
     - x 分量约 0.1761，对应约 0.0088 m；y 分量约 0.3752，对应约 0.0188 m。
       模型仍输出了示范中没有的 y、z 和旋转请求。
   * - 预测夹爪
     - 约 0.1481，对应归一化开口请求约 0.5741；若使用路线一的二值 Hand，仍会超过 0.5 的开爪阈值。
       训练损失比较的是连续动作数值，不是二值设备最终是否开爪。
   * - mask 与误差
     - 第 0 行全七维人工，masked MSE 约 0.131285。它是归一化动作空间里的误差，不是 m 或 rad。

训练日志最后一步的 MSE 来自更新前抽样 batch，这里来自更新后的第 0 行，数值不必一致。
如果检查的是 mask=1111110 的行，夹爪仍会输出一个值，但它不进入该行监督误差。
这不能据此认定夹爪维度已学好；还要检查有夹爪监督的示范。

**你应该看到**：能够从任一预测分量说出动作模式、尺度和人工标记。
本例只有四次更新，输出偏差仍明显；“可以加载和推理”已经验证，任务可用性尚未验证。
真实数据应再检查来自独立完整回合的多条观测，不能只挑一条训练样本给出成功结论。

**如果不对**：

* 把预测理解成绝对 TCP 目标：重新查 action_mode，本例是相对当前状态的增量。
* 误差很小但某个维度明显不对：先看 mask，该维度可能没有参与监督。
* GELLO joint_abs 结果按米解释：用那次录制的 joint_limits 解码七个关节目标；不要套用本表。

3. 保存部署要核对的记录
--------------------------------------------------------------------------------

查看脚本生成的 offline-check.json，让部署章能把当前工作站与模型依赖逐项比较。

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
* action_mode、cartesian_semantics、动作维数、尺度、joint_limits、夹爪映射和控制频率。
  动作约定还包括 experiment 中的空间约束与 teleop_control 来源。
* 当前 ``local/station.yaml`` 的设备、home、工作区与控制设置，以及
  ``local/task.yaml`` 与训练时配置的差异。保留原 session resolved-config 作为录制依据。

**如果不对**：

* 记录与预期数据路径不同：确认上一章实际训练的是 first-demo 还是合成 demos，不给合成 checkpoint 贴真机标签。
* 只有 final.pt，没有输入记录或约定：把整个 run 目录与示范 sidecar 一起归档后，再进入部署章。
* 换了相机、坐标系或动作模式：先重新核对／准备数据与模型；不能只改部署参数让维数凑齐。

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

路线二完成。接下来 :doc:`deploy`，对照这份记录准备受监护的部署与推理。
