准备 BC 训练数据
================================================================================

本章从 :doc:`inspect` 的产物开始：数据集目录 ``local/first-demo``、
session 目录的 ``resolved-config.yaml``，以及
``local/station.yaml``、``local/task.yaml``。本章全部离线，在仓库根目录、
装有 learning 依赖的环境中操作。

这几章选 ``real_world_bc``，因为它是本仓训练注册表中直接读取示范数据集目录、
保留逐维人工监督的 BC 入口。CLI 要求显式写算法名，没有自动选择算法的默认值。
它构建 ``MaskedBC``，默认是 Gaussian actor、MLP 主干，图像用 plain_conv 编码。
另一条路径是 rl-garden 的 ACT 轨迹训练器，配套本仓的 deploy-act；
它预测动作块，checkpoint 格式不同，而且仍读 pkl 加 sidecar，
需要先用 ``rwrl dataset export --format pkl`` 从数据集导出（见 :doc:`bc-train` 末尾）。
rl-garden 另有 bc、diffusion_bc、flow_bc、mean_flow_bc、a2a_bc、consistency_distill_bc，
这些不属于本仓 real_world_bc 的单步示范加载入口。这几章不切换算法。

1. 选定同一会话的数据和配置
--------------------------------------------------------------------------------

先把数据集里的成功示范和各回合对上，避免把不同动作约定的数据混成一份训练集。

.. literalinclude:: ../examples/bc-data.sh
   :language: bash
   :lines: 2-2

.. tutorial-test: skip
.. literalinclude:: ../examples/bc-data.sh
   :language: bash
   :lines: 3-3

第一个命令是仓库自带的检查器，直接读数据集目录；第二个是本章提供的离线检查脚本。
脚本用训练同一个函数取出训练行，读每行 obs、next_obs、action 和 human_action_mask，核对形状、有限值、
动作范围与 mask；用首行 obs 比较数据集头 ``action_contract`` 中记录的观测键、形状和 dtype，
其余 obs／next_obs 检查键、形状和有限值，不逐行核对 dtype。

**你应该看到**：数据集中的回合及人工区间，和训练行（kept 回合筛选后）的 transitions 数。
按前面的配置，观测应有 ``state`` 与 ``rgb_wrist``，action 七维；
``human masks`` 显示每种人工维度组合的行数。
训练行不包含 discarded 回合，也默认剔除对齐步骤；筛选依据见 :doc:`inspect`。
先选一个已经核对的数据集。以后合并多个时，逐个完成同样检查并比较完整 contract，
不要把 discarded 回合当成成功示范。

**如果不对**：

* 不是数据集或没有训练行：检查目录里有 ``dataset.json``，以及上一章是否保留了成功人工步骤（``rwrl dataset list``）。
* ``observation shape/dtype differs from contract``：检查是否拿错数据集，或曾单独修改图像／状态。
* ``action/mask shape mismatch``：不能用七维 mask 配八维动作；回到原始数据集查来源。
* mask 全零或没有有效动作：先查数据集的人工区间。全零行不会提供 BC 监督。

接着找到录制时最终使用的配置，对照当前本地文件：

.. tutorial-test: skip
.. literalinclude:: ../examples/bc-data.sh
   :language: bash
   :start-after: # start session
   :end-before: # end session

**你应该看到**：一个或多个带 uuid 的 session 配置路径。
按同一目录中的 summary 和 provenance 确认哪一个生成了 first-demo，记下完整路径。
resolved-config 是 station 与任务设置合并后的结果；训练以它和数据集头的 action_contract 为依据。
当前 station/task 可能已被修改，不能用它们覆盖录制时的相机、尺度或坐标约定。
保留这两份本地文件供部署时检查设备连接。

**如果不对**：找不到 session 就先找回录制目录；出现多个候选时按 summary 的 output
核对，不用通配符一次把多个 YAML 交给训练。

2. 解释一次观测和动作
--------------------------------------------------------------------------------

先核对 state 的拼接顺序和动作的物理含义，再决定哪些示范可以一起使用。
录制时的 task 选择 ``[tcp_pose, tcp_vel, joint_pos, joint_vel]``，因此 state 有 27 项：
前 7 项是 XYZ 与 xyzw 四元数，接着 6 项 TCP 速度、7 项关节位置、7 项关节速度。
``rgb_wrist`` 是名为 wrist 的相机，经任务设置处理后的图像。
contract 的 ``experiment.observation`` 给出最终形状、dtype、帧堆叠与旋转转换；
``experiment.preprocess`` 给出裁剪等处理。不要重新处理已经保存的观测。

SpaceMouse tracker 例子使用 ``cartesian_delta``、``current`` 语义。
动作顺序是三维位置增量、三维旋转向量、夹爪。
前三维乘 ``xyz_scale_m=0.05``，旋转三维乘 ``rot_scale_rad=0.3``；
旋转增量左乘当前姿态。位置与旋转的坐标约定还要看 task_frame。
夹爪在 current 语义下由 [-1, 1] 映射到 [0, 1]；前面配置的二值 Hand 的 +1 是打开，-1 是关闭。
这些是送入环境的请求，实际目标还会经过限位与控制器处理。

GELLO 默认练习同样使用笛卡尔映射，也能得到七维动作。
如果完成了 :doc:`gello` 的关节扩展，检查另一份录制：

.. tutorial-test: skip
.. literalinclude:: ../examples/bc-data.sh
   :language: bash
   :start-after: # start gello
   :end-before: # end gello

**你应该看到**：关节扩展的 ``action_mode: joint_abs``、七个 arm 维度加一个夹爪维度，
并有七组 ``limits.joints``。归一化动作 a 对应关节目标
``low + (a + 1) / 2 * (high - low)``，不是弧度值本身。
若改成 joint_delta，则目标为当前关节角加 ``a * joint_scale_rad``。

例如让夹爪在空间里向右移动，SpaceMouse 可以记录一个笛卡尔 x 增量，
GELLO joint_abs 会记录七个归一化关节目标；同一句操作意图不能确定动作含义。
:doc:`inspect` 的关节 fake 示例使用 ±2 rad 限位，此时 a=0.2 对应 0.4 rad；
下面笛卡尔合成例子的 a=0.2 则对应 0.01 m 的 x 请求。±2 是测试数值，不是现场限位。

**如果不对**：

* GELLO 仍是七维：先看 action_mode，笛卡尔 GELLO 本来就是七维，不要强改成八维。
* 维数相同但含义不同：检查 cartesian_semantics、task_frame、尺度、限位、夹爪约定及
  experiment.sources 中的 teleop_control；不同约定分开训练。
* 不知道某项 state 是什么：按录制时 state_fields 与旋转转换重建顺序，不能沿用别的示例切片。

3. 没有真机数据时，生成一个可复现的小例子
--------------------------------------------------------------------------------

生成一个八步的回合，写成与 ``rwrl record`` 相同格式的数据集，
先验证后续训练和加载流程。这里使用真实环境类的空间定义，绝不调用 reset 或 step。
图像是随机 uint8，状态是合成数值，动作固定；它不是机器人仿真。

源码运行时先设置导入路径；本章的 CPU 验证关闭 CUDA：

.. literalinclude:: ../examples/bc-data.sh
   :language: bash
   :start-after: # start source-env
   :end-before: # end source-env

再运行，要求输出目录尚不存在。它写出一个只有一个回合的数据集目录 ``demos``，和一份 resolved-config：


.. tutorial-test: skip
.. literalinclude:: ../examples/bc-data.sh
   :language: bash
   :start-after: # start synthetic
   :end-before: # end synthetic

**你应该看到**：生成数据集 ``demos/``（含 ``dataset.json`` 和一个回合）以及 resolved-config.yaml。
检查脚本的实际输出节选如下，后面还会打印完整 contract：

.. literalinclude:: ../examples/bc-data-output.txt
   :language: text
   :start-after: # start summary
   :end-before: # end summary

``[1, 224, 224, 3]`` 是 batch、H、W、RGB 通道；``[1, 27]`` 是 batch 和状态维度。
四行 mask 是 1111111，另外四行是 1111110；后者夹爪不参与人工监督，
即使 action 的最后一项仍然为 +1。

**如果不对**：

* 目录已存在：选新目录并同步修改后续命令，避免混用运行结果。
* 缺少 torch 或 rl_garden：回到 :doc:`install` 使用 learning 环境；源码运行保留上面的 PYTHONPATH。
* 生成时出现 ``offline demos has no action-contract metadata``：这是建立合成空间时的提示，
  动作约定在生成数据集时已写入 ``dataset.json``。训练真实数据时出现它，说明加载的不是数据集目录，应找回数据集。

本章产物
--------------------------------------------------------------------------------

* 真机数据：已核对的数据集 ``local/first-demo``，
  以及对应的 session resolved-config 和 station/task。
* 无硬件练习：数据集 ``local/bc-synthetic/demos`` 和 resolved-config；
  后两章使用这套替代输入，不把它冒充真机示范。

.. note::

   **本章哪些内容需要真机确认**

   wrist 的实际视角与颜色、状态反馈顺序、SpaceMouse 方向与坐标系、GELLO 标定、
   动作尺度、夹爪开合和 mask 与操作者控制的对应，需要对照真机录制确认。
   合成数据只检查格式与软件流程。

检查清单：能指出一行观测的键与形状、动作模式与维数、哪些维度由人控制；
能找到同一会话的配置与数据集，并解释 SpaceMouse 和 GELLO 动作是否可混用。

接下来 :doc:`bc-train`，训练第一份 checkpoint。
