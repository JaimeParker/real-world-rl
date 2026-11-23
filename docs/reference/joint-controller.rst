关节控制器（\ ``joint_controller: trajectory_impedance``\ ）
================================================================================

rwrl 可以用一个专门给 GELLO 遥操作写的 Franka 关节控制器执行关节 servo 目标。``native/joint_controller/`` 是一个 C++/pybind11 包，
它的运动类派生自 franky 的扭矩运动类，franky 本身没有改动，做法和 ``native/serl_controller`` 相同。
franky 自带的关节运动把每个目标当作阶跃；这个运动把一串稀疏的目标变成连续的参考轨迹（位置、速度、加速度都连续，并在速度和加速度限值之内），
再用 GELLO 的关节阻抗律去跟踪它。见 `native/joint_controller/README.md <https://github.com/JaimeParker/real-world-rl/blob/main/native/joint_controller/README.md>`__\ 。

参考版本：阻抗律、速度滤波器和默认增益来自公开的 GELLO 控制器（``wuphilipp/gello_software`` @ ``9fc8c26``，Apache-2.0，Copyright (c) 2025 Franka Robotics GmbH）；
关节位置限位来自 franka_description（``franka_ros`` @ ``ddd2fff``，Apache-2.0）；运动运行在未修改的 `franky <https://github.com/TimSchneider42/franky>`__ （MIT）里。
参考生成器（五次多项式规划器）是本仓自己的实现。详见\ :ref:`来源与许可 <joint-controller-provenance-and-licences>`\ 。

本页是参考。新机器人上的选择、安装和第一次检查按步骤写在 :doc:`../tutorials/controller` 第 10 步；GELLO 本身见 :doc:`gello` 和 :doc:`../tutorials/gello`。

选择控制器
--------------------------------------------------------------------------------

关节 servo 由 station 的 ``joint_controller`` 决定，它和 ``cartesian_controller`` 互不依赖，两者可以任意组合：

.. list-table::
   :header-rows: 1

   * - ``joint_controller``
     - 谁执行关节 servo
     - 要装什么
   * - ``impedance`` （默认）
     - franky 的 ``JointImpedanceTracker``，目标按 ``control_hz`` 到达时直接跟踪；增益来自 ``control:``
     - 官方 franky wheel 即可
   * - ``trajectory_impedance``
     - 本页的控制器：每个新目标经五次多项式规划器变成 1 kHz 参考轨迹，再用 GELLO 的关节阻抗律跟踪
     - 同一次构建的 franky、``serl_controller`` 和 ``joint_controller``，需要 ``arm_backend: franky``

``move_to_joints`` （归位）不受影响：``cartesian_controller: serl`` 时仍是 SERL 的 10 s 关节复位，其余仍是 franky 的 ``JointMotion``\ 。

.. code-block:: yaml

   # station (robot PC), e.g. local/station.yaml; restart `rwrl daemon` afterwards
   joint_controller: trajectory_impedance   # impedance | trajectory_impedance
   realtime: enforce                        # PREEMPT_RT kernel recommended
   joint_trajectory: {}                     # optional overrides, see Parameters

为什么用它做 GELLO：默认的 ``impedance`` 在每个 ``control_hz`` 周期收到一个阶跃目标；``trajectory_impedance`` 在目标之间补出连续的位置、速度和加速度，从臂运动更平滑。
建议 GELLO 采集把 ``real_robot.control_hz`` 提到 15–30 Hz：间隔（10 Hz 时 100 ms）越短，每次重规划的位移越小，主臂读数到执行的延迟也越小。
这是依据规划器行为的建议，不是实测结论。每步的上限（``max_delta_per_step.joint_rad``\ 、\ ``joint_scale_rad``\ ）按每步计，提高 ``control_hz`` 后相同的每步值对应更快的速度，
实际速度还受 ``joint_trajectory`` 里 ``max_velocity`` 的限制。``control_hz`` 属于实验约定，采集、训练、部署必须一致。

.. _joint-controller-installing:

安装
--------------------------------------------------------------------------------

``joint_controller`` 继承 franky 的 C++ 类和 pybind11 类 ``franky.BaseTorqueMotion``\ ，与 ``serl_controller`` 的耦合规则相同：
**franky、joint_controller 和 serl_controller 必须来自同一次构建**\ （同一台机器或容器、同一个编译器、pybind11 3.0.4、libfranka 和 franky 头文件），并一起部署。
不要把它装在来自别处的 franky（包括 PyPI 的官方 wheel）旁边。

* **在机器人主机上直接构建**\ （不需要 Docker），项目环境已激活：

  .. tutorial-test: skip
  .. code-block:: bash

     LIBFRANKA_VERSION=0.17.0 bash native/joint_controller/scripts/install-on-ubuntu.sh

  环境里已经有用同一个 libfranka 和同一个 ``WORK`` （默认 ``~/.cache/rwrl-native-build``）从源码构建的 franky v2.0.0 时，脚本复用它；
  否则先运行 ``native/serl_controller/scripts/install-on-ubuntu.sh``\ （构建 libfranka 和 franky，并重建 ``serl_controller``\ ），再用 ``--no-build-isolation`` 装 ``joint_controller``\ ，
  运行 CTest（含 franky 运动测试）、Python 测试、导入检查，以及 ``build_info.json`` 的 ``libfranky.so`` sha256 与已装文件的配对检查。
  ``REBUILD_FRANKY=1`` 强制重做 franky 那一步，日志在 ``$WORK/logs/jc_*``\ 。

* **用 Docker 一次构建三个 wheel**\ （给一样的机器人主机用）：

  .. tutorial-test: skip
  .. code-block:: bash

     WORK=$HOME/rwrl-franky-build native/joint_controller/docker/build-on-host.sh
     # $WORK/out/wheels: franky_control-*.whl, serl_controller-*.whl, joint_controller-*.whl, SHA256SUMS
     pip uninstall -y franky-control serl-controller joint-controller
     pip install franky_control-*.whl serl_controller-*.whl joint_controller-*.whl

  ``WITH_SERL=0`` 只构建 franky 和 joint_controller。

两种方式的 ``build_info.json`` 含 ``joint_controller_version`` 和 ``gello_commit``\ 。装好 ``joint_controller`` 之后不要运行 ``rwrl doctor --install-sdk`` 或 ``pip install franky-control``\ ：
它们会换掉这一对。station 已写 ``joint_controller: trajectory_impedance`` 时，doctor 拒绝 ``--install-sdk`` 并报告 BLOCKED。

实时内核
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

控制循环是 1 kHz 的扭矩循环，建议和 ``serl`` 一样使用 PREEMPT_RT 内核并设置 ``realtime: enforce``\ （安装见 :doc:`../tutorials/controller` 第 3 步）。
``rwrl setup --joint-controller trajectory_impedance`` 会建议 ``enforce``\ ；doctor 在不是 ``enforce`` 时给 WARN，而不是失败。

诊断检查
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

使用 ``joint_controller: trajectory_impedance`` 时，\ ``rwrl doctor`` 给这台机器人多加这几行（见\ :ref:`doctor 参考 <doctor-joint-controller-checks>`\ ）：

.. list-table::
   :header-rows: 1

   * - 行
     - 层
     - 结果
     - 含义与处理
   * - ``sdk:joint_controller``
     - 1 sdk
     - PASS ``joint_controller franky build X; installed franky Y`` / FAIL
     - 能导入 ``joint_controller`` 并有 ``TrajectoryImpedanceMotion``\ ，且 ``build_info`` 记录的 franky 版本（``franky_dist_version``\ ）等于已装 franky 的版本。FAIL 时重跑安装脚本，或从同一个构建目录重装三个 wheel。
   * - ``mode:joint_trajectory_realtime``
     - 5 mode
     - PASS / WARN ``joint_controller: trajectory_impedance with realtime: ...``
     - ``realtime`` 不是 ``enforce`` 时 WARN。安装 PREEMPT_RT 内核并设置 ``realtime: enforce``\ 。

运行方式
--------------------------------------------------------------------------------

.. list-table::
   :header-rows: 1

   * - rwrl 调用
     - 实现
   * - ``set_control_mode(JOINT_SERVO)``
     - 用当前 ``joint_trajectory`` 新建 ``TrajectoryImpedanceMotion``\ ，``robot.move(motion, asynchronous=True)``\ ；参考从测得的 ``q`` 开始，静止，第 0 个 tick 的弹簧项为 0
   * - ``servo_joints(q)``
     - 先经 arm 的安全层（每步从实测起不超过 ``max_delta_per_step.joint_rad``\ ，再夹进 ``joint_limits``\ ），再 ``motion.set_target(q)``\ ；目标含非有限值时被拒绝，arm 报 ``joint target rejected (non-finite)``
   * - ``update_joint_trajectory_params(partial)``
     - 部分合并并校验；控制器运行时下一个 tick 生效（见\ :ref:`在线更新 <joint-controller-live-update>`\ ）
   * - ``stop()``\ 、离开该模式
     - franky 的扭矩停止；下一个 servo 目标会重新启动控制器（从测得的关节开始）
   * - 反射（任何运动错误）
     - 下一个 servo 目标按 ``serl_auto_recover`` 清错误并在测得的关节处重启控制器，每回合最多 ``max_per_episode`` 次；超过或清不掉时 servo 报可恢复错误，需要 ``rwrl recover``\ 。\ ``stop()`` 引起的结束不算反射，不自动恢复
   * - ``move_to_joints``\ （IDLE）
     - 不变（franky ``JointMotion``\ ，或 ``cartesian_controller: serl`` 时 SERL 的关节复位）

控制律
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

每个 tick（``dt`` 是实测周期：franky 的 ``time_step``\ ，正常 1 ms，丢包后 2–5 ms）：

1. **参考。** ``(q_ref, dq_ref, ddq_ref)`` 前进 ``dt``\ 。不是有限数的新目标被拒绝，否则夹进 ``[lower_limit + limit_margin, upper_limit - limit_margin]``\ 。
   每个关节都从当前参考状态（位置、速度、加速度保持连续）重新规划到 ``(目标, 0, 0)``\ ，每个关节一条五次多项式，所有关节共用一个时长 ``T``\ ，之后精确保持目标。一个 tick 内来了多个目标时只用最新的。
2. **超时。** ``command_timeout_s`` 内没有新目标：有界停止并保持；``request_stop()`` 立即做同样的事。下一个目标恢复跟踪。
3. **速度滤波（GELLO）。** ``dq_f = (1 - alpha) dq_f + alpha dq``\ ，起点 ``dq_f = 0``\ 。
4. **力矩（GELLO）。** ``tau = K (q_ref - q) - D dq_f``\ ；``compensate_coriolis`` 为真时加模型的科氏力矩（从不加重力，libfranka 会加），再把每个关节的 ``|tau - tau_prev|`` 限制在 ``max_delta_tau`` 以内。
5. **起点。** 参考从测得的 ``q`` 开始，静止。替换一个正在运行的扭矩运动时，\ ``tau_prev`` 取 franky 上一次的力矩指令，否则为 0。

上游 GELLO 把原始的主臂关节状态直接作为 ``q_goal``\ ；这里的 ``q_goal`` 是 ``q_ref``\ 。franky 的 ``Robot.move`` 默认值（``limit_rate=False``\ 、\ ``cutoff_frequency=100``\ ）可以保持：力矩变化率已经在控制律里限制到每个 tick ``max_delta_tau``\ 。

规划器与有界停止
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**时长选择。** 对每个候选 ``T``\ ，每个关节的五次多项式都被确定。所有关节的 ``|v| <= max_velocity``\ 、\ ``|a| <= max_acceleration`` 且位置在限位之内时，候选可行；
这些都是在多项式的精确极值处检查的（急动度的根、加速度和速度的根）。``T`` 从 ``min_segment_duration_s`` 和每个关节的两个下界（``|h| / v_max``\ ，以及双积分器在 ``|a| <= a_max`` 下从 ``(p0, v0)`` 到目标处静止的 bang-bang 最短时间）中的最大值开始，每步乘 1.1 增大直到所有关节可行，最后一步二分 20 次，取可行的一端。
从静止出发时结果就是精确的最小值。

**规划失败时。** 从运动状态出发，可能不存在公共的 ``T``\ ：一个仍在加速的关节只在较短的 ``T`` 可行，另一个要走很远的关节需要较长的 ``T``\ 。
这时目标保持待定，参考用有界停止刹车，并且每个 tick 重试规划（静止时，限位之内的每个目标都可行）。
README 记录的模拟结果：在抖动目标加大跳变的 300 000 tick 测试里，这影响 2.7% 的 tick，最长等待 0.46 s；主臂平滑移动、10、20 或 30 Hz 时从未发生。

**有界停止。** 每个关节从 ``(p0, v0, a0)`` 到 ``p0 + v0 T / 2 + a0 T² / 12`` 处静止，速度单峰：只在参考仍在加速时 ``|v|`` 上升，然后单调降到 0，不反向。
``T`` 取在限值内（速度、加速度、不反向、位置限位）的最小网格点；各关节用各自的时长。没有可行停止的关节（在速度限值上仍在加速）保持当前可行的、以静止结束的段，并每个 tick 重试。
停止点在参考的前方，而不是超时时刻的参考位置：在那里停会让每个移动的关节反向。

跟随滞后
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

每次重规划都在最新目标处静止，所以移动的主臂被跟随时滞后约一个刹车距离，滞后随 ``max_acceleration`` 增大而减小。主臂平滑地以约 0.5 rad/s 移动时：

.. list-table::
   :header-rows: 1

   * - ``max_acceleration``
     - 滞后（约）
   * - 2 rad/s²
     - 0.20 rad
   * - 4 rad/s² （默认）
     - 0.06 rad
   * - 8 rad/s²
     - 0.04 rad

主臂 0.19 rad/s 时滞后保持在 0.02 rad 以内。这些数字来自 README 的模拟，不是真机测量。更快的跟随调大 ``joint_trajectory.max_acceleration``\ ，更柔和的运动调小。

.. _joint-controller-parameters:

参数
--------------------------------------------------------------------------------

station 的 ``joint_trajectory:`` 块是这台机器人的覆盖值；省略的键使用原生默认值（``joint_controller.default_params(robot_model)``\ ，限位按 ``robot_model`` 取 Panda 或 FR3，不使用 station 的 ``joint_limits``\ ）。
12 个键如下，带 (7) 的是长度为 7 的向量（单位逐关节）：

.. list-table::
   :header-rows: 1

   * - 键
     - 默认值
     - 单位
     - 规则
     - 含义与来源
   * - ``stiffness`` (7)
     - 240, 240, 240, 240, 100, 60, 20
     - Nm/rad
     - ≥ 0
     - 阻抗律的 K。GELLO ``controllers.yaml`` ``k_gains``
   * - ``damping`` (7)
     - 20, 20, 20, 10, 10, 10, 5
     - Nms/rad
     - ≥ 0
     - 阻抗律的 D。GELLO ``d_gains``
   * - ``velocity_filter_alpha``
     - 0.99
     - 无
     - [0, 1]
     - 速度滤波系数。GELLO ``k_alpha``
   * - ``max_velocity`` (7)
     - 0.6（关节 1–4），0.75（5–7）
     - rad/s
     - > 0
     - 规划器的每关节速度上限。规格值
   * - ``max_acceleration`` (7)
     - 4.0
     - rad/s²
     - > 0
     - 规划器的每关节加速度上限；决定跟随滞后（见上表）。本仓选定
   * - ``min_segment_duration_s``
     - 0.1
     - s
     - > 0
     - 一段轨迹的最短时长。规格值
   * - ``lower_limit`` (7)、\ ``upper_limit`` (7)
     - ``robot_model`` 的 franka_description 限位
     - rad
     - lower + 2·margin < upper
     - 关节位置限位。franka_ros ``joint_limits.yaml``
   * - ``limit_margin``
     - 0.05
     - rad
     - ≥ 0
     - 目标被夹到限位内侧的余量。规格值
   * - ``command_timeout_s``
     - 0.5
     - s
     - > 0
     - 这么久没有新目标就有界停止并保持。GELLO ``kJointStateTimeout``
   * - ``max_delta_tau``
     - 1.0
     - Nm／tick
     - > 0
     - 每个 tick 每关节力矩变化的上限。规格值（与 SERL 的 ``delta_tau_max`` 相同）
   * - ``compensate_coriolis``
     - ``true``
     - 无
     - 布尔
     - 是否加科氏力矩（从不加重力）。规格值

所有值必须是有限数。不合法的值被\ **拒绝**\ ，不会像 SERL 的参数那样被夹到范围内。station 加载时只检查键名（未知键直接报错，列出可选键）；数值由原生包在连接时和在线更新时校验。
改变参考相关的限值（限位、速度、加速度）会让当前目标（重新裁剪后）从当前状态重新规划。

.. _joint-controller-live-update:

在线更新
--------------------------------------------------------------------------------

``arm.joint_trajectory_params`` 返回完整的生效参数（工作站覆盖值加原生默认值；不是 ``trajectory_impedance`` 时是 ``None``\ ），
``arm.update_joint_trajectory_params({...})`` 做部分更新：

.. code-block:: python

   arm.joint_trajectory_params                              # the full effective set
   effective = arm.update_joint_trajectory_params(
       {"max_acceleration": [6.0] * 7})                     # merged into the current values

* 控制器运行时，更新由原生的 ``update_params`` 合并并校验，**下一个 tick** 生效；全部生效或者报错且什么都不改。
* 控制器没在运行时（未连接或不在关节 servo 模式），先用合并后的完整参数构造一个原生运动来校验，再保存，下次启动时使用。
* 更新保留到再次修改，也被反射后的自动恢复和 ``stop()`` 之后的重启沿用；它不会改写 station 文件。
* 不是 ``trajectory_impedance`` 的 arm 调用更新会报 ``has no joint trajectory parameters``\ ，提示使用 franky 与 ``joint_controller: trajectory_impedance``\ 。

通过守护进程（\ :doc:`daemon`\ ）时，方法名是 ``arm.joint_trajectory_params`` （只读，断线重连后可安全重试）和 ``arm.update_joint_trajectory_params`` （命令，持有 daemon 的锁执行）；
远程 arm 对象 ``RemoteArm`` 提供同名方法，键名在客户端先检查一遍。没有对应的 ``rwrl`` 命令行。

遥测
--------------------------------------------------------------------------------

原生运动有 ``telemetry()``\ ，返回每个 tick 的快照：\ ``ticks``\ 、\ ``rel_time``\ 、\ ``q``\ 、\ ``dq``\ 、\ ``dq_filtered``\ 、\ ``coriolis``\ 、\ ``q_ref``\ 、\ ``dq_ref``\ 、\ ``ddq_ref``\ 、
``target`` （最后接受的、已裁剪的目标）、\ ``goal`` （参考将在哪里停下：目标或停止点）、\ ``tau_cmd``\ 、\ ``segment_t``\ 、\ ``segment_T``\ 、\ ``time_since_target``\ 、\ ``timed_out``\ 、\ ``stopped``\ 、
``target_pending``\ 、\ ``plan_retries``\ 、\ ``applied_command_seq``\ 、\ ``applied_params_seq``\ 。
rwrl 目前不通过 arm、daemon 或命令行暴露它（后端只在自动恢复重启后读 ``ticks`` 等第一个 tick）；需要时直接用原生 Python API：

.. code-block:: python

   import joint_controller as jc

   params = jc.default_params("fr3")               # "panda" | "fr3"
   motion = jc.TrajectoryImpedanceMotion(params)   # a franky.BaseTorqueMotion
   jc.validate_params(params)                      # validate a full dict without a robot
   jc.build_info()                                 # build_info.json

``jc.TrajectoryImpedanceLaw(params)`` 不需要 franky 和机器人，可以离线运行控制律（``step(q, dq, coriolis, dt)``\ ）。

开销
--------------------------------------------------------------------------------

``update()`` 不分配内存，每个 tick 的工作量有界（每次规划最多 50 次网格加 20 次二分的可行性检查）。README 在 WSL2 笔记本（``-O2``\ ）上的测量：模拟中每个 tick 平均约 1 µs，一次平均重规划 35 µs，
最坏的 tick（一次失败的规划加刹车的开头）取 5 次重复的最小值是 104 µs；单次运行的最大值约 0.5 ms，是这台笔记本的抖动。真机的实时抖动要在现场测量。

.. _joint-controller-provenance-and-licences:

来源与许可
--------------------------------------------------------------------------------

``joint_controller`` 整个包的许可是 ``MIT AND Apache-2.0``\ （见 ``native/joint_controller/LICENSE``\ ，它列出每一部分和它的许可）：

.. list-table::
   :header-rows: 1

   * - 部分
     - 来源
     - 许可
   * - 关节阻抗律、速度滤波器、默认增益（刚度、阻尼、滤波系数）和命令超时
     - 公开的 GELLO 控制器 ``wuphilipp/gello_software`` @ ``9fc8c26``\ （\ ``ros2/src/franka_fr3_arm_controllers/`` 的 ``joint_impedance_controller.cpp``\ 、\ ``config/controllers.yaml``\ ），Copyright (c) 2025 Franka Robotics GmbH
     - Apache-2.0（``LICENSE.Apache-2.0`` 是该目录的许可文本）
   * - Panda／FR3 关节位置限位表
     - franka_description（``frankaemika/franka_ros`` @ ``ddd2fff``\ ）
     - Apache-2.0
   * - 参考生成器（五次多项式规划器、时长选择、有界停止）、控制律的其余部分（参考、超时停止、科氏力、力矩变化率限制）、参数表与校验、franky 运动类、Python 绑定、测试与脚本
     - 本仓自己的代码
     - MIT
   * - franky（被链接的未修改的 v2.0.0）
     - `TimSchneider42/franky <https://github.com/TimSchneider42/franky>`__
     - MIT

速度、加速度上限、``min_segment_duration_s``\ 、\ ``limit_margin`` 和 ``max_delta_tau`` 不来自 GELLO，是本仓的规格值（``max_delta_tau`` 与 SERL 的 ``delta_tau_max`` 相同）。
README 的“致谢与引用”一节同样列出上游 GELLO 和 franky。

测试
--------------------------------------------------------------------------------

原生包有自己的测试（仓库根的测试不收集 ``native/joint_controller/tests``\ ）：C++ 的规划器、控制律、参数、浸泡测试（2 × 300 000 tick 对模拟机械臂，含 GELLO 式 10–30 Hz 重定向、超出限位的跳变、非有限目标、超过超时的停顿、丢包和在线增益变化，每个 tick 检查所有不变量）、franky 运动测试；Python 绑定测试。
仓库的 Python 测试使用 fake 的 ``joint_controller`` 模块，覆盖 station 校验、模式矩阵、在线更新、doctor 行和向导标志，不需要硬件。
