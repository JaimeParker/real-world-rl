选择并配置 Franka 控制器
================================================================================

本章给一台\ **新接入**\ 的 Franka 选定笛卡尔控制器，在机器人主机（NUC）上装好它需要的软件，
把对应字段写进 station，再用 doctor 和一次归位检查。
先完成 :doc:`station` 的第 1–3 步（station 能加载、doctor 能连上机械臂）。
本章命令都在 NUC 的仓库根目录、已激活的项目环境中运行。

``cartesian_controller`` 是 station 里的一行，决定\ **谁把末端目标变成电机力矩**\ 。
策略或遥操作每一步给出一个末端目标（位置 + 姿态），控制器负责让机械臂跟上它。

.. tip::

   有两个开关容易混，它们互不依赖：

   * station 的 ``cartesian_controller``：**机器人端**\ 怎么执行目标（本章）。
   * ``teleop run --mode tracker|serl``：**SpaceMouse 端**\ 怎么把推帽变成目标（见 :doc:`spacemouse`）。

   SpaceMouse 的读数先在桌面坐标系里，再转到机器人基座系，最后才交给本章的控制器；
   这条链在 :doc:`station` 第 11 步讲。

1. 认识三种控制器，选一个
--------------------------------------------------------------------------------

.. list-table::
   :header-rows: 1

   * - ``cartesian_controller``
     - 谁在执行
     - 机器人主机要装什么
     - 适合
   * - ``impedance`` （默认）
     - franky 的扭矩级阻抗 tracker，像弹簧一样把 TCP 拉向目标；增益来自 station 的 ``control:``
     - 与机器人匹配的官方 franky wheel；普通内核也能跑
     - 第一次接机器人、先把其他部分调通
   * - ``internal``
     - 机器人自带的控制器。每个目标变成一段基座系速度运动，以当前误差除以 0.2 s 计算速度，
       限速 0.25 m/s、1.0 rad/s，300 ms 没有新目标就自己减速停下
     - 官方 franky wheel + PREEMPT_RT 内核
     - 希望机器人自己处理 IK 和负载
   * - ``serl``
     - SERL 原版的笛卡尔阻抗控制器（从 ``serl_franka_controllers`` 逐行移植，结果逐位相同）；
       ``move_to_joints`` 也换成 SERL 的 10 s 关节复位；增益来自 station 的 ``serl:``
     - 在本机\ **一起从源码构建**\ 的 franky + ``serl_controller``，建议 PREEMPT_RT
     - 想要 SERL／HIL-SERL 的操作手感

三种模式下，关节 servo 都是 franky 的关节阻抗 tracker，``move_to_pose`` 都是 Ruckig 轨迹。

和 SpaceMouse 两种模式的搭配：

.. list-table::
   :header-rows: 1

   * - 控制器 \\ SpaceMouse
     - ``--mode tracker`` （默认）
     - ``--mode serl``
   * - ``impedance``
     - :doc:`spacemouse` 默认用的组合，按 button_0 接管
     - 可用；推帽即接管，但执行端不是 SERL 的控制器
   * - ``internal``
     - 可用
     - 可用
   * - ``serl``
     - 可用；推荐先用它做第一次运动
     - 最接近 HIL-SERL：推帽即接管，SERL 控制器执行

拿不准时先选 ``impedance``：它不需要额外构建，以后换控制器只改 station 的一行。

**你应该看到**：你已经决定这台机器人用哪一个控制器。

**如果不对**：不确定内核是否实时、要不要 ``serl``：先按 ``impedance`` 做完本章和 :doc:`spacemouse`，
以后回到本章第 3、4 步再切换。

2. 记下这台机器人的三个事实
--------------------------------------------------------------------------------

后面的安装和配置都由这三个事实决定，先把它们写在纸上。

**型号** ``robot_model``。在 Desk 打开 Settings → System，看系统版本：

.. list-table::
   :header-rows: 1

   * - Desk 系统版本
     - 机器人
     - 填写
   * - 4.x
     - Franka Emika Robot（FER，常叫 Panda）
     - ``robot_model: panda``
   * - 5.x
     - Franka Research 3
     - ``robot_model: fr3``

它决定 ``rwrl setup`` 给 ``joint_limits`` 的默认值（Panda 和 FR3 的手册限位，第 6、7 关节差别很大），
也决定 SERL 关节复位用哪张限位表。

**robot server version**：机器人控制柜里那一端通信协议的版本号，决定 libfranka 能用哪个版本。
最可靠的读法是让 doctor 去握手：

.. literalinclude:: ../examples/controller.sh
   :language: bash
   :start-after: # start server-version
   :end-before: # end server-version

装的 franky 自带的 libfranka 和机器人不匹配时，``handshake`` 一行写出两个版本：

.. literalinclude:: ../examples/controller-output.txt
   :language: text
   :start-after: # start mismatch
   :end-before: # end mismatch

``robot server 9`` 里的 9 就是要记的数。握手\ **通过**\ 时，说明已装 franky 的 libfranka 正好合适，
查看它的版本：

.. tutorial-test: skip
.. literalinclude:: ../examples/controller.sh
   :language: bash
   :start-after: # start franky-version
   :end-before: # end franky-version

输出 ``2.0.0+libfranka.0.17.0`` 表示 libfranka 0.17.0；官方 v2.0.0 的默认 wheel 版本号为 ``2.0.0``，使用 libfranka 0.21.2。
自建包或其他 franky 版本不能仅凭没有后缀推断 libfranka 版本。
在下表里按 libfranka 版本反查 server version。

**libfranka 版本**：按 server version 查下表。表来自 franky v2.0.0 的
`doc/compatibility.json <https://github.com/TimSchneider42/franky/blob/v2.0.0/doc/compatibility.json>`_
（每个 server version 需要的最低 libfranka，即 Franka 的
`compatibility matrix <https://frankarobotics.github.io/docs/compatibility.html>`_ 与 libfranka-common 的版本定义），
“建议版本”是 franky 为该 server version 发布 wheel 时用的 libfranka：

.. list-table::
   :header-rows: 1

   * - robot server version
     - Desk 系统版本（参考）
     - 可用的 libfranka
     - 建议 ``LIBFRANKA_VERSION``
   * - 10
     - 5.9.0 起（FR3）
     - ≥ 0.18.0
     - 0.21.2
   * - 9
     - 5.7.2 等（FR3）
     - 0.15.0 – 0.17.x
     - 0.17.0（本仓已在干净 Ubuntu 22.04 上完整验证）
   * - 8
     - 5.x（FR3）
     - 0.14.1 – 0.14.x
     - 0.14.2
   * - 7
     - 5.x（FR3）
     - 0.13.3 – 0.14.0
     - 0.13.3
   * - 6
     - 5.2.0 起（FR3）
     - 0.10.0 – 0.13.2
     - 0.12.1
   * - 5
     - 4.2.x（FER 的最后一版）
     - 0.9.1 – 0.9.x
     - 0.9.2
   * - 4
     - 4.x（FER）
     - 0.8.0 – 0.9.0
     - 0.8.0
   * - ≤ 3
     - 更早的 FER
     - < 0.8
     - ``serl`` 不支持（需要 libfranka ≥ 0.8）

server version 以握手报告为准；“Desk 系统版本”一列只用来交叉核对，精确对应关系看上面的 compatibility matrix。
server version ≥ 6 的一定是 5.x 系统，也就是 FR3；≤ 5 的是 FER。和 Desk 读到的型号不一致时，以 Desk 为准并重新检查。

**内核是否实时**：

.. tutorial-test: skip
.. literalinclude:: ../examples/controller.sh
   :language: bash
   :start-after: # start rt-check
   :end-before: # end rt-check

**你应该看到**：三行记录，例如 “fr3；server 9 → libfranka 0.17.0；内核不是 PREEMPT_RT”。

**如果不对**：

* doctor 的 ``handshake`` 连不上（不是版本不匹配）：回到 :doc:`station` 第 3 步修网络、FCI 和控制权。
* 握手报告的 server version 推出的型号和 Desk 不一致：先在 Desk 确认，不要凭外观猜型号。

3. 装实时内核（internal、serl 需要）
--------------------------------------------------------------------------------

``internal`` 的速度接口和 ``serl`` 的 1 kHz 扭矩循环都怕延迟：普通内核偶尔让一个控制周期迟到，
机器人就触发通信类反射并停下。SERL 当年就是在 ``realtime_config: enforce`` 下运行的。
``impedance`` 可以跳过本步。

上一步检查不是 PREEMPT_RT 时，按 Franka 官方文档
`Setting up the real-time kernel <https://frankarobotics.github.io/docs/installation_linux.html#setting-up-the-real-time-kernel>`_
安装 PREEMPT_RT 内核（Ubuntu 22.04／24.04 也可以用 Ubuntu Pro 的 ``sudo pro enable realtime-kernel``），重启进入新内核。
然后给当前用户实时优先级，重新登录后再跑一次第 2 步的检查：

.. tutorial-test: skip
.. literalinclude:: ../examples/controller.sh
   :language: bash
   :start-after: # start rt-limits
   :end-before: # end rt-limits

**你应该看到**：``uname -v`` 含 ``PREEMPT_RT``，``/sys/kernel/realtime`` 为 1，``ulimit -r`` 为 99。

**如果不对**：

* ``ulimit -r`` 仍是 0：确认已经重新登录，``id`` 里有 ``realtime`` 组。
* NVIDIA 驱动装不上实时内核：见 franky 的 FAQ（``IGNORE_PREEMPT_RT_PRESENCE=1``）；或把 GPU 程序放到别的机器。

4. 装控制器需要的软件
--------------------------------------------------------------------------------

.. important::

   **安装顺序规则。** ``serl`` 需要的 franky 由本步的脚本从源码构建，它和 ``serl_controller`` 是\ **一对**\ 。
   用 ``serl`` 时\ **不要**\ 运行 ``rwrl doctor --install-sdk`` 或 ``pip install franky-control``：
   它们会换成官方 wheel，这一对就拆散了（station 已写 ``cartesian_controller: serl`` 时，
   doctor 会拒绝 ``--install-sdk`` 并给出重建命令）。
   脚本之前装过官方 wheel 没关系，脚本会先卸载它。

4A. impedance 和 internal：官方 franky wheel
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

:doc:`install` 的 ``franky`` extra 没有固定版本；franky v2.0.0 的 PyPI 默认构建使用
libfranka 0.21.2（server 10），其他版本先核对其兼容表。第 2 步握手通过时什么都不用做。
握手报告版本不匹配时，让 doctor 按握手读到的 server version 换 wheel：

.. literalinclude:: ../examples/controller.sh
   :language: bash
   :start-after: # start install-wheel
   :end-before: # end install-wheel

**你应该看到**：doctor 列出 ``PLAN sdk:franky`` 和一条 ``pip install franky-control --extra-index-url .../by-robot-server-version/9/``，
确认后安装；重跑 ``rwrl doctor`` 时 ``handshake`` 通过。

**如果不对**：

* 报 ``sdk.franky.serl`` BLOCKED：station 已经是 ``serl``，按 4B 构建，不要用 ``--install-sdk``。
* 计划里有 ``Advice: serl_controller is installed ...``：这台机器装过 serl 那一对；换成官方 wheel 后 ``serl`` 不能用，
  以后要用 ``serl`` 时重做 4B。

4B. serl：在机器人主机上一起构建 franky 和 serl_controller
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

``serl_controller`` 是一个 C++ 扩展，它的运动类直接继承 franky 的 C++ 类，并链接 franky 的 ``libfranky.so`` 和 libfranka。
所以它只能和\ **同一次构建、同一个 libfranka**\ 的 franky 一起用，PyPI／GitHub 上的官方 franky wheel 不行。
仓库自带的脚本在 NUC 自己的 Ubuntu 上完成整个构建，不需要 Docker：

1. 用 apt 装编译工具和库（``build-essential``、``cmake``、``libboost-all-dev`` 等，需要 sudo）；
2. 在当前环境装构建工具（CMake ≥ 3.24、scikit-build-core、pybind11 3.0.4，与 franky v2.0.0 相同）；
3. 下载 franky v2.0.0 源码，核对 commit，确认未被修改；
4. 用 franky 自己的依赖脚本从源码装 Eigen、pybind11、ruckig 和指定版本的 libfranka（0.14 起还有 pinocchio）到 ``/usr/local``；
   已经装了同版本 libfranka 时跳过；
5. 从源码把未修改的 franky 装进当前环境，版本号带 ``+libfranka.X``；
6. 用同一个编译器、同一个 franky 构建并安装 ``serl_controller``；
7. 跑 C++ 测试（含与 SERL 原版逐 tick 比较的 golden 测试）、Python 绑定测试、导入检查，最后跑 doctor 的配对检查。

把第 2 步查到的版本填进 ``LIBFRANKA_VERSION`` 后运行（示例是 server 9 → 0.17.0）：

.. tutorial-test: skip
.. literalinclude:: ../examples/controller.sh
   :language: bash
   :start-after: # start install-native
   :end-before: # end install-native

.. list-table::
   :header-rows: 1

   * - 环境变量
     - 怎么填
   * - ``LIBFRANKA_VERSION``
     - 必填，第 2 步表里的“建议版本”。
   * - ``WORK``
     - 源码、构建目录和日志的位置，默认 ``~/.cache/rwrl-native-build``。
   * - ``REPLACE_LIBFRANKA=1``
     - ``/usr/local`` 里已经有\ **另一个**\ 版本的 libfranka 时，脚本默认停下（换掉它会影响这台电脑上所有用 libfranka 的程序）；确认要换才设置。
   * - ``SKIP_APT=1`` ／ ``SKIP_DEPS=1``
     - apt 包或 C++ 依赖已经自己装好时跳过对应步骤。
   * - ``PIP_INDEX_URL``
     - PyPI 慢时换成镜像，例如 ``https://mirrors.aliyun.com/pypi/simple/``。

在 64 核的机器上从零装完约 11 分钟；普通 NUC（4–8 核）预计 30–60 分钟，大部分时间在编译 pinocchio（libfranka ≥ 0.14 才需要）。
日志在 ``$WORK/logs/``；未设置 ``WORK`` 时为 ``~/.cache/rwrl-native-build/logs/``。
脚本面向 Ubuntu（20.04／22.04／24.04）；其他发行版只打印警告，不保证构建可用。
它要求已激活的项目环境；项目要求 Python ≥ 3.10，
Ubuntu 20.04 自带 3.8，需要先另装 3.10 并用它建环境。

**你应该看到**：脚本最后打印下面这些行（版本、路径和 build id 以你的机器为准）：

.. literalinclude:: ../examples/controller-output.txt
   :language: text
   :start-after: # start native
   :end-before: # end native

``golden_equivalence`` 和 ``joint_reset_golden`` 通过，说明在这台机器、这个编译器上，移植的控制律和 SERL 原版每个 tick 都一致；
最后的 ``PASS  sdk:serl_controller`` 就是 doctor 的配对检查。之后也可以随时确认：

.. tutorial-test: skip
.. literalinclude:: ../examples/controller.sh
   :language: bash
   :start-after: # start install-native-check
   :end-before: # end install-native-check

``build_info`` 里 ``franky_layout`` 是 ``source``，``libfranka_path`` 是脚本装的系统 libfranka。

**如果不对**：

* ``attempt 1/3 failed (network?)``：GitHub 连接不稳，脚本会自动重试；全部失败时设置 ``RETRIES=6`` 再跑一次，已完成的步骤会很快。
* ``activate the project venv / conda env first``：先 ``source .venv/bin/activate``，不要装进系统 Python。
* ``libfranka 0.x is installed; installing ... replaces it``：本机已有别的 libfranka（例如给 ROS 用的）。确认可以替换再设 ``REPLACE_LIBFRANKA=1``。
* 某一步失败：脚本打印该步日志的最后几十行，完整日志在 ``logs/`` 下对应文件。
* 以后 ``pip install franky-control`` 或 ``--install-sdk`` 换掉了 franky：``sdk:serl_controller`` 会 FAIL，重跑本步脚本。
* 重新安装或升级了 libfranka：``sdk:serl_controller`` 报 ``changed since serl_controller was built``，重跑本步脚本。

这对 franky 是未修改的 v2.0.0，``impedance`` 和 ``internal`` 照常可用，所以之后切换控制器不需要重装。

.. note::

   **可选：在别处构建 wheel。** 有多台一样的 NUC（同 Ubuntu、同 Python、同 libfranka）时，可以在一台有 Docker 的
   构建机上用仓库的 Docker 配方构建一对 manylinux wheel，再把整个目录拷到各台 NUC 一起安装。NUC 本身不需要 Docker：

   .. tutorial-test: skip
   .. literalinclude:: ../examples/controller.sh
      :language: bash
      :start-after: # start wheels-docker
      :end-before: # end wheels-docker

   .. tutorial-test: skip
   .. literalinclude:: ../examples/controller.sh
      :language: bash
      :start-after: # start wheels-install
      :end-before: # end wheels-install

   这对 wheel 自带 libfranka，必须两个一起装；Python 版本必须和 wheel 的 cp 标签一致（默认 cp310）。

5. 写 station：控制器、型号与实时
--------------------------------------------------------------------------------

改之前先备份，回滚时直接拷回来：

.. tutorial-test: skip
.. literalinclude:: ../examples/controller.sh
   :language: bash
   :start-after: # start backup
   :end-before: # end backup

在 ``local/station.yaml`` 顶层（与 ``robot_ip`` 同级）按所选控制器修改。三个公共字段：

.. list-table::
   :header-rows: 1

   * - 字段
     - 怎么得到这个值
   * - ``cartesian_controller``
     - 第 1 步的选择：``impedance`` ｜ ``internal`` ｜ ``serl``。
   * - ``robot_model``
     - 第 2 步 Desk 的结果：4.x 填 ``panda``，5.x 填 ``fr3``。
   * - ``realtime``
     - 第 3 步检查通过才填 ``enforce``；内核不是 PREEMPT_RT 时 ``enforce`` 会让连接失败。
       ``internal``、``serl`` 应填 ``enforce``；``impedance`` 在普通内核上填 ``ignore``。

也可以让向导改写这几项，它保留 station 里其余的值，并把旧文件备份为 ``station.yaml.bak.<UTC>``：

.. literalinclude:: ../examples/controller.sh
   :language: bash
   :start-after: # start setup-switch
   :end-before: # end setup-switch

向导先问型号，再按型号写 ``joint_limits`` 的默认值；选 ``internal``／``serl`` 时建议 ``realtime: enforce``，
内核不是 PREEMPT_RT 时打印警告；改成 ``serl`` 时去掉 ``control:`` 里 impedance 专用的笛卡尔增益（关节项保留），
已有的 ``serl:`` 参数保留；未设置时使用 SERL 默认值。手写时按下面三小节之一填写。

preset 的 ``real_robot.control`` 可按键覆盖 station 的 ``control``，省略的键沿用 station。
若 station 设置了 ``max_delta_tau``、``translational_error_clip`` 或 ``rotational_error_clip``，
preset 只能保持或收紧这些上限（向量逐元素比较），不能放宽；违反时抛出 ``SafetyViolation``。
刚度、阻尼仍须满足后端支持的字段、维数与数值约束。环境在 reset 的 IDLE 阶段应用配置，
不能用它在运动中切换控制器；``cartesian_controller`` 仍在 station 中选择。
SERL 的笛卡尔参数使用 ``serl``／``serl_profiles``，不使用这里的笛卡尔 tracker 增益。

5A. impedance
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. literalinclude:: ../examples/controller-impedance.yaml
   :language: yaml
   :start-after: # start controller
   :end-before: # end controller

下面的笛卡尔增益写在 ``control:``，只对 ``impedance`` 的笛卡尔 servo 有效。新机械臂用下面这组（``rwrl setup`` 写的就是它）：

.. literalinclude:: ../examples/controller-impedance.yaml
   :language: yaml
   :start-after: # start control
   :end-before: # end control

.. list-table::
   :header-rows: 1

   * - 字段
     - 含义
   * - ``translational_stiffness`` ／ ``rotational_stiffness``
     - 平移（N/m）／旋转（Nm/rad）刚度。franky 自带的 500 N/m 太软，推不动关节静摩擦，会跟不上、走偏。
   * - ``cartesian_damping``
     - 六个阻尼，前三个平移、后三个旋转。
   * - ``translational_error_clip`` ／ ``rotational_error_clip``
     - 误差上限。每轴平移弹簧项的上限为刚度 × 上限：2000 × 0.01 = 20 N；
       不包括阻尼等项，不能当作总接触力的安全上限。

这组值是 SERL 默认阻抗的近似（旋转误差定义不同，只在小角度时相等），不是 SERL 的控制器本身。

5B. internal
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. literalinclude:: ../examples/controller-internal.yaml
   :language: yaml
   :start-after: # start controller
   :end-before: # end controller

``internal`` 的笛卡尔速度接口没有增益可调：误差除以固定的 0.2 s 得到请求速度，
再限速到 0.25 m/s／1.0 rad/s；0.2 s 不是保证到达时间。300 ms 无新目标后减速停下。
``control:`` 里的关节项仍用于关节 servo；``relative_dynamics_factor`` 是 ``move_to_*`` 的速度比例。
``internal`` 还没有在真机上验收过，第一次用时手放在急停上。

5C. serl
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. literalinclude:: ../examples/controller-serl.yaml
   :language: yaml
   :start-after: # start controller
   :end-before: # end controller

``serl:`` 是 SERL 控制器的增益和限幅，键名、默认值、范围和 SERL 的 ``compliance_param.cfg`` 完全一样。
第一次上机写 ``null``：

.. literalinclude:: ../examples/controller-serl.yaml
   :language: yaml
   :start-after: # start serl-null
   :end-before: # end serl-null

需要改个别值时只写要改的键；全部写出来就是下面这样（都是默认值）：

.. literalinclude:: ../examples/controller-serl-params.yaml
   :language: yaml
   :start-after: # start serl
   :end-before: # end serl

.. list-table::
   :header-rows: 1

   * - 字段
     - 含义（范围）
   * - ``translational_stiffness`` / ``translational_damping``
     - 平移刚度 N/m（0–6000）／阻尼（0–400）
   * - ``rotational_stiffness`` / ``rotational_damping``
     - 旋转刚度（0–300）／阻尼（0–30）
   * - ``nullspace_stiffness`` / ``joint1_nullspace_stiffness``
     - 零空间保持起始姿态的刚度（0–100），第 1 关节单独一项
   * - ``translational_clip_{x,y,z}`` / ``translational_clip_neg_{x,y,z}``
     - 每轴平移误差的正／负上限，m（0–0.1）。刚度 × 上限限制该轴弹簧项，默认 2000 × 0.01 = 20 N；不限制总接触力
   * - ``rotational_clip_*``
     - 旋转误差（四元数向量部分）的正／负上限（0–0.1）
   * - ``translational_Ki`` / ``rotational_Ki``
     - 积分增益（0–100），默认 0

超出范围的值会被\ **夹到范围内**\ 并记录警告（和 SERL 的 dynamic_reconfigure 一样）；写错键名或写非数字直接报错。
``control:`` 里 impedance 的笛卡尔增益（``translational_stiffness``、``rotational_stiffness``、``cartesian_damping``、
``translational_error_clip``、``rotational_error_clip``）在 ``serl`` 下无效，doctor 会报 ``config:serl_control`` WARN；
删掉它们，关节项留着。

**自动恢复。** 推到障碍物、碰撞阈值等会触发机器人反射（reflex），SERL 控制器随之停下。
HIL-SERL 在每个 ``/pose`` 前调用 ``/clearerr``，所以操作员感觉不到中断。
``serl_auto_recover`` 做同样的事：下一个 servo 目标到来时清错误，在实测位置\ **新建**\ 一个 SERL 控制器，再执行这个目标：

.. literalinclude:: ../examples/controller-serl.yaml
   :language: yaml
   :start-after: # start recover
   :end-before: # end recover

``enabled`` 默认 ``true``，想每次反射都停下来人工确认时填 ``false``；``max_per_episode`` 是每个回合最多自动恢复几次（默认 5，0 等于关闭）。
超过次数后 servo 报可恢复错误，需要 ``rwrl recover``。新建的控制器刚度和阻尼从 0 爬升约 0.2 s，开头一瞬间会软一点，这是 SERL 每次重启控制器的原样行为。

**负载**\ （三种控制器都生效）。``load`` 在每次连接后、任何运动前发给机器人（libfranka 的 ``setLoad``）：

.. literalinclude:: ../examples/controller-serl.yaml
   :language: yaml
   :start-after: # start load
   :end-before: # end load

只在手爪上装了 Desk 没登记的东西时才写，格式如下；机器人拒绝时 ``connect`` 失败：

.. literalinclude:: ../examples/controller-serl-params.yaml
   :language: yaml
   :start-after: # start load
   :end-before: # end load

**你应该看到**：station 能加载（下一步的 doctor 会检查），三个公共字段与你的记录一致。

**如果不对**：

* ``cartesian_controller: serl needs arm_backend franky``：真实控制使用 ``arm_backend: franky``；配置也允许 ``fake`` 做模拟检查。
* ``robot_model 'x' invalid``：只能是 ``panda`` 或 ``fr3``。
* 已有的 ``joint_limits`` 不会因为改了 ``robot_model`` 自动变化。是新 station 时，删掉这一行再用 ``rwrl setup --station local/station.yaml`` 重写，向导会按型号给默认值。

6. 让 doctor 检查
--------------------------------------------------------------------------------

在 Desk 取得控制权、激活 FCI 后运行：

.. literalinclude:: ../examples/controller.sh
   :language: bash
   :start-after: # start doctor
   :end-before: # end doctor

**你应该看到**：除了 :doc:`station` 里的那些行，还有下面几行，且都是 PASS。
这是按源码格式写的示例，现场以自己的报告为准：

.. literalinclude:: ../examples/controller-output.txt
   :language: text
   :start-after: # start doctor
   :end-before: # end doctor

.. list-table::
   :header-rows: 1

   * - 行
     - 检查什么
   * - ``sdk:serl_controller`` （serl）
     - franky 和 serl_controller 是否来自同一次构建：不导入扩展，比较 ``build_info.json`` 与已装 franky 的 ``libfranky.so`` 指纹和 libfranka
   * - ``config:serl_control`` （serl）
     - ``control:`` 里有没有 serl 模式会忽略的笛卡尔增益
   * - ``mode:serl_realtime`` ／ ``mode:internal_realtime``
     - 用 ``serl`` ／ ``internal`` 时是否 ``realtime: enforce``
   * - ``mode:realtime_kernel``
     - 内核是否 PREEMPT_RT；``enforce`` 且非实时内核时报 FAIL，``ignore`` 时仍报告内核状态
   * - ``mode:robot_model``
     - station 的 ``robot_model`` 和机器人自报的型号（机器人提供的 URDF 里的关节名）是否一致

**如果不对**：

.. literalinclude:: ../examples/controller-output.txt
   :language: text
   :start-after: # start doctor-warn
   :end-before: # end doctor-warn

* ``sdk:serl_controller`` FAIL：不是同一次构建，回到第 4B 步重跑脚本。
* ``mode:serl_realtime``／``mode:internal_realtime`` WARN：回到第 3 步；临时测试可以先忽略，但要预期偶发的通信反射。
* ``mode:robot_model`` WARN：回到第 2 步在 Desk 确认，再改 ``robot_model``。
* ``handshake`` 报 libfranka mismatch：第 2 步的 server version 查错了；``serl`` 时按正确版本重跑 4B，其他控制器用 4A。

7. 第一次运动：归位
--------------------------------------------------------------------------------

先只做关节运动。``rwrl home go`` 把机械臂移到 station 的 ``home_joints``。
``impedance``／``internal`` 用 Ruckig 轨迹；``serl`` 用 SERL 的关节复位：10 s 线性插值到 home，比 Ruckig 慢，路径也不同。

.. warning::

   这一步会移动机械臂。先清空路径，核对 home（:doc:`station` 第 4 步），手放在急停上，保持现场监护。

.. literalinclude:: ../examples/controller.sh
   :language: bash
   :start-after: # start home
   :end-before: # end home

``home show`` 只读，打印 home 和当前关节差；``home go`` 先显示目标和最大关节差，确认后才运动。
它只做 reset 的 home 阶段：阻塞式关节归位，然后执行 preset 中
``task.reset.gripper`` 的 ``open``／``close``；``keep`` 或未设置时不改变夹爪。
它跳过抬升、途经点和稳定等待，不能用这次检查代替完整 reset 验收。
若 preset 有自己的 home，会优先于 station，先看 ``home show`` 的来源。

.. warning::

   home 可能同时开闭夹爪；清空归位路径并固定可能掉落的物体。
   ``home go`` 和 ``gripper open/close/move`` 只在输入 ``y``（不区分大小写）后运动，
   其他回答或输入结束会取消并退出 2。无终端时拒绝运动，提示 ``refusing to move without --yes``，同样退出 2。
   全局 ``--yes`` 放在命令前会跳过询问，只能在现场审阅目标后使用。
   直连时 ``rwrl stop`` 虽不询问，仍需取得 FCI 连接；它不能通过第二条连接停止录制器或 actor 持有的臂。
   先结束持有连接的进程，紧急情况使用物理停止装置。

**你应该看到**：``serl`` 下约 10–12 s 后停在 home，``max delta`` 小于 0.01 rad；其他控制器几秒内到达：

.. literalinclude:: ../examples/controller-output.txt
   :language: text
   :start-after: # start home
   :end-before: # end home

**如果不对**：

* 日志有 ``joint reset TIMEOUT``：约 30 s 内关节没到 home 附近（每关节约 0.01 rad），复位仍会结束；检查是否被挡住、home 是否在限位边上。
* 一启动就报 ``communication_constraints_violation`` 一类反射：回第 3 步确认实时内核。
* ``outside joint_limits``：home 不在 station 的关节限位内，回查第 5 步的型号和 ``joint_limits``。

8. 接下来：第一次笛卡尔运动与反射恢复
--------------------------------------------------------------------------------

笛卡尔运动要有人给目标，所以在 :doc:`spacemouse` 里完成：先按 :doc:`station` 第 11 步确定桌面坐标系，
再按 :doc:`spacemouse` 第 4 步用 tracker 模式沿一个方向轻推约 1 cm。控制器不改变方向；方向错了查坐标系和方向档案。

用 ``serl`` 的，做完那一步后回到这里检查自动恢复：接管时把末端轻轻推向桌面或软物体，直到机器人触发反射。

**你应该看到**：日志里一行 WARNING，下一次推帽机器人从当前位置继续动：

.. literalinclude:: ../examples/controller-output.txt
   :language: text
   :start-after: # start recover-log
   :end-before: # end recover-log

**如果不对**：超过 ``max_per_episode`` 或错误清不掉时，按 Desk 和现场情况处理后运行：

.. literalinclude:: ../examples/controller.sh
   :language: bash
   :start-after: # start recover
   :end-before: # end recover

``rwrl recover`` 成功后，下一个 servo 目标会启动一个新的 SERL 控制器。

任务需要不同的刚度或限幅时，可以在 preset 的 ``real_robot:`` 下写命名档案（完整文件见 ``docs/examples/controller-task.yaml``）。
切换档案是在当前参数上做\ **部分更新**\ ：档案没写的键保持之前的值，和 HIL-SERL 的 ``update_param`` 一样。
环境不会自动切换它们，由任务代码在需要时调用 ``arm.update_serl_params``：

.. literalinclude:: ../examples/controller-task.yaml
   :language: yaml
   :start-after: # start profiles
   :end-before: # end profiles
   :dedent: 2

9. 切换与回滚
--------------------------------------------------------------------------------

切换控制器只改 station（运行 daemon 时重启它）。回到原来的行为，把备份拷回来再检查：

.. tutorial-test: skip
.. literalinclude:: ../examples/controller.sh
   :language: bash
   :lines: 47-47

.. literalinclude:: ../examples/controller.sh
   :language: bash
   :lines: 48-48

**你应该看到**：doctor 的结果回到改动前的样子。

**如果不对**：保存过的实验用另一种控制器录的，恢复（resume）时会因为控制元数据不一致被拒绝，这是有意的；
新开一次实验，不要混用两种控制器的数据。

源码构建的 franky 可以留着：它就是未修改的 v2.0.0，三种控制器都能用。只有想换回官方 wheel 时，
先把 station 改成 ``impedance`` 或 ``internal``，再运行 ``rwrl doctor --install-sdk``；之后 ``serl`` 不再可用，直到重跑 4B。

.. note::

   **本章哪些内容需要真机确认**

   实时内核与抖动、Desk 型号、SERL 关节复位路径、第一次笛卡尔运动的方向和滞后、反射后的自动恢复都要现场确认。
   安装脚本已在干净的 Ubuntu 22.04（Python 3.10、libfranka 0.17.0）上完整跑通，但没有连真机。
   按顺序的上机验收步骤和通过标准见
   :doc:`../reference/serl-robot-validation`；
   深层参考见 :doc:`../reference/serl-controller`。

检查清单：知道选了哪个控制器、为什么；server version 和 libfranka 版本对得上；doctor 相关行都通过；能归位；知道怎么回滚。

接着进入 :doc:`spacemouse`；用 GELLO 的读者看 :doc:`gello`。
