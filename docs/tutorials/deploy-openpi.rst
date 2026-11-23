部署已有 openpi / RLinf 策略
================================================================================

你拿到了一份 π0.5 checkpoint，却不确定它能不能驱动眼前的 Franka。
先把权重和训练任务对上，再让 GPU 服务返回一块动作，最后才连接工作站执行。
本章接续 :doc:`station` 的 ``local/station.yaml``；
``local/task.yaml`` 留作核对已有观测与动作约定，本章入口不会自动读取它。
本仓训练的 BC checkpoint 走 :doc:`deploy`。

.. tip::

   本章设备命令省略了 ``--station``：从仓库根目录运行时，会用 :doc:`station` 第 10 步 ``rwrl use`` 记住的
   station；没记住时也会直接找到 ``local/station.yaml``。要换文件，在命令里显式写 ``--station``。

1. 先认清拿到的 checkpoint
--------------------------------------------------------------------------------

先记录四件事，避免把“能读文件”当成“能在这台机器人上执行”。
``checkpoint`` 是模型保存的权重与配套资产；``loader`` 是把这些文件恢复成模型的代码。
在保存权重的机器上，把路径占位符换成实际目录，再列文件和文本配置：

.. tutorial-test: skip
.. literalinclude:: ../examples/deploy-openpi.sh
   :language: bash
   :start-after: # start inspect
   :end-before: # end inspect

.. list-table:: 四项核对
   :header-rows: 1
   :widths: 22 78

   * - 记录什么
     - 怎样核对
   * - 权重布局
     - 保存目录清单、模型来源和版本。区分模型参数、训练恢复状态、配置和资产；具体文件名、分片与格式需核实，不能只按扩展名判断可加载性。
   * - 加载器
     - 找到该版本训练／导出说明指定的 loader，确认服务端配置注册表包含目标配置。记录源码提交号与启动命令。
   * - 归一化统计
     - 找到训练配置引用的资产目录和统计标识，核对状态与动作字段的顺序、维数和单位。记录统计对应的数据集；文件名、选择规则和缺失时行为需核实。
   * - 训练任务
     - 记录物体、目标、初始布局、语言指令、相机视角、状态和动作契约。Franka 型号相同仍需逐项匹配。

**你应该看到**：一份能追溯到同一训练配置的目录清单、loader、统计和任务说明。
本章不提供虚构的权重目录输出；读者拿到的版本才是核对对象。

**如果不对**：没有文本匹配不等于没有统计，继续检查随模型附带的配置与资产；
只有训练恢复文件时，先找该版本的导出流程；只有模型名时，请权重提供者补齐训练配置。
四项中有空白，就先停在离线核对阶段。

2. 从官方 pi05_base 到 RLinf，再核对 Pick_Red
--------------------------------------------------------------------------------

用官方 ``pi05_base`` 作为转换起点，把“更换权重表示”和“适配现场任务”分别核对。
本仓没有收录它的权重、转换脚本或上游 loader；以下是转换时必须查验的交付链，
不是已经跑通的转换命令。

.. literalinclude:: ../examples/deploy-openpi-output.txt
   :language: text
   :start-after: # start conversion
   :end-before: # end conversion

按顺序完成：

#. 对 ``pi05_base`` 执行上一节的目录检查，确认源 loader 能恢复模型。
   **需核实**：这个版本的原始权重布局、下载来源和精确 loader。
#. 在使用的 RLinf checkout 中找到它针对该源格式的转换入口，核对输入、输出和依赖版本。
   **需核实**：转换脚本路径、命令参数、参数映射和目标格式。本仓没有可直接复用的转换命令。
#. 用目标 loader 重新加载转换产物，比较同一固定输入的输出，并核对归一化资产是否需要单独复制。
   **需核实**：数值容差、统计资产处理方式和模型变换。转换成功本身不证明学会了 Pick_Red。
#. 若要通过 openpi WebSocket 服务部署，再核对 RLinf 产物能否由该 checkout 的
   ``serve_policy.py`` 加载，或还需要导出。**需核实**：这一步的导出／服务加载兼容性。

本仓 :doc:`../reference/deploy-openpi` 与中文版支持的服务示例是
``pi05_franka_pnp``，要求服务端注册 ``LeRobotRealworldDataConfig``、
``RealworldInputs`` 和 ``RealworldOutputs``。本仓 ``profiles.py`` 明确把
``rlinf_franka`` 对接到这一类输入；使用 ``FrankaEEInputs`` 的配置是另一种契约。
这并没有证明任意名为 π0.5 的模型都适合该 profile。
本仓客户端只组装观测并收发协议，不替你寻找归一化统计文件；
必须在服务端配置核对统计如何用于输入与输出。

现在把 ``RLinf-Pi05-Pick_Red`` 的单臂 Franka 任务配置放在旁边，逐项检查：

* 模型实际选用哪一个配置、loader 和统计资产；是否确实使用上述 ``RealworldInputs`` 契约。
* 是否为单臂七维笛卡尔动作；前六维的坐标系、增量旋转和尺度；末维的开闭阈值。
* 是否需要两幅图像，分别来自哪里；裁切、颜色顺序、尺寸和语言指令是否一致。
* reset 的参考位姿、夹爪行程、控制频率、物体和目标位置是否与现场一致。

**需核实**：提纲记载已找到 Pick_Red 上游真机配置，但本仓没有其配置正文或固定版本；
不能据此断言它就是 ``pi05_franka_pnp``，也不能把下文示例 prompt 当成它的训练指令。
请把上述每项的配置值与版本保存下来。

**独立验证项：本仓通过 WebSocket 加载 RLinf-Pi05-Pick_Red 并完成推理尚未验证。**
要关闭这一项，必须用实际权重、配套 loader 和统计启动服务，完成本章 dry-run，
再用现场观测检查输入输出契约。测试中的假策略不算这项验证。

**你应该看到**：源权重、转换产物、服务可加载产物各有明确 loader；
Pick_Red 的配置能逐项对应第 7 节的契约。

**如果不对**：转换后不能加载，先查源／目标版本与参数映射；缺统计时补训练资产；
输入使用 ``FrankaEEInputs`` 时停止套用 ``rlinf_franka``，需要另行实现并验证适配。
本章只采用本仓文档与代码，不据未收录的上游细节推导可执行命令。
深层参考见 :doc:`../reference/deploy-openpi`。

3. 在 GPU 上启动策略服务
--------------------------------------------------------------------------------

让 GPU 负责推理，让 NUC 留在机器人旁负责设备和限制，先把两条连接分清。
``daemon`` 是 NUC 的设备服务；``profile`` 是本仓把设备观测整理成策略输入的规则。

.. literalinclude:: ../examples/deploy-openpi-output.txt
   :language: text
   :start-after: # start topology
   :end-before: # end topology

GPU 上需要两个环境：openpi 环境加载权重，rwrl 环境执行部署循环。
在 GPU 的 rwrl 环境、仓库根目录安装客户端依赖，并查看本章用到的帮助：

.. tutorial-test: skip
.. literalinclude:: ../examples/deploy-openpi.sh
   :language: bash
   :start-after: # start install
   :end-before: # end install

.. literalinclude:: ../examples/deploy-openpi.sh
   :language: bash
   :start-after: # start help
   :end-before: # end help

``jpeg`` 提供图像解码和缩放依赖；NUC 的设备依赖沿用 :doc:`install`。
下面是本地实际运行的 deploy 帮助节选，完整的
:download:`deploy 帮助 <../examples/deploy-openpi-help.txt>`、
:download:`daemon 帮助 <../examples/deploy-openpi-daemon-help.txt>` 和
:download:`doctor 帮助 <../examples/deploy-openpi-doctor-help.txt>` 可供对照：

.. literalinclude:: ../examples/deploy-openpi-help.txt
   :language: text
   :lines: 10-19

在 GPU 的 openpi checkout 及其环境中，用已确认的导出目录启动服务：

.. tutorial-test: skip
.. literalinclude:: ../examples/deploy-openpi.sh
   :language: bash
   :start-after: # start serve
   :end-before: # end serve

这条命令来自本仓部署文档。**需核实**：当前上游 checkout 的命令行及启动日志；
本仓没有 ``scripts/serve_policy.py``，本次无法运行它的帮助或实际加载权重。
文档说明官方服务默认监听所有接口且无认证；这一上游版本行为也需核实，
现场把 8000 端口的访问范围限制在需要的客户端。

**你应该看到**：服务进程完成加载并持续运行；第 6 节的 ``link.policy`` 能读取元数据。
启动日志因上游版本而异，这里不编造成功输出。

**如果不对**：配置名不存在，使用包含 RLinf 配置注册的 checkout；
权重或统计加载失败，回到第 1、2 节核对；端口不一致，同时修改服务端端口与
客户端 ``--policy-port``。``--api-key`` 只向支持它的网关／服务发送认证头，不能替服务端开启认证。

4. 在 NUC 上启动 daemon
--------------------------------------------------------------------------------

把监听地址写进已有 station，让设备检查、服务和 SSH 转发使用同一个端口。
在 :doc:`station` 已填写并验证的 ``local/station.yaml`` 顶层加入下面的块，
与 ``type``、``cameras`` 同级；不要用本章模板覆盖已经测好的 home 和 ``limits``。

.. literalinclude:: ../examples/deploy-openpi-station.yaml
   :language: yaml
   :start-after: # start daemon
   :end-before: # end daemon

.. list-table::
   :header-rows: 1

   * - 必须改的字段
     - 怎么得到这个值
   * - ``daemon.bind``
     - 本章选 NUC 回环地址 ``127.0.0.1:50051``。若端口已占用，按 ``link.daemon.listen`` 查明占用者，选定空闲端口后同步改隧道目标和 doctor 的 ``--remote-bind``。

其余字段保留未设置：``daemon.token`` 默认无令牌；需要令牌时在 station 设置它，
并在 doctor 的 ``--token`` 与部署的 ``--real-robot.token`` 传同一值。
``--watchdog-s`` 是 daemon 命令行参数，不写进这个 YAML 块；CLI 默认 0.5 秒。
下面的 2 秒来自部署文档示例，必须按第 9 节的实测间隔审阅。
不写 ``daemon.bind`` 时默认是 ``0.0.0.0:50051``，因此这里显式填写回环地址。

在 NUC 已激活的设备环境中运行：

.. literalinclude:: ../examples/deploy-openpi.sh
   :language: bash
   :start-after: # start daemon
   :end-before: # end daemon

**你应该看到**：先输出设备 preflight 检查，再出现以下服务行。这是按源码格式写的示例：

.. literalinclude:: ../examples/deploy-openpi-output.txt
   :language: text
   :start-after: # start daemon
   :end-before: # end daemon

**如果不对**：``preflight failed`` 时按 :doc:`station` 修复握手或相机反馈；
``link.daemon.listen`` 无监听时检查进程是否退出、配置路径和 bind；
设备已占用时先结束原控制程序，再启动服务。

5. 从 GPU 打开 SSH 隧道
--------------------------------------------------------------------------------

把 GPU 的本地 50061 转发到 NUC 的本地 50051，让部署循环通过隧道使用 gRPC。
先确认现场的 NUC SSH 用户、主机名、主机指纹和登录密钥，再在 GPU 的独立终端运行：

.. tutorial-test: skip
.. literalinclude:: ../examples/deploy-openpi.sh
   :language: bash
   :start-after: # start tunnel
   :end-before: # end tunnel

**你应该看到**：命令保持运行，通常没有输出；接下来 doctor 的 ``link.daemon`` 应通过。
两个 ``127.0.0.1`` 分别属于 GPU 与 NUC，策略端口 8000 不经过这条转发。

**如果不对**：本地端口占用时检查已有隧道；认证失败时核对 SSH 用户和密钥；
主机密钥变化时先核验 NUC 指纹；``link.forward`` 报禁止转发时在 NUC 检查
``link.sshd``，包括 ``AllowTcpForwarding``、``DisableForwarding``、``PermitOpen``
及用户专属规则。

6. 在两端读 doctor，再做 dry-run
--------------------------------------------------------------------------------

先确认设备链路和策略握手，再发送一次合成观测，把网络问题与模型输入问题分开定位。
在 NUC 上运行：

.. literalinclude:: ../examples/deploy-openpi.sh
   :language: bash
   :start-after: # start doctor-nuc
   :end-before: # end doctor-nuc

在 GPU 的 rwrl 环境中运行：

.. literalinclude:: ../examples/deploy-openpi.sh
   :language: bash
   :start-after: # start doctor-gpu
   :end-before: # end doctor-gpu

``--link`` 是当前机器看到的 daemon 地址。GPU 的 ``--ssh`` 会真的登录 NUC 并测试临时转发。
本章验证未执行远程登录，只用下面的本地空端口命令捕获失败行：

.. literalinclude:: ../examples/deploy-openpi.sh
   :language: bash
   :start-after: # start doctor-empty
   :end-before: # end doctor-empty

.. literalinclude:: ../examples/deploy-openpi-doctor-gpu.txt
   :language: text
   :start-after: # start output
   :end-before: # end output

下面是同一开发机器模拟 NUC 角色的真实输出节选：使用临时的 fake station，
声明回环 ``daemon.bind``，运行 NUC 形式的 ``--link`` 命令，不启动服务、也不使用
``--mock``。``--mock`` 会跳过链路检查，不能用于捕获这些行。

.. literalinclude:: ../examples/deploy-openpi-doctor-nuc.txt
   :language: text
   :start-after: # start output
   :end-before: # end output

.. list-table:: 按行读取结果
   :header-rows: 1
   :widths: 30 70

   * - 行名
     - 通过说明什么；失败后做什么
   * - ``link.daemon.bind`` / ``link.daemon.listen``
     - NUC 配置和监听进程。无监听则先修 daemon；bind 通过不代表服务已启动。
   * - ``link.sshd`` / ``link.firewall``
     - NUC SSH 转发权限和入站规则。回环地址的 firewall 跳过是预期，不需要开放 50051 到局域网。
   * - ``link.ssh`` / ``link.tunnel`` / ``link.forward``
     - GPU 登录、当前隧道与独立临时转发。临时转发通过但当前链路失败，检查本地转发端口和服务状态。
   * - ``link.daemon`` / ``link.deps``
     - 已取得设备描述；通信依赖可导入。令牌错误按 ``UNAUTHENTICATED (token mismatch)`` 的 hint 对齐两端令牌。
   * - ``link.preprocess``
     - NUC 能否使用 OpenCV 预处理。不可用时可能传全尺寸图像，由 GPU 处理，需重测带宽。
   * - ``link.quality``
     - 10 次 describe 的 RTT 和 3 次观测传输；post-reset gap 是三倍 p95，尚未包含策略推理。先修前面的 FAIL 再读时序。
   * - ``link.policy``
     - 仅 WebSocket 握手和元数据，不发送推理。失败时核对服务端地址、代理和服务日志。
   * - ``link.proxy``
     - 代理绕过和 SSH fake-IP 风险；处理方法见第 9 节。

NUC 的直接回环检查没有配置 SSH，``link.ssh``、``link.forward`` 会跳过；
没有隧道服务时 ``link.tunnel`` 的提示也不表示 NUC 必须再建一条隧道。
重点读服务端监听行与 ``link.daemon``；隧道检查在 GPU 做。
完整行定义见 :doc:`../reference/doctor` 的链路检查说明。
``link.quality`` 可能连接空闲设备来采样观测，检查完会断开它建立的连接。

然后在 GPU 执行一次合成输入推理：

.. literalinclude:: ../examples/deploy-openpi.sh
   :language: bash
   :start-after: # start dry-run
   :end-before: # end dry-run

**你应该看到**：``dry-run actions=(动作块长度, 7)``，以及客户端往返和服务端耗时。
下面是使用 ``tests/test_deploy_openpi.py`` 的假策略和协议 fixture 实际运行的输出，
其中动作块长度 3、服务端耗时 3.0 ms 都来自 fake；它没有加载 π0.5：

.. literalinclude:: ../examples/deploy-openpi-dry-run.txt
   :language: text
   :start-after: # start output
   :end-before: # end output

dry-run 用零状态和黑图像，只需要策略服务，不创建机器人环境。
动作必须是有限数的二维数组 ``(horizon, action_dim)``；通过不说明尺度和任务匹配。
``--policy-timeout-s`` 默认 30 秒，用于策略连接与接收；它不同于机器人重连的
``--reconnect-timeout-s``。策略超时会退出，不进入机器人 WAITING 流程。

**如果不对**：``could not connect to openpi`` 对照 ``link.policy`` 修复地址和代理；
``openpi server error during inference`` 查服务端输入键、统计和模型日志；
``expected finite actions shaped (horizon, 7)`` 核对输出布局，不能截取维度凑数。

7. 对齐 station、profile 和动作映射
--------------------------------------------------------------------------------

在运动前，把策略每个输入和输出对应到真实设备，尤其不要沿用上一章的单相机与动作尺度。
在 NUC 读取已经启动的 daemon：

.. literalinclude:: ../examples/deploy-openpi.sh
   :language: bash
   :start-after: # start mapping
   :end-before: # end mapping

**你应该看到**：实际相机名称、home、``limits``（关节范围、TCP 盒和单步限制）。
:doc:`station` 的 ``wrist`` 只是一台相机；本章服务示例默认需要
``wrist_1`` 和 ``wrist_2``。只有一台时先核对模型是否允许缺第二视角，
不能把同一帧复制成两台相机来宣称匹配。
新增相机须按工作站章发现序列号、验证画面，再填写 station。
``--camera-keys`` 中左侧填 daemon 的实际相机名，右侧填训练配置的策略键。
例如 ``status`` 若列出 ``wrist`` 和 ``side``，左侧就应改为这两个名称。

.. list-table:: ``rlinf_franka`` 的本仓契约
   :header-rows: 1
   :widths: 26 74

   * - 输入／输出
     - 顺序、单位和处理
   * - ``observation/state``：19 维
     - 夹爪宽度 1（m）、TCP 力 3、TCP 位姿 6（相对 reset 的 XYZ 与 xyz 欧拉角 rad）、TCP 力矩 3、TCP 速度 6（逆伴随变换到末端系）。力和力矩从基座旋转到刚度系。
   * - 两幅图像
     - 默认 ``wrist_1 → observation/image``、``wrist_2 → observation/extra_view_image``。中心方形裁切、OpenCV 线性缩放至 128 × 128，HWC uint8 BGR；裁切可用 ``--crop-regions`` 指定比例边界。
   * - ``prompt``
     - 由命令行语言指令填入；必须核对训练任务。文档所述模型端再缩放到 224 × 224 的上游实现需核实。
   * - 动作 0–5
     - 七维动作中的前六维经当前位姿伴随变换、裁剪到 [-1, 1]，应用平移／旋转尺度，再把欧拉增量左乘实测姿态；这与工作站章的 ``current`` 语义不同。
   * - 动作 6
     - 乘 ``gripper_action_scale`` 后，达到 +0.5 打开、达到 −0.5 关闭，中间区间保持；还会检查实测开闭状态。阈值可用 ``--real-robot.rlinf-gripper-threshold`` 调整，Hand 使用二值模式。

``--stiffness-quat`` 默认单位四元数，假定刚度系与 TCP 轴对齐；现场有标定旋转时填
相对 TCP 的 xyzw 四元数。保持训练时的裁切、颜色与相机顺序。
默认平移／旋转尺度是 0.02 m／0.1 rad；部署文档的 RLinf SFT 示例显式使用 1.0／1.0。
**这些是示例，不是已确认的 Pick_Red 参数。** 应以已核实的训练配置为准，
同时保留 station 的 ``limits``（含单步限制）和安全 home。

``generic`` 则按 ``--real-robot.state-fields`` 拼接原生单位的状态，默认 224 × 224、
Pillow 双线性缩放、RGB；动作按环境配置解释。改 profile 会改变契约，不是修复维数错误的快捷方式。

**如果不对**：``real_robot.cameras must match the camera_keys mapping`` 时对齐两份名称列表；
``rlinf_franka requires a camera mapped to observation/image`` 时修主视角映射；
缺力／力矩反馈或 reset 参考不同，先补设备能力并核对训练定义；
方向、尺度或夹爪语义不一致，就停在本节，不执行下一步。

8. 运行一个受监护回合并读执行记录
--------------------------------------------------------------------------------

把已核对的配置用于一次短回合，同时保存终端记录，以便把实测运动和报告对应起来。
以下参数沿用本仓部署文档示例：10 Hz、30 步上限、每次推理最多执行 5 步；
``120`` 秒是本章为断链等待选定的示例上限，并非程序默认值。
先将相机名、prompt、尺度、频率改成第 2、7 节已经核实的值。

.. warning::

   下一条命令会 reset、归位、开爪并执行策略动作。先确认 home 路径、负载、工作区、
   单步限制和停止方式，保持现场监护与物理停止装置可用。
   dry-run 通过和 daemon 在线都不能代替这些检查。

.. tutorial-test: skip
.. literalinclude:: ../examples/deploy-openpi.sh
   :language: bash
   :start-after: # start episode
   :end-before: # end episode

**你应该看到**：先有一次合成 ``warmup``，再 reset，使用 reset 后的真实观测请求第一块动作。
每块执行至多 ``--replan-steps`` 步，默认不指定时执行整个 horizon。
下面是按 ``loop.py`` 格式写的回合输出示例，数值不代表硬件测试：

.. literalinclude:: ../examples/deploy-openpi-output.txt
   :language: text
   :start-after: # start episode
   :end-before: # end episode

读 ``steps``、``health_ok``、``outcome``，再对照 ``rtt_ms_mean``、
``inference_ms_mean/max`` 和 ``server_infer_ms_mean``。没有服务端计时会显示 ``n/a``。
达到步数上限是截断，不是成功；没有任务成功判定时不能把完成一个回合当作抓取成功。
部署循环打印的 ``success_rate`` 来自环境 outcome，不识别视频里的任务结果。

需要提前结束时，在部署终端按 Ctrl-C；循环的清理路径会关闭环境并断开设备。
现场仍须确认机械臂实际停止，异常运动时用物理停止装置。
该入口没有本章前面遥操作章节的接管按钮流程。
随后读取刚才保存的记录：

.. tutorial-test: skip
.. literalinclude:: ../examples/deploy-openpi.sh
   :language: bash
   :start-after: # start log
   :end-before: # end log

``local/deploy-openpi.log`` 保存摘要、时序和断链事件，不是逐步动作／观测轨迹。
若进入真机强化学习时需要逐步比对策略动作与执行动作，须另行安排轨迹采集；
本章命令没有生成这样的归档。不要把摘要当作 :doc:`inspect` 中的完整录制结果。

**如果不对**：首次 reset 失败时回查 home、安全边界和设备反馈；反复出现 watchdog
``interrupted`` 时按下一节测动作间隔；``health_ok=False`` 时停止并检查 daemon 日志；
动作与预期不符时保存记录，回到第 7 节，不靠增大边界继续试。

9. 断链、等待和恢复
--------------------------------------------------------------------------------

先认识等待状态，再维护隧道，避免恢复连接后误以为还在执行原来的动作块。
``WAITING`` 表示部署循环在等机器人链路或操作员，不是策略正在继续完成任务。
它在回合内遇到可恢复的 transport 错误，或可恢复的 daemon watchdog 故障时出现。
策略 WebSocket 错误、安全和不可恢复配置错误会退出，不进入这条机器人重连流程。
首次创建环境就连接失败也不保证进入 WAITING。

当隧道断开，NUC watchdog 会在超时后让伺服中的机械臂停下并进入 IDLE。
循环丢弃当前动作块和预取结果，保留策略连接；按 1、2、4、8、10 秒上限的退避重试。
重连会重读设备描述、按 daemon 身份协商能力、检查机械臂健康并处理报告的可恢复错误。
源码格式示例如下：

.. literalinclude:: ../examples/deploy-openpi-output.txt
   :language: text
   :start-after: # start waiting
   :end-before: # end waiting

**你应该看到**：重连后先提示确认。现场检查完再按 Enter，程序会 reset 并开始新回合，
不会从旧动作块中间接着执行。被打断的尝试计入 ``interrupted_episodes``，
不计入完成回合数和成功率分母；``disconnects`` 与 ``disconnected_s`` 保留断链记录。
中断时长从该回合最后一次成功机器人交互计起；尚无成功交互时从错误出现计起。

无交互 stdin 时会打印 ``WAITING for operator confirmation (no interactive stdin)`` 并等待。
``--auto-resume`` 跳过确认，允许重连后自动 reset；只在现场明确允许自动复位时启用。
建议在 GPU 的可重新附着终端（如 tmux）运行部署，保持人工确认入口。

``--reconnect-timeout-s`` 默认不设限；第 8 节设为 120 秒。
它从进入 WAITING 起计算，约束重试及无交互 stdin 的等待，超时走设备错误退出码 2。
**实现边界**：交互式 ``input()`` 等 Enter 时不会被这个期限打断；
已发出的单次设备调用也不由它强制中止。因此它不是精确的墙钟总时限。

让隧道独立于终端存活
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

先在 GPU 安装 autossh 并配置 SSH 密钥认证，再把服务放进用户 systemd。
下面内容由本仓 ``link_host.tunnel_service`` 生成，端口与本章手动隧道一致：

.. literalinclude:: ../examples/deploy-openpi.service
   :language: ini
   :start-after: # start tunnel-service
   :end-before: # end tunnel-service

.. tutorial-test: skip
.. literalinclude:: ../examples/deploy-openpi.sh
   :language: bash
   :start-after: # start service
   :end-before: # end service

把 ``command -v autossh`` 的路径填入 ``ExecStart``；例如输出为 ``/usr/bin/autossh``
时正好对应上面的路径。用户名、NUC 地址沿用第 5 节；先结束手动隧道再启动服务，
避免争用 50061。``enable-linger`` 用于退出登录后继续运行，按主机权限完成。

**你应该看到**：用户服务为 active，GPU doctor 的 ``link.daemon`` 重新通过。
CLI-only doctor 使用 ``rwrl-tunnel-50061.service``；若通过 station 的 ``link:``
配置，服务名会按机器人名生成。使用 doctor 的 ``--fix`` 前先读它给出的服务名和计划。

**如果不对**：``link.tunnel`` 显示 systemd 状态不可用时在 GPU 主机的用户会话中检查；
服务反复重启时查看 SSH 密钥、主机指纹和端口占用；``link.forward`` 失败时修 NUC 转发配置。
autossh 只恢复隧道，不替部署循环确认 reset，也不替代 watchdog。

给 watchdog 留出实测时间
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

先读第 8 节日志和 ``link.quality``，再决定 NUC 的 ``--watchdog-s``，不要照抄 2 秒。
每次 reset 后，首次动作前约有三次 daemon 往返，再加策略推理；
还要算入图像传输、隧道抖动、控制周期和实际设备调用耗时。
当前 station 构建的 franky Hand 后端异步提交开合；不要再按旧的阻塞式 Hand
实现估计耗时，应测量当前后端的实际动作间隔。
例如本仓文档讨论的 400 ms RTT，三次往返已约 1.2 秒，1 秒超时会反复打断刚恢复的回合。

**你应该看到**：选定值覆盖现场最长正常动作间隔并留余量，且断链停止延迟仍可接受。
``link.quality`` 的估算不含推理，必须补上真实服务测量。
如果另外启用环境的 ``real_robot.watchdog_s``，也要按同样预算核对。

**如果不对**：一直 ``interrupted`` 时先降低传输负担、修代理路径并核对控制频率；
不要无限增大超时。``--prefetch-steps`` 默认 0；启用可重叠推理，但下一块动作依据的是较旧观测，
需另行评估。本章先保持关闭。

处理 Clash / mihomo fake-IP
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

如果 ``link.proxy`` 报告代理未绕过回环地址，先按实际 hint 设置环境再重跑 doctor：

.. tutorial-test: skip
.. literalinclude:: ../examples/deploy-openpi.sh
   :language: bash
   :start-after: # start proxy
   :end-before: # end proxy

若报告 SSH 主机解析到 ``198.18.0.0/15``，doctor 会给出两条路线：

* 在 NUC 查询真实地址，再在 GPU 的 SSH 配置为该目标设置 ``HostName``，
  同时按 hint 保留 ``HostKeyAlias``，避免失去原 known_hosts 对应关系。
* 在 Clash/mihomo 配置的 ``dns.fake-ip-filter`` 加入主机名，并把该域名的 DIRECT 规则放在前面；
  或用 ``tun.route-exclude-address`` 排除 NUC 的真实地址。

.. tutorial-test: skip
.. literalinclude:: ../examples/deploy-openpi.sh
   :language: bash
   :start-after: # start real-ip
   :end-before: # end real-ip

**你应该看到**：查到的真实地址经现场确认后，新的 SSH 路径不再经过 fake-IP；
重测 ``link.quality`` 的 RTT 和图像吞吐量。仅设置 ``no_proxy`` 不会修正 SSH 的 DNS 路径。

**如果不对**：``Host key verification failed`` 时核验指纹和 ``HostKeyAlias``；
DNS 查询仍返回假地址时先修本地解析；代理警告消失但 RTT 仍高时继续检查链路与图像大小。
这些是本仓 doctor 的诊断建议，本章未在真实 Clash/NUC 网络上验证效果。

扩展阅读：双臂 Clean Desk
--------------------------------------------------------------------------------

把双臂 ``Clean Desk`` 当作契约扩展来读：同为 Franka，也多了左右臂的布局和同步问题。
用第 1 节的检查方法列出它的训练配置，另外核对左右状态的拼接顺序、各自的 reset 参考系、
各视角对应哪只手臂、两侧夹爪编码，以及一个动作块怎样拆分到两台设备。
还要确认双臂时序、两边的 home／边界和跨臂碰撞空间。

**你应该看到**：一份明确到“哪一维属于哪只臂、哪幅图来自哪里”的输入输出表。
**需核实**：Clean Desk 的上游权重、loader、归一化统计、确切维数和相机布局。
本地未收录这些配置，不将单臂 19／7 维简单翻倍当作已确认契约。

**如果不对**：本章 ``deploy-openpi`` 构造的是单个 ``RealRobotEnv``，
现有两个 profile 都不能直接宣称支持这份双臂策略；需要单独实现并验证双臂适配。
不要把双臂动作塞进当前单臂入口。

.. note::

   **本章哪些内容需要真机确认**

   实际 Pick_Red 权重的 WebSocket 加载与推理尚未验证；还要确认真实相机的视角、
   裁切和颜色，状态／力矩与刚度坐标系，动作方向、尺度与夹爪阈值，home 与 ``limits``，
   推理和传输时序、watchdog 停止效果、断链后的人工确认及 reset。
   autossh、systemd 和 Clash 路由需要在两台现场主机验证；Clean Desk 还需双臂契约和同步验证。
   本章本地结果只覆盖 CLI、假策略协议和无服务时的诊断，不能作为真机验收。

检查清单：进入真机强化学习前
--------------------------------------------------------------------------------

* 已验证 actor 所用的观测与动作配置；本章使用的 profile、相机、尺度与 station 已逐项核对。
* 知道策略如何从观测产生动作块，以及每次执行多少步；实际 Pick_Red WebSocket 推理验证项已有结果。
* 知道如何停止当前回合，能读取执行摘要、断链记录，并知道摘要不含逐步动作轨迹。
* 真机强化学习（从 :doc:`rl-overview` 开始）复用这套工作站和已验证配置；HIL-SERL 的策略初始化方式在那几章单独说明。

下一章进入 :doc:`rl-overview`。
