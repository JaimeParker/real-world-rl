安装与初验
================================================================================

1. 你是哪种情况？
--------------------------------------------------------------------------------

先选程序要运行的位置，这样只装这台机器需要的依赖。

``extra`` 是安装时可选的一组依赖，写在 ``--extras`` 后面。
``daemon`` 是留在 NUC 上的设备服务，其他机器通过它读状态、发动作。
下面以 Franka Hand、RealSense 和 SpaceMouse 为例。

在选定机器上取得源码，进入仓库根目录；已经检出源码的读者直接进入该目录。

.. tutorial-test: skip
.. literalinclude:: ../examples/install.sh
   :language: bash
   :start-after: # start checkout
   :end-before: # end checkout

在这台机器上，从下面四种情况中选一条安装命令。

* **NUC 直接录制、推理或运行 actor**：在 NUC 执行，SpaceMouse 也插在这里。

  .. tutorial-test: skip
  .. literalinclude:: ../examples/install.sh
     :language: bash
     :start-after: # start direct
     :end-before: # end direct

* **NUC 只提供设备服务**：在 NUC 执行，录制或推理程序放在另一台机器。

  .. tutorial-test: skip
  .. literalinclude:: ../examples/install.sh
     :language: bash
     :start-after: # start daemon
     :end-before: # end daemon

* **另一台机器通过 NUC 录制或推理**：在客户端执行，SpaceMouse 插在客户端。

  .. tutorial-test: skip
  .. literalinclude:: ../examples/install.sh
     :language: bash
     :start-after: # start client
     :end-before: # end client

* **GPU 服务器只运行 learner 或离线 BC**：在 GPU 服务器执行。

  .. tutorial-test: skip
  .. literalinclude:: ../examples/install.sh
     :language: bash
     :start-after: # start learner
     :end-before: # end learner

设备 SDK 装在实际打开设备的机器上。前两种情况的 NUC 需要 ``franky``；
后两种情况不需要。``franky`` extra 安装的包名是 ``franky-control``。
它是 PyPI 上的官方 franky；以后选用 ``serl`` 控制器时，franky 会和 ``serl_controller`` 一起在 NUC 上从源码重新构建，
见 :doc:`controller` 第 4B 步（这里照常安装即可，构建脚本会先卸载它）。

.. code-block:: text

   Franka + Hand <--> NUC <--- 观测 / 动作 ---> 远程录制或推理客户端
   RealSense ------> NUC
                     |
                     +--- 样本 / 新参数 ---> GPU learner
                     +--- 观测 / 策略动作 <-> GPU 策略服务

   直接录制时，录制程序就在 NUC 上，不经过 daemon。

**你应该看到**：安装结束打印 ``Installation complete``，并提示激活 ``.venv``。

**如果不对**：

* 找不到 ``python3.10``：给安装命令加 ``--python``，指定已安装的 Python 3.10 或更新版本。
* 找不到项目文件：回到刚检出的仓库根目录，再运行安装命令。
* GPU 只运行 openpi 策略服务：使用该模型服务自己的安装流程；这里的 ``learning`` 用于本仓训练程序。

2. 激活安装好的环境
--------------------------------------------------------------------------------

在当前终端激活刚安装的环境，让后续命令使用这份依赖。

.. tutorial-test: skip
.. literalinclude:: ../examples/install.sh
   :language: bash
   :start-after: # start activate
   :end-before: # end activate

未指定 ``--extras`` 或 ``--daemon-only`` 时，\ ``scripts/install.sh`` 默认安装
``learning,franka-gello,shell``\ 。\ ``shell`` extra 安装交互式终端 ``rwrl shell`` 所需的依赖。

脚本默认用 ``python3.10`` 创建 ``.venv``。选择 ``learning`` 时，它还会安装
rl-garden 子模块和 ``test`` extra。``--daemon-only`` 是 ``franky,grpc`` 的快捷方式，
本章设备服务命令额外加了相机需要的 ``realsense``。

还可给安装脚本传入以下选项；完整列表用 ``bash scripts/install.sh --help`` 查看。

.. list-table:: 安装选项
   :header-rows: 1

   * - 选项
     - 用途
   * - ``--venv PATH``
     - 创建或复用指定 venv；默认是 ``.venv``，后续要激活这个路径。
   * - ``--torch-index URL``
     - 先从指定索引安装 torch；必须同时选择 ``learning``。
   * - ``--station YAML``
     - 最后对指定真实 station 运行 doctor；省略时做模拟检查。
   * - ``--no-doctor``
     - 跳过最后的 doctor；安装成功不代表设备检查通过。

如果需要手动安装，下面是本章“NUC 直接录制”的等价依赖步骤。
已有项目 venv 或非 base conda 环境时，跳过创建和激活 venv 两行。
先清除继承的 ``PYTHONPATH``，避免导入另一份源码。其他机器按第 1 步选择 extras；
仅设备服务不安装 ``rl-garden`` 和 ``learning``。

.. tutorial-test: skip
.. literalinclude:: ../examples/install.sh
   :language: bash
   :start-after: # start manual
   :end-before: # end manual

手动安装后仍要做第 4 步的检查。

**你应该看到**：激活通常不打印内容，终端提示符可能出现 ``(.venv)``。
后面会用导入命令确认环境。分机运行时，保存前面两条 Git 命令的提交号，
让各机器使用同一版本。

**如果不对**：

* 子模块目录为空：在仓库根目录运行 ``git submodule update --init``，再重跑安装命令。
* 新终端找不到 ``rwrl``：重新执行上面的激活命令。

3. 按手里的设备补齐依赖
--------------------------------------------------------------------------------

把实际使用设备的 extra 加到已选安装命令中，再在读取该设备的机器检查一次导入。

.. list-table:: 设备依赖
   :header-rows: 1
   :widths: 22 28 50

   * - 设备
     - 加哪个 extra
     - 装好后的一条检查命令
   * - Franka / Franka Hand
     - ``franky``
     - .. literalinclude:: ../examples/install.sh
          :language: bash
          :start-after: # start check-franky
          :end-before: # end check-franky
   * - RealSense
     - ``realsense``
     - .. literalinclude:: ../examples/install.sh
          :language: bash
          :start-after: # start check-realsense
          :end-before: # end check-realsense
   * - ZED UVC 彩色
     - ``zed-uvc`` （OpenCV，无深度）
     - .. literalinclude:: ../examples/install.sh
          :language: bash
          :start-after: # start check-zed-uvc
          :end-before: # end check-zed-uvc
   * - ZED SDK
     - 无 extra；安装厂商 ``pyzed`` 绑定
     - .. literalinclude:: ../examples/install.sh
          :language: bash
          :start-after: # start check-zed
          :end-before: # end check-zed
   * - SpaceMouse
     - ``spacemouse`` （``hidapi==0.14.0.post4``）
     - .. literalinclude:: ../examples/install.sh
          :language: bash
          :start-after: # start check-spacemouse
          :end-before: # end check-spacemouse
   * - GELLO
     - ``gello``
     - .. literalinclude:: ../examples/install.sh
          :language: bash
          :start-after: # start check-gello
          :end-before: # end check-gello
   * - GELLO 键盘／踏板
     - ``gello-events`` （``evdev``）
     - .. literalinclude:: ../examples/install.sh
          :language: bash
          :start-after: # start check-events
          :end-before: # end check-events
   * - Pico VR
     - ``pico`` （``pyzmq``）
     - .. literalinclude:: ../examples/install.sh
          :language: bash
          :start-after: # start check-pico
          :end-before: # end check-pico
   * - 桌面键盘
     - ``reward-collection`` （``pynput``）
     - .. literalinclude:: ../examples/install.sh
          :language: bash
          :start-after: # start check-keyboard
          :end-before: # end check-keyboard
   * - SSH 终端／JSONL 事件
     - 无需 extra；事件源名为 ``stdin``／``jsonl``
     - .. literalinclude:: ../examples/install.sh
          :language: bash
          :start-after: # start check-terminal
          :end-before: # end check-terminal
   * - 实体主臂（通过设备服务读取）
     - 主臂端装 ``franky``、``pyagxarm`` 或 ``piper-sdk``；通信两端加 ``grpc``
     - .. literalinclude:: ../examples/install.sh
          :language: bash
          :start-after: # start check-grpc
          :end-before: # end check-grpc
   * - 远程设备连接
     - ``grpc``，设备服务与客户端两端都装
     - .. literalinclude:: ../examples/install.sh
          :language: bash
          :start-after: # start check-grpc
          :end-before: # end check-grpc
   * - JPEG 图像传输
     - ``jpeg``，两端都装；原始传输不用
     - .. literalinclude:: ../examples/install.sh
          :language: bash
          :start-after: # start check-jpeg
          :end-before: # end check-jpeg
   * - openpi 策略客户端
     - ``openpi-client``
     - .. literalinclude:: ../examples/install.sh
          :language: bash
          :start-after: # start check-openpi
          :end-before: # end check-openpi

**你应该看到**：导入命令打印 ``OK`` 或版本号；HID 和事件设备命令打印设备列表；
``tty`` 打印当前终端路径。没有实体设备时，列表可以为空。
这些命令只检查依赖或枚举，实际读数在设备章节检查。

**如果不对**：

* ``ModuleNotFoundError``：在当前激活环境中重跑安装命令，补上对应 extra。
* SpaceMouse 列表为空或打开失败：检查 USB 和 HID 权限，后续步骤见 :doc:`spacemouse`。
* GELLO 串口、踏板权限或主从配置不通：后续检查见 :doc:`gello`。
* ``pynput`` 在 SSH 中报显示服务错误：改用 ``stdin``；Pico 的数据源检查素材暂存于 :doc:`gello`。

4. 做一次不接硬件的检查
--------------------------------------------------------------------------------

运行模拟设备检查，确认当前环境能导入项目、找到命令并读取设备反馈。

``station`` 是一份 YAML，写这台机器接了哪些设备、怎么连。
``mock`` 表示把真实设备换成模拟设备。
``doctor`` 会生成检查报告，每一行是一项检查。
安装脚本不带 station 时已经做过一次 mock 检查；现在单独运行，方便以后复查。

.. tutorial-test: skip
.. literalinclude:: ../examples/install.sh
   :language: bash
   :lines: 24-24

.. literalinclude:: ../examples/install.sh
   :language: bash
   :lines: 25-26

安装了 ``learning`` 的机器再运行：

.. tutorial-test: skip
.. literalinclude:: ../examples/install.sh
   :language: bash
   :start-after: # start learning-import
   :end-before: # end learning-import

**你应该看到**：导入命令打印 ``Imports OK:`` 和本机路径。
``rwrl --help`` 开头如下，这是实际输出节选：

.. literalinclude:: ../examples/rwrl-help.txt
   :language: text
   :lines: 1-8

完整 mock 报告如下；与机器相关的值使用占位符：

.. doctor-example: mock
.. literalinclude:: ../examples/mock-doctor.txt
   :language: text

每行格式为 ``STATUS id: message``；非 PASS 行有提示内容时附上 ``hint:``。
支持颜色的终端按下表显示；管道、文件和 JSON 输出不着色。
设置 ``NO_COLOR=1`` 也会关闭颜色（只要存在 ``NO_COLOR`` 环境变量就生效）。

.. list-table:: 检查状态
   :header-rows: 1

   * - 状态
     - 含义
     - 颜色
     - 阻止通过
   * - ``PASS``
     - 已检查且正常
     - 绿色
     - 否
   * - ``WARN``
     - 需要阅读提示
     - 黄色
     - 否
   * - ``SKIPPED``
     - 没有检查，不能证明可用
     - 灰色
     - 否
   * - ``FAIL``
     - 已检查且异常
     - 红色
     - 是
   * - ``BLOCKED``
     - 无法检查或修复，例如设备占用、权限不足
     - 品红色
     - 是
   * - ``PENDING_ACTION``
     - 仍需操作员完成重新登录、重新插拔等操作
     - 黄色
     - 是

mock 会跳过真实网络检查。``host.python`` 应指向你选定的项目环境解释器；
本章默认安装时是 ``.venv/bin/python``，自选 venv 或非 base conda 路径也可以。
该行 WARN 时先核对环境。``Report:`` 是本次报告目录，其中的 ``report.json`` 可留存排查。
主机环境和报告目录因机器而异。

**如果不对**：

* 找不到 ``rwrl`` 或项目包：确认已激活安装时创建的 ``.venv``。
* ``rl_garden`` 导入失败：重跑带 ``learning`` 的安装命令，检查子模块是否完整。
* ``handshake`` 报锁目录不可写：检查用户缓存目录权限；可用 ``XDG_CACHE_HOME`` 指定可写目录。
* 有其他 ``FAIL``：按该行的 ``hint`` 修复后重跑，保留打印出的报告目录。

现在你可以进入 :doc:`station`，连接真实设备。

.. note::

   **本章哪些内容需要真机确认**

   本章确认了软件导入、命令入口和模拟反馈。FCI 网络、SDK 与控制柜版本、
   HID 权限和读数、GELLO 总线与事件设备、Pico 数据流、夹爪反馈、相机画面、
   遥操作标定、控制时序和运动边界，要接入现场设备后确认。
