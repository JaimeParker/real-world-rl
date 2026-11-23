录制与标注示范
================================================================================

完成 :doc:`spacemouse` 或 :doc:`gello` 后，两种设备使用同一录制流程。
继续使用 ``local/station.yaml``、``local/task.yaml``；本章是单臂笛卡尔会话，
不套用 GELLO 章末尾自动复位的关节／双臂扩展。
一个回合是从 reset 后开始，到成功、失败、重录、超时或退出为止的一次尝试。

.. tip::

   本章命令省略了 ``--station`` 和 ``--preset``，用的是 :doc:`station` 第 10 步 ``rwrl use`` 记住的两份文件。
   还没做这一步时，先运行 ``rwrl use preset local/task.yaml``。要临时换文件，在命令里显式写出参数即可。

1. 启动有配额的录制
--------------------------------------------------------------------------------

先给会话一个新输出名（一个数据集目录，``local/first-demo``，不带后缀），并限制成功数和总尝试数，避免无意间继续下一轮。
同一个目录再次运行会在其后追加，所以换一次实验就换一个名字。

.. warning::

   启动后第一次 reset 就会归位并开爪。先检查 home、路径、物体摆放和停止方式。
   之后每次按 n 也可能归位；不要在保持控制时进入机器人运动范围。

.. literalinclude:: ../examples/record.sh
   :language: bash
   :start-after: # start record
   :end-before: # end record

``teleop run`` 从 station 选择设备，并把连接信息与 preset 合成会话配置。
``--num-successes 2`` 表示保留两条成功示范；``--max-episodes 5`` 表示至多五次完成的尝试，
失败、重录和超时也占一次。练习只做一轮时两项都设 1。
不传时成功配额为 20，总尝试数不限。每回合步数仍由 task 的 max_episode_steps 控制，本教程为 200。
``--record`` 指向数据集目录：每个回合在结束时写成一个 ``episode_000000/`` 子目录，
内存里始终只有正在录的那一个回合。成功配额数的是数据集里已保留的回合，包括之前运行留下的。

**你应该看到**：确认提示、reset，然后中立等待；成功结束时出现回合统计。
下面是源码格式示例，数值不是实测：

.. literalinclude:: ../examples/record-output.txt
   :language: text
   :start-after: # start episode
   :end-before: # end episode

**如果不对**：

* ``--record must be a demo dataset directory (new or existing), not a file``：``--record`` 要给目录名，不要带 ``.pkl``，也不要指向已有文件。
* ``teleop run requires --preset with home and task limits``：还没记住 preset；运行 ``rwrl use preset local/task.yaml``，或在命令里写 ``--preset local/task.yaml``。
* 停在 START_NEUTRAL：SpaceMouse 松帽和按钮；GELLO always 模式按上一章对齐主从臂；toggle 模式关闭实体 toggle。
* 配额一直不增加：看 discarded 原因；成功但没有人工步骤也不会保留为示范。

2. 完成回合并标注
--------------------------------------------------------------------------------

先用选定设备完成任务，再用事件按键标注结果，让成功示范和失败尝试能够区分。
本章默认事件源是运行命令的终端 ``stdin``，适合本机终端或 SSH 的交互终端。

.. list-table:: 当前代码的按键
   :header-rows: 1
   :widths: 15 25 60

   * - 键
     - 事件／状态
     - 本章怎样使用
   * - s
     - success
     - 确认任务成功后按；默认事件奖励启用，成功奖励为 1，并结束回合。
   * - f
     - failure
     - 标记失败并结束；同一轮出现失败时，不保留为成功示范。
   * - r
     - rerecord
     - 结束这次尝试；回合仍存入数据集，标为 discarded（rerecord），不进训练。
   * - Esc
     - episode_end
     - 结束回合，不自动代表成功。
   * - q
     - operator_quit + episode_end
     - 结束会话；保持阶段也用它退出控制。已结束的回合此前就已写入数据集。
   * - n
     - next_episode
     - 仅 session/hold 模式请求下一轮；还要通过中立检查。
   * - t / h
     - 接管／交还状态
     - 本章 GELLO 活动回合使用；SpaceMouse tracker 用 button_0，SERL 用运动／按钮电平。

**你应该看到**：s 后统计为 kept，successes 加一；f、r 或未成功的结束显示 discarded。
接管中才有人工候选步骤。夹爪按键只控制夹爪，不等于成功标注。
本章没有接奖励分类器；默认 ``reward_from_teleop_events: true`` 让标注产生奖励。

**如果不对**：

* 按键不生效：把焦点放回录制终端，检查终端仍连接；不要在另一个 shell 直接敲 s。
* ``discarded (no success)``：Esc、达到步数上限都不自动算成功，完成任务时明确按 s。
* ``discarded (no intervened steps)``：只按 s 没有操作，不能成为示范；先完成一次真实接管。

3. 观察保持，再开始下一轮
--------------------------------------------------------------------------------

回合结束后先观察保持状态，准备好现场再主动请求 reset。

.. literalinclude:: ../examples/record-output.txt
   :language: text
   :start-after: # start hold
   :end-before: # end hold

**你应该看到**：普通结束进入 EPISODE_HOLD，运动输入被忽略，不再调用环境 step，
也不把等待时间加入轨迹。仍有预算时，准备好下一轮，松开输入并按 n；
连续中立默认 0.2 秒后才 reset，reset 后再次等待中立再开始采样。
GELLO always 模式的中立门检查主从臂对齐；toggle 模式需关闭设备 toggle。t/h 只管活动回合。

需要更长的稳定等待时，GELLO 的 teleop run 命令可加 ``--neutral-stable-s`` 和秒数；
SpaceMouse 会话需在 preset 顶层设置 ``neutral_stable_s``，不接受这个 GELLO 专用 CLI 参数。
默认 0.2 秒，必须是非负有限数。它改变连续中立的时长，不取消中立条件。

预算耗尽会显示 ``Episode budget reached; next is disabled.``，仍等待 q，
不会自动退出控制。q 后关闭设备；各回合在结束时已写入数据集。正常退出证据保存在 session 目录。

.. warning::

   按 n 会允许复位和开爪。确认物体已放稳、路径已清空；需要进入工作区时，
   先按现场流程停止控制，不把 EPISODE_HOLD 当作断电状态。

**如果不对**：

* 按 n 后不动：检查预算是否已耗尽、帽／按钮／实体 toggle 是否仍处于接管状态，或 GELLO always 模式主从臂是否未对齐。
* 一结束就归位：检查是否绕过 teleop run 用了 ``rwrl record`` 的默认 auto 行为。
* 看不到成功示范：先正常按 q 退出，再用下一章的 ``rwrl dataset list`` 看各回合的 status 与原因；没有保留的回合仍在数据集里，只是标为 discarded。

4. 需要脚本事件或其他事件源时
--------------------------------------------------------------------------------

事件源只传接管状态和回合标注，运动仍来自 SpaceMouse 或 GELLO。
当前 teleop run 默认 stdin；给 events-file 时改用 jsonl。
先确保下面的事件文件不存在，再创建空文件，单独开一个新会话：

.. tutorial-test: skip
.. literalinclude:: ../examples/record.sh
   :language: bash
   :lines: 6-7

.. literalinclude:: ../examples/record.sh
   :language: bash
   :lines: 8-8

完成实际操作后，在第二个终端写入一个事件：

.. tutorial-test: skip
.. literalinclude:: ../examples/record.sh
   :language: bash
   :start-after: # start success
   :end-before: # end success

看到 EPISODE_HOLD 且现场已准备好后，才写入下一轮请求：

.. tutorial-test: skip
.. literalinclude:: ../examples/record.sh
   :language: bash
   :start-after: # start next
   :end-before: # end next

最后正常退出：

.. tutorial-test: skip
.. literalinclude:: ../examples/record.sh
   :language: bash
   :start-after: # start quit
   :end-before: # end quit

**你应该看到**：与键盘相同的状态转换。合法 event 名为 takeover、handback、success、
failure、rerecord、end、next、quit。每次 poll 只消费一行；不要预先填满整场会话的事件。
普通文件读到结尾会等待追加；``--events-file -`` 读管道，quit 前断流会报错。

桌面键盘可在独立 recorder 配置中选择 ``teleop_event_device: keyboard``，
依赖 pynput 和桌面会话；这不是 teleop run 的默认源。
GELLO 的 evdev 默认 Space 切换、S/F/R 标注、Esc 结束，无默认 n/q 绑定。
本章用终端／JSONL 补齐会话控制，不用踏板替代所有终端键。

**如果不对**：

* ``invalid JSONL operator event``：每行只能有一个 event 字段，使用上列名字并带换行。
* ``JSONL event stream closed before quit``：生产端先写 quit 再关闭管道。
* 旧事件被重放：换一个新空事件文件；touch 不会清空已有文件。

5. 找到录制产物
--------------------------------------------------------------------------------

正常退出后，按输出名找到数据集目录和会话证据。
下面是使用 first-demo 时的路径示例；uuid 由程序生成。

.. literalinclude:: ../examples/record-output.txt
   :language: text
   :start-after: # start files
   :end-before: # end files

**你应该看到**：数据集每个回合一个目录，所有回合都在，包括无人接管、失败、重录和中断的：
``meta.json`` 记录 status（kept 或 discarded）、原因、步数、接管区间和事件；``data.npz`` 是逐步数据；
``frames/<键>.npz`` 是无损图像；``video/<键>.mp4`` 是按控制频率合成的回看视频，只给彩色相机。
训练只用 kept 回合里的人工步骤，默认排除 ``teleop_syncing`` 对齐步骤。
动作／观测约定保存在 ``dataset.json``，不再有 pkl、``.contract.json``、SQLite 或 ``.trajectory.pt``。
回合写完才出现：正在写的目录叫 ``.tmp-episode_...``，崩溃后下次打开会自动清掉，不会被当作回合。

.. note::

   图像按无损保存，磁盘占用比旧 pkl 大。224×224 的 RGB 一帧约 150 KB，每个相机每个控制步一帧，压缩前就是这个量级；
   本教程 10 Hz、200 步的回合，单相机约 30 MB。录制前确认目标磁盘有空间。

session 目录里的 resolved-config 是最终配置，provenance 是来源信息，summary 是会话结果；
diagnostics 记录控制诊断，session-events 记录 READY、ACTIVE、EPISODE_HOLD 等阶段。
diagnostics、session-events、stop-trace 文件名都带时间戳和随机后缀。
正常停止时生成 stop-trace 和同名 assessment；故障或未运行到该阶段时可能缺少。
GELLO 额外保存提取出来的 leader-robot 和 leader-options。

**如果不对**：

* 数据集里没有 kept 回合：看终端 ``[record] episode=...`` 行的 discarded 原因，到下一章用 ``rwrl dataset list`` 核对。
* summary 的 stop_assessment 为 BLOCKED：本次缺少正常停止证据，不把会话返回码当成停止效果已验证。
* 找不到输出：查看 summary 的 output、report_directory，以及运行时的工作目录。

6. 需要直接使用普通录制器时
--------------------------------------------------------------------------------

需要独立管理源与环境选项时才走这条扩展。把已验证会话的 resolved-config.yaml
复制为 ``local/record-cartesian.yaml``，保留机器人连接、home、限位、观测和源选项文件。
确认其中 leader-options 等路径仍可访问。换输入源前删除复制配置中的 ``teleop_input`` 缓存，
否则旧的源选项快照可能继续生效；使用 GELLO 会话配置作为下面两条命令的起点。普通 ``rwrl record`` 读取这份配置，
不会像 teleop run 一样自动合并当前 station；核对 real_robot 的 task_frame、workspace。

.. warning::

   下列命令显式选择 auto：回合结束后可能自动 reset、归位并开爪，不等待 n。
   启动前核对复位路径，现场留人监护；直连时先停止占用同一从臂的 daemon。

.. literalinclude:: ../examples/record.sh
   :language: bash
   :start-after: # start ordinary
   :end-before: # end ordinary

supervise 在独立进程监视录制器心跳，正常心跳超时默认 2.0 秒，超时按
SIGINT → SIGTERM → SIGKILL 升级。它不能独立向机械臂发送停止命令；强制结束后依赖连接断开与固件停止。
命令还显式设置 ``--real_robot.watchdog_s 0.3``，让进程内 StepWatchdog 检测缺失步骤；
整个硬件进程卡住时，不能依赖这个进程内检查独立运行。

**你应该看到**：reset 后保持，GELLO 用 t/h 接管与交还，标注后显示回合结果及保存行。
普通录制器不会生成 teleop run 那套完整会话报告。

**如果不对**：启动失败先读 supervise.jsonl 与终端错误，核对配置文件和设备占用；
不要因监督器仍在运行就认为机器人已正常停止。

从 GELLO 配置改用 SpaceMouse 时，清掉主臂源选项，避免将它们传给 SpaceMouse：

.. literalinclude:: ../examples/record.sh
   :language: bash
   :start-after: # start ordinary-spacemouse
   :end-before: # end ordinary-spacemouse

**你应该看到**：tracker 模式用按钮 0 接管／交还，终端负责标注；t/h 不控制此输入源。

**如果不对**：源报未知选项时确认已传 ``--teleop-options-yaml None``；
方向配置也须按 :doc:`spacemouse` 核验，不能只换设备名称就开始采集。

7. 中断后恢复普通录制
--------------------------------------------------------------------------------

先查看监督器日志，检查工位、机器人故障和停止状态，再决定是否重新启动。
重跑第 6 步同一个输出目录的命令，就是追加：已有回合保留，新回合接着编号，编号不会重用。
追加要求本次的环境与数据集头（动作约定、观测形状、控制频率）一致，否则报错并要求换新的 ``--output-dir``。
被 Ctrl-C 或错误打断的回合，只要已有一步，就存为 discarded（interrupted）。
每个回合先写成临时目录、写完才改名，所以崩溃只会丢正在录的那一个回合。
进程崩溃与断电边界见 :doc:`../reference/real-world-experiments`。

.. warning::

   重启也会 reset 和操作夹爪。先重新检查现场与复位路径，不因复用文件名就跳过运动检查。

**你应该看到**：启动行如 ``[record] dataset local/ordinary-demo: 3 episodes (2 kept); recording until 5 kept``；
``--num-successes`` 数的是数据集里已保留的回合（含旧的），所以只会补录差额。已经够数时一个回合也不录。

**如果不对**：报错说数据集用了不同的 ``action_contract``、``obs_spec`` 之类的键：换新的 ``--output-dir``，
或用录这个数据集时的设置重跑。不要手改 ``dataset.json`` 来绕过，也不要把 teleop run 的会话报告目录当作普通录制器恢复入口。

.. note::

   **本章哪些内容需要真机确认**

   任务成功／失败标准、标注按键时机、首次和下一轮 reset 路径、开爪影响、
   保持阶段实际运动、中立等待、终端断连和正常停止效果，需要现场确认。
   JSONL 格式验证与离线归档检查只能确认软件路径。

检查清单：至少完成一个成功回合和一次失败尝试；见过 EPISODE_HOLD；
能用 n 开始下一轮、q 正常退出；找到数据集目录里各回合的 meta.json。

接下来 :doc:`inspect`，指出哪一段是自己操作的。
