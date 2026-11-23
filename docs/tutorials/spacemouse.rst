用 SpaceMouse 遥操作
================================================================================

本章沿用 :doc:`station` 的 ``local/station.yaml`` 和 ``local/task.yaml``。
在插着 SpaceMouse 的录制机器、仓库根目录运行命令。先读输入，再让机械臂运动。
工作区沿用 :doc:`station` 的 ``base``；SpaceMouse 的方向以 :doc:`station` 第 11 步的桌面坐标系为准。

.. tip::

   本章命令省略了 ``--station`` 和 ``--preset``，用的是 :doc:`station` 第 10 步 ``rwrl use`` 记住的两份文件。
   还没做这一步时，先运行 ``rwrl use preset local/task.yaml``。要临时换文件，在命令里显式写出参数即可。

1. 找到设备，补上 teleop 块
--------------------------------------------------------------------------------

先确认 USB 节点可访问，再告诉 station 使用哪个输入后端。
``teleop`` 是 station 中描述遥操作设备的一块配置。

.. tutorial-test: skip
.. literalinclude:: ../examples/spacemouse.sh
   :language: bash
   :start-after: # start hid
   :end-before: # end hid

把下面这段加到 ``local/station.yaml`` 顶层；已有 teleop 块时替换它。

.. literalinclude:: ../examples/spacemouse-config.txt
   :language: yaml
   :start-after: # start teleop
   :end-before: # end teleop

.. list-table::
   :header-rows: 1

   * - 必须改的字段
     - 怎么得到这个值
   * - ``teleop.backend``
     - 本章填 ``spacemouse``。
   * - ``options.device_index``
     - 只有一台受支持设备时保留 0；多台时逐台拔插，用下步读数确认索引对应的设备。

其余字段保留示例默认：``name``、``deadzone: 0.001``、``invert_axes: []``、
``poll_interval_s: 0.005``。只有静置噪声确实影响输入时才调整 deadzone；
方向先用下面的档案标定，不要同时随意改 invert_axes。轮询间隔改变前先测输入时序。

**你应该看到**：HID 列表包含设备的 vendor/product ID，节点权限允许当前用户读写。
仓库 udev 规则对 ``256f``、``046d`` 的 hidraw 设置 ``MODE="0664"``、``GROUP="plugdev"``。

**如果不对**：

* ``franka.sdk:teleop:spacemouse`` 失败或列表为空：检查 USB、``spacemouse`` extra；实际使用的是 ``hidapi``，导入名为 ``hid``。
* doctor 的 ``franka.os:hid``／``franka.os:teleop:spacemouse`` 失败：核对节点组与 ``id``，按安装章的 doctor 流程审阅权限修复建议，重新登录并插拔后复查。
* 型号未知：核对 HID 解码表；任意 ``/dev/hidraw*`` 可能是其他设备，放宽权限不能补齐解码支持。

型号未知时，先采原始证据，不启动机器人控制：

.. literalinclude:: ../examples/spacemouse.sh
   :language: bash
   :start-after: # start probe
   :end-before: # end probe

存在多个候选设备时，用 ``--device VID:PID`` 指定 HID 列表中的十六进制 ID。
采样时逐轴推帽、按按钮。审阅报告后，将 candidate.yaml 中的设备条目合入
``local/spacemouse-devices.yaml`` 的 devices 映射，再做下一步读数检查。

**你应该看到**：报告目录中的 reports.jsonl 和 candidate.yaml；读到描述符时还有 descriptor.hex。
候选只是解码草稿，先核对轴布局、符号、按钮和其中的 TODO，再合并；probe 不会自动安装它。

**如果不对**：没有报告时重新采样并移动输入帽；无法读取描述符时保留已有报告，
不能把缺失证据当作型号已经验证。路径或权限错误先修复再采，不任意尝试其他 hidraw 设备。

2. 读六轴和按钮
--------------------------------------------------------------------------------

逐个推、拉、扭输入帽并按两个按钮，确认软件读到了哪一轴、哪一个按钮。
这两条命令只读取输入，不启动机器人控制器。

.. literalinclude:: ../examples/spacemouse.sh
   :language: bash
   :start-after: # start read
   :end-before: # end read

**你应该看到**：``teleop read`` 每行有 ``twist`` 的六个数、``gripper``、
``intervening`` 和 ``events``；短按分别出现 ``button_0``、``button_1``。
六轴顺序是 x、y、z、rx、ry、rz，设备层固定映射为解码值的
``[-y, x, z, -roll, -pitch, -yaw]``，再应用 invert_axes。
这里的 gripper 和 intervening 是设备层读数；接管练习使用下一节所述的 tracker 规则。

``diagnose`` 给出 ``report_directory``。其中 ``samples.jsonl`` 保存逐次采样的
``decoded``、``mapped``、``task_input``、``base_input``、``buttons`` 和 ``fresh_report``。
用它检查轴正负和按钮电平；缓存的轮询读数不算新 HID 报告。

设备层每次轮询最多读取 64 条积压的 HID 报告，保留本次读到的最新轴值，
并锁存其中的按钮按下事件；超过上限的积压要等后续轮询，不能保证输入永不滞后。
会话输入诊断中的 ``hid_reports_this_poll`` 是设备层本次读取的报告数；持续达到上限时应检查积压。
推帽时一般是 0–2；长期很大说明轮询太慢。
``teleop diagnose`` 的 samples.jsonl 使用逐条读取，记录 ``report_count`` 和 ``fresh_report``，没有这个字段。

**如果不对**：

* 一轴操作带出其他轴：先减小手部耦合、逐轴重试，再查看原始 decoded 与 mapped。
* 静置值不归零：检查帽是否受力，用诊断报告确认噪声后再调整阈值。
* ``exclusive input diagnostic blocked``：结束占用输入的录制／诊断程序后再运行。

3. 建立方向档案，核对坐标系
--------------------------------------------------------------------------------

先确认 :doc:`station` 第 11 步做完了：机械臂不是正装时，station 里要有测好的 ``task_frame``
（例如 ``name: table``）。SpaceMouse 的读数先按桌面方向理解（前 = 桌面 +x，左 = +y，上 = 竖直 +z），
再由 ``task_frame`` 的旋转 ``base_R_table`` 转到机器人基座系，最后才变成末端目标交给控制器。
方向档案只在这之上再加一个 yaw：SpaceMouse 在桌上朝哪个方向摆。

方向档案保存当前设备摆放方向与任务方向之间的旋转，避免每次凭手感猜方向。
正装、没有 ``task_frame`` 的机器人用下面的命令（``--frame base``）：

.. literalinclude:: ../examples/spacemouse.sh
   :language: bash
   :start-after: # start calibrate
   :end-before: # end calibrate

按提示依次静置，再沿指定的正负方向操作，每组默认采样 3 秒。
提示用操作员的说法描述方向，方括号里是数据标签，例如
``roll + (about forward X: tilt the cap so its right side goes down) [+rx]`` ：
roll／pitch／yaw 分别绕前向 X、左向 Y、向上 Z 旋转（右手规则）。
``+ry`` 是帽子前倾、前缘向下，``+rz`` 是从上方看逆时针扭帽。
``--frame base`` 指方向标签按机器人基座系解释；``--profile operator-base`` 是自己取的档案名。
``--output`` 相对 station 所在目录，因此写入 ``local/spacemouse-profiles.yaml``。
``--use`` 会在确认后保存档案，并写入 station 的 ``teleop_profiles`` 引用。

**你应该看到**：结果有 ``candidate``、``frame``、``yaw_rad``、
``horizontal_validated``、``rotation_validated`` 和保存路径。
完整采完六轴正负方向后检查两个 validated 都为 true。
``profiles show`` 应选中 operator-base，``workspace show`` 应仍显示上一章审阅过的 base 边界。
只采水平四向能拟合 yaw，但不能证明旋转方向已经验证。

有 ``task_frame`` 的机器人（例如侧装 45°）用桌面系标定，``--frame`` 写 ``task_frame`` 的 ``name``：

.. literalinclude:: ../examples/spacemouse.sh
   :language: bash
   :start-after: # start calibrate-table
   :end-before: # end calibrate-table

这时提示里的方向按桌面理解（前、左、上都是桌面的），档案的 yaw 与 ``base_R_table`` 组合成最终旋转。
一定要先定 ``task_frame`` 再标定档案；之后改了 ``task_frame``，档案要重新标定。方向档案不改变工作区几何。

**如果不对**：

* ``calibration frame must match station task_frame``：frame 名称必须是 base 或已配置的 task_frame.name。
* ``direction error``：检查是否沿反方向操作，重新采该方向；不要把失败结果当成已标定。
* ``connected SpaceMouse does not match profile``：设备身份与档案不符，选择对应档案或重新标定。

离线可复现检查
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

没有输入设备时，用 fake 读数检查输出格式，再用测试中的方向样本验证拟合入口。

.. literalinclude:: ../examples/spacemouse-offline.sh
   :language: bash
   :start-after: # start read
   :end-before: # end read

**你应该看到**：以下为实际 mock 输出，时间会略有变化；它不验证 HID 解码。

.. literalinclude:: ../examples/spacemouse-mock-output.txt
   :language: text
   :start-after: # start output
   :end-before: # end output

.. tutorial-test: skip
.. literalinclude:: ../examples/spacemouse-offline.sh
   :language: bash
   :lines: 5-12

.. literalinclude:: ../examples/spacemouse-offline.sh
   :language: bash
   :lines: 13-13

**你应该看到**：实际拟合结果如下。只提供水平样本，所以 rotation_validated 为 false；
没有保存，也不能把它当成自己的设备档案。

.. literalinclude:: ../examples/spacemouse-calibration-output.txt
   :language: json
   :start-after: # start output
   :end-before: # end output

**如果不对**：找不到测试文件时回到仓库根目录；缺少 pytest 时补齐 test extra。

需要重复检查某一方向时，给 calibrate 加 ``--samples PATH`` 读取已有 JSON 样本，
用 ``--capture-label +x`` 只重采该方向；默认替换该组，加 ``--append-trial`` 才保留旧试次。
采样后的 merged-samples.json 路径会打印出来，下次用它作 samples 输入。
水平四向不齐时输出 NEEDS_SAMPLES，先补齐再拟合；不要把单向采样当成完整标定。

保存与选用是两步：``--save`` 确认后只保存档案，``--use`` 才同时写 station 引用。
已保存到 station 所引用档案文件的配置可以直接列出、切换，不必重新采样：

.. literalinclude:: ../examples/spacemouse.sh
   :language: bash
   :start-after: # start profiles-select
   :end-before: # end profiles-select

**你应该看到**：list 列出已保存档案，use 确认后更新默认选择，show 显示对应档案。

**如果不对**：找不到档案时检查 station 的 teleop_profiles.file 与档案名；
只用 --save 写到另一个文件，不会自动让 station 引用它。

4. 用默认 tracker 练习接管、夹爪和交还
--------------------------------------------------------------------------------

先做一次短会话，确认手部输入、实测运动和控制权切换一致。
``tracker`` 会维护一个末端目标，并限制它的速度和领先实测位置的距离。
默认目标速度为 0.05 m/s、0.3 rad/s，领先上限为 0.05 m、0.3 rad。

.. warning::

   下面会启动机械臂，首次 reset 会归位并开爪。先清空归位路径、核对 home、
   工作区和停止方式；夹爪附近不要放手。保持现场监护。

.. literalinclude:: ../examples/spacemouse.sh
   :language: bash
   :start-after: # start practice
   :end-before: # end practice

确认运行后，会话 reset 并停在 READY：手臂在 home，帽子和按钮的运动被忽略。摆好物体，在终端按空格开始回合（ACTIVE）。
松开输入帽和两个按钮，按 button_0 接管，
轻推一个方向，再松开帽；目标停止推进，机器人可能继续走完剩余领先距离。
接管中按 button_1 切换开／闭爪；再次按 button_0 交还。
交还后录制器保持当前位置。终端的 ``t``／``h`` 不控制这个模式。
最后按 ``s`` 标记练习成功，回合进入 POST_EPISODE，按 ``q`` 退出。空格、``n``、``o``/``c`` 等回合之间的键见 :doc:`record`。

**你应该看到**：按 button_0 接管时终端打印 ``INTERVENTION ON: the operator drives the arm`` ，
交还后约 0.3 s 打印 ``INTERVENTION OFF: the operator handed back`` ；按 ``s`` 或 ``f`` 打印
``EPISODE MARK: success`` 或 ``failure`` 。只有按 button_0 后推帽才运动；接管瞬间保持上次夹爪命令，
不会自动打开。松帽不等于交还。完成回合后显示 POST_EPISODE，详见 :doc:`record`。

推帽时手腕难免让帽子同时倾斜。tracker 模式默认 ``axis_lock: dominant`` ：每次读数只保留
平移、旋转两组中峰值较大的一组，另一组清零，所以单纯推 x 不会带出 pitch。
需要同时平移和旋转时，在 station 的 ``teleop.options`` 写 ``axis_lock: "off"`` （serl 模式不接受此项）。

.. note::

   机械臂由谁执行这些目标，取决于 station 的 ``cartesian_controller`` （``impedance`` 默认、``internal``、``serl``），
   与这里的 tracker／serl 模式无关，两者可以任意组合，见 :doc:`controller`。
   控制器收到的已经是基座系目标，不会再旋转；换控制器只改 station 里的一行，方向档案和 preset 都不用改。

**单轴检查。** 每次只沿一个方向推（或绕一个轴扭）一下，看 TCP 是否沿\ **桌面**\ 的同一方向走（或绕同一轴转）。
平移用 ``workspace capture`` 读数：会话前读一次，会话里按空格开始，只推一个方向约 3 cm 后交还、按 ``s``、按 ``q`` 退出，再读一次：

.. literalinclude:: ../examples/spacemouse.sh
   :language: bash
   :start-after: # start axis-check
   :end-before: # end axis-check

.. literalinclude:: ../examples/spacemouse-axis-output.txt
   :language: text
   :start-after: # start capture
   :end-before: # end capture

``task_position`` 是 TCP 在桌面系里的位置（没有 ``task_frame`` 时就是基座系）。只推了“上”时，应该只有 z 明显增加：

.. literalinclude:: ../examples/spacemouse-axis-output.txt
   :language: text
   :start-after: # start check
   :end-before: # end check

前、左、上三个方向各做一次；三个旋转用眼睛看：绕“前”轴扭帽，夹爪绕桌面前方的轴转，依此类推。
推“上”而 TCP 斜着走，说明 ``task_frame`` 没写或角度不对，回到 :doc:`station` 第 11 步。

需要限制诊断输入时，给上述 ``teleop run`` 加 ``--diagnose translation``、
``rotation`` 或 ``all``；这是会运动的会话，要求 station 已有审阅过的 workspace。
此时 ``--gain`` 默认 0.2，范围 (0, 1]；``--duration-s`` 默认 5 秒，范围 (0, 300]，
换算成回合步数上限。未显式指定时，成功配额和回合预算都为 1；回合结束仍进入 POST_EPISODE，预算用完后 ``n`` 或 ``q`` 结束会话。
不带 ``--diagnose`` 时，gain 和 duration-s 不生效；输入过滤设置会写入控制约定。

tracker 的领先上限不能超过 preset 一步的动作尺度，否则会话启动时报
``needs xyz_scale_m >= lead_cap_m and rot_scale_rad >= lead_cap_rad``。
例如按 SERL 的习惯把 ``real_robot.xyz_scale_m`` 设为 0.02、``rot_scale_rad`` 设为 0.1 时，
要在 preset 顶层同时写 ``teleop_lead_cap_m: 0.02`` （默认角度上限 0.3 rad 超过 0.1，也要写 ``teleop_lead_cap_rad: 0.1``）。

**如果不对**：

* 一直停在 READY：开始回合要在运行 ``rwrl`` 的终端按空格，帽子和按钮不管开始。档案的 neutral_threshold（默认 0.05）只用于 ``hold`` 模式。
* ``WORKSPACE HOLD: <axes>``：请求越界，回查 workspace 和 safety_box；交还后重新接管，不要扩大边界绕过检查。
* 方向或幅度不符：退出会话，回查方向档案、任务系及 preset 的动作尺度，再做单轴练习。

5. 再决定是否使用 SERL 模式
--------------------------------------------------------------------------------

只有需要靠推帽即时接管的操作习惯时，才另开 SERL 会话并重新核对动作约定。
先删除 station 的 ``teleop.options.deadzone`` 和显式设置过的 ``axis_lock`` 字段（设备层仍使用默认 deadzone）；
SERL 源会拒绝 tracker 专用选项。再复制 preset；SERL 要求环境层同时具备 XYZ 和 RPY 边界，
上一章的 task 模板没有这些字段。

.. tutorial-test: skip
.. literalinclude:: ../examples/spacemouse.sh
   :language: bash
   :lines: 45-45

.. literalinclude:: ../examples/spacemouse.sh
   :language: bash
   :lines: 46-47

.. tutorial-test: skip
.. literalinclude:: ../examples/spacemouse.sh
   :language: bash
   :lines: 48-49

在已经独立确认安全的姿态下读取 TCP，用同一段换算程序把 xyzw 转成弧度。
如需手动引导，沿用 station 章的停止控制、现场监护流程；此处读数本身不运动。
将以下片段合入新文件的 real_robot 块，把所有占位符换成审阅过的数值：

.. literalinclude:: ../examples/spacemouse-config.txt
   :language: yaml
   :start-after: # start serl
   :end-before: # end serl

.. list-table::
   :header-rows: 1

   * - 必须改的字段
     - 怎么得到这个值
   * - ``real_robot.safety_box``
     - 从 workspace show 复制已审阅的 base_safety_box，顺序保持 xmin、ymin、zmin、xmax、ymax、zmax。
   * - ``safety_box_rpy_low`` / ``safety_box_rpy_high``
     - 用本步 TCP 换算值和允许的姿态范围取上下界；roll 用绝对值，pitch／yaw 保留正负，单位 rad。

其余保持默认：gripper_action_scale 为 1、gripper_sleep_s 为 0.6 秒；
只有核对夹爪反馈与实际开闭间隔后才调整。SERL 的 roll 限制保留原符号、裁剪幅值。
例如 station 章的示例四元数 ``[1, 0, 0, 0]`` 换算为 ``[pi, 0, 0]``，
这是一个姿态点，不能由此推断允许范围。跨 Euler 分支或奇异点时先重新选择可表达的姿态区间。


.. warning::

   SERL 中推帽本身就会接管；两个按钮也换了含义。先让帽静置，确认现场空间后再启动。

.. literalinclude:: ../examples/spacemouse.sh
   :language: bash
   :start-after: # start serl
   :end-before: # end serl

这条命令显式写了 ``--preset local/task-serl.yaml``，所以本次用它，不用记住的 ``local/task.yaml``；
stderr 也不会再打印 ``using preset``。

**你应该看到**：运动输入超过 0.001 或任一夹爪按钮按住时接管；
松开帽和按钮交还。button_0 按住关闭，button_1 按住打开，同时按时关闭优先。
它输出原始归一化输入，不使用 tracker 的目标速度与领先限制。
方向：serl 模式默认只用 station ``task_frame`` 的旋转 ``base_R_table`` 把输入转到基座系，\ **不加**\ 第 3 步档案的 yaw。
所以 SpaceMouse 要按 HIL-SERL 的方式\ **摆正**\ ：帽子的“前”对着桌面 +x（离开机器人底座的方向），“左”对着桌面 +y，“上”是竖直向上。
从上往下看：

.. code-block:: text

                 桌面 +x（前，离开底座）
                        ▲
                        │
          +y（左） ◄── [SpaceMouse]       线缆一侧朝向操作员
                        │
                 ┌──────┴──────┐
                 │  Franka 底座 │          +z：竖直向上（垂直纸面朝外）
                 └─────────────┘

正装的机器人（没有 ``task_frame``）里桌面系就是基座系，这正是 HIL-SERL 的做法：它假设机器人正装，直接用原始基座系。
侧装的机器人有了 ``task_frame``，推“上”仍是竖直向上，不会沿倾斜的基座 z 轴走。
摆正后 SERL 不需要用 yaw 修正方向，但当前 ``teleop run`` 仍要求 station 引用一个方向档案，
用于设备身份和中立阈值等信息；保留第 3 步的档案。做下面六个单轴检查，每次只动一个轴，看 TCP 是否沿（或绕）桌面系的同一个轴运动：

.. list-table::
   :header-rows: 1

   * - 操作
     - TCP 应该
   * - 向前推帽
     - 沿桌面 +x 走（离开底座）
   * - 向左推帽
     - 沿桌面 +y 走
   * - 向上提帽
     - 竖直向上走
   * - 绕前后轴转帽（右侧向下）
     - 绕桌面 x 轴转，夹爪右侧向下
   * - 帽子前倾（前缘向下）
     - 绕桌面 y 轴转，夹爪前端向下
   * - 从上方看逆时针扭帽
     - 从上方看逆时针转

平移的方向用 ``workspace capture`` 的 ``task_position`` 读数确认（与第 4 步相同，会话前后各读一次，每次会话只推一个方向）：

.. literalinclude:: ../examples/spacemouse.sh
   :language: bash
   :start-after: # start serl-axis-check
   :end-before: # end serl-axis-check

实在不能这样摆（例如操作员只能站在侧面）时，用第 3 步的档案：在 station 的 ``teleop.options``
写 ``use_profile_rotation: true``，serl 模式就用档案的完整旋转（``base_R_table`` × 档案 yaw）。
它只接受 true／false；tracker 模式始终使用档案（没有档案时用 ``task_frame``）。
这个选择会写进录制的 teleop_control 记录；用旧版本录制、隐式套用了档案旋转的 RL 运行，
在新版本恢复（resume）时会因为记录不一致被拒绝，而不是悄悄换方向。
会话同时把环境 ``cartesian_semantics`` 切成 ``serl``；增加 RPY 裁剪并改变夹爪执行约定，
不要把两种模式的数据按相同 contract 合并训练。终端 t/h 在这里同样无效。

**如果不对**：

* ``serl requires complete safety_box`` 或 ``safety_box_rpy``：补齐新 preset 的环境层边界；仅 station 中有位置边界还不够。
* 以为短按会保持接管：SERL 使用当前输入和按钮电平，改回 tracker 才有锁存接管。
* 想调 tracker 速度却没有效果：SERL 不使用这些选项；重新审阅环境尺度及位置／姿态边界。
* 方向和 tracker 模式不一样：serl 模式默认不加档案 yaw；SpaceMouse 没按上图摆正时方向会转一个角度，这是预期行为。摆正它，或写 ``use_profile_rotation: true``。
* 推“上”时 TCP 斜着走：``task_frame`` 没写或不准，回到 :doc:`station` 第 11 步。

.. note::

   **本章哪些内容需要真机确认**

   HID 权限、型号解码、六轴方向、按钮编号、静置噪声、任务系变换、工作区、
   reset 路径、速度与领先距离、夹爪开闭，以及 tracker／SERL 的交还后运动都需现场确认。
   离线测试不能替代这些检查。

检查清单：能读六轴和按钮；方向档案已保存并选中；单轴运动符合预期；
能接管、开闭爪、交还并退出。

选择 GELLO 的读者转到 :doc:`gello`；用 ``serl`` 控制器的读者回到 :doc:`controller` 第 8 步检查反射后的自动恢复；
否则完成 SpaceMouse 练习后直接进入 :doc:`record`。
