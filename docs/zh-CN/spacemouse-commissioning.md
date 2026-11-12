<!-- English source: docs/spacemouse-commissioning.md -->

<a id="spacemouse-commissioning-profiles-task-frame-workspace-sessions"></a>

# SpaceMouse 调试：标定档案、任务坐标系工作区与会话

前置说明见 `rwrl` 的[全局选项](cli.md#global-options)与[连接归属](cli.md#connection-ownership)。

<a id="site-device-table"></a>

## 站点设备表

HID 读取器按设备的厂商号与产品号（`vid:pid`）精确匹配条目来解码 SpaceMouse。内置表即 PySpaceMouse 1.0.3 的表：SpaceExplorer、SpaceNavigator、SpaceMouse USB、SpaceMouse Compact、SpaceMouse Pro Wireless、SpaceMouse Pro、SpaceMouse Wireless、3Dconnexion Universal Receiver、SpacePilot 和 SpacePilot Pro。站点使用的其他型号或固件变体写入该站点的 local 设备表 `spacemouse-devices.yaml`，该文件不进 git。当 `rwrl discover` 或 `rwrl doctor` 打印

```text
unknown 3Dconnexion-vendor HID device 256f:xxxx; add it to spacemouse-devices.yaml (see docs/spacemouse-commissioning.md#site-device-table)
```

时，说明所连设备不在表中，`open()` 会拒绝它。该警告覆盖所有不在表中的 3Dconnexion（`256f`）设备；Logitech（`046d`）设备只有在其 HID descriptor 声明为多轴控制器时才报告，因此键盘或鼠标接收器不会被报告。

**位置。** 按以下规则取第一个适用的文件：

1. 工作站机器人的 `teleop.options.device_table`。相对路径相对于工作站文件所在目录。
2. 工作站文件旁的 `spacemouse-devices.yaml`：`local/station.yaml` 对应 `local/spacemouse-devices.yaml`，用户级工作站对应 `~/.config/robot_infra/spacemouse-devices.yaml`。
3. 没有工作站时（setup 探测、按未指定设备表的选项构建的设备，以及未找到工作站时 `rwrl doctor` 的主机层 USB 检查与 udev 规则）：取 `./local/spacemouse-devices.yaml` 与 `$XDG_CONFIG_HOME/robot_infra/spacemouse-devices.yaml` 中第一个存在的文件。

以上都没有时，只使用内置表。`hid_id` 与内置条目相同的 local 条目覆盖该内置条目；其他 local 条目作为新增。

**格式。** 下例以内置 SpaceMouse Wireless 条目的解码方式声明 `256f:c63a`（这只是格式示例，不是对该产品的结论：请按下文检查实际设备的解码）：

```yaml
# local/spacemouse-devices.yaml (site-specific, not committed)
schema_version: 1
devices:
  SpaceMouse Wireless BT (USB cable):
    hid_id: [0x256F, 0xC63A]            # or "256f:c63a"
    layout: SpaceMouse Wireless         # copy led_id, mappings, button_mapping, axis_scale
    evidence:                           # free-form; never used for decoding
      descriptor: evidence/256f-c63a/report-descriptor.txt
      checked: YYYY-MM-DD
      note: what the descriptor and the axis/button capture showed
```

`layout:` 只能引用内置条目，不能引用另一个 local 条目；与它并列写出的字段覆盖复制来的字段。不写 `layout:` 时，条目须完整写出解码方式：

```yaml
  My SpaceMouse:
    hid_id: "256f:c6ff"                 # placeholder: the vid:pid that rwrl discover reports
    led_id: null                        # or two integers, for example [0x8, 0x4B]
    axis_scale: 350.0
    mappings:                           # exactly x, y, z, roll, pitch, yaw
      x:     {channel: 1, byte1: 1,  byte2: 2,  scale: 1}
      y:     {channel: 1, byte1: 3,  byte2: 4,  scale: -1}
      z:     {channel: 1, byte1: 5,  byte2: 6,  scale: -1}
      pitch: {channel: 1, byte1: 7,  byte2: 8,  scale: -1}
      roll:  {channel: 1, byte1: 9,  byte2: 10, scale: -1}
      yaw:   {channel: 1, byte1: 11, byte2: 12, scale: 1}
    button_mapping:                     # button_0 first; the teleop reads the first two
      - {channel: 3, byte: 1, bit: 0}
      - {channel: 3, byte: 1, bit: 1}
```

文件在加载时校验（构建 SpaceMouse 遥操作设备时，以及 `rwrl discover` 和 `rwrl doctor` 运行时）；任一条目无效即整表拒绝，错误信息写明文件和条目名。顶层键只能是 `schema_version: 1` 和非空的 `devices` 映射；条目键为 `hid_id`、`layout`、`led_id`、`mappings`、`button_mapping`、`axis_scale` 和 `evidence`。`hid_id` 为两个 0..0xFFFF 的整数（或文本 `"vvvv:pppp"`），在文件内唯一；local 条目只有在使用内置条目的同一 `hid_id` 时才能沿用该内置条目的名字。轴的 `channel` 为 0..255，`byte1`/`byte2` 为 1..63，`scale` 为 -1 或 1；按钮的 `channel`、`byte`（1..63）、`bit`（0..7）均为整数，或三者都为 `null`；`axis_scale` 为有限正数。

**证据。** 把支持某条目的材料（HID report descriptor、每个轴和按钮的简短采集、日期）放在 `local/` 下设备表旁，并用 `evidence` 指向它，路径相对于设备表所在目录。`rwrl teleop diagnose` 在 summary 中记录解码该设备的条目（`device_spec`：名字、`hid_id`、`source` 为 `builtin` 或 `local`，local 条目另有设备表的 sha256）。录制和训练不把设备表放进参与比较的实验契约：`teleop_input` 只把 local 设备表的文件、sha256 和条目记作 provenance，`source_options` 中不含任何本机路径。

```bash
# rwrl teleop probe-hid：探测表外设备并起草条目（步骤见 docs/spacemouse-commissioning.md#site-device-table）
rwrl --station local/station.yaml teleop probe-hid --device 256f:c63a --duration-s 10
```

**复核。** 编辑设备表后，在机器人主机的仓库根目录、激活项目环境的情况下运行：

```bash
rwrl --station local/station.yaml discover
rwrl --station local/station.yaml doctor
rwrl --station local/station.yaml --json teleop diagnose --duration-s 5
```

`discover` 在 `spacemouse:` 下列出该设备且没有警告，`doctor` 的 `usb.spacemouse` 和遥操作检查通过，诊断结果中每个推动的轴和按下的按钮都有预期的符号（见[实时输入标定与诊断](#live-input-calibration-and-diagnosis)）。

<a id="spacemouse-profiles-and-task-frame-workspace"></a>

## SpaceMouse 标定档案与任务坐标系工作区

工作站机器人可声明任务坐标系（task frame）`task_frame`，包含 `name` 和有效的 4×4 `base_T_frame` 刚体变换。`teleop_profiles` 指定标定档案（profile）YAML 的 `file`（相对于工作站文件）和 `default` 档案名。档案文件包含 `schema_version: 1` 和 `profiles` 映射。每个档案声明 `frame`、`yaw_rad` 和 `axis_mapping_version: serl_hid_axes_v1`；`neutral_threshold` 默认为 0.05。此启动容差与正常介入及运动死区相互独立。

`rwrl workspace frame set` 写入 `task_frame`，无需手工编辑。在机器人主机的仓库目录激活项目环境后，由操作员摆放机械臂采集三个 TCP 点（坐标系原点、+X 上一点、X 轴 +Y 一侧的一点），再提出并保存坐标系：

```bash
rwrl --station local/station.yaml workspace capture --name frame-origin --output frame-points.yaml --save
rwrl --station local/station.yaml workspace capture --name frame-x --output frame-points.yaml --save
rwrl --station local/station.yaml workspace capture --name frame-y --output frame-points.yaml --save
rwrl --station local/station.yaml workspace frame set --name table \
  --from-points frame-points.yaml --origin frame-origin --x-axis frame-x --y-axis frame-y
rwrl --station local/station.yaml workspace frame show
```

该坐标系为右手系：X 从原点指向 X 点，Z 垂直于三点所在平面，Y 为 Y 点方向相对 X 正交化后的方向。候选结果报告两条实测轴长，以及实测与最终 Y 方向之间的夹角；X 点距原点不足 1 毫米或三点共线时拒绝。也可用 `--base-T-frame` 后跟 16 个行优先数字（米）给出已知变换。默认 `source` 来源为 `points:<file> origin=A x=B y=C`；`--source TEXT` 可另行设置。加 `--save` 后，在审阅差异并确认后只写入 `task_frame`。定义在另一坐标系中的工作站工作区会被拒绝；沿用其坐标系名称会对该工作区重新定基，因此需重新运行 `workspace validate` 并审阅边界。在旧坐标系中保存的点文件必须重新采集。`workspace frame show` 打印 `task_frame` 及其在基坐标系中的原点和单位轴。

`teleop_profiles` 是 real_world_rl 的站点分节（section），不是 `RobotConfig` 字段。设备层工作站加载器通过已安装 real_world_rl 包的 `robot_infra.station_sections` entry point 发现其校验器。未安装该包时，加载会报 unknown-key 错误并附安装提示。显式 `sections={}` 始终要求严格的设备解析。`rwrl` 和 Python 调用方统一使用 `real_world_rl.station.load_station`（或 `load_station_with_env`）注册其校验器。现有站点 YAML 无需迁移；`rwrl teleop calibrate --use` 仍将此分节写在机器人映射中。

在已安装的项目环境中，准备好工作站文件后运行：

```bash
rwrl --station local/station.yaml --json teleop profiles list
rwrl --station local/station.yaml teleop profiles show --profile rear
rwrl --station local/station.yaml teleop profiles use --profile rear
rwrl --station local/station.yaml workspace show
```

离线标定接受 JSON 映射，将 `+x`、`+y`、`-x`、`-y` 映射到**经过固定 HID 轴映射后**的归一化六轴样本列表。正方向标签用于拟合 yaw，负方向标签用于独立验证。最大方向残差默认为 25 度（`--max-error-deg`）。这些结果不表示旋转方向已经验证。

```bash
rwrl --station local/station.yaml --json teleop calibrate \
  --samples local/direction-samples.json --profile rear --frame table
```

添加 `--save --output profiles.yaml`，确认后保存。输出路径相对于工作站文件。结果包含 `station_reference`，用于配置该机器人的 `teleop_profiles`。保存时将旧文件保留在名称唯一的 `<file>.<id>.bak` 备份中，权限为 `0600`；被编辑档案之外的 YAML 注释、引号和键顺序均保留。确认前，stderr 显示相对于精确读取文件的 unified diff。审阅或暂存期间检测到变化会拒绝保存。这是乐观冲突检测，不会锁住外部编辑器；保存时请勿编辑同一文件。非交互保存需要全局 `--yes`。

只重采一个方向时，使用 `teleop calibrate --profile rear --samples local/direction-samples.json --capture-label=+x`。这会替换该方向并保留其它组；`--append-trial` 则添加一次独立推动。每次推动的拟合权重相同，不受持续时间或力度影响。JSON 的每个方向可使用旧式样本行或 `{"trials": [rows, rows]}`。每次留出验证试验都必须通过，包括带正负方向的旋转试验。新报告包含 `merged-samples.json`，下次采集将其路径作为 `--samples`。原样本文件不会被覆盖。水平组不足时返回 `NEEDS_SAMPLES` 和 `required_fields`，不保存标定档案。没有终端时，带标签采集需要全局 `--yes`，并立即开始持续 `--duration-s` 秒（默认 3 秒）。提示及合并文件位置写入 stderr。混合/离线样本不能确定单个物理设备身份；只有完整的实时采集才绑定设备身份。

原始输入 JSONL 区分新收到的 HID 报告（`fresh_report`）与缓存状态轮询。只有新报告参与带标签样本拟合；每次采集必须收到至少一份新报告。空 HID 读取保留状态，因此仅凭静默不判断松开或断连。没有报告的采集会因证据不可用而失败，SDK 读取错误会向上传播。此仅输入采集检查不能证明设备持续连接，也不能证明分拆 HID 报告中每个轴都是最新的。

输入采集和受监督的 CLI 会话会在打开输入或启动控制前保存私密的 `provenance.json` 溯源文件。它记录 `real_world_rl`、`robot_infra` 和 `rl_garden` 的 Python 源文件哈希、已安装发行包名称和版本，以及规范化配置摘要。会话记录还会计算工作站、预设、标定档案和解析后配置文件的哈希。路径不影响包的源码摘要；内容修改和文件重命名会影响。这是运行前的磁盘证据，不是对已加载内存或所有依赖二进制的证明。运行期间保持源码/配置不变，将这些记录与原始输入、轨迹和停止报告一起归档。

录制/训练时，用 `--teleop-options-yaml` 提供工作站，选择 `--teleop-device spacemouse`，并可覆盖 `--teleop-profile` 和 `--teleop-robot`。解析后的输入旋转、档案名称/文件和 yaw 在保存配置前冻结，保留在不参与比较的 `teleop_input` 溯源中。恢复运行复用这些数值选项；重新标定操作员不会改变基坐标系中已记录动作的含义。旧工作站的 `teleop.options.input_rotation` 不能与具名标定档案或工作站 `task_frame` 混用。应将其安装变换部分迁到任务坐标系，并用标定档案选择操作员 yaw。没有这两个新字段的旧配置保留原输入旋转行为。

`workspace set --frame table --low X Y Z --high X Y Z` 预览任务坐标系位置盒；`--save` 在确认后应用。现有基坐标系 `safety_box` 约束仍生效。设备层拒绝任务坐标系工作区外的目标，包括经基坐标系裁剪后移到工作区外的目标。训练环境先在任务坐标系中裁剪策略目标，并将超限姿态投影到参考锥。在 SERL 模式下，先运行原有 XYZ/RPY 安全盒裁剪。可选的 `--orientation-xyzw X Y Z W --max-angle-rad A` 定义姿态参考和 SO(3) 角半径，避免欧拉角分支切点。

`workspace capture` 读取当前 TCP，报告基坐标系位姿和任务坐标系位置，不移动机器人。采集点是测量值，不是已验收边界。`workspace validate --pose X Y Z QX QY QZ QW` 根据已配置工作区和基坐标系安全盒检查基坐标系目标。越界验证返回 `valid: false` 及违规详情：JSON 模式退出码为 `0`，文本模式为 `5`。格式错误的配置或输入仍报告错误。

`workspace show` 包含米/弧度/xyzw 单位、工作站和标定来源、基坐标系安全盒，以及有向工作区在自身坐标系和基坐标系中的八个顶点。它还报告 `joint_limits`、`max_delta_per_step`、`home_joints` 和 `home_within_limits`（未配置 home 或限位时为 `null`；`home_violations` 列出每个超出限位的关节）。两种约束取交集。顶点用于检查，不会被更大的轴对齐包围盒替代，也不能证明空间无碰撞。此命令显示配置，不代表实时 daemon 约束执行或物理标定验收。

在机器人主机的仓库目录激活项目环境后，按以下方式保存具名测量点并预览其极值（工作站必须已定义 `table` 坐标系，例如用上文的 `workspace frame set`）：

```bash
rwrl --station local/station.yaml workspace capture --name near --output points.yaml --save
rwrl --station local/station.yaml workspace capture --name far --output points.yaml --save
rwrl --station local/station.yaml workspace set --frame table --from-points points.yaml
```

仅在操作员已将机械臂安全摆放后采点；这些命令不会移动机械臂。点文件路径相对于工作站目录。测量保存基坐标系位姿、时间戳、米制单位和完整坐标变换。复用名称需进行相同的保存审阅，并将旧文件保留在备份中。坐标系变化后必须重采。点必须覆盖三个轴的范围，不会凭空增加边距。仅凭极值不能证明包围体积安全。审阅候选配置后，再用 `--save` 重复 `workspace set`；数值 `--low/--high` 不能与 `--from-points` 混用。

<a id="supervised-recording-sessions"></a>

### 受监督的录制会话

`rwrl teleop run` 在 `robot_infra.supervise` 下使用现有示范录制器。提供已选择 SpaceMouse 标定档案的工作站，以及含 home 和工作区设置的任务预设。reset 可能移动机械臂和操作夹爪；命令先预览会话，并使用正常运动确认。

```bash
rwrl --station local/station.yaml --preset local/task.yaml teleop run \
  --mode tracker --record local/demo.pkl
```

使用 `--mode serl` 选择原生 SERL 接管语义；其预设必须提供所需的位置和 RPY 限制。`--profile NAME` 覆盖工作站默认档案。`--num-successes` 默认为 20；`--max-episodes` 可选。每个回合可包含多次人工/策略交接。模式的接管行为保持不变。

回合结束后，会话保持设备最后接受的目标并忽略手柄运动。`n` 请求下一回合，包括任务 reset；六个输入轴必须在所选档案的回中阈值内（默认 0.05），且按钮释放，持续 0.2 秒。`q` 释放控制并退出。达到录制预算后禁用 `n`，但保持状态持续到 `q`。故障和信号仍触发清理。保持状态不会向已完成轨迹添加转移。

Python 录制器默认保留现有的自动回合行为。传入 `--episode-end-behavior hold` 可选择相同的会话行为，也可显式指定 `--neutral-threshold` 和 `--neutral-stable-s`。目前要求 `real_robot` 笛卡尔环境、SpaceMouse 和独立事件设备。其它配置在 reset 前被拒绝。

私密的同级 `.session-<id>` 目录保存实际生效配置和会话结果。使用全局 `--json` 时，子进程输出转到 stderr。实际正常停止稳定性仍需独立的物理验证；进程成功退出不是实测的停止验收结果。

<a id="live-input-calibration-and-diagnosis"></a>

### 实时输入标定与诊断

省略 `--samples` 可运行交互式标定向导。它只打开 SpaceMouse，要求按任务坐标系进行带正负方向的平移和旋转，默认每个标签记录三秒（`--duration-s`）。提示描述期望 TCP 方向；操作员从当前位置选择自然的推动方向。旋转正负遵循右手定则。此向导不会连接机器人。

```bash
rwrl --station local/station.yaml teleop calibrate \
  --profile rear --frame table --output profiles.yaml --use
rwrl --station local/station.yaml --json teleop diagnose \
  --duration-s 5 --profile rear
```

`--use` 保存候选档案，并在审阅后在工作站中选用它。不带 `--save` 或 `--use` 时，标定只产生候选结果。如果保存档案后取消保存工作站，已保存档案仍可用，但不会自动被选中。

输入诊断记录解码轴、固定轴映射、任务/基坐标系输入、按钮、报告计数器和时间戳。仅输入诊断不会命令机械臂。报告位于可写源码检出目录的 `local/teleop/` 或用户状态目录；`--report-dir` 覆盖父目录。每次运行都有私密目录，包含 JSONL 样本、带标签 JSON 样本和摘要。采集失败保留错误报告。缺少旋转标签时报告未验证，不视为测试通过。

实时交互采集需要终端。agent 可用 `--samples` 拟合同一带标签 JSON 格式；`--yes` 不会生成用户方向标签。受支持的 SpaceMouse 读取器使用设备路径的协作式占用声明，因此第一个读取器关闭前，第二个项目读取器会被拒绝。诊断也检查现有硬件持有进程，不会接管其设备。

保持会话中的每次 reset 后都有回中检查：机械臂保持 reset 后实测位姿，直到手柄和按钮保持中立达到 `neutral_stable_s`（默认 0.2 秒）。检查期间可用 `q` 退出。同一轨迹内在策略和操作员之间交接控制时不运行此检查。

由 agent 驱动事件输入时，为 `rwrl teleop run` 添加 `--events-file local/events.jsonl`，或用 `--events-file -` 接收 stdin 管道。启动前创建文件，每行追加一个 JSON 对象，例如 `{"event":"takeover"}`。支持事件为 `takeover`、`handback`、`success`、`failure`、`rerecord`、`end`、`next` 和 `quit`。tracker 使用 takeover/handback；SERL 保留基于运动的介入。该输入替代终端按键。等待 `EPISODE_HOLD` 后再发送 `next`；reset 可能移动机器人或回 home。事件按顺序消费，每次控制轮询至多一个。普通文件以 tail 方式读取；在 `quit` 前关闭管道属于故障。格式错误或未知事件会失败，不会静默继续。直接 Python/训练入口使用 `--teleop-event-device jsonl --teleop-events-file <path-or-dash>`。

无需连接硬件或加载工作站即可评估已保存的正常停止证据。在任意机器的仓库目录，激活项目环境后运行：

```bash
rwrl --json teleop inspect-stop --trace local/teleop/stop-trace.json
```

schema-1 JSON 停止轨迹包含主机单调时钟上的 `stop_start_s`、`stop_end_s` 和 `samples` 列表。每个样本提供 `monotonic_s`、`device_time_s`（秒，递增的 SDK 时间）、`tcp_pose`（基坐标系 XYZ，米，及 xyzw 四元数）、`joint_vel`（rad/s，Franka 全部七个关节）、`robot_mode`（SDK 名称）和 `errors`（列表）。停止开始边界记录 `control_mode` 和 `tracker_active`。已空闲/不活动的机械臂为 BLOCKED，原因 `no active motion at stop`；仅静止样本不能证明活动控制下的停止。机器人模式未知/缺失会阻塞验收，故障模式会导致失败。`inspect-stop` 对 PASS 退出 `0`、FAIL 退出 `3`、BLOCKED 退出 `4`；一般错误保留原有错误码。`joint_count` 可声明其它机械臂的关节数。按工作站配置 schema 包含已确认的 `workspace` 和可选 `task_frame`；缺少工作区证据不能通过。停止失败时保留 `stop_error`。

评估包含基坐标系 XYZ 位移的有符号最小值、最大值和最终值。停止轨迹提供 `task_frame` 时，还报告任务坐标系分量、最大向下/向上 Z 位移和最大水平位移。这些值以最后一个停止前样本为参考，包含中间峰值，而不只是最终位姿。只有操作员已验证坐标系标定时，任务 Z 才代表物理竖直方向。没有任务坐标系时，明确报告这些指标不可用；不假定基坐标系 Z 为竖直方向。分量指标不替代完整 3D 验收阈值。

默认标准为采样到的最大 3D 位移 10 mm、最大 SO(3) 旋转 5 度、最终 0.5 秒窗口内全部关节速度不超过 0.02 rad/s、停止完成后至少 2 秒观测，以及样本间隔至多 0.1 秒。停止前 0.5 秒静止窗口用于建立参考。用 `--thresholds` 提供 JSON 对象，可覆盖 `displacement_m`、`angle_deg`、`joint_speed_rad_s`、`stationary_s`、`post_stop_s` 或 `max_sample_gap_s`；报告包含所有实际生效阈值。

已知违规产生 `FAIL`；证据不足或陈旧产生 `BLOCKED`。`PASS` 退出码为 0，`FAIL` 为 3，`BLOCKED` 为 4，输入无效为 2。结果包含输入文件 SHA-256。评估不平滑或修改样本。`PASS` 仅适用于提供的样本，不能确定连续时间最大值。仅两个端点快照不够。此命令评估证据，不采集停止轨迹，也不证明物理停止问题已修复。

HIL-SERL、residual HIL-SERL 和 SiLRI 训练 actor 也可通过 `--episode-end-behavior hold`、`--neutral-threshold 0.05` 和 `--neutral-stable-s 0.2` 启用此行为。默认仍为 `auto`。要求与录制相同的笛卡尔真机后端和 SpaceMouse 组合事件源。每个回合在保持前封存；保持不添加转移或策略步。显式 `next` 允许 reset，随后再次检查回中输入。FWBW 的免 reset 回合处理不同，因此拒绝此选项。Python 用户可在 `actor.run()` 前调用 `actor.configure_episode_session("hold", ...)`。有限步数预算耗尽后禁用 next 并保持到 quit，即使最后一步没有终止回合也是如此。

实时标定记录 USB 厂商/产品标识，并在设备提供序列号时记录序列号。tracker 和 SERL 输入源都会在打开输入时、回合 reset 前检查此身份。不匹配会拒绝操作，并释放输入源自己持有的输入；调用方持有的输入不会断开。如果设备没有序列号，厂商/产品 ID 匹配只能确定型号，不能确定单台设备。替换此类设备后重新验证方向映射。没有设备身份的离线标定档案保留显式未绑定行为；不会根据设备索引猜测身份。

配置任务几何后，它成为实验契约的一部分。存在工作区时，记录其数值变换、有向边界、可选 SO(3) 姿态限制和反馈裕量。单独的任务坐标系仅用于显示/采样，不增加空间语义。坐标系标签和溯源文本不改变数值契约。没有工作区的旧配置保留原契约。部署从训练配置恢复任务坐标系和工作区，并拒绝冲突的 CLI 几何设置。

远程训练时，提供与 daemon 工作站几何一致的 `real_robot.workspace` 和 `real_robot.task_frame`。`rwrl teleop run` 从所选工作站填入这些设置。reset 前，环境要求 daemon 声明 `task_workspace_v1`，并检查实际执行的几何是否与预期工作区一致。缺少能力或几何不匹配时拒绝 reset；客户端不会静默替换成更宽松的盒。本地机器人字典直接提供预期几何。现有基坐标系安全限制也继续适用。

对于新的任务工作区，环境在 reset 运动前、每个笛卡尔步前、获取观测后及会话保持期间检查实测 TCP 反馈。逐步检查复用最后一次原始观测，不额外发出机械臂状态 RPC。反馈检查允许 `measured_margin_m`（默认 `0.003 m`）和 `measured_margin_rad`（默认 `0.03 rad`）；目标检查保留 `1e-9` 容差。实测越界超过这些裕量会锁存故障并尝试停止。该环境实例拒绝后续步进和自动 reset；开始新会话前应检查机器人及反馈。它不会将 TCP 驱回盒内。关节动作环境拒绝任务工作区，因为这些边界不能保证关节轨迹的 TCP 路径被包含其中。这些采样 TCP 检查不是全臂碰撞检测。未配置任务工作区时，现有基坐标系安全盒裁剪语义不变。

`rwrl teleop run` 通过仅运行时注入的 `RWRL_STOP_TRACE_PATH` 启用正常停止证据。`RWRL_DIAGNOSTICS_PATH` 和 `RWRL_SESSION_EVENTS_PATH` 选择其它运行时证据位置；默认全部关闭。这些路径不在 `RealRobotEnvConfig` 或持久化配置中。Python 调用方将独立的 `RuntimeReportPaths(stop_trace=..., diagnostics=..., session_events=...)` 传给 `RealRobotEnv(..., report_paths=...)`。省略 `report_paths` 时，环境通过 `RuntimeReportPaths.from_environ()` 读取这些环境变量。后端必须在 reset 前声明 `stop_trace_v1`（目前为 Franky 和更新后的 Franka daemon）。控制期间利用现有连接采样，并保留滚动的停止前窗口。显式正常退出会采集停止过程，并在释放连接前观测 2.1 秒。`stop-trace-<timestamp>-<id>.json` 及对应 `.assessment.json` 写入会话报告目录；`summary.json` 包含评估。已知验收失败会使子进程在导出证据后以失败状态退出；证据不足仍为 BLOCKED。

CLI 会话还写入私密的 `session-events-<timestamp>-<id>.jsonl`，独立于轨迹转移。每个 schema-1 行包含序号、单调时间、回合索引、阶段和原因。阶段为 `READY`（等待 reset/回中）、`ACTIVE`、`EPISODE_HOLD`（保持状态）、`STOPPING`、`CLOSED` 和 `FAULT`。`CLOSED` 表示清理成功返回，不表示硬件停止验收通过。进程中断可能使最后阶段未完成；缺少 `CLOSED` 不能被解释为已释放。Python 录制器/actor 用户可在运行时注入 `RuntimeReportPaths(session_events=...)`。文件名在运行目录内唯一，再次调用环境 `close()` 不会重复导出。清理记录当前异常类型；已有的 `operator_quit` STOPPING 行不会重复。

故障和中断立即清理，跳过正常观测等待。daemon 停止观测期间允许 stop/read 调用，拒绝新的控制写入。现有 SDK 停止控制律不变。报告的质量/质心和 SDK/服务端元数据是观测值，明确不是已验证的负载标定。停止元数据还为 SDK 外观接口、实现和父类模块文件生成 SHA-256 指纹（包括 Python 包装后的原生模块）。缺失文件记为不可用。这是磁盘指纹，不证明已加载内存或链接的系统库。报告的模型证据包含可用的质量、质心、惯量、法兰到末端及末端到刚度坐标系变换；缺少的 SDK 字段保持 null。环境导出的停止轨迹包含 `metadata.session_exit`：已知退出原因、回合索引和环境配置 SHA-256。在活动轨迹或保持期间显式 q 记录为 `operator_quit`；其它正常关闭仍为 `normal_close`。上下文本身不证明停止成功。Python 调用方可在同一已连接 Franky/远程机械臂上使用 `begin_stop_trace()`，再调用 `stop_with_trace()`；停止抛出异常时 `last_stop_trace` 仍可用。带轨迹停止成功后，后端在停止后观测前已为 IDLE，因此 daemon 动作截止时间过期不会再触发停止。观测期间拒绝新运动或进入伺服模式；普通 stop 仍可用。之后恢复需要显式选择控制模式。停止失败不会错误地将后端标为空闲，也不会禁用 watchdog 的故障回退。

调试机器人运动时，在 NUC 仓库激活项目环境，使用已审阅的工作站工作区和任务预设：

```bash
rwrl --station local/station.yaml --preset local/task.yaml teleop run \
  --diagnose translation --duration-s 10 --record local/diag-translation.pkl
```

选择 `translation`、`rotation` 或 `all`；`--mode tracker|serl` 选择原控制模式。`--gain` 默认为 0.2，相对于预设控制设置缩放输入，不是绝对速度保证。正常 `teleop run` 默认保留增益 1 和全部六轴。诊断过滤和增益进入实验契约，以区分这些数据和正常数据。`teleop diagnose` 仅检查输入。机器人运动诊断使用 `teleop run --diagnose translation|rotation|all`；`--gain`（默认 0.2）和 `--duration-s`（默认 5）仅影响诊断会话。未指定 `--diagnose` 时，这两个选项均被忽略。诊断会话默认一次成功且最多一个回合；正常会话保留 20 次成功且不限制回合数。

运动诊断与正常会话使用相同的 supervisor、确认、录制器、回中检查和回合保持。时长确定名义控制步数预算，不是硬性墙钟截止时间。达到预算后回合保持至 `q`，不会静默释放控制。`--events-file` 也适用于 agent 事件。

会话 `diagnostics-<timestamp>-<id>.jsonl` 记录环境输入动作、请求位姿、环境限制后位姿、设备接受位姿及实测基/任务坐标系位姿，并带单调时间戳。原因区分环境位姿限制和设备限制。这里的输入动作是归一化控制动作，不是原始 HID 报告；解码 HID 证据使用仅输入诊断。日志输出路径不改变实验语义。只有启用诊断时才执行额外的远程已接受目标查询。

诊断行区分 `kind: step` 和 `kind: hold`。保持行不改变回合步数计数器，动作/输入字段为 null；它们不是轨迹转移。保持的 `measurement_phase` 为 `before_command`（刷新目标前检查的反馈），正常步为 `after_command`。保持日志复用验证保持所需的已接受目标读取，不另发目标查询。日志写入失败会传播到清理；缺失行不能被当作成功保持的证据。

发送失败且日志仍可用时，记录 `kind: command_failed`、尝试发送的目标及错误。执行状态为 `unknown`；已接受目标和实测位姿为 null，因为传输错误不能证明设备是否执行命令。此错误路径不额外查询设备。如果证据写入也失败，仍抛出原始设备错误。日志同步写入/刷新，因此应在实际运行文件系统和传输链路上验证延迟；它不是实时回调，也不会延长 watchdog 截止时间。

逐步 `teleop` 字段将消费的 SpaceMouse 样本与对应控制行关联：样本序号/时间、按声明顺序排列的解码 HID 轴、可用时的 HID 报告计数/时间、映射后的设备输入，以及所选旋转/过滤后的基坐标系输入。它还记录该样本是否介入。元数据随该样本一起复制，不从后续异步读取取得。读取器证据缺失时保持 null。`reset_index` 区分步数计数重新开始的各次 reset。没有遥操作样本的纯策略行将 `teleop` 留为 null。
