<a id="spacemouse-commissioning-profiles-task-frame-workspace-sessions"></a>

# SpaceMouse 调试：标定档案、任务坐标系工作空间与会话

前置条件见 `rwrl` [全局选项](cli.md#global-options)和[连接所有权](cli.md#connection-ownership)。

<a id="site-device-table"></a>

## 站点设备表

HID 读取器用精确 vendor/product ID（`vid:pid`）对应条目解码 SpaceMouse。内置表来自 PySpaceMouse 1.0.3，包含 SpaceExplorer、SpaceNavigator、SpaceMouse USB、SpaceMouse Compact、SpaceMouse Pro Wireless、SpaceMouse Pro、SpaceMouse Wireless、3Dconnexion Universal Receiver、SpacePilot、SpacePilot Pro。站点使用的其他型号或固件变体应写入站点本地 `spacemouse-devices.yaml`，不纳入 git。当 `rwrl discover` 或 `rwrl doctor` 输出：

```text
unknown 3Dconnexion-vendor HID device 256f:xxxx; add it to spacemouse-devices.yaml (see docs/spacemouse-commissioning.md#site-device-table)
```

表示设备不在表内，`open()` 将拒绝。警告覆盖所有未列出的 3Dconnexion（`256f`）设备；Logitech（`046d`）仅在 HID 描述符声明多轴控制器时报告，因此键盘/鼠标接收器不会被报告。

**位置。** 使用第一条适用规则选文件：

1. 工作站机器人 `teleop.options.device_table`。相对路径以工作站文件目录为基准。
2. 工作站文件旁的 `spacemouse-devices.yaml`：`local/station.yaml` 对应 `local/spacemouse-devices.yaml`，用户级工作站对应 `~/.config/rwrl/spacemouse-devices.yaml`（0.1.0 前的 `~/.config/robot_infra/` 仍在最后读取）。
3. 无工作站时（setup 探测、选项未指定表的设备，以及 doctor 找不到工作站时的主机 USB 检查/udev 规则），取 `./local/spacemouse-devices.yaml` 和 `$XDG_CONFIG_HOME/rwrl/spacemouse-devices.yaml` 中首个存在者（最后再读 0.1.0 前的 `$XDG_CONFIG_HOME/robot_infra/`）。

都不存在则只使用内置表。本地条目与内置条目 `hid_id` 相同时替换，否则新增。

**格式。** 下例为 `256f:c63a` 声明内置 SpaceMouse Wireless 解码（仅演示格式，不代表该产品实际行为；须按下文核对设备解码）：

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

`layout:` 只能引用内置条目，不能引用本地条目；旁边的字段覆盖复制值。无 `layout:` 时需完整指定解码：

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

构建 SpaceMouse teleop 及运行 `rwrl discover`、`rwrl doctor` 时加载并验证文件；任一无效条目会拒绝整个文件，错误注明文件和条目。顶层只能有 `schema_version: 1` 与非空 `devices` 映射；条目键为 `hid_id`、`layout`、`led_id`、`mappings`、`button_mapping`、`axis_scale`、`evidence`。`hid_id` 为两个 0..0xFFFF 整数（或文本 `"vvvv:pppp"`），文件内唯一；本地名称仅在 `hid_id` 相同时可复用内置名称。轴 `channel` 为 0..255，`byte1`/`byte2` 为 1..63，`scale` 为 -1 或 1；按钮包含整数 `channel`、`byte`（1..63）、`bit`（0..7），或三者均为 `null`；`axis_scale` 须为正有限值。

**证据。** 将条目依据（HID 报告描述符、各轴和按钮的短采集、日期）保存在 `local/` 中设备表旁，以相对设备表目录的 `evidence` 指向它。`rwrl teleop diagnose` 在汇总记录所用解码条目（`device_spec`：名称、`hid_id`、`source` 为 `builtin` 或 `local`，本地条目还带设备表 sha256）。录制与训练不将设备表纳入实验比较约定：`teleop_input` 仅把本地表文件、sha256 和条目作为来源信息记录，机器路径不进入 `source_options`。

**探测未知设备。** `rwrl teleop probe-hid` 为新条目采集证据并生成草稿。在机器人主机仓库根目录、项目环境已激活（`spacemouse` 扩展提供 hidapi），且无其他会话占用 SpaceMouse 时运行：

```bash
rwrl --station local/station.yaml teleop probe-hid --device 256f:c63a --duration-s 10
```

采集时沿各轴推动、旋转操纵帽并按每个按钮。Probe 读取 HID 报告描述符（优先 hidapi，否则 sysfs）和原始输入报告，不向设备发送任何内容，并新建 `local/teleop/probes/256f-c63a/`（工作站文件旁；可用 `--output DIR` 替代，无工作站时必须指定），包含 `descriptor.hex`、`reports.jsonl`（每报告一个 `{t, report_id, length, bytes_hex}`）和 `candidate.yaml`。无 `--device` 时探测唯一的 3Dconnexion 或 Logitech 多轴设备；有多个时列出候选。绝不读取不是多轴控制器的 Logitech 接口。已有探测目录会被拒绝：需删除或改用其他 `--output`。不记录序列号，也不写设备表。`--json` 输出相同证据及草稿条目。

`candidate.yaml` 是含单个草稿条目的完整设备表，`evidence` 指向探测目录：

1. 带 `layout:` 时，某内置条目能解码相同的 report ID 和长度，轴符号尚未检查。
2. 无 `layout:` 时，没有合适的内置条目；根据描述符尽可能填入完整解码，每个 `TODO`（各轴 `scale`、描述符未展示的按钮）均须替换。含 `TODO` 的条目会拒绝整个表。
3. 将条目复制到 `local/spacemouse-devices.yaml` 的 `devices:` 下（无表时可直接从 `candidate.yaml` 开始），然后运行下方检查；`teleop diagnose` 显示每轴符号是否符合预期。

**检查。** 编辑表后，在机器人主机仓库根目录、项目环境已激活时运行：

```bash
rwrl --station local/station.yaml discover
rwrl --station local/station.yaml doctor
rwrl --station local/station.yaml --json teleop diagnose --duration-s 5
```

`discover` 在 `spacemouse:` 下列出设备且无警告，`doctor` 通过 `usb.spacemouse` 和遥操作检查，诊断显示每次推轴和按键的预期符号（见[实时输入标定与诊断](#live-input-calibration-and-diagnosis)）。

<a id="spacemouse-profiles-and-task-frame-workspace"></a>

## SpaceMouse 标定档案与任务坐标系工作空间

工作站机器人可声明 `task_frame`，包含 `name` 和合法的 4×4 `base_T_frame` 变换。`teleop_profiles` 指定档案 YAML 的 `file`（相对工作站文件）与 `default` 档案名。档案文件含 `schema_version: 1`、`profiles` 映射。每个档案声明 `frame`、`yaw_rad`、`axis_mapping_version: serl_hid_axes_v1`；`neutral_threshold` 默认 0.05。此启动容差独立于正常介入与运动死区。

可用 `rwrl workspace frame set` 写入 `task_frame`，无需手工编辑。在机器人主机仓库中激活项目环境，由操作员定位机械臂后采集三个 TCP 点（原点、+X 点、位于 X 轴 +Y 侧的点），再生成并保存坐标系：

```bash
rwrl --station local/station.yaml workspace capture --name frame-origin --output frame-points.yaml --save
rwrl --station local/station.yaml workspace capture --name frame-x --output frame-points.yaml --save
rwrl --station local/station.yaml workspace capture --name frame-y --output frame-points.yaml --save
rwrl --station local/station.yaml workspace frame set --name table \
  --from-points frame-points.yaml --origin frame-origin --x-axis frame-x --y-axis frame-y
rwrl --station local/station.yaml workspace frame show
```

坐标系为右手系：X 从原点指向 X 点，Z 为三点平面的法向，Y 为将 Y 点方向对 X 正交化后的方向。候选报告两条实测轴长度，以及实测 Y 与最终 Y 的夹角；X 点距原点不足 1 mm 或三点共线会拒绝。也可用 `--base-T-frame` 后跟 16 个行优先数值（米）提供已知变换。默认 `source` 来源为 `points:<file> origin=A x=B y=C`，`--source TEXT` 可另设。加 `--save`，在审阅差异并确认后仅写入 `task_frame`。工作站 workspace 若定义在其他坐标系会被拒绝；复用其坐标系名称会重新定义 workspace 基准，须重跑 `workspace validate` 并审阅边界。旧坐标系保存的点文件须重新采集。`workspace frame show` 输出 `task_frame` 及其在基坐标系中的原点与单位轴。

`teleop_profiles` 是 real_world_rl 工作站段，不是 `RobotConfig` 字段。`real_world_rl.discovery.autoconfig` 的设备层加载器在未传 sections 时严格解析；遇到未知 `teleop_profiles` 键会报错并指向 `real_world_rl.station.load_station`。所有 `rwrl` 命令（包括 daemon、setup、discover、doctor）与 Python 调用者均使用 `real_world_rl.station.load_station`（或 `load_station_with_env`）注册验证器。现有工作站 YAML 无需迁移；`rwrl teleop calibrate --use` 继续在机器人映射中写入此段。

在已安装的项目环境中，准备工作站文件后运行：

```bash
rwrl --station local/station.yaml --json teleop profiles list
rwrl --station local/station.yaml teleop profiles show --profile rear
rwrl --station local/station.yaml teleop profiles use --profile rear
rwrl --station local/station.yaml workspace show
```

离线标定接受 JSON 映射，将 `+x`、`+y`、`-x`、`-y` 对应到**固定 HID 轴映射之后**的归一化六轴样本列表。正向标签拟合 yaw，负向标签独立验证。最大方向残差默认 25 度（`--max-error-deg`）。结果不表示已验证旋转方向。

```bash
rwrl --station local/station.yaml --json teleop calibrate \
  --samples local/direction-samples.json --profile rear --frame table
```

加 `--save --output profiles.yaml` 可确认后保存，输出路径相对工作站文件。结果包含应配置到机器人 `teleop_profiles` 的 `station_reference`。保存时以唯一 `<file>.<id>.bak`、权限 `0600` 备份旧文件，保留编辑档案之外的 YAML 注释、引号与键顺序。确认前 stderr 显示与实际读取文件的 unified diff。审阅或暂存期间检测到变化会拒绝保存。这是乐观冲突检测，不是外部编辑锁；保存期间避免编辑同一文件。非交互保存需全局 `--yes`。

仅重采一个方向时用 `teleop calibrate --profile rear --samples local/direction-samples.json --capture-label=+x`。这会替换该方向而保留其他组；`--append-trial` 则添加一次独立推动。每次推动拟合权重相同，不受时长/力度影响。JSON 每方向接受旧式样本行或 `{"trials": [rows, rows]}`。每个留出试验都须通过，包括带符号旋转试验。新报告含 `merged-samples.json`，下次采集用其路径作为 `--samples`。不覆盖原始样本文件。水平组不足时结果为 `NEEDS_SAMPLES`，包含 `required_fields`，不保存档案。无终端时，带标签采集需全局 `--yes`，立即采集 `--duration-s` 秒（默认 3）。提示和合并文件位置写入 stderr。混合/离线样本不能确立单一物理设备身份，只有完整实时采集才绑定身份。

原始输入 JSONL 区分新 HID 报告（`fresh_report`）和缓存状态轮询。仅新报告参与带标签拟合，每次采集至少须收到一份新报告。空 HID 读取保留状态，因此静默本身不解释为释放或断连。无报告采集以证据不可用失败，SDK 读取错误向外传播。此输入采集检查不证明连续设备连接，也不证明拆分 HID 报告中每轴均为最新。

输入采集和受监督 CLI 会话在打开输入或启动控制前保存私有 `provenance.json`，记录 `real_world_rl`、`rl_garden` 的 Python 源文件哈希、已安装发行包名称/版本和规范化配置摘要。会话还对工作站、预设、档案、解析后配置文件取哈希。路径不影响包源码摘要，内容修改和文件改名会影响。这是运行前磁盘证据，不是加载内存或全部依赖二进制的证明。运行期间保持源码/配置不变，并将这些记录与原始输入、轨迹、停止报告一同归档。

录制/训练时将工作站作为 `--teleop-options-yaml`，选择 `--teleop-device spacemouse`，可选覆盖 `--teleop-profile`、`--teleop-robot`。解析后的输入旋转、档案名/文件及 yaw 在保存配置前冻结，保存在不参与比较的 `teleop_input` 来源信息中。恢复运行复用这些数值选项；重标定操作员方向不会改变已记录基坐标动作含义。旧 `teleop.options.input_rotation` 不能与命名档案或工作站 `task_frame` 组合。应将安装方向部分迁到任务坐标系，选档案描述操作员 yaw。不含新字段的旧配置保留原输入旋转行为。

`workspace set --frame table --low X Y Z --high X Y Z` 预览任务坐标系位置框；`--save` 确认后应用。已有基坐标 `safety_box` 仍生效。设备层拒绝任务工作空间外目标，包括被基坐标裁剪移到其外的目标。训练环境先在任务坐标裁剪策略目标，将过大姿态投影到参考锥。SERL 模式先执行原 XYZ/RPY 安全框裁剪。可选 `--orientation-xyzw X Y Z W --max-angle-rad A` 定义姿态参考和 SO(3) 角半径，避免欧拉角分支切割。

`workspace capture` 读取当前 TCP，报告基坐标位姿和任务坐标位置，不移动机器人。采集点是测量，不代表已接受边界。`workspace validate --pose X Y Z QX QY QZ QW` 对照 workspace 和基坐标安全框验证基坐标目标。越界结果包含 `valid: false` 与违规细节：JSON 模式退出 `0`，文本模式退出 `5`。错误配置或输入仍报错。

`workspace show` 包含米/弧度/xyzw 单位、工作站和标定来源、基坐标安全框，以及有向工作空间八个顶点在自身和基坐标系中的位置。还报告 `joint_limits`、`max_delta_per_step`、`home_joints`、`home_within_limits`（未配置 home 或限制时为 `null`；`home_violations` 列出各越界关节）。两种约束取交集。顶点仅用于检查，不替换为更大的轴对齐包围框，也不证明空间无碰撞。此命令展示配置，不代表 daemon 实时执行或物理标定已验收。

在机器人主机仓库中激活项目环境，保存命名测量并预览其极值如下（工作站须已定义 `table`，如上文 `workspace frame set`）：

```bash
rwrl --station local/station.yaml workspace capture --name near --output points.yaml --save
rwrl --station local/station.yaml workspace capture --name far --output points.yaml --save
rwrl --station local/station.yaml workspace set --frame table --from-points points.yaml
```

每个点仅在操作员安全定位机械臂后采集；命令绝不移动机器人。点路径相对工作站目录。测量存储基坐标位姿、时间戳、米单位和完整坐标变换。复用名称需要同样的保存审阅，并备份旧文件。坐标系改变须重采。点须覆盖三轴，不自动添加边距。仅凭极值不能证明包围体积安全。重复 `workspace set` 并加 `--save` 前审阅候选；数值 `--low/--high` 不能与 `--from-points` 组合。

<a id="supervised-recording-sessions"></a>

### 受监督的录制会话

`rwrl teleop run` 在 `rwrl supervise` 下使用现有示范录制器。提供已选择 SpaceMouse 档案的工作站和包含 home/workspace 的任务预设。Reset 可移动机械臂、操作夹爪；命令预览会话并执行正常运动确认。

```bash
rwrl --station local/station.yaml --preset local/task.yaml teleop run \
  --mode tracker --record local/demo.pkl
```

`--mode serl` 使用原生 SERL 接管语义；预设须提供所需位置和 RPY 限制。`--profile NAME` 覆盖工作站默认。`--num-successes` 默认 20，`--max-episodes` 可选。每回合可包含多次人工/策略交接，模式的接管行为不变。

回合结束后，会话保持最后的设备已接受目标，忽略操纵帽运动。`n` 请求下一回合（包括任务 reset）；全部六轴须在所选档案中立阈值（默认 0.05）内且按钮松开，持续 0.2 秒。`q` 释放控制并退出。达到录制预算后禁用 `n`，但保持到 `q`。故障和信号仍执行清理。保持期间不向已完成轨迹添加 transition。

Python 录制器默认保留原自动回合行为。传 `--episode-end-behavior hold` 可启用相同会话行为，可显式设 `--neutral-threshold`、`--neutral-stable-s`。目前要求 `real_robot` 笛卡尔环境、SpaceMouse 和独立事件设备。其他配置在 reset 前拒绝。

私有同级 `.session-<id>` 目录保存生效配置与会话结果。全局 `--json` 下子进程输出到 stderr。实际正常停止稳定性仍需独立物理验证；进程成功退出不是实测停止验收结果。

<a id="live-input-calibration-and-diagnosis"></a>

### 实时输入标定与诊断

省略 `--samples` 运行交互标定向导。它只打开 SpaceMouse，要求任务坐标系带符号平移/旋转，默认每标签记录三秒（`--duration-s`）。提示描述预期 TCP 方向，操作员按当前位置选择自然推动方向。旋转符号遵循右手规则。此向导从不连接机器人。

```bash
rwrl --station local/station.yaml teleop calibrate \
  --profile rear --frame table --output profiles.yaml --use
rwrl --station local/station.yaml --json teleop diagnose \
  --duration-s 5 --profile rear
```

`--use` 保存候选，经审阅后在工作站选择它。无 `--save` 或 `--use` 时仅产生候选。若档案保存后取消工作站保存，档案仍可用，但不自动选择。

输入诊断记录解码轴、固定轴映射、任务/基坐标输入、按钮、报告计数和时间戳。仅输入诊断从不向机械臂发命令。报告保存在可写源码目录的 `local/teleop/` 或用户状态目录，`--report-dir` 覆盖父目录。每次运行有私有目录，包含 JSONL 样本、带标签 JSON 样本和汇总。采集失败保留错误报告。缺少旋转标签会报告未验证，不视为通过。

实时交互采集需要终端。代理可用 `--samples` 拟合同样的带标签 JSON；`--yes` 不会生成人工方向标签。受支持 SpaceMouse 读取器获取建议性设备路径 claim，第二个项目读取器会被拒绝，直到第一个关闭。诊断也检查现有硬件进程拥有者，不接管其设备。

保持会话的每次 reset 后都有中立门控：机械臂保持 reset 后实测位姿，直到操纵帽/按钮持续中立 `neutral_stable_s`（默认 0.2 秒）。门控期间 `q` 可退出。轨迹内策略与人工交接时不执行此门控。

代理驱动事件输入时，在 `rwrl teleop run` 加 `--events-file local/events.jsonl`，或 `--events-file -` 从 stdin 管道读取。启动前创建文件，每行追加一个 JSON 对象，如 `{"event":"takeover"}`。支持事件为 `takeover`、`handback`、`success`、`failure`、`rerecord`、`end`、`next`、`quit`。Tracker 使用 takeover/handback；SERL 保留基于运动的介入。此输入替代终端按键。发送 `next` 前等待 `EPISODE_HOLD`；reset 可移动/归位机器人。事件按顺序消费，每次控制轮询最多一个。普通文件以 tail 方式读取；`quit` 前关闭管道是故障。格式错误或未知事件会失败，不会静默继续。直接 Python/训练入口使用 `--teleop-event-device jsonl --teleop-events-file <path-or-dash>`。

已保存的正常停止证据可离线评估，无需连接硬件或加载工作站。在任意机器仓库中激活项目环境后运行：

```bash
rwrl --json teleop inspect-stop --trace local/teleop/stop-trace.json
```

schema-1 JSON 轨迹含主机单调时钟上的 `stop_start_s`、`stop_end_s` 和 `samples` 列表。每样本提供 `monotonic_s`、`device_time_s`（秒，递增 SDK 时间）、`tcp_pose`（基坐标 XYZ 米与 xyzw 四元数）、`joint_vel`（rad/s，全部七个 Franka 关节）、`robot_mode`（SDK 名称）、`errors`（列表）。停止起始边界记录 `control_mode`、`tracker_active`。已空闲/未激活的机械臂得到 BLOCKED，原因 `no active motion at stop`；仅静止样本不能证明主动控制停止。未知/缺失机器人模式阻止验收，故障模式导致失败。`inspect-stop` 的退出码为 PASS `0`、FAIL `3`、BLOCKED `4`；一般错误沿用既有退出码。`joint_count` 可声明其他机械臂关节数。按工作站配置 schema 提供已确认 `workspace` 和可选 `task_frame`；缺少 workspace 证据不能通过。停止失败时保留 `stop_error`。

评估包含基坐标 XYZ 位移带符号最小值、最大值和最终值。提供 `task_frame` 时，还报告任务坐标分量、最大向下/向上 Z 位移及最大水平位移。以停止前最后样本为参考，包含中间峰值，而非仅最终位姿。仅当操作员验证坐标标定后，任务 Z 才代表物理竖直方向。无任务坐标系时这些指标明确不可用，绝不假定基坐标 Z 为竖直方向。分量指标不替代完整 3D 验收阈值。

默认：最大采样 3D 位移 10 mm，最大 SO(3) 旋转 5 度，最终 0.5 秒窗口内全部关节速度不超过 0.02 rad/s，停止完成后至少观察 2 秒，样本间隔不超过 0.1 秒。停止前 0.5 秒静止窗口确立参考。可用 `--thresholds` JSON 对象覆盖 `displacement_m`、`angle_deg`、`joint_speed_rad_s`、`stationary_s`、`post_stop_s`、`max_sample_gap_s`；所有生效阈值均进入报告。

已知违规产生 `FAIL`，证据不足或陈旧产生 `BLOCKED`。退出码 PASS 为 0、FAIL 为 3、BLOCKED 为 4、输入无效为 2。结果含输入文件 SHA-256。评估不平滑或修改样本。`PASS` 仅适用于所给样本，不能确立连续时间最大值；仅两个端点快照不够。此命令评估证据，不采集轨迹，也不证明物理停止问题已修复。

HIL-SERL、残差 HIL-SERL、SiLRI 训练 actor 也可用 `--episode-end-behavior hold`、`--neutral-threshold 0.05`、`--neutral-stable-s 0.2` 启用，默认仍为 `auto`。要求与录制相同的笛卡尔真机后端和 SpaceMouse 复合事件源。每回合在保持前封存；保持不添加 transition 或策略步。显式 `next` 允许 reset，再检查中立。FWBW 因免 reset 回合处理不同而拒绝此选项。Python 用户可在 `actor.run()` 前调用 `actor.configure_episode_session("hold", ...)`。有限步数预算会禁用 next 并保持到 quit，即使最后一步未终止回合。

实时标定记录 USB vendor/product 标识，以及设备提供的序列号。Tracker 和 SERL 源在打开输入、回合 reset 前检查身份。不匹配会拒绝运行并释放源拥有的输入；不关闭调用者拥有的输入。无序列号时，vendor/product 匹配只能证明型号，不能证明个体。更换此类设备后需重验方向映射。无设备身份的离线档案仍明确不绑定身份，绝不从设备索引猜测身份。

配置任务几何后，它属于实验约定。存在 workspace 时记录数值变换、有向边界、可选 SO(3) 姿态限制及反馈裕量。单独 task frame 仅用于显示/采样，不增加空间语义。坐标名称与来源文本不改变数值约定。无 workspace 的旧配置保留原约定。Deploy 从训练配置恢复 task frame/workspace，拒绝冲突的 CLI 几何参数。

远程训练须提供与 daemon 工作站几何一致的 `real_robot.workspace`、`real_robot.task_frame`。`rwrl teleop run` 从所选工作站填入。Reset 前环境要求 daemon 声明 `task_workspace_v1`，并核对实际执行几何与预期 workspace。能力缺失或几何不匹配会拒绝 reset，客户端不会静默改用较弱安全框。本地机器人字典直接提供预期几何。已有基坐标安全限制仍生效。

对于新任务工作空间，环境在 reset 运动前、每个笛卡尔步前、取得观测后以及会话保持期间检查实测 TCP 反馈。逐步检查复用最后的原始观测，不额外发机械臂状态 RPC。反馈检查允许 `measured_margin_m`（默认 `0.003 m`）、`measured_margin_rad`（默认 `0.03 rad`）；目标检查保留 `1e-9` 容差。超出裕量的实测违规会锁存故障并尝试停止，拒绝此环境实例后续 step 和自动 reset；开始新会话前检查机器人和反馈。不会驱动 TCP 回到安全框。关节动作环境拒绝任务工作空间，因为这些边界无法证明关节轨迹的 TCP 路径始终在内。采样 TCP 检查不是整臂碰撞检查。未配置任务工作空间时，原基坐标框裁剪语义不变。

`rwrl teleop run` 通过仅运行时注入的 `RWRL_STOP_TRACE_PATH` 启用正常停止证据。`RWRL_DIAGNOSTICS_PATH`、`RWRL_SESSION_EVENTS_PATH` 指定其他运行时证据路径，默认均关闭。这些路径不在 `RealRobotEnvConfig` 或持久配置中。Python 调用者向 `RealRobotEnv(..., report_paths=...)` 单独传入 `RuntimeReportPaths(stop_trace=..., diagnostics=..., session_events=...)`。省略 `report_paths` 时，环境通过 `RuntimeReportPaths.from_environ()` 读取变量。后端须在 reset 前声明 `stop_trace_v1`（目前为 Franky 和更新后的 Franka daemon）。控制期间使用同一连接采样，保留滚动停止前窗口。显式正常退出会采集停止，观察 2.1 秒后释放连接。会话报告目录写入 `stop-trace-<timestamp>-<id>.json` 和配套 `.assessment.json`，`summary.json` 含评估。已知验收失败会在导出证据后令子进程失败退出；证据不足保留 BLOCKED。

CLI 会话还写入独立于轨迹 transition 的私有 `session-events-<timestamp>-<id>.jsonl`。每个 schema-1 行含序号、单调时间、回合索引、阶段、原因。阶段为 `READY`（等待 reset/中立）、`ACTIVE`、`EPISODE_HOLD`、`STOPPING`、`CLOSED`、`FAULT`。`CLOSED` 表示清理成功返回，不代表硬件停止验收通过。中断可能留下未结束阶段；不能从缺少 `CLOSED` 推断已释放。Python recorder/actor 用户可运行时注入 `RuntimeReportPaths(session_events=...)`。运行目录内文件名唯一，第二次环境 `close()` 不重复导出。清理记录当前异常类型；已有 `operator_quit` STOPPING 行不重复。

故障与中断立即清理，跳过正常观察等待。Daemon 停止观察允许 stop/read，拒绝新控制写入。原 SDK 停止规律不变。报告质量/质心及 SDK/服务器元数据是观测，不是已验证负载标定。停止元数据还以 SHA-256 指纹记录 SDK facade、实现及父类模块文件（含 Python 包装背后的原生模块）。缺失文件记录不可用。这是磁盘指纹，不证明已加载内存或链接系统库。模型证据包含可取得的质量、质心、惯量、法兰到末端和末端到刚度坐标系变换；SDK 缺失字段保留 null。环境导出轨迹包含 `metadata.session_exit`：已知退出原因、回合索引、环境配置 SHA-256。活动轨迹或保持中显式 q 记录 `operator_quit`，其他正常关闭为 `normal_close`。上下文本身不证明停止成功。Python 调用者可在同一已连接 Franky/remote arm 先 `begin_stop_trace()` 再 `stop_with_trace()`；停止抛错后仍可读 `last_stop_trace`。成功带轨迹停止后，后端在停止后观察前进入 IDLE，所以 daemon 动作期限到期不会再次停止。观察期间拒绝新运动或进入伺服模式，普通 stop 仍可用。之后恢复需显式选择控制模式。停止失败不会错误标记为空闲，也不会禁用 watchdog 故障回退。

调试机器人运动时，在 NUC 仓库激活项目环境，使用已审阅的工作站 workspace 和任务预设：

```bash
rwrl --station local/station.yaml --preset local/task.yaml teleop run \
  --diagnose translation --duration-s 10 --record local/diag-translation.pkl
```

选择 `translation`、`rotation` 或 `all`，`--mode tracker|serl` 选择原控制模式。`--gain` 默认 0.2，相对预设控制设置缩放输入，不保证绝对速度。普通 `teleop run` 默认 gain 1、全部六轴。诊断过滤和增益进入实验约定，以区分这些数据与正常数据。`teleop diagnose` 仅检查输入。运动诊断使用 `teleop run --diagnose translation|rotation|all`；`--gain`（默认 0.2）与 `--duration-s`（默认 5）仅影响诊断会话，无 `--diagnose` 则忽略两者。诊断默认一次成功、最多一回合；普通会话仍为 20 次成功、无回合数上限。

运动诊断与普通会话使用相同 supervisor、确认、录制器、中立门控与回合保持。时长决定名义控制步数预算，不是严格墙钟期限。预算到达后回合保持直到 `q`，不会静默释放控制。`--events-file` 同样支持代理事件。

会话 `diagnostics-<timestamp>-<id>.jsonl` 记录环境输入动作、请求位姿、环境限制后位姿、设备接受位姿、实测基/任务位姿及单调时间戳。原因区分环境位姿限制和设备限制。输入动作是归一化控制动作，不是原始 HID 报告；解码 HID 证据用仅输入诊断。日志路径不改变实验语义。只有启用诊断时才额外远程查询已接受目标。

诊断行区分 `kind: step` 与 `kind: hold`。Hold 行不改变回合步数，动作/输入字段为 null，不是轨迹 transition。Hold 的 `measurement_phase` 为 `before_command`（刷新目标前检查反馈），普通步为 `after_command`。Hold 日志复用验证保持已需要的目标读取，不另发查询。日志写入失败传播到清理；缺行不能当作成功保持证据。

发送失败且日志仍可用时，记录 `kind: command_failed`、尝试目标和错误。执行状态为 `unknown`，已接受目标和实测位姿为 null，因为传输错误不能证明设备是否执行。错误路径不增加设备查询。若证据写入也失败，仍抛出原设备错误。日志同步写入/flush，因此须在实际文件系统和传输上验证延迟；它不是实时回调，不延长 watchdog 期限。

逐步 `teleop` 字段将消费的 SpaceMouse 样本关联到控制行：样本序号/时间、按声明顺序解码的 HID 轴、可用时的 HID 报告数/时间、映射设备输入及选择旋转/过滤后的基坐标输入，同时记录该样本是否介入。元数据随样本复制，不从之后的异步读取取得。缺少读取器证据保持 null。`reset_index` 区分步数重启的 reset。无遥操作样本的纯策略行 `teleop` 为 null。
