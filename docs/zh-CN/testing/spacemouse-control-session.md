<!-- English source: docs/testing/spacemouse-control-session.md -->

<a id="spacemouse-tabletop-control-and-normal-stop-acceptance"></a>

# SpaceMouse 桌面控制与正常停止验收

在机器人 NUC 的仓库根目录，用 Bash 并激活已安装的项目环境运行（venv 安装使用 `source .venv/bin/activate`）。采集证据前，本清单为 **NOT_RUN**。它覆盖一台已调试、使用 `real_robot` 的 Franka；tracker 和 SERL 分别运行运动检查。不认证其它后端或无人值守机器人运行。

运动前，监督操作员必须确认机械臂身份、清空的工作区、物理停止装置、home/reset 路径、附件质量/质心、TCP 和安装设置。使用已审阅的 `local/station.yaml` 和 `local/task.yaml`；任何示例 IP、home、工作区或模型值都不是已验收物理标定。SERL 还需要现有 XYZ/RPY 安全盒。诊断失败时不要通过更改网卡路由、负载或固件走捷径。输入 yaw 标定不标定重力。

下文使用的 `table` 任务坐标系即工作站的 `task_frame`；`teleop run` 将其传给 `local/task.yaml` 会话，标定档案标定也需要它。在 A1 之前由三个采集的 TCP 点创建：操作员将机械臂摆到坐标系原点、+X 上一点和 +Y 一侧的一点（这些命令只读取 TCP，从不移动机械臂）。`workspace frame set` 预览坐标系、其轴长以及实测与最终 Y 轴之间的夹角；`--save` 在审阅差异后只写入 `task_frame`：

```bash
rwrl --station local/station.yaml workspace capture --name frame-origin --output frame-points.yaml --save
rwrl --station local/station.yaml workspace capture --name frame-x --output frame-points.yaml --save
rwrl --station local/station.yaml workspace capture --name frame-y --output frame-points.yaml --save
rwrl --station local/station.yaml workspace frame set --name table --from-points frame-points.yaml --origin frame-origin --x-axis frame-x --y-axis frame-y --save
rwrl --station local/station.yaml workspace frame show
```

点文件路径相对于工作站目录。每次坐标系标定使用新的点文件：点文件记录其采集时所在的坐标系，之后写入它的采集必须与该坐标系一致。若 `local/task.yaml` 带有自己的机器人副本，`rwrl --station local/station.yaml --preset local/task.yaml doctor` 会对每个与工作站不同的字段（包括 `task_frame`）发出警告。

准备新的证据目录并复制记录模板。以下准备仅为 shell 记录管理，不是设备检查：

<!-- tutorial-test: skip -->
```bash
RUN="local/spacemouse-acceptance-$(date -u +%Y%m%dT%H%M%SZ)"
mkdir "$RUN"
cp docs/testing/spacemouse-control-session-record.csv "$RUN/record.csv"
git rev-parse HEAD > "$RUN/commit.txt"
git diff --binary > "$RUN/working-tree.patch"
cp local/station.yaml local/task.yaml "$RUN/"
python -m pip freeze > "$RUN/dependencies.txt"
```

将原始标定档案（profile）及所有未跟踪的源码修改与此记录一起保存；仅提交号和已跟踪补丁不能标识未提交的安装状态。每次 CLI 输入采集创建唯一报告目录。录制产生归档及同级 `*.session-*` 目录。写入进程关闭后，将这些目录复制到 RUN，保留存在的 SQLite WAL/SHM 文件。将路径记入 CSV。运动项目需录制带时间戳的机械臂及操作员输入视频。任务成功标签不是硬件 PASS。

<a id="a-input-only-commissioning"></a>

## A：仅输入调试

A1 只打开 SpaceMouse。站在机械臂后方定义固定桌面坐标：+X 向前、+Y 向左、+Z 向上。做带标签的自然推动时，站在实际操作位置。包含 +X/+Y、独立的 -X/-Y、两个 Z 方向和全部三轴的带正负方向旋转。在前方操作位置以及至少一种非直角设备摆放下重复，保存独立标定档案。

```bash
rwrl --station local/station.yaml teleop calibrate --profile rear --frame table --save --output profiles.yaml --use
rwrl --station local/station.yaml teleop diagnose --profile rear --duration-s 5
```

全部带标签方向符合意图、残差满足所选阈值（默认 25 度），且保留原始证据时 A1 通过。允许自然跨轴输入；方向和可用性比精确端点定位更重要。符号错误、反射映射或证据不足需要重采，而不是再次猜测安装 roll 补偿。单组重采见 [CLI 标定](../spacemouse-commissioning.md#spacemouse-profiles-and-task-frame-workspace)。

<a id="b-static-feedback-and-workspace"></a>

## B：静态反馈与工作区

B1 检查主机和已配置设备健康。B2 只读采集 TCP 样本并审阅当前约束，不驱动至边界。

```bash
rwrl --station local/station.yaml doctor
rwrl --station local/station.yaml workspace show
rwrl --station local/station.yaml workspace frame show
rwrl --station local/station.yaml workspace capture --name reference --output workspace-points.yaml --save
```

B1/B2 仅在具备独立核验的机械臂身份、新鲜反馈、正确坐标系原点/方向（`workspace frame show` 给出基坐标系中的原点和单位轴；与桌面以及创建坐标系所用的 `frame-points.yaml` 点比对）、home 位于关节限位内（`workspace show` 中的 `home_within_limits`）、已审阅的基坐标系/任务坐标系（task frame）约束交集及实际安装/负载/TCP 证据时通过。仅 doctor 或 SDK 报告的质量不能验证物理模型。缺少测量为 BLOCKED。具名采集点可用于提出边界，但不能证明整个包围体积安全。

<a id="c-supervised-direction-checks"></a>

## C：受监督的方向检查

C1 平移、C2 旋转、C3 全轴使用低诊断增益。每条命令都在 `robot_infra.supervise` 下启动录制器；不要再套第二层 supervisor。reset 可能使机械臂回 home。达到名义步数预算后保持到 q；`--duration-s` 不是墙钟终止截止时间。

```bash
rwrl --station local/station.yaml --preset local/task.yaml teleop run --mode tracker --diagnose translation --gain 0.2 --duration-s 10 --record local/check-translation.pkl
rwrl --station local/station.yaml --preset local/task.yaml teleop run --mode tracker --diagnose rotation --gain 0.2 --duration-s 10 --record local/check-rotation.pkl
rwrl --station local/station.yaml --preset local/task.yaml teleop run --mode tracker --diagnose all --gain 0.2 --duration-s 10 --record local/check-all.pkl
```

使用 `--mode serl` 和新输出路径重复。PASS 要求正负方向符合意图、不同 TCP 姿态下仍沿固定桌面轴、组合控制可用，且无实测工作区越界或故障。比较 `diagnostics-<timestamp>-<id>.jsonl` 中的原始输入、请求/限制后/已接受目标及实测基/任务坐标系位姿；裁剪与映射错误分开报告。轻微手部耦合和跟踪滞后本身不导致方向验收失败。控制不符合预期时停止，不要仅为填完表格行而继续。

<a id="d-trajectory-ownership-and-end-holding"></a>

## D：轨迹控制归属与结束保持

```bash
rwrl --station local/station.yaml --preset local/task.yaml teleop run --mode tracker --record local/check-session.pkl --num-successes 2 --max-episodes 3
```

D1：在同一轨迹内至少两次执行人工 → 交还 → 人工 → 交还。tracker 用按钮 0；SERL 用运动/按钮，空闲时返回。录制交还使用其保持策略。验证**已学习策略**恢复时，使用已审阅的[训练流程](../tutorial/07-hil-training.md)和 `episode_end_behavior: hold` 重复 D1；仅录制器运行不能通过此子项。记录策略版本、人工掩码及同一回合中至少两次控制权转换；交还时不得发生 reset 或回中检查。

D2：标记成功/失败，再移动手柄。最后接受的目标必须保持固定，不得增加轨迹转移，新轨迹只能在 n 和稳定回中输入后开始。默认六轴最大幅值 ≤0.05、按钮释放持续 0.2 秒；记录任何覆盖值。达到回合配额时，n 必须保持禁用，q 可退出。用新输出在 SERL 模式重复。

D3：验证 tracker 候选边界拒绝会交还，且不发送越界目标；按钮 0 可以重新接管。不要为了测试故障而故意将实测 TCP 推到安全体积外。策略和遥操作都必须保留设备约束。故障/断连/watchdog 检查需另行审阅的受监督方案或无硬件测试；绝不能将拔线或杀死活动控制器当作未计划实验。

<a id="e-normal-stop-evidence-and-f-remediation"></a>

## E：正常停止证据与 F：修复

E1 使用会话的 `stop-trace-<timestamp>-<id>.json` 停止轨迹及评估。写入进程关闭后，将所选停止轨迹复制到以下路径并离线评估：

```bash
rwrl --json teleop inspect-stop --trace local/stop-trace.json
```

必须覆盖停止前静止阶段、整个停止区间、至少 2 秒停止后观测，设备时间戳递增，且覆盖间隙不超过 0.1 秒。相对于最后停止前位姿，采样到的最大 3D 位移必须 ≤10 mm，SO(3) 角 ≤5 度，最终 0.5 秒内每个关节始终满足 |dq| ≤0.02 rad/s。停止开始时机械臂必须处于活动 tracker 控制；已处于 IDLE 的机械臂为 BLOCKED，即使样本静止也如此。不得有控制故障或超过 `measured_margin_m` / `measured_margin_rad`（默认 `0.003 m` / `0.03 rad`）的实测工作区越界。目标限制仍严格执行。单独报告任务坐标系向下位移；它不替代 3D 误差。保留原始样本、SDK 指纹/模型字段、评估和视频。覆盖不足为 BLOCKED，已知阈值违规为 FAIL。测试两种模式；不得声称稀疏端点比较满足完整区间要求。

F1：E1 失败时，先区分固定目标保持不稳定、安装/负载/TCP 配置错误及 SDK 停止转换。不要盲调增益、添加重力补偿或延长超时。任何模型/SDK 修改都需要测量依据、操作员审阅、可复现版本，并重新执行受影响的 C–E 检查。所需 SDK 修改尚未交付时为 BLOCKED。紧急停止和故障清理不继承正常停止的位移承诺。本页不宣称此前观察到的停止下移问题已修复。
