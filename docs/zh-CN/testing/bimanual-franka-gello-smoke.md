# 双臂 Franka 与 GELLO 单机接管测试

计划执行日期：2026 年 11 月 5 日。状态：待现场执行。可用时间：3～4 小时。至少两人配合：一人操作两套 GELLO，一人负责按键、观察机器人、记录及停止操作。

本次在一台新的 Ubuntu 22.04 上位机上验证双 Franka 的人工接管与记录。旧 NUC 已运行过双臂 GELLO 遥操作，明天从该机读取实际配置作为迁移基线。新上位机运行两路设备守护进程和本机录制程序，使用 `joint_delta` 与对齐后接管 `arm`。无需策略 checkpoint。左侧、右侧、双侧三种接管组合各完成一次功能冒烟；空载通过后，再做轻软物体拿放。

独立 actor 服务器的安装、跨机器连接、actor/learner 上传和训练进入下一份测试大纲。本次的设备、键盘与进程均在新上位机。仓库的其他部署方式继续保留。

[中文入口](../../README.zh-CN.md) · [填写用 CSV 记录表](bimanual-franka-gello-smoke-record.csv) · [双臂参考](../bimanual.md) · [录制器参数](../recording-demos.md)

## 本次交付与通过条件

- [本机测试预设](../../../examples/presets/bimanual_gello_smoke.yaml)：两路 loopback 服务、状态观测、独立 GELLO 输入，10 Hz，`joint_scale_rad=0.01`。
- 录制器 `--max-episodes 1`：成功、失败、重录、Esc 或超时后结束本次尝试，释放设备控制，再汇总与导出记录。下一轮由人重新启动，重新启动会执行 reset。
- [轨迹检查工具](../../../examples/inspect_teleop_archive.py)：只读检查 SQLite，并导出逐步 CSV。
- 填写用 CSV 的每一行对应一个检查项；实际结果、执行人、时间和证据路径均在现场填写。

每个接管组合核对：左右映射正确、被接管侧跟随、另一侧保持、交还后保持、夹爪状态正确、轨迹与视频能对应。出现非预期运动、夹爪误动作或映射错误时，停止该项，保存证据，排查后再重测。记录表中的 `PASS` 表示该检查项通过；拿放回合的人工成功标签单独记录。程序正常退出与任务成功分别判断。

本次产生的是人工遥操作与功能验收记录。动作由人完成，录制器在无人接管时发送保持动作；不据此评价学习策略的自主成功率。

## 时间安排与现场放行

| 阶段 | 预算 | 完成标志 |
|---|---:|---|
| P00 读取 NUC 与盘点设备 | 25 分钟 | 两臂型号、固件、驱动、标定及接线已留档 |
| P01 安装并检查新机环境 | 45 分钟 | 依赖、实时运行条件、SDK 接口和权限通过 |
| P02 配置迁移与静态检查 | 40 分钟 | 串口、键盘、关节映射、home 与限制经复核 |
| T00～T03 空载与接管 | 35 分钟 | 左、右、双侧各一次通过 |
| T04～T05 轻物拿放 | 25 分钟 | 每臂各一次，抓持中交还保持通过 |
| T06～T09 退出与数据核对 | 25 分钟 | 失败、重录、超时、中断均有对应记录 |
| 收尾与问题登记 | 15 分钟 | 原始数据、视频、CSV 和问题记录归档 |

合计约 210 分钟，余量约 30 分钟用于 RGB 扩展或故障排查。P00～P02 未完成时，后续动作项填 `BLOCKED`；保留已取得证据和具体阻断原因。

首次 reset 前，由现场熟悉这套设备的人核定以下内容，并在 CSV 的 P02 行签名：

- 两台机器人确切型号及固件；目前只确认两臂型号相同，具体型号待查。
- 末端夹爪型号、负载、工具与 TCP 配置，以及现有驱动能否支持。
- 每臂已确认的 home 关节角、关节范围、TCP 工作空间和每步限幅。工作空间数值在各臂自己的基坐标系中定义，需要结合实际安装位置核对双臂几何关系。
- 两臂从当前位置到 home 的路径均清空。当前普通双臂 reset 会并发复位；逐臂预检通过后，仍需核准双臂同时复位路径。仓库尚无双臂互碰检查。
- 实体停止装置与现场停止流程可用，职责明确。先按现场设备规程启用 FCI、处理错误并确认机器人状态。

## 从旧 NUC 读取迁移基线

先保留旧 NUC 的原始文件和运行方式。只在旧机上读取、导出资料；新机接管控制前，按现场流程停止旧机的机器人控制进程。

在旧 NUC 上找到平时启动遥操作的终端、启动脚本或服务。运行中可用以下命令定位，再查看对应进程的工作目录和启动参数：

```bash
ps -eo pid,args | rg -i 'gello|franka|franky|polymetis|robot_infra'
```

若未装 `rg`，可用 `ps -ef` 查看。进入实际运行的项目与 Python/Conda 环境，保存下列输出；`pip show` 只会列出该环境确实安装的包，其他驱动记录其实际名称和版本。

```bash
python --version
python -m pip show franky-control dynamixel-sdk polymetis
git rev-parse HEAD
git diff
uname -r
ip -br address
ip route
ls -l /dev/serial/by-id/
```

| 要读取的内容 | 常见位置或方法 | 要带走的记录 |
|---|---|---|
| GELLO 标定 | 原版 GELLO 的 `gello/agents/gello_agent.py` 中 `PORT_CONFIG_MAP`，或启动脚本加载的 YAML | 左右串口、波特率、关节 ID、方向、偏移、夹爪 ID 和端点 |
| 本仓库设备配置 | `--config` 指定路径、`ROBOT_INFRA_CONFIG`、项目下 `local/station.yaml`、用户目录 `~/.config/robot_infra/station.yaml` | 整份 YAML 及实际来源路径；自定义 `XDG_CONFIG_HOME` 时查看其下 `robot_infra/station.yaml` |
| Franka 信息 | 两台设备铭牌和各自的 Desk，现有控制程序的依赖与启动参数 | 型号、系统固件、robot server 版本、FCI IP、夹爪、负载/TCP |
| 机器人网络 | 原 NUC 的网卡地址和路由 | 两条 FCI 链路的网卡/子网对应，管理网地址 |
| 运动与复位参数 | 现有控制代码和配置 | 已验证 home、限位、工作空间、控制器参数、运行频率和启动顺序 |

[原版 GELLO](https://github.com/wuphilipp/gello_software/blob/main/gello/agents/gello_agent.py) 的 `gripper_config` 端点使用度；本仓库 `gripper_open_rad`、`gripper_closed_rad` 使用弧度。迁移时按 `rad = deg * π / 180` 转换，并核对左右对应、关节绕回区间与静态读数。原版配置常含设备专属数据，保留完整原文件和本地修改。

## 准备新 Ubuntu 上位机

### 安装项目与依赖

以下命令在新上位机执行。项目已有副本时，先核对版本及本地修改，再更新到包含本案例和 `--max-episodes` 的提交。

```bash
git clone --recurse-submodules https://github.com/JaimeParker/real-world-rl.git
cd real-world-rl
sudo apt-get update
sudo apt-get install python3-venv python3-dev build-essential pkg-config
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
python -m pip install -e rl-garden -e 'robot_infra[grpc,gello,gello-events]' -e '.[dev]'
```

安装完成后，先运行与本次路径对应的无硬件测试：

```bash
python -m pytest tests/test_real_demo_recorder.py tests/test_bimanual_gello_smoke_case.py -q
python examples/record_real_demos.py --help
```

预期测试通过，帮助中包含 `--max-episodes`。测试使用 fake 设备和本机临时端口。

本次状态录制使用 CPU。键盘经上位机的 `evdev` 读取，录制命令使用设备端按键。独立 actor 与桌面键盘输入的部署在后续案例中安排。

根据实机固件与 robot server 版本选择 franky wheel。按 [franky 官方安装说明](https://github.com/TimSchneider42/franky/blob/master/README.md)及[Franka 兼容性说明](https://frankarobotics.github.io/docs/)核定整数 server 版本后执行：

```bash
read -r -p '输入已核定的 robot server 整数版本: ' FRANKA_SERVER_VERSION
python -m pip install --pre --only-binary=franky-control franky-control \
    --extra-index-url "https://timschneider42.github.io/franky/whl/by-robot-server-version/${FRANKA_SERVER_VERSION}/"
```

在连接设备前检查本仓库使用的 SDK 接口：

```bash
python - <<'PY'
from importlib.metadata import version
import franky
for name in ('Robot', 'Frame', 'JointMotion', 'JointImpedanceTracker', 'CartesianImpedanceTracker'):
    assert hasattr(franky, name), f'当前 franky 缺少 {name}，先核对安装版本'
print('franky-control:', version('franky-control'))
print('SDK 接口检查通过')
PY
```

阻抗跟踪接口见 [franky 官方说明](https://timschneider42.github.io/franky/guide/impedance-control.html)。SDK 版本不匹配或缺少接口时，先解决安装问题，再进行设备检查。

### 实时条件与设备权限

按旧 NUC 已验证的设置及 [Franka 实时内核要求](https://frankarobotics.github.io/docs/doc/libfranka/docs/real_time_kernel.html)配置新机的内核、运行优先级和用户限制，并记录核对结果。`realtime: enforce` 要求满足实时条件；仓库的 `realtime: ignore` 选项会跳过强制实时检查，doctor 的提示需要结合实际 FCI 通信表现处理。

```bash
uname -r
cat /sys/kernel/realtime
sudo usermod -aG dialout,input "$USER"
```

加入用户组后注销并重新登录，再激活 `.venv`，用 `id -nG` 确认权限。若 `/sys/kernel/realtime` 不存在，记录该事实并先确认现场采用的实时运行方案。

两套 GELLO 使用稳定的 `/dev/serial/by-id/…` 路径。新机沿用已经核实的机器人 IP 和网卡/子网对应关系。双臂共享一台 PC 时，两条 FCI 链路都需单独可达；若存在相同 IP 或路由冲突，由现场负责人先调整网络配置。本次两路设备服务只绑定本机 loopback。

## 建立本次目录与设备配置

在项目根目录的 Bash 终端执行；后续录制终端沿用这些变量：

```bash
source .venv/bin/activate
RUN_ROOT="$PWD/local/commissioning/$(TZ=Asia/Taipei date +%Y%m%d-%H%M%S)"
mkdir -p "$RUN_ROOT/baseline" "$RUN_ROOT/videos"
cp docs/zh-CN/testing/bimanual-franka-gello-smoke-record.csv "$RUN_ROOT/record.csv"
git rev-parse HEAD > "$RUN_ROOT/baseline/git-commit.txt"
git diff > "$RUN_ROOT/baseline/local-changes.patch"
python -m pip freeze > "$RUN_ROOT/baseline/pip-freeze.txt"
uname -a > "$RUN_ROOT/baseline/kernel.txt"
ip -br address > "$RUN_ROOT/baseline/interfaces.txt"
ip route > "$RUN_ROOT/baseline/routes.txt"
```

将 NUC 导出的文件和设备照片放进 `baseline/`，并创建一份现场基线记录，填写两臂型号/固件、夹爪、负责人、home、限制、允许运动幅度及观测容差。`local/` 已被 Git 忽略，原始现场资料保存在此目录。

可用向导生成双臂配置骨架，再逐项填入已经核实的数据：

```bash
python -m robot_infra.setup --arms 2 --arm-names left right \
    --cameras none none --teleop gello --no-calibrate \
    --output local/station.smoke.yaml
cp examples/presets/bimanual_gello_smoke.yaml local/bimanual.smoke.yaml
```

向导会探测设备并在结束时运行 doctor。`--no-calibrate` 用于先生成配置，随后将每套 GELLO 的标定完整替换为 NUC 实际值；运动前重新执行下文的正式检查。向导的 Panda 默认 home、限位和建议工作空间都需按实机复核。

编辑 `local/station.smoke.yaml`，保留 `robots: {left: …, right: …}` 结构：

| 字段 | left | right |
|---|---|---|
| `robot_ip` | 现场左臂 FCI IP | 现场右臂 FCI IP |
| `home_joints`、`joint_limits`、`safety_box` | 左臂核准值 | 右臂核准值 |
| `max_delta_per_step` | 按现场核准的每步限幅 | 按现场核准的每步限幅 |
| `gripper` | 实际夹爪及兼容后端 | 实际夹爪及兼容后端 |
| `cameras` | `[]` | `[]` |
| `daemon.bind` | `127.0.0.1:50051` | `127.0.0.1:50052` |
| `teleop.options.port` | 左 GELLO 稳定串口 | 右 GELLO 稳定串口 |
| GELLO ID、波特率、偏移、方向、夹爪端点 | 左侧实际标定 | 右侧实际标定 |
| `teleop.options.intervention` | `toggle` | `toggle` |
| `teleop.hold_s` | `0.0` | `0.0` |

两个 `teleop.options.event_device` 指向同一只专用键盘的 `/dev/input/by-id/…-event-kbd`。分别在两臂的 `teleop.options` 中设置下面的按键映射；其余标定字段保留现场值：

```yaml
# left 的 teleop.options 内
intervention: toggle
event_device: /dev/input/by-id/REPLACE_WITH_CONFIRMED_KEYBOARD-event-kbd
event_keys:
  toggle: KEY_F1
  episode_end: KEY_ESC
  success: KEY_S
  failure: KEY_F
  rerecord: KEY_R
```

右侧使用同一键盘路径，`toggle` 改为 `KEY_F2`，其余按键一致。`S/F/R/Esc` 作用于整个双臂回合。F1/F2 是状态切换键，按一次开启，再按一次关闭；两侧均开启时测试双臂接管。每次 reset 清除接管状态和复位期间的事件。

若设备 YAML 设置了 `daemon.token`，在环境 YAML 对应臂的 `token` 中填写一致值。

复制两臂已核准 `joint_limits` 到 `local/bimanual.smoke.yaml` 的对应臂配置中。该预设的 `joint_scale_rad=0.01` 在 10 Hz 下限定每步请求关节增量，属于本次调试建议值，仍受设备侧限幅约束；现场负责人核定后保持左右一致。`arm` 对齐阈值默认 0.1 rad，先静态确认映射再使用。主从误差在此阈值内才会启用接管。

本次预设不配置 `teleop_event_device`，使用两套 GELLO 设备端的独立开关。`robot_teleop_action_mode: joint_abs` 声明 GELLO 部件报告绝对关节角，环境 `action_mode: joint_delta` 决定转换后的执行动作。

## 静态核对与启动设备服务

### 仅读取两套 GELLO

先停止新机的设备守护进程，确认串口未被其他程序占用。下面只连接 GELLO 部件，读取关节、扳机和按键；机器人控制连接尚未启动。逐次小幅移动一个主臂关节，核对侧别、方向及关节顺序，再与从臂已经确认的静态关节角对照。按 F1、F2、S、F、R、Esc，观察两侧读数中的开关和事件是否符合映射。完成后 Ctrl-C 退出，再启动后续程序。

```bash
python - <<'PY'
import time
from robot_infra.devices import Robot
from robot_infra.discovery.autoconfig import load_station
station = load_station('local/station.smoke.yaml')
leaders = {name: Robot.from_config(entry.config).teleop
           for name, entry in station.robots.items()}
connected = []
try:
    for leader in leaders.values():
        assert leader is not None
        leader.connect()
        connected.append(leader)
    while True:
        for name, leader in leaders.items():
            sample = leader.read(1.0)
            print(name, 'q=', sample.action.round(3).tolist(),
                  'gripper=', sample.gripper, 'active=', sample.intervening,
                  'events=', {k: v for k, v in sample.events.items() if v}, flush=True)
        time.sleep(0.3)
except KeyboardInterrupt:
    pass
finally:
    for leader in reversed(connected):
        leader.disconnect()
PY
```

GELLO 夹爪读数范围为 `[0,1]`，录制器动作中的夹爪为 `[-1,1]`。对齐、扳机端点或关节方向不符时，回到标定核对；在从臂运动前解决差异。

### 正式设备检查

由设备负责人按现场流程完成 FCI 准备。守护进程仍保持关闭，逐臂运行 doctor；该检查会连接设备、读取反馈并检查状态。

```bash
set -o pipefail
python -m robot_infra.doctor --config local/station.smoke.yaml --robot left \
    2>&1 | tee "$RUN_ROOT/baseline/doctor-left.log"
python -m robot_infra.doctor --config local/station.smoke.yaml --robot right \
    2>&1 | tee "$RUN_ROOT/baseline/doctor-right.log"
```

两臂的 SDK、权限、握手、反馈及模式检查通过后，再启动设备服务。`VersionMismatch` 先核对固件和 wheel；`DeviceBusy` 先核对原 NUC 或本机是否还有控制进程；串口/键盘错误先检查路径、供电和组权限。

在另一个终端的项目根目录激活同一环境，运行下面命令，并保持其前台运行：

```bash
source .venv/bin/activate
python -m robot_infra.server --config local/station.smoke.yaml --watchdog-s 5
```

预期出现两路服务：`127.0.0.1:50051` 和 `127.0.0.1:50052`，均完成预检。5 秒是该调试命令的 watchdog 值，需与现场响应和复位耗时核定；watchdog 负责控制流中断后的停止路径。现场保留实体停止手段。

## 每轮启动和退出方法

回到定义 `RUN_ROOT` 的录制终端。先定义以下 Bash 函数；定义函数本身不会连接机器人。每次调用先显示本轮目录，只有输入 `GO` 后才启动录制；**首次启动会让双臂 reset 回 home 并开夹爪**。负责人应在输入前完成双臂路径和物体状态确认。

```bash
record_case() {
    local case_id="$1"
    local step_cap="${2:-1200}"
    local trial_dir="$RUN_ROOT/${case_id}-$(TZ=Asia/Taipei date +%H%M%S)"
    local confirmation
    mkdir -p "$trial_dir"
    cp local/station.smoke.yaml "$trial_dir/station.yaml"
    cp local/bimanual.smoke.yaml "$trial_dir/env.yaml"
    printf '本轮目录: %s\n' "$trial_dir"
    read -r -p '确认两臂复位路径清空、物体已妥善放置；输入 GO 开始: ' confirmation
    if [ "$confirmation" != GO ]; then
        return 1
    fi
    set -o pipefail
    python examples/record_real_demos.py --config local/bimanual.smoke.yaml \
        --output-path "$trial_dir/demo.pkl" --run-id "$(basename "$trial_dir")" \
        --num-successes 1 --max-episodes 1 --max-episode-steps "$step_cap" \
        2>&1 | tee "$trial_dir/recorder.log"
    local recorder_status=${PIPESTATUS[0]}
    printf '%s\n' "$recorder_status" > "$trial_dir/exit-code.txt"
    if [ -f "$trial_dir/demo.trajectory.sqlite3" ]; then
        python examples/inspect_teleop_archive.py "$trial_dir/demo.trajectory.sqlite3" \
            --csv "$trial_dir/steps.csv" > "$trial_dir/summary.json"
        cat "$trial_dir/summary.json"
    fi
    printf '请填写 record.csv，确认现场状态后再手动调用下一项。\n'
    return "$recorder_status"
}
```

如果 Ctrl-C 中断了整个终端流水线，回到对应目录后可单独执行轨迹检查命令。每次使用新目录和新文件；已有 `demo.pkl` 会被录制器拒绝覆盖，原始失败归档也保留。

| 按键 | 本次作用 | 使用时机 |
|---|---|---|
| F1 | 切换左臂及左夹爪接管 | 主从姿态与扳机状态核对后，由按键负责人操作 |
| F2 | 切换右臂及右夹爪接管 | 同上；两键都开启时双臂接管 |
| S | 标注整个回合成功并结束 | 拿放物体已放稳、仍在介入时按；本轮随后退出 |
| F | 标注失败并结束 | 本轮需按失败结束时；随后退出，保留 SQLite |
| R | 丢弃本轮示范并结束尝试 | 用于验证重录事件；`--max-episodes 1` 下随后退出 |
| Esc | 无成功标签地结束本轮 | 空载检查结束；结果未知并截断 |
| Ctrl-C | 中断录制，执行清理 | 本次空载退出检查及正常软件停止；记录为中断 |

F1/F2 交还后的保持在录制程序仍运行时检查。每轮结束时程序释放控制，按现场规程处理机器人状态，再开始清场和下一轮。键盘结束、软件停止与实体停止装置各按其用途使用；出现非预期运动时由现场人员执行设备停止流程。

## 空载与轻物验收操作

先摆好现场视频，画面同时覆盖双臂与两套 GELLO，能听到按键负责人的口令。每轮开始口述 case ID 和目录时间；记录视频文件与时间段。本次左右工作区分离，物体选轻软、易放稳的物品。

### T00 启动复位与无人接管保持

```bash
record_case T00
```

输入 `GO` 前确认首次双臂复位路径。启动后暂不按 F1/F2，观察两臂到达核准 home、夹爪打开，并保持数秒。移动未接管的 GELLO，从臂应保持。用 Esc 结束，程序应退出并保留一个未知结果回合的归档。人工核对本项，记录 `PASS/FAIL`；此项通常没有成功示范 pkl。

### T01 左侧接管与交还

```bash
record_case T01
```

1. 确认两臂已完成 reset，右侧接管保持关闭。将左 GELLO 主从姿态与扳机状态对齐。
2. 按 F1，确认左侧开始接管。先小幅移动一个关节，再依次核对其他关节方向和跟随；动作范围使用 P02 核定值。
3. 测试左夹爪开合；右臂和右夹爪保持。右 GELLO 未接管时的小幅移动不应驱动右臂。
4. 按 F1 交还，持续观察数秒：左臂保持当前位置，夹爪保持最后命令。再次接管前，先让扳机匹配实际夹爪状态。
5. 全部检查结束后 Esc 退出。核对逐步 CSV 中出现左侧人工掩码，右侧机械臂命令为零、右夹爪命令保持。

需要检查未对齐保护时，在空载且运动范围已核准的条件下，让主臂误差超过配置阈值再切换接管。此时应维持未介入；进入对齐阈值后才启用。测试人员需预先知晓该启用时刻，记录对应视频和掩码变化。

### T02 右侧接管与交还

```bash
record_case T02
```

按 T01 对称执行，使用 F2；核对右侧跟随、右夹爪、左侧保持及右侧交还后的保持。Esc 结束并核对右侧人工掩码。

### T03 双侧接管与逐侧交还

```bash
record_case T03
```

1. 两套 GELLO 均对齐、扳机状态匹配后，按 F1 和 F2 开启两侧接管。
2. 两臂各自在核准的独立工作区做小幅动作，分别开合夹爪。
3. 按 F1 交还左侧，右侧继续做小幅动作；左臂及左夹爪应保持。
4. 按 F2 交还右侧，检查两臂及夹爪保持数秒。
5. Esc 结束。归档中应能对应双侧、仅右侧、无人介入的掩码与动作片段。

T00～T03 全部通过后，进入轻物阶段。任一映射、保持或夹爪异常先处理并重测该项。

### T04 左侧轻软物体拿放

```bash
record_case T04
```

左侧接管并抓取轻软物体，低位保持；右侧保持。按 F1 交还，在程序仍运行时观察夹爪保持、物体未意外掉落。先匹配扳机状态，再按 F1 重新接管，将物体放稳并松爪，保持接管按 S 结束。程序退出后应有 SQLite、成功人工示范 pkl 及约定 sidecar。记录物体、抓持/交还/再接管的视频时段和实际结果。

### T05 右侧轻软物体拿放

```bash
record_case T05
```

按 T04 对称执行，使用 F2。两臂同时接管已在空载验证；本次拿放分别完成，物体放稳后再结束回合。

## 失败与退出记录验收

以下项目在空载且 P02 条件已确认后执行。S 已在 T04/T05 验证，Esc 已在空载项目验证。每项只运行一次尝试，结束后检查程序已退出、没有进入下一次 home/开爪。

| 编号与命令 | 操作 | 预期记录 |
|---|---|---|
| `record_case T06` | 运行数步后按 F | `task_success=false`、人工失败来源；完整 SQLite，无成功 pkl |
| `record_case T07` | 运行数步后按 R | 丢弃回合，`discarded=true`；完整 SQLite，无成功 pkl |
| `record_case T08 30` | F1/F2 均关闭，等待 30 步上限 | 10 Hz 下约 3 秒步进后截断；结果未知，只有一个回合 |
| `record_case T09` | F1/F2 均关闭、已经开始步进后按 Ctrl-C | 清理并退出，归档有中断结果；人工确认机器人已停止控制 |

Ctrl-C 通常返回状态 130，成功/失败回合都可能正常返回 0；用归档结果及现场观察判定检查项。每项应只有一个 `episode_start` 和一个 `episode_end`。若中断发生在首次 reset 或首步之前，可能尚未产生回合归档；按“启动期中断”登记，待现场状态确认后重新执行“步进后中断”项。

本次不主动拔断正在工作的 FCI/GELLO 线缆。真实断线、外部 actor 网络中断与训练上传恢复另设测试条件；软件异常先在 fake 测试中覆盖。

## 数据核对与收尾记录

每轮目录包含配置快照、终端日志，以及按实际情况产生的文件：

| 文件 | 用途 |
|---|---|
| `demo.trajectory.sqlite3` | 完整回合与逐步记录，包括失败、重录、未知和中断尝试 |
| `demo.pkl`、`demo.contract.json` | 有成功人工步骤时导出的示范与约定；无保留成功回合时不生成 |
| `steps.csv` | 从 SQLite 只读导出的逐步证据，包括动作、观测、掩码、事件与终止标志 |
| `summary.json` | 回合数、步骤数、人工掩码计数及显式结果 |
| `recorder.log`、`exit-code.txt` | 软件过程与退出状态；Ctrl-C 中断流水线时按日志补记退出情况 |
| `station.yaml`、`env.yaml` | 该轮实际使用的设备与环境配置快照 |
| `videos/` 下的现场视频 | 画面及口令对应 case ID，CSV 填写文件和时间段 |

可单独执行检查工具；输出 CSV 必须是新文件：

```bash
python examples/inspect_teleop_archive.py /path/to/demo.trajectory.sqlite3 \
    --csv /path/to/new-steps.csv
```

核对以下项目后填写总表：

1. 一次尝试对应一个开始与一个结束，步骤编号连续、时间有序；Ctrl-C 的结束结果为中断。工具以只读方式打开数据库，未完成回合保持原状。
2. 状态顺序为 `[左关节7维, 左夹爪状态, 右关节7维, 右夹爪状态]`；动作顺序为 `[左臂7维, 右臂7维, 左夹爪命令, 右夹爪命令]`。状态中的关节角单位为 rad。
3. `human_action_mask` 中的 1 表示实际由人控制的维度；在本次两侧均有夹爪通道的配置下，对照下表。`override_mask` 在局部接管、空闲侧保持时可能全为 1，保持维度的人工掩码仍为 0。
4. `action` 为交给环境的归一化动作，设备侧仍会执行限幅；从臂实际位置以 `state/next_state` 与视频核对。无人接管时两臂增量命令为零，夹爪保持上次命令。
5. 成功、失败、重录、Esc/超时和中断分别与对应事件、结果及现场口令一致。失败优先于成功；仅按标注键不计为动作介入。

| 人工控制状态 | 16 位人工掩码 |
|---|---|
| 无人接管 | `0000000000000000` |
| 仅左侧 | `1111111000000010` |
| 仅右侧 | `0000000111111101` |
| 双侧 | `1111111111111111` |

填写 `record.csv`：实际开始时间、执行人/复核人、观察结果、`PASS/FAIL/BLOCKED/NOT_RUN`、本轮目录、视频时间段和问题编号。未执行项目保持 `NOT_RUN`，前置条件不满足用 `BLOCKED` 并写明原因。拿放失败与功能失败分开描述，例如“物体放置失败，但人工接管/交还行为符合预期”。

收尾顺序：确认录制程序已经退出、现场设备状态妥善，再停止两路守护进程；将原始轨迹、配置、视频、软件基线和填写后的 CSV 一起备份。SQLite 使用 WAL `synchronous=NORMAL`，持久保证覆盖进程崩溃，操作系统崩溃或断电可能丢失最新提交。复制时先关闭写入程序；仍存在的 `-wal`、`-shm` 文件与数据库一起保留。

## 常见阻断与处理

| 现象 | 下一步 |
|---|---|
| SDK/固件不兼容，或缺少 tracker 接口 | 核对确切型号、robot server 版本和官方 wheel，记录实际错误；修复后重跑 P01 |
| 新机 FCI 不可达或通信不稳定 | 对照 NUC 的网卡/路由和实时配置，按现场流程做逐臂通信检查；保留两臂分别的结果 |
| 键盘无响应或串口权限不足 | 核对稳定设备路径、当前用户组与重新登录状态，回到静态读数步骤 |
| F1/F2 控制了错误一侧 | 停止本轮，核对左右串口、机器人 IP、配置名称与键位 |
| 接管键开启后仍保持 | 观察主从误差与 `arm` 对齐条件；重新核对标定、关节绕回区间和从臂姿态 |
| 接管时夹爪误开 | 停止该项，核对扳机端点、二值化方向及接管前状态匹配；见[已知问题 S3](../known-issues.md) |
| 执行后没有 pkl | 查看完整 SQLite 和结果：失败、丢弃、未知、中断，或没有可保留的人工步骤时符合预期 |
| 本轮结束后出现下一次复位 | 核对代码版本和命令确实含 `--max-episodes 1`；保留日志，暂停后续项排查 |
| 非预期运动或误夹爪 | 执行现场停止流程、确认设备状态、保存日志和视频；复核后再重新启动该项 |

## 可选 RGB 扩展与下一次测试

必测项完成且时间充足时，可在新一轮配置中加入已就绪的相机 RGB：记录相机序列号和驱动，左右相机命名唯一，设备 YAML 的相机列表、环境逐臂列表与 `obs.rgb` 按左后右顺序一致。本次不要求深度、裁剪或训练。RGB 配置使用新的运行目录与归档，保留它自己的配置快照。

下一份案例再覆盖独立无桌面 actor 的环境配置、与 Ubuntu 上位机的连接，以及后续 actor/learner 数据链路。本次结果归档后，以实际通过项、阻断项和新增问题作为下一次准备依据。
