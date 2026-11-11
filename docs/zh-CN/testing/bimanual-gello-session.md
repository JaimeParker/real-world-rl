<!-- English source: docs/testing/bimanual-gello-session.md -->

Reference only: the first hardware session followed the long form of this page; keep for the next bimanual session, not maintained with the tutorials.

仅供参考：首次实机测试使用本文的长版；保留给下次双臂测试，不随教程维护。

# 双臂 Franka / GELLO 测试

所有命令在 **NUC / Ubuntu 控制机、仓库根目录、Bash** 执行；P01 无需激活环境；之后激活安装器创建的环境（默认为 `.venv`），例如执行 `source .venv/bin/activate`；后续命令和其它终端均使用项目环境（project environment）。不启动 learner、无需 checkpoint。本页不代表实机通过。
P02 前按 [setup](../robot-infra.md) 配好 `local/station.smoke.yaml`：实际左右 IP、Hand、home、关节限位、安全框、每步限幅、独立稳定 GELLO 串口及完整标定；复制 `examples/presets/bimanual_gello_smoke.yaml` 为 `local/bimanual.smoke.yaml`，填写逐臂 joint_limits/token。连接从臂前核对 leader 静态读数。
设备服务绑定 127.0.0.1:50051 / :50052，相机为空，teleop.hold_s=0.0、intervention=toggle；两侧使用同一已确认 evdev 键盘 event_device，左 toggle KEY_F1、右 KEY_F2；episode_end/success/failure/rerecord 分别 KEY_ESC/KEY_S/KEY_F/KEY_R。保留预设设备端事件（不加 stdin 覆盖）、joint_delta、10 Hz、joint_scale_rad=0.01、leader joint_abs 及 arm 对齐容差 0.1 rad。
确认实体停止可用、双臂同时 home/reset 路径清空、工作区分离；仓库没有双臂互碰检查。接管前扳机须匹配夹爪；此关节路线没有 cartesian 夹爪锁存保护。reset 会开爪并清除接管状态。非预期运动或开爪立即停止，保留证据再重测。
选一个全新 RUN_ROOT；准备命令拒绝已有目录。只填写其中的[记录 CSV](bimanual-gello-session-record.csv)，结论限 PASS/FAIL/BLOCKED/NOT_RUN；缺前置条件/证据为 BLOCKED。保留配置、软件/设备盘点及带项目编号的时间视频；退出码不等于检查结论。

```bash
RUN_ROOT=local/commissioning/bimanual-next
```

<!-- tutorial-test: skip -->
```bash
set -o pipefail
mkdir -p local/commissioning && mkdir "$RUN_ROOT" || exit 1
mkdir "$RUN_ROOT/videos"
printf '%s\n' "$RUN_ROOT" > local/bimanual-current-run
cp docs/testing/bimanual-gello-session-record.csv "$RUN_ROOT/record.csv"
check() {
  local id=$1; shift
  mkdir "$RUN_ROOT/$id" || return
  "$@" 2>&1 | tee "$RUN_ROOT/$id/console.log"
  local status=("${PIPESTATUS[@]}") rc
  rc=${status[0]}; if [[ $rc == 0 && ${status[1]} != 0 ]]; then rc=${status[1]}; fi
  printf '%s\n' "$rc" > "$RUN_ROOT/$id/exit-code.txt"
  return "$rc"
}
read_leaders() {
  python - <<'STATIC_PY'
import time
from robot_infra.devices import Robot
from robot_infra.discovery.autoconfig import load_station
station = load_station('local/station.smoke.yaml')
leaders = {name: Robot.from_config(entry.config).teleop for name, entry in station.robots.items()}
connected = []
try:
    for leader in leaders.values():
        assert leader is not None
        leader.connect()
        connected.append(leader)
    while True:
        for name, leader in leaders.items():
            sample = leader.read(1.0)
            print(name, 'q=', sample.action.round(3).tolist(), 'gripper=', sample.gripper, 'active=', sample.intervening, 'events=', {k: v for k, v in sample.events.items() if v}, flush=True)
        time.sleep(0.3)
except KeyboardInterrupt:
    pass
finally:
    for leader in reversed(connected):
        leader.disconnect()
STATIC_PY
}
record_case() {
  local id=$1 step_cap=${2:-1200} trial_dir="$RUN_ROOT/$1" confirmation rc
  [[ ! -e "$trial_dir" ]] || { printf 'Use a fresh item directory\n' >&2; return 2; }
  read -r -p 'Both reset paths clear, triggers matched, objects secure? Type GO: ' confirmation
  [[ $confirmation == GO ]] || return 1
  mkdir "$trial_dir" || return
  cp local/station.smoke.yaml "$trial_dir/station.yaml" || return
  cp local/bimanual.smoke.yaml "$trial_dir/env.yaml" || return
  rwrl supervise --log "$trial_dir/supervise.jsonl" -- rwrl record --config local/bimanual.smoke.yaml --output-path "$trial_dir/demo.pkl" --run-id "$(basename "$trial_dir")" --num-successes 1 --max-episodes 1 --max-episode-steps "$step_cap" 2>&1 | tee "$trial_dir/recorder.log"
  local status=("${PIPESTATUS[@]}")
  rc=${status[0]}; if [[ $rc == 0 && ${status[1]} != 0 ]]; then rc=${status[1]}; fi
  printf '%s\n' "$rc" > "$trial_dir/exit-code.txt"
  if [[ -f "$trial_dir/demo.trajectory.sqlite3" ]]; then
    python examples/inspect_teleop_archive.py "$trial_dir/demo.trajectory.sqlite3" --csv "$trial_dir/steps.csv" > "$trial_dir/summary.json" || return
  fi
  return "$rc"
}
```

P00–P03-R 期间守护进程关闭。两侧 doctor 通过后，在第二终端（同机器/根目录/项目环境）启动以下命令并留存日志，禁止与 doctor 并发。5 s 为需现场核准的调试用 daemon watchdog，和 supervisor 心跳超时不同：

<!-- tutorial-test: skip -->
```bash
RUN_ROOT=$(cat local/bimanual-current-run)
robot-infra daemon --station local/station.smoke.yaml --watchdog-s 5 2>&1 | tee "$RUN_ROOT/daemon.log"
```

`record_case` 每次 reset 前要求 GO，仅允许一次尝试（默认 1200 步），运行 supervisor、记录退出码并导出归档证据。以下为 T00 内层命令的参考展开；实际使用表格函数完成日志/证据导出，不要重复执行：

```bash
rwrl supervise --log "$RUN_ROOT/T00/supervise.jsonl" -- \
    rwrl record --config local/bimanual.smoke.yaml \
    --output-path "$RUN_ROOT/T00/demo.pkl" --run-id T00 \
    --num-successes 1 --max-episodes 1 --max-episode-steps 1200
```

| 编号 / 项目 | 命令 | 操作与通过标准 | 主要记录文件（相对 RUN_ROOT） |
|---|---|---|---|
| P00 基线与设备盘点 | `check P00 bash -c 'git rev-parse HEAD; git diff; uname -a; ip -br address; ip route'` | 提交/本地修改、双臂型号/固件/server 版本、Hand/负载/TCP、FCI IP 和 GELLO 串口/标定均已留档 | `P00/console.log` |
| P01 环境与 SDK | `check P01 bash scripts/install.sh --extras franka-gello --no-doctor` | 安装成功；核对本机实时条件及 dialout/input 权限，随后激活安装器创建的环境（默认为 `.venv`） | `P01/console.log` |
| P02 配置与静态放行 | `check P02 read_leaders` | 逐关节移动 leader，核对标定/方向/静态读数与 F1/F2/S/F/R/Esc 事件，Ctrl-C 关闭 leader；核准 home/限位、分离工作区及同时复位路径 | `P02/console.log` |
| P03-L 左侧 doctor | `check P03-L rwrl doctor --station local/station.smoke.yaml --robot left` | 左侧 SDK、权限、握手、反馈及模式检查通过；守护进程保持关闭 | `P03-L/console.log` |
| P03-R 右侧 doctor | `check P03-R rwrl doctor --station local/station.smoke.yaml --robot right` | 右侧 SDK、权限、握手、反馈及模式检查通过；守护进程保持关闭 | `P03-R/console.log` |
| T00 复位与无人接管保持 | `record_case T00` | 核准 home/开爪；未接管时移动 leader 不驱动 follower；Esc 后仅一个未知/截断回合并退出 | `T00/summary.json` |
| T01 左侧接管与交还 | `record_case T01` | 对齐后 F1：左关节/夹爪正确、右侧保持；F1 交还保持；超容差等待对齐；Esc；左掩码与视频相符 | `T01/steps.csv` |
| T02 右侧接管与交还 | `record_case T02` | 按 T01 对称执行 F2：右跟随/夹爪正确，左保持，右交还保持；Esc；右掩码相符 | `T02/steps.csv` |
| T03 双侧接管与逐侧交还 | `record_case T03` | 两侧对齐后 F1/F2 各自跟随；F1 交还左侧时右侧跟随，再 F2 交还右侧；Esc；双/右/无人掩码相符 | `T03/steps.csv` |
| T04 左侧轻软物体拿放 | `record_case T04` | F1 抓取，交还抓持不掉落，扳机匹配后再接管，放稳后在介入中 S；SQLite/pkl/contract 齐 | `T04/summary.json` |
| T05 右侧轻软物体拿放 | `record_case T05` | 按 T04 对称执行 F2；交还抓持保持，放稳后 S；成功人工示范及 sidecar 齐 | `T05/summary.json` |
| T06 失败 | `record_case T06` | 步进后 F：task_success=false/人工失败，SQLite 完整，无成功 pkl；仅一回合退出释放，无下一次复位 | `T06/summary.json` |
| T07 重录 | `record_case T07` | 步进后 R：discarded=true，归档完整，无成功 pkl；仅一回合退出释放，无下一次复位 | `T07/summary.json` |
| T08 超时 | `record_case T08 30` | 不接管；10 Hz 下 30 步约 3 s 步进后截断/未知；仅一回合退出，无下一次复位 | `T08/summary.json` |
| T09 步进后 Ctrl-C | `record_case T09` | 不接管且开始步进后 Ctrl-C，清理并释放控制，记录 interrupted；仅一回合退出，无下一次复位 | `T09/summary.json` |
| CLOSE 证据核对与归档 | `tar -czf "$RUN_ROOT.tar.gz" -C "$(dirname "$RUN_ROOT")" "$(basename "$RUN_ROOT")"` | 录制退出、机器人静止、守护进程关闭；步骤/掩码/结果与视频相符；CSV 已填，配置/SQLite/WAL/SHM/视频/日志已打包 | `$RUN_ROOT.tar.gz` |
| X01 可选 RGB | `record_case X01` | 必测全通过后，新本地配置加入核准的唯一相机；左右 device/env/obs.rgb 顺序一致；独立归档及快照 | `X01/summary.json` |

T00–T03 空载全通过后，才做 T04–T05 轻软物体。F1/F2 分别切换本侧臂与爪；S/F/R/Esc 作用整个回合。介入中按 S；F/R/Esc/超时结束唯一尝试，Ctrl-C 清理。无保留人工成功时不生成 pkl 属正常；检查通过不代表自主任务成功。不主动测试工作中拔线。
核对每项一个 episode_start/end、步骤连续、时间有序且结果对应视频（reset/首步前中断可能没有回合，另记启动期中断，再于步进后重测）。state 为 [左关节7, 左爪, 右关节7, 右爪]；action 为 [左臂7, 右臂7, 左爪, 右爪]。human mask：无人 0000000000000000、左 1111111000000010、右 0000000111111101、双 1111111111111111；override_mask 可含非人工保持维度。关节单位 rad。
inspect 为只读，不补写未结束回合。保留 recorder.log、exit-code.txt、supervise.jsonl、station.yaml/env.yaml、SQLite、steps.csv、summary.json、仅成功产出的 pkl/contract 及视频时段。X01 在介入中按 S 验证 RGB 示范导出。CLOSE 前确认退出/静止、停守护进程、填 CSV、关闭数据库写入，SQLite 连同仍存在的 WAL/SHM 保存。WAL synchronous=NORMAL 覆盖进程崩溃，不保证全部断电更新。CSV 更新后重新打包，禁止复用项目目录。
