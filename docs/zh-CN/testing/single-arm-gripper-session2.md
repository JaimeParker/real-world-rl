<!-- English source: docs/testing/single-arm-gripper-session2.md -->

# 第二次单臂＋夹爪现场测试：GELLO / SpaceMouse

范围：一台 NUC、单臂、真实 Franka Hand，硬件由一个录制子进程直接连接；不启动训练。本文未代表实机通过。
所有命令在 **NUC、仓库根目录、Bash** 执行；S01 无需激活环境；成功后激活安装器创建的环境（默认为 `.venv`），例如执行 `source .venv/bin/activate`；其余命令均使用项目环境（project environment）。
先按 [教程 01](../../tutorial/01-install.md) 上电、解锁、End Effector 上电、核对 Control 网线与 Desk；确认机器人身份和物理 Hand，不能凭默认 IP 推断。
按 [教程 08](../../tutorial/08-record-demos.md#prepare-the-nuc) 配好实际 IP、home、限位、工作空间及 GELLO 标定；`gello_leader.yaml` 指向独立 fake-arm leader YAML，不得重复连接实臂。
预设为 state-only、50 Hz、cartesian_delta，尺度 0.05 m / 0.3 rad；领先上限 0.05 m / 0.3 rad，目标速度 0.05 m/s / 0.3 rad/s。先确认物理停止装置可用。
每项结束后填运行目录内的 [记录 CSV](single-arm-gripper-session2-record.csv)：计划日期、实际开始时间和观察均现场填写；初始为 NOT_RUN，无证据用 BLOCKED，实测超标用 FAIL。
测量阶段同时拍摄机械臂与带时间的终端，视频放入该项目目录；不要为测试放宽阈值或工作空间。

先运行以下准备命令（NUC、仓库根、无 venv 也可）；每次新建 RUN，禁止复用已有项目目录：

<!-- tutorial-test: skip -->
```bash
set -o pipefail
RUN="runs/single-arm-session2-$(date -u +%Y%m%dT%H%M%SZ)"
PRESET=examples/presets/gello_cartesian.yaml
mkdir -p local runs && mkdir "$RUN" || exit 1
printf '%s\n' "$RUN" > local/session2-current-run
cp docs/testing/single-arm-gripper-session2-record.csv "$RUN/record.csv"
git rev-parse HEAD > "$RUN/commit.txt"
cp "$PRESET" "$RUN/preset.yaml"
cp local/station.yaml "$RUN/station.yaml"
cp gello_leader.yaml gello_station.yaml "$RUN/"
check() {
  local id=$1; shift
  mkdir "$RUN/$id" || return
  "$@" 2>&1 | tee "$RUN/$id/console.log"
  local status=("${PIPESTATUS[@]}") rc
  rc=${status[0]}; if [[ $rc == 0 && ${status[1]} != 0 ]]; then rc=${status[1]}; fi
  printf '%s\n' "$rc" > "$RUN/$id/exit-code.txt"
  return "$rc"
}
record() {
  local id=$1 device=$2 rc; shift 2
  [[ ! -e "$RUN/$id" ]] || { printf "Item directory exists; create a fresh RUN\n" >&2; return 2; }
  local options=()
  if [[ $device == spacemouse ]]; then options=(--teleop-options-yaml None); fi
  check "$id" rwrl supervise --log "$RUN/$id/supervise.jsonl" -- rwrl record --config "$PRESET" --teleop-device "$device" --teleop-event-device stdin "${options[@]}" --real_robot.watchdog_s 0.3 --output-path "$RUN/$id/demo.pkl" --num-successes 1 --max-episodes "${RECORD_ATTEMPTS:-1}" "$@"
  rc=$?
  if [[ -f "$RUN/$id/demo.trajectory.sqlite3" ]]; then
    rwrl archive inspect "$RUN/$id/demo.trajectory.sqlite3" --csv "$RUN/$id/steps.csv" --measurements "$RUN/$id/measurements.csv" > "$RUN/$id/summary.json" || return
  fi
  [[ -f "$RUN/$id/measurements.csv" ]] || printf 'phase,start_s,end_s,max_drift_mm,residual_mm,settle_s,stop_to_exit_s,stop_to_robot_stop_s,evidence\n' > "$RUN/$id/measurements.csv"
  return "$rc"
}
```

`record` 默认最多一回合；S10-r 两回合会复位两次；失败不产出 pkl 是正常行为。成功须在接管中按 `s`，避免漏掉人类示范的终止步。
`exit-code.txt` 是命令退出码，不是检查结论；异常退出测试即使退出码非零，仍按下表证据判定。
按顺序执行下表，每行一个命令；记录文件路径均相对于 `$RUN`，同目录保留 console、supervise、SQLite、steps 和视频等全部证据。

| 编号 / 项目 | 命令 | 操作与通过标准 | 主要记录文件 |
|---|---|---|---|
| S01 安装与检测 | `check S01 bash scripts/install.sh --extras franka-gello,spacemouse --station local/station.yaml` | doctor 各层无 FAIL，实际设备必检项均通过；成功后激活安装器创建的环境（默认为 `.venv`） | `S01/console.log` |
| S02 Desk / FCI | `check S02 rwrl doctor --station local/station.yaml --preset "$PRESET"` | 逐项确认页面可达、绿灯、已登录、本机控制权、已安装 FCI、会话已激活；刷新正确 Desk，handshake OK；保存六项证据 | `S02/console.log` |
| S03 Hand 反馈 | `check S03-open rwrl doctor --station local/station.yaml --preset "$PRESET"`，再运行 `check S03-closed rwrl doctor --station local/station.yaml --preset "$PRESET"` | doctor 两次运行：Hand 张开时一次、在 Desk 上闭合后再一次；两次打印的宽度与肉眼观察的开口一致（Franka Hand 张开约 0.08 m，闭合接近 0），无 WARN | `S03-open/console.log`、`S03-closed/console.log` |
| S03 后可选 | `rwrl gripper show` | 同一 NUC、仓库根、项目环境激活，启动 recorder 前执行：应打印与 doctor 相同的夹爪宽度行（米、归一化宽度、抓取状态和后端来源）；保留上方 S02/S03 doctor 检查 | 可选对照，无 CSV 项目 |
| S04 复位与保持 | `record S04 leader_cartesian --max-episode-steps 750` | 复位后不接管至少 10 s；inspector 写入的 `measurements.csv` 中 `hold_window` 行（亦即 `summary.json` 中该保持段的 `max_drift_window_mm`）覆盖至少 10 s，且其 `max_drift_mm` ≤5 mm | `S04/measurements.csv` |
| S05 GELLO t/h/t | `record S05 leader_cartesian` | `t` 驱动、`h` 保持 10 s、`t` 再接管后 `q`；`h` 保持段的 `hold_window` 行 `max_drift_mm` ≤5 mm，再接管无跳动（`summary.json` 中其后接管段的 `jump_mm`，并以视频确认），`summary.json` 的分段与接管/交还一致 | `S05/measurements.csv` |
| S06 GELLO 领先上限 | `record S06 leader_cartesian` | 在已核定范围内快速移动 leader 再静止，并在第二终端标记 leader 停止时刻（见下）；最后 `q`，再用该时刻重跑 inspector：`settle` 行 `settle_s` ≤1.5 s、`residual_mm` ≤50 mm | `S06/measurements-stop.csv` |
| S07 GELLO 夹爪 S3 | `record S07 leader_cartesian` | 空载开合后闭合，`h` 交还、释放扳机再 `t`；夹爪保持闭合，匹配后才响应；无误开，最后 `q` | `S07/console.log` |
| S08 GELLO 拿放成功 | `record S08 leader_cartesian` | 拿放并在接管中 `s`；pkl＋contract＋SQLite 齐，task_success=true、had_intervention=true，因此 autonomous_success=false | `S08/summary.json` |
| S09 SpaceMouse 合并项 | `record S09 spacemouse` | 只推杆不接管；键0接管/交还，交还保持10 s漂移≤5 mm，再接管无跳动；键1夹爪无误开；拿放中 `s`，结果/文件同 S08 | `S09/summary.json` |
| S10-f 失败 | `record S10-f leader_cartesian` | 按 `f`；归档结束，task_success=false / manual_failure，无成功 pkl，进程退出、设备释放 | `S10-f/summary.json` |
| S10-r 重录 | `RECORD_ATTEMPTS=2 record S10-r leader_cartesian` | 首回合按 `r`，discarded=true；第二回合新ID并复位后按 `q`，两回合归档结束，无成功 pkl，进程退出、设备释放 | `S10-r/summary.json` |
| S10-timeout 超时 | `record S10-timeout leader_cartesian --max-episode-steps 500` | 不接管/不打标签，约10 s后 truncated、无成功标签，归档结束，进程退出、设备释放 | `S10-timeout/summary.json` |
| S10-int Ctrl-C | `record S10-int leader_cartesian` | 运行中 Ctrl-C；supervisor 转发信号并等待清理，归档标记 interrupted，进程退出、设备释放 | `S10-int/summary.json` |
| S10-q 退出 | `record S10-q leader_cartesian` | 无成功标签时按 `q`；operator_quit 事件、结束记录齐，无成功 pkl，进程退出、设备释放 | `S10-q/steps.csv` |
| S11 监督停止 | `record S11 leader_cartesian` | reset完成，空载低速跟随且有稳定step心跳后执行下方 STOP；要求 STOP 后≤8 s子进程终止（默认参数：2 s 心跳超时 + 3 s SIGINT + 2 s SIGTERM 后 SIGKILL，被 STOP 的进程只对 SIGKILL 有反应）且机器人停止、Desk 可恢复；记录实际耗时；计时缺证据 BLOCKED，超标 FAIL | `S11/measurements.csv` |
| S12 home / 断开 / 归档 | `record S12 leader_cartesian --max-episode-steps 1 && tar -czf "$RUN.tar.gz" -C "$(dirname "$RUN")" "$(basename "$RUN")"` | 返回已核定 home。没有 recorder 运行时，`rwrl --station local/station.yaml --preset "$PRESET" home go` 也可不录制直接回 home；它会移动机械臂、应用 `task.reset.gripper` 并请求确认。进程退出后确认机器人静止/连接已释放，再物理断开；整个运行目录已打包，更新 CSV 后重新打包 | `$RUN.tar.gz`（RUN 同级） |

S06 运行 `record S06` 时，另开第二终端（NUC、仓库根、项目环境激活），预先输入下列命令，在 leader 刚静止的瞬间按回车；该终端须在视频画面内，以便对照时间戳：

<!-- tutorial-test: skip -->
```bash
RUN=$(cat local/session2-current-run)
date +%s.%N | tee "$RUN/S06/leader-stop.txt"
```

录制进程退出后，用该停止时刻对同一归档重跑 inspector（输出到新文件；inspector 从不覆盖已有证据）：

<!-- tutorial-test: skip -->
```bash
rwrl archive inspect "$RUN/S06/demo.trajectory.sqlite3" --stop-time-s "$(cat "$RUN/S06/leader-stop.txt")" --measurements "$RUN/S06/measurements-stop.csv" > "$RUN/S06/summary-stop.json"
```

结果在 `summary-stop.json` 的 `settle` 下；`settled: false` 或出现 `problem`（停止时刻不在回合内，或回合结束前机械臂未静止）判 FAIL 或 BLOCKED，不能判 PASS。带 `--stop-time-s` 时，该命令在所有停止都已稳定时退出 0，有一次未稳定时退出 3，某个停止时间没有覆盖它的 TCP 回合时退出 4；将该退出码与 JSON 一并记录。

S11 第二终端同样在 NUC、仓库根、项目环境激活；只在复位结束、已核定路径内空载低速跟随且有稳定 step 心跳后执行，绝不 STOP 控制箱或 supervisor：

<!-- tutorial-test: skip -->
```bash
RUN=$(cat local/session2-current-run)
PID=$(python -c 'import json,sys; rows=[json.loads(s) for s in open(sys.argv[1])]; print(next(r["pid"] for r in rows if r["event"]=="spawn"))' "$RUN/S11/supervise.jsonl")
ps -p "$PID" -o pid=,args= | tee "$RUN/S11/process.txt"
[[ $(ps -p "$PID" -o args=) == *"rwrl record"* ]] || exit 1
python -c 'import json,sys,time; print(json.dumps({"pid":int(sys.argv[1]),"t_wall":time.time(),"t_monotonic":time.monotonic()}))' "$PID" | tee "$RUN/S11/stop-request.json"
kill -STOP "$PID"
```

S11 默认心跳超时 **2 s 后才开始升级**，再等待 SIGINT 3 s、SIGTERM 2 s 后才 SIGKILL；被 STOP 的进程对 SIGINT/SIGTERM 无反应，预期在 STOP 后约 7 s 被 SIGKILL，这就是 8 s 判据的来源。
将 stop-request、supervise 的 child_exit 时刻及视频停机时刻填入 measurements；STOP前的记录不能证明之后的物理停止。不要为达标擅改默认超时。
强制杀进程靠丢失 libfranka 连接触发固件 Reflex，不是受控 stop；确认静止并清空工作区后，运行 `rwrl --station local/station.yaml recover`（NUC、仓库根、项目环境激活）清除 Reflex，而不使用 Desk；若失败，再按 [Supervisor 说明](../../daemon.md#supervising-a-single-process-robot-run) 在 Desk 恢复。记录恢复结果再做 S12。

证据核查：`rwrl archive inspect`（离线，不连接工作站或机器人）只读归档，输出 outcome、mask 计数并导出步骤，并把每个回合按相同 human_action_mask 切分成段（`summary.json` 的 `episodes[].segments[]`：`hold` 或 `intervene`，含以 mm 计的 TCP 漂移、残余位移和接管跳动）。`measurements.csv` 每段一行、每个保持段另有一行 `hold_window`，给出 `--stop-time-s` 时再加一行 `settle`；自填的行（S11）写在其后。
S04/S05/S09 以 `hold_window` 行的 `max_drift_mm` 判≤5 mm，即保持段前 10 s（`--drift-window-s`）内 TCP 到该段首个采样的最大欧氏距离；并由 `start_s`/`end_s`（归档墙钟时间）确认保持至少 10 s，结合视频对齐时间窗。
S06 的 `settle_s` 是从记录的 leader 停止时刻到停稳采样的时间；停稳采样是停止时刻之后第一个满足“自该采样起 TCP 速度持续 0.5 s 低于 2 mm/s”的采样（`--settle-speed-mm-s`、`--settle-hold-s`）；`residual_mm` 是停止时刻与该采样之间的 TCP 距离。原始视频时间段也须填写。若停止时间戳与视频无法可靠对齐则 BLOCKED。
无跳动/误开须有可回看的视频证据；不要只凭 mask 或正奖励判定。`jump_mm` 已输出但未设数值阈值：保留现场观察，争议填 BLOCKED。
S03 使用两次 doctor 运行中 `feedback:gripper` 打印的米制宽度、占 `max_width_m` 的比例和后端来源；保留两份日志，并核对数值与肉眼观察的开口一致。S08/S09 的 autonomous_success 由 completed、task_success、had_intervention 推导，inspect 不直接输出该布尔字段。
S10-int 正常清理应完成 interrupted 记录；S11 强杀允许最后回合未结束，只要求已提交行可读、无伪造成功；不要重开同一 recorder 补尾记录。
所有录制项目都需确认释放：child_exit＋进程退出＋机器人静止；随后下一次 doctor/单一连接成功为补充证据，禁止 doctor 与录制并发。
最终填完 CSV 后仅重新打包（NUC、仓库根、项目环境激活）：`tar -czf "$RUN.tar.gz" -C "$(dirname "$RUN")" "$(basename "$RUN")"`，不要重跑 S12 的录制。
现场发现的问题写入 CSV 问题编号与备注；所有初始行保留 NOT_RUN，完成后只改运行目录内副本。无需本次执行任何实机命令。
