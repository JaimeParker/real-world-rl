Reference only: the first hardware session followed the long form of this page; keep for the next bimanual session, not maintained with the tutorials.

# Bimanual Franka / GELLO session

Run on the **NUC / Ubuntu control PC, repository root, Bash**; P01 needs no venv, then `source .venv/bin/activate` for all subsequent commands/terminals. No learner or checkpoint. This reference is not evidence of hardware acceptance.
Before P02, commission `local/station.smoke.yaml` with [setup](../../robot_infra/README.md): actual left/right IPs, Hands, home, joint limits, safety boxes, per-step limits, distinct stable GELLO ports and complete calibrations; copy `examples/presets/bimanual_gello_smoke.yaml` to `local/bimanual.smoke.yaml` and supply per-arm joint_limits/tokens. Inspect static leader readings before connecting followers.
Keep station daemons at 127.0.0.1:50051 / :50052, cameras empty, teleop.hold_s=0.0, intervention=toggle, one confirmed evdev keyboard event_device for both arms, toggle KEY_F1 left / KEY_F2 right, and episode_end/success/failure/rerecord KEY_ESC/KEY_S/KEY_F/KEY_R. Preserve preset device-side events (no stdin override), joint_delta, 10 Hz, joint_scale_rad=0.01, leader joint_abs and arm onset tolerance 0.1 rad.
Confirm physical stop, clear both simultaneous home/reset paths and separate workspaces; there is no inter-arm collision check. Trigger and gripper must match before takeover; this joint-space route has no cartesian gripper latch. Reset opens grippers and clears takeover. Stop on unexpected motion or opening; retain evidence before retrying.
Choose a fresh RUN_ROOT below; setup refuses an existing directory. Fill only its [record CSV](bimanual-gello-session-record.csv), with PASS/FAIL/BLOCKED/NOT_RUN; missing prerequisites/evidence mean BLOCKED. Keep configuration, software/device inventory and timestamped video with case IDs; command exit code is not the conclusion.

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
  python -m robot_infra.supervise --log "$trial_dir/supervise.jsonl" -- python examples/record_real_demos.py --config local/bimanual.smoke.yaml --output-path "$trial_dir/demo.pkl" --run-id "$(basename "$trial_dir")" --num-successes 1 --max-episodes 1 --max-episode-steps "$step_cap" 2>&1 | tee "$trial_dir/recorder.log"
  local status=("${PIPESTATUS[@]}")
  rc=${status[0]}; if [[ $rc == 0 && ${status[1]} != 0 ]]; then rc=${status[1]}; fi
  printf '%s\n' "$rc" > "$trial_dir/exit-code.txt"
  if [[ -f "$trial_dir/demo.trajectory.sqlite3" ]]; then
    python examples/inspect_teleop_archive.py "$trial_dir/demo.trajectory.sqlite3" --csv "$trial_dir/steps.csv" > "$trial_dir/summary.json" || return
  fi
  return "$rc"
}
```

Run P00–P03-R with daemons off. After both doctor checks pass, start this in a second terminal (same PC/root/venv) and retain its log; never run doctor concurrently. The 5 s daemon watchdog is a separately approved debugging value, not the supervisor heartbeat timeout:

<!-- tutorial-test: skip -->
```bash
RUN_ROOT=$(cat local/bimanual-current-run)
python -m robot_infra.server --config local/station.smoke.yaml --watchdog-s 5 2>&1 | tee "$RUN_ROOT/daemon.log"
```

`record_case` prompts GO before each reset, allows exactly one attempt (default 1200 steps), runs the supervisor, captures exit status and exports archive evidence. Reference expansion of its T00 recorder invocation follows; use the table's helper, which also captures/exports evidence, rather than running this a second time:

```bash
python -m robot_infra.supervise --log "$RUN_ROOT/T00/supervise.jsonl" -- \
    python examples/record_real_demos.py --config local/bimanual.smoke.yaml \
    --output-path "$RUN_ROOT/T00/demo.pkl" --run-id T00 \
    --num-successes 1 --max-episodes 1 --max-episode-steps 1200
```

| ID / item | Command | Procedure and pass criterion | Main record file (relative to RUN_ROOT) |
|---|---|---|---|
| P00 Baseline and inventory | `check P00 bash -c 'git rev-parse HEAD; git diff; uname -a; ip -br address; ip route'` | Commit/local changes, both robot models/firmware/server versions, Hand/load/TCP, FCI IPs and GELLO serial/calibration inventory recorded | `P00/console.log` |
| P01 Environment and SDK | `check P01 bash scripts/install.sh --extras franka-gello --no-doctor` | Install succeeds; real-time setup and dialout/input permissions verified for this PC. Activate .venv afterward | `P01/console.log` |
| P02 Configuration and static release | `check P02 read_leaders` | Move each leader joint; verify calibration/direction/static readings and F1/F2/S/F/R/Esc events; Ctrl-C closes leaders. Approve homes/limits, disjoint workspaces and simultaneous reset paths | `P02/console.log` |
| P03-L Left doctor | `check P03-L python -m robot_infra.doctor --config local/station.smoke.yaml --robot left` | Left SDK, permissions, handshake, feedback and mode checks pass; daemons remain off | `P03-L/console.log` |
| P03-R Right doctor | `check P03-R python -m robot_infra.doctor --config local/station.smoke.yaml --robot right` | Right SDK, permissions, handshake, feedback and mode checks pass; daemons remain off | `P03-R/console.log` |
| T00 Reset and idle hold | `record_case T00` | Approved homes/open grippers; moving inactive leaders does not move followers; Esc finalizes exactly one unknown/truncated episode and exits | `T00/summary.json` |
| T01 Left takeover and hand-back | `record_case T01` | Align then F1: correct left joints/gripper, right holds; F1 hand-back holds; out-of-tolerance onset waits for alignment; Esc; left mask matches video | `T01/steps.csv` |
| T02 Right takeover and hand-back | `record_case T02` | T01 mirrored with F2: correct right following/gripper, left holds, right hand-back holds; Esc; right mask matches | `T02/steps.csv` |
| T03 Both sides and separate hand-back | `record_case T03` | Align both, F1/F2 drive separately; F1 hands left back while right follows, then F2 hands right back; Esc; both/right/idle masks match | `T03/steps.csv` |
| T04 Left soft-object pick/place | `record_case T04` | F1 pick; hand back while gripping without a drop, match trigger before re-taking; place securely, S while intervening; SQLite/pkl/contract present | `T04/summary.json` |
| T05 Right soft-object pick/place | `record_case T05` | T04 mirrored with F2; held object retained during hand-back, placed securely before S; successful human demo and sidecar present | `T05/summary.json` |
| T06 Failure | `record_case T06` | After steps press F: task_success=false/manual failure, complete SQLite, no successful pkl, one episode then exit/release without another reset | `T06/summary.json` |
| T07 Rerecord | `record_case T07` | After steps press R: discarded=true, complete archive, no successful pkl, one episode then exit/release without another reset | `T07/summary.json` |
| T08 Timeout | `record_case T08 30` | No takeover; 30 steps at 10 Hz is about 3 s of stepping, truncated/unknown outcome; one episode then exit without another reset | `T08/summary.json` |
| T09 Ctrl-C after stepping | `record_case T09` | With takeover off and stepping begun, Ctrl-C cleans up, releases control and records interrupted; one episode, exit, no next reset | `T09/summary.json` |
| CLOSE Evidence review and archive | `tar -czf "$RUN_ROOT.tar.gz" -C "$(dirname "$RUN_ROOT")" "$(basename "$RUN_ROOT")"` | Recorder exited, robots still, daemons stopped; steps/masks/outcomes match video; CSV completed; configurations, SQLite/WAL/SHM, videos and logs archived | `$RUN_ROOT.tar.gz` |
| X01 Optional RGB | `record_case X01` | Only after mandatory checks pass: add commissioned unique cameras to a fresh local env config; left/right device/env/obs.rgb ordering agrees; separate archive and snapshots | `X01/summary.json` |

Pass T00–T03 unloaded before T04–T05 with light, soft objects. F1/F2 toggle their side including its gripper; S/F/R/Esc label the whole episode. Press S while intervening; F/R/Esc/timeout end the one allowed attempt, and Ctrl-C cleans up. No pkl without a kept human success is expected; do not confuse a successful check with autonomous task success. No active cable-disconnect test.
Validate one episode_start/end, continuous step numbers, ordered timestamps and outcomes against video (an interrupt before reset/first step may have no episode; record that separately and repeat after stepping). State is [left joints7, left gripper, right joints7, right gripper]; action is [left joints7, right joints7, left gripper, right gripper]. Human masks: idle 0000000000000000, left 1111111000000010, right 0000000111111101, both 1111111111111111. override_mask can include held dimensions without human control; joint angles are radians.
The inspector is read-only and does not finalize unfinished episodes. Retain recorder.log, exit-code.txt, supervise.jsonl, station.yaml/env.yaml, SQLite, steps.csv, summary.json, success-only pkl/contract and video intervals. For X01, finish with S while intervening to verify RGB demo export as well. Before CLOSE, confirm exit/stillness, stop daemons, fill CSV, close database writers, and retain any WAL/SHM alongside SQLite. WAL synchronous=NORMAL covers process crashes, not every power-loss update. Archive again after any CSV change; never reuse a prior item directory.
