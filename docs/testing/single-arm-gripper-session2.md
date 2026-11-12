# Second single-arm and gripper hardware session: GELLO / SpaceMouse

Scope: one NUC, one arm and a physical Franka Hand, connected directly by a single recorder child; no training. This page does not claim hardware acceptance.
Run every command on the **NUC, repository root, in Bash**. S01 needs no environment active; after it succeeds, activate the environment the installer created (`.venv` by default), for example `source .venv/bin/activate`; all later commands use the project environment.
First follow [tutorial 01](../tutorial/01-install.md): power on, unlock, power the End Effector, and check the Control cable and Desk. Verify robot identity and the physical Hand; do not infer them from a default IP.
Follow [tutorial 04](../tutorial/04-record-demos.md#prepare-the-nuc) to commission the actual IP, home, limits, workspace and GELLO calibration. `gello_leader.yaml` must point to a separate fake-arm leader YAML, never a second connection to the physical arm.
The preset is state-only, 50 Hz, cartesian_delta, with scales 0.05 m / 0.3 rad, lead caps 0.05 m / 0.3 rad, and target speeds 0.05 m/s / 0.3 rad/s. Confirm the physical stop works first.
After each item, fill the run directory's [record CSV](single-arm-gripper-session2-record.csv): enter planned date, actual start time and observations on site. Initial status is NOT_RUN; missing evidence is BLOCKED; a measured limit violation is FAIL.
During measurements, video both the arm and a timestamped terminal; store the video in the item's directory. Do not relax thresholds or workspace bounds for the test.

Prepare on the NUC, repository root (no venv required). Create a fresh RUN each time; never reuse an existing item directory:

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

`record` allows one episode by default; S10-r allows two episodes and therefore two resets. No pkl after failure is normal. Press `s` while intervening so the human demonstration includes the successful terminal step.
`exit-code.txt` holds the command exit code, not the check conclusion. For abnormal-exit tests, judge the evidence below even when the command exits nonzero.
Run the table in order, one command per row. Record paths are relative to `$RUN`; keep console, supervisor, SQLite, steps and video evidence alongside them.

| ID / item | Command | Procedure and pass criterion | Main record file |
|---|---|---|---|
| S01 Installation and diagnosis | `check S01 bash scripts/install.sh --extras franka-gello,spacemouse --station local/station.yaml` | No doctor layer reports FAIL; every required check for the actual devices passes. Activate the environment the installer created (`.venv` by default) afterward | `S01/console.log` |
| S02 Desk / FCI | `check S02 rwrl doctor --station local/station.yaml --preset "$PRESET"` | Verify all six: page reachable, green light, logged in, local control, FCI installed, session activated. Refresh the correct Desk; handshake OK; retain evidence of all six | `S02/console.log` |
| S03 Hand feedback | `check S03-open rwrl doctor --station local/station.yaml --preset "$PRESET"`, then `check S03-closed rwrl doctor --station local/station.yaml --preset "$PRESET"` | Run doctor twice: once with the Hand open, then again after closing it in Desk. Both reported widths must match the visible opening (Franka Hand open about 0.08 m, closed near 0), with no WARN | `S03-open/console.log`, `S03-closed/console.log` |
| Optional after S03 | `rwrl gripper show` | On this NUC, repository root, project environment active, before starting a recorder: expect the same gripper width line as doctor (metres, normalized width, grasp state and backend source); keep the S02/S03 doctor checks above | Optional comparison; no CSV item |
| S04 Reset and hold | `record S04 leader_cartesian --max-episode-steps 750` | After reset, do not take over for at least 10 s; the `hold_window` row of `measurements.csv` (written by the inspector; also `max_drift_window_mm` of the hold segment in `summary.json`) spans at least 10 s and its `max_drift_mm` is ≤5 mm | `S04/measurements.csv` |
| S05 GELLO t/h/t | `record S05 leader_cartesian` | Drive with `t`, hold with `h` for 10 s, resume with `t`, then `q`; the `hold_window` row of the `h` hold has `max_drift_mm` ≤5 mm, no jump on takeover (`jump_mm` of the following intervene segment in `summary.json`, confirmed on video), and the `summary.json` segments match takeover/hand-back | `S05/measurements.csv` |
| S06 GELLO lead cap | `record S06 leader_cartesian` | Move the leader quickly within the approved range, then stop it and mark the leader stop time in a second terminal (below). Finish with `q`, then rerun the inspector with that stop time: the `settle` row has `settle_s` ≤1.5 s and `residual_mm` ≤50 mm | `S06/measurements-stop.csv` |
| S07 GELLO gripper S3 | `record S07 leader_cartesian` | Open/close unloaded, finish closed; hand back with `h`, release trigger, then `t`. Gripper stays closed until trigger matches; no unintended opening; finish with `q` | `S07/console.log` |
| S08 GELLO successful pick/place | `record S08 leader_cartesian` | Pick/place, then `s` while intervening; pkl, contract and SQLite exist, task_success=true and had_intervention=true, hence autonomous_success=false | `S08/summary.json` |
| S09 SpaceMouse combined check | `record S09 spacemouse` | Puck motion alone never takes over. Button 0 takes over/hands back; hand-back hold drift ≤5 mm over 10 s, no jump on takeover; button 1 never opens unintentionally. Press `s` during pick/place; files and outcomes as S08 | `S09/summary.json` |
| S10-f Failure | `record S10-f leader_cartesian` | Press `f`; archive finalized, task_success=false / manual_failure, no successful pkl, process exits and releases devices | `S10-f/summary.json` |
| S10-r Rerecord | `RECORD_ATTEMPTS=2 record S10-r leader_cartesian` | First episode: `r`, discarded=true. Second: fresh ID and reset, then `q`; both episodes finalized, no successful pkl, process exits and releases devices | `S10-r/summary.json` |
| S10-timeout Timeout | `record S10-timeout leader_cartesian --max-episode-steps 500` | No takeover or label; truncated after about 10 s, no success label, archive finalized, process exits and releases devices | `S10-timeout/summary.json` |
| S10-int Ctrl-C | `record S10-int leader_cartesian` | Ctrl-C during recording; supervisor forwards the signal and waits for cleanup, archive marked interrupted, process exits and releases devices | `S10-int/summary.json` |
| S10-q Quit | `record S10-q leader_cartesian` | Press `q` without a success label; operator_quit event and episode end recorded, no successful pkl, process exits and releases devices | `S10-q/steps.csv` |
| S11 Supervised stop | `record S11 leader_cartesian` | After reset, with unloaded low-speed following and stable step heartbeats, issue STOP below. Child must terminate within ≤8 s of STOP (defaults: 2 s heartbeat timeout + 3 s SIGINT + 2 s SIGTERM, then SIGKILL; a stopped process responds only to SIGKILL), robot stops and Desk can recover. Record elapsed time; missing timing evidence is BLOCKED, exceeding the threshold is FAIL | `S11/measurements.csv` |
| S12 Home / disconnect / archive | `record S12 leader_cartesian --max-episode-steps 1 && tar -czf "$RUN.tar.gz" -C "$(dirname "$RUN")" "$(basename "$RUN")"` | Return to approved home. `rwrl --station local/station.yaml --preset "$PRESET" home go` also returns home without recording, once no recorder is running; it moves the arm, applies `task.reset.gripper` and asks for confirmation. After process exit, confirm the robot is still and the connection released before physical disconnection. Archive the entire run directory; repack after updating the CSV | `$RUN.tar.gz` (beside RUN) |

For S06, prepare a second terminal on the NUC, repository root, project environment active, while `record S06` runs. Type the command below and press Enter the moment the leader comes to rest; keep the terminal in the video so the stamp can be checked against it:

<!-- tutorial-test: skip -->
```bash
RUN=$(cat local/session2-current-run)
date +%s.%N | tee "$RUN/S06/leader-stop.txt"
```

After the recorder exits, rerun the inspector on the same archive with that stop time (new output files; the inspector never overwrites evidence):

<!-- tutorial-test: skip -->
```bash
rwrl archive inspect "$RUN/S06/demo.trajectory.sqlite3" --stop-time-s "$(cat "$RUN/S06/leader-stop.txt")" --measurements "$RUN/S06/measurements-stop.csv" > "$RUN/S06/summary-stop.json"
```

`summary-stop.json` lists the result under `settle`; `settled: false` or a `problem` (the stop time lies outside the episode, or the arm never came to rest before the episode ended) is a FAIL or BLOCKED, not a PASS. With `--stop-time-s` the command also exits 0 when every stop settled, 3 when one did not and 4 when a stop time has no TCP episode spanning it; record that code alongside the JSON.

For S11, use a second terminal on the NUC, repository root, project environment active. Proceed only after reset, while following unloaded at low speed on an approved path with stable step heartbeats. Never STOP the control box or supervisor:

<!-- tutorial-test: skip -->
```bash
RUN=$(cat local/session2-current-run)
PID=$(python -c 'import json,sys; rows=[json.loads(s) for s in open(sys.argv[1])]; print(next(r["pid"] for r in rows if r["event"]=="spawn"))' "$RUN/S11/supervise.jsonl")
ps -p "$PID" -o pid=,args= | tee "$RUN/S11/process.txt"
[[ $(ps -p "$PID" -o args=) == *"rwrl record"* ]] || exit 1
python -c 'import json,sys,time; print(json.dumps({"pid":int(sys.argv[1]),"t_wall":time.time(),"t_monotonic":time.monotonic()}))' "$PID" | tee "$RUN/S11/stop-request.json"
kill -STOP "$PID"
```

S11 starts escalation only **after the default 2 s heartbeat timeout**, then waits 3 s after SIGINT and 2 s after SIGTERM before SIGKILL. A stopped process cannot respond to SIGINT/SIGTERM; expected SIGKILL is about 7 s after STOP, which motivates the 8 s criterion.
Enter stop-request, the supervisor child_exit time and the video robot-stop time in measurements. Records from before STOP do not prove physical stopping afterward. Do not change default timeouts to pass.
Forced termination relies on loss of the libfranka connection triggering firmware Reflex, not a controlled stop. After confirming stillness and clearing the workspace, run `rwrl --station local/station.yaml recover` (NUC, repository root, project environment active) to clear the Reflex instead of using Desk; if that fails, recover in Desk per [Supervisor](../devices/README.md#supervising-a-single-process-robot-run). Record recovery before S12.

Evidence review: `rwrl archive inspect` (offline; no station or robot connection) reads the archive read-only, reports outcomes and mask counts, exports steps and splits each episode into segments of identical human_action_mask (`summary.json` `episodes[].segments[]`: `hold` or `intervene`, with TCP drift, residual and takeover jump in mm). `measurements.csv` gets one row per segment, one `hold_window` row per hold segment and, with `--stop-time-s`, one `settle` row; add your own rows (S11) below them.
For S04/S05/S09, the `hold_window` row's `max_drift_mm` is the maximum Euclidean TCP distance from the hold segment's first sample over its first 10 s (`--drift-window-s`); compare it with ≤5 mm, and check from `start_s`/`end_s` (archive wall time) that the hold lasted at least 10 s. Align the window with the video.
For S06, `settle_s` runs from the recorded leader stop time to the first sample after which the TCP speed stays below 2 mm/s for 0.5 s (`--settle-speed-mm-s`, `--settle-hold-s`); `residual_mm` is the TCP distance between the stop time and that sample. Enter the raw video interval as well. If the stop stamp and the video cannot be reliably aligned, use BLOCKED.
No-jump/no-unintended-opening claims require reviewable video, not mask counts or positive rewards alone. `jump_mm` is reported without a numeric threshold: retain observations and mark disputes BLOCKED.
S03 uses the `feedback:gripper` width in metres, fraction of `max_width_m` and backend source from both doctor runs; retain both logs and compare the values with the visible opening. For S08/S09, autonomous_success is derived from completed, task_success and had_intervention; inspect does not print that boolean directly.
S10-int should finalize an interrupted outcome during normal cleanup. S11 may leave the final episode unfinished after forced termination; committed rows must remain readable with no false success. Do not reopen the same recorder to manufacture an ending.
Every recording item must confirm release: child_exit, process exit and a still robot. A later doctor or single connection succeeding is supplementary evidence; never run doctor concurrently with recording.
After completing the CSV, only repack (NUC, repository root, project environment active): `tar -czf "$RUN.tar.gz" -C "$(dirname "$RUN")" "$(basename "$RUN")"`; do not repeat S12's recording.
Enter discovered issues in the CSV issue ID and notes. All template rows remain NOT_RUN; update only the run-directory copy. No hardware commands need to be executed while preparing this page.
