# SpaceMouse commissioning: profiles, task-frame workspace, sessions

For prerequisites, see the `rwrl` [global options](cli.md#global-options) and [connection ownership](cli.md#connection-ownership).

## Site device table

The HID reader decodes a SpaceMouse with the entry of its exact vendor and product id
(`vid:pid`). The built-in table is the PySpaceMouse 1.0.3 table: SpaceExplorer,
SpaceNavigator, SpaceMouse USB, SpaceMouse Compact, SpaceMouse Pro Wireless, SpaceMouse Pro,
SpaceMouse Wireless, 3Dconnexion Universal Receiver, SpacePilot and SpacePilot Pro. Any other
model or firmware variant a site uses goes into that site's local device table,
`spacemouse-devices.yaml`, which stays out of git. When `rwrl discover` or `rwrl doctor` prints

```text
unknown 3Dconnexion-vendor HID device 256f:xxxx; add it to spacemouse-devices.yaml (see docs/spacemouse-commissioning.md#site-device-table)
```

the connected device is not in the table, and `open()` refuses it. The warning covers every
unlisted 3Dconnexion (`256f`) device, and Logitech (`046d`) devices only when their HID
descriptor declares a multi-axis controller, so a keyboard or mouse receiver is not reported.

**Location.** The first rule that applies selects the file:

1. `teleop.options.device_table` of the station robot. A relative path is relative to the
   directory of the station file.
2. `spacemouse-devices.yaml` next to the station file: `local/spacemouse-devices.yaml` for
   `local/station.yaml`, `~/.config/rwrl/spacemouse-devices.yaml` (the pre-0.1.0 `~/.config/robot_infra/` is still read, last) for the user-scope
   station.
3. Without a station (setup probes, a device built from options that name no table, and the
   host-level USB checks and udev rules of `rwrl doctor` when no station is found): the first
   existing of
   `./local/spacemouse-devices.yaml` and `$XDG_CONFIG_HOME/rwrl/spacemouse-devices.yaml` (then the pre-0.1.0 `$XDG_CONFIG_HOME/robot_infra/`).

With none of these, only the built-in table applies. A local entry with the `hid_id` of a
built-in entry replaces it; any other local entry is added.

**Format.** An example that declares `256f:c63a` with the decoding of the built-in
SpaceMouse Wireless entry (an example of the format, not a statement about that product: check
the decoding of the actual unit as described below):

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

`layout:` names a built-in entry, never another local one; fields written next to it
override the copied ones. Without `layout:` the entry spells out its decoding in full:

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

The file is checked when it is loaded (when a SpaceMouse teleop is built, and by `rwrl
discover` and `rwrl doctor`), and one invalid entry rejects the whole file with an error that
names the file and the entry. Top-level keys are exactly `schema_version: 1` and a nonempty
`devices` mapping; entry keys are `hid_id`, `layout`, `led_id`, `mappings`, `button_mapping`,
`axis_scale` and `evidence`. `hid_id` is two integers 0..0xFFFF (or the text `"vvvv:pppp"`),
unique in the file; a local
name may reuse a built-in name only with that entry's `hid_id`. Axis `channel` is 0..255,
`byte1`/`byte2` 1..63 and `scale` -1 or 1; a button has integer `channel`, `byte` (1..63) and
`bit` (0..7), or all three `null`; `axis_scale` is finite and positive.

**Evidence.** Keep what justified an entry next to the table under `local/` (the HID report
descriptor, a short capture of each axis and button, the date) and point `evidence` at it,
relative to the table's directory. `rwrl teleop diagnose` records in its summary which entry
decoded the device (`device_spec`: name, `hid_id`, `source` `builtin` or `local`, and the
table's sha256 for a local entry). Recording and training runs keep the table out of the
compared experiment contract: `teleop_input` records the local table's file, sha256 and entries
as provenance only, and no machine path enters its `source_options`.

**Check.** On the robot host, repository root, project environment active, after editing the
table:

```bash
rwrl --station local/station.yaml discover
rwrl --station local/station.yaml doctor
rwrl --station local/station.yaml --json teleop diagnose --duration-s 5
```

`discover` lists the device under `spacemouse:` with no warning, `doctor` passes
`usb.spacemouse` and the teleop checks, and the diagnosis shows each pushed axis and pressed
button with the expected sign (see [Live input calibration and
diagnosis](#live-input-calibration-and-diagnosis)).

## SpaceMouse profiles and task-frame workspace

A station robot can declare `task_frame` with `name` and a proper 4×4
`base_T_frame` transform. `teleop_profiles` specifies a profile YAML `file`
(relative to the station file) and a `default` profile name. The profile file
contains `schema_version: 1` and a `profiles` mapping. Each profile declares
`frame`, `yaw_rad`, and `axis_mapping_version: serl_hid_axes_v1`;
`neutral_threshold` defaults to 0.05. This startup tolerance is separate from
normal intervention and movement deadzones.

`rwrl workspace frame set` writes `task_frame` instead of hand-editing it. On
the robot host, from the repository with its project environment active,
capture three TCP points with the arm positioned by an operator (the frame
origin, a point on +X, and a point on the +Y side of the X axis), then propose
and save the frame:

```bash
rwrl --station local/station.yaml workspace capture --name frame-origin --output frame-points.yaml --save
rwrl --station local/station.yaml workspace capture --name frame-x --output frame-points.yaml --save
rwrl --station local/station.yaml workspace capture --name frame-y --output frame-points.yaml --save
rwrl --station local/station.yaml workspace frame set --name table \
  --from-points frame-points.yaml --origin frame-origin --x-axis frame-x --y-axis frame-y
rwrl --station local/station.yaml workspace frame show
```

The frame is right-handed: X points from the origin to the X point, Z is normal
to the plane of the three points, and Y is the Y point's direction made
orthogonal to X. The candidate reports both measured axis lengths and the angle
between the measured and final Y directions; an X point closer than 1 mm to the
origin, or three collinear points, is refused. A known transform can be given
instead as `--base-T-frame` followed by 16 row-major numbers (metres). The default
`source` provenance is `points:<file> origin=A x=B y=C`; `--source TEXT` sets
another. Add `--save` to write only `task_frame` after the reviewed diff and
confirmation. A station workspace defined in another frame is refused; reusing
its frame name rebases that workspace, so rerun `workspace validate` and review
the bounds. Points files saved in the old frame must be recaptured.
`workspace frame show` prints `task_frame` with its origin and unit axes in the
base frame.

`teleop_profiles` is a real_world_rl station section, not a `RobotConfig`
field. The device-layer loaders in `real_world_rl.discovery.autoconfig` parse strictly
unless given sections: an unknown `teleop_profiles` key there fails with an error naming
`real_world_rl.station.load_station`. Every `rwrl` command (the daemon, setup, discover
and doctor included) and Python callers use `real_world_rl.station.load_station` (or `load_station_with_env`)
to register its validator. Existing station YAML needs no migration; `rwrl teleop calibrate
--use` continues to write this section in the robot mapping.

From an installed project environment, with your station file available:

```bash
rwrl --station local/station.yaml --json teleop profiles list
rwrl --station local/station.yaml teleop profiles show --profile rear
rwrl --station local/station.yaml teleop profiles use --profile rear
rwrl --station local/station.yaml workspace show
```

Offline calibration accepts a JSON mapping of `+x`, `+y`, `-x`, `-y` to lists
of normalized six-axis samples **after the fixed HID axis mapping**. Positive
labels fit the yaw; negative labels independently validate it. The default
maximum directional residual is 25 degrees (`--max-error-deg`). Results do not
claim that rotation directions have been validated.

```bash
rwrl --station local/station.yaml --json teleop calibrate \
  --samples local/direction-samples.json --profile rear --frame table
```

Add `--save --output profiles.yaml` to save after confirmation. The output path
is relative to the station file. The result includes the `station_reference`
to configure as that robot's `teleop_profiles`. Saving preserves the previous
file in a uniquely named `<file>.<id>.bak` backup with mode `0600`; YAML comments,
quotes and key order outside the edited profile are preserved.
Before confirmation, stderr shows a unified diff against the exact file read.
Changes detected during review or staging reject the
save. This is optimistic conflict detection, not a lock against external editors;
avoid editing the same file during a save. Noninteractive saving requires global
`--yes`.

To recapture just one direction, use `teleop calibrate --profile rear
--samples local/direction-samples.json --capture-label=+x`. This replaces that
direction while retaining the other groups; `--append-trial` instead adds an
independent push. Each push has equal fit weight regardless of its duration or
strength. JSON accepts either legacy sample rows or `{"trials": [rows, rows]}`
per direction. Every held-out trial must pass, including signed rotation trials.
The new report contains `merged-samples.json`; use its path as `--samples` for
the next capture. Original sample files are never overwritten. Without enough
horizontal groups the result is `NEEDS_SAMPLES`, with `required_fields`; no
profile is saved. Without a terminal, labelled capture requires global `--yes`
and starts immediately for `--duration-s` seconds (default 3). Prompts and the
merged file location go to stderr. Mixed/offline samples do not establish a
single physical device identity; only a complete live acquisition binds one.

Raw input JSONL distinguishes newly received HID reports (`fresh_report`) from
polls of cached state. Only new reports contribute labelled fitting samples;
every capture must receive at least one new report. Empty HID reads preserve
state, so silence alone is not interpreted as either release or disconnection.
No-report captures fail with unavailable evidence, and SDK read errors propagate.
This input-only acquisition check does not certify continuous device connectivity
or the freshness of every axis in a split HID report.

Input acquisitions and supervised CLI sessions save private `provenance.json`
before opening input or launching control. It records Python source file hashes
for `real_world_rl` and `rl_garden`, installed distribution names
and versions, and a canonical configuration digest. Session records additionally
hash station, preset, profile and resolved-config files. Paths do not affect a
package's source digest; content edits and file renames do. This is pre-run
on-disk evidence, not an attestation of loaded memory or every dependency binary.
Keep source/configuration unchanged during a run and archive these records with
the raw input, trajectory and stop reports.

For recording/training, supply the station as `--teleop-options-yaml`, select
`--teleop-device spacemouse`, and optionally override `--teleop-profile` and
`--teleop-robot`. Resolved input rotation, profile name/file and yaw are frozen before
saving configuration and retained in non-compared `teleop_input` provenance. Restoring the
run reuses those numeric options; recalibrating the operator does not change the meaning of
base-frame recorded actions. A legacy station `teleop.options.input_rotation` cannot be combined
with a named profile or station `task_frame`. Migrate its mounting component to
the task frame and select a profile for operator yaw. A legacy configuration
without either new field retains its previous input-rotation behavior.

`workspace set --frame table --low X Y Z --high X Y Z` previews a task-frame
position box; `--save` applies it after confirmation. Existing base-frame
`safety_box` constraints remain active. Targets outside the task-frame workspace
are rejected by the device layer, including targets moved outside it by base-frame clipping.
The training environment first clips policy targets in task coordinates and projects excessive
orientation onto the reference cone. In SERL mode, the original XYZ/RPY safety-box clipping
runs first.
Optional `--orientation-xyzw X Y Z W --max-angle-rad A` defines an orientation
reference and an SO(3) angular radius, avoiding Euler branch cuts.

`workspace capture` reads the current TCP and reports base pose and task-frame
position without moving the robot. A captured point is a measurement, not an
accepted boundary. `workspace validate --pose X Y Z QX QY QZ QW` checks a
base-frame target against the configured workspace and base safety box. Out-of-bounds
validation is a result with `valid: false` and violation details: JSON mode exits `0`, text mode
exits `5`. Malformed configuration or input still reports an error.

`workspace show` includes metre/radian/xyzw units, station and calibration source,
the base safety box and the eight oriented-workspace vertices in both its own
frame and the base frame. It also reports `joint_limits`, `max_delta_per_step`,
`home_joints` and `home_within_limits` (`null` when home or limits are not
configured; `home_violations` lists each joint outside its limits). Both constraints apply as an intersection. The vertices
are for inspection; they are not replaced by a larger axis-aligned bounding box
and do not certify collision-free space. This command displays configuration,
not live daemon enforcement or physical calibration acceptance.

On the robot host, from the repository with its project environment active,
save named measurements and preview their extrema as follows (the station must
already define the `table` frame, for example with `workspace frame set` above):

```bash
rwrl --station local/station.yaml workspace capture --name near --output points.yaml --save
rwrl --station local/station.yaml workspace capture --name far --output points.yaml --save
rwrl --station local/station.yaml workspace set --frame table --from-points points.yaml
```

Capture each point only after an operator has positioned the arm safely; these
commands never move it. Point paths are relative to the station directory.
Measurements store base poses, timestamps, metre units and the complete frame
transform. Reusing a name requires the same save review and preserves the old
file in a backup. A changed frame requires recapture. Points must span all three
axes; no padding is invented. Extrema alone do not establish that the enclosed
volume is safe. Review the candidate before repeating `workspace set` with
`--save`; numeric `--low/--high` cannot be combined with `--from-points`.

### Supervised recording sessions

`rwrl teleop run` uses the existing demo recorder under `rwrl supervise`.
Supply a station with a selected SpaceMouse profile and a task preset containing
home and workspace settings. The reset can move the arm and operate the gripper;
the command previews the session and uses the normal motion confirmation.

```bash
rwrl --station local/station.yaml --preset local/task.yaml teleop run \
  --mode tracker --record local/demo.pkl
```

Use `--mode serl` for native SERL takeover semantics; its preset must provide the
required position and RPY limits. `--profile NAME` overrides the station default.
`--num-successes` defaults to 20; `--max-episodes` is optional. Each episode can
contain multiple human/policy handovers. The mode's takeover behavior is unchanged.

After an episode ends, the session holds the last device-accepted target and
ignores puck movement. `n` requests the next episode, including the task's reset;
all six input axes must stay within the selected profile's neutral threshold
(default 0.05) with released buttons for 0.2 seconds. `q` releases control and
exits. Reaching the recording budget disables `n` but continues holding until
`q`. Faults and signals still invoke cleanup. Holding does not add transitions
to the completed trajectory.

The Python recorder retains its existing automatic episode behavior by default.
Pass `--episode-end-behavior hold` to opt into the same session behavior, with
`--neutral-threshold` and `--neutral-stable-s` available explicitly. This currently
requires a `real_robot` Cartesian environment, SpaceMouse, and a separate event
device. Other configurations are rejected before reset.

A private sibling `.session-<id>` directory saves the effective configuration and
session result. With global `--json`, child output goes to stderr. Actual normal
stop stability still needs independent physical verification; a successful
process exit is not a measured stop-acceptance result.

### Live input calibration and diagnosis

Omit `--samples` to run the interactive calibration wizard. It opens only the
SpaceMouse, asks for signed task-frame translations and rotations, and records
three seconds per label by default (`--duration-s`). The prompts describe the
intended TCP direction; the operator chooses the natural push direction from
their current position. Rotation signs follow the right-hand rule. The robot
is never connected by this wizard.

```bash
rwrl --station local/station.yaml teleop calibrate \
  --profile rear --frame table --output profiles.yaml --use
rwrl --station local/station.yaml --json teleop diagnose \
  --duration-s 5 --profile rear
```

`--use` saves the candidate and, after review, selects it in the station.
Without `--save` or `--use`, calibration only produces a candidate. If station
saving is cancelled after profile saving, the saved profile remains available
but is not automatically selected.

Input diagnosis records decoded axes, the fixed axis mapping, task/base-frame
inputs, buttons, report counters and timestamps. An input-only diagnostic never
commands the arm. Reports are stored under `local/teleop/` in a writable source
checkout or the user's state directory; `--report-dir` overrides the parent.
Each run has a private directory containing JSONL samples, labelled JSON samples
and a summary. Failed acquisition preserves an error report. An absent rotation
label is reported as unvalidated, rather than treated as a passing test.

Live interactive acquisition needs a terminal. Agents can fit the same labelled
JSON format with `--samples`; `--yes` does not manufacture human direction
labels. Supported SpaceMouse readers take an advisory device-path claim, so a
second project reader is refused until the first closes. Diagnostics also check
for existing hardware process owners and do not take over their devices.

Every reset in a holding session is followed by a neutral gate: the arm holds
its post-reset measured pose until the puck and buttons remain neutral for
`neutral_stable_s` (default 0.2 seconds). `q` can exit during this gate. The gate
does not run when handing control between policy and operator within a trajectory.

For agent-driven event input, add `--events-file local/events.jsonl` to
`rwrl teleop run`, or `--events-file -` for a pipe on stdin. Create the file
before launching and append one JSON object per line, for example
`{"event":"takeover"}`. Supported events are `takeover`, `handback`, `success`,
`failure`, `rerecord`, `end`, `next` and `quit`. Tracker uses takeover/handback;
SERL retains motion-based intervention. This input replaces terminal keys.
Wait for `EPISODE_HOLD` before sending `next`; reset can move/home the robot.
Events are consumed in order, at most one per control poll. A regular file is
tailed; closing a pipe before `quit` is a fault. Malformed or unknown events
fail rather than silently continuing. Direct Python/training entry points use
`--teleop-event-device jsonl --teleop-events-file <path-or-dash>`.

Saved normal-stop evidence can be evaluated without connecting to hardware or
loading a station. From the repository on any machine with its project environment
active:

```bash
rwrl --json teleop inspect-stop --trace local/teleop/stop-trace.json
```

The schema-1 JSON trace contains `stop_start_s` and `stop_end_s` on the host
monotonic clock, and a `samples` list. Each sample supplies `monotonic_s`,
`device_time_s` (seconds, advancing SDK time), `tcp_pose` (base XYZ metres and
xyzw quaternion), `joint_vel` (rad/s, all seven Franka joints), `robot_mode` (SDK name), and
`errors` (list). The stop-start boundary records `control_mode` and `tracker_active`.
An already idle/inactive arm is BLOCKED with `no active motion at stop`; stationary samples
alone cannot establish an active-control stop. Unknown/missing robot modes block acceptance;
fault modes fail it. `inspect-stop` exits `0` for PASS, `3` for FAIL and `4` for BLOCKED;
general errors retain their existing codes.
`joint_count` can declare another arm's joint count. Include the confirmed
`workspace` and optional `task_frame` using the station configuration schema;
missing workspace evidence cannot pass. Preserve `stop_error` if stopping failed.

The assessment includes signed minimum, maximum and final XYZ displacement in
the base frame. When the trace supplies `task_frame`, it also reports task-frame
components, maximum downward/upward Z displacement and maximum horizontal
displacement. These values use the last pre-stop sample as reference and include
intermediate peaks, not just the final pose. Task Z represents physical vertical
only when the operator has verified that frame calibration. Without a task
frame, these metrics are explicitly unavailable; base Z is never assumed to be
vertical. Component metrics do not replace the full 3D acceptance threshold.

Defaults are 10 mm maximum sampled 3D displacement, 5 degrees maximum SO(3)
rotation, all joint speeds at most 0.02 rad/s over a final 0.5-second window,
at least 2 seconds after stop completes, and at most 0.1 seconds between samples.
A stationary pre-stop window of 0.5 seconds establishes the reference. Supply a
JSON object with `--thresholds` to override `displacement_m`, `angle_deg`,
`joint_speed_rad_s`, `stationary_s`, `post_stop_s` or `max_sample_gap_s`;
all effective thresholds appear in the report.

Known violations produce `FAIL`; insufficient or stale evidence produces
`BLOCKED`. Exit code is 0 for `PASS`, 3 for `FAIL`, 4 for `BLOCKED`, 2 for invalid input.
Results include the input file's SHA-256. Evaluation neither smooths nor changes
samples. `PASS` applies to supplied samples and cannot establish a continuous-time
maximum. Two endpoint snapshots alone are insufficient. This command evaluates
evidence; it does not collect a trace or certify that a physical stop is fixed.

HIL-SERL, residual HIL-SERL and SiLRI training actors can also opt in with
`--episode-end-behavior hold`, `--neutral-threshold 0.05` and
`--neutral-stable-s 0.2`. Their default remains `auto`. This requires the same
Cartesian real-robot backend and SpaceMouse composite event source as recording.
Each episode is sealed before holding; holding adds no transitions or policy
steps. Explicit `next` permits reset, then neutral input is checked again.
FWBW rejects this option because its reset-free episode handling is different.
Python users can call `actor.configure_episode_session("hold", ...)` before
`actor.run()`. A finite step budget disables next and holds until quit, including
when the last step did not terminate the episode.

Live calibration records USB vendor/product identifiers and the serial number
when the device supplies one. Both tracker and SERL sources check this identity
when opening input, before an episode reset. A mismatch refuses operation and
releases input owned by the source; caller-owned input is not disconnected.
If a device supplies no serial number, matching vendor/product IDs establish
only the model, not the individual unit. Revalidate direction mapping after
replacing such a device. Offline profiles without device identity retain their
explicitly unbound behavior; identity is never guessed from a device index.

Task geometry is part of the experiment contract when configured. Its numeric
transform, oriented bounds, optional SO(3) orientation limit and feedback margins are recorded
when a workspace exists. A task frame alone is for display/sampling and adds no spatial
semantics. Frame labels and provenance text do not change the numeric contract. Legacy
configurations without a workspace retain their previous contract. Deploy restores task frame
and workspace from training configuration and rejects conflicting CLI geometry.

For remote training, supply `real_robot.workspace` and `real_robot.task_frame`
matching the daemon's station geometry. `rwrl teleop run` fills these from the
selected station. Before reset, the environment requires the daemon to advertise
`task_workspace_v1` and checks its enforced geometry against the expected
workspace. Missing capability or mismatching geometry refuses reset; no client
silently substitutes a weaker box. Local robot dictionaries supply the expected
geometry directly. Existing base-frame safety limits continue to apply as well.

For the new task workspace, the environment checks measured TCP feedback before
reset motion, before each Cartesian step and after observations, as well as during
session holding. The per-step check reuses the last raw observation rather than issuing an
extra arm-state RPC. Feedback checks allow `measured_margin_m` (default `0.003 m`) and
`measured_margin_rad` (default `0.03 rad`); target checks retain `1e-9` tolerance.
A measured violation beyond these margins latches a fault and attempts to stop. It
refuses further steps and automatic resets on that environment instance; inspect
the robot and feedback before starting a new session. It does not drive the TCP
back into the box. Joint-action environments reject the task workspace because
these bounds cannot establish containment of a joint trajectory's TCP path.
These sampled TCP checks are not whole-arm collision checking. Existing base-box
clipping semantics are unchanged when no task workspace is configured.

`rwrl teleop run` enables normal-stop evidence through
runtime-only `RWRL_STOP_TRACE_PATH` injection. `RWRL_DIAGNOSTICS_PATH` and
`RWRL_SESSION_EVENTS_PATH` select the other runtime evidence destinations; all default off.
These paths are absent from `RealRobotEnvConfig` and persisted configuration. Python callers
pass a separate `RuntimeReportPaths(stop_trace=..., diagnostics=..., session_events=...)`
to `RealRobotEnv(..., report_paths=...)`. When `report_paths` is omitted, the environment
reads these environment variables through `RuntimeReportPaths.from_environ()`. The backend
must advertise `stop_trace_v1` before reset (currently Franky and an updated
Franka daemon). It samples on the existing connection during control, retaining
a rolling pre-stop window. Explicit normal exit captures the stop and observes
for 2.1 seconds before releasing the connection. `stop-trace-<timestamp>-<id>.json` and
its `.assessment.json` companion are written in the session report directory;
`summary.json` includes the assessment. Known failed acceptance makes the child
exit unsuccessfully after evidence export; insufficient evidence remains BLOCKED.

CLI sessions also write private `session-events-<timestamp>-<id>.jsonl`, separate from trajectory
transitions. Each schema-1 row has sequence, monotonic time, episode index, phase
and reason. Phases are `READY` (awaiting reset/neutral), `ACTIVE`, `EPISODE_HOLD`,
`STOPPING`, `CLOSED` and `FAULT`. `CLOSED` means cleanup returned successfully,
not that hardware stop acceptance passed. An interrupted process may leave the
last phase unfinished; absence of `CLOSED` must not be inferred as release.
Python recorder/actor users can inject `RuntimeReportPaths(session_events=...)` at runtime. Filenames are unique
within the run directory, and a second environment `close()` does not export again. Cleanup
records the active exception's type; an earlier `operator_quit` STOPPING row is not duplicated.

Faults and interrupts use immediate cleanup, skipping the normal observation
wait. Daemon stop observation permits stop/read calls and rejects new control
writes. The existing SDK stop law is unchanged. Reported mass/centre-of-mass and
SDK/server metadata are observations, explicitly not verified payload calibration.
Stop metadata also fingerprints SDK facade, implementation and superclass module
files with SHA-256 (including native modules behind Python wrappers). Missing
files are recorded as unavailable. These are on-disk fingerprints, not an
attestation of loaded memory or linked system libraries. Reported model evidence
includes available mass, centre-of-mass, inertia, flange-to-end-effector and
end-effector-to-stiffness transforms; absent SDK fields remain null.
Environment-exported traces include `metadata.session_exit`: the known exit
reason, episode index and environment-configuration SHA-256. Explicit q in an
active trajectory or hold records `operator_quit`; other normal closes remain
`normal_close`. The context does not itself prove that stopping succeeded.
Python callers can use `begin_stop_trace()`, then `stop_with_trace()` on the same
connected Franky/remote arm; `last_stop_trace` remains available when stop raises.
After a successful traced stop the backend is IDLE before post-stop observation,
so an expired daemon action deadline does not initiate another stop. New motion
or servo-mode entry is refused during observation; plain stop remains available.
Resuming later requires an explicit control-mode selection. A failed stop does
not falsely mark the backend idle or disable the watchdog's fault fallback.

To commission robot motion, use the NUC repository with its project environment
active, an already reviewed station workspace and a task preset:

```bash
rwrl --station local/station.yaml --preset local/task.yaml teleop run \
  --diagnose translation --duration-s 10 --record local/diag-translation.pkl
```

Choose `translation`, `rotation` or `all`; `--mode tracker|serl` selects the
original control mode. `--gain` defaults to 0.2 and scales input relative to the
preset's control settings; it is not an absolute velocity guarantee. The default
normal `teleop run` keeps gain 1 and all six axes. Diagnostic filters and gain
enter the experiment contract so these data are distinguishable from normal data.
`teleop diagnose` only inspects input. Robot motion diagnostics use
`teleop run --diagnose translation|rotation|all`; `--gain` (default 0.2) and
`--duration-s` (default 5) affect only diagnostic sessions. Without `--diagnose`,
both options are ignored. Diagnostic sessions default to one success and at most
one episode; normal sessions retain 20 successes and no episode-count limit.

Motion diagnostics use the same supervisor, confirmation, recorder, neutral gate
and episode holding as a normal session. Duration determines a nominal control-step
budget, not a hard wall-clock deadline. At the budget the episode holds until `q`;
it does not silently release control. `--events-file` also works for agent events.

Session `diagnostics-<timestamp>-<id>.jsonl` records the environment input action, requested pose,
environment-limited pose, device-accepted pose and measured base/task pose with a
monotonic timestamp. Reasons distinguish environment pose limits from device
limits. These input actions are normalized control actions, not raw HID reports;
use input-only diagnostics for decoded HID evidence. Log output paths do not alter
experiment semantics. The additional remote accepted-target query is performed
only when diagnostics are enabled.

Diagnostic rows distinguish `kind: step` from `kind: hold`. Hold rows keep the
episode step counter unchanged and have null action/input fields; they are not
trajectory transitions. `measurement_phase` is `before_command` for hold (the
feedback checked before refreshing the target), and `after_command` for normal
steps. Hold logging reuses the accepted-target read already required to verify
holding, rather than issuing another target query. A log write failure propagates
to cleanup; a missing row must not be treated as evidence of a successful hold.

A failed send records `kind: command_failed` with attempted targets and the
error when logging remains available. Execution status is `unknown`; accepted
target and measured pose are null because a transport error cannot prove whether
the device executed the command. No additional device query is made on this
error path. If evidence writing also fails, the original device error remains
the raised failure. Logging uses synchronous writes/flushes, so validate its
latency on the actual run filesystem and transport; it is not a realtime
callback and does not extend watchdog deadlines.

The per-step `teleop` field associates the consumed SpaceMouse sample with its
control row: sample sequence/time, decoded HID axes in the declared order, HID
report count/time when available, mapped device input and base input after the
selected rotation/filter. It also records whether the sample intervened. Metadata
is copied with that sample, not fetched from a later asynchronous read. Missing
reader evidence remains null. `reset_index` distinguishes resets whose step
counters restart. Policy-only rows without a teleop sample leave `teleop` null.
