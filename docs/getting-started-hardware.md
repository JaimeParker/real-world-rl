# Getting started on hardware

A first-day walkthrough from an empty robot PC to a deployed policy: configure the station, collect
demos with a GELLO leader arm, train a BC policy, run HIL-SERL, residual RL or SiLRI, then deploy.
It covers one Franka arm (the default track) and two arms (callouts marked "Two arms:"). Each step
says what to run, what you should see, and what to do if you do not.

Everything hardware-side in this repository has been verified against fakes only (see
[Known limits](#10-known-limits)). Treat the first runs as commissioning: keep a hand on the
emergency stop, start with the robot in a clear workspace, and read each step's output before moving
on.

The guide uses these names; replace them with your own:

| Name | Meaning |
|---|---|
| `ROBOT_PC` | the machine wired to the arm(s), cameras and GELLO(s); runs the robot daemon |
| `LEARNER_IP` | the GPU machine that runs the learner (and BC training); may be the robot PC |
| actor machine | the machine with the keyboard that runs the actor and the recorder (often the robot PC) |

On a single machine use `127.0.0.1` wherever the guide says `ROBOT_PC` or `LEARNER_IP`.

Contents: [1 Before you start](#1-before-you-start) |
[2 Install](#2-install) | [3 Configure the station](#3-configure-the-station) |
[4 Start the daemon and check](#4-start-the-daemon-and-check) | [5 Dry run](#5-dry-run-without-hardware) |
[6 Record demos](#6-record-demos) | [7 Train BC](#7-train-bc) | [8 HIL training](#8-hil-training) |
[9 Deploy](#9-deploy) | [10 Known limits](#10-known-limits)

## 1. Before you start

**Hardware checklist**

- One Franka Panda per arm with the Franka Hand, and the FCI feature activated in Desk.
- A Linux robot PC wired by Ethernet to each arm's Control box (not the arm base).
- One Intel RealSense camera per view on USB 3; the wizard names a wrist camera per arm and any
  extra cameras `front`, `front_2`, ...
- One GELLO per arm with its USB bus adapter (ROBOTIS U2D2 or OpenRB-150) and powered servos. A
  U2D2 does not power the Dynamixel bus from USB ([gello.md](gello.md#1-hardware) has the
  hardware details, including the FTDI latency timer).
- A GPU machine for the learner and BC training.
- A keyboard on the actor machine (the intervention toggle and episode keys).

**Network.** The arm's FCI address is set in Desk (the default is `172.16.0.2`; the wizard also
probes `172.16.1.2` for a second arm). Give the robot PC an address in the same subnet on the
interface that is wired to the Control box (this repository does not configure it). Activate FCI
in Desk before you start; the doctor cannot do it for you.

**Linux permissions**

- GELLO serial port: the user needs the `dialout` group (`sudo usermod -aG dialout $USER`, then
  log in again) or a udev rule.
- RealSense: install the librealsense udev rules (`scripts/setup_udev_rules.sh` in the librealsense
  sources) and use USB 3 ports and cables.
- An evdev pedal or keyboard read on the robot PC (the optional `event_device` of the GELLO
  options, see [gello.md](gello.md#6-hil-training)) needs the user in the `input` group. The
  default keyboard route in this guide does not.
- Real-time kernel: optional. The default `realtime: ignore` works without a PREEMPT_RT kernel and
  the doctor then reports `control jitter possible`; `realtime: enforce` needs a PREEMPT_RT kernel
  (see the [robot_infra troubleshooting table](../robot_infra/README.md#troubleshooting)).

**Docker.** If the daemon runs in a container it needs the USB nodes passed in before it starts
(`--device=/dev/serial/by-id/...` for the GELLO, the camera devices), network access to the robot
(the FCI link is UDP) and its gRPC port reachable from the actor (`--network host`, or publish the
port). Containers are not covered by this repository's tests; the robot_infra README lists udev and
Docker as
[unverified](../robot_infra/README.md#known-limitations-and-needs-hardware-verification).

## 2. Install

Clone with the rl-garden submodule, then install by role. The extras come from
`robot_infra/pyproject.toml` and `pyproject.toml`:

```bash
git clone --recurse-submodules https://github.com/JaimeParker/rlgarden-real-world.git
cd rlgarden-real-world

# Robot PC: wizard, doctor, discovery and the daemon.
pip install -e 'robot_infra[franky,grpc,realsense,gello]'
# Add gello-events only for an evdev pedal or keyboard on the robot PC:
#   pip install -e 'robot_infra[gello-events]'

# GPU machine and actor machine: all three distributions. The robot_infra README lists the
# grpc extra for the robot PC and for the GPU machine (the client side of the daemon).
pip install -e rl-garden -e 'robot_infra[grpc]' -e .

# Actor machine: pynput for --teleop-event-device keyboard (needs that machine's desktop session).
pip install -e '.[reward-collection]'
```

| Extra | Package | Needed for |
|---|---|---|
| `robot_infra[franky]` | `franky-control` | Franka arm and Hand (for other robot firmware install the matching wheel, see the [Franka troubleshooting row](../robot_infra/README.md#troubleshooting)) |
| `robot_infra[realsense]` | `pyrealsense2` | RealSense cameras |
| `robot_infra[gello]` | `dynamixel-sdk` | the GELLO backend and its calibration |
| `robot_infra[gello-events]` | `evdev` | a GELLO `event_device` (Linux) |
| `robot_infra[grpc]` | `grpcio` | the daemon and every client of it |
| `reward-collection` | `pynput` | `--teleop-event-device keyboard` |

On one machine that does everything:
`pip install -e rl-garden -e 'robot_infra[franky,grpc,realsense,gello]' -e '.[reward-collection]'`.

Check the install without hardware: `python -m robot_infra.doctor --config <yaml> --mock`
(after step 3) swaps every backend for its fake, and
`python -m pytest tests robot_infra/tests` runs the fake-backed test suite (needs pytest, the `dev`
extra).

## 3. Configure the station

`python -m robot_infra.setup` is an interactive wizard. Run it on the robot PC with the arm(s),
cameras and GELLO(s) plugged in and powered. It probes the hardware, asks a few questions per arm,
calibrates each GELLO inline, writes one annotated `station.yaml`, runs the doctor on it and prints
the next commands. The questions are in `robot_infra/setup/wizard.py`; every one has a flag
(`python -m robot_infra.setup --help`).

```bash
python -m robot_infra.setup
```

### What it asks (one arm)

The transcript below is a `--mock` run (canned hardware, so you can try it anywhere), with the
answers you would give typed after each colon; an empty answer accepts the default in brackets. A
real run prints what it actually found: the Franka hosts that answered on TCP 22, 443 or 1337, the
RealSense serials, and each U2D2 or OpenRB-150 port with the servo ids that answered a broadcast ping.

```text
$ python -m robot_infra.setup --mock --output /tmp/station.mock.yaml
hardware probe (--mock: canned results):
franka hosts:
  172.16.0.2: open ports 22, 443, 1337
  172.16.1.2: open ports 22, 443, 1337
realsense cameras:
  123422070001 (D400)
  123422070002 (D400)
  123422070003 (D400)
gello / dynamixel buses:
  /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_MOCK0-if00-port0: 57600 baud, servo ids [1, 2, 3, 4, 5, 6, 7, 8]
  /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_MOCK1-if00-port0: 57600 baud, servo ids [1, 2, 3, 4, 5, 6, 7, 8]
spacemouse devices:
  none
number of arms (1 or 2) [1]:
arm name(s) [arm]:
-- arm 'arm' (1/1) --
franka hosts that answered:
  1) 172.16.0.2
  2) 172.16.1.2
robot IP of 'arm' (number or IP) [172.16.0.2]:
arm backend (franky | serl_http | fake) [franky]:
gripper backend (franky | serl_http | fake | none) [franky]:
home joints of 'arm' (7 values in rad, or none) [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]:
realsense cameras:
  1) 123422070001 (D400)
  2) 123422070002 (D400)
  3) 123422070003 (D400)
cameras of 'arm': numbers or serials, each optionally =name (e.g. 1=wrist,2=front), or none [123422070001=wrist,123422070002=front,123422070003=front_2]: 1=wrist
teleop device of 'arm' (gello | spacemouse | fake | none) [gello]:
gello / dynamixel buses:
  1) /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_MOCK0-if00-port0: 57600 baud, ids [1, 2, 3, 4, 5, 6, 7, 8]
  2) /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_MOCK1-if00-port0: 57600 baud, ids [1, 2, 3, 4, 5, 6, 7, 8]
GELLO serial port of 'arm' (number or path) [/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_MOCK0-if00-port0]:
GELLO baud rate [57600]:
GELLO joint servo ids (7, in the Panda's joint order) [1,2,3,4,5,6,7]:
GELLO gripper trigger servo id (or none) [8]:
intervention (always | toggle: toggle needs a pedal or keyboard) [always]:
event device for toggle/episode keys (/dev/input/by-id/... or none) [none]:
calibrate the GELLO on /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_MOCK0-if00-port0 now (yes | no) [yes]:
  ... the calibration prompts (below) ...
daemon port of 'arm' [50051]:
wrote /tmp/station.mock.yaml
checking it with the doctor (--mock):
  ... one doctor line per check, then "summary: ..." ...
next steps:
  ...
```

Answer notes:

- **Robot IP, cameras, GELLO port** take a list number or the value itself. A value the probe did not
  find is refused unless you pass `--allow-unprobed`.
- **Cameras** are `SERIAL[=NAME]` or a list number, comma separated. The `NAME` is the observation
  key later used in `obs.rgb` (`rgb_<name>`), so remember it.
- **Intervention.** Keep `always`: the keyboard in steps 6 and 8 supplies the toggle. With `always`
  and no event device the operator would be in control on every step. `toggle` is for an evdev pedal
  or keyboard on the robot PC and needs an event device
  ([gello.md](gello.md#6-hil-training)).
- **Without `--output` or `--scope`** it asks last where to write
  (`write the station to (local = ./local/station.yaml, user = ~/.config/robot_infra/station.yaml, or a path) [local]:`).
  An existing file is first copied to `station.yaml.bak.<UTC timestamp>`.

### The inline GELLO calibration

Answering `yes` to the calibration question runs the same procedure as
`python -m robot_infra.devices.backends.teleops.gello_calibrate` (torque is off, so you move the
leader by hand). You do, in order:

1. For each of the seven joints: move that joint in the robot's positive direction and press Enter.
   It prints `joint N: observed sign +1 (current +1); use it? [Y/n]`. Press Enter to accept; answer `n`
   to keep the current sign. A joint moved by less than 0.1 rad keeps its sign (in a `--mock` run
   nothing moves, so you see `moved less than 0.1 rad; keeping sign +1`).
2. Hold the leader in the calibration pose and press Enter. The prompt prints the pose,
   `[0.0, 0.0, 0.0, -1.5708, 0.0, 1.5708, 0.0]`: all joints at 0 except joint 4 at -90 degrees and
   joint 6 at +90 degrees. It then prints `offset residuals (rad): [...]`. A residual above pi/8
   prints a warning that the leader was not in the pose or a sign is wrong; run the calibration
   again.
3. Hold the trigger fully open, press Enter; hold it fully closed, press Enter. It fails if the
   trigger moved less than 0.05 rad.

If you answer `no`, the yaml gets zero offsets, all `+1` signs and a `# TODO calibrate` comment;
calibrate later with `python -m robot_infra.setup --from local/station.yaml --only teleop --calibrate`.
A failed calibration asks `retry the calibration? [Y/n]`.

### Where the file lands

The wizard writes `./local/station.yaml` (relative to the directory you run it in; `local/` is
git-ignored) by default (`--scope local`), or `~/.config/robot_infra/station.yaml` with
`--scope user`, or any path with `--output`. The daemon, the doctor and discovery find the file
without `--config`, taking the first of:

1. `$ROBOT_INFRA_CONFIG`
2. `./local/station.yaml`
3. `$XDG_CONFIG_HOME/robot_infra/station.yaml` (default `~/.config/robot_infra/station.yaml`)

`--config <yaml>` always wins. Run all later `robot_infra` commands from the same directory, or set
`ROBOT_INFRA_CONFIG`.

### Check the safety limits

The wizard writes the Panda joint limits and a servo step cap for every Franka arm
(`joint_limits`, `max_delta_per_step: {xyz_m: 0.05, joint_rad: 0.04}`) and asks for the
`safety_box` of each arm, suggesting the example workspace box. Open `station.yaml` and check the
three against your cell before the first motion: the box is in the robot base frame
(`tcp_pose = O_T_EE`, metres) and must contain the home pose; `home_joints` must lie inside
`joint_limits`. The daemon clips servo targets to these limits and refuses out-of-range `move_to_*`
targets ([Safety rules](../robot_infra/README.md#safety-rules)). The training env can only be
tighter than the yaml. Re-running the wizard with `--from` keeps the values you edited.

### Regenerating and rehearsing

```bash
python -m robot_infra.setup --from local/station.yaml                    # old values are the defaults
python -m robot_infra.setup --from local/station.yaml --only cameras     # redo one section: cameras | teleop | daemon
python -m robot_infra.setup --mock --output /tmp/station.mock.yaml       # rehearse without hardware
```

`--mock` also stands in for a two-arm station: its canned probe has two Frankas, three cameras and
two GELLO buses. Use `--output` for a rehearsal so it does not replace your real `local/station.yaml`.
For scripted use see `--non-interactive` in the
[robot_infra README](../robot_infra/README.md#setup-wizard).

### Two arms

Answer `2` to `number of arms`; the default names are `left right`. The wizard then asks every
per-arm question twice (`-- arm 'left' (1/2) --`, `-- arm 'right' (2/2) --`):

- the second arm needs a different robot IP, GELLO port and daemon port (it refuses a repeat, and
  tags earlier picks `(used by another arm)`); the daemon ports default to 50051 and 50052;
- the suggested cameras give each arm the next free camera as its wrist camera (`wrist_left`,
  `wrist_right`); the last arm also takes the rest (`front`, ...);
- every GELLO is calibrated in turn.

The result is one file with a `robots:` mapping, each robot with its own cameras, `teleop:` section and
`daemon:` block (abbreviated here; the real file is annotated and complete, see
[`station.bimanual.example.yaml`](../robot_infra/docs/station.bimanual.example.yaml)):

```yaml
robots:
  left:
    type: franka
    robot_ip: 172.16.0.2
    cameras: [{name: wrist_left, serial: "..."}]
    teleop: {name: gello, backend: gello, options: {port: /dev/serial/by-id/...LEFT..., ...}}
    daemon: {bind: 0.0.0.0:50051, token: null}
  right:
    type: franka
    robot_ip: 172.16.1.2
    cameras: [{name: wrist_right, serial: "..."}, {name: front, serial: "..."}]
    teleop: {name: gello, backend: gello, options: {port: /dev/serial/by-id/...RIGHT..., ...}}
    daemon: {bind: 0.0.0.0:50052, token: null}
```

Check the safety limits of the previous section on both robots; the two boxes should not overlap
unless the task needs it (there is no inter-arm collision check). The wizard's last lines name the
env backend for two arms, `real_robot_bimanual` ([bimanual.md](bimanual.md)).

### The actor preset

The training commands take the env settings from a YAML preset (`--config`). It names the cameras
by the station's camera names and points at the daemon address(es). Create `hil.yaml`
(`token` only if you set `daemon.token` in the station file):

```yaml
# hil.yaml (one arm)
env_backend: real_robot
env_id: my_task
obs:
  rgb: [wrist]                 # the camera names from station.yaml
  image_size: [128, 128]       # required with cameras: a remote robot reports its size only after connecting
real_robot:
  remote: ROBOT_PC:50051
  num_joints: 7
  action_mode: joint_delta
  joint_scale_rad: 0.05
  state_fields: [joint_pos, gripper_pos]
  control_hz: 10
  max_episode_steps: 200
```

Two arms: `bimanual.yaml`. `obs.rgb` lists every camera in arm order (left's, then right's), each
arm lists its own cameras, the daemon addresses come from the ports in the station file, and the
two arms must agree on `action_mode`, `state_fields` and the other shared fields
([bimanual.md](bimanual.md#config)):

```yaml
# bimanual.yaml (two arms)
env_backend: real_robot_bimanual
env_id: my_bimanual_task
obs:
  rgb: [wrist_left, wrist_right, front]
  image_size: [128, 128]
real_robot_bimanual:
  max_episode_steps: 200
  left:
    remote: ROBOT_PC:50051
    cameras: [wrist_left]
    action_mode: joint_delta
    state_fields: [joint_pos, gripper_pos]
  right:
    remote: ROBOT_PC:50052
    cameras: [wrist_right, front]
    action_mode: joint_delta
    state_fields: [joint_pos, gripper_pos]
```

Check a preset without touching the robot:

```bash
python examples/train_real_world.py hil_serl --config hil.yaml --print-config
```

**Why `joint_delta` throughout.** The recorded demos, the BC checkpoint, and every HIL run must use the
same `action_mode`: the learner rejects a demo pkl whose contract sidecar differs from its env, and a
residual base must act in the env's action space. This guide uses `joint_delta` for all stages (the
setting the two-arm smoke test exercises). [gello.md](gello.md) and
[recording-demos.md](recording-demos.md) call `joint_abs` the BC setting; if you prefer it, record a
separate `joint_abs` set, use it only for BC and its deploy, and keep `joint_delta` demos for the HIL
stages. `joint_abs` additionally needs `joint_limits` in the preset and
`max_delta_per_step.joint_rad` in the station yaml.

## 4. Start the daemon and check

Check the station first, with nothing serving the robots:

```bash
python -m robot_infra.discovery     # what hardware is visible (never connects to a robot)
python -m robot_infra.doctor        # the five-layer check of every robot of the station
```

*You should see* for discovery: the camera serials, the GELLO ports, and per robot
`172.16.0.2: ping=ok open ports=<list>` (for Franka, port 1337 open usually means FCI is reachable).
With nothing attached the sections read `none` ([sample output](../robot_infra/README.md#2-plug-in-and-discover)).
For the doctor: one line per check, `[1/5] sdk:arm ... OK  ...` through `[5/5] mode:...`, then
`summary: N ok, 0 failed, S skipped`; the exit code is 1 if anything failed. Two arms print
`== robot 'left' ==` and `== robot 'right' ==` blocks and a final `overall: 2 robots, all passed`.
The doctor connects to the robot and disconnects it; run it again with `--robot left` to check one,
`--fix` to run a failed check's autofix, `--json` for machine output.

Then serve, with no `--config` (it uses the station file):

```bash
python -m robot_infra.server --watchdog-s 5
```

`--watchdog-s 5` matters for training: the daemon idles an arm that gets no action for the watchdog
time (default 0.5 s), and the first policy step after a reset can take longer. *You should see* the
same doctor lines first (the daemon's preflight), then the robot's parts and
`serving franka 'arm' on 0.0.0.0:50051`. The robot connects when the first client does. Leave the
terminal open; Ctrl-C stops the daemon without powering the robot off.

Two arms: the same one command starts both daemons. It prints
`starting 2 daemons from <path>: 'left' on 0.0.0.0:50051, 'right' on 0.0.0.0:50052`, then each daemon
prints its own preflight and `serving ...` line. The ports come from the file's `daemon.bind`
entries. `--robot left` serves just one; `--bind` and `--token` are errors for several robots (set
them per robot in the file). Do not run `python -m robot_infra.doctor` while a daemon serves the same
robot: the doctor takes the same device claim and reports a failed `handshake`.

### If a check fails: the three common failures

Each failing line is followed by `hint: ...`; follow it. The most common:

1. **GELLO serial permission.**
   `[2/5] os:serial ... FAIL  no read/write access to /dev/serial/by-id/...` with the hint to add the
   user to the `dialout` group (`sudo usermod -aG dialout $USER`, then log in again) or a udev rule.
   `serial port ... not found` means a cable or the wrong port: list `/dev/serial/by-id/`.
2. **Franka not reachable.** `os:ping` FAIL and `handshake ... FAIL  cannot reach Franka at <ip>: ...`
   with the hint `check: ping <ip>; FCI activated in Desk; cable on the Control (not Arm) port;
   'UDP receive: Timeout' means a firewall`. Activate FCI in Desk, plug into the Control box,
   allow UDP from the robot through the firewall, and check the robot IP in the yaml.
3. **Camera serial ambiguous or missing.** `os:camera:<name> ... FAIL  serial unset and N devices
   attached; set serial to one of [...]`, or `camera '<name>' serial '<s>' not found`. The wizard
   writes serials, so this appears after a hand edit or a re-plugged camera: copy the serial from
   `python -m robot_infra.discovery` into `cameras[].serial`. `no frames within 2s` means USB 3
   bandwidth or udev rules; try `--fix`.

The full table, including libfranka version mismatch, Reflex mode and the real-time kernel, is the
[robot_infra troubleshooting table](../robot_infra/README.md#troubleshooting).

## 5. Dry run without hardware

**One arm.** The README's
[End-to-end try-out with the fake robot](../README.md#end-to-end-try-out-with-the-fake-robot) runs a
daemon on fake backends, a learner and an actor, and a deploy of the checkpoint on one machine.
Run it once so the commands in steps 6 to 9 are familiar. To exercise the actor without any operator,
use `--teleop-device fake` (a scripted source that never intervenes, [teleop.md](teleop.md#devices))
instead of `--teleop-device robot`; the README's scripted `teleop: backend: fake` robot part is the
variant whose operator does intervene.

**Two arms.** `tests/test_bimanual_station_smoke.py` is the two-arm workflow on fakes. It writes a
station with `robots: {left, right}` (fake arm, gripper, cameras and a scripted `joint_abs` leader
part per arm), serves each robot with its own gRPC daemon on loopback, builds the
`real_robot_bimanual` env against them, and runs in order the `hil_serl`, recorder, `residual_hil_serl`
and `silri` actors, the learners loading the recorded demos, `real_world_bc` training and the BC
deploy. Run it with:

```bash
python -m pytest tests/test_bimanual_station_smoke.py -q
```

It is skipped when `grpcio` is not installed. [bimanual.md](bimanual.md#verified-hardware-free)
lists the equivalent commands.

## 6. Record demos

`examples/record_real_demos.py` builds the env exactly as the HIL actor does and records only the
steps where you intervene. Start the daemon (step 4), then:

```bash
ENV_FLAGS="--config hil.yaml --teleop-device leader_arm --robot-teleop-action-mode joint_delta \
    --teleop-event-device keyboard"
python examples/record_real_demos.py $ENV_FLAGS --teleop-onset arm \
    --output-path demos/task.pkl --num-successes 20
```

- `--teleop-device leader_arm` reads the GELLO that the daemon serves as the robot's `teleop:` part.
  `--robot-teleop-action-mode` declares that part before the remote robot connects; for a leader arm
  it only has to name a joint space.
- `--teleop-event-device keyboard` gives the intervention toggle and the episode keys.
- `--teleop-onset arm` makes the toggle take effect only once the leader is within
  `--teleop-arm-tolerance-rad` (0.1) of the follower, so every takeover starts from a still,
  aligned robot. The default `ramp` takes over at once. Before the first Space, hold the leader near
  the follower's pose: in `joint_delta` the follower moves towards the leader at up to
  `joint_scale_rad` (0.05 rad) per step.

Two arms: replace the device flags with
`--config bimanual.yaml --teleop-device bimanual --teleop-arm-device leader_arm
--robot-teleop-action-mode joint_delta --teleop-event-device keyboard`; each arm reads its own GELLO
and a step is recorded when either operator intervenes (the other arm is held still). See
[bimanual.md](bimanual.md#teleop-and-demo-recording).

*How an episode runs.* The env resets (the arm homes, the gripper opens). Press **Space** to start
and stop intervening, move the GELLO to do the task, press **s** when it succeeded and **Esc** to end
the episode (the step cap also ends it). **f** discards the episode, **r** discards it and starts
the next at once. Only successful episodes with at least one recorded step are kept. Ctrl-C stops the
session and still saves what was kept.

*You should see* one line per finished episode and a final save line (numbers here are examples):

```text
[record] episode=0 steps=87 transitions=87 kept successes=1/20
[record] episode=1 steps=40 transitions=0 discarded (<reason>) successes=1/20
...
[record] saved 20 episodes (1734 transitions) to demos/task.pkl
```

and two files, `demos/task.pkl` and its contract sidecar `demos/task.contract.json` (action mode,
widths, joint limits, scales, state fields). The learners and BC load the pkl together with the
sidecar and raise a `ValueError` naming the differing keys if the env does not match. `--output-path`
must be a new `.pkl` file. More in [recording-demos.md](recording-demos.md).

*If not:* a step cap that is too short ends episodes before the task is done (raise
`max_episode_steps` in the preset); `--keep-syncing-steps` and `--start-tolerance-rad` are explained
in [recording-demos.md](recording-demos.md#recorder-flags). A GELLO that is not read at all shows up as
the daemon's `feedback:teleop` doctor check, or as a `DeviceFault` from the actor.

## 7. Train BC

`real_world_bc` is offline behaviour cloning over the demo pkls. It has no roles and does not connect
to the robot; the `--config` preset only gives it the env's observation and action spaces. Run it on
the GPU machine:

```bash
python examples/train_real_world.py real_world_bc --config hil.yaml \
    --demo-dataset-paths 'demos/*.pkl' \
    --log-dir runs --exp-name bc --buffer-size 100000 \
    --num-offline-steps 20000 --no-tanh-squash
```

- `--no-tanh-squash`: recorded gripper actions sit exactly at +1 or -1, where the default tanh-squashed
  actor's log-probability is numerically degenerate (see the comment on `tanh_squash` in
  rl-garden's `OfflineBCArgs`).
- `--buffer-size` is the replay capacity the demos are loaded into (default 1000000, on
  `--buffer-device cuda`); keep it at least as large as the number of recorded transitions and size
  it to your GPU memory.
- `--num-offline-steps` (default 100000) is the number of gradient steps; 20000 is an example.
- `--log-freq` (default 1000) is how often the BC losses are logged to `--log-type`.

*You should see* a metrics line every `--log-freq` steps; the run is done when the process exits and
`runs/bc/checkpoints/final.pt` and `runs/bc/config.json` exist (`<log-dir>/<exp-name>/...`). The
`config.json` next to `checkpoints/` is what deploy and the residual base provider read to rebuild the
policy. Two arms: the same command with `--config bimanual.yaml`.

*If not:* a `ValueError: ... recorded action contract does not match the env's (differs in [...])`
means the demos were recorded with different settings than the preset (action mode, joint limits,
scales, state fields, arm count): record again, or fix the preset. `No demo transitions loaded`
means the glob matched nothing.

## 8. HIL training

Three methods share the same actor, learner and eval roles, teleop flags, demo pkls and sync
protocol. Run one at a time, each with its own `--exp-name`:

| Method | What it adds | Guide |
|---|---|---|
| `hil_serl` | RLPD with human interventions, a growing demo buffer and a DQN gripper head | [README](../README.md#running-on-hardware) |
| `residual_hil_serl` | a bounded residual on top of a frozen base policy (your BC checkpoint) | [residual.md](residual.md) |
| `silri` | an actor pulled towards a learned model of your interventions | [silri.md](silri.md) |

Processes and where they run: the daemon on the robot PC (step 4, `--watchdog-s 5`), the learner on
the GPU machine, the actor on the actor machine. The learner binds `--sync-host` and the actor
connects to it, so both get the learner's address.

```bash
ENV_FLAGS="--config hil.yaml --teleop-device leader_arm --robot-teleop-action-mode joint_delta \
    --teleop-event-device keyboard"
COMMON="$ENV_FLAGS --sync-host LEARNER_IP --sync-port 6000 --log-dir runs \
    --checkpoint-freq 1000 --buffer-size 100000"
```

Two arms: only `ENV_FLAGS` changes (and note `num_grippers=2` is derived automatically from the env's
action contract for all three agents; you do not pass it):

```bash
ENV_FLAGS="--config bimanual.yaml --teleop-device bimanual --teleop-arm-device leader_arm \
    --robot-teleop-action-mode joint_delta --teleop-event-device keyboard"
```

Notes on `COMMON`: `--checkpoint-freq` defaults to 0, in which case only a final checkpoint is written
when the learner stops; `--buffer-size` is the replay capacity (default 200000 for `hil_serl`, held on
`--buffer-device cuda`), so with several 128 x 128 cameras size it to your GPU memory or use
`--buffer-device cpu`. The learner ignores the teleop flags (it never opens a device).

### 8a. hil_serl

```bash
python examples/train_real_world.py hil_serl --role learner $COMMON --exp-name hil \
    --demo-dataset-paths 'demos/*.pkl'
python examples/train_real_world.py hil_serl --role actor $COMMON --exp-name hil
```

Start the learner first. The learner loads the demos into its demo buffer, waits for
`--learning-starts` transitions (default 4000), then trains and publishes weights every
`--publish-freq` updates (default 100); until the first weights arrive the actor acts with an
untrained policy. Evaluate a checkpoint with deterministic episodes on the robot:

```bash
python examples/train_real_world.py hil_serl --role eval $COMMON --exp-name hil \
    --load-checkpoint runs/hil/checkpoints/checkpoint_<update>.pt --eval-n-trajs 10
```

### 8b. residual_hil_serl on top of the BC policy

```bash
BASE='checkpoint:runs/bc/checkpoints/final.pt?algo=real_world_bc'   # quoted: '?' is a glob character
RESIDUAL="--residual-variant hybrid --residual-action-scale 0.1 --residual-bc-weight 0.1 \
    --critic-only-updates 500"
python examples/train_real_world.py residual_hil_serl --role learner $COMMON $RESIDUAL \
    --exp-name residual --base-provider "$BASE"
python examples/train_real_world.py residual_hil_serl --role actor $COMMON $RESIDUAL \
    --exp-name residual --base-provider "$BASE"
```

The actor executes `clamp(base + scale * residual, -1, 1)` where the base is the BC policy; your
interventions override the total action. The BC run must use the same `hil.yaml` (same observation and
action spaces). Teleop recordings carry no base actions, so they are not passed with
`--demo-dataset-paths` here (the learner rejects them unless you add `--allow-base-less-demos`, see
[residual.md](residual.md#human-intervention)).

Two arms: the BC base needs nothing extra (`checkpoint:...?algo=real_world_bc`). A two-arm
`hil_serl` (`rlpd_hybrid`) checkpoint as the base names its gripper count:
`checkpoint:<path>?algo=rlpd_hybrid&num_grippers=2`
([provider table](residual.md#base-action-providers)).

### 8c. silri

```bash
python examples/train_real_world.py silri --role learner $COMMON --exp-name silri \
    --demo-dataset-paths 'demos/*.pkl'
python examples/train_real_world.py silri --role actor $COMMON --exp-name silri
```

Before its first update the SiLRI learner pretrains its human-behaviour model on the demo buffer
(`--bc-pretrain-steps`, default 500) and blocks ingestion while it does; see [silri.md](silri.md).

### What to watch, and when to intervene

*Reward.* The `real_robot` env returns reward 0 and never terminates on its own; only the step cap, an
Esc press or a reward classifier ends an episode with a result. A task reward comes from
`--classifier-checkpoint <path>` (with `--classifier-threshold` and `--classifier-image-keys`),
which wraps the env in a reward classifier, and `--role eval` reads success from it. This guide does
not train a classifier: the repository's collection tool (`rl_garden_real_world/reward/collect_data.py`)
is wired for the `franka_real` backend only. Without a classifier these runs learn from zero reward
and your interventions.

*Metrics.* The real-world learner loops call `agent.train(..., compute_info=False)` and log no
scalars, so no `train/...` curves are produced today (the values the algorithms compute, such as
`actor/intervened_fraction` for residual and `actor/distance`, `lambda/mean` for SiLRI, are never
requested). What you can watch instead:

- `checkpoint_<update>.pt` files appearing in `runs/<exp-name>/checkpoints/` every `--checkpoint-freq`
  updates (the number is the learner's update count);
- the crash-recovery snapshots next to them: `buffer/transitions_<n>.pkl` (every transition) and
  `demo_buffer/transitions_<n>.pkl` (your intervened steps only), one pair per `--buffer-period`
  received transitions (default 1000). A `demo_buffer` that stops growing means you are no longer
  intervening;
- `[eval] episode=... success=... return=...` lines and the final `[eval] success_rate=...` line of a
  `--role eval` run, which is the only quantitative signal in this loop.

*When to intervene.* Press Space and take over when the policy is about to fail the task, collide, or
leave the safe region; release when it is back on track. Do not intervene while it is doing fine: the
actor stores every step, and only your intervened steps enter the demo buffer. Expect to intervene a
lot at first and less as the policy improves. Esc ends an episode (it counts as truncated) and the
robot homes. In `residual_hil_serl` the intervention is your total action, not a correction of the
base.

*If not:* `DeviceFault: the daemon watchdog stopped the arm ...` on the first step after a reset
means the daemon's `--watchdog-s` is too small. `Unknown env backend 'real_robot'` means the
`rlgarden-real-world` package is not installed. `TransportError: robot daemon unreachable at <addr>`
means the daemon is not running or the preset's `remote` is wrong. The actor stopping with a
`DeviceFault` from the teleop means the GELLO read failed (it is never silently dropped).

## 9. Deploy

Deployment runs a checkpoint on the robot with no learner, teleop or data collection. The training
run's `config.json` (next to `checkpoints/`) decides the action and observation contract, so the
command line gives only the connection.

Deploy against a daemon started from a copy of the station yaml **without the `teleop:` sections**
(a configured teleop must be present for a robot to connect, and a deployment has no operator): copy
`local/station.yaml` to `local/station.deploy.yaml` and delete each robot's `teleop:` block. Stop the
training daemon first (a second daemon on the same robot gets `DeviceBusy`), and start the deploy
daemon with the same `--jpeg-quality` as training (this guide used none):

```bash
python -m robot_infra.server --config local/station.deploy.yaml --watchdog-s 5
```

Then:

```bash
python examples/deploy_real_robot.py --checkpoint runs/bc/checkpoints/final.pt \
    --algo real_world_bc --real_robot.remote ROBOT_PC:50051 \
    --episodes 5 --max-steps-per-episode 200
```

| Run | `--algo` | Checkpoint |
|---|---|---|
| BC | `real_world_bc` | `runs/bc/checkpoints/final.pt` |
| `hil_serl` | `rlpd_hybrid` | `runs/hil/checkpoints/checkpoint_<update>.pt` |
| `residual_hil_serl` | `residual_hil_serl` (continuous variant) or `residual_hil_serl_hybrid` | `runs/residual/checkpoints/checkpoint_<update>.pt`; the base provider is rebuilt from the checkpoint metadata (`--base-provider` replaces it) |
| `silri` | `silri` | `runs/silri/checkpoints/checkpoint_<update>.pt` |

Add `--real_robot.token T` if the daemon requires a token, and `--device cuda` for a GPU policy
(default `cpu`).

Two arms: add `--env-backend real_robot_bimanual` and give each arm's connection and cameras (the
cameras are not filled from the training run for two arms):

```bash
python examples/deploy_real_robot.py --checkpoint runs/bc/checkpoints/final.pt \
    --algo real_world_bc --env-backend real_robot_bimanual \
    --real_robot_bimanual.left.remote ROBOT_PC:50051 \
    --real_robot_bimanual.right.remote ROBOT_PC:50052 \
    --real_robot_bimanual.left.cameras wrist_left \
    --real_robot_bimanual.right.cameras wrist_right front
```

*You should see* one line per episode and a summary
(`[deploy] episode=0 steps=... return=... truncated=... rtt_ms=... health_ok=True`, then
`[deploy] summary episodes=... mean_steps=... mean_return=... rtt_ms=... all_healthy=True`). A device
error (daemon unreachable, watchdog, safety) prints `[deploy] device error: ...` and a
`[deploy] hint: ...` line to stderr and exits with code 2. A contract field on the command line that
differs from the training run (`--real_robot.action-mode joint_abs` against a `joint_delta` run, for
example) is a `ValueError`; drop it, the run decides. A run trained on one backend cannot be deployed
with the other ([bimanual.md](bimanual.md#deploy)).

## 10. Known limits

- **Everything hardware-side was verified against fakes only.** In the robot_infra README's words:
  "Nothing below has been exercised on real hardware; the fakes cover the logic only"
  ([Known limitations](../robot_infra/README.md#known-limitations-and-needs-hardware-verification)),
  and [gello.md](gello.md#8-not-verified-yet): "no run on a physical GELLO or robot has been made
  yet". The GELLO hardware numbers (baud rate, read time, latency), the per-unit offsets and signs,
  and torque-on behaviour are unverified. Expect to debug on the first day, and commission with low
  `max_delta_per_step` values.
- **No inter-arm collision check.** Each arm is clipped only by its own safety box, joint limits and
  step cap. Keep the two arms' `safety_box` regions disjoint ([bimanual.md](bimanual.md#limits)).
- **`leader_cartesian`**: the Panda forward-kinematics DH table used to turn a GELLO into a cartesian
  twist has not been compared with a real robot ([teleop.md](teleop.md#verify-the-dh-table-on-the-robot)).
  This guide does not use it.
- **`franka_real`** (the SERL HTTP bridge) is single-arm and cartesian only; two arms exist only on
  `real_robot_bimanual`.
- **Reward and metrics** (step 8): no reward without a classifier, and no learner scalars are logged.
- **Configured teleop is mandatory**: a robot yaml with a `teleop:` section fails to connect when the
  device is absent, so deployment uses a yaml without it.
