# robot_infra: the device layer

`robot_infra` is the real-robot layer of [rl-garden](https://github.com/JaimeParker/rl-garden).
It is a package of the [real-world-rl](https://github.com/JaimeParker/real-world-rl)
distribution, at the repository root next to `real_world_rl/` (it was the standalone
`rlgarden-robot-infra` repository, whose history is kept). The device layer, its commands
and the daemon do not import torch; only the legacy `controller/` needs torch. A robot PC that
only serves the daemon installs the core with its hardware extras, without the `rl-garden/`
submodule or torch: from the repository root, `bash scripts/install.sh --daemon-only`, or
`pip install -e '.[franky,grpc]'` in an environment of your own (see [Install](#install)).
The GPU machine installs rl-garden (the `rl-garden/` submodule) and this project with
`pip install -e ./rl-garden -e '.[learning]'`.
It provides:

- a **device layer**: typed `Arm` / `Gripper` / `Camera` / `Teleop` contracts with backends for
  Franka (franky, legacy ROS1 bridge), AgileX Piper (pyAgxArm, piper_sdk), Intel RealSense and
  SpaceMouse, plus device-side **safety limits** (TCP box, joint limits, per-step caps);
- **discovery** of attached cameras, teleop devices, CAN interfaces, robot hosts, and HID devices;
- a **doctor** that checks a robot config layer by layer and prints a fix hint for each failure;
- a **robot daemon** that serves one robot over gRPC from the robot PC;
- `RemoteRobot`, a client-side mirror of that robot for use on a GPU machine.

The gym environment for real-world RL lives in
[real-world-rl](https://github.com/JaimeParker/real-world-rl), which imports
this package. `robot_infra` itself does not depend on `rl_garden`.

## Device commands in rwrl

The device layer's native commands, `setup`, `daemon`, `supervise`, `discover` and
`gello-calibrate`, are `rwrl` subcommands; there is no separate binary and no
`python -m robot_infra.*` entry point. `rwrl doctor` runs the device checks of this layer
for every station robot, together with its host and project checks
([cli.md](../cli.md#robot-host-diagnosis-and-repair)); the device doctor itself is a
library (`robot_infra.doctor`). The command protocol and parser do not import torch.
Station input is `--station PATH`. Daemon and discover retain
`--config PATH`, and setup retains `--from PATH`, as temporary aliases marked
"(alias, will be removed)" in help. Setup uses that file as defaults, not as
its output path; station lookup defaults are unchanged. Errors use stderr,
and exit-code constants live in `robot_infra.cli.errors`.

On the robot PC, from the repository root with the project environment active:

```bash
rwrl --help
rwrl daemon --help
rwrl doctor --station local/station.yaml --json
```

`rwrl doctor` accepts `--yes` to skip confirmation while still running preflight,
even without a repair flag. Repair confirmation accepts only `y` (ignoring
case and surrounding whitespace). The shared exit policy is 0 for success,
1 for runtime/device failure or unresolved doctor findings, and 2 for usage
or configuration errors. Setup interruption returns 130; supervisor launch
failure returns 127 and stop escalation returns 1, otherwise child status is
preserved.

Commands without JSON output (`setup`, `daemon`, `supervise`,
`gello-calibrate`) reject the global `--json` option with usage exit code 2.
`doctor` and `discover` support `--json`; `rwrl --json discover` prints
`schema_version: 1` and the lists `cameras`, `teleops`, `can_interfaces`,
`hosts`, `hid_devices` and `unknown_hid` (see [Plug in and discover](#2-plug-in-and-discover)).
`supervise` retains its `-- <child command...>` boundary; child flags are passed
to the child unchanged. `gello-calibrate` prints a teleop YAML block;
`setup --only teleop --calibrate` runs the same calibration in the wizard and
writes station YAML.

## Install

Requires Python >= 3.10. From the repository root, project environment active:

```bash
pip install -e .                    # core: devices, discovery, doctor, rwrl, fake backends
pip install -e '.[franky,grpc]'     # e.g. a Franka robot PC running the daemon
```

`bash scripts/install.sh --daemon-only` does the second line in a new `.venv`. The learning
side (training, recording, deployment, `rwrl teleop run`) is the `learning` extra with the
`rl-garden/` submodule: `pip install -e ./rl-garden -e '.[learning]'`.

| Extra | Installs | You need it when | Machine |
|---|---|---|---|
| `franky` | `franky-control` | Franka arm/hand via libfranka (`arm_backend: franky`). The default wheel bundles one libfranka version; for other robot firmware, install the matching wheel (see [Installing the right franky wheel](#installing-the-right-franky-wheel)). | robot PC |
| `pyagxarm` | `pyAgxArm` (from GitHub) | Piper via `arm_backend: pyagxarm` (the default for Piper) | robot PC |
| `piper-sdk` | `piper_sdk` | Piper via the vendor SDK (`arm_backend: piper_sdk`) | robot PC |
| `realsense` | `pyrealsense2` | RealSense cameras | robot PC |
| `spacemouse` | `hidapi==0.14.0.post4` (trezor/cython-hidapi, import `hid`) | `teleop: {backend: spacemouse}` (in-tree reader and discovery) | robot PC |
| `gello` | `dynamixel-sdk>=3.7.31,<3.9` | `teleop: {backend: gello}` (GELLO leader arm) and its calibration CLI | robot PC |
| `gello-events` | `evdev` | a gello `event_device` (pedal or keyboard toggle and events, Linux) | robot PC |
| `grpc` | `grpcio>=1.60` | the daemon (`rwrl daemon`) and `RemoteRobot` | robot PC **and** GPU machine |
| `jpeg` | `pillow`, `opencv-python-headless` (`cv2`) | `rwrl daemon --jpeg-quality N`; the JPEG path in `server/codec.py` imports `cv2` | robot PC **and** GPU machine |
| `legacy-bridge` | `flask`, `pyzmq`, `scipy`, `pyspacemouse` | legacy Flask bridge/gripper server and the legacy `teleop/spacemouse/` (old `read_all()` API, separate from the in-tree reader; ROS1 must be installed separately) | robot PC |
| `sim` | `mani_skill` | the ManiSkill impedance controllers in `controller/simulator/` and the ManiSkill examples | GPU machine |
| `dev` | `pytest`, `scipy` | running the tests | any |

Vendor SDKs are imported lazily. The core package, the CLIs and `--mock` runs work
without any SDK installed.

### Setup wizard

For a Franka station (one arm, or two arms for bimanual work), plug in the robot (Ethernet),
the RealSense cameras and the GELLO leader arm(s) (USB), then run on the robot PC:

```bash
pip install -e '.[franky,grpc,realsense,gello]'
rwrl setup
```

It probes the hardware (TCP 22/443/1337 of `172.16.0.2`, `172.16.1.2`, `$ROBOT_IP` and
`--robot-ip`, plus every host of `--scan-subnet CIDR`; RealSense serials; a Dynamixel broadcast
ping on every `/dev/serial/by-id/usb-FTDI_*` and `usb-ROBOTIS_*` port at 57600 to 4000000 baud;
SpaceMouse devices), then asks per arm for the robot IP, arm and gripper backend, home joints,
cameras (`1=wrist,2=front`), teleop device (GELLO port and servo ids, the last id offered as the
gripper trigger), intervention mode and daemon port, calibrates each GELLO inline (the same
steps as the calibration CLI in [step 7](#7-add-a-teleop-device); skipped, it writes zero offsets
with a `# TODO calibrate` comment), writes the annotated station yaml, runs the doctor on it and
prints the next commands.

Every arm gets device-side safety limits: the Panda datasheet `joint_limits` (`--joint-limits`),
a servo step cap `max_delta_per_step: {xyz_m: 0.05, joint_rad: 0.04}` (`--max-delta-xyz-m`,
`--max-delta-joint-rad`) and a `safety_box` it asks for (base frame, metres, suggested
`[0.3, -0.3, 0.05, 0.7, 0.3, 0.6]`, `--safety-box`), which must contain the home pose.

The daemon, the doctor and discovery find the station yaml without `--station`. They take the
first of:

1. `$ROBOT_INFRA_CONFIG`
2. `./local/station.yaml` (relative to the current directory; the wizard's default, `--scope local`; `local/` is git-ignored)
3. `$XDG_CONFIG_HOME/robot_infra/station.yaml`, default `~/.config/robot_infra/station.yaml` (`--scope user`)

`--station <yaml>` still wins over all three.

```bash
rwrl setup --mock                     # canned hardware and a simulated GELLO; the doctor runs with --mock
rwrl setup --station local/station.yaml  # the old file's values are the defaults
rwrl setup --station local/station.yaml --only cameras   # redo one section: cameras | teleop | daemon
rwrl setup --non-interactive --mock --arms 1 --arm-names arm --robot-ip 172.16.0.2 \
    --arm-backend franky --gripper franky --home-joints 0,-0.785,0,-2.356,0,1.571,0.785 \
    --cameras 123422070001=wrist --teleop none --daemon-port 50051 --output /tmp/station.yaml
```

Every question has a flag (`rwrl setup --help`); per-arm flags take one value
per arm or one for all arms. `--non-interactive` fails on the first question that has neither a
flag nor a `--station` value; `--non-interactive --station X` rewrites `X` unchanged. An existing file
is first copied to `station.yaml.bak.<UTC %Y%m%dT%H%M%SZ>`, then replaced atomically. A probed
value is required unless `--allow-unprobed`.


### SpaceMouse Python dependencies and udev

In the existing project environment, the `spacemouse` extra installs
`hidapi==0.14.0.post4` (trezor/cython-hidapi, import `hid`). On the NUC/device host,
repository root, project environment active:

```bash
pip install -e '.[spacemouse]'
```

The in-tree reader ports all 10 models from the vendored PySpaceMouse 1.0.3 table;
Enterprise is not in that table. Other models a site meets go into its local device table,
`spacemouse-devices.yaml` next to the station file (or the path in the teleop option
`device_table`), which adds entries or overrides the built-in entry of the same id; see
[Site device table](../spacemouse-commissioning.md#site-device-table). `discover()`,
doctor and `open()` match the exact vendor and product id pair of that merged table, so
other devices of the same vendor (for example a Logitech `046d` keyboard receiver) are not
taken for a SpaceMouse. It uses `hid.device()` and does not import
`pyspacemouse`/`easyhid`. Do not install the different distribution named `hid`.
The legacy `legacy-bridge` extra and `SpaceMouseExpert` still use their existing API.
For a fresh project environment use the installer on the NUC, repository root,
no environment active:

```bash
bash scripts/install.sh --extras learning,franka-gello,spacemouse
source .venv/bin/activate
```

The target is Ubuntu 22.04 / CPython 3.10 / x86_64. The wheel route avoids
system `libhidapi` lookup through easyhid; the wheel's ELF/system-library
requirements still need a clean-machine check. Installation and device operation
are **not yet verified on hardware**.

The rules file ships as `robot_infra.setup/99-spacemouse.rules`, permitting vendor
`256f` and `046d` hidraw devices with `MODE="0664", GROUP="plugdev"`. Install it
once on the NUC, repository root, any shell (no project environment required),
with sudo permission:

```bash
sudo cp robot_infra/setup/99-spacemouse.rules /etc/udev/rules.d/
sudo usermod -aG plugdev "$USER"
sudo udevadm control --reload-rules && sudo udevadm trigger --subsystem-match=hidraw
```

Log out and log back in, then unplug and replug the device. Alternatively, on the
NUC from any directory, **project environment active**, with sudo permission,
copy the packaged file below, then run the same group/reload commands:

```bash
sudo cp "$(python -c 'import robot_infra.setup, os; print(os.path.dirname(robot_infra.setup.__file__))')/99-spacemouse.rules" /etc/udev/rules.d/
```

See [SpaceMouse installation](../teleop.md#spacemouse-installation-and-linux-permissions)
for the full installation boundary. The device layer's button levels and latched
press events remain distinct from the application's take-over/gripper semantics
([step 7](#7-add-a-teleop-device)).

## How to Start

These steps run on the robot PC (the computer wired to the arm, CAN adapter and cameras),
except step 5, which runs on the GPU machine. For the training side (demos, BC, HIL-SERL, deploy)
on top of a running daemon, see the real-world tutorial
[`docs/tutorial/`](../tutorial/00-overview.md).

### 1. Write `robot.yaml`

A robot config is one YAML mapping with a `type` key. It can also be nested under a
top-level `robot:` key. Unknown keys are rejected with the list of valid ones. Omitted
keys take the defaults shown below. Full annotated copies are in
[`examples/stations/robot.franka.example.yaml`](../../examples/stations/robot.franka.example.yaml) and
[`examples/stations/robot.piper.example.yaml`](../../examples/stations/robot.piper.example.yaml). The
[setup wizard](#setup-wizard) writes this file for a Franka station.

A station with several robots (for example two Frankas for bimanual work) lists one robot
mapping per name under `robots:`; a mapping without `name` takes its key. Each robot may carry a
`daemon:` block (also allowed in a single-robot file), which the daemon reads and strips before
the robot config is parsed:

```yaml
robots:
  left:
    type: franka
    robot_ip: 172.16.0.2
    # ... any robot key, as above
    daemon: {bind: 0.0.0.0:50051, token: null}   # default bind 0.0.0.0:50051, no token
  right:
    type: franka
    robot_ip: 172.16.1.2
    daemon: {bind: 0.0.0.0:50052, token: null}
```

`rwrl daemon` then starts one daemon process per robot, and
`rwrl doctor` checks each robot in turn; `--robot <name>` picks one. A full
example with two GELLOs and three cameras is
[`examples/stations/station.bimanual.example.yaml`](../../examples/stations/station.bimanual.example.yaml).
`Robot.from_yaml` reads single-robot files without a `daemon:` block only.

Upper-layer packages can register additional robot-mapping sections through
`parse_station`, `load_station`, or `load_station_with_env` using
`sections={name: validator}`. Like `daemon:`, these sections describe the robot
without being device configuration: the loader removes each registered key before
`RobotConfig.from_dict`, validates its value, and stores the returned value in
`StationRobot.sections`. The device layer does not interpret section contents.
Without sections (`sections=None` or `{}`) these loaders, and `load_config` and
`load_config_with_env`, parse strictly; there is no entry-point discovery. real_world_rl's
sections (`teleop_profiles`) are registered by `real_world_rl.station.load_station` and
`load_station_with_env`, which every `rwrl` command uses, the daemon, setup, discover and
the device doctor included. An unknown key names that loader and the registered section names.
Unregistered unknown keys still fail; single-robot and `robots:` mappings both
support sections. The setup wizard renders device configuration, not upper-layer
sections.

**Franka Panda + Franka Hand + one RealSense**

```yaml
type: franka
name: franka                  # default "franka"
arm_backend: franky           # franky (default) | serl_http (legacy ROS1 bridge) | fake
robot_ip: 172.16.0.2          # FCI IP set in Desk (default 172.16.0.2)
# bridge_url: http://127.0.0.1:5000   # serl_http only: URL of the legacy Flask bridge
realtime: ignore              # ignore (default) | enforce (requires a PREEMPT_RT kernel)
relative_dynamics_factor: 0.2 # franky velocity/accel/jerk scale for move_to_* (default 0.2)
home_joints: [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]  # 7 values (default shown)
safety_box: [0.3, -0.3, 0.05, 0.7, 0.3, 0.6]  # [xmin, ymin, zmin, xmax, ymax, zmax] m, base frame of tcp_pose (O_T_EE)
joint_limits:                 # 7 [lo, hi] pairs in rad; home_joints must lie inside
  - [-2.8, 2.8]
  - [-1.7, 1.7]
  - [-2.8, 2.8]
  - [-3.0, -0.1]
  - [-2.8, 2.8]
  - [0.0, 3.7]
  - [-2.8, 2.8]
max_delta_per_step: {xyz_m: 0.05, joint_rad: 0.1}  # servo step cap from the measured state
claim: true                   # daemon takes a per-host exclusive lock while connected
gripper:                      # null = no gripper
  backend: franky             # franky (default) | serl_http | fake
  max_width_m: 0.08
  mode: binary                # binary (default) | continuous
  binary_threshold: 0.5       # binary mode: an action > threshold opens
  shares_arm_session: false   # the Franka Hand opens its own connection
cameras:                      # default: []
  - name: wrist
    backend: realsense        # realsense (default) | fake
    serial: ""                # "" = use the only attached camera; required when >1 is attached
    width: 640
    height: 480
    fps: 30
    enable_depth: false
```

**AgileX Piper (CAN)**

```yaml
type: piper
name: piper
arm_backend: pyagxarm         # pyagxarm (default) | piper_sdk | fake
can_interface: can0           # default can0
bitrate: 1000000              # must match the interface (default 1000000)
firmware_profile: null        # pyagxarm only: default | v183 | v188 | v189; null probes the arm
home_joints: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 6 values, rad
safety_box: [0.1, -0.3, 0.1, 0.5, 0.3, 0.5]   # m, base frame; tcp_pose is the FLANGE pose (no tool offset)
joint_limits: [[-2.6, 2.6], [0.0, 3.1], [-2.9, 0.0], [-1.7, 1.7], [-1.2, 1.2], [-2.0, 2.0]]  # rad
max_delta_per_step: {xyz_m: 0.02, joint_rad: 0.05}
claim: true
gripper:
  backend: pyagxarm           # must match the arm backend: pyagxarm | piper_sdk
  max_width_m: 0.07
  mode: continuous            # Piper default
  binary_threshold: 0.5
  shares_arm_session: true    # required: the Piper gripper borrows the arm's CAN session
cameras: []
```

Safety limits and teleop (all optional, all three keys are read from the robot PC's yaml):

- `safety_box`: `[xmin, ymin, zmin, xmax, ymax, zmax]` in metres, in the frame of
  `read_state()["tcp_pose"]` (Franka: `O_T_EE`, Piper: the **flange**, not the gripper tip).
  Only the position is limited; the orientation is left untouched.
- `joint_limits`: one `[lo, hi]` pair per joint, in rad. `home_joints` must lie inside it.
- `max_delta_per_step`: `xyz_m` and/or `joint_rad`, a cap on how far a servo target may be from
  the measured state. Servo only; `move_to_*` is not capped.
- The yaml on the robot PC is the source of truth. `servo_*` targets outside the limits are
  **clipped** and counted (`safety_clips`); `move_to_*` targets outside them are **refused** with
  `SafetyViolation` and nothing is sent (`safety_rejects`). The same applies through the daemon,
  `send_action`, and the local API, see [Safety rules](#safety-rules).
- `teleop:` adds an operator input device as a part of the robot (see [step 7](#7-add-a-teleop-device)).

Notes:

- `shares_arm_session: true` is mandatory for the `pyagxarm` and `piper_sdk` grippers and
  forbidden for `franky` and `serl_http`. A mismatch raises `ValueError` at build time.
- With `arm_backend: piper_sdk`, also set `gripper.backend: piper_sdk`. That backend ignores
  `firmware_profile` and always opens the bus at 1 Mbit/s, so `bitrate` is only checked
  against the interface.
- `serl_http` has no joint-space control and needs the legacy bridge running (see [Legacy](#legacy)).

Build a robot without touching hardware to check the file:

```python
from robot_infra.devices import Robot

robot = Robot.from_yaml("robot.yaml")   # builds parts; does not connect
print(robot.describe_text())
```

```text
franka (franka)
  arm: FrankyArm [sdk=franky, connected=False]
  gripper: FrankyGripper [sdk=franky, connected=False]
  cameras: PartGroup [sdk=None, connected=False]
    wrist: RealSenseCamera [sdk=pyrealsense2, connected=False]
```

With a `teleop:` section a last line is added, for example `  teleop: SpaceMouseTeleop [sdk=hid, connected=False]`.

### 2. Plug in and discover

```bash
rwrl discover --station robot.yaml
```

Discovery only probes. It never connects to a robot. `--station` is optional: the given
config's (or else the [station file's](#setup-wizard)) `robot_ip` of every robot is pinged and
TCP-probed on ports 22, 80, 443, 1337 and 50051.

On a machine with nothing attached (real output):

```text
$ rwrl discover --station robot.yaml
cameras:
  none
teleops:
  none
can interfaces:
  none
hosts:
  172.16.0.2: ping=no reply open ports=none
hid devices:
  none
```

With hardware attached, the lines take the form `realsense: <serial>, <serial>`,
`spacemouse: <vid>:<pid>:<path>` (or `/dev/hidrawN` when `hidapi` is not installed),
`can0: operstate=up bitrate=1000000`, `172.16.0.2: ping=ok open ports=<list>` and
`/dev/hidraw0`. The `fake` backends are not listed.

`rwrl --station robot.yaml --json discover` (or `rwrl --json discover`) prints the same
report as one JSON object: `schema_version: 1` and six lists, each empty when nothing is found,
`cameras` (`backend`, `serial`), `teleops` (`backend`, `device`), `can_interfaces` (`name`,
`operstate`, `bitrate`), `hosts` (`ip`, `ping_ok`, `open_ports`), `hid_devices` (paths) and
`unknown_hid` (`vid`, `pid`, `product`, `path`, `message`: HID devices that look like a SpaceMouse
but are missing from the device table). The machine above gives (derived from the same report,
not a captured run):

```json
{"schema_version": 1, "cameras": [], "teleops": [], "can_interfaces": [], "hosts": [{"ip": "172.16.0.2", "ping_ok": false, "open_ports": []}], "hid_devices": [], "unknown_hid": []}
```

| Section | Source | What to look for |
|---|---|---|
| `cameras` | each camera backend's `discover()` (RealSense serials) | copy the serial into `cameras[].serial` |
| `teleops` | each teleop backend's `discover()` (SpaceMouse: `vid:pid:path` through `hidapi` when installed, else every `/dev/hidraw*`; gello: `/dev/serial/by-id/usb-FTDI_*` and `usb-ROBOTIS_*`) | the device you put in `teleop:` shows up |
| `can interfaces` | `/sys/class/net/*` with type CAN, bitrate from `ip -details link show` | `operstate=up` and the bitrate in your yaml |
| `hosts` | `ping` and TCP connect to the config's `robot_ip` | `ping=ok`. For Franka, 1337 open usually means FCI is reachable. |
| `hid devices` | `/dev/hidraw*` | SpaceMouse and other HID teleop devices |
| `warnings` (only when there are any) | 3Dconnexion (`256f`) HID devices, and Logitech (`046d`) multi-axis controllers, whose id is missing from the SpaceMouse device table | `WARN unknown 3Dconnexion-vendor HID device <vid>:<pid>; add it to spacemouse-devices.yaml (see docs/spacemouse-commissioning.md#site-device-table) (<path>)`: add a [site device table](../spacemouse-commissioning.md#site-device-table) entry |

Without `--station` and without a station file, `hosts:` prints `none`.

### 3. Check with the doctor

```bash
rwrl doctor --station robot.yaml [--robot NAME] [--mock] [--fix] [--json] [--install-sdk [--yes]]
```

`rwrl doctor` runs the device checks below for every station robot, next to its host and
project checks, and reports each one as a `<robot>.<check>` row (for example `arm.handshake`)
with `PASS`, `FAIL`, `WARN` or `SKIPPED`; see [cli.md](../cli.md#robot-host-diagnosis-and-repair)
for its report, flags and exit codes. The `[layer/5]` listings in this section show the
device checks as the device doctor library (`robot_infra.doctor.cli`) formats them, which is
also what the `rwrl daemon` preflight prints.

Without `--station` the doctor checks the [station file](#setup-wizard). A station with several
robots is checked robot by robot; `--robot NAME` checks one.

The doctor runs six layers in order, 0 to 5 (a `teleop:` section adds checks to layers 0, 1, 2 and 4):

| Layer | Checks | Notes |
|---|---|---|
| 0 `deps` | the pip extras this robot needs are installed: one per backend (`franky`, `pyagxarm`, `piper-sdk`, `realsense`, `spacemouse`, `gello`, plus `gello-events` for a gello `event_device`), and `grpc` when the robot's yaml has a `daemon:` block. FAIL `missing extras for this station: franky, gello`, hint `pip install -e '.[franky,gello]' (from the repository root)`; OK `extras ok: franky, gello, grpc`. Without a `daemon:` block (the single-process route) a missing `grpc` is a WARN `optional, not installed: grpc (daemon route)`; a missing `jpeg` is only mentioned | with `--mock` the fakes need none and grpc is never required |
| 1 `sdk` | each backend name resolves and its SDK imports (`sdk:arm`, `sdk:gripper`, `sdk:camera:<name>`, `sdk:teleop:<backend>`) | an unknown backend skips layers 3-5 |
| 2 `os` | ping `robot_ip` and port 1337 (informational); CAN interface exists / is up / bitrate; camera serial attached; `os:teleop:<name>` (a device of that backend is discovered); `os:hid` (a readable and writable `/dev/hidraw*` exists, SpaceMouse only); `os:serial` (the gello `port` exists and is readable and writable; `--fix` tries `setfacl -m u:$USER:rw <port>` without sudo); `os:groups` (the user's login session is in `dialout` for the gello port and `input` for its `event_device`); `os:brltty` (brltty would claim a CH340 adapter); `os:latency_timer` (an FTDI adapter's `latency_timer` is 1 ms) | driven by config fields; skipped for backends that need no SDK (fakes); never runs sudo |
| 3 `handshake` | takes the robot's device claim (unless `claim: false`), then `robot.connect()` | a failure (including a held claim) skips layers 4-5 |
| 4 `feedback` | the arm returns state; the gripper reports its width; each camera delivers a frame within 2 s; `feedback:teleop:<name>`: the teleop poll thread is alive and healthy | |
| 5 `mode` | backend preflight: Franka RT kernel, robot mode and libfranka version; Piper status, follower feedback and firmware profile (pyagxarm) or gripper stroke (piper_sdk) | |

`feedback:gripper` prints the width in metres, its fraction of `max_width_m`, grasp status and
backend source; non-finite values, a fraction outside [0, 1] or `max_width_m <= 0` produce a
`WARN` with a hint to check `max_width_m` and the SDK's width units.

Flags:

- `--mock` swaps every backend for its fake and skips layer 2. Use it to check the YAML and
  the install with no hardware.
- `rwrl doctor --fix` repairs the host and software only; it never recovers or reopens a
  device. A Franka in Reflex/UserStopped/Guiding mode (`mode:robot_mode`) is cleared with
  `rwrl recover`, and a camera that delivers no frames is checked again with `rwrl status`
  after reconnecting it; the device rows' hints say so.
- `--json` prints `rwrl doctor`'s report (`schema_version: 1`, `status`, `checks`,
  `repairs`); the device checks of each robot are under `results` as a list of
  `{layer, name, ok, message, hint, skipped, warning}` objects keyed by robot name.
- `--install-sdk` (with `--yes`: without asking) installs the franky wheel that matches the
  robot after a libfranka version mismatch; see
  [Installing the right franky wheel](#installing-the-right-franky-wheel).
- Exit code: `1` if any check failed (on any robot), else `0`. Skipped checks do not fail the run.
  A `WARN` line is an ok check with a hint worth acting on (for example a CH340 adapter that
  brltty may claim on the next replug); it never fails the run, and the summary counts it as
  `N ok (M with warnings)`.

The doctor connects to the robot and disconnects it again. It never powers anything off.

The device checks of a `--mock` run of the Franka example:

```text
[0/5] deps ... OK  no extras needed (fake or SDK-free backends)
[1/5] sdk:arm ... OK  no SDK needed
[1/5] sdk:gripper ... OK  no SDK needed
[1/5] sdk:camera:wrist ... OK  no SDK needed
[2/5] os:ping ... SKIP  skipped (--mock)
[2/5] os:tcp ... SKIP  skipped (--mock)
[2/5] os:camera:wrist ... SKIP  skipped (--mock)
[3/5] handshake ... OK  robot connected
[4/5] feedback:arm ... OK  arm feedback received
[4/5] feedback:gripper ... OK  gripper width 0.0800 m (1.00 of max_width_m 0.080), grasped=unknown, source FakeGripper
[4/5] feedback:camera:wrist ... OK  frames arriving
[5/5] mode:none ... OK  backend declares no mode checks
summary: 9 ok, 0 failed, 3 skipped
```

The same config on a machine with no franky, no robot and no camera:

```text
[0/5] deps ... FAIL  missing extras for this station: franky
    hint: pip install -e '.[franky]' (from the repository root); if the handshake then reports a libfranka mismatch, rerun the doctor with --install-sdk
[1/5] sdk:arm ... FAIL  FrankyArm needs the 'franky' package
    hint: pip install franky-control (or, from the repository root: pip install -e '.[franky]')
[1/5] sdk:gripper ... FAIL  FrankyGripper needs the 'franky' package
    hint: pip install franky-control (or, from the repository root: pip install -e '.[franky]')
[1/5] sdk:camera:wrist ... OK  'pyrealsense2' importable
[2/5] os:ping ... FAIL  172.16.0.2 does not reply
    hint: check: robot powered on and brakes released; Desk reachable at https://172.16.0.2/desk; you hold control in Desk; FCI activated (Desk → Activate FCI); cable on the Control port; firewall allows UDP
[2/5] os:tcp ... OK  port 1337 closed (informational)
[2/5] os:camera:wrist ... FAIL  no RealSense device found (serial unset)
    hint: plug in the camera and run `rwrl discover`
[3/5] handshake ... FAIL  FrankyArm needs the 'franky' package
    hint: pip install franky-control (or, from the repository root: pip install -e '.[franky]')
[4/5] feedback ... SKIP  skipped: handshake failed
[5/5] mode ... SKIP  skipped: handshake failed
summary: 2 ok, 6 failed, 2 skipped
```

With a `teleop: {backend: spacemouse}` section, `--mock` swaps the teleop for the fake one
(its options are dropped) and the extra lines are:

```text
...
[1/5] sdk:teleop:fake ... OK  no SDK needed
[2/5] os:teleop:teleop ... SKIP  skipped (--mock)
...
[4/5] feedback:teleop:teleop ... OK  teleop poll thread alive
```

Without `--mock`, on a machine without `hidapi` and without a SpaceMouse, the teleop
lines read (the `os:hid` line is only checked for the SpaceMouse backend):

```text
[1/5] sdk:teleop:spacemouse ... FAIL  SpaceMouse needs the 'hidapi==0.14.0.post4' distribution (import hid)
    hint: pip install -e '.[spacemouse]'
[2/5] os:teleop:teleop ... FAIL  no 'spacemouse' teleop device found
    hint: plug in the teleop device and run `rwrl discover`
[2/5] os:hid ... FAIL  no HID device found
    hint: no /dev/hidraw* node: check the USB cable and port; inside Docker pass it with `docker run --device=/dev/hidrawN`
```

When a `/dev/hidraw*` node exists but this user cannot open it, `os:hid` fails with
`no read/write access to ['/dev/hidraw3']` and a hint to install the
[packaged vendor rules](#spacemouse-python-dependencies-and-udev) with `sudo cp`,
join `plugdev`, reload the rules, log in again and replug. A missing Python SDK
instead gives the pip command above (NUC, repository root, project environment
active); native import failures report wheel/platform loading separately.

The Piper example with no CAN adapter plugged in (trimmed):

```text
[2/5] os:can_exists ... FAIL  CAN interface can0 not found
    hint: run `bash can_activate.sh can0 1000000` from piper_sdk, or `sudo ip link set can0 up type can bitrate 1000000`
[2/5] os:can_up ... FAIL  can0 operstate=missing
[2/5] os:can_bitrate ... FAIL  can0 bitrate=None, expected 1000000
[3/5] handshake ... FAIL  CAN interface 'can0' not found
    hint: plug in the USB-CAN adapter and run: bash can_activate.sh can0 1000000  (piper_sdk) or: sudo ip link set can0 up type can bitrate 1000000
```

Without pyAgxArm installed, the SDK hint reads `pip install -e '.[pyagxarm]' (from the repository root; pyAgxArm is installed from GitHub)`.
pyAgxArm is not on PyPI, so the extra installs it from GitHub.

#### Troubleshooting

The table quotes the messages and hints these checks print. `<ip>`, `can0` and similar
values are filled in from your config.

| Symptom | Where / what you see | What to do |
|---|---|---|
| Missing pip extras | `deps` FAIL `missing extras for this station: franky, gello`, hint `pip install -e '.[franky,gello]' ...` | Run the quoted command from the repository root (the list is exactly the missing extras). |
| Franka unreachable: robot off, brakes locked, no control or FCI in Desk, cable on the Arm port, firewall | `os:ping` FAIL `<ip> does not reply` or `handshake` FAIL `cannot reach Franka at <ip>: ...` (for example `libfranka: Connection to FCI refused. Please install FCI feature or enable FCI mode in Desk.`), hint `check: robot powered on and brakes released; Desk reachable at https://<ip>/desk; you hold control in Desk; FCI activated (Desk → Activate FCI); cable on the Control port; firewall allows UDP` | Work through the hint in order. A Desk page that loads is not enough: you must hold control and FCI must be activated. Plug the PC into the Control box (not the arm base). Allow UDP from the robot through the firewall. |
| libfranka version mismatch | `handshake` FAIL `libfranka mismatch: robot server 10, installed library 9`, hint `the robot server version 10 comes from libfranka's handshake error (authoritative); to cross-check, read the system version in Desk → Settings → System and look it up in the compatibility matrix https://frankarobotics.github.io/docs/compatibility.html; then: rwrl doctor --station <cfg> --install-sdk` | See [Installing the right franky wheel](#installing-the-right-franky-wheel). `mode:libfranka_version` shows the installed libfranka once connected. |
| User not in `dialout` / `input` | `os:groups` FAIL `not in group(s) dialout, input; no read/write access to [...]` (WARN when an ACL or udev rule grants access anyway), hint `sudo usermod -aG dialout,input $USER, then log out and back in` | Run the command, then log in again: a running session keeps its old groups. |
| Gello port not readable | `os:serial` FAIL `no read/write access to <port>`, hint `grant this user access now: setfacl -m u:$USER:rw <port> ...` | `--fix` tries the `setfacl` without sudo; otherwise prefix sudo. An ACL is lost on replug; the group or a udev rule is permanent. |
| brltty claims a CH340 adapter | `os:brltty` FAIL `brltty active and a CH340 adapter (USB id 1a86) is attached; brltty has likely claimed <port>` (WARN while the port still exists), hint `sudo systemctl mask --now brltty-udev.service brltty.service, then replug the adapter` | Mask brltty (a `--runtime` mask is lost on reboot), then replug. |
| Slow FTDI reads | `os:latency_timer` WARN `ttyUSB0 latency_timer is 16 ms; 1 ms is recommended for Dynamixel reads`, hint with the udev rule `ACTION=="add", SUBSYSTEM=="usb-serial", DRIVER=="ftdi_sio", ATTR{latency_timer}="1"` | Install the udev rule (the hint names the file and the reload command) and replug. CH340 adapters have no `latency_timer`. |
| No PREEMPT_RT kernel | with `realtime: enforce`: `mode:realtime_kernel` FAIL `realtime: enforce but the kernel is not PREEMPT_RT`, or `handshake` FAIL `libfranka realtime setup failed ...`; with `realtime: ignore` the check passes with `control jitter possible` | Install a PREEMPT_RT kernel and realtime group limits, or use `realtime: ignore`. |
| Franka in Reflex / UserStopped / Guiding | `mode:robot_mode` FAIL `robot mode is Reflex`, hint `release the external activation device / clear errors in Desk, then run with --fix (recover)` | Release the activation device, then run the doctor with `--fix`. |
| CAN interface missing / down / wrong bitrate | `os:can_exists`, `os:can_up`, `os:can_bitrate` FAIL (see output above); `handshake` FAIL `CAN interface 'can0' is down, not up` or `runs at N bit/s, expected 1000000` | `sudo ip link set can0 up type can bitrate 1000000` (or piper_sdk's `can_activate.sh can0 1000000`). |
| Piper does not enable (power, e-stop) | `handshake` FAIL `Piper on 'can0' did not enable within 5s`, hint `check that the arm is powered, that 'can0' is up at 1000000 bit/s, and that its emergency stop is released` | Power the arm, release the e-stop, rerun. `mode:piper_status` names the arm status once connected. |
| No CAN feedback (arm in master/teach mode) | `handshake` FAIL `... enabled but sent no joint feedback within 2s`, or `mode:follower` FAIL `no joint feedback`; hint `no CAN feedback: the arm may be in master/teach mode; switch it to follower (slave) mode and power-cycle` | Switch the arm to follower (slave) mode and power-cycle it. |
| Wrong Piper firmware profile (pyagxarm) | `handshake` FAIL `firmware probe ... failed` / `cannot map Piper firmware ...`, hint `set firmware_profile explicitly (default\|v183\|v188\|v189) in the robot config` | Set `firmware_profile` in the yaml. |
| Camera serial not found | `os:camera:<name>` FAIL `camera '<name>' serial '<s>' not found`, hint `set serial; discovered: [...]` (with `serial` unset: `no RealSense device found (serial unset)` or `serial unset and N devices attached; set serial to one of [...]`); `handshake` FAIL `RealSense serial '<s>' for camera '<name>' not attached` | Copy a serial from `rwrl discover`. With more than one camera attached, `serial` is required. |
| Camera gives no frames | `feedback:camera:<name>` FAIL `no frames within 2s`, hint `check USB 3 bandwidth; try another port; run with --fix to reopen`; or `RealSense '<s>' failed to start`, hint `check the USB 3 cable/port and udev rules (librealsense scripts/setup_udev_rules.sh)` | Use a USB 3 port and cable (not a hub shared with other cameras), install the librealsense udev rules, then try `--fix`. |
| Daemon unreachable | client `TransportError: robot daemon unreachable at <addr>` | Start the daemon, check `--bind`, the address and the firewall. |
| Token mismatch | client `TransportError: token mismatch`, hint `pass the same --token to the daemon and the remote config` | Use the same token on both sides. |
| Watchdog stopped the arm | client `DeviceFault: the daemon watchdog stopped the arm after 0.62s without actions (limit 0.5s)`, hint `... call set_control_mode(...) again to resume servoing, and keep the action rate above 1/watchdog_s` | Send actions faster than `1/watchdog_s`, or start the daemon with a larger `--watchdog-s`. Then call `set_control_mode` again. |
| Robot already in use | client `DeviceBusy: device 'franka:172.16.0.2' is claimed by another process`, hint `stop the other process using 'franka:172.16.0.2', or set `claim: false` in the robot config` | Stop the other process on this host that serves this robot through `RobotService` (a daemon or a `LocalClient` setup). Plain `Robot.from_yaml` scripts do not take the claim. To opt out deliberately, set `claim: false` in the yaml. |

#### Installing the right franky wheel

The `franky` extra installs the default `franky-control` wheel, which bundles one libfranka
version. A robot whose robot server version differs refuses the handshake, and the doctor
reports `libfranka mismatch: robot server <N>, installed library <M>`. Then:

```bash
rwrl doctor --station robot.yaml --install-sdk         # asks before installing
rwrl doctor --station robot.yaml --install-sdk --yes   # installs without asking
```

With `--install-sdk` the doctor prints the robot server version from libfranka's handshake error
(authoritative). Desk does not show that number: to cross-check, read the system version (for
example 5.9.2) in Desk → Settings → System and look it up in the
[compatibility matrix](https://frankarobotics.github.io/docs/compatibility.html), which maps
system versions to robot server versions. Then the doctor prints the exact command

```bash
<this python> -m pip install franky-control --extra-index-url https://timschneider42.github.io/franky/whl/by-robot-server-version/<N>/ --force-reinstall --no-deps
```

and asks `Install now? [y/N]`. Only `y` (case-insensitive, surrounding whitespace ignored) installs (into the interpreter running the
doctor); then it re-runs the doctor for those robots in a fresh interpreter, because the old
libfranka stays loaded in the current one, and exits with that run's code. Nothing is installed
without confirmation unless `--yes` is given. With `--json` the doctor never prompts or installs:
its envelope wraps `{"results": <the original device report>, "install_sdk": [{robot, server_version,
library_version, command}]}`. Robots that report different server versions are not installed
automatically (one Python environment holds one franky wheel); the doctor prints one command per
robot instead. `--no-deps` assumes franky's dependencies (`numpy`, `websockets`) are already
installed, which installing the `franky` extra first takes care of.

With `real-world-rl` installed in the project environment, `rwrl doctor` is a separate,
broader doctor: it checks the host, the project environment and the configured devices,
previews targeted repairs (`--fix` applies host and software repairs), and writes its own
versioned JSON envelope with host checks and repairs; its device probes never repair
(`fix=False`). Run it on the NUC with the project environment active (see
[rwrl](../cli.md#robot-host-diagnosis-and-repair)).

### 4. Serve the robot

```bash
pip install -e '.[grpc]'   # plus the robot's backend extras
rwrl daemon --station robot.yaml --bind 0.0.0.0:50051 [--token T] [--watchdog-s 0.5] [--no-preflight]
```

| Flag | Default | Meaning |
|---|---|---|
| `--station` | the [station file](#setup-wizard) | robot or station yaml |
| `--robot` | every robot | serve one robot of a station; without it a station with several robots starts one daemon process per robot |
| `--bind` | the robot's `daemon.bind`, else `0.0.0.0:50051` | `host:port` to listen on (single robot only) |
| `--token` | the robot's `daemon.token`, else none | shared secret; clients must send the same one (gRPC metadata `x-robot-token`) (single robot only) |

With several robots and no `--robot`, the command is a supervisor: it starts
`rwrl daemon --station <station> --robot <name>` per robot (each in its own
process session, with `--watchdog-s`, `--jpeg-quality`, `--no-preflight` and `--transport`
passed on), forwards SIGINT/SIGTERM to them, waits for all and exits with the first non-zero
child exit code. `--bind`/`--token` are then an error (set `daemon.bind`/`daemon.token` per
robot), and so are two robots on the same `daemon.bind`. Killing the supervisor with SIGKILL
leaves the daemons running; stop them with SIGTERM.
| `--watchdog-s` | `0.5` | idle a servoing arm after this many seconds without actions |
| `--no-preflight` | off | skip the doctor run before serving |
| `--transport` | `grpc` | the only transport |
| `--jpeg-quality` | none | JPEG quality 1-100 for every camera's `color` image; see below |

`--jpeg-quality N` (needs `pip install -e '.[jpeg]'`, Pillow or OpenCV, on the daemon **and** on
every client) sends each camera's `color` image as a JPEG instead of raw bytes, which cuts the
bandwidth over WiFi. It is **lossy** and daemon-global: only `color` is compressed, `depth` and
all other fields stay raw, and the client decodes back to a `uint8 (H, W, 3)` array. The policy
therefore sees compressed pixels, so **train and deploy with the same value** (or none). The
value is reported by `robot.describe` as the top-level `image_encoding` (`{"jpeg": 80}`, or `null`
when raw) and as `robot.cameras["wrist"].image_encoding` on the client. A client without
Pillow/OpenCV fails at `connect()` with `SdkMissing: JPEG frames need Pillow or OpenCV`
(hint `pip install -e '.[jpeg]' (from the repository root)`).

Daemon and client ship from the same repo; upgrade the **client first, then the daemon**. A new
daemon's `arm.health` carries two extra fields (`safety_clips`, `safety_rejects`) that a
pre-upgrade client cannot parse, and enabling `--jpeg-quality` needs a client that can decode
JPEG. A client that predates JPEG support cannot decode the frames (its error was not checked).

What happens:

1. **Config.** The yaml is loaded. Then every config field that is empty (`""` or null) is
   filled from the environment variable of the same name in upper case, for example
   `FIRMWARE_PROFILE=v188`. `ROBOT_IP` and `CAN_INTERFACE` apply only if the yaml sets
   `robot_ip: ""` / `can_interface: ""`, because both have non-empty defaults. A camera
   with an empty `serial` takes the serial when exactly one camera of its backend is
   attached. The daemon, the doctor and `rwrl discover --station` all apply
   this autofill; `Robot.from_yaml` reads the yaml as written.
2. **Preflight.** Unless you pass `--no-preflight`, the full doctor (not `--mock`) runs
   and prints its report. If any check fails, the daemon prints `preflight failed; fix the
   items above or start with --no-preflight` and exits with code 1. The doctor connects
   and disconnects the robot while holding its device claim, so a running daemon on the same
   robot is reported as a failed `handshake`. Fake backends skip the OS checks
   (`backend 'fake' needs no OS device`), so demo or fake configs start without
   `--no-preflight`.
3. **Startup output** (from a fake-backend config):

   ```text
   franka (franka)
     arm: FakeArm [sdk=None, connected=False]
     gripper: FakeGripper [sdk=None, connected=False]
     cameras: PartGroup [sdk=None, connected=False]
       wrist: FakeCamera [sdk=None, connected=False]
     teleop: FakeTeleop [sdk=None, connected=False]
   serving franka 'franka' on 127.0.0.1:50061 (token required)
   ```

   The robot is connected only when a client calls `connect()`. At that point the daemon
   takes the device claim, and it releases the claim on `disconnect()`.
4. **Watchdog.** While the arm is in `CARTESIAN_SERVO` or `JOINT_SERVO`, the daemon checks
   every `watchdog_s / 4` how long it has been since the last action. Actions are
   `send_action`, `step`, `servo_cartesian` and `servo_joints`; entering servo mode also
   starts the clock. After more than `watchdog_s` with no action, the daemon calls
   `arm.stop()`, switches the arm to `IDLE`, and logs `watchdog: no action for 0.624s in
   joint_servo mode; stopping arm 'arm' and idling it`. Later servo commands raise the
   watchdog `DeviceFault` until the client calls `set_control_mode(...)` again.
   `move_to_*` calls (which hardware backends allow only in `IDLE`) are not watched.
5. **Teleop.** If the yaml has a `teleop:` section the daemon serves `teleop.<name>.read` and
   `teleop.<name>.reset`. Reads do not refresh the watchdog, and a client's `timeout_s` is
   clamped to 2 s so a client cannot park a gRPC worker thread. A `teleop` section is
   mandatory once present: `connect()` fails when the device is absent and takes the whole
   robot down with it, so a deployment that needs no teleop should use a yaml without the section.
6. **Ctrl-C / SIGTERM.** The daemon stops the transport, stops the watchdog, disconnects the
   robot and releases the claim. It never powers the robot off and never disables the
   motors (see [Safety rules](#safety-rules)).

`robot.describe` is what a client builds its mirror from. For the fake robot above (trimmed):

```text
arm: {"num_joints": 7, "supported_state_fields": ["tcp_pose", "tcp_vel", "joint_pos", "joint_vel", "tcp_force", "tcp_torque"], "control_mode": "idle", "safety": {"safety_box": [-0.5, -0.5, -0.5, 0.5, 0.5, 0.5], "joint_limits": [[-1.0, 1.0], ...], "max_delta_per_step": {"xyz_m": 0.05, "joint_rad": 0.1}}, "home_joints": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}
image_encoding: {'jpeg': 80}
camera cfg: {'name': 'wrist', 'serial': '', 'width': 64, 'height': 48, 'fps': 30, 'enable_depth': False, 'backend': 'fake'}
teleop: {'name': 'teleop', 'backend': 'fake', 'action_space': 'joint_abs', 'has_gripper': True, 'hold_s': 0.0}
```

`arm.safety` is `null` when no limit is configured; `arm.home_joints` is the yaml's value (or `null`).
The description also includes `arm.safety_capabilities`: `task_workspace_v1` for Cartesian
workspace enforcement, `accepted_cartesian_target_v1` for accepted-target holding, and, on
Franky, `stop_trace_v1`. Clients negotiate capabilities before using new behavior; older
servers cannot certify support. Connected teleop descriptions carry `device_identity` (USB
vendor/product and available serial/model); `RemoteTeleop.device_identity` exposes that same
identity for profile verification. Before connecting, identity may be unavailable.

Four RPCs supplement the existing arm commands: `arm.last_cartesian_target`,
`arm.begin_stop_trace`, `arm.stop_with_trace` (`post_stop_s=2.1`), and
`arm.last_stop_trace`. `last_cartesian_target` is a copy of the latest Cartesian target in the
current control segment that passed the arm's safety limits and was handed to the backend.
Any `stop()`, `stop_with_trace()`, control-mode switch or disconnect ends the segment, and
the value is then `null` until the next successful `servo_cartesian`. It is never measured
feedback. `piper_sdk` and `pyagxarm` still convert the target in the backend (millimetre or
milli-degree rounding, quaternion to RPY), so it is not exactly what those arms received.
`Arm.stop()` is a template: it clears the target and then calls the backend's `_stop()`.
A backend implements `_stop()`, never `stop()`. `begin_stop_trace` starts bounded sampling
on the existing connection;
SDK fingerprints/static metadata are collected outside the stop/service locks and cached
per connection. Prior sampler joins also occur outside the stop lock.

`stop()` retains its ordinary Franky error semantics: failures are noted in health without
raising. Only `stop_with_trace` raises `DeviceFault` for stop failure, retaining FAIL evidence
in `last_stop_trace`. It sends stop before attempting to create a missing recorder; preparation
failure produces BLOCKED evidence after direct stop. Read/stop RPCs remain available during
post-stop observation, while control writes are refused. Slow camera acquisition in
`robot.step` does not hold the dispatch lock needed by `arm.stop`.

Stop traces record `control_mode` and `tracker_active` at stop start, and each sample's SDK
`robot_mode`. An already IDLE/inactive arm yields BLOCKED (`no active motion at stop`), even
if every pose is stationary. Robot-mode faults and stop failures yield FAIL; missing or
unknown mode/coverage evidence cannot yield PASS. Sampled stop acceptance is not a
continuous-time physical guarantee.

### 5. Use the robot from the GPU machine

```bash
pip install -e '.[grpc]'
```

```python
import robot_infra.server  # registers robot type "remote"
from robot_infra.devices import ControlMode, Robot

robot = Robot.from_dict({"type": "remote", "address": "robot-pc:50051", "token": "T"})
robot.connect()                       # builds arm/gripper/cameras from the daemon's describe
print(robot.describe_text())
robot.arm.set_control_mode(ControlMode.JOINT_SERVO)
q = robot.arm.read_state()["joint_pos"]
obs = robot.step({"arm": {"joint_pos": q}, "gripper": {"gripper": [1.0]}})  # action, then observation
print(robot.health())                 # Health(ok=True, detail='rtt=1.2 ms, watchdog_trips=0', ...)
robot.arm.set_control_mode(ControlMode.IDLE)
robot.disconnect()
```

Without `import robot_infra.server`, `Robot.from_dict` raises
`unknown robot type 'remote'; registered: ['franka', 'piper']`.

- Actions are nested per part, and the arm key depends on the control mode: `tcp_pose`
  (7 values: xyz plus quaternion xyzw) in `CARTESIAN_SERVO`, `joint_pos` in
  `JOINT_SERVO`. The gripper takes a normalized width in [0, 1].
- `step()` returns `{"arm": {...}, "gripper": {"gripper_pos": ...}, "cameras": {<name>:
  {"color": ...}}}`. The arm fields are the backend's `SUPPORTED_STATE_FIELDS`.
- Other `RemoteRobotConfig` keys: `timeout_s` (5.0), `reconnect_attempts` (5),
  `reconnect_backoff_s` (0.5). After a dropped connection, the client reconnects and
  retries only read-only calls. Commands are never retried: reconnect and set the control
  mode again.
- For single-machine use without gRPC, wrap a local robot in-process:

  ```python
  from robot_infra.devices import Robot
  from robot_infra.server import RemoteRobot, RobotService
  from robot_infra.server.transport import LocalClient

  service = RobotService(Robot.from_yaml("robot.yaml"), watchdog_s=0.5)
  robot = RemoteRobot.from_client(LocalClient(service))   # same API as above, incl. watchdog
  ```

- The daemon's device-side facts are mirrored on the client:
  `robot.arm.safety_limits` (the dict under `arm.safety` above, or `None`),
  `robot.arm.home_joints` (list or `None`), `robot.cameras["wrist"].image_encoding`, and
  `robot.teleop` (`None` without a `teleop:` section). The client does not enforce limits
  itself (`RemoteRobotConfig` ignores `safety_box`, `joint_limits`, `max_delta_per_step` and
  `teleop`); the daemon does.
- A command outside the limits behaves as follows on the client (run it before `robot.disconnect()`):

  ```python
  import numpy as np
  from robot_infra.devices import SafetyViolation

  robot.arm.set_control_mode(ControlMode.JOINT_SERVO)
  robot.arm.servo_joints(np.full(7, 5.0))      # no error: clipped on the daemon, safety_clips += 1
  robot.arm.set_control_mode(ControlMode.IDLE)
  try:
      robot.arm.move_to_joints(np.full(7, 5.0))   # outside joint_limits
  except SafetyViolation as error:
      print(error.message, error.hint)            # nothing was sent; the arm and its mode are unchanged
  print(robot.arm.health().safety_clips, robot.arm.health().safety_rejects)   # cumulative counters since the daemon started, e.g. 2 1
  print(robot.remote_health()["arm"]["safety_clips"])                         # the same counters from the daemon's robot.health
  ```

  `SafetyViolation` is a `DeviceError` (not a `DeviceFault`): it does not latch a fault, does
  not need `recover()`, and does not refresh the daemon watchdog. `robot.health()` is a
  `Health`; the per-arm counters live in `robot.arm.health()` and `robot.remote_health()`.
- `real-world-rl` wraps this robot in a gym environment.

### 6. Use the robot locally (no daemon)

On the robot PC, you can drive the device layer directly:

```python
import numpy as np
from robot_infra.devices import ControlMode, Robot

robot = Robot.from_yaml("robot.yaml")
robot.connect()
robot.arm.set_control_mode(ControlMode.CARTESIAN_SERVO)
pose = robot.arm.read_state()["tcp_pose"]
robot.send_action({"arm": {"tcp_pose": pose}, "gripper": {"gripper": np.array([0.0])}})
obs = robot.get_observation()
robot.arm.set_control_mode(ControlMode.IDLE)
robot.disconnect()
```

This path has no watchdog and takes no device claim. Those are daemon features. The safety
limits, however, are enforced here too: `Robot.from_yaml` builds the arm with the yaml's
`safety_box`/`joint_limits`/`max_delta_per_step`, so `robot.arm.safety_limits`,
`robot.arm.health().safety_clips` and `SafetyViolation` behave exactly as through the daemon.

#### Supervising a single-process robot run

On the robot host, from the repository root with the full project environment active,
use `rwrl supervise` and `rwrl record`. Without a daemon, the robot process drives the arm through the in-process SDK, so nothing outside it can call `arm.stop()`. `rwrl supervise` runs it as the child of a separate watchdog process:

    rwrl supervise --log runs/<run>/supervise.jsonl -- rwrl record --config hil.yaml ...

| Option | Default | Meaning |
|---|---|---|
| `--heartbeat-timeout-s` | `2.0` | silence after which SIGINT, then SIGTERM, then SIGKILL go to the child's process group |
| `--grace-s` | `3.0` | SIGINT to SIGTERM |
| `--kill-s` | `2.0` | SIGTERM to SIGKILL |
| `--startup-s` | `30.0` | no heartbeat required until the first message or this elapses |
| `--long-op-s` | `60.0` | silence allowed after the child announces a long operation (an env reset) |
| `--log` | none | jsonl: pid, escalations with monotonic timestamps, exit code/signal |

The child runs in its own session and receives the write end of a pipe in `ROBOT_INFRA_HEARTBEAT_FD`. `robot_infra.supervise.Heartbeat.from_env()` is the child side: `beat()` once per step, `begin_long_operation()` before a reset. Without the variable it does nothing. Ctrl-C, SIGTERM or a closed terminal reach only the supervisor, which forwards SIGINT and waits for the child; it exits with the child's code, non-zero after an escalation. If the supervisor dies, the child gets SIGINT and its next heartbeat raises `SupervisorLost`. Linux only (prctl, process groups).

The supervisor's only lever is the kill. libfranka then drops the control connection and the robot stops at the firmware level, in `Reflex`. Clear it in Desk or with `rwrl --station robot.yaml recover` (it calls the arm's `recover()`); `rwrl doctor --fix` repairs only the host and software and never recovers the arm. A controlled out-of-process `stop()` exists only on the daemon route (`--watchdog-s`). The in-process first layer is `StepWatchdog`, enabled with `real_robot.watchdog_s` (see `docs/real-world-experiments.md`).

### 7. Add a teleop device

A `teleop:` section makes an operator input device a part of the robot. Hardware-free, with
the fake backend (a script advanced one step per `read()`):

```yaml
teleop:
  name: teleop                # the daemon serves teleop.<name>.read / .reset
  backend: fake               # fake | spacemouse | gello
  hold_s: 0.0                 # keep intervening this long after the last activity (default 0)
  options:                    # backend-private
    action_space: joint_abs   # fake: cartesian_delta (default) | joint_abs | joint_delta
    num_joints: 7             # joint spaces only
    has_gripper: true         # default true
    on_end: hold_last         # hold_last (default) | idle | loop, what happens after the script
    script:
      - {action: [0, 0, 0, 0, 0, 0, 0], active: false, hold: 2}
      - {action: [0.5, 0, 0, 0, 0, 0, 0], gripper: 0.0, active: true, hold: 2}
      - {events: {episode_end: true}}
```

A step is `{action, gripper, buttons, events, active, hold}`; `hold` is the number of reads the
step lasts and its events fire on the first of them. `fail_after: n` makes every poll after the
n-th sample fail, to test the error path.

```python
robot = Robot.from_yaml("robot.yaml")          # local; RemoteRobot works the same way
robot.connect()
sample = robot.teleop.read()                   # blocks for a new sample, consumes the events
print(sample.seq, sample.action_space, sample.action, sample.gripper, sample.intervening, sample.events["episode_end"])
latest = robot.teleop.latest()                 # no blocking, does not consume events
robot.teleop.reset()                           # drops latched events and the hold_s hysteresis
```

`read()` returns a `TeleopSample`; over the daemon (`teleop.<name>.read`) it travels as a dict
with the same keys:

| Field | Type | Meaning |
|---|---|---|
| `action` | `float32` ndarray | `cartesian_delta`: `(6,)` normalized twist in [-1, 1]; `joint_abs` / `joint_delta`: `(N,)` joint values in rad |
| `action_space` | `str` | `cartesian_delta`, `joint_abs` or `joint_delta` |
| `gripper` | `float` or `None` | normalized width target in [0, 1]; `None` = the device has no gripper channel |
| `buttons` | `dict[str, bool]` | current levels (SpaceMouse: `button_0`, `button_1`) |
| `events` | `dict[str, bool]` | latched edges `episode_end`, `success`, `failure`, `rerecord` (always all four keys), plus any backend-specific edge that fired (SpaceMouse: `button_0`, `button_1` presses); `read()` clears them |
| `intervening` | `bool` | a human is operating (backend's `active`, extended by `hold_s`) |
| `timestamp` | `float` | `time.monotonic()` on the machine that polled, not comparable across hosts |
| `seq` | `int` | increases with every polled sample, also across reconnects; a gap means dropped samples |

`read(timeout_s)` raises `DeviceFault` on a timeout and a recoverable `DeviceFault` once 5 polls
in a row failed (reconnect the robot); it raises `DeviceNotConnected` if the robot is
disconnected while it waits. There is a single consumer: two clients reading the same teleop
split the events between them.

`backend: spacemouse` (extra `spacemouse`; options `device_index` 0, `deadzone` 1e-3,
`invert_axes`, `poll_interval_s` 0.005) is a `cartesian_delta` device with a gripper: the puck
gives the twist `[-y, x, z, -roll, -pitch, -yaw]`, every button press is latched as the event
`button_0`/`button_1` (so a press between two reads is not lost), and button 0 toggles the sticky
gripper target (1.0 = open at connect). Neither button raises `episode_end`. real_world_rl's
`--teleop-device spacemouse` gives the buttons its own meaning (button 0 take-over, button 1
gripper) and ignores `gripper` and `intervening`. It needs read/write access to
`/dev/hidraw*`; `rwrl doctor` checks this as `os:hid` and prints the udev
rule installation command. See [Known limitations](#known-limitations-and-needs-hardware-verification).

`backend: gello` (extra `gello`, plus `gello-events` for `event_device`) is a GELLO leader arm
(passive Dynamixel servos, Protocol 2.0, through a U2D2 or OpenRB-150): a `joint_abs` device
with one joint per `joint_ids` entry and a gripper channel when `gripper_id` is set. Each poll
is one sync read; joint `i` is `wrap((raw_rad - joint_offsets_rad[i]) * joint_signs[i])` into
`[wrap_centers_rad[i] - pi, wrap_centers_rad[i] + pi)`, so no start pose is needed at connect
(set the centres to the midpoints of `joint_limits`; Panda joint 6 reaches past pi and needs a centre > 0).
The gripper is `clip((raw - gripper_closed_rad) / (gripper_open_rad - gripper_closed_rad), 0, 1)`,
1 = open, optionally binarized with `gripper_binarize_threshold`. `intervention: always`
(default) reports `intervening` on every poll; `toggle` follows a toggle key read with `evdev`
from `event_device` (a pedal or keyboard on the robot PC), whose other keys raise
`episode_end`/`success`/`failure`/`rerecord` (`event_keys`). Torque is turned off on connect and
on disconnect; a missing servo or a failed sync read fails the poll (no stale samples). The
full option list with defaults is in [`examples/stations/robot.franka.example.yaml`](../../examples/stations/robot.franka.example.yaml).
Calibrate once per unit (torque off, the leader is moved by hand); the [setup wizard](#setup-wizard)
runs these steps inline and writes the result into the station yaml. Standalone, the CLI prints
the block to paste:

```bash
rwrl gello-calibrate \
    --port /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_XXXX-if00-port0 \
    --joint-ids 1 2 3 4 5 6 7 --gripper-id 8 \
    --calibration-pose 0 0 0 -1.57 0 1.57 0 --joint-limits-yaml robot.yaml
```

It checks each joint's sign (move one joint in the robot's positive direction when asked),
solves the offsets on the quarter-turn grid from the calibration pose, captures the trigger's
open and closed angles, and takes the wrap centres from the yaml's `joint_limits`. The port
needs read/write access (`dialout` group); the doctor checks it as `os:serial` and `os:groups`,
and warns about brltty on CH340 adapters (`os:brltty`) and a slow FTDI `latency_timer`
(`os:latency_timer`). The baud rate
is never written to the servos: change it once in Dynamixel Wizard (1000000 gives about 130 Hz
for 8 servos, 57600 about 25 Hz).

## Architecture

```text
Connection (ABC)                 connect()/disconnect() lifecycle, owner borrowing, SDK, health()
└── Part                         observation_features, get_observation()
    ├── Camera   [registry]      background capture thread, read()/latest()/reopen()
    │   ├── realsense, fake
    │   └── (RemoteCamera is a Part, not a Camera)
    ├── Teleop   [registry]      background poll thread, read()/latest()/reset(), latched events
    │   ├── gello, spacemouse, fake
    │   └── (RemoteTeleop is a Part, not a Teleop)
    └── ControllablePart         action_features, send_action()
        ├── Arm      [registry]  read_state, servo_cartesian/joints, move_to_pose/joints
        │   │                    (template methods, see below), stop, recover, set_control_mode, preflight
        │   ├── franky, serl_http, pyagxarm, piper_sdk, fake
        │   └── RemoteArm
        ├── Gripper  [registry]  open/close/move(width in [0, 1]), width; binary | continuous
        │   ├── franky, serl_http, pyagxarm, piper_sdk, fake
        │   └── RemoteGripper
        └── PartGroup            named children; connect in order, disconnect in reverse
            └── Robot  [type registry]  config -> arm (+ gripper) (+ cameras group)
                ├── FrankaRobot "franka"   (FrankaConfig)
                ├── PiperRobot  "piper"    (PiperConfig)
                └── RemoteRobot "remote"   (RemoteRobotConfig; registered by importing robot_infra.server)

Robot PC:   Robot ── RobotService (method table, watchdog, DeviceClaim) ── GrpcTransport
GPU side:   RemoteRobot ── GrpcClient (msgpack over gRPC, token metadata, read-only retry)
```

Backends register by name (`Arm.register("franky")`). `Arm.backends()`, `Gripper.backends()`,
`Camera.backends()` and `Teleop.backends()` list them, and `Robot.types()` lists robot types. A gripper with
`shares_arm_session: true` borrows the arm's session as its `owner` instead of opening its
own.

| Backend | SDK (extra) | Joint control | Wrench (`tcp_force`/`tcp_torque`) | Notes |
|---|---|---|---|---|
| arm `franky` | `franky` (`franky`) | yes | yes | torque-level impedance trackers for servo modes, Ruckig `move_to_*` (`IDLE` only); `realtime` and `relative_dynamics_factor` apply here |
| arm `serl_http` | none (HTTP) | no | yes | legacy client of the ROS1 bridge at `bridge_url`; Cartesian servo only; state has no `joint_pos`/`joint_vel` |
| arm `pyagxarm` | `pyAgxArm` (`pyagxarm`) | yes | no | Piper default; `firmware_profile` probed or set; state `tcp_pose`, `joint_pos`, `joint_vel` (flange pose) |
| arm `piper_sdk` | `piper_sdk` (`piper-sdk`) | yes | no | vendor SDK, fixed at 1 Mbit/s; `stop()` holds by re-sending the measured joints |
| arm `fake` | none | yes | yes | deterministic first-order arm for tests and `--mock` |
| gripper `franky` | `franky` | - | - | Franka Hand; own session |
| gripper `serl_http` | none | - | - | binary only, through the bridge |
| gripper `pyagxarm` / `piper_sdk` | as the arm | - | - | requires `shares_arm_session: true` and the matching arm backend |
| camera `realsense` | `pyrealsense2` (`realsense`) | - | - | RGB (+ aligned depth in metres with `enable_depth`) |
| `fake` gripper / camera | none | - | - | tests and `--mock` |
| teleop `spacemouse` | `hidapi==0.14.0.post4`, import `hid` (`spacemouse`) | - | - | cartesian twist + sticky gripper; `Teleop.backends()` is `fake`, `gello`, `spacemouse` |
| teleop `gello` | `dynamixel-sdk` (`gello`; `evdev` via `gello-events`) | - | - | GELLO leader arm, `joint_abs` + optional gripper; torque off; evdev toggle and events |
| teleop `fake` | none | - | - | scripted samples, `push()` injection, `fail_after`; tests and `--mock` |

**`Arm` is a template-method class.** The public `servo_cartesian`, `servo_joints`,
`move_to_pose` and `move_to_joints` sanitize the input (shape, finiteness, unit quaternion),
apply the safety limits and only then call the backend hooks `_servo_cartesian`,
`_servo_joints`, `_move_to_pose` and `_move_to_joints`. Likewise the public `stop()` clears
`last_cartesian_target` and then calls the backend hook `_stop`. A backend implements the
hooks (plus `read_state`, `recover`, `_enter_control_mode`) and never overrides the public
methods, so no path to the motors skips the guard. A backend's `read_state()` must return its
result through `self._remember_state(state)`: the step cap reuses that cached state (at most
`MAX_DELTA_STATE_AGE_S` = 0.5 s old, else it re-reads) instead of querying the robot on every
servo call. `Arm.from_config(config)` builds the backend with `ArmSafety.from_config(config)`.
A third-party backend that skips `_remember_state` still works but costs one extra
`read_state()` per capped servo call.

### Safety rules

- **`disconnect()` never powers off.** Releasing an arm only closes the SDK session. The
  Piper backends never call `disable()`/`reset()`/`DisableArm`/`ResetPiper`, because a
  powered-down Piper falls. `stop()` brings motion to rest and leaves power and brakes as
  they are.
- **Servo commands are refused when the arm is not ready.** Every backend refuses servo
  commands outside the matching servo mode. franky, pyagxarm and piper_sdk also refuse them
  while the arm is faulted (call `recover()` first). The Piper backends also refuse on stale
  feedback: no new joint frame for 0.5 s on pyagxarm, 1.0 s on piper_sdk. On the hardware
  backends, `move_to_*` requires `IDLE`. Actions are checked for shape and finiteness, and
  `tcp_pose` quaternions are checked for unit norm.
- **Connecting enables the motors.** Connecting a Piper (directly, through the doctor, or
  through the daemon when a client connects) enables its motors, and they stay enabled
  after `disconnect()`. Clear the workspace before running the doctor.
- **Safety limits.** `safety_box`, `joint_limits` and `max_delta_per_step` in the robot yaml on
  the robot host are the source of truth. `Arm` enforces them in the public command methods, so
  they hold on every path: local `Robot`, the daemon (`arm.servo_*`, `arm.move_to_*`,
  `robot.send_action`, `robot.step`) and `send_action`. A `RemoteArm` has no guard of its own and
  forwards to the daemon's.
  - `servo_*`: the target is first shortened to `max_delta_per_step` from the measured state, then
    clipped into the base box / joint limits. Any accepted change counts one `safety_clips`.
    With `workspace` configured, Cartesian targets must also satisfy its oriented position box
    and optional SO(3) orientation radius, both before and after base-box clipping. A workspace
    rejection raises `SafetyViolation`, increments `safety_rejects`, and sends nothing.
  - `move_to_*`: a target outside the box / joint limits raises `SafetyViolation`, counts one
    `safety_rejects`, and nothing is sent. `move_to_*` is not step-capped.
  - `SafetyViolation` is not a device fault: it is raised before the backend is touched, so it
    cannot latch a fault, needs no `recover()`, keeps the control mode, and through the daemon it
    does not refresh the watchdog. A client sees the same class.
  - `arm.health()` reports `safety_clips` / `safety_rejects`; `robot.describe` exposes the limits
    as `arm.safety`. The checks are validated at build time: `home_joints` must lie inside
    `joint_limits`, a box must have min <= max.
  - `safety_box` and the Cartesian step cap limit position only. A workspace may additionally
    constrain orientation with `orientation_xyzw` (in its frame) and `max_angle_rad` (SO(3)
    geodesic radius, avoiding Euler branch cuts). Orientation has no device-side step cap.
    Joint commands are not checked against Cartesian workspace boundaries: these limits do not
    certify a joint trajectory, whole-arm clearance, or collision-free motion.
  - Target checks use `1e-9` numerical tolerance. Feedback checks use workspace keys
    `measured_margin_m` (default `0.003 m`) and `measured_margin_rad` (default `0.03 rad`),
    both finite and nonnegative. The environment and teleop tracker use these feedback margins;
    they never relax target admission. A `task_frame` without a workspace is a display/sampling
    reference and adds no spatial contract. With a workspace, both geometries are compared
    against the daemon before reset.
- **Watchdog.** The daemon idles a servoing arm whose client went quiet (see step 4).
- **`DeviceClaim`.** An advisory, node-local file lock in `~/.cache/robot_infra/claims/`,
  keyed `<type>:<robot_ip | can_interface | name>`. The daemon holds it while a client is
  connected, so a second daemon on the same robot gets `DeviceBusy`. It does not protect
  against other hosts or against code that does not take the claim. Set `claim: false`
  to opt out. The SpaceMouse reader also claims its HID endpoint by default and honors the
  same `RobotConfig.claim` setting when built as a robot teleop part. Its built-in device
  table equals PySpaceMouse 1.0.3; a site's local device table adds or overrides models.

### Error model

Device errors subclass `DeviceError` (a `RuntimeError`) and carry `message`, `hint` and
`device`. `str(error)` prints the hint on a second line. Errors survive the
`to_dict()`/`from_dict()` round trip, so a client re-raises the same type the daemon raised.

| `kind` | Class | Typical cause |
|---|---|---|
| `sdk_missing` | `SdkMissing` | vendor package not installed |
| `device_not_found` | `DeviceNotFound` | robot unreachable, CAN interface or camera serial missing |
| `device_not_connected` | `DeviceNotConnected` | call before `connect()` |
| `device_already_connected` | `DeviceAlreadyConnected` | defined; not raised by current backends (`connect()` is idempotent) |
| `device_busy` | `DeviceBusy` | another process holds the claim |
| `device_fault` | `DeviceFault` (`recoverable: bool`) | reflex, e-stop, no feedback, wrong mode, watchdog |
| `version_mismatch` | `VersionMismatch` | libfranka vs robot server version; the wrong distribution `hid` without trezor's `hid.device()` API |
| `safety_violation` | `SafetyViolation` | a Cartesian servo/move target outside `workspace` (position or orientation), or a `move_to_*` target outside `safety_box` / `joint_limits`; refused before sending, counted by `safety_rejects` |
| `transport` | `TransportError` (a `DeviceFault`) | daemon unreachable, timeout, token mismatch |

## Testing

The device-layer tests are `tests/devices/` of the repository test suite. From the repository
root, project environment active:

```bash
pip install -e '.[dev,grpc]'
python -m pytest tests/devices -q
```

The tests use the fake backends and SDK stubs. No hardware is needed. The gRPC tests are
skipped when `grpcio` is missing. The maintainers run them in a remote container:

```bash
docker exec <container> bash -lc 'cd <checkout> && python -m pytest tests/devices -q'
```

## Known limitations and needs hardware verification

Nothing below has been exercised on real hardware; the fakes cover the logic only.

- **SpaceMouse USB vendor ids.** `discover()` filters on `0x256f` (3Dconnexion) and `0x046d`
  (Logitech, older units). Both are unverified; without `hidapi` it lists every `/dev/hidraw*`.
- **udev and Docker.** The udev rule printed by `os:hid` and the `docker run --device=/dev/hidrawN`
  hint are untested on a robot PC. A container needs the node passed in before the daemon starts.
- **SpaceMouse wheel and timing.** The in-tree reader uses `set_nonblocking(1)` and
  polls one report every `poll_interval_s` (default 0.005 s, 200 Hz). Queue latency,
  split-report timing, device directions and disconnect behavior are not yet verified
  on hardware. The `hidapi==0.14.0.post4` wheel's ELF/system-library dependencies
  remain unverified on a clean Ubuntu 22.04 / CPython 3.10 / x86_64 machine.
- **Piper master arm** as a teleop (leader-follower) is not implemented; the leader-arm backend is
  `gello` (Dynamixel). `joint_delta` is exercised by the fake only.
- **GELLO on hardware.** The `gello` backend and its calibration CLI run against a fake
  `dynamixel_sdk` and a fake `evdev` only. Unverified: the U2D2/OpenRB-150 by-id names that
  `discover()` globs, sync-read timing per baud rate, the FTDI latency timer (16 ms by default;
  1 ms is the usual Dynamixel tip, not set by the backend), and that the present position stays
  continuous when a joint crosses a turn boundary.
- **Orientation step size.** A workspace can bound orientation relative to a reference;
  `safety_box` and `max_delta_per_step` still limit only position (`xyz_m`) and joints
  (`joint_rad`). There is no device orientation step cap. The gym env bounds its own rotation
  increment with `rot_scale_rad` and projects policy targets into the configured workspace.
- **Step cap baseline.** The cap is measured from the last read state (at most 0.5 s old), not
  from the previous target, so the worst distance between a target and the true position is about
  twice the cap, without accumulating.
- **Bypassing the guard.** The legacy ROS1 bridge (`serl_http`) is an HTTP server: a client
  that talks to `bridge_url` directly does not pass through `Arm`, so none of the limits apply.
  Third-party backends must implement the `_servo_*`/`_move_*` hooks, not override the public methods.
- **A configured teleop is mandatory.** `connect()` fails and rolls the whole robot back when the
  teleop device is absent, including for deployment. Use a yaml without `teleop:` to deploy.
- **Version skew.** A daemon with these features breaks a pre-upgrade client at `arm.health()`
  (new fields), and an old client cannot read JPEG frames. Upgrade the client first.
- **`--jpeg-quality` is not recorded** in a training run's config; deploy cannot check that it
  matches the value used for training.
- **Teleop read clamp and clocks.** `teleop.<name>.read` over the daemon waits at most 2 s;
  `timestamp` is the robot PC's monotonic clock. `RemoteTeleop.get_observation()` returns the
  client's last sample, while the daemon's `robot.get_observation()` contains a `teleop` entry
  taken from the daemon's latest sample (events not consumed).

## Legacy

These modules predate the device layer and are kept for existing users. Planned work is in
[`docs/devices/robot-infra-roadmap.md`](robot-infra-roadmap.md).

- **`controller/`**: pure-torch Cartesian impedance and pose controllers (`core/`, `adapters/`),
  wrapped for ManiSkill in `controller/simulator/maniskill/` (extra `sim`). They serve
  GPU-batched simulation, not real hardware. See [`robot_infra/controller/README.md`](../../robot_infra/controller/README.md).
- **`controller/real/franka_bridge.py`** and **`gripper_server.py`**: a Flask/HTTP forwarder to
  the ROS1 `serl_franka_controllers` impedance controller and the Franka Hand. The `serl_http`
  backend is their client. Run the bridge on the ROS machine with
  `python -m robot_infra.controller.real.franka_bridge --robot_ip <ip> --port 5000` (extra
  `legacy-bridge`; ROS1 is installed separately, see the module docstring).
- **`teleop/`**: the pre-device-layer teleop (`teleop/spacemouse/` on the old `pyspacemouse`
  `read_all()` API, extra `legacy-bridge`; `teleop/utils/` twist teleop for Pico over ZeroMQ;
  its inspector is `examples/run_twist_input_inspector.py`). The `Teleop` part ([step 7](#7-add-a-teleop-device)) supersedes the
  SpaceMouse code. The Pico path is not migrated yet and `real-world-rl`'s
  `--teleop-device pico` still imports these modules, so they stay until it is.
