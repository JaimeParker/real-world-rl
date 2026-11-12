# 2. Set up the station

**Goal:** one checked `local/station.yaml` that names this arm, its Hand, cameras and GELLO,
with home and limits that fit the cell. Every later `rwrl` command reads this file.

All commands run on the NUC, from the repository root, with the project environment active
(see [01](01-install.md)), unless a block says otherwise.

## Before you start

The wizard probes the hardware, so it should be powered, cabled and reachable. Check each line;
the details are folded below.

1. **Arm powered and unlocked.** The control box is on, the emergency stop is released, Desk's
   **Unlock joints** is done and **End Effector Power On** is on if a Hand is fitted. *Check:*
   the Desk page shows the arm connected with joints unlocked.
2. **Control-port network.** The NUC is wired to the control box's **Control** port (not the
   **Arm** port at the arm base); a switch is fine. The NUC has `172.16.0.1/24` on that
   interface, the factory FCI address is `172.16.0.2`. *Check (no environment needed):*

   <!-- tutorial-test: skip -->
   ```bash
   ip route get 172.16.0.2
   ping -c 3 -W 1 172.16.0.2
   ```

   The route goes out through the Ethernet interface wired to Control, with source
   `172.16.0.1`, and the ping replies.
3. **Desk control and FCI.** In a browser on the NUC, `https://172.16.0.2/desk/` loads, you are
   logged in, this machine holds control (**Request control**, then refresh) and the FCI session
   is **activated**. *Check:* the six conditions below on a freshly refreshed page.
4. **No other process holds the devices.** Stop any recorder, actor or daemon first.

<details>
<summary>Give the NUC an address on the Control network</summary>

Inspect first, then pick the interface wired to Control. On the NUC, repository root, no
environment needed:

<!-- tutorial-test: skip -->
```bash
ip -br -4 addr
ROBOT_IF=enp86s0  # replace with the Ethernet interface wired to Control
```

This adds the address **temporarily** (lost after reboot):

<!-- tutorial-test: skip -->
```bash
sudo ip addr add 172.16.0.1/24 dev "$ROBOT_IF"
sudo ip link set "$ROBOT_IF" up
ping -I "$ROBOT_IF" -c 3 -W 1 172.16.0.2
```

For a persistent address on a NUC whose Ethernet is managed by NetworkManager, reuse the
interface's existing connection profile, or create one if it has none:

<!-- tutorial-test: skip -->
```bash
ROBOT_CONNECTION="$(nmcli -g GENERAL.CONNECTION device show "$ROBOT_IF")"
if [ -z "$ROBOT_CONNECTION" ] || [ "$ROBOT_CONNECTION" = "--" ]; then
  ROBOT_CONNECTION=franka-control
  sudo nmcli connection add type ethernet ifname "$ROBOT_IF" con-name "$ROBOT_CONNECTION"
fi
sudo nmcli connection modify "$ROBOT_CONNECTION" \
  ipv4.method manual ipv4.addresses 172.16.0.1/24 ipv4.gateway "" \
  ipv4.never-default yes connection.autoconnect yes
sudo nmcli connection up "$ROBOT_CONNECTION"
ip route get 172.16.0.2
```

Keep the Internet connection's default route; the robot link needs no gateway. Each arm needs a
unique address (the two-arm example uses `172.16.0.2` and `172.16.0.4`). If `nmcli` reports
the device as unmanaged, configure it with the network manager that owns it. Check VPN routes
before changing the robot network. Recheck the route after a reboot.

</details>

<details>
<summary>The six Desk conditions, and the versions to record</summary>

Open `https://172.16.0.2/desk/` on the NUC (expect a self-signed certificate warning; check
that it is your control box). Desk runs on the control box; nothing is installed on the NUC.

1. **Page reachable** for the intended physical arm.
2. **Green status light**; inspect any fault.
3. **Logged in.**
4. **Control belongs to this machine:** use **Request control** if another machine has it, and
   refresh, since an old page can show stale state.
5. **FCI installed:** System → **Installed Features** lists Franka Control Interface. An entry
   such as **FCI 1.0.0** is a feature version, not the firmware or libfranka version.
6. **FCI activated** from Desk's menu (its place varies with the system version). Running a
   Desk task is not a substitute.

Record the Dashboard's **Control** and **System image** versions (for example `5.9.2`). The
franky wheel is chosen by the *robot server* version, which the doctor reads from libfranka's
handshake error on page 03; the
[compatibility matrix](https://frankarobotics.github.io/docs/compatibility.html) maps the
system version to it.

</details>

If the arm is not reachable yet, the wizard can still write the file: pass `--allow-unprobed`
to accept an IP, camera serial or serial port that the probe did not find, and check it on
[03](03-franka-bringup.md).

## 1. See what is connected

```bash
rwrl discover
rwrl --json discover
```

*You should see:* sections `cameras:` (RealSense serials), `teleops:` (for example the GELLO's
`/dev/serial/by-id/...` path), `can interfaces:`, `hosts:` and `hid devices:`, each `none` when
nothing is found. `hosts:` lists the robot IPs of an existing station file only; `discover`
never connects to them. With `--json` the same report is one object with `schema_version: 1`
and the lists `cameras`, `teleops`, `can_interfaces`, `hosts`, `hid_devices` and `unknown_hid`.

## 2. Run the wizard

```bash
rwrl setup
```

It probes the hardware, asks a few questions, calibrates the GELLO if you say yes, writes one
annotated `station.yaml`, runs the doctor on it and prints the next commands. Every question has
a flag (`rwrl setup --help`); any flag you give is not asked. For example, one arm with a GELLO,
everything else asked:

```bash
rwrl setup --arms 1 --arm-names arm --robot-ip 172.16.0.2 --teleop gello --scope local
```

| Flag | Meaning and default when omitted |
|---|---|
| `--arms 1` or `--arms 2` | number of arms; a new station suggests 1 |
| `--arm-names arm` or `--arm-names left right` | names in the file and in doctor ids (`arm.handshake`) |
| `--robot-ip 172.16.0.2` | FCI address per arm, also probed |
| `--gripper franky` | `franky` (Franka Hand), `none` (no Hand), `fake` or `serl_http` |
| `--cameras none` or `--cameras SERIAL=wrist` | per arm: serials from `rwrl discover` and their observation names, comma separated |
| `--camera-resolution 1280x720`, `--camera-fps 15`, `--camera-depth yes` | per arm, for that arm's cameras; unset, new cameras get 640x480, 30 fps, no depth and kept cameras keep their values |
| `--teleop gello` | `gello`, `spacemouse`, `fake` or `none` |
| `--gello-port PATH` | the leader's `/dev/serial/by-id/...` path (or the list number the wizard prints) |
| `--no-calibrate` | defer GELLO calibration; interactive default is to ask |
| `--scope local` / `--scope user` / `--output PATH` | write `./local/station.yaml`, `~/.config/rwrl/station.yaml` or a path |
| `--allow-unprobed` | accept a value the probe did not find |

Answer notes:

- **Robot IP, cameras, GELLO port** take a list number or the value itself.
- **Camera names** become observation keys: `wrist` is `rgb_wrist` in `obs.rgb`. This tutorial
  uses one camera named `wrist`.
- **Intervention:** keep `always`; the actor's key events replace it. `toggle` is for an evdev
  pedal on the NUC ([gello.md](../gello.md#6-hil-training)).
- **GELLO gripper:** a new GELLO gripper channel gets `gripper_binarize_threshold: 0.5`, so the
  stored gripper action is exactly `+1` (open) or `-1` (closed).
- An existing file is first copied to `station.yaml.bak.<UTC timestamp>`.

With every answer as a flag the wizard asks nothing. On the NUC, repository root, project
environment active, with your own GELLO path:

```bash
GELLO_PORT=/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0
rwrl setup --non-interactive --arms 1 --arm-names arm --robot-ip 172.16.0.2 \
    --arm-backend franky --gripper franky --home-joints 0,-0.785,0,-2.356,0,1.571,0.785 \
    --cameras none --teleop gello --gello-port $GELLO_PORT --gello-baudrate 57600 \
    --gello-joint-ids 1,2,3,4,5,6,7 --gello-gripper-id 8 --intervention always \
    --event-device none --daemon-port 50051 --scope local
```

`--non-interactive` defers calibration and fails on any question that has neither a flag nor a
`--station` value.

**GELLO calibration.** Answering `yes` to the calibration question runs the same guided
procedure as `rwrl teleop calibrate --backend gello`: with torque off you move each joint in
the robot's positive direction, hold the calibration pose, then open and close the trigger.
Page [05](05-teleop-gello.md) and [gello.md](../gello.md#2-calibration) describe it. Without
calibration the file gets zero offsets, `+1` signs and a `# TODO calibrate` comment; do not move
the robot with those values. Calibrate later with either command:

```bash
rwrl setup --station local/station.yaml --only teleop --calibrate
rwrl --station local/station.yaml teleop calibrate --backend gello --use
```

**Rehearse without hardware.** `--mock` uses canned probe results and a simulated GELLO; write
somewhere other than your real station:

```bash
rwrl setup --mock --output local/station.mock.yaml
```

*You should see:* the annotated YAML path, the doctor's device rows ending in `Result:` and the
next commands. *If not:* see [If it fails](#if-it-fails).

## 3. Where the file lands

`--scope local` writes `./local/station.yaml`, relative to the directory you run it in
(`local/` is git-ignored). Later commands find it without `--station`, taking the first of
`--station PATH`, `$RWRL_STATION`, `./local/station.yaml` and `~/.config/rwrl/station.yaml`
([station lookup](../cli.md#station-lookup)). Run later commands from the same directory, or
set `RWRL_STATION`.

## 4. Check limits and home

The wizard writes the Panda joint limits, a servo step cap
(`max_delta_per_step: {xyz_m: 0.05, joint_rad: 0.04}`) and the `safety_box` you accepted. The
box is in the robot base frame (`tcp_pose = O_T_EE`, metres) and must contain the home pose, the
object and the bin; `home_joints` must lie inside `joint_limits`. The device layer clips servo
targets to these limits and refuses out-of-range `move_to_*` targets
([safety rules](../devices.md#safety-rules)); a training env can only tighten them.

Review them without opening the file (this reads the station file only and connects nothing):

```bash
rwrl --station local/station.yaml workspace show
```

*You should see:* JSON with `joint_limits`, `max_delta_per_step`, `home_joints`,
`base_safety_box` and `home_within_limits: true`. `false` comes with `home_violations`, one entry
per joint outside its limits; `null` means the home or the limits are missing. Edit the file
and rerun; rerunning the wizard with `--station` keeps your edits.

## 5. What the station file contains

| Key | Meaning |
|---|---|
| `type`, `name` | robot type (`franka`) and the name used in doctor ids |
| `arm_backend`, `robot_ip`, `realtime` | `franky`, the FCI address, `ignore` or `enforce` (PREEMPT_RT) |
| `gripper` | the Hand (`backend: franky`) or `null` |
| `cameras` | name, `serial`, `width`, `height`, `fps`, `enable_depth` per camera |
| `teleop` | backend and options, for GELLO the port, servo ids and calibration |
| `home_joints`, `joint_limits`, `max_delta_per_step`, `safety_box` | the home pose and the limits above |
| `control` | optional impedance profile ([control profiles](../control-profiles.md)) |
| `daemon` | only for the remote-actor route ([remote actor](../remote-actor.md)) |
| `task_frame`, `workspace` | optional task frame and workspace ([04](04-first-motion.md#5-optional-task-frame-and-workspace)) |

Every field is described in the [device-layer guide](../devices.md#station-file-reference).

## Multiple arms

Each arm has its own IP, its own Desk page and its own activated FCI session; in this example
**left is `172.16.0.2`, right is `172.16.0.4`**. Verify each against the physical arm before
assigning cameras, GELLO ports or a home pose, and activate FCI on both. An interactive two-arm
run with no cameras and calibration deferred:

```bash
rwrl setup --arms 2 --arm-names left right \
    --robot-ip 172.16.0.2 172.16.0.4 --cameras none none \
    --teleop gello --no-calibrate --scope local
```

Select a distinct serial port for each GELLO and calibrate both before motion. Select no gripper
for an arm without a Hand; a software width reading does not prove a Hand exists. Never copy one
arm's home joints to the other. *You should see:* `robots.left` and `robots.right` with the
intended IPs. Page [03](03-franka-bringup.md) checks one arm at a time; further two-arm settings
are in [bimanual.md](../bimanual.md).

## Redo one section

```bash
rwrl setup --station local/station.yaml
rwrl setup --station local/station.yaml --only cameras
```

The first reruns every question with the old values as defaults; `--only cameras`, `teleop`
or `daemon` redoes one section and keeps the rest.

## If it fails

- **A device the wizard should list is missing:** run `rwrl discover`, check the cable and power,
  then rerun; or pass `--allow-unprobed` and the value.
- **The wizard's doctor shows red rows:** look at which row is red, then go to the matching
  section of [rwrl doctor: red lights](../doctor.md#red-lights-what-to-do). An unreachable arm is
  [Network and FCI](../doctor.md#network-and-fci); a GELLO port is
  [Serial, input groups and USB latency](../doctor.md#serial-input-groups-and-usb-latency).
- **`home_within_limits: false`:** fix `home_joints` or `joint_limits` in the file.

Next: [03 Bring up the Franka](03-franka-bringup.md).
