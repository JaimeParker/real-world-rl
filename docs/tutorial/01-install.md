# 1. From power-on to a doctor handshake

This page commissions a NUC and one or two Franka FR3 arms. Work at the cell with
an accessible emergency stop and a clear workspace. A successful doctor handshake
checks connectivity and feedback; it does not validate a motion path or a task.

**Command environment.** Run all shell blocks on the NUC, from this repository's
root (`real-world-rl/`). Steps 1–3 use Ubuntu system tools and need no Python venv.
Step 4 installs the project environment; activate it before every Python command
in steps 5–6 and on the following station page. Replace the example Ethernet interface after
inspecting your own machine. Browser checks also take place on the NUC.

## 1. Power on the cell

Power on the control box and wait for Desk to become accessible. With the cell
clear, release the emergency stop, use Desk's **Unlock joints**, and switch
**End Effector Power On** on if a Hand is fitted. The next two steps establish
the network and Desk session needed to finish these checks.

*You should see:* Desk reports the arm connected, joints unlocked and end-effector
power on for an installed Hand. Check the physical arm as well as the page.

*If not:* check control-box power, the emergency-stop chain and Desk's reported
fault. A robot that is powered off will not answer ping. Do not substitute a
Desk motion task or Hand reinitialization for a connectivity check: those actions
can move hardware. A station without a physical Hand must use `gripper: null`
and `gripper_in_action: false` later.

## 2. Connect the Control network

Connect the NUC to the control box's **Control** port, not the **Arm** port at the
arm base. A switch connecting the NUC and control boxes is also valid; a direct
point-to-point cable is not required. Each arm needs a unique IP address.

The factory FCI address is `172.16.0.2`; the two-arm example uses left
`172.16.0.2` and right `172.16.0.4`. Verify the physical identity of each arm.
The NUC example is `172.16.0.1/24`. Keep the Internet connection's default route;
the isolated robot link does not need an Internet gateway.

On the NUC, repository root, no venv needed, inspect before adding an address:

<!-- tutorial-test: skip -->
```bash
ip -br -4 addr
ip route get 172.16.0.2
ROBOT_IF=enp86s0  # replace with the Ethernet interface wired to Control
```

If the address is absent, this adds it **temporarily** (lost after reboot):

<!-- tutorial-test: skip -->
```bash
sudo ip addr add 172.16.0.1/24 dev "$ROBOT_IF"
sudo ip link set "$ROBOT_IF" up
ping -I "$ROBOT_IF" -c 3 -W 1 172.16.0.2
ip route get 172.16.0.2
```

For persistence, the following uses **NetworkManager**, on a NUC where it manages
the Ethernet interface. Reuse that interface's existing connection profile;
choose a new profile only if it has none. Run on the NUC, repository root, no venv:

<!-- tutorial-test: skip -->
```bash
nmcli device status
nmcli connection show
ROBOT_CONNECTION="$(nmcli -g GENERAL.CONNECTION device show "$ROBOT_IF")"
if [ -z "$ROBOT_CONNECTION" ] || [ "$ROBOT_CONNECTION" = "--" ]; then
  ROBOT_CONNECTION=franka-control
  sudo nmcli connection add type ethernet ifname "$ROBOT_IF" con-name "$ROBOT_CONNECTION"
fi
sudo nmcli connection modify "$ROBOT_CONNECTION" \
  ipv4.method manual ipv4.addresses 172.16.0.1/24 ipv4.gateway "" \
  ipv4.never-default yes connection.autoconnect yes
sudo nmcli connection up "$ROBOT_CONNECTION"
ping -I "$ROBOT_IF" -c 3 -W 1 172.16.0.2
ip route get 172.16.0.2
```

*You should see:* ping replies and a route through the intended Ethernet interface
with source `172.16.0.1`. Recheck after reboot to verify persistence. For a second
arm repeat ping and route checks with `172.16.0.4`.

*If not:* confirm power, Control-port cabling, link state, unique addresses and
the actual configured subnet. Check VPN routes before changing the robot network.
If `nmcli` reports an unmanaged device, configure persistence through the network
manager that owns it; do not add a competing NetworkManager profile. Desk/FCI on
the local network does not require Franka World to be connected.

## 3. Verify six separate Desk conditions

On the NUC, open **https://172.16.0.2/desk/** in a browser. Expect a self-signed
certificate warning; verify that this is your control box before proceeding.
Each arm has its own Desk page (`https://172.16.0.4/desk/` for the second arm).
Desk runs on the control box; there is no Desk package to install on the NUC.

Check each condition separately:

1. **Page reachable:** the Desk page loads for the intended physical arm.
2. **Green status light:** the arm/control status is ready; inspect any fault.
3. **Logged in:** the current browser session is authenticated.
4. **Control belongs to this machine:** if another machine has opened Desk,
   use **Request control** here. An old page can display stale state; refresh
   before deciding which machine has control.
5. **FCI installed:** the System page's **Installed Features** includes Franka
   Control Interface. An entry such as **FCI 1.0.0** is a feature version,
   not the firmware, robot server or libfranka version.
6. **FCI activated:** activate the FCI session from Desk's menu. Its location
   varies with system version. If the entry is missing, first check control
   ownership, the installed FCI feature, robot mode and any running task.
   Running a Desk task is not a substitute for activating FCI.

Record the Dashboard's **Control** and **System image** versions (for example
`5.9.2`) for each arm. Desk shows the *system* version; the franky wheel is chosen
by the *robot server* version, which the
[compatibility matrix](https://frankarobotics.github.io/docs/compatibility.html)
maps from the system version and which the doctor reads from libfranka's handshake
error. Do not infer it from the FCI feature version or from an unavailable Software
Updates check.

*You should see:* all six conditions hold on a freshly refreshed page. Step 5's
actual FCI handshake confirms the session; a green page alone does not.

*If not:* resolve the failing condition before reinstalling Python packages.
For Hand initialization trouble, refresh the correct arm's Desk page and verify
power and physical presence first; [multi-arm checks](02-station.md#multiple-arms)
explain the common wrong-page case.

## 4. Install the project environment

The **project environment** is the Python environment (venv or conda) where this
step installs the three local packages: rl-garden, robot_infra and real-world-rl.
`scripts/install.sh` creates `.venv` by default; the manual route can use any
activated venv or conda environment.

Start with a checkout containing the initialized `rl-garden/` submodule. On a
fresh NUC, from a parent directory and with no environment active:

<!-- tutorial-test: skip -->
```bash
git clone --recurse-submodules https://github.com/JaimeParker/real-world-rl.git
cd real-world-rl
```

The installer `scripts/install.sh` and the `franka-gello` and `test` extras
(defined in `pyproject.toml`) are part of this checkout. The installer requires
Python 3.10 with venv support and package-download access.

On the NUC, repository root, no environment active:

<!-- tutorial-test: skip -->
```bash
bash scripts/install.sh --extras franka-gello
source .venv/bin/activate
```

The installer creates or reuses `.venv` (default interpreter `python3.10`), clears
inherited `PYTHONPATH`, installs rl-garden, robot_infra and this package in editable
mode, then runs doctor. With no station supplied its final check is **mock**, run
against a throwaway station whose arm and gripper are fake; that success does not
establish a hardware handshake. RealSense stations also
need the `robot_infra[realsense]` extra in this same project environment.

<details>
<summary>Manual equivalent of the default installation (what the installer does)</summary>

On the NUC, repository root, run the pip commands in any activated venv or conda
environment. Create and activate an environment, for example:

<!-- tutorial-test: skip -->
```bash
unset PYTHONPATH
python3.10 -m venv .venv
source .venv/bin/activate
python -m pip install -U pip
python -m pip install -e ./rl-garden -e ./robot_infra -e '.[franka-gello,test]'
python -c 'import rl_garden, robot_infra, real_world_rl'
rwrl doctor --mock
```

</details>

For RealSense, on the NUC, repository root, project environment active:

<!-- tutorial-test: skip -->
```bash
python -m pip install -e 'robot_infra[realsense]'
```

*You should see:* successful imports and a mock doctor summary with zero failures.

*If not:* check the failed installer stage and its retry hint, Python/venv
availability and submodule initialization. An `Unknown env backend 'real_robot'`
error means this package is missing from the interpreter actually running it.
Installing `franky-control` supplies the `franky` import and its libfranka build;
it does not require ROS, and the package name is not `franka`.

### Optional SpaceMouse dependencies

On the NUC (or the machine with the device), repository root, with the project
environment from step 4 active:

<!-- tutorial-test: skip -->
```bash
pip install -e '.[spacemouse]'
```

This adds `hidapi==0.14.0.post4` (trezor/cython-hidapi, import `hid`) for the
in-tree device reader, without `pyspacemouse`/`easyhid` on this path. Do not install
the different distribution named `hid`. From an empty environment instead, on the
NUC, repository root, no environment active:

<!-- tutorial-test: skip -->
```bash
bash scripts/install.sh --extras franka-gello,spacemouse
source .venv/bin/activate
```

Complete the one-time [SpaceMouse udev steps](../teleop.md#spacemouse-installation-and-linux-permissions)
on the NUC before opening the device. The wheel target is Ubuntu 22.04 / CPython
3.10 / x86_64; clean-machine wheel/system-library checks remain pending, and this
path is **not yet verified on hardware**.

## 5. Configure once, then run doctor

For a fresh station, run the [setup wizard](02-station.md) now to create
`local/station.yaml`. It probes hardware, asks for arm/Hand/camera/GELLO details,
offers calibration, and runs doctor. If peripherals are not connected yet, select
`none` for those parts and add them later. Do not invent calibration values just
to pass a check. For an existing station, reuse its file.

With no recorder, actor or daemon holding the devices, on the NUC, repository
root, project environment active:

```bash
rwrl doctor --station local/station.yaml
```

| Layer | What it verifies | If it fails |
|---|---|---|
| 1 SDK | backend imports | install the missing extra in the project environment |
| 2 OS | network, device presence and permissions | revisit steps 1–3 or apply step 6's device fix |
| 3 Handshake | connect to each configured device | check FCI activation, ownership and SDK compatibility |
| 4 Feedback | actual state/frame reads | inspect the named device, cable and camera bandwidth |
| 5 Mode | robot mode permits use | inspect Desk faults and mode before attempting motion |

*You should see:* per-check results and `summary: N ok, 0 failed, S skipped`.
For a real handshake use the real configuration without `--mock`; investigate
skipped checks rather than treating them as hardware verification. Doctor connects
and disconnects; it does not execute a home trajectory. Configured device
initialization can still have side effects, such as GELLO torque-off.

*If not:* follow the named check's hint. For an unreachable robot, return to
steps 1–3: power, Control cable, IP/route, refreshed Desk control and active FCI.
For UDP timeouts inspect firewall rules on this link. An occupied device claim
means another process may still own the robot; stop that process first.

For a libfranka/server mismatch, use the guided repair `--install-sdk`. On the
NUC, repository root, project environment active:

<!-- tutorial-test: skip -->
```bash
rwrl doctor --station local/station.yaml --install-sdk
```

The guided repair reads the robot server version from libfranka's handshake
error, asks you to cross-check it (system version in Desk → Settings → System,
mapped through the compatibility matrix), and installs the matching wheel only
after confirmation. Use the **robot server version**, not `FCI 1.0.0`, to choose
a wheel. Rerun the normal doctor command after installation. Retain the Desk
version record; do not repeatedly install the same default wheel expecting a
different handshake result.

## 6. Fix serial and keyboard checks

Doctor reports missing or inaccessible GELLO serial devices, and also diagnoses
CH340/brltty conflicts, input permissions and USB latency. Start with the
guided repair, which previews each change and asks for confirmation before it
touches the OS. On the NUC, repository root, project environment active:

<!-- tutorial-test: skip -->
```bash
rwrl doctor --station local/station.yaml --fix
```

Log out and back in if doctor changed group membership, then rerun the plain
`rwrl doctor` command. The manual commands below are the fallback when
`--fix` does not offer a repair for your case or you prefer to apply it by hand.
All of them run on the NUC, repository root; system commands do not need
a venv, and the final doctor command uses the project environment.

For a `dialout` permission failure:

<!-- tutorial-test: skip -->
```bash
ls -l /dev/serial/by-id/
sudo usermod -aG dialout "$(id -un)"
```

Log out and back in, reactivate the project environment, and check permissions again.
Configure the stable `/dev/serial/by-id/` path, not an assumed `ttyUSB0` number.

For an evdev keyboard or pedal permission failure:

<!-- tutorial-test: skip -->
```bash
sudo usermod -aG input "$(id -un)"
```

Log out and back in. `input` grants access to input events; it is only needed for
the evdev device route. Terminal stdin keys need no input group; a `pynput`
listener uses the logged-in desktop session and does not work merely because
an SSH shell has joined `input`.

For a CH340 adapter that appears and then disappears because brltty claims it,
first inspect the log. If it confirms that conflict and the service is not needed
for a braille device, temporarily release the adapter, then unplug/replug it:

<!-- tutorial-test: skip -->
```bash
journalctl -b -u brltty-udev.service --no-pager
sudo systemctl mask --runtime --now brltty-udev.service
ls -l /dev/serial/by-id/
```

The runtime mask ends at reboot. Resolve any recurring conflict in the station's
OS configuration rather than relying on a temporary ACL or mask surviving reboot.

For a high USB serial `latency_timer`, select the actual adapter path from the
listing above. This example is the CH340 path; change it if your adapter differs:

<!-- tutorial-test: skip -->
```bash
GELLO_PORT=/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0
GELLO_TTY="$(basename "$(readlink -f "$GELLO_PORT")")"
LATENCY_FILE="/sys/bus/usb-serial/devices/$GELLO_TTY/latency_timer"
if [ -f "$LATENCY_FILE" ]; then
  cat "$LATENCY_FILE"
  echo 1 | sudo tee "$LATENCY_FILE"
  cat "$LATENCY_FILE"
fi
```

The value is in milliseconds and this write is temporary. Only drivers exposing
that attribute support this fix; an absent attribute is not a permission failure
and does not justify creating a file. Recheck after replug/reboot.

*You should see:* the serial path stays present and is readable/writable, the
selected input route works, and the latency value reads `1` where supported.
With the project environment active, rerun:

```bash
rwrl doctor --station local/station.yaml
```

*If not:* verify the selected USB device and driver, refreshed group membership,
and that no other program owns the serial port. A missing camera or frames check
is separate: verify its configured serial, USB 3 connection and vendor udev rules.
`realtime: ignore` permits a generic kernel but does not guarantee control timing;
`realtime: enforce` requires a suitable real-time setup.

## A robot someone else has already commissioned

Use the existing `local/station.yaml` and the project environment on the NUC.
Do just these three things before starting a session:

1. Open the intended arm's Desk on this NUC, **Request control** if necessary,
   and refresh to verify control ownership.
2. Confirm that this arm's **FCI session is activated**.
3. With other device-owning processes stopped, run on the NUC, repository root,
   project environment active:

```bash
rwrl doctor --station local/station.yaml
```

*You should see:* a real handshake and zero failed checks. *If not:* use the
failing layer above; do not overwrite a commissioned station or recalibrate it
just because another browser retained control.

Next: [02 Station](02-station.md).
