"""Read-only host discovery and targeted, capability-dependent repair proposals."""

from __future__ import annotations

import ast
import getpass
import importlib.metadata as metadata
import importlib.util
import os
import platform
import re
import shutil
import sys
from pathlib import Path

try:
    import grp
    import pwd
    import resource
except (
    ImportError
):  # The CLI and Python checks must remain importable on non-POSIX hosts.
    grp = pwd = resource = None

from real_world_rl.common.device_ownership import process_owners

from .doctor_platform import Capabilities
from .doctor_repair import (
    Action,
    Finding,
    Report,
    file_action,
    project_environment,
    redact,
)

TOOLS = {
    "lsusb": "usbutils",
    "setfacl": "acl",
    "udevadm": "udev",
    "ip": "iproute2",
    "ping": "iputils-ping",
    "fuser": "psmisc",
}
SDK_MODULES = {
    "franky": "franky",
    "spacemouse": "hid",
    "gello": "dynamixel_sdk",
    "gello-events": "evdev",
    "realsense": "pyrealsense2",
    "grpc": "grpc",
    "jpeg": "cv2",
}
FALLBACK_REQUIREMENTS = {
    "franky": ["franky-control"],
    "spacemouse": ["hidapi==0.14.0.post4"],
    "gello": ["dynamixel-sdk>=3.7.31,<3.9"],
    "gello-events": ["evdev"],
    "realsense": ["pyrealsense2"],
    "grpc": ["grpcio>=1.60"],
    "jpeg": ["pillow", "opencv-python-headless"],
}


def read(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8").strip()
    except (OSError, UnicodeError):
        return ""


def spacemouse_ids(root: Path | None) -> set[tuple[int, int]]:
    """Read the canonical table without importing numpy, drivers, or an absent SDK."""
    candidates = (
        [root / "robot_infra/robot_infra/devices/backends/teleops/_spacemouse_hid.py"]
        if root
        else []
    )
    spec = importlib.util.find_spec("robot_infra")
    if spec and spec.submodule_search_locations:
        candidates.extend(
            Path(folder) / "devices/backends/teleops/_spacemouse_hid.py"
            for folder in spec.submodule_search_locations
        )
    for candidate in candidates:
        if candidate.is_file():
            tree = ast.parse(candidate.read_text(encoding="utf-8"))
            return {
                tuple(ast.literal_eval(node.value))
                for node in ast.walk(tree)
                if isinstance(node, ast.keyword) and node.arg == "hid_id"
            }
    return set()




class Host:
    def __init__(
        self,
        root: Path | None,
        report: Report,
        *,
        sysfs: Path = Path("/sys"),
        dev: Path = Path("/dev"),
        etc: Path = Path("/etc"),
    ):
        self.root, self.report, self.sysfs, self.dev, self.etc = (
            root,
            report,
            sysfs,
            dev,
            etc,
        )
        self.capabilities = Capabilities.detect(etc=etc, sysfs=sysfs)
        self.user = pwd.getpwuid(os.getuid()).pw_name if pwd else getpass.getuser()
        self.usb = self.discover_usb()
        self.ports: list[str] = []
        self.findings: list[Finding] = []
        self.actions: list[Action] = []
        self.expected: set[str] = set()
        self.allowed = False

    def add(self, key, status, message, hint=""):
        self.findings.append(Finding(key, status, redact(message), redact(hint)))

    def command(self, argv, timeout=15):
        return self.report.run(list(argv), timeout=timeout)

    def propose(self, action):
        if action.python:
            action.capability = "python"
        elif action.argv:
            action.capability = {
                "apt-get": "packages",
                "groupadd": "groups",
                "usermod": "groups",
                "udevadm": "udev",
                "systemctl": "systemd",
            }.get(action.argv[0], action.capability)
        reason = self.capabilities.block_reason(action.capability)
        if reason:
            action.status, action.detail = "BLOCKED", reason
        if all(item.id != action.id for item in self.actions):
            self.actions.append(action)

    def discover_usb(self):
        mice = spacemouse_ids(self.root)
        devices = []
        for path in sorted((self.sysfs / "bus/usb/devices").glob("*")):
            vid, pid = read(path / "idVendor"), read(path / "idProduct")
            if not re.fullmatch("[0-9a-fA-F]{4}", vid) or not re.fullmatch(
                "[0-9a-fA-F]{4}", pid
            ):
                continue
            product = read(path / "product")
            pair = int(vid, 16), int(pid, 16)
            kind = (
                "spacemouse"
                if pair in mice
                else (
                    "realsense"
                    if vid.lower() == "8086" and "realsense" in product.lower()
                    else ""
                )
            )
            bus, number = read(path / "busnum"), read(path / "devnum")
            node = (
                str(self.dev / f"bus/usb/{int(bus):03}/{int(number):03}")
                if bus.isdigit() and number.isdigit()
                else ""
            )
            devices.append(
                dict(
                    vid=vid.lower(),
                    pid=pid.lower(),
                    product=product,
                    kind=kind,
                    node=node,
                    sysfs=str(path),
                    serial=read(path / "serial"),
                )
            )
        return devices

    def busy(self):
        paths = [*self.ports, *(d["node"] for d in self.usb if d["kind"])]
        # RealSense uses video nodes too; an owner may not hold the USB bus node.
        for subsystem in ("video4linux", "hidraw"):
            for entry in (self.sysfs / "class" / subsystem).glob("*"):
                ancestors = (entry / "device").resolve().parents
                if any(
                    Path(d["sysfs"]).resolve() in ancestors
                    for d in self.usb
                    if d["kind"]
                ):
                    paths.append(str(self.dev / entry.name))
        return process_owners(paths)

    def serial_identity(self, device):
        parent = (self.sysfs / "class/tty" / device.name / "device").resolve()
        usb = next(
            (d for d in self.usb if Path(d["sysfs"]).resolve() in parent.parents), None
        )
        if not usb:
            return None, ""
        serial = usb["serial"]
        selector = (
            f'ATTRS{{idVendor}}=="{usb["vid"]}", ATTRS{{idProduct}}=="{usb["pid"]}"'
        )
        if serial and re.fullmatch(r"[A-Za-z0-9_.:-]+", serial):
            selector += f', ATTRS{{serial}}=="{serial}"'
        elif re.fullmatch(r"[0-9]+-[0-9.]+", Path(usb["sysfs"]).name):
            selector += f', KERNELS=="{Path(usb["sysfs"]).name}"'
        else:
            return usb, ""
        return usb, selector

    def inspect_system(self, *, mock=False):
        caps = self.capabilities
        self.allowed = (
            True  # Individual operations have their own capability/precondition checks.
        )
        self.add(
            "host.platform",
            "PASS" if caps.device_probes or mock else "WARN",
            f"{caps.system} {caps.distribution} {caps.version} {caps.architecture}; container={caps.container}; wsl={caps.wsl}",
            "native Linux supports device checks; system repairs also require the relevant host tools; other hosts retain Python diagnostics",
        )
        self.add(
            "host.python",
            "PASS" if project_environment() else "WARN",
            sys.executable,
            "Python repairs require an active project venv or non-base conda environment",
        )
        if mock or not caps.linux:
            if not caps.linux and not mock:
                self.add(
                    "host.tools",
                    "SKIPPED",
                    "Linux host tools are not applicable on this platform",
                )
            return
        missing = sorted(
            {package for tool, package in TOOLS.items() if not shutil.which(tool)}
        )
        self.add(
            "host.tools",
            "FAIL" if missing else "PASS",
            "missing packages: " + ", ".join(missing)
            if missing
            else "diagnostic tools available",
        )
        if missing:
            self.propose(
                Action(
                    "apt:tools",
                    "host.tools",
                    "Install missing host diagnostic tools",
                    ["apt-get", "install", "-y", *missing],
                    privileged=True,
                )
            )
        self.expected.update(d["kind"] for d in self.usb if d["kind"])
        if not self.usb:
            self.add(
                "usb.discovery",
                "WARN",
                "no USB devices visible",
                "check cabling; this is not evidence that hardware passed",
            )

    def groups(self, group, check):
        if grp is None or not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_.@-]*[$]?", self.user):
            self.add(
                check + ".group",
                "BLOCKED",
                "user-group repair is unavailable for this account/platform",
            )
            return []
        listed = False
        try:
            entry = grp.getgrnam(group)
            listed = self.user in entry.gr_mem or entry.gr_gid == os.getgid()
            active = entry.gr_gid in {os.getgid(), *os.getgroups()}
        except KeyError:
            active = False
        if active:
            return []
        if listed:
            self.add(
                check + ".group",
                "PENDING_ACTION",
                f"{group} added but not effective in this session",
                "log out and back in, then rerun doctor",
            )
            return []
        create = Action(
            "group:" + group,
            check,
            f"Ensure {group} group exists",
            ["groupadd", "-f", group],
            privileged=True,
        )
        join = Action(
            "member:" + group,
            check,
            f"Add {self.user} to {group}",
            ["usermod", "-aG", group, self.user],
            privileged=True,
            dependencies=[create.id],
            pending="log out and back in before device access is verified",
        )
        self.propose(create)
        self.propose(join)
        return [join.id]

    def rules(self, check, kind, devices):
        lines = ["# Managed by rwrl doctor; targeted at supported devices."]
        if kind == "spacemouse":
            pairs = sorted(spacemouse_ids(self.root))
            if not pairs:
                self.add(
                    check + ".rules",
                    "BLOCKED",
                    "canonical SpaceMouse device table not found; no rule generated",
                )
                return
        else:
            pairs = sorted({(int(d["vid"], 16), int(d["pid"], 16)) for d in devices})
        for vid, pid in pairs:
            for subsystem in (
                ["usb", "hidraw"]
                if kind == "spacemouse"
                else ["usb", "video4linux", "hidraw"]
            ):
                lines.append(
                    f'SUBSYSTEM=="{subsystem}", ATTRS{{idVendor}}=="{vid:04x}", ATTRS{{idProduct}}=="{pid:04x}", MODE="0660", GROUP="plugdev"'
                )
        content = "\n".join(lines) + "\n"
        target = self.etc / f"udev/rules.d/99-rwrl-{kind}.rules"
        if read(target) == content.strip():
            return
        action = file_action(
            check,
            f"Install {kind} USB/backend permission rules",
            str(target),
            content,
            pending="replug the target device, then rerun doctor",
        )
        self.propose(action)

    def inspect_usb(self):
        for kind in ("spacemouse", "realsense"):
            if kind not in self.expected:
                continue
            devices = [d for d in self.usb if d["kind"] == kind]
            check = "usb." + kind
            if not devices:
                self.add(
                    check,
                    "BLOCKED",
                    f"no supported {kind} USB device attached",
                    "attach the configured device and rerun",
                )
                continue
            denied = [
                d["node"]
                for d in devices
                if not d["node"] or not os.access(d["node"], os.R_OK | os.W_OK)
            ]
            self.add(
                check,
                "FAIL" if denied else "PASS",
                f"{kind}: "
                + (
                    f"USB node access denied: {denied}"
                    if denied
                    else "USB nodes readable/writable"
                ),
                "USB access and SDK reads must both pass; hidraw access alone does not prove libusb access",
            )
            if denied:
                self.groups("plugdev", check)
                self.rules(check, kind, devices)

    def inspect_gello(self, configs):
        candidates = []
        for config in configs:
            teleop = getattr(config, "teleop", None)
            if teleop is not None and teleop.backend == "gello":
                candidates.append(
                    (teleop.options.get("port"), teleop.options.get("event_device"))
                )
                self.expected.add("gello")
                if teleop.options.get("event_device"):
                    self.expected.add("gello-events")
        # A serial adapter alone is not proof that a particular calibration/leader is GELLO.
        if not candidates:
            ports = sorted(
                str(path)
                for pattern in ("ttyUSB*", "ttyACM*")
                for path in self.dev.glob(pattern)
            )
            if ports:
                self.add(
                    "gello.candidates",
                    "SKIPPED",
                    "uncommissioned serial candidates: " + ", ".join(ports),
                    "select the GELLO port and calibration in a station/preset before device reads or repairs",
                )
        for index, (port, event) in enumerate(candidates):
            check = f"gello.{index}"
            if not port:
                self.add(
                    check,
                    "BLOCKED",
                    "GELLO serial port is not configured",
                    "commission the leader port and calibration; doctor never calibrates it",
                )
                continue
            self.ports.append(port)
            device = Path(port).resolve()
            accessible = device.exists() and os.access(device, os.R_OK | os.W_OK)
            self.add(
                check,
                "PASS" if accessible else "FAIL",
                f"serial {port}: "
                + ("readable/writable" if accessible else "missing or access denied"),
            )
            usb, selector = self.serial_identity(device)
            if device.exists() and not accessible:
                self.groups("dialout", check)
                if selector:
                    target = self.etc / f"udev/rules.d/99-rwrl-gello-{index}.rules"
                    content = (
                        f'SUBSYSTEM=="tty", {selector}, MODE="0660", GROUP="dialout"\n'
                    )
                    self.propose(
                        file_action(
                            check,
                            "Install target GELLO serial permission rule",
                            str(target),
                            content,
                            pending="replug the adapter, then rerun doctor",
                        )
                    )
            if event:
                self.ports.append(event)
                ok = Path(event).exists() and os.access(event, os.R_OK | os.W_OK)
                self.add(
                    check + ".events", "PASS" if ok else "FAIL", f"event device {event}"
                )
                if Path(event).exists() and not ok:
                    self.groups("input", check + ".events")
            timer = (
                self.sysfs / "bus/usb-serial/devices" / device.name / "latency_timer"
            )
            latency = read(timer)
            if latency and latency != "1" and selector and usb["vid"] == "0403":
                self.add(
                    check + ".latency",
                    "FAIL",
                    f"{device.name} latency timer is {latency} ms; target 1 ms",
                )
                target = self.etc / f"udev/rules.d/99-rwrl-gello-latency-{index}.rules"
                content = f'ACTION=="add", SUBSYSTEM=="usb-serial", {selector}, DRIVER=="ftdi_sio", ATTR{{latency_timer}}="1"\n'
                action = file_action(
                    check + ".latency",
                    "Set persistent target FTDI latency to 1 ms",
                    str(target),
                    content,
                    pending="replug the target adapter and rerun doctor",
                )
                self.propose(action)
            elif latency == "1":
                self.add(check + ".latency", "PASS", "FTDI latency is 1 ms")
            # Require a kernel log linking brltty to this specific tty, not merely installed software.
            code, logs = self.command(
                ["journalctl", "-k", "-b", "--no-pager", "-n", "500"]
            )
            proven = False
            for candidate in self.usb:
                if (candidate["vid"], candidate["pid"]) != ("1a86", "7523"):
                    continue
                topology = Path(candidate["sysfs"]).name
                lines = [
                    line
                    for line in logs.splitlines()
                    if re.search(
                        r"(?<![\d.-])" + re.escape(topology) + r"(?![\d.-])", line
                    )
                ]
                stolen = any(
                    "brltty" in line.lower()
                    and ("claimed" in line or "sets config" in line)
                    for line in lines
                )
                linked = any(
                    re.search(r"\b" + re.escape(device.name) + r"\b", line)
                    for line in lines
                )
                proven |= bool(not code and stolen and linked and not device.exists())
            if proven:
                self.add(
                    check + ".brltty",
                    "FAIL",
                    "kernel log links brltty to missing target serial port",
                    "review the affected accessibility service before approving its disablement",
                )
                self.propose(
                    Action(
                        "brltty:" + device.name,
                        check + ".brltty",
                        "Disable brltty-udev.service: affects braille device auto-detection",
                        ["systemctl", "mask", "--now", "brltty-udev.service"],
                        privileged=True,
                        exclusive=True,
                        pending="replug the target adapter, then rerun doctor",
                    )
                )
            elif not code and "brltty" in logs.lower():
                self.add(
                    check + ".brltty",
                    "WARN",
                    "brltty mentioned, but no proven conflict with target port; no service change proposed",
                )

    def inspect_network(self, configs):
        """Inspect route selection without changing NIC addresses or routing tables."""
        for ip in sorted({config.robot_ip for config in configs if config.robot_ip}):
            code, output = self.command(["ip", "-j", "route", "get", str(ip)])
            self.add(
                "network.route." + str(ip),
                "FAIL" if code else "PASS",
                output.strip()[-1800:],
                "review source address/interface and robot subnet manually; doctor never changes IP/routes",
            )

    def inspect_realtime(self):
        if "franky" not in self.expected:
            return
        if resource is None or not hasattr(resource, "RLIMIT_RTPRIO"):
            self.add(
                "realtime.limits",
                "BLOCKED",
                "Linux realtime resource limits are unavailable on this platform",
            )
            return
        if not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_.@-]*[$]?", self.user):
            self.add(
                "realtime.limits",
                "BLOCKED",
                "cannot generate a safe limits entry for this account; configure limits manually",
            )
            return
        kernel = read(self.sysfs / "kernel/realtime")
        self.add(
            "realtime.kernel",
            "PASS" if kernel == "1" else "WARN",
            f"kernel {platform.release()}: PREEMPT_RT flag={kernel or 'absent'}",
            "MAJOR CHANGE / DOUBLE CHECK: kernel installation, boot changes and reboot are manual; verify the cell's realtime requirements",
        )
        priority, _ = resource.getrlimit(resource.RLIMIT_RTPRIO)
        memory, _ = resource.getrlimit(resource.RLIMIT_MEMLOCK)
        ok = priority >= 99 and memory == resource.RLIM_INFINITY
        target = self.etc / "security/limits.d/99-rwrl-realtime.conf"
        content = f"{self.user} - rtprio 99\n{self.user} - memlock unlimited\n"
        if ok:
            self.add("realtime.limits", "PASS", "rtprio >=99 and memlock unlimited")
        elif read(target) == content.strip():
            self.add(
                "realtime.limits",
                "PENDING_ACTION",
                "limits configured but not effective in this session",
                "log out and back in through a PAM session, then rerun doctor",
            )
        else:
            self.add(
                "realtime.limits",
                "FAIL",
                f"rtprio={priority}, memlock={memory}",
                "user-specific limits require a new login; no kernel change",
            )
            self.propose(
                file_action(
                    "realtime.limits",
                    "Configure only the current user's realtime limits",
                    str(target),
                    content,
                    exclusive=False,
                    pending="log out and back in through a PAM session, then rerun doctor",
                )
            )

    def inspect_dependencies(self):
        # Python wheels cannot repair absent system shared libraries. Check separately.
        if self.capabilities.linux and self.expected & {"spacemouse", "realsense"}:
            for library, package in (("usb-1.0", "libusb-1.0-0"), ("udev", "libudev1")):
                script = (
                    "import ctypes, ctypes.util; p=ctypes.util.find_library("
                    + repr(library)
                    + "); assert p, 'shared library not found'; ctypes.CDLL(p)"
                )
                code, output = self.command([sys.executable, "-c", script])
                check = "system.library." + library
                self.add(
                    check,
                    "FAIL" if code else "PASS",
                    output.strip()[-1200:] if code else library + " loadable",
                )
                if code:
                    self.propose(
                        Action(
                            "apt:" + package,
                            check,
                            "Install missing/broken " + library + " system library",
                            ["apt-get", "install", "--reinstall", "-y", package],
                            privileged=True,
                            exclusive=True,
                        )
                    )
        from packaging.requirements import Requirement
        from packaging.utils import canonicalize_name

        selected = set(self.expected)
        try:
            raw = metadata.requires("rlgarden-robot-infra") or []
            requirements = [Requirement(row) for row in raw]
        except metadata.PackageNotFoundError:
            requirements = [
                Requirement(row)
                for extra in selected
                for row in FALLBACK_REQUIREMENTS.get(extra, [])
            ]
        # Include the mandatory requirements of all three local distributions.
        for distribution in ("real-world-rl", "rl_garden"):
            try:
                requirements.extend(
                    Requirement(row)
                    for row in metadata.requires(distribution) or []
                    if Requirement(row).marker is None
                    or Requirement(row).marker.evaluate({"extra": ""})
                )
            except metadata.PackageNotFoundError:
                pass
        wanted = {}
        for requirement in requirements:
            if requirement.marker is None or any(
                requirement.marker.evaluate({"extra": extra})
                for extra in ["", *selected]
            ):
                name = canonicalize_name(requirement.name)
                # pip evaluates 'extra' markers as empty for a direct requirement, so strip
                # the marker once its activation has been decided here.
                if requirement.url:
                    concrete = requirement
                else:
                    specifier = str(requirement.specifier)
                    if name in wanted and str(wanted[name].specifier):
                        specifier = ",".join(
                            filter(None, [str(wanted[name].specifier), specifier])
                        )
                    concrete = Requirement(requirement.name + specifier)
                wanted[name] = concrete
        for name, requirement in wanted.items():
            check = "deps." + name
            try:
                version = metadata.version(name)
                satisfied = not requirement.specifier or requirement.specifier.contains(
                    version, prereleases=True
                )
            except metadata.PackageNotFoundError:
                version, satisfied = "missing", False
            self.add(
                check,
                "PASS" if satisfied else "FAIL",
                f"{name}: {version}; requires {requirement}",
            )
            if not satisfied:
                if requirement.url:
                    self.add(
                        check + ".source",
                        "BLOCKED",
                        "non-index package repair requires manual review",
                    )
                    continue
                local = {
                    "rl-garden": "rl-garden",
                    "rlgarden-robot-infra": "robot_infra",
                }
                argv = [sys.executable, "-m", "pip", "install"]
                if name in local:
                    source = self.root / local[name] if self.root else None
                    if source is None or not (source / "pyproject.toml").is_file():
                        self.add(
                            check + ".source",
                            "BLOCKED",
                            "matching local distribution source is unavailable",
                            f"install {requirement} from your trusted project distribution source, then rerun doctor",
                        )
                        continue
                    argv += ["-e", str(source)]
                else:
                    argv += [str(requirement)]
                self.propose(
                    Action(
                        "pip:" + name,
                        check,
                        f"Install {requirement.name} satisfying {requirement.specifier or 'project metadata'}",
                        argv,
                        python=True,
                        exclusive=True,
                    )
                )
        for extra in sorted(selected & SDK_MODULES.keys()):
            module = SDK_MODULES[extra]
            script = (
                "import importlib; m=importlib.import_module("
                + repr(module)
                + "); "
                + (
                    "assert callable(getattr(m,'device',None)), 'wrong hid distribution; trezor hidapi required'"
                    if extra == "spacemouse"
                    else "print('import OK')"
                )
            )
            code, output = self.command([sys.executable, "-c", script], timeout=45)
            self.add(
                "sdk." + extra,
                "FAIL" if code else "PASS",
                output.strip()[-1200:] if code else module + " importable",
            )
            reqs = FALLBACK_REQUIREMENTS.get(extra, [])
            dependency_ids = [
                "pip:" + canonicalize_name(Requirement(row).name) for row in reqs
            ]
            planned = [item.id for item in self.actions]
            if code and not any(item in planned for item in dependency_ids):
                # Reinstall the installed version, never silently upgrade a version that satisfies metadata.
                for row in reqs:
                    req = Requirement(row)
                    name = canonicalize_name(req.name)
                    try:
                        exact = f"{req.name}=={metadata.version(req.name)}"
                    except metadata.PackageNotFoundError:
                        exact = row
                    self.propose(
                        Action(
                            "pip:" + name,
                            "sdk." + extra,
                            f"Repair broken {module} import by reinstalling {exact}",
                            [
                                sys.executable,
                                "-m",
                                "pip",
                                "install",
                                "--force-reinstall",
                                "--no-deps",
                                exact,
                            ],
                            python=True,
                            exclusive=True,
                        )
                    )
