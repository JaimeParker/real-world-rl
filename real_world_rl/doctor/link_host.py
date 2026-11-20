"""Local Linux listener, SSH server, firewall and tunnel-service diagnostics."""
from __future__ import annotations

import ipaddress
import os
import re
import shlex
import shutil
from dataclasses import dataclass
from pathlib import Path

from real_world_rl.common.link_config import LinkConfig, endpoint, host_port
from real_world_rl.doctor.platform import read_text
from real_world_rl.doctor.repair import Action, digest, file_action, user_service_directory

_LINGER = 'for logout persistence, run loginctl enable-linger "$(id -un)"'
_SSHD_MANUAL = "configure global AllowTcpForwarding local, then run sudo sshd -t and reload sshd"
_SSHD_CONTENT = "# Managed by rwrl doctor\nAllowTcpForwarding local\n"


def loopback(host: str) -> bool:
    if host.lower().rstrip(".") == "localhost":
        return True
    try:
        address = ipaddress.ip_address(host)
        return address.is_loopback or bool(getattr(address, "ipv4_mapped", None)
                                          and address.ipv4_mapped.is_loopback)
    except ValueError:
        return False


def _unmapped(address: str) -> str:
    """``::ffff:127.0.0.1`` (a dual-stack socket's IPv4-mapped form, as ss prints it) -> ``127.0.0.1``."""
    try:
        mapped = ipaddress.ip_address(address)
    except ValueError:
        return address
    mapped_v4 = getattr(mapped, "ipv4_mapped", None)
    return str(mapped_v4) if mapped_v4 is not None else address


@dataclass
class Listener:
    address: str
    processes: tuple[str, ...]
    daemon: bool


def listeners(host, address: str) -> tuple[list[Listener], str]:
    bind, port = endpoint(address)
    code, output = host.command(["ss", "-H", "-ltnp", f"sport = :{port}"], timeout=4)
    if code:
        return [], output.strip()
    matches = []
    for line in output.splitlines():
        fields = line.split(maxsplit=5)
        if len(fields) < 5 or fields[0] != "LISTEN":
            continue
        local, _, local_port = fields[3].rpartition(":")
        local = _unmapped(local.strip("[]"))
        if local_port != str(port):
            continue
        if bind.lower().rstrip(".") == "localhost":
            matched = loopback(local) or local in ("*", "0.0.0.0", "::")
        else:
            matched = (
                local == bind or bind in ("0.0.0.0", "::") or local in ("*", "::")
                or (local == "0.0.0.0" and ":" not in bind)
            )
        if not matched:
            continue
        process = fields[5] if len(fields) > 5 else ""
        names = tuple(dict.fromkeys(re.findall(r'\("([^"]+)",pid=', process)))
        # ss only shows command names. Verify the rwrl command without printing its arguments.
        daemon = False
        for pid in re.findall(r"pid=([0-9]+)", process):
            argv = read_text(Path(f"/proc/{pid}/cmdline")).strip("\0").split("\0")
            daemon |= any(
                (Path(arg).name == "rwrl" and "daemon" in argv[i + 1:])
                or arg == "real_world_rl.server.daemon"
                or (arg == "real_world_rl.cli" and "daemon" in argv[i + 1:])
                for i, arg in enumerate(argv[:4])
            )
        matches.append(Listener(fields[3], names, daemon))
    return matches, ""


def _listener_description(items: list[Listener]) -> str:
    return ", ".join(
        f"{item.address} ({'rwrl daemon' if item.daemon else ', '.join(item.processes) or 'owner hidden'})"
        for item in items
    )


def _daemon_listen(host, address: str) -> None:
    items, error = listeners(host, address)
    if error:
        host.add("link.daemon.listen", "BLOCKED", f"{address}: cannot inspect listeners: {error}",
                 "install iproute2 or inspect the listener on the daemon host")
    elif not items:
        host.add("link.daemon.listen", "FAIL", f"{address}: no TCP listener",
                 "check the daemon log and the configured bind, then start rwrl daemon if needed")
    elif all(item.daemon for item in items):
        host.add("link.daemon.listen", "PASS", _listener_description(items))
    else:
        known = all(item.processes for item in items)
        host.add("link.daemon.listen", "FAIL" if known else "WARN", _listener_description(items),
                 "verify that rwrl daemon owns this address" if known else
                 "inspect sudo ss -ltnp to identify the listener")


def _privileged_probe(argv: list[str]) -> list[str]:
    return ["sudo", "-n", *argv] if getattr(os, "geteuid", lambda: -1)() != 0 else argv


def _sshd_include(main: Path, target: Path) -> bool:
    """Check for a global Include that would read the dedicated drop-in."""
    for line in read_text(main).splitlines():
        try:
            parts = shlex.split(line, comments=True)
        except ValueError:
            return False
        if not parts:
            continue
        keyword, _, attached = parts[0].partition("=")
        args = ([attached] if attached else []) + parts[1:]
        if keyword.lower() == "match":
            break
        if keyword.lower() == "include" and any(
            target.match(str(main.parent / pattern))
            for pattern in args if pattern != "="
        ):
            return True
    return False


def _permit_open(value: str, remote_bind: str) -> bool:
    remote = host_port(*endpoint(remote_bind))
    bind, port = remote.rsplit(":", 1)
    return value.lower() == "any" or bool(
        set(value.split()) & {remote, f"{bind}:*", f"*:{port}", "*:*"}
    )


def _sshd(host, remote_bind: str) -> None:
    key = "link.sshd"
    if host.command(["systemctl", "--version"], timeout=4)[0] != 0:
        # No systemd (a container, another init): the service state is unknown, not absent.
        running = host.command(["pgrep", "-x", "sshd"], timeout=4)[0] == 0
        host.add(key, "BLOCKED",
                 "cannot inspect the SSH service without systemd" + ("; an sshd process is running" if running else ""),
                 "run rwrl doctor on the NUC host itself, outside containers")
        return
    service = next((name for name in ("ssh.service", "sshd.service")
                    if host.command(["systemctl", "is-active", "--quiet", name], timeout=4)[0] == 0), None)
    socket_active = service is None and host.command(
        ["systemctl", "is-active", "--quiet", "ssh.socket"], timeout=4,
    )[0] == 0
    if not service and not socket_active:
        host.add(key, "FAIL", "no active ssh.service, sshd.service or ssh.socket",
                 "check the SSH server on the NUC, or use the same-LAN gRPC route")
        return
    sshd = shutil.which("sshd") or "/usr/sbin/sshd"
    main = host.etc / "ssh/sshd_config"
    target = main.parent / "sshd_config.d/99-rwrl.conf"
    config_args = ["-f", str(main)]
    code, output = host.command(_privileged_probe([sshd, "-T", *config_args]), timeout=5)
    if code:
        host.add(key, "BLOCKED", f"cannot read effective sshd configuration: {output.strip()[-500:]}",
                 "run sudo -v in a terminal and rerun doctor, or review sudo sshd -T on the NUC")
        return
    settings = {parts[0].lower(): parts[1] for line in output.splitlines()
                if len(parts := line.split(None, 1)) == 2}
    forwarding = settings.get("allowtcpforwarding", "unknown").lower()
    disabled = settings.get("disableforwarding", "unknown").lower()
    permitted = settings.get("permitopen", "unknown")
    message = (f"{service or 'ssh.socket'} active; AllowTcpForwarding={forwarding}, "
               f"DisableForwarding={disabled}, PermitOpen={permitted}")
    if disabled != "no" or not _permit_open(permitted, remote_bind):
        host.add(key, "BLOCKED", message,
                 f"review DisableForwarding and PermitOpen for {remote_bind} with the SSH administrator")
    elif forwarding in ("yes", "all", "local"):
        host.add(key, "PASS", message)
    elif forwarding not in ("no", "remote"):
        host.add(key, "BLOCKED", message, _SSHD_MANUAL)
    elif not service or not target.parent.is_dir() or not _sshd_include(main, target):
        host.add(key, "BLOCKED", message + "; no usable global sshd_config.d Include or reloadable service",
                 _SSHD_MANUAL)
    else:
        try:
            action = file_action(key, "MAJOR CHANGE / DOUBLE CHECK: allow SSH local forwarding",
                                 str(target), _SSHD_CONTENT, exclusive=False)
            action.prerequisites = {str(main): digest(main)}
        except OSError:
            host.add(key, "BLOCKED", message + "; configuration is unreadable", _SSHD_MANUAL)
            return
        if target.is_symlink() or read_text(target) == _SSHD_CONTENT.strip():
            host.add(key, "BLOCKED", message + "; a symlink or another setting takes precedence", _SSHD_MANUAL)
            return
        action.capability = "systemd"
        action.validate_argv = [sshd, "-t", *config_args]
        action.verify_argv = [sshd, "-T", *config_args]
        action.verify_line = "allowtcpforwarding local"
        action.after_argv = [["systemctl", "reload", service]]
        host.add(key, "FAIL", message, "review the dedicated drop-in repair with --fix")
        host.propose(action)


def _firewall(host, address: str) -> None:
    bind, port = endpoint(address)
    key = "link.firewall"
    if loopback(bind):
        host.add(key, "SKIPPED", "loopback daemon bind; no direct LAN ingress needed")
        return
    if not shutil.which("ufw"):
        host.add(key, "SKIPPED", "ufw unavailable")
        return
    code, output = host.command(_privileged_probe(["ufw", "status", "verbose"]), timeout=5)
    if code:
        host.add(key, "BLOCKED", "cannot inspect ufw", "authorize sudo or check sudo ufw status verbose")
        return
    if re.search(r"^Status:\s+inactive", output, re.M | re.I):
        host.add(key, "SKIPPED", "ufw inactive")
        return
    if not re.search(r"^Status:\s+active", output, re.M | re.I):
        host.add(key, "BLOCKED", "unrecognized ufw status", "review sudo ufw status verbose")
        return
    default_allow = bool(re.search(r"^Default:\s+allow \(incoming\)", output, re.M | re.I))
    family = 6 if ":" in bind else 4
    restricted = False
    for line in output.splitlines():
        match = re.match(r"(.+?)\s+(ALLOW|DENY|REJECT|LIMIT)(?: IN)?\s+(.+)$", line)
        if not match:
            continue
        destination, rule, source = match.groups()
        v6 = "(v6)" in destination
        if family != (6 if v6 else 4):
            continue
        dest = destination.removesuffix(" (v6)").split()[0]
        if dest not in (str(port), f"{port}/tcp", "Anywhere"):
            restricted |= bool(re.search(rf"(?<!\d){port}(?!\d)", destination))
            continue
        if source.strip() not in ("Anywhere", "Anywhere (v6)") or " on " in destination:
            restricted = True
            continue
        if not restricted:
            default_allow = rule in ("ALLOW", "LIMIT")
        break
    status = "WARN" if restricted else "PASS" if default_allow else "FAIL"
    message = ("source/interface-specific rules need review" if restricted else
               "allowed by ufw" if default_allow else "no matching ufw allowance")
    host.add(key, status, f"{address}: {message}",
             "review sudo ufw status verbose for the GPU host, or use the SSH tunnel")


def inspect_server(host, address: str, token: str | None) -> None:
    bind, port = endpoint(address)
    local = loopback(bind)
    host.add("link.daemon.bind", "PASS" if local or token else "WARN",
             f"{address}: " + ("loopback (SSH tunnel)" if local else
                              "non-loopback, token configured" if token else "non-loopback, no token"),
             "use a loopback bind with SSH forwarding or configure a daemon token")
    _daemon_listen(host, address)
    _sshd(host, address if local else host_port("127.0.0.1" if bind == "0.0.0.0" else
                                              "::1" if bind == "::" else bind, port))
    _firewall(host, address)


def ssh_options() -> list[str]:
    return [
        "-o", "BatchMode=yes", "-o", "ConnectTimeout=10",
        "-o", "StrictHostKeyChecking=yes", "-o", "UpdateHostKeys=no",
        "-o", "PermitLocalCommand=no", "-o", "GatewayPorts=no",
        "-o", "ForkAfterAuthentication=no", "-o", "ForwardAgent=no", "-o", "ForwardX11=no",
    ]


def tunnel_service(link: LinkConfig, autossh: str) -> str:
    # Escape the executable for systemd (its specifier expansion is independent of quoting).
    executable = '"' + autossh.replace("\\", "\\\\").replace('"', '\\"').replace("%", "%%") + '"'
    local_bind = host_port(*endpoint(link.address))
    remote_bind = host_port(*endpoint(link.remote_bind))
    argv = ["-M", "0", "-N", "-o", "ServerAliveInterval=5", "-o", "ServerAliveCountMax=3",
            "-o", "ExitOnForwardFailure=yes", *ssh_options(), "-o", "ControlPath=none", "-L", f"{local_bind}:{remote_bind}", "--", link.ssh]
    return (
        "[Unit]\nDescription=rwrl daemon SSH tunnel\n\n"
        "[Service]\nEnvironment=AUTOSSH_GATETIME=0\nExecStart=" + executable + " " + " ".join(argv) + "\n"
        "Restart=always\nRestartSec=5\n\n[Install]\nWantedBy=default.target\n"
    )


def inspect_tunnel(host, link: LinkConfig, name: str, *, ssh_ok: bool) -> None:
    key = "link.tunnel"
    bind, port = endpoint(link.address)
    if not loopback(bind):
        host.add(key, "SKIPPED" if not link.ssh else "BLOCKED",
                 f"{link.address} is a direct gRPC endpoint",
                 "use a loopback --link address to configure an SSH tunnel")
        return
    # Include the local port for CLI-only links (whose name is the port) so units do not collide.
    unit = "rwrl-tunnel-" + (re.sub(r"[^a-z0-9-]+", "-", name.lower()).strip("-") or str(port)) + ".service"
    directory = user_service_directory()
    target = directory / unit
    ctl = ["systemctl", "--user"]
    state_code, state_output = host.command([*ctl, "is-active", unit], timeout=4)
    state = state_output.strip()
    state_known = state in {"active", "inactive", "failed", "activating", "deactivating", "reloading"}
    items, error = listeners(host, link.address)
    suffix = f"{unit}={state}" if state_known else f"{unit} state unavailable"
    if error:
        host.add(key, "BLOCKED", f"cannot inspect {link.address}: {error}; {suffix}",
                 "install iproute2 and check the local SSH forward")
    elif items:
        is_ssh = all(item.processes and set(item.processes) <= {"ssh", "autossh"} for item in items)
        host.add(key, "PASS" if is_ssh else "WARN", f"{_listener_description(items)}; {suffix}",
                 "verify who owns the port; use a free loopback port if needed")
    elif not link.ssh:
        host.add(key, "BLOCKED", f"{link.address} has no listener; {suffix}", "pass --ssh [USER@]HOST")
    elif not state_known or not ssh_ok or state_code == 0 or state not in ("inactive", "failed"):
        host.add(key, "BLOCKED", f"{link.address} has no listener; {suffix}",
                 "check link.ssh, the user systemd session, and the unit journal")
    else:
        code, fragment = host.command([*ctl, "show", "--property=FragmentPath", "--value", unit], timeout=4)
        if code or (fragment.strip() and Path(fragment.strip()) != target) or target.is_symlink():
            host.add(key, "BLOCKED", f"{suffix}; cannot establish ownership of the service file",
                     "inspect the unit with systemctl --user cat before making repairs")
            return
        autossh = shutil.which("autossh")
        dependencies = []
        if not autossh:
            host.propose(Action("apt:autossh", key, "Install autossh",
                                ["apt-get", "install", "-y", "autossh"], privileged=True))
            autossh = "/usr/bin/autossh"
            dependencies.append("apt:autossh")
        try:
            before = digest(target)
        except OSError:
            host.add(key, "BLOCKED", f"cannot read {target}", "check its permissions")
            return
        host.add(key, "FAIL", f"{link.address} has no listener; {suffix}",
                 "run doctor --fix to install or restart the user tunnel service")
        commands = [[*ctl, "daemon-reload"]]
        if state == "failed":
            commands.extend([[*ctl, "enable", unit], [*ctl, "restart", unit]])
        else:
            commands.append([*ctl, "enable", "--now", unit])
        host.propose(Action(
            "file:" + str(target), key, f"Configure and start {unit}",
            target=str(target), content=tunnel_service(link, autossh),
            before=before, dependencies=dependencies, after_argv=commands, advice=_LINGER,
        ))
