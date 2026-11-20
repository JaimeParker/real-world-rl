"""Read-only client probes for the daemon, SSH forwarding and openpi policy."""
from __future__ import annotations

import contextlib
import importlib.metadata
import ipaddress
import math
import os
import socket
import statistics
import sys
import time
from pathlib import Path
from tempfile import TemporaryDirectory

from real_world_rl.common.link_config import LinkConfig, endpoint, host_port
from real_world_rl.doctor.deps import install_extras_hint
from real_world_rl.doctor.link_host import inspect_tunnel, ssh_options
from real_world_rl.doctor.repair import Action

_RPC_TIMEOUT_S = 2
_FAKE_IP = ipaddress.ip_network("198.18.0.0/15")


def _imports(host, module: str) -> bool:
    return host.command([sys.executable, "-c", f"import {module}"], timeout=10)[0] == 0


def _repair_extra(host, check: str, extra: str) -> None:
    if extra == "msgpack":
        action_id = "pip:msgpack"
        args = ["--force-reinstall", "msgpack"]
    else:
        action_id = "pip:extra:" + extra
        if host.root:
            args = ["-e", f"{host.root}[{extra}]"]
        else:
            try:
                version = importlib.metadata.version("real-world-rl")
            except importlib.metadata.PackageNotFoundError:
                return
            args = [f"real-world-rl[{extra}]=={version}"]
    host.propose(Action(action_id, check, f"Install {extra} in the project environment",
                        [sys.executable, "-m", "pip", "install", *args], python=True, exclusive=True))


def inspect_preprocess(host) -> None:
    key = "link.preprocess"
    if _imports(host, "cv2"):
        host.add(key, "PASS", "OpenCV importable for frame_preprocess_cv2_v1")
    else:
        host.add(key, "WARN", "OpenCV unavailable; cv2 preprocessing falls back to full frames over the tunnel",
                 install_extras_hint(["jpeg"]))
        _repair_extra(host, key, "jpeg")


def _dependencies(host) -> list[str]:
    checks = {"grpc": "grpc", "msgpack": "msgpack", "openpi-client": "websockets.sync.client"}
    missing = [extra for extra, module in checks.items() if not _imports(host, module)]
    for extra in missing:
        _repair_extra(host, "link.deps", extra)
    return missing


def _report_dependencies(host, missing: list[str], *, jpeg_missing: bool) -> None:
    message = f"missing or broken: {', '.join(missing)}" if missing else "grpc, msgpack and websockets importable"
    if jpeg_missing:
        message += "; the daemon advertises JPEG and neither Pillow nor OpenCV can decode it"
        _repair_extra(host, "link.deps", "jpeg")
    host.add("link.deps", "FAIL" if missing or jpeg_missing else "PASS", message,
             "activate the project environment and use doctor --fix to repair the link dependencies")


def _bypasses(host: str, port: int, bypass: str) -> bool:
    host = host.lower().rstrip(".")
    for raw in bypass.split(","):
        rule = raw.strip().lower()
        if rule == "*":
            return True
        if not rule:
            continue
        try:
            if ipaddress.ip_address(host) in ipaddress.ip_network(rule, strict=False):
                return True
        except ValueError:
            pass
        try:
            rule, rule_port = endpoint(rule)
        except ValueError:
            rule = rule.strip("[]")
        else:
            if rule_port != port:
                continue
        suffix = rule.lstrip(".").rstrip(".")
        if host == suffix or host.endswith("." + suffix):
            return True
    return False


def _proxy(host, link: LinkConfig) -> None:
    key = "link.proxy"
    address, port = endpoint(link.address)
    env = {key.lower(): value for key, value in os.environ.items()}
    env.update({key: value for key, value in os.environ.items() if key == key.lower()})
    proxies = [name for name in ("grpc_proxy", "https_proxy", "http_proxy", "all_proxy") if env.get(name)]
    bypass = env.get("no_grpc_proxy", env.get("no_proxy", ""))
    warnings = []
    hints = []
    if proxies and not _bypasses(address, port, bypass):
        warnings.append(f"{', '.join(proxies)} set; {address} has no proxy bypass")
        hosts = ",".join(dict.fromkeys(("127.0.0.1", "localhost", address)))
        hints.append(f'export no_proxy="${{no_proxy:+$no_proxy,}}{hosts}"')
        if "no_grpc_proxy" in env:
            hints.append('export no_grpc_proxy="${no_grpc_proxy:+$no_grpc_proxy,}$no_proxy"')
    resolution_failed = False
    if link.ssh:
        ssh_host = link.ssh.rpartition("@")[2]
        code, config = host.command(["ssh", "-G", "--", link.ssh], timeout=4)
        if not code:
            ssh_host = next((line.split()[1] for line in config.splitlines()
                             if line.startswith("hostname ") and len(line.split()) == 2), ssh_host)
        code, addresses = host.command(
            [sys.executable, "-c",
             "import socket, sys; print(*(r[4][0] for r in socket.getaddrinfo(sys.argv[1], None)), sep='\\n')",
             ssh_host], timeout=4,
        )
        resolution_failed = bool(code)
        if not code and any(_fake_ip(value) for value in addresses.split()):
            warnings.append(f"{ssh_host} resolves inside 198.18.0.0/15 (proxy fake-IP range)")
            hints.append("bypass the SSH destination in the TUN proxy or use the NUC's real IP")
    if warnings:
        host.add(key, "WARN", "; ".join(warnings), "\n".join(hints))
    elif resolution_failed:
        host.add(key, "BLOCKED", "no gRPC proxy warning; could not resolve the SSH host",
                 "check DNS and link.ssh; SSH configuration aliases may resolve only inside SSH")
    else:
        host.add(key, "PASS", "no proxy interception detected")


def _fake_ip(value: str) -> bool:
    try:
        return ipaddress.ip_address(value) in _FAKE_IP
    except ValueError:
        return False


def _ssh_failure(output: str) -> tuple[str, str]:
    stderr = output.lower()
    for pattern, message, hint in (
        ("identification has changed", "SSH host key changed", "verify the NUC fingerprint before updating known_hosts"),
        ("host key verification failed", "SSH host key verification failed",
         "verify the NUC fingerprint, then connect interactively once to add its key to known_hosts"),
        ("permission denied (publickey", "SSH permission denied (publickey)",
         "check the SSH username; load the private key with ssh-add and install its public key on the NUC"),
        ("permission denied", "SSH permission denied", "check SSH account and key authentication on the NUC"),
        ("timed out", "SSH connection timed out", "check the NUC address, route and SSH firewall"),
        ("timeout", "SSH connection timed out", "check the NUC address, route and SSH firewall"),
        ("could not resolve", "SSH could not resolve the host", "check DNS or use the NUC's LAN IP in --ssh"),
        ("connection refused", "SSH connection refused", "check ssh.service/sshd.service and the SSH port on the NUC"),
    ):
        if pattern in stderr:
            return message, hint
    return "SSH handshake failed", "try SSH from this account and inspect commands.log"


def _ssh(host, target: str | None) -> bool:
    if not target:
        host.add("link.ssh", "SKIPPED", "SSH target not configured")
        return False
    start = time.perf_counter()
    code, output = host.command(
        ["ssh", *ssh_options(), "-o", "ClearAllForwardings=yes", "-o", "ControlPath=none", "--", target, "true"], timeout=12,
    )
    elapsed = time.perf_counter() - start
    if code:
        message, hint = _ssh_failure(output)
        host.add("link.ssh", "FAIL", f"{target}: {message} after {elapsed:.2f}s", hint)
        return False
    host.add("link.ssh", "WARN" if elapsed > 2 else "PASS", f"{target}: handshake {elapsed:.2f}s",
             "check SSH routing, DNS and proxy latency")
    return True


def _grpc(link: LinkConfig, address: str, *, proxy: bool = True):
    from real_world_rl.server.transport.grpc_transport import GrpcClient

    return contextlib.closing(GrpcClient(address, token=link.token, timeout_s=_RPC_TIMEOUT_S,
                                         reconnect_attempts=0, enable_http_proxy=proxy))


def _rpc_hint(error: Exception, token: str | None) -> tuple[str, str]:
    message = str(error)
    if "token mismatch" in message.lower() or "unauthenticated" in message.lower():
        return "UNAUTHENTICATED (token mismatch)", "use the daemon's token with --token or link.token"
    if token:
        message = message.replace(token, "<redacted>")
    return message, "check the daemon log, bind, firewall, and the SSH tunnel if used"


def _forward(host, link: LinkConfig, *, ssh_ok: bool, grpc_ok: bool) -> None:
    key = "link.forward"
    if not link.ssh or not ssh_ok or not grpc_ok:
        host.add(key, "SKIPPED", "temporary forward requires working SSH and gRPC dependencies")
        return
    if os.name != "posix":
        host.add(key, "BLOCKED", "temporary forward requires Unix SSH control sockets",
                 "check the configured link.daemon endpoint directly")
        return
    try:
        # OpenSSH cannot allocate a port for -L. Release a loopback reservation just before ssh binds.
        with socket.socket() as reservation:
            reservation.bind(("127.0.0.1", 0))
            port = reservation.getsockname()[1]
        address = f"127.0.0.1:{port}"
        remote = host_port(*endpoint(link.remote_bind))
        with TemporaryDirectory(prefix="rwrl-ssh-") as directory:
            control = Path(directory) / "ctl"
            argv = ["ssh", *ssh_options(), "-M", "-S", str(control), "-N", "-o", "ExitOnForwardFailure=yes",
                    "-L", f"{address}:{remote}", "--", link.ssh]
            with host.command(argv, background=True) as (process, diagnostics):
                deadline = time.monotonic() + 12
                try:
                    while True:
                        if process.poll() is not None or time.monotonic() >= deadline:
                            raise TimeoutError("temporary SSH forward did not start")
                        if not control.exists():
                            time.sleep(0.05)
                            continue
                        try:
                            with socket.create_connection(("127.0.0.1", port), timeout=0.25):
                                pass
                        except OSError:
                            time.sleep(0.05)
                        else:
                            break
                    with _grpc(link, address, proxy=False) as client:
                        client.open()
                        client.call("robot.describe")
                    if process.poll() is not None:
                        raise OSError("temporary SSH forward exited")
                except Exception as error:
                    stderr = diagnostics().lower()
                    if "administratively prohibited" in stderr:
                        host.add(key, "FAIL", "SSH local forwarding administratively prohibited",
                                 "run doctor on the NUC and review link.sshd, Match rules and authorized_keys")
                    elif "connect failed: connection refused" in stderr:
                        host.add(key, "FAIL", f"SSH reached the NUC but it refused {link.remote_bind}",
                                 "check that rwrl daemon is running on the NUC and check its bind")
                    else:
                        message, hint = _rpc_hint(error, link.token)
                        host.add(key, "FAIL", f"temporary forward: {message}", hint + "; see commands.log")
                    return
    except OSError as error:
        host.add(key, "BLOCKED", f"cannot start temporary forward: {error}", "check local socket and ssh access")
        return
    host.add(key, "PASS", f"temporary SSH forward to {link.remote_bind} returned robot.describe")


def _observation_rate(client, describe: dict) -> tuple[str, float | None]:
    """The sampled observation throughput, and the seconds one observation takes."""
    cameras = describe.get("cameras") or {}
    if not cameras:
        return "no advertised cameras", None
    cv2 = describe.get("frame_preprocess_cv2_v1")
    prep = None
    if cv2 or describe.get("frame_preprocess_v1"):
        prep = {
            camera: {"config": {"resize": "cv2_linear" if cv2 else "pil_bilinear"},
                     "size": [128, 128], "color": True, "depth": False}
            for camera in cameras
        }
    samples = 3
    total_bytes = 0
    # Connect only an idle daemon robot, and disconnect only that connection: a running client
    # (deploy-openpi, training) keeps its own connection.
    connected_here = not client.call("robot.health").get("connected", False)
    if connected_here:
        _slow_call(client, "robot.connect")
    disconnect_note = ""
    try:
        start = time.perf_counter()
        for _ in range(samples):
            _slow_call(client, "robot.get_observation", **({"frame_preprocess": prep} if prep is not None else {}))
            total_bytes += client.last_response_bytes
        elapsed = time.perf_counter() - start
    finally:
        if connected_here:
            try:
                _slow_call(client, "robot.disconnect")
            except Exception as error:  # Keep the measurement; the daemon idles an unused robot.
                disconnect_note = f"; robot.disconnect after sampling failed: {getattr(error, 'message', error)}"
    mode = "128x128 daemon preprocess" if prep is not None else "full frames"
    per_observation = elapsed / samples
    return (f"{total_bytes:,} response bytes / {elapsed:.3f}s "
            f"({total_bytes / max(elapsed, 1e-9) / 1e6:.2f} MB/s, {per_observation:.3f}s per observation, "
            f"{mode}){disconnect_note}"), per_observation


# Device connect/disconnect on the daemon (a franky handshake, camera start/stop) and camera
# frames over a slow link outlast a probe RPC; a slow frame is a measurement, not a failure.
_DEVICE_CALL_TIMEOUT_S = 20.0
# Without a known control rate, an observation slower than this is reported.
_SLOW_OBSERVATION_S = 0.2


def _slow_call(client, method: str, **kwargs):
    timeout = getattr(client, "timeout_s", None)
    if timeout is None:
        return client.call(method, **kwargs)
    client.timeout_s = max(timeout, _DEVICE_CALL_TIMEOUT_S)
    try:
        return client.call(method, **kwargs)
    finally:
        client.timeout_s = timeout


def _positive_number(value) -> bool:
    return type(value) in (float, int) and 0 < value < math.inf


def _quality(host, client, link: LinkConfig, describe: dict, *, jpeg_missing: bool = False) -> None:
    key = "link.quality"
    hints = []
    rtts = []
    try:
        for _ in range(10):
            start = time.perf_counter()
            client.call("robot.describe")
            rtts.append(time.perf_counter() - start)
    except Exception as error:
        message, hint = _rpc_hint(error, link.token)
        host.add(key, "FAIL", f"could not sample describe RTT: {message}", hint)
        return
    # Nearest-rank p95 with 10 samples is the slowest sample.
    p50, p95 = statistics.median(rtts), max(rtts)
    gap = 3 * p95  # The policy metadata probe sends no inference.
    watchdog = describe.get("watchdog_s")
    control_hz = link.control_hz or describe.get("control_hz")
    note = f"watchdog {watchdog:g}s" if _positive_number(watchdog) else "watchdog unavailable on daemon"
    if _positive_number(watchdog) and gap >= watchdog:
        suggested = math.ceil(max(gap * 1.25, gap + 0.1) * 10) / 10
        hints.append(f"estimated gap reaches the watchdog; review --watchdog-s {suggested:g} on the daemon "
                     "and allow for real policy inference time")
    if _positive_number(control_hz) and p95 > 1 / control_hz:
        hints.append(f"p95 exceeds the {control_hz:g} Hz control period; review network delay or the control rate")
    if jpeg_missing:
        rate = "observation sampling skipped"
        hints.append("repair the JPEG decoder reported in link.deps")
    else:
        try:
            rate, per_observation = _observation_rate(client, describe)
            if per_observation is not None:
                if _positive_number(control_hz) and per_observation > 1 / control_hz:
                    hints.append(f"one observation takes {per_observation:.2f}s, longer than the {control_hz:g} Hz "
                                 "control period; lower --real-robot.control-hz, shrink the camera streams, or use a "
                                 "faster route (same LAN, no proxy)")
                elif not _positive_number(control_hz) and per_observation > _SLOW_OBSERVATION_S:
                    hints.append(f"one observation takes {per_observation:.2f}s, so steps cannot run faster than "
                                 f"{1 / per_observation:.1f} Hz; check link bandwidth (same LAN, no proxy)")
        except Exception as error:
            hints.append("check daemon camera availability and link bandwidth")
            message, _ = _rpc_hint(error, link.token)
            rate = f"observation sampling failed: {message}"
    host.add(key, "WARN" if hints else "PASS",
             f"describe RTT p50 {p50 * 1000:.1f} ms, p95 {p95 * 1000:.1f} ms; "
             f"post-reset gap {gap:.3f}s (no inference timing), {note}; {rate}", "\n".join(hints))


def _missing_jpeg_decoder(describe: dict) -> bool:
    if "jpeg" not in (describe.get("image_encoding") or {}):
        return False
    from real_world_rl.server.codec import require_jpeg_decoder

    try:
        require_jpeg_decoder()
    except (ImportError, RuntimeError):
        return True
    return False


def _daemon(host, link: LinkConfig, *, grpc_ok: bool) -> bool:
    key = "link.daemon"
    if grpc_ok:
        try:
            with _grpc(link, link.address) as client:
                client.open()
                describe = client.call("robot.describe")
                if not isinstance(describe, dict):
                    raise ValueError("robot.describe returned an invalid reply")
                host.add(key, "PASS",
                         f"{link.address}: robot.describe; daemon_instance_id "
                         f"{'present' if describe.get('daemon_instance_id') else 'absent'}; "
                         f"frame_preprocess_v1={bool(describe.get('frame_preprocess_v1'))}; "
                         f"frame_preprocess_cv2_v1={bool(describe.get('frame_preprocess_cv2_v1'))}; "
                         f"image_encoding={describe.get('image_encoding') or 'raw/unspecified'}")
                jpeg_missing = _missing_jpeg_decoder(describe)
                _quality(host, client, link, describe, jpeg_missing=jpeg_missing)
                return jpeg_missing
        except Exception as error:
            message, hint = _rpc_hint(error, link.token)
            host.add(key, "FAIL", f"{link.address}: {message}", hint)
    else:
        host.add(key, "BLOCKED", "gRPC or msgpack unavailable", "repair link.deps")
    host.add("link.quality", "SKIPPED", "daemon describe unavailable")
    return False


def _policy(host, address: str | None, *, policy_ok: bool) -> None:
    key = "link.policy"
    if not address:
        host.add(key, "SKIPPED", "policy endpoint not configured")
        return
    if not policy_ok:
        host.add(key, "BLOCKED", "openpi client dependencies unavailable", "repair link.deps")
        return
    from real_world_rl.common.openpi_client import OpenPiClient

    start = time.perf_counter()
    try:
        with OpenPiClient(*endpoint(address), timeout_s=4) as client:
            elapsed = time.perf_counter() - start
            fields = len(client.metadata)
    except Exception:
        host.add(key, "FAIL", f"{address}: websocket handshake or metadata read failed",
                 "check the openpi server address, websocket proxy and server log")
    else:
        host.add(key, "PASS", f"{address}: connected and read metadata ({fields} fields) in {elapsed:.2f}s")


def inspect_client(host, link: LinkConfig, name: str) -> None:
    missing = _dependencies(host)
    grpc_ok = not ({"grpc", "msgpack"} & set(missing))
    policy_ok = not ({"openpi-client", "msgpack"} & set(missing))
    _proxy(host, link)
    ssh_ok = _ssh(host, link.ssh)
    inspect_tunnel(host, link, name, ssh_ok=ssh_ok)
    _forward(host, link, ssh_ok=ssh_ok, grpc_ok=grpc_ok)
    jpeg_missing = _daemon(host, link, grpc_ok=grpc_ok)
    _report_dependencies(host, missing, jpeg_missing=jpeg_missing)
    _policy(host, link.policy, policy_ok=policy_ok)
