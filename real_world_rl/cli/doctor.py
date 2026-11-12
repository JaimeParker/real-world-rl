"""Robot-host diagnostics: inspect, preview, confirm, repair, then inspect again."""

from __future__ import annotations

import dataclasses
import json
import os
import re
import shlex
import io
import sys
from pathlib import Path

from robot_infra.cli.command import affirmative
from robot_infra.cli.render import (
    BOLD,
    BOLD_GREEN,
    BOLD_RED,
    CYAN,
    pad_then_paint,
    paint,
    status_style,
    supports_color,
)

from .command import Result
from .errors import SUCCESS_EXIT_CODE
from .doctor_host import Host
from .doctor_platform import find_checkout
from .doctor_preset import preset_station_checks
from .doctor_repair import Action, Report, execute, exit_status, payload, redact

# Device-layer hints name `robot-infra doctor --fix`; `rwrl doctor --fix` never recovers or reopens devices.
_DEVICE_FIX_HINTS = (
    (re.compile(r"(?:run with )?--fix \(recover\)"), "run rwrl recover (or robot-infra doctor --fix)"),
    (re.compile(r"(?:run with )?--fix to reopen"),
     "rerun rwrl status after reconnecting the camera (or robot-infra doctor --fix)"),
)


def rwrl_device_hint(hint: str) -> str:
    if not hint:
        return hint
    for pattern, replacement in _DEVICE_FIX_HINTS:
        hint = pattern.sub(replacement, hint)
    return hint


_STATUS_WIDTH = 14
_UNRESOLVED_STATUSES = ("FAIL", "BLOCKED", "PENDING_ACTION")


def status_line(status: str, item_id: str, text: str, *, color: bool, indent: str = "") -> str:
    return f"{indent}{pad_then_paint(status, _STATUS_WIDTH, enabled=color)} {item_id}: {text}"


def hint_lines(hint: str, *, indent: str = "    ") -> str:
    """``hint:``-labelled; continuation lines line up under the first line's text."""
    label = indent + "hint: "
    first, *rest = hint.splitlines() or [""]
    return "\n".join([label + first, *(" " * len(label) + line for line in rest)])


def _last_line(text: str) -> str:
    # Multi-line messages are captured command output, whose last line names the error.
    lines = [line for line in text.splitlines() if line.strip()]
    return lines[-1].strip() if lines else ""


def needs_attention(findings, actions, *, color: bool) -> str:
    """Every FAIL/BLOCKED/PENDING_ACTION check and repair with a one-line hint; empty when none."""
    rows = [(row.status, row.id, row.message, row.hint) for row in findings if row.status in _UNRESOLVED_STATUSES]
    rows += [
        (action.status, action.id, action.detail or action.title, "")
        for action in actions
        if action.status in _UNRESOLVED_STATUSES
    ]
    if not rows:
        return ""
    counts = ", ".join(
        f"{count} {status}"
        for status in _UNRESOLVED_STATUSES
        if (count := sum(row[0] == status for row in rows))
    )
    lines = [paint("Needs attention:", BOLD, enabled=color) + " " + counts]
    for status, item_id, text, hint in rows:
        lines.append(status_line(status, item_id, _last_line(text), color=color, indent="  "))
        if hint.strip():
            lines.append("    hint: " + hint.strip().splitlines()[0])
    return "\n".join(lines)


def station_configured(args) -> bool:
    """A station is given (``--station``, ``$ROBOT_INFRA_CONFIG``) or found in a station scope."""
    config_home = Path(os.environ.get("XDG_CONFIG_HOME", Path.home() / ".config"))
    return bool(
        args.config
        or os.environ.get("ROBOT_INFRA_CONFIG")
        or (Path.cwd() / "local/station.yaml").is_file()
        or (config_home / "robot_infra/station.yaml").is_file()
    )


def spacemouse_table_inputs(args):
    """``(station directory, spacemouse teleop options)`` for the host-level SpaceMouse device
    table, so USB classification and udev rules use the device layer's table: the station that
    ``load_targets`` checks and the options of its first selected spacemouse teleop. ``(None,
    None)`` without a station, or when it cannot be loaded (``load_targets`` reports why)."""
    if not station_configured(args):
        return None, None
    try:
        from real_world_rl.station import load_station
        from robot_infra.discovery.autoconfig import resolve_station_config_path

        path = resolve_station_config_path(args.config)
        station = load_station(path)
    except Exception:
        return None, None
    for name in [args.robot] if args.robot else list(station.robots):
        robot = station.robots.get(name)
        teleop = robot.config.teleop if robot is not None else None
        if teleop is not None and teleop.backend == "spacemouse":
            return Path(path).resolve().parent, dict(teleop.options)
    return Path(path).resolve().parent, None


def load_targets(args):
    # Host-only diagnosis must still work when hardware/core SDK imports are broken.
    if not (station_configured(args) or args.preset):
        if args.robot:
            raise ValueError("--robot requires a station configuration")
        return [], set()
    from real_world_rl.station import load_station
    import yaml
    from robot_infra.devices import Robot
    from robot_infra.discovery.autoconfig import (
        fill_from_env,
        resolve_station_config_path,
    )

    targets = []
    expected = set()
    path = None
    try:
        path = resolve_station_config_path(args.config)
    except FileNotFoundError:
        if args.config or os.environ.get("ROBOT_INFRA_CONFIG"):
            raise
    if path is not None:
        station = load_station(path)
        for name in [args.robot] if args.robot else list(station.robots):
            selected = station.select(name)
            targets.append(
                (
                    name,
                    fill_from_env(selected.config),
                    str(path),
                    selected.daemon_declared,
                    selected.bind,
                )
            )
    elif args.robot:
        raise ValueError("--robot requires a station configuration")
    if args.preset:
        values = yaml.safe_load(Path(args.preset).read_text(encoding="utf-8")) or {}
        if not isinstance(values, dict):
            raise ValueError("preset must be a YAML mapping")
        mode = values.get("teleop_device")
        if mode == "spacemouse":
            expected.add("spacemouse")
        if mode in ("gello", "leader_arm", "leader_cartesian"):
            expected.add("gello")
        if values.get("teleop_event_device") == "keyboard":
            expected.add("keyboard")
        robot = (values.get("real_robot") or {}).get("robot")
        if not targets and robot:
            if isinstance(robot, str):
                station = load_station(robot)
                targets.extend(
                    (
                        name,
                        fill_from_env(entry.config),
                        robot,
                        entry.daemon_declared,
                        entry.bind,
                    )
                    for name, entry in station.robots.items()
                )
            elif isinstance(robot, dict):
                targets.append(
                    ("preset", Robot.from_dict(robot).config, args.preset, False, "")
                )
        options = values.get("teleop_options_yaml")
        if "gello" in expected and options and str(options).lower() != "none":
            leader = yaml.safe_load(Path(options).read_text(encoding="utf-8")) or {}
            if leader.get("robot_yaml"):
                station = load_station(leader["robot_yaml"])
                for name, entry in station.robots.items():
                    if entry.config.teleop:
                        # Read the commissioned leader part only. Never open a second physical follower.
                        config = dataclasses.replace(
                            entry.config,
                            arm_backend="fake",
                            gripper=None,
                            cameras=[],
                            robot_ip=None,
                        )
                        targets.append(
                            ("leader:" + name, config, str(options), False, "")
                        )
    return targets, expected


def inspect(args, host: Host):
    try:
        targets, extra = load_targets(args)
        preset_checks = preset_station_checks(args)
    except ImportError as error:
        host.add(
            "configuration.dependencies",
            "BLOCKED",
            "configuration/device imports unavailable: " + str(error),
            "repair base dependencies, then rerun doctor",
        )
        targets, extra, preset_checks = [], set(), []
    host.inspect_system(mock=args.mock)
    host.expected.update(extra)
    for name, config, path, daemon, bind in targets:
        if not args.mock:
            from robot_infra.doctor.deps import required_extras

            host.expected.update(required_extras(config, serves_daemon=daemon))
    if not args.mock:
        if host.capabilities.linux:
            host.inspect_gello([entry[1] for entry in targets])
            host.inspect_usb()
            host.inspect_network([entry[1] for entry in targets])
            host.inspect_realtime()
        if not host.capabilities.device_probes and host.expected:
            host.add(
                "devices.platform",
                "BLOCKED",
                "device/OS checks require a native Linux robot host",
            )
        host.inspect_dependencies()
    results_by_robot = {}
    mismatches = []
    owners = [] if args.mock or not host.capabilities.device_probes else host.busy()
    for name, config, path, daemon, bind in targets:
        if not args.mock and not host.capabilities.device_probes:
            host.add(
                name + ".device",
                "BLOCKED",
                "device probes require native Linux outside containers and WSL",
            )
            continue
        if daemon and not args.mock:
            from .connect import client_address, probe_daemon

            if probe_daemon(client_address(bind)):
                owners = [*owners, f"daemon owns {name} at {bind}"]
        if owners:
            host.add(
                name + ".device",
                "BLOCKED",
                "device inspection requires an idle station: " + ", ".join(owners),
                "finish the controlling program and rerun; doctor never stops it",
            )
            continue
        request = dict(
            config=config.to_dict(), mock=args.mock, path=path, daemon=daemon
        )
        code, output = host.report.run(
            [sys.executable, "-m", "real_world_rl.cli.doctor_probe"],
            input_text=json.dumps(request),
            timeout=45,
            stdout_only=True,
        )
        if code:
            host.add(
                name + ".device",
                "FAIL",
                output.strip()[-1500:],
                "inspect SDK/permissions; device probe failed or timed out",
            )
            continue
        try:
            rows = json.loads(output)
        except (ValueError, TypeError):
            host.add(name + ".device", "FAIL", "device probe returned invalid JSON")
            continue
        results_by_robot[name] = rows
        for row in rows:
            # The old doctor checks every hidraw node; the new USB/backend-specific check supersedes it.
            if row["name"] == "os:hid" and not args.mock:
                continue
            status = (
                "SKIPPED"
                if row.get("skipped")
                else ("WARN" if row.get("warning") else "PASS")
                if row["ok"]
                else "FAIL"
            )
            if "claimed by another process" in row.get("message", ""):
                status = "BLOCKED"
            host.add(
                name + "." + row["name"],
                status,
                row["message"],
                rwrl_device_hint(row.get("hint", "")),
            )
            details = row.get("details", {})
            if (
                row["name"] == "handshake"
                and not row["ok"]
                and "libfranka_server_version" in details
            ):
                mismatches.append((name, details["libfranka_server_version"]))
    for check in preset_checks:
        host.add(*check)
    if mismatches:
        versions = {version for _, version in mismatches}
        if len(versions) != 1:
            host.add(
                "sdk.franky.conflict",
                "BLOCKED",
                f"robots require different server versions: {mismatches}",
                "use separate project environments; doctor will not select one robot's SDK",
            )
        else:
            from robot_infra.doctor.deps import franky_wheel_install_argv

            version = next(iter(versions))
            host.propose(
                Action(
                    "sdk:franky",
                    "sdk.franky",
                    f"Install Franky wheel for handshake-confirmed server version {version}",
                    franky_wheel_install_argv(version),
                    python=True,
                    exclusive=True,
                )
            )
    # When no station was commissioned, USB-only sources can still be inspected independently.
    if not targets:
        host.add(
            "station",
            "SKIPPED",
            "no station configured; arm handshake not performed",
            "run rwrl setup when ready; software/USB success is not robot acceptance",
        )
    for kind in ("spacemouse", "realsense"):
        if (
            args.mock
            or not host.capabilities.device_probes
            or kind not in host.expected
            or any(
                (config.teleop and config.teleop.backend == kind)
                or (
                    kind == "realsense"
                    and any(c.backend == kind for c in config.cameras)
                )
                for _, config, *_ in targets
            )
        ):
            continue
        if owners:
            host.add(
                "read." + kind,
                "BLOCKED",
                "device read requires an idle station: " + ", ".join(owners),
                "finish the device owner and rerun; doctor never takes over control",
            )
            continue
        if any(
            row.status != "PASS" and row.id in ("sdk." + kind, "usb." + kind)
            for row in host.findings
        ):
            continue
        if not any(d["kind"] == kind for d in host.usb):
            continue
        config = {
            "type": "franka",
            "name": "doctor-usb",
            "arm_backend": "fake",
            "gripper": None,
            "cameras": [],
        }
        if kind == "spacemouse":
            config["teleop"] = {"backend": kind, "name": "doctor-space"}
        else:
            config["cameras"] = [
                {"name": "doctor-camera", "backend": kind, "serial": d["serial"]}
                for d in host.usb
                if d["kind"] == kind
            ]
        code, output = host.report.run(
            [sys.executable, "-m", "real_world_rl.cli.doctor_probe"],
            input_text=json.dumps(dict(config=config)),
            timeout=20,
            stdout_only=True,
        )
        try:
            rows = json.loads(output) if not code else []
            relevant = [
                row
                for row in rows
                if row["name"].startswith(("feedback:teleop", "feedback:camera"))
            ]
            ok = bool(relevant) and all(row["ok"] for row in relevant)
        except (ValueError, TypeError):
            ok = False
        host.add(
            "read." + kind,
            "PASS" if ok else "FAIL",
            "device read succeeded" if ok else "USB SDK read failed; see commands.log",
            "enumeration alone is not evidence of successful reading",
        )
    if not args.mock:
        for kind, title in (
            ("franky", "Franka/Hand"),
            ("spacemouse", "SpaceMouse"),
            ("gello", "GELLO"),
            ("realsense", "RealSense"),
        ):
            if kind not in host.expected:
                host.add(
                    "coverage." + kind,
                    "SKIPPED",
                    title + " not configured or identified; not hardware-validated",
                )
    # udev changes always require reload, but never trigger all devices during a run.
    file_ids = [
        item.id for item in host.actions if item.target.startswith("/etc/udev/")
    ]
    for file_id in file_ids:
        host.propose(
            Action(
                "reload:" + file_id,
                "udev",
                "Reload successfully installed udev rule",
                ["udevadm", "control", "--reload-rules"],
                dependencies=[file_id],
                privileged=True,
            )
        )
    # A freshly installed module must be checked in another interpreter, not this process.
    return results_by_robot


def run(ctx, usage_error):
    """Execute diagnostics using the shared CLI context; return a renderable report."""
    args = ctx.args
    # Internal loader compatibility; the public station option lives in Context.
    args.config = args.station
    root = find_checkout()
    try:
        report = Report(root, directory=args.report_dir)
    except OSError as error:
        raise usage_error(
            f"cannot create private doctor report: {error}; use --report-dir with a writable directory"
        )
    station_dir, spacemouse_options = spacemouse_table_inputs(args)
    try:
        host = Host(root, report, station_dir=station_dir, spacemouse_options=spacemouse_options)
        results = inspect(args, host)
    except Exception as error:
        report.save("error.json", {"error": str(error)})
        raise usage_error(redact(str(error)), str(report.path))
    actions = host.actions
    before = payload(host.findings, actions, report, results=results)
    report.save("before.json", before)
    report.save("plan.json", before["repairs"])
    stream = ctx.stderr if args.json else ctx.stdout
    stream_color = supports_color(stream)
    if not args.json:
        for row in host.findings:
            print(status_line(row.status, row.id, row.message, color=stream_color), file=stream)
            if row.hint and row.status != "PASS":
                print(hint_lines(row.hint), file=stream)
    for action in actions:
        if args.install_sdk and not args.fix and not action.id.startswith("sdk:"):
            continue
        print(f"{paint('PLAN', CYAN, enabled=stream_color)} {action.id}: {action.title}", file=stream)
        if action.status == "BLOCKED":
            blocked = paint("BLOCKED", status_style("BLOCKED"), enabled=stream_color)
            print(f"  {blocked}: {action.detail}", file=stream)
        if action.argv:
            print("  " + shlex.join(action.argv), file=stream)
        if action.target:
            print("  " + action.target + "\n" + action.content, file=stream)
        if action.pending:
            print("  Afterward: " + action.pending, file=stream)
    repairing = args.fix or args.install_sdk
    selected = [a for a in actions if args.fix or a.id.startswith("sdk:")]
    confirmed = False
    if repairing and selected:
        if args.yes:
            confirmed = True
        elif sys.stdin.isatty():
            print("Apply this plan? [y/N] ", end="", file=stream, flush=True)
            try:
                confirmed = affirmative(input())
            except EOFError:
                pass
        if not confirmed:
            for action in selected:
                action.status, action.detail = (
                    "BLOCKED",
                    "plan not confirmed; no repairs executed",
                )
        else:
            execute(
                actions,
                report,
                allowed=host.allowed and not args.mock,
                busy=host.busy,
                assume_yes=args.yes,
                sdk_only=args.install_sdk and not args.fix,
                capabilities=host.capabilities,
            )
            after_host = Host(root, report, station_dir=station_dir, spacemouse_options=spacemouse_options)
            try:
                results = inspect(args, after_host)
                host = after_host
            except Exception as error:
                host.add("recheck", "FAIL", str(error))
    result = payload(
        host.findings,
        actions,
        report,
        results=results,
        confirmed=confirmed,
        remaining_plan=[
            dataclasses.asdict(a)
            for a in host.actions
            if a.id
            not in {b.id for b in actions if b.status in ("PASS", "PENDING_ACTION")}
        ],
    )
    result = redact(result)
    report.save("report.json", result)
    code = exit_status(host.findings, actions)
    output = io.StringIO()
    if not args.json:
        # The buffer itself is no terminal; it is printed to ctx.stdout once the command returns.
        output_color = supports_color(ctx.stdout)
        print(f"{paint('Report:', BOLD, enabled=output_color)} {report.path}", file=output)
        if confirmed:
            for action in actions:
                print(status_line(action.status, action.id, action.detail, color=output_color), file=output)
            for row in host.findings:
                print(status_line(row.status, row.id, row.message, color=output_color), file=output)
        attention = needs_attention(host.findings, actions, color=output_color)
        if attention:
            print(attention, file=output)
        succeeded = code == SUCCESS_EXIT_CODE
        print(
            paint(
                "Result: "
                + (
                    "no blocking findings; review SKIPPED/WARN coverage"
                    if succeeded
                    else "unresolved failures / blocked checks / pending user actions"
                ),
                BOLD_GREEN if succeeded else BOLD_RED,
                enabled=output_color,
            ),
            file=output,
        )
    return Result(result, output.getvalue().removesuffix("\n"), code, ensure_ascii=False)


def main(argv=None):
    """Transitional Python/module entry point; delegate to the unified rwrl doctor."""
    from . import main as rwrl_main
    return rwrl_main(["doctor", *(sys.argv[1:] if argv is None else argv)])


if __name__ == "__main__":
    raise SystemExit(main())
