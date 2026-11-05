"""Robot-host diagnostics: inspect, preview, confirm, repair, then inspect again."""

from __future__ import annotations

import argparse
import dataclasses
import json
import os
import shlex
import sys
from pathlib import Path

from .doctor_host import Host
from .doctor_platform import find_checkout
from .doctor_repair import Action, Report, execute, exit_status, payload, redact


class _Parser(argparse.ArgumentParser):
    json_output = False
    report_directory = None

    def error(self, message):
        if self.json_output:
            print(
                json.dumps(
                    {
                        "schema_version": 1,
                        "error": redact(message),
                        "report_directory": self.report_directory,
                    }
                )
            )
        super().error(redact(message))


def load_targets(args):
    # Host-only diagnosis must still work when hardware/core SDK imports are broken.
    config_home = Path(os.environ.get("XDG_CONFIG_HOME", Path.home() / ".config"))
    if not (
        args.config
        or args.preset
        or os.environ.get("ROBOT_INFRA_CONFIG")
        or (Path.cwd() / "local/station.yaml").is_file()
        or (config_home / "robot_infra/station.yaml").is_file()
    ):
        if args.robot:
            raise ValueError("--robot requires a station configuration")
        return [], set()
    import yaml
    from robot_infra.devices import Robot
    from robot_infra.discovery.autoconfig import (
        fill_from_env,
        load_station,
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
    except ImportError as error:
        host.add(
            "configuration.dependencies",
            "BLOCKED",
            "configuration/device imports unavailable: " + str(error),
            "repair base dependencies, then rerun doctor",
        )
        targets, extra = [], set()
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
                name + "." + row["name"], status, row["message"], row.get("hint", "")
            )
            details = row.get("details", {})
            if (
                row["name"] == "handshake"
                and not row["ok"]
                and "libfranka_server_version" in details
            ):
                mismatches.append((name, details["libfranka_server_version"]))
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
            or owners
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


def main(argv=None):
    parser = _Parser(prog="rwrl doctor", description=__doc__, allow_abbrev=False)
    parser.json_output = "--json" in (sys.argv[1:] if argv is None else argv)
    parser.add_argument("--config")
    parser.add_argument("--robot")
    parser.add_argument("--preset")
    parser.add_argument("--mock", action="store_true")
    parser.add_argument("--fix", action="store_true")
    parser.add_argument(
        "--install-sdk",
        action="store_true",
        help="repair only a handshake-confirmed Franky mismatch",
    )
    parser.add_argument("--yes", action="store_true")
    parser.add_argument("--json", action="store_true")
    parser.add_argument(
        "--report-dir",
        type=Path,
        help="report base directory (overrides RWRL_DOCTOR_REPORT_DIR)",
    )
    args = parser.parse_args(argv)
    if args.yes and not (args.fix or args.install_sdk):
        parser.error("--yes requires --fix or --install-sdk")
    root = find_checkout()
    try:
        report = Report(root, directory=args.report_dir)
    except OSError as error:
        parser.error(
            f"cannot create private doctor report: {error}; use --report-dir with a writable directory"
        )
    parser.report_directory = str(report.path)
    try:
        host = Host(root, report)
        results = inspect(args, host)
    except Exception as error:
        report.save("error.json", {"error": str(error)})
        parser.error(redact(str(error)))
    actions = host.actions
    before = payload(host.findings, actions, report, results=results)
    report.save("before.json", before)
    report.save("plan.json", before["repairs"])
    stream = sys.stderr if args.json else sys.stdout
    if not args.json:
        for row in host.findings:
            print(f"{row.status:14} {row.id}: {row.message}", file=stream)
            if row.hint and row.status != "PASS":
                print("  " + row.hint, file=stream)
    for action in actions:
        if args.install_sdk and not args.fix and not action.id.startswith("sdk:"):
            continue
        print(f"PLAN {action.id}: {action.title}", file=stream)
        if action.status == "BLOCKED":
            print("  BLOCKED: " + action.detail, file=stream)
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
                confirmed = input().strip().lower() in ("y", "yes")
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
            after_host = Host(root, report)
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
    if args.json:
        print(json.dumps(result, ensure_ascii=False))
    else:
        print(f"Report: {report.path}")
        if confirmed:
            for action in actions:
                print(f"{action.status:14} {action.id}: {action.detail}")
            for row in host.findings:
                print(f"{row.status:14} {row.id}: {row.message}")
        print(
            "Result: "
            + (
                "no blocking findings; review SKIPPED/WARN coverage"
                if code == 0
                else "unresolved failures / blocked checks / pending user actions"
            )
        )
    return code
