"""Station-backed operator configuration commands; no robot access unless capture is requested."""
from __future__ import annotations

import json
import os
from pathlib import Path
import sys
import uuid


from .errors import CliError


def add_commands(commands):
    teleop = commands.add_parser("teleop", help="calibrate and select operator direction profiles")
    sub = teleop.add_subparsers(dest="teleop_command", required=True)
    calibrate = sub.add_parser("calibrate")
    calibrate.add_argument("--samples", help="JSON mapping +x,+y,-x,-y to normalized N x 6 samples")
    calibrate.add_argument("--capture-label", choices=("+x", "+y", "-x", "-y", "+z", "-z",
                                                    "+rx", "-rx", "+ry", "-ry", "+rz", "-rz"),
                           help="capture one direction; replace that group in --samples")
    calibrate.add_argument("--append-trial", action="store_true",
                           help="retain existing trials for --capture-label instead of replacing")
    calibrate.add_argument("--profile", required=True)
    calibrate.add_argument("--duration-s", type=float, default=3.)
    calibrate.add_argument("--report-dir")
    calibrate.add_argument("--use", action="store_true", help="save and select this profile in station")
    calibrate.add_argument("--frame", default="table")
    calibrate.add_argument("--output", help="profile YAML; defaults to station teleop_profiles.file")
    calibrate.add_argument("--max-error-deg", type=float, default=25.)
    calibrate.add_argument("--neutral-threshold", type=float, default=.05)
    calibrate.add_argument("--save", action="store_true", help="save validated candidate after confirmation")
    diagnose = sub.add_parser("diagnose", help="input-only by default; explicit --motion runs a supervised session")
    diagnostic_mode = diagnose.add_mutually_exclusive_group()
    diagnostic_mode.add_argument("--input-only", action="store_true")
    diagnostic_mode.add_argument("--motion", choices=("translation", "rotation", "all"))
    diagnose.add_argument("--gain", type=float, default=.2)
    diagnose.add_argument("--mode", choices=("tracker", "serl"), default="tracker")
    diagnose.add_argument("--record", help="new .pkl output required with --motion")
    diagnose.add_argument("--events-file")
    diagnose.set_defaults(num_successes=1, max_episodes=1)
    diagnose.add_argument("--duration-s", type=float, default=5.)
    diagnose.add_argument("--profile")
    diagnose.add_argument("--report-dir")
    run = sub.add_parser("run", help="supervised SpaceMouse recording; labels end episodes, n requests next, q releases control")
    run.add_argument("--mode", choices=("tracker", "serl"), default="tracker")
    run.add_argument("--profile")
    run.add_argument("--events-file", help="append-only JSONL file, or - for piped stdin")
    run.add_argument("--record", required=True, help="new output .pkl path")
    run.add_argument("--num-successes", type=int, default=20)
    run.add_argument("--max-episodes", type=int)
    inspect_stop = sub.add_parser("inspect-stop", help="evaluate a saved stop trace without hardware access")
    inspect_stop.add_argument("--trace", required=True, help="schema-1 JSON stop trace")
    inspect_stop.add_argument("--thresholds", help="JSON object overriding stop threshold values")
    profiles = sub.add_parser("profiles")
    ps = profiles.add_subparsers(dest="action", required=True)
    ps.add_parser("list")
    for name in ("show", "use"):
        ps.add_parser(name).add_argument("--profile", required=True)
    workspace = commands.add_parser("workspace", help="inspect and configure task-frame workspace")
    ws = workspace.add_subparsers(dest="action", required=True)
    ws.add_parser("show")
    validate = ws.add_parser("validate")
    validate.add_argument("--pose", nargs=7, type=float, required=True, metavar="VALUE")
    capture = ws.add_parser("capture")
    capture.add_argument("--name", help="measurement name; required when saving")
    capture.add_argument("--output", help="points YAML, relative to station directory")
    capture.add_argument("--save", action="store_true")
    setter = ws.add_parser("set")
    setter.add_argument("--frame", required=True)
    setter.add_argument("--low", nargs=3, type=float)
    setter.add_argument("--high", nargs=3, type=float)
    setter.add_argument("--from-points", help="propose bounds from captured points YAML")
    setter.add_argument("--orientation-xyzw", nargs=4, type=float)
    setter.add_argument("--max-angle-rad", type=float)
    setter.add_argument("--save", action="store_true")


def station_document(args):
    import yaml
    from robot_infra.discovery.autoconfig import load_station, resolve_station_config_path
    path = resolve_station_config_path(args.station).resolve()
    raw = path.read_bytes()
    document = yaml.safe_load(raw)
    station = load_station(path)
    selected = station.select(args.robot)
    if "robots" in document:
        key = next(key for key, value in station.robots.items() if value is selected)
        node = document["robots"][key]
    else:
        node = document.get("robot", document)
    return path, raw, document, node, selected.config


def confirm_write(args, path, document, expected):
    import difflib
    from real_world_rl.common.yaml_edit import render_changes
    candidate = render_changes(document, expected)
    previous = expected.decode('utf-8') if expected is not None else ''
    difference = ''.join(difflib.unified_diff(previous.splitlines(keepends=True),
        candidate.splitlines(keepends=True), fromfile=str(path), tofile=str(path) + ' (proposed)'))
    print(difference or f"No content changes for {path}", file=sys.stderr)
    if args.yes:
        return
    if not sys.stdin.isatty():
        raise CliError("saving requires --yes without a terminal", code=2)
    print("Save? [y/N] ", file=sys.stderr, end="", flush=True)
    if sys.stdin.readline().strip().lower() != "y":
        raise CliError("cancelled", code=2)


def atomic_save(path, document, expected):
    from real_world_rl.common.yaml_edit import atomic_replace, render_changes
    return atomic_replace(path, render_changes(document, expected), expected)


def execute(args):
    if args.command == "teleop" and args.teleop_command == "inspect-stop":
        import hashlib
        from robot_infra.devices.stop_trace import StopThresholds, evaluate_stop_trace
        try:
            source = Path(args.trace).read_bytes()
            thresholds = StopThresholds(**json.loads(args.thresholds)) if args.thresholds else StopThresholds()
            result = evaluate_stop_trace(json.loads(source), thresholds)
        except (OSError, ValueError, TypeError, KeyError) as error:
            raise CliError(f"invalid stop trace or thresholds: {error}", code=2) from error
        result.update(trace=str(Path(args.trace).resolve()), trace_sha256=hashlib.sha256(source).hexdigest())
        return result, json.dumps(result, indent=2)
    from real_world_rl.teleop.profiles import load_profiles, resolve_profile, validate_profile, AXIS_MAPPING_VERSION
    path, raw, document, node, config = station_document(args)
    result = {"schema_version": 1, "station": str(path), "robot": config.name}
    if args.command == "teleop" and args.teleop_command == "run":
        return run_session(args)
    if args.command == "workspace":
        result.update(workspace_command(args, config, path, raw, document, node))
    elif args.teleop_command == "diagnose":
        if args.motion:
            if not args.record:
                raise CliError("motion diagnostics require --record", code=2, required_fields=["--record"])
            if config.workspace is None:
                raise CliError("motion diagnostics require a reviewed station workspace", code=2)
            if not 0 < args.gain <= 1 or not 0 < args.duration_s <= 300:
                raise CliError("gain must be in (0,1] and duration in (0,300]", code=2)
            return run_session(args)
        from real_world_rl.teleop.input_diagnostics import collect
        resolved = resolve_profile(path, config, args.profile) if config.teleop_profiles else None
        if args.profile and resolved is None:
            raise CliError("--profile requires station teleop_profiles", code=2)
        _, report = collect(config, duration_s=args.duration_s, directory=args.report_dir,
                            rotation=resolved["input_rotation"] if resolved else None,
                            yaw_rad=resolved["profile"]["yaw_rad"] if resolved else 0.)
        result.update(report)
    elif args.teleop_command == "profiles":
        reference = config.teleop_profiles
        if not reference:
            raise CliError("station has no teleop_profiles", code=2)
        file = Path(reference["file"])
        if not file.is_absolute():
            file = path.parent / file
        if args.action == "list":
            result.update(default=reference["default"], profiles=load_profiles(file))
        else:
            result.update(resolve_profile(path, config, args.profile))
            if args.action == "use":
                node["teleop_profiles"] = {**reference, "default": args.profile}
                confirm_write(args, path, document, raw)
                result["saved"] = atomic_save(path, document, raw)
    else:
        from real_world_rl.teleop.calibration import fit_yaw, validate_axes
        acquisition = None
        if args.frame != "base" and (not config.task_frame or args.frame != config.task_frame["name"]):
            raise CliError("calibration frame must match station task_frame", code=2)
        if args.append_trial and not args.capture_label:
            raise CliError("--append-trial requires --capture-label", code=2, required_fields=["--capture-label"])
        if args.samples:
            samples = json.loads(Path(args.samples).read_text())
        elif args.capture_label:
            samples = {}
        else:
            from real_world_rl.teleop.input_diagnostics import collect, LABELS
            if not sys.stdin.isatty():
                raise CliError("missing --samples: live calibration requires a terminal", code=2, required_fields=["--samples"])
            samples, acquisition = collect(config, duration_s=args.duration_s,
                                           directory=args.report_dir, labels=LABELS)
            result.update(acquisition)
        if args.capture_label:
            from real_world_rl.teleop.input_diagnostics import collect, write_json
            from real_world_rl.teleop.calibration import sample_trials
            if not sys.stdin.isatty() and not args.yes:
                raise CliError("unattended labelled capture requires --yes", code=2)
            captured, acquisition = collect(config, duration_s=args.duration_s,
                directory=args.report_dir, labels=[args.capture_label], interactive=sys.stdin.isatty())
            label = args.capture_label
            trials = ([rows.tolist() for rows in sample_trials(samples[label], label)]
                      if args.append_trial and label in samples else [])
            samples[label] = {"trials": trials + [captured[label]]}
            merged = Path(acquisition['report_directory']) / 'merged-samples.json'
            write_json(merged, samples)
            print(f"Merged calibration samples: {merged}", file=sys.stderr)
            result.update(acquisition, samples_file=str(merged))
            required = [label for label in ('+x', '+y', '-x', '-y') if label not in samples]
            if required:
                result.update(status='NEEDS_SAMPLES', required_fields=required, saved=None)
                return result, json.dumps(result, indent=2, allow_nan=False)
        fitted = fit_yaw(samples, max_error_deg=args.max_error_deg)
        fitted.update(validate_axes(samples, fitted["yaw_rad"], max_error_deg=args.max_error_deg))
        profile = validate_profile({"frame": args.frame, "yaw_rad": fitted["yaw_rad"],
            "neutral_threshold": args.neutral_threshold, "axis_mapping_version": AXIS_MAPPING_VERSION,
            "horizontal_validated": True, "rotation_validated": fitted["rotation_validated"],
            "evidence": fitted})
        if acquisition is not None:
            profile["device_identity"] = acquisition["device_identity"]
        result.update(profile=args.profile, candidate=profile, saved=None)
        if args.save or args.use:
            reference = config.teleop_profiles or {}
            output = args.output or reference.get("file")
            if not output:
                raise CliError("saving a first profile requires --output", code=2, required_fields=["--output"])
            file = Path(output)
            if not file.is_absolute():
                file = path.parent / file
            previous = file.read_bytes() if file.exists() else None
            if previous is not None:
                import yaml
                load_profiles(file)  # Keep validation on the existing safe-load path.
                payload = yaml.safe_load(previous)
            else:
                payload = {"schema_version": 1, "profiles": {}}
            payload["profiles"][args.profile] = profile
            confirm_write(args, file, payload, previous)
            result["saved"] = atomic_save(file, payload, previous)
            result["station_reference"] = {"file": str(file), "default": args.profile}
            if args.use:
                node["teleop_profiles"] = {"file": os.path.relpath(file, path.parent), "default": args.profile}
                confirm_write(args, path, document, raw)
                result["station_saved"] = atomic_save(path, document, raw)
            else:
                result["next_action"] = "Use --use to also select this profile in the station"
    return result, json.dumps(result, indent=2, allow_nan=False)


def workspace_command(args, config, path, raw, document, node):
    from robot_infra.devices.workspace import Workspace, task_transform
    from real_world_rl.teleop.workspace_points import point_set, candidate_bounds
    import yaml
    if args.action == "capture":
        if args.save and (not args.name or not args.output):
            raise CliError("saving measurements requires --name and --output", code=2,
                           required_fields=[flag for flag, value in (("--name", args.name), ("--output", args.output)) if not value])
        file = Path(args.output) if args.output else None
        if file is not None and not file.is_absolute():
            file = path.parent / file
        previous = file.read_bytes() if file is not None and file.exists() else None
        points, _ = point_set(yaml.safe_load(previous) if previous else None, config.task_frame)
        from .connect import open_robot
        with open_robot(args) as connection:
            pose = connection.arm.read_state()["tcp_pose"].tolist()
        transform = task_transform(config.task_frame)
        import numpy as np
        position = transform[:3, :3].T @ (np.asarray(pose[:3]) - transform[:3, 3])
        result = {"base_pose": pose, "task_position": position.tolist(), "units": "m; quaternion xyzw",
                "frame": config.task_frame["name"] if config.task_frame else "base",
                "note": "Measurement only; not an accepted workspace boundary"}
        if args.save:
            from datetime import datetime, timezone
            points['points'][args.name] = {'base_pose': pose, 'captured_at': datetime.now(timezone.utc).isoformat()}
            point_set(points, config.task_frame)
            confirm_write(args, file, points, previous)
            result['saved'] = atomic_save(file, points, previous)
        return result
    if args.action == "show":
        result = {"task_frame": config.task_frame, "workspace": config.workspace,
                  "base_safety_box": config.safety_box,
                  "units": {"position": "m", "angle": "rad", "quaternion": "xyzw"},
                  "source": {"station": str(path), "robot": config.name,
                             "frame_calibration": (config.task_frame or {}).get("source")},
                  "scope": "configured constraints only; no live daemon or physical calibration verification",
                  "combination": "intersection of base safety box and oriented workspace; orientation limits also apply"}
        if config.workspace is not None:
            from itertools import product
            import numpy as np
            geometry = Workspace(config.workspace, config.task_frame)
            corners = np.asarray(list(product(*zip(geometry.low, geometry.high))))
            result["workspace_corners_base_m"] = (corners @ geometry.rotation.T + geometry.origin).tolist()
            result["workspace_corners_frame_m"] = corners.tolist()
            result["corner_note"] = "oriented box vertices, not replacement axis-aligned bounds; not proof of collision-free interior"
        return result
    if args.action == "validate":
        if config.workspace is None:
            raise CliError("no workspace configured", code=2)
        from robot_infra.devices.safety import ArmSafety
        safety = ArmSafety.from_config(config)
        from robot_infra.devices.errors import SafetyViolation
        try:
            safety.require_pose_inside(args.pose)
        except SafetyViolation as error:
            return {"valid": False, "pose": args.pose, "violations": [error.message]}
        return {"valid": True, "pose": args.pose, "violations": []}
    if args.from_points:
        if args.low is not None or args.high is not None:
            raise CliError("--from-points cannot be combined with --low/--high", code=2)
        file = Path(args.from_points)
        if not file.is_absolute():
            file = path.parent / file
        candidate = candidate_bounds(yaml.safe_load(file.read_text()), config.task_frame)
        if candidate['frame'] != args.frame:
            raise CliError("--frame must match captured points", code=2)
    else:
        if args.low is None or args.high is None:
            raise CliError("provide --low and --high, or --from-points", code=2)
        candidate = {"frame": args.frame, "low": args.low, "high": args.high}
    if args.orientation_xyzw is not None:
        candidate["orientation_xyzw"] = args.orientation_xyzw
    if args.max_angle_rad is not None:
        candidate["max_angle_rad"] = args.max_angle_rad
    Workspace(candidate, config.task_frame)
    result = {"candidate": candidate, "saved": None,
              "note": "Candidate only; captured extrema do not certify the enclosed volume is safe"}
    if args.save:
        node["workspace"] = candidate
        confirm_write(args, path, document, raw)
        result["saved"] = atomic_save(path, document, raw)
    return result


def run_session(args):
    """Prepare a normal recorder configuration, then supervise its owning child."""
    import dataclasses
    import subprocess
    import yaml
    from .commands import confirm
    from .connect import client_address, probe_daemon
    from .preset_edit import Preset
    from robot_infra.discovery.autoconfig import load_station
    from real_world_rl.teleop.profiles import resolve_profile

    if args.preset is None:
        raise CliError("teleop run requires --preset with home and task limits", code=2, required_fields=["--preset"])
    if os.environ.get("ROBOT_INFRA_HEARTBEAT_FD"):
        raise CliError("teleop run starts its own supervisor; invoke it directly", code=2)
    path, _, _, _, config = station_document(args)
    selected = load_station(path).select(args.robot)
    resolved = resolve_profile(path, config, args.profile)
    output = Path(args.record).resolve()
    if output.suffix != ".pkl" or output.exists():
        raise CliError("--record must be a new .pkl path", code=2)
    if args.num_successes < 1 or (args.max_episodes is not None and args.max_episodes < 1):
        raise CliError("episode limits must be positive", code=2)
    preset = Preset.load(args.preset)
    env = dict(preset.env)
    for name in ("stop_trace_path", "diagnostics_path", "session_events_path"):
        env.pop(name, None)
    env.update(task_frame=config.task_frame, workspace=config.workspace)
    if selected.daemon_declared and not args.direct:
        env.update(robot=None, remote=client_address(selected.bind), token=selected.token)
    else:
        if selected.daemon_declared and probe_daemon(client_address(selected.bind)):
            raise CliError("daemon owns arm; omit --direct", code=2)
        robot = {"type": config.TYPE, **dataclasses.asdict(config)}
        if args.mock:
            from robot_infra.doctor.checks import mocked_config
            robot = {"type": config.TYPE, **dataclasses.asdict(mocked_config(config))}
        env.update(robot=robot, remote=None, token=None)
    settings = {**preset.document, "real_robot": env, "env_backend": "real_robot",
                "teleop_device": "spacemouse",
                "teleop_event_device": "jsonl" if args.events_file else "stdin",
                "teleop_events_file": (str(Path(args.events_file).resolve())
                                       if args.events_file and args.events_file != "-" else "-"),
                "spacemouse_mode": args.mode, "teleop_options_yaml": str(path),
                "teleop_robot": args.robot, "teleop_profile": resolved["name"],
                "episode_end_behavior": "hold", "neutral_threshold": resolved["profile"]["neutral_threshold"],
                "num_successes": args.num_successes, "max_episodes": args.max_episodes,
                "output_path": str(output)}
    if args.mode == "serl":
        env["cartesian_semantics"] = "serl"
    if getattr(args, "motion", None):
        import math
        settings.update(teleop_diagnostic_axes=args.motion, teleop_diagnostic_gain=args.gain,
                        max_episode_steps=max(1, math.ceil(args.duration_s * settings.get("control_hz", env.get("control_hz", 10.)))))
    # Resolve all typed configuration before asking for hardware execution.
    output.parent.mkdir(parents=True, exist_ok=True)
    report = output.parent / (output.stem + ".session-" + uuid.uuid4().hex)
    report.mkdir(mode=0o700)
    child_environment = {**os.environ,
        "RWRL_STOP_TRACE_PATH": str(report / "stop-trace.json"),
        "RWRL_DIAGNOSTICS_PATH": str(report / "diagnostics.jsonl"),
        "RWRL_SESSION_EVENTS_PATH": str(report / "session-events.jsonl")}
    config_path = report / "resolved-config.yaml"
    config_path.write_text(yaml.safe_dump(settings, sort_keys=False))
    os.chmod(config_path, 0o600)
    from real_world_rl.demos.record import parse_record_args
    parsed = parse_record_args(["--config", str(config_path)])
    from real_world_rl.training.env_building import resolve_teleop_input
    resolve_teleop_input(parsed)
    settings["teleop_input"] = parsed.teleop_input
    config_path.write_text(yaml.safe_dump(settings, sort_keys=False))
    confirm(args, f"SpaceMouse {args.mode} session on {config.name}; task reset may home; "
                  "s/f ends episode and holds; n resets next; q exits", 0., "session")
    from real_world_rl.common.provenance import runtime_provenance
    from real_world_rl.teleop.input_diagnostics import write_json
    write_json(report / "provenance.json", runtime_provenance(config=settings, files={
        "station": path, "preset": args.preset, "profiles": resolved["file"],
        "resolved_config": config_path}))
    command = [sys.executable, "-m", "robot_infra.supervise", "--", sys.executable,
               "-m", "real_world_rl.demos", "--config", str(config_path)]
    completed = subprocess.run(command, stdout=sys.stderr if args.json else None, check=False, env=child_environment)
    import hashlib
    result = {"schema_version": 1, "report_directory": str(report), "returncode": completed.returncode,
              "resolved_config_sha256": hashlib.sha256(config_path.read_bytes()).hexdigest(),
              "profile": resolved["name"], "output": str(output)}
    assessments = sorted(report.glob("stop-trace-*.assessment.json"))
    if assessments:
        result["stop_assessment"] = json.loads(assessments[-1].read_text())
    else:
        result["stop_assessment"] = {"status": "BLOCKED", "reason": "no normal-stop evidence was exported"}
    (report / "summary.json").write_text(json.dumps(result, indent=2))
    if completed.returncode:
        raise CliError(f"session exited {completed.returncode}; report: {report}")
    return result, json.dumps(result, indent=2)
