"""Prepare recording configuration and launch the supervised owning child."""
from __future__ import annotations

import os
import sys
import uuid
import json
from pathlib import Path

from .command import Result
from .errors import CliError, USAGE_EXIT_CODE
from .station_edit import station_document


def run_session(args, ctx):
    """Prepare a normal recorder configuration, then supervise its owning child."""
    import dataclasses
    import subprocess
    import yaml
    from .connect import client_address, probe_daemon
    from .preset_edit import Preset
    from real_world_rl.teleop.profiles import resolve_profile

    if args.preset is None:
        raise CliError("teleop run requires --preset with home and task limits", code=USAGE_EXIT_CODE, required_fields=["--preset"])
    if os.environ.get("ROBOT_INFRA_HEARTBEAT_FD"):
        raise CliError("teleop run starts its own supervisor; invoke it directly", code=USAGE_EXIT_CODE)
    path, _, _, _, selected = station_document(args)
    config = selected.config
    if args.diagnose:
        if config.workspace is None:
            raise CliError("motion diagnostics require a reviewed station workspace", code=USAGE_EXIT_CODE)
        if not 0 < args.gain <= 1 or not 0 < args.duration_s <= 300:
            raise CliError("gain must be in (0,1] and duration in (0,300]", code=USAGE_EXIT_CODE)
    if args.num_successes is None:
        args.num_successes = 1 if args.diagnose else 20
    if args.max_episodes is None and args.diagnose:
        args.max_episodes = 1
    resolved = resolve_profile(path, selected, args.profile)
    output = Path(args.record).resolve()
    if output.suffix != ".pkl" or output.exists():
        raise CliError("--record must be a new .pkl path", code=USAGE_EXIT_CODE)
    if args.num_successes < 1 or (args.max_episodes is not None and args.max_episodes < 1):
        raise CliError("episode limits must be positive", code=USAGE_EXIT_CODE)
    preset = Preset.load(args.preset)
    env = dict(preset.env)
    for name in ("stop_trace_path", "diagnostics_path", "session_events_path"):
        env.pop(name, None)
    env.update(task_frame=config.task_frame, workspace=config.workspace)
    if selected.daemon_declared and not args.direct:
        env.update(robot=None, remote=client_address(selected.bind), token=selected.token)
    else:
        if selected.daemon_declared and probe_daemon(client_address(selected.bind)):
            raise CliError("daemon owns arm; omit --direct", code=USAGE_EXIT_CODE)
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
    if args.diagnose:
        import math
        settings.update(teleop_diagnostic_axes=args.diagnose, teleop_diagnostic_gain=args.gain,
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
    ctx.confirm("proceed? [y/N] ", detail=(
        f"target SpaceMouse {args.mode} session on {config.name}; task reset may home; "
        "s/f ends episode and holds; n resets next; q exits; max delta 0.0000 session"))
    from real_world_rl.common.provenance import runtime_provenance
    from real_world_rl.teleop.input_diagnostics import write_json
    write_json(report / "provenance.json", runtime_provenance(config=settings, files={
        "station": path, "preset": args.preset, "profiles": resolved["file"],
        "resolved_config": config_path}))
    command = [sys.executable, "-m", "real_world_rl.cli", "supervise", "--", sys.executable,
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
    return Result(result, json.dumps(result, indent=2))
