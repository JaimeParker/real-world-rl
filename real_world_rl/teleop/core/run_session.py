"""Backend-independent preparation and supervision of teleop recording sessions."""
from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, field
import json
import os
from pathlib import Path
import sys
import uuid

from real_world_rl.cli.command import Result
from real_world_rl.cli.errors import CliError, USAGE_EXIT_CODE
from real_world_rl.cli.station_edit import station_document


@dataclass
class SessionPlan:
    """Backend settings and evidence, merged without reordering existing preset keys.

    ``prepare_report`` writes backend-owned inputs after preflight and before typed parsing;
    it can update settings and provenance files with paths under the new report directory.
    """

    settings: dict
    env_overrides: dict = field(default_factory=dict)
    confirm_detail: str = ""
    provenance_files: dict = field(default_factory=dict)
    summary_extra: dict = field(default_factory=dict)
    diagnose: dict | None = None
    prepare_report: Callable[[Path, dict, dict], None] | None = None


def add_common_run_arguments(parser):
    """Recorder flags accepted by every session backend, displayed once by dispatch."""
    add = getattr(parser, "add_shared_argument", parser.add_argument)
    add("--events-file", help="append-only JSONL file, or - for piped stdin")
    add("--record", required=True, help="new output .pkl path")
    add("--num-successes", type=int, help="default: 20; 1 with --diagnose")
    add("--max-episodes", type=int)


def load_session_station(args, diagnose=None):
    """Common preflight, also used before a backend resolves its station-owned settings."""
    if args.preset is None:
        raise CliError("teleop run requires --preset with home and task limits", code=USAGE_EXIT_CODE, required_fields=["--preset"])
    if os.environ.get("RWRL_HEARTBEAT_FD"):
        raise CliError("teleop run starts its own supervisor; invoke it directly", code=USAGE_EXIT_CODE)
    station = station_document(args)
    if diagnose:
        if station[4].config.workspace is None:
            raise CliError("motion diagnostics require a reviewed station workspace", code=USAGE_EXIT_CODE)
        if not 0 < diagnose["gain"] <= 1 or not 0 < diagnose["duration_s"] <= 300:
            raise CliError("gain must be in (0,1] and duration in (0,300]", code=USAGE_EXIT_CODE)
    return station


def run_supervised_session(args, ctx, plan: SessionPlan) -> Result:
    """Prepare a normal recorder configuration, then supervise its owning child."""
    import dataclasses
    import subprocess
    import yaml
    from real_world_rl.cli.connect import client_address, probe_daemon
    from real_world_rl.cli.preset_edit import Preset

    path, _, _, _, selected = load_session_station(args, plan.diagnose)
    config = selected.config
    if args.num_successes is None:
        args.num_successes = 1 if plan.diagnose else 20
    if args.max_episodes is None and plan.diagnose:
        args.max_episodes = 1
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
            from real_world_rl.doctor.checks import mocked_config
            robot = {"type": config.TYPE, **dataclasses.asdict(mocked_config(config))}
        env.update(robot=robot, remote=None, token=None)
    settings = {**preset.document, "real_robot": env, "env_backend": "real_robot",
                "teleop_device": plan.settings["teleop_device"],
                "teleop_event_device": "jsonl" if args.events_file else "stdin",
                "teleop_events_file": (str(Path(args.events_file).resolve())
                                       if args.events_file and args.events_file != "-" else "-"),
                **{key: value for key, value in plan.settings.items() if key != "teleop_device"},
                "num_successes": args.num_successes, "max_episodes": args.max_episodes,
                "output_path": str(output)}
    env.update(plan.env_overrides)
    if plan.diagnose:
        import math
        settings.update(teleop_diagnostic_axes=plan.diagnose["axes"], teleop_diagnostic_gain=plan.diagnose["gain"],
                        max_episode_steps=max(1, math.ceil(plan.diagnose["duration_s"] * settings.get("control_hz", env.get("control_hz", 10.)))))
    # Resolve all typed configuration before asking for hardware execution.
    output.parent.mkdir(parents=True, exist_ok=True)
    report = output.parent / (output.stem + ".session-" + uuid.uuid4().hex)
    report.mkdir(mode=0o700)
    files = {"station": path, "preset": args.preset, **plan.provenance_files}
    if plan.prepare_report is not None:
        plan.prepare_report(report, settings, files)
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
    ctx.confirm("proceed? [y/N] ", detail=plan.confirm_detail)
    from real_world_rl.common.json_io import write_json
    from real_world_rl.common.provenance import runtime_provenance
    files["resolved_config"] = config_path
    write_json(report / "provenance.json", runtime_provenance(config=settings, files=files))
    command = [sys.executable, "-m", "real_world_rl.cli", "supervise", "--", sys.executable,
               "-m", "real_world_rl.demos", "--config", str(config_path)]
    completed = subprocess.run(command, stdout=sys.stderr if args.json else None, check=False, env=child_environment)
    import hashlib
    result = {"schema_version": 1, "report_directory": str(report), "returncode": completed.returncode,
              "resolved_config_sha256": hashlib.sha256(config_path.read_bytes()).hexdigest(),
              **plan.summary_extra, "output": str(output)}
    assessments = sorted(report.glob("stop-trace-*.assessment.json"))
    if assessments:
        result["stop_assessment"] = json.loads(assessments[-1].read_text())
    else:
        result["stop_assessment"] = {"status": "BLOCKED", "reason": "no normal-stop evidence was exported"}
    (report / "summary.json").write_text(json.dumps(result, indent=2))
    if completed.returncode:
        raise CliError(f"session exited {completed.returncode}; report: {report}")
    return Result(result, json.dumps(result, indent=2))

