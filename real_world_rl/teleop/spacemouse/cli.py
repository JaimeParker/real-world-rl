"""SpaceMouse teleop commands: direction-profile calibration, profile selection, input
diagnosis, the supervised recording session (``teleop run``) and the HID probe
(``teleop probe-hid``).

``real_world_rl.cli.teleop_dispatch`` finds the handlers and their ``add_<op>_arguments`` here by
name. Importing this module is cheap: the device, profile and learning modules load in the
handlers."""
from __future__ import annotations

import os
import sys
import json
from pathlib import Path

from real_world_rl.cli.command import Result
from real_world_rl.cli.errors import CliError, USAGE_EXIT_CODE
from real_world_rl.cli.station_edit import station_document, confirm_write, atomic_save

_CAPTURE_LABELS = ("+x", "+y", "-x", "-y", "+z", "-z", "+rx", "-rx", "+ry", "-ry", "+rz", "-rz")
# `teleop calibrate` value flags default to None so a flag given for another backend can be
# refused; ``calibrate`` applies these defaults.
CALIBRATE_DEFAULTS = {"duration_s": 3., "frame": "table", "max_error_deg": 25., "neutral_threshold": .05}
_CALIBRATE_FLAGS = (
    ("--samples", dict(help="JSON mapping +x,+y,-x,-y to normalized N x 6 samples")),
    ("--capture-label", dict(choices=_CAPTURE_LABELS, help="capture one direction; replace that group in --samples")),
    ("--append-trial", dict(action="store_true",
                            help="retain existing trials for --capture-label instead of replacing")),
    ("--profile", dict(help="profile name; required with --backend spacemouse")),
    ("--duration-s", dict(type=float, help="capture duration")),
    ("--report-dir", dict()),
    ("--frame", dict(help="station task_frame name or base")),
    ("--output", dict(help="profile YAML; defaults to station teleop_profiles.file")),
    ("--max-error-deg", dict(type=float)),
    ("--neutral-threshold", dict(type=float)),
    ("--save", dict(action="store_true", help="save validated candidate after confirmation")),
)
GROUP_DESCRIPTIONS = {"calibrate": "direction profile flags; only with --backend spacemouse"}
SHARED_FLAG_HELP = {"calibrate": {"--use": "save and select this profile in the station"}}


def add_calibrate_arguments(parser):
    for flag, options in _CALIBRATE_FLAGS:
        options = dict(options)
        default = CALIBRATE_DEFAULTS.get(flag[2:].replace("-", "_"))
        if default is not None:
            options["help"] = f"{options.get('help', '')} (default: {default})".strip()
        parser.add_argument(flag, **options)


def add_profiles_arguments(parser):
    actions = parser.add_subparsers(dest="action", required=True)
    actions.add_parser("list")
    for name in ("show", "use"):
        actions.add_parser(name).add_argument("--profile", required=True)


def add_diagnose_arguments(parser):
    parser.add_argument("--duration-s", type=float, default=5.)
    parser.add_argument("--profile")
    parser.add_argument("--report-dir")


def add_run_arguments(parser):
    from real_world_rl.teleop.core.run_session import add_common_run_arguments
    parser.add_argument("--mode", choices=("tracker", "serl"))
    parser.add_argument("--profile")
    add_common_run_arguments(parser)
    parser.add_argument("--diagnose", choices=("translation", "rotation", "all"))
    parser.add_argument("--gain", type=float, help="diagnostic input gain; ignored without --diagnose")
    parser.add_argument("--duration-s", type=float, help="diagnostic duration; ignored without --diagnose")


def add_probe_hid_arguments(parser):
    from real_world_rl.teleop.spacemouse.probe import DEFAULT_DURATION_S, MAX_DURATION_S, PROBES_DIRECTORY
    parser.add_argument("--device", metavar="VID:PID",
                        help="the HID device to probe, e.g. 256f:c63a; default: the only candidate")
    parser.add_argument("--duration-s", type=float, default=DEFAULT_DURATION_S,
                        help=f"raw report capture time in (0, {MAX_DURATION_S:g}] s (default: {DEFAULT_DURATION_S:g})")
    parser.add_argument("--output", metavar="DIR",
                        help=f"new probe directory; default: <station dir>/{PROBES_DIRECTORY}/<vid>-<pid>/ "
                             "(required without a station)")


def calibrate(args, ctx):
    if args.profile is None:
        raise CliError("teleop calibrate --backend spacemouse requires --profile", code=USAGE_EXIT_CODE,
                       required_fields=["--profile"])
    station = station_document(args)
    for name, value in CALIBRATE_DEFAULTS.items():
        if getattr(args, name) is None:
            setattr(args, name, value)
    return calibrate_spacemouse(args, ctx, station)


def profiles(args, ctx):
    from real_world_rl.teleop.spacemouse.profiles import load_profiles, resolve_profile
    path, raw, document, node, selected = station_document(args)
    config = selected.config
    result = {"schema_version": 1, "station": str(path), "robot": config.name}
    reference = selected.sections.get("teleop_profiles")
    if not reference:
        raise CliError("station has no teleop_profiles", code=USAGE_EXIT_CODE)
    file = Path(reference["file"])
    if not file.is_absolute():
        file = path.parent / file
    if args.action == "list":
        result.update(default=reference["default"], profiles=load_profiles(file))
    else:
        result.update(resolve_profile(path, selected, args.profile))
        if args.action == "use":
            node["teleop_profiles"] = {**reference, "default": args.profile}
            confirm_write(ctx, path, document, raw)
            result["saved"] = atomic_save(path, document, raw)
    return Result(result, json.dumps(result, indent=2, allow_nan=False))


def calibrate_spacemouse(args, ctx, station):
    from real_world_rl.teleop.spacemouse.profiles import load_profiles, validate_profile, AXIS_MAPPING_VERSION
    path, raw, document, node, selected = station
    config = selected.config
    result = {"schema_version": 1, "station": str(path), "robot": config.name}
    from real_world_rl.teleop.spacemouse.calibration import fit_yaw, validate_axes
    acquisition = None
    if args.frame != "base" and (not config.task_frame or args.frame != config.task_frame["name"]):
        raise CliError("calibration frame must match station task_frame", code=USAGE_EXIT_CODE)
    if args.append_trial and not args.capture_label:
        raise CliError("--append-trial requires --capture-label", code=USAGE_EXIT_CODE, required_fields=["--capture-label"])
    if args.samples:
        samples = json.loads(Path(args.samples).read_text())
    elif args.capture_label:
        samples = {}
    else:
        from real_world_rl.teleop.spacemouse.diagnostics import collect, LABELS
        if not sys.stdin.isatty():
            raise CliError("missing --samples: live calibration requires a terminal", code=USAGE_EXIT_CODE, required_fields=["--samples"])
        samples, acquisition = collect(config, duration_s=args.duration_s,
                                       directory=args.report_dir, labels=LABELS)
        result.update(acquisition)
    if args.capture_label:
        from real_world_rl.teleop.spacemouse.diagnostics import collect, write_json
        from real_world_rl.teleop.spacemouse.calibration import sample_trials
        if not sys.stdin.isatty() and not args.yes:
            raise CliError("unattended labelled capture requires --yes", code=USAGE_EXIT_CODE)
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
            return Result(result, json.dumps(result, indent=2, allow_nan=False))
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
        reference = selected.sections.get("teleop_profiles") or {}
        output = args.output or reference.get("file")
        if not output:
            raise CliError("saving a first profile requires --output", code=USAGE_EXIT_CODE, required_fields=["--output"])
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
        confirm_write(ctx, file, payload, previous)
        result["saved"] = atomic_save(file, payload, previous)
        result["station_reference"] = {"file": str(file), "default": args.profile}
        if args.use:
            node["teleop_profiles"] = {"file": os.path.relpath(file, path.parent), "default": args.profile}
            confirm_write(ctx, path, document, raw)
            result["station_saved"] = atomic_save(path, document, raw)
        else:
            result["next_action"] = "Use --use to also select this profile in the station"
    return Result(result, json.dumps(result, indent=2, allow_nan=False))


def diagnose(args, ctx):
    from real_world_rl.teleop.spacemouse.profiles import resolve_profile
    path, raw, document, node, selected = station_document(args)
    config = selected.config
    result = {"schema_version": 1, "station": str(path), "robot": config.name}
    from real_world_rl.teleop.spacemouse.diagnostics import collect
    resolved = resolve_profile(path, selected, args.profile) if selected.sections.get("teleop_profiles") else None
    if args.profile and resolved is None:
        raise CliError("--profile requires station teleop_profiles", code=USAGE_EXIT_CODE)
    _, report = collect(config, duration_s=args.duration_s, directory=args.report_dir,
                        rotation=resolved["input_rotation"] if resolved else None,
                        yaw_rad=resolved["profile"]["yaw_rad"] if resolved else 0.)
    result.update(report)
    return Result(result, json.dumps(result, indent=2, allow_nan=False))


def run_session(args, ctx):
    """Resolve the SpaceMouse profile and supervise the common recorder session."""
    from real_world_rl.teleop.core.run_session import SessionPlan, load_session_station, run_supervised_session
    from real_world_rl.teleop.spacemouse.profiles import resolve_profile

    mode = args.mode or "tracker"
    diagnostic = ({"axes": args.diagnose, "gain": .2 if args.gain is None else args.gain,
                   "duration_s": 5. if args.duration_s is None else args.duration_s} if args.diagnose else None)
    path, _, _, _, selected = load_session_station(args, diagnostic)
    resolved = resolve_profile(path, selected, args.profile)
    plan = SessionPlan(
        settings={"teleop_device": "spacemouse", "spacemouse_mode": mode,
                  "teleop_options_yaml": str(path), "teleop_robot": args.robot,
                  "teleop_profile": resolved["name"], "episode_end_behavior": "hold",
                  "neutral_threshold": resolved["profile"]["neutral_threshold"]},
        env_overrides={"cartesian_semantics": "serl"} if mode == "serl" else {},
        confirm_detail=(f"target SpaceMouse {mode} session on {selected.config.name}; task reset may home; "
                        "s/f ends episode and holds; n resets next; q exits; max delta 0.0000 session"),
        provenance_files={"profiles": resolved["file"]}, summary_extra={"profile": resolved["name"]},
        diagnose=diagnostic)
    return run_supervised_session(args, ctx, plan)


def _probe_station_directory(args):
    """The directory of the station file in use, or None without a station."""
    from real_world_rl.discovery.autoconfig import resolve_station_config_path
    try:
        path = resolve_station_config_path(args.station)
    except FileNotFoundError:
        return None
    if not path.is_file():
        raise CliError(f"station file {path} does not exist", code=USAGE_EXIT_CODE)
    return path.resolve().parent


def probe_hid(args, ctx):
    """Record a SpaceMouse-like HID device's report descriptor and raw reports and draft its
    device table entry, without moving a robot or writing the device table."""
    from real_world_rl.teleop.spacemouse import probe
    from real_world_rl.teleop.spacemouse.hid import load_device_specs
    if not 0 < args.duration_s <= probe.MAX_DURATION_S:
        raise CliError(f"--duration-s must be in (0, {probe.MAX_DURATION_S:g}]", code=USAGE_EXIT_CODE)
    try:
        hid_id = None if args.device is None else probe.parse_hid_id(args.device)
    except ValueError as error:
        raise CliError(str(error), code=USAGE_EXIT_CODE) from error
    station_dir = _probe_station_directory(args)
    if args.output is None and station_dir is None:
        raise CliError("teleop probe-hid needs --output without a station", code=USAGE_EXIT_CODE,
                       required_fields=["--output"])
    try:
        table = load_device_specs(station_dir)  # only names a device the table already knows
    except ValueError:
        table = None
    try:
        payload = probe.run_probe(hid_id=hid_id, duration_s=args.duration_s, output=args.output,
                                  station_dir=station_dir, table=table,
                                  echo=lambda text: print(text, file=ctx.stderr, flush=True))
    except probe.AmbiguousDevice as error:
        hint = "pass --device VID:PID"
        raise CliError(str(error), code=USAGE_EXIT_CODE, hint=hint, required_fields=["--device"],
                       payload={"error": str(error), "hint": hint, "candidates": error.candidates}) from error
    except FileExistsError as error:
        raise CliError(f"probe directory {error.filename} exists; pass --output DIR or remove it",
                       code=USAGE_EXIT_CODE) from error
    return Result(payload, probe.text_report(payload))
