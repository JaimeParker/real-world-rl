"""Input acquisition, calibration and named profile management."""
from __future__ import annotations

import os
import sys
import json
from pathlib import Path

from .command import Result
from .errors import CliError, USAGE_EXIT_CODE
from .station_edit import station_document, confirm_write, atomic_save


def profiles(args, ctx):
    from real_world_rl.teleop.profiles import load_profiles, resolve_profile
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


def _check_calibrate_flags(args, backend):
    from .cmd_teleop import CALIBRATE_BACKENDS, given_calibrate_flags
    misplaced = [flag for other in CALIBRATE_BACKENDS if other != backend
                 for flag in given_calibrate_flags(args, other)]
    if misplaced:
        raise CliError(f"{', '.join(misplaced)} not applicable to --backend {backend}", code=USAGE_EXIT_CODE)
    if backend == "spacemouse" and args.profile is None:
        raise CliError("teleop calibrate --backend spacemouse requires --profile", code=USAGE_EXIT_CODE,
                       required_fields=["--profile"])


def _station_teleop_backend(selected):
    from .cmd_teleop import CALIBRATE_BACKENDS
    teleop = selected.config.teleop
    if teleop is None:
        raise CliError("station has no teleop; pass --backend", code=USAGE_EXIT_CODE, required_fields=["--backend"])
    if teleop.backend not in CALIBRATE_BACKENDS:
        raise CliError(f"station teleop backend '{teleop.backend}' has no calibration",
                       code=USAGE_EXIT_CODE, hint="pass --backend " + " or --backend ".join(CALIBRATE_BACKENDS))
    return teleop.backend


def calibrate(args, ctx):
    if args.backend is not None:
        _check_calibrate_flags(args, args.backend)
    station = station_document(args)
    backend = args.backend or _station_teleop_backend(station[4])
    if args.backend is None:
        _check_calibrate_flags(args, backend)
    if backend == "gello":
        return calibrate_gello_teleop(args, ctx, station)
    from .cmd_teleop import SPACEMOUSE_CALIBRATE_DEFAULTS
    for name, value in SPACEMOUSE_CALIBRATE_DEFAULTS.items():
        if getattr(args, name) is None:
            setattr(args, name, value)
    return calibrate_spacemouse(args, ctx, station)


# GELLO calibrate flag destination -> station teleop.options key supplying its default.
GELLO_STATION_DEFAULTS = {"port": "port", "joint_ids": "joint_ids", "gripper_id": "gripper_id",
                          "baudrate": "baudrate", "joint_signs": "joint_signs"}
GELLO_DEFAULT_BAUDRATE = 57600
# teleop.options keys that only mean something with a gripper channel.
GELLO_GRIPPER_OPTIONS = ("gripper_id", "gripper_open_rad", "gripper_closed_rad", "gripper_binarize_threshold")


class _UsageErrors:
    """The ``parser`` that ``gello_calibrate.validate_args`` reports through."""

    @staticmethod
    def error(message):
        raise CliError(message, code=USAGE_EXIT_CODE)


def calibrate_gello_teleop(args, ctx, station):
    """Guided GELLO calibration; unset flags fall back to the station teleop options."""
    from types import SimpleNamespace
    from robot_infra.devices.backends.teleops import gello_calibrate
    path, raw, document, node, selected = station
    config = selected.config
    station_teleop = config.teleop
    if args.use and station_teleop is not None and station_teleop.backend != "gello":
        raise CliError(f"--use would replace the options of the station's {station_teleop.backend} teleop",
                       code=USAGE_EXIT_CODE)
    station_gello = station_teleop is not None and station_teleop.backend == "gello"
    station_options = dict(station_teleop.options) if station_gello else {}
    values = {name: getattr(args, name) if getattr(args, name) is not None else station_options.get(key)
              for name, key in GELLO_STATION_DEFAULTS.items()}
    missing = [flag for flag, name in (("--port", "port"), ("--joint-ids", "joint_ids")) if values[name] is None]
    if missing:
        raise CliError("GELLO calibration needs the leader port and joint ids (station teleop.options or flags)",
                       code=USAGE_EXIT_CODE, required_fields=missing)
    joint_ids = [int(joint_id) for joint_id in values["joint_ids"]]
    signs = values["joint_signs"]
    if args.joint_signs is None and signs is not None and len(signs) != len(joint_ids):
        signs = None  # The station signs belong to another joint set; the sign check starts from +1.
    checked = gello_calibrate.validate_args(SimpleNamespace(
        joint_ids=joint_ids, calibration_pose=args.calibration_pose,
        joint_signs=None if signs is None else [int(sign) for sign in signs]), _UsageErrors())
    if args.joint_limits_yaml is not None:
        joint_limits = gello_calibrate.load_joint_limits(args.joint_limits_yaml)
        source = args.joint_limits_yaml
    else:
        joint_limits, source = config.joint_limits, "the station joint_limits"
    if joint_limits is not None and len(joint_limits) != len(joint_ids):
        raise CliError(f"{source} has {len(joint_limits)} joint_limits rows for {len(joint_ids)} leader joints",
                       code=USAGE_EXIT_CODE, hint="pass --joint-limits-yaml for the follower arm")
    if not sys.stdin.isatty():
        raise CliError("GELLO calibration prompts the operator at every step and needs a terminal",
                       code=USAGE_EXIT_CODE)

    def echo(text):
        print(text, file=ctx.stderr, flush=True)

    def ask(prompt):
        print(prompt, file=ctx.stderr, end="", flush=True)
        answer = sys.stdin.readline()
        if not answer:
            raise CliError("cancelled", code=USAGE_EXIT_CODE)
        return answer.rstrip("\n")

    options = gello_calibrate.calibrate_gello(
        port=str(values["port"]), joint_ids=joint_ids, gripper_id=values["gripper_id"],
        baudrate=int(values["baudrate"] if values["baudrate"] is not None else GELLO_DEFAULT_BAUDRATE),
        calibration_pose=checked.calibration_pose, joint_signs=checked.joint_signs, joint_limits=joint_limits,
        skip_sign_check=args.skip_sign_check, ask=ask, echo=echo)
    block = gello_calibrate.format_yaml_block(**options)
    result = {"backend": "gello", "options": options, "yaml_block": block}
    if args.use:
        teleop_node = node.get("teleop")
        previous = {} if teleop_node is None else (teleop_node.get("options") or {})
        merged = {**previous, **options}
        if options["gripper_id"] is None:
            # A leader without a gripper channel must not keep the old gripper settings.
            for key in GELLO_GRIPPER_OPTIONS:
                merged.pop(key, None)
        if teleop_node is None:
            node["teleop"] = {"name": "gello", "backend": "gello", "options": merged}
        else:
            teleop_node["options"] = merged
        confirm_write(ctx, path, document, raw)
        result["station_saved"] = atomic_save(path, document, raw)
        echo(f"saved teleop.options to {path}")
    return Result(result, block.rstrip("\n"))


def calibrate_spacemouse(args, ctx, station):
    from real_world_rl.teleop.profiles import load_profiles, validate_profile, AXIS_MAPPING_VERSION
    path, raw, document, node, selected = station
    config = selected.config
    result = {"schema_version": 1, "station": str(path), "robot": config.name}
    from real_world_rl.teleop.calibration import fit_yaw, validate_axes
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
        from real_world_rl.teleop.input_diagnostics import collect, LABELS
        if not sys.stdin.isatty():
            raise CliError("missing --samples: live calibration requires a terminal", code=USAGE_EXIT_CODE, required_fields=["--samples"])
        samples, acquisition = collect(config, duration_s=args.duration_s,
                                       directory=args.report_dir, labels=LABELS)
        result.update(acquisition)
    if args.capture_label:
        from real_world_rl.teleop.input_diagnostics import collect, write_json
        from real_world_rl.teleop.calibration import sample_trials
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
    from real_world_rl.teleop.profiles import resolve_profile
    path, raw, document, node, selected = station_document(args)
    config = selected.config
    result = {"schema_version": 1, "station": str(path), "robot": config.name}
    from real_world_rl.teleop.input_diagnostics import collect
    resolved = resolve_profile(path, selected, args.profile) if selected.sections.get("teleop_profiles") else None
    if args.profile and resolved is None:
        raise CliError("--profile requires station teleop_profiles", code=USAGE_EXIT_CODE)
    _, report = collect(config, duration_s=args.duration_s, directory=args.report_dir,
                        rotation=resolved["input_rotation"] if resolved else None,
                        yaw_rad=resolved["profile"]["yaw_rad"] if resolved else 0.)
    result.update(report)
    return Result(result, json.dumps(result, indent=2, allow_nan=False))


