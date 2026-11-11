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


def calibrate(args, ctx):
    from real_world_rl.teleop.profiles import load_profiles, validate_profile, AXIS_MAPPING_VERSION
    path, raw, document, node, selected = station_document(args)
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


