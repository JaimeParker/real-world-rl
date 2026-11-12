"""Workspace inspection, measurement, validation and reviewed updates."""
from __future__ import annotations

import dataclasses
import json
from pathlib import Path

from .command import Result
from .errors import CliError, USAGE_EXIT_CODE, SUCCESS_EXIT_CODE, WORKSPACE_INVALID_EXIT_CODE
from .station_edit import station_document, confirm_write, atomic_save


def home_limit_report(home_joints, joint_limits) -> dict:
    """Whether each station home joint lies inside its joint limits; None when either is not configured."""
    if home_joints is None or joint_limits is None:
        return {"home_within_limits": None}
    violations = [{"joint": joint, "value": value, "low": low, "high": high}
                  for joint, (value, (low, high)) in enumerate(zip(home_joints, joint_limits))
                  if not low <= value <= high]
    report = {"home_within_limits": not violations}
    if violations:
        report["home_violations"] = violations
    return report


FRAME_POINT_FLAGS = ("--origin", "--x-axis", "--y-axis")


def frame_candidate(args, station_path: Path) -> tuple[dict, dict]:
    """A validated task_frame from 16 row-major values or three captured points, plus point quality."""
    from real_world_rl.devices.workspace import task_transform
    point_names = dict(zip(FRAME_POINT_FLAGS, (args.origin, args.x_axis, args.y_axis)))
    if args.base_T_frame is not None:
        if any(name is not None for name in point_names.values()):
            raise CliError("--origin/--x-axis/--y-axis apply only to --from-points", code=USAGE_EXIT_CODE)
        values = args.base_T_frame
        candidate = {"name": args.name, "base_T_frame": [values[row:row + 4] for row in range(0, 16, 4)]}
        quality = {}
        source = args.source
    else:
        missing = [flag for flag, name in point_names.items() if name is None]
        if missing:
            raise CliError("--from-points requires --origin, --x-axis and --y-axis", code=USAGE_EXIT_CODE,
                           required_fields=missing)
        import yaml
        from real_world_rl.teleop.core.workspace_points import (frame_from_points, frame_point_quality,
                                                            point_base_positions)
        file = Path(args.from_points)
        if not file.is_absolute():
            file = station_path.parent / file
        try:
            positions = point_base_positions(yaml.safe_load(file.read_text()))
        except ValueError as error:
            raise CliError(f"{file}: {error}", code=USAGE_EXIT_CODE) from error
        unknown = [name for name in point_names.values() if name not in positions]
        if unknown:
            raise CliError(f"points {unknown} are not in {file}; captured points: {sorted(positions)}",
                           code=USAGE_EXIT_CODE)
        selected = [positions[name] for name in point_names.values()]
        try:
            transform = frame_from_points(*selected)
            quality = frame_point_quality(*selected)
        except ValueError as error:
            raise CliError(str(error), code=USAGE_EXIT_CODE) from error
        candidate = {"name": args.name, "base_T_frame": transform.tolist()}
        source = args.source
        if source is None:
            source = f"points:{file.name} origin={args.origin} x={args.x_axis} y={args.y_axis}"
    if source is not None:
        candidate["source"] = source
    try:
        task_transform(candidate)
    except ValueError as error:
        raise CliError(f"invalid task frame: {error}", code=USAGE_EXIT_CODE) from error
    return candidate, quality


def frame_command(args, config, path, raw, document, node, ctx):
    from real_world_rl.teleop.core.workspace_points import frame_axes
    if args.frame_action == "show":
        return {"task_frame": config.task_frame, **frame_axes(config.task_frame)}
    candidate, quality = frame_candidate(args, path)
    workspace_frame = None if config.workspace is None else config.workspace.get("frame", "base")
    # A base-frame workspace does not depend on the task frame; any other workspace frame must keep its name.
    if workspace_frame not in (None, "base", candidate["name"]):
        raise CliError(f"the station workspace is defined in frame '{workspace_frame}'; "
                       f"task frame '{candidate['name']}' would invalidate it", code=USAGE_EXIT_CODE,
                       hint=f"use --name {workspace_frame} to recalibrate that frame, "
                            "or move the workspace to another frame first with rwrl workspace set")
    try:
        dataclasses.replace(config, task_frame=candidate)
    except ValueError as error:
        raise CliError(f"the station rejects this task frame: {error}", code=USAGE_EXIT_CODE) from error
    result = {"candidate": candidate, **quality, "saved": None,
              "note": "Candidate only; base_T_frame maps task-frame coordinates into the robot base frame. "
                      "Points files captured in another frame must be recaptured after saving"}
    if workspace_frame == candidate["name"]:
        result["workspace_frame_rebased"] = True
        result["workspace_note"] = (f"workspace bounds in frame '{workspace_frame}' move with the new frame; "
                                    "rerun rwrl workspace validate and review the bounds")
    if args.save:
        node["task_frame"] = candidate
        confirm_write(ctx, path, document, raw)
        result["saved"] = atomic_save(path, document, raw)
    return result


def workspace_command(args, config, path, raw, document, node, ctx):
    from real_world_rl.devices.workspace import Workspace, task_transform
    from real_world_rl.teleop.core.workspace_points import point_set, candidate_bounds
    import yaml
    if args.action == "frame":
        return frame_command(args, config, path, raw, document, node, ctx)
    if args.action == "capture":
        if args.save and (not args.name or not args.output):
            raise CliError("saving measurements requires --name and --output", code=USAGE_EXIT_CODE,
                           required_fields=[flag for flag, value in (("--name", args.name), ("--output", args.output)) if not value])
        file = Path(args.output) if args.output else None
        if file is not None and not file.is_absolute():
            file = path.parent / file
        previous = file.read_bytes() if file is not None and file.exists() else None
        points, _ = point_set(yaml.safe_load(previous) if previous else None, config.task_frame)
        with ctx.open_robot() as connection:
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
            confirm_write(ctx, file, points, previous)
            result['saved'] = atomic_save(file, points, previous)
        return result
    if args.action == "show":
        result = {"task_frame": config.task_frame, "workspace": config.workspace,
                  "base_safety_box": config.safety_box,
                  "joint_limits": config.joint_limits,
                  "max_delta_per_step": config.max_delta_per_step,
                  "home_joints": config.home_joints,
                  **home_limit_report(config.home_joints, config.joint_limits),
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
            raise CliError("no workspace configured", code=USAGE_EXIT_CODE)
        from real_world_rl.devices.safety import ArmSafety
        safety = ArmSafety.from_config(config)
        from real_world_rl.devices.errors import SafetyViolation
        try:
            safety.require_pose_inside(args.pose)
        except SafetyViolation as error:
            return {"valid": False, "pose": args.pose, "violations": [error.message]}
        return {"valid": True, "pose": args.pose, "violations": []}
    if args.from_points:
        if args.low is not None or args.high is not None:
            raise CliError("--from-points cannot be combined with --low/--high", code=USAGE_EXIT_CODE)
        file = Path(args.from_points)
        if not file.is_absolute():
            file = path.parent / file
        candidate = candidate_bounds(yaml.safe_load(file.read_text()), config.task_frame)
        if candidate['frame'] != args.frame:
            raise CliError("--frame must match captured points", code=USAGE_EXIT_CODE)
    else:
        if args.low is None or args.high is None:
            raise CliError("provide --low and --high, or --from-points", code=USAGE_EXIT_CODE)
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
        confirm_write(ctx, path, document, raw)
        result["saved"] = atomic_save(path, document, raw)
    return result


class WorkspaceCommand:
    name = "workspace"
    help = "inspect and configure task-frame workspace"

    def register(self, parser):
        ws = parser.add_subparsers(dest="action", required=True)
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
        frame = ws.add_parser("frame", help="show or set the station task frame (task_frame)")
        frame_actions = frame.add_subparsers(dest="frame_action", required=True)
        frame_actions.add_parser("show", help="print task_frame with its origin and unit axes in the base frame")
        frame_set = frame_actions.add_parser(
            "set", help="propose a task_frame; --save writes it to the station after a reviewed diff")
        frame_set.add_argument("--name", required=True, help="task frame name (not 'base')")
        definition = frame_set.add_mutually_exclusive_group(required=True)
        definition.add_argument("--base-T-frame", nargs=16, type=float, metavar="VALUE",
                                help="4x4 base_T_frame as 16 row-major numbers (m)")
        definition.add_argument("--from-points", metavar="PTS",
                                help="points YAML from workspace capture --save, relative to the station directory")
        frame_set.add_argument("--origin", metavar="POINT", help="captured point name for the frame origin")
        frame_set.add_argument("--x-axis", metavar="POINT", help="captured point name on the +x axis")
        frame_set.add_argument("--y-axis", metavar="POINT",
                               help="captured point name on the +y side of the x axis")
        frame_set.add_argument("--source", metavar="TEXT",
                               help="calibration provenance; default for --from-points: "
                                    "'points:<file> origin=A x=B y=C', none for --base-T-frame")
        frame_set.add_argument("--save", action="store_true", help="write task_frame to the station")

    def run(self, args, ctx):
        path, raw, document, node, selected = station_document(args)
        config = selected.config
        result = {"schema_version": 1, "station": str(path), "robot": config.name}
        result.update(workspace_command(args, config, path, raw, document, node, ctx))
        code = SUCCESS_EXIT_CODE
        if args.action == "validate" and not result["valid"] and not args.json:
            code = WORKSPACE_INVALID_EXIT_CODE
        return Result(result, json.dumps(result, indent=2, allow_nan=False), code)


WORKSPACE = WorkspaceCommand()
