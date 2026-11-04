"""Hardware primitive implementations; no env, training, or task runtime imports."""
from __future__ import annotations

from contextlib import redirect_stdout
from dataclasses import asdict
import math
import sys

from .connect import open_robot
from .errors import CliError
from .preset_edit import Preset, joints


def numbers(values) -> str:
    return "[" + ", ".join(f"{value:.4f}" for value in values) + "]"


def confirm(args, target: str, delta: float, units: str) -> None:
    print(f"target {target}; max delta {delta:.4f} {units}")
    if args.yes:
        return
    if not sys.stdin.isatty():
        raise CliError("refusing to move without --yes", code=2)
    try:
        # input() writes its prompt to stdout; JSON mode reserves stdout for the result.
        with redirect_stdout(sys.stderr) if args.json else redirect_stdout(sys.stdout):
            answer = input("proceed? [y/N] ")
    except EOFError:
        answer = ""
    if answer.strip().lower() != "y":
        raise CliError("cancelled", code=2)


def gripper_feedback(gripper) -> tuple[dict, str]:
    norm = float(gripper.width)
    max_width = float(gripper.max_width_m)
    grasped = gripper.is_grasped
    source = type(gripper).__name__
    data = {"width": norm, "width_m": norm * max_width, "max_width_m": max_width,
            "grasped": grasped, "source": source}
    text = (f"gripper width {norm * max_width:.4f} m ({norm:.2f} of max_width_m {max_width:.3f}), "
            f"grasped={grasped if grasped is not None else 'unknown'}, source {source}")
    return data, text


def current_joints(connection) -> list[float]:
    value = connection.arm.read_state()["joint_pos"]
    values = value.tolist() if hasattr(value, "tolist") else list(value)
    return joints(values, connection.arm.num_joints, "joint_pos")


def resolve_home(preset: Preset, connection) -> tuple[list[float], str]:
    home = preset.reset.get("home_joints")
    source = "preset task.reset.home_joints"
    if home is None:
        home = preset.env.get("home_joints")
        source = "preset home_joints"
    if home is None:
        home = (getattr(connection.arm, "home_joints", None) if connection.remote
                else connection.station_config.home_joints)
        source = "station home_joints" + (" (daemon arm.home_joints)" if connection.remote else "")
    if home is None:
        raise CliError("no home joint configuration", hint="set home_joints in the preset or station")
    return joints(home, connection.arm.num_joints), source


def delta_to(home: list[float], current: list[float]) -> tuple[list[float], float]:
    delta = [target - actual for target, actual in zip(home, current)]
    return delta, max(abs(value) for value in delta)


def home_command(args, connection, preset: Preset) -> tuple[dict, str]:
    current = current_joints(connection)
    if args.action == "set":
        limits = [("station joint_limits", connection.station_config.joint_limits)]
        arm_limits = connection.arm.safety_limits
        if arm_limits:
            limits.append(("arm joint_limits", arm_limits.get("joint_limits")))
        result = preset.save_home(current, limits, num_joints=connection.arm.num_joints)
        return result, (f"task.reset.home_joints: {result['old']} -> {numbers(result['home_joints'])}\n"
                        f"{result['warning']}")
    home, source = resolve_home(preset, connection)
    delta, largest = delta_to(home, current)
    if args.action == "go":
        gripper_action = preset.reset.get("gripper")
        target = "home " + numbers(home)
        if connection.gripper is not None and gripper_action in ("open", "close"):
            width_target = 1.0 if gripper_action == "open" else 0.0
            width_delta = abs(width_target - connection.gripper.width)
            target += f"; gripper {gripper_action}, max width delta {width_delta:.4f} norm"
        confirm(args, target, largest, "rad")
        connection.arm.move_to_joints(home, blocking=True)
        if connection.gripper is not None and gripper_action in ("open", "close"):
            getattr(connection.gripper, gripper_action)()
        current = current_joints(connection)
        delta, largest = delta_to(home, current)
    result = {"source": source, "home_joints": home, "joint_pos": current,
              "delta": delta, "max_delta": largest}
    text = (f"source: {source}\nhome (rad): {numbers(home)}\n"
            f"q (rad): {numbers(current)}\ndelta (home - q, rad): {numbers(delta)}\n"
            f"max delta: {largest:.6f} rad")
    return result, text


def gripper_command(args, connection) -> tuple[dict, str]:
    gripper = connection.gripper
    if gripper is None:
        raise CliError("robot has no gripper")
    if args.action != "show":
        target = args.fraction if args.action == "move" else float(args.action == "open")
        confirm(args, f"gripper {args.action} {target:.4f}", abs(target - gripper.width), "norm")
        if args.action == "move":
            gripper.move(target)
        else:
            getattr(gripper, args.action)()
    data, text = gripper_feedback(gripper)
    return {"gripper": data}, text


def status(connection) -> tuple[dict, str]:
    description = connection.describe()
    connection.arm.read_state()  # Populate feedback before reporting arm health.
    health = asdict(connection.arm.health())
    data = {"description": description, "arm_health": health,
            "cameras": list(connection.cameras),
            "teleop": None if connection.teleop is None else connection.teleop.name,
            "gripper": None}
    text = [f"connection: {connection.path}",
            f"robot: {description['name']} ({description['type']})",
            f"arm: {type(connection.arm).__name__}, ok={health['ok']}, "
            f"mode={health['control_mode']}, errors={health['errors']}"]
    if connection.gripper is not None:
        data["gripper"], feedback = gripper_feedback(connection.gripper)
        text.append(feedback)
    else:
        text.append("gripper: none")
    text += ["cameras: " + (", ".join(data["cameras"]) or "none"),
             "teleop: " + (data["teleop"] or "none")]
    if connection.remote:
        connection.health()
        data["rtt_ms"] = connection.client.last_rtt_ms
        rtt = data["rtt_ms"]
        text.append("daemon rtt: " + ("n/a" if rtt is None else f"{rtt:.3f} ms"))
    return data, "\n".join(text)


def execute(args) -> tuple[dict, str]:
    if args.command == "home" and args.action == "set" and args.preset is None:
        raise CliError("home set requires --preset", code=2)
    if args.command == "gripper" and args.action == "move":
        if not math.isfinite(args.fraction) or not 0 <= args.fraction <= 1:
            raise CliError("gripper move F requires 0 <= F <= 1", code=2)
    preset = Preset.load(args.preset) if args.command == "home" else None
    with open_robot(args) as connection:
        if args.command == "status":
            data, output = status(connection)
        elif args.command == "q":
            q = current_joints(connection)
            data, output = {"joint_pos": q}, "q (rad): " + numbers(q)
        elif args.command == "tcp":
            value = connection.arm.read_state()["tcp_pose"]
            values = value.tolist() if hasattr(value, "tolist") else list(value)
            pose = joints(values, 7, "tcp_pose")
            data = {"tcp_pose": pose}
            output = "tcp xyz (m): " + numbers(pose[:3]) + "; quat (xyzw): " + numbers(pose[3:])
        elif args.command == "home":
            data, output = home_command(args, connection, preset)
        elif args.command == "gripper":
            data, output = gripper_command(args, connection)
        else:
            getattr(connection.arm, args.command)()
            data, output = {"ok": True}, args.command + ": ok"
        command = args.command
        if hasattr(args, "action"):
            command += " " + args.action
        return {"command": command, "path": connection.path, **data}, output
