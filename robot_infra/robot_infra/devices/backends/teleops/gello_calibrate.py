"""Calibrate a GELLO leader arm and print the robot yaml ``teleop:`` block.

    python -m robot_infra.devices.backends.teleops.gello_calibrate --port /dev/serial/by-id/... \\
        --joint-ids 1 2 3 4 5 6 7 --gripper-id 8 [--baudrate 57600] \\
        [--calibration-pose 0 0 0 -1.57 0 1.57 0] [--joint-signs 1 -1 1 -1 1 1 1] \\
        [--joint-limits-yaml robot.yaml] [--skip-sign-check]

Steps: a guided per-joint sign check, the joint offsets from a known calibration pose, the
gripper trigger's open and closed raw angles, and the wrap centres from the follower's
``joint_limits``. The procedure follows GELLO (MIT license, Philipp Wu,
https://github.com/wuphilipp/gello_software); the helpers below are pure functions over a
reader callable so they run without hardware."""
from __future__ import annotations

from robot_infra.cli.cmd_gello_calibrate import build_parser
import argparse
from pathlib import Path
from typing import Any, Callable, Mapping, Sequence

import numpy as np
import yaml

from robot_infra.devices.backends.teleops.gello import (
    DEFAULT_EVENT_KEYS,
    DynamixelBus,
    GelloTeleop,
)

# Offsets are searched on the quarter-turn grid in [-8 pi, 8 pi] (servo horns mount in 90 degree steps).
OFFSET_GRID_RAD = np.linspace(-8 * np.pi, 8 * np.pi, 33)
FRANKA_CALIBRATION_POSE = (0.0, 0.0, 0.0, -np.pi / 2, 0.0, np.pi / 2, 0.0)
MIN_SIGN_CHECK_MOTION_RAD = 0.1
MIN_GRIPPER_TRAVEL_RAD = 0.05
_DECIMALS = 4

RawReader = Callable[[], np.ndarray]  # raw multi-turn angles in rad, one per servo
Ask = Callable[[str], str]
Echo = Callable[[str], None]


def solve_offsets(raw_rad: Sequence[float], calibration_pose: Sequence[float], signs: Sequence[float]) -> np.ndarray:
    """Per joint, the grid offset minimising ``|sign * (raw - offset) - pose|``."""
    raw = np.asarray(raw_rad, dtype=np.float64)
    pose = np.asarray(calibration_pose, dtype=np.float64)
    sign = np.asarray(signs, dtype=np.float64)
    if not raw.shape == pose.shape == sign.shape or raw.ndim != 1:
        raise ValueError(
            f"raw_rad, calibration_pose and signs need one entry per joint, got shapes "
            f"{raw.shape}, {pose.shape}, {sign.shape}"
        )
    errors = np.abs(sign[:, None] * (raw[:, None] - OFFSET_GRID_RAD[None, :]) - pose[:, None])
    return OFFSET_GRID_RAD[np.argmin(errors, axis=1)]


def offset_residuals(
    raw_rad: Sequence[float], calibration_pose: Sequence[float], signs: Sequence[float], offsets: Sequence[float]
) -> np.ndarray:
    """``|sign * (raw - offset) - pose|`` per joint; near 0 when the leader sat in the pose."""
    raw, pose = np.asarray(raw_rad, dtype=np.float64), np.asarray(calibration_pose, dtype=np.float64)
    return np.abs(np.asarray(signs) * (raw - np.asarray(offsets)) - pose)


def observed_joint_sign(
    before_rad: Sequence[float],
    after_rad: Sequence[float],
    joint_index: int,
    min_motion_rad: float = MIN_SIGN_CHECK_MOTION_RAD,
) -> int | None:
    """+1/-1 from the raw change of one joint moved in the robot's positive direction; None when
    it moved less than ``min_motion_rad``."""
    delta = float(after_rad[joint_index]) - float(before_rad[joint_index])
    if abs(delta) < min_motion_rad:
        return None
    return 1 if delta > 0 else -1


def check_joint_signs(
    read_raw_rad: RawReader,
    initial_signs: Sequence[int],
    ask: Ask,
    echo: Echo = print,
    *,
    min_motion_rad: float = MIN_SIGN_CHECK_MOTION_RAD,
) -> list[int]:
    """Guided check: one joint at a time the operator moves it in the robot's positive
    direction; the observed sign is offered and taken unless the operator answers ``n``."""
    signs = [int(sign) for sign in initial_signs]
    for index in range(len(signs)):
        before = read_raw_rad()
        ask(f"joint {index + 1}: move it in the robot's positive direction, then press Enter ")
        observed = observed_joint_sign(before, read_raw_rad(), index, min_motion_rad)
        if observed is None:
            echo(f"joint {index + 1}: moved less than {min_motion_rad} rad; keeping sign {signs[index]:+d}")
            continue
        answer = ask(
            f"joint {index + 1}: observed sign {observed:+d} (current {signs[index]:+d}); use it? [Y/n] "
        )
        if answer.strip().lower() not in ("n", "no"):
            signs[index] = observed
    return signs


def capture_gripper_range(
    read_raw_rad: RawReader, gripper_index: int, ask: Ask, *, min_travel_rad: float = MIN_GRIPPER_TRAVEL_RAD
) -> tuple[float, float]:
    """Raw trigger angles ``(open_rad, closed_rad)`` captured with the trigger held open, then closed."""
    ask("gripper: hold the trigger fully OPEN, then press Enter ")
    open_rad = float(read_raw_rad()[gripper_index])
    ask("gripper: hold the trigger fully CLOSED, then press Enter ")
    closed_rad = float(read_raw_rad()[gripper_index])
    if abs(open_rad - closed_rad) < min_travel_rad:
        raise ValueError(
            f"gripper trigger moved only {abs(open_rad - closed_rad):.3f} rad between open and closed; "
            "check gripper_id and repeat"
        )
    return open_rad, closed_rad


def wrap_centers_from_joint_limits(joint_limits: Sequence[Sequence[float]]) -> list[float]:
    return [(float(low) + float(high)) / 2 for low, high in joint_limits]


def load_joint_limits(robot_yaml: str | Path) -> list[list[float]] | None:
    """``joint_limits`` of a robot yaml (top level or under ``robot:``), None when unset."""
    data = yaml.safe_load(Path(robot_yaml).read_text())
    if isinstance(data, Mapping) and "robot" in data:
        data = data["robot"]
    if not isinstance(data, Mapping):
        raise ValueError(f"{robot_yaml} is not a robot config mapping")
    return data.get("joint_limits")


def _number(value: float) -> str:
    rounded = round(float(value), _DECIMALS)
    return repr(0.0 if rounded == 0 else rounded)


def _flow_list(values: Sequence[Any], render: Callable[[Any], str] = _number) -> str:
    return "[" + ", ".join(render(value) for value in values) + "]"


def format_yaml_block(
    *,
    port: str,
    joint_ids: Sequence[int],
    joint_offsets_rad: Sequence[float],
    joint_signs: Sequence[int],
    wrap_centers_rad: Sequence[float] | None = None,
    baudrate: int = 57600,
    gripper_id: int | None = None,
    gripper_open_rad: float | None = None,
    gripper_closed_rad: float | None = None,
    gripper_binarize_threshold: float | None = None,
    intervention: str = "always",
    event_device: str | None = None,
    event_keys: Mapping[str, str] | None = None,
    poll_interval_s: float = 0.0,
    name: str = "gello",
    hold_s: float = 0.0,
) -> str:
    """The robot yaml ``teleop:`` block with every gello option."""
    centers = [0.0] * len(joint_ids) if wrap_centers_rad is None else list(wrap_centers_rad)
    keys = dict(DEFAULT_EVENT_KEYS if event_keys is None else event_keys)
    key_text = "{" + ", ".join(f"{action}: {key or 'null'}" for action, key in keys.items()) + "}"

    def optional_number(value: float | None) -> str:
        return "null" if value is None else _number(value)

    lines = [
        "teleop:",
        f"  name: {name}",
        "  backend: gello",
        f"  hold_s: {_number(hold_s)}",
        "  options:",
        f"    port: {port}",
        f"    baudrate: {int(baudrate)}",
        f"    joint_ids: {_flow_list(joint_ids, lambda value: str(int(value)))}",
        f"    joint_offsets_rad: {_flow_list(joint_offsets_rad)}",
        f"    joint_signs: {_flow_list(joint_signs, lambda value: str(int(value)))}",
        f"    wrap_centers_rad: {_flow_list(centers)}",
        f"    gripper_id: {'null' if gripper_id is None else int(gripper_id)}",
        f"    gripper_open_rad: {optional_number(gripper_open_rad)}",
        f"    gripper_closed_rad: {optional_number(gripper_closed_rad)}",
        f"    gripper_binarize_threshold: {optional_number(gripper_binarize_threshold)}",
        f"    intervention: {intervention}",
        f"    event_device: {'null' if event_device is None else event_device}",
        f"    event_keys: {key_text}",
        f"    poll_interval_s: {_number(poll_interval_s)}",
    ]
    return "\n".join(lines) + "\n"




def _parse_args(argv: Sequence[str] | None) -> argparse.Namespace:
    parser = build_parser()
    return validate_args(parser.parse_args(argv), parser)


def validate_args(args, parser):
    num_joints = len(args.joint_ids)
    if args.calibration_pose is None:
        if num_joints != len(FRANKA_CALIBRATION_POSE):
            parser.error(f"--calibration-pose is required for {num_joints} joints")
        args.calibration_pose = list(FRANKA_CALIBRATION_POSE)
    if args.joint_signs is None:
        args.joint_signs = [1] * num_joints
    for option, values in (("--calibration-pose", args.calibration_pose), ("--joint-signs", args.joint_signs)):
        if len(values) != num_joints:
            parser.error(f"{option} needs {num_joints} values, got {len(values)}")
    if any(sign not in (1, -1) for sign in args.joint_signs):
        parser.error(f"--joint-signs entries must be 1 or -1, got {args.joint_signs}")
    return args


def _rounded(value: float) -> float:
    rounded = round(float(value), _DECIMALS)
    return 0.0 if rounded == 0 else rounded


def calibrate_gello(
    *,
    port: str,
    joint_ids: Sequence[int],
    gripper_id: int | None = None,
    baudrate: int = 57600,
    calibration_pose: Sequence[float] | None = None,
    joint_signs: Sequence[int] | None = None,
    joint_limits: Sequence[Sequence[float]] | None = None,
    skip_sign_check: bool = False,
    ask: Ask = input,
    echo: Echo = print,
    bus: Any = None,
) -> dict[str, Any]:
    """Run the guided calibration on the leader and return the gello options it determines:
    ``port``, ``baudrate``, ``joint_ids``, ``joint_offsets_rad``, ``joint_signs``,
    ``wrap_centers_rad``, ``gripper_id``, ``gripper_open_rad``, ``gripper_closed_rad`` (angles
    rounded to 4 decimals); ``format_yaml_block(**options)`` renders them. ``bus`` replaces the
    ``DynamixelBus`` (an unopened object with ``open``, ``read_positions_rad`` and ``close``);
    by default the SDK is required before the first prompt. ``calibration_pose`` defaults to the
    Franka pose for 7 joints, ``joint_signs`` to all +1."""
    joint_ids = [int(joint_id) for joint_id in joint_ids]
    num_joints = len(joint_ids)
    if calibration_pose is None:
        if num_joints != len(FRANKA_CALIBRATION_POSE):
            raise ValueError(f"calibration_pose is required for {num_joints} joints")
        calibration_pose = list(FRANKA_CALIBRATION_POSE)
    initial_signs = [1] * num_joints if joint_signs is None else [int(sign) for sign in joint_signs]
    for option, values in (("calibration_pose", calibration_pose), ("joint_signs", initial_signs)):
        if len(values) != num_joints:
            raise ValueError(f"{option} needs {num_joints} values, got {len(values)}")
    servo_ids = joint_ids + ([] if gripper_id is None else [int(gripper_id)])
    if bus is None:
        bus = DynamixelBus(GelloTeleop.require_sdk(), port, baudrate, servo_ids, device="gello_calibrate")
    bus.open()  # torque off: the leader moves freely
    try:
        signs = list(initial_signs)
        if not skip_sign_check:
            signs = check_joint_signs(bus.read_positions_rad, signs, ask, echo)
        ask(f"hold the leader in the calibration pose {_flow_list(calibration_pose)}, then press Enter ")
        raw = bus.read_positions_rad()
        offsets = solve_offsets(raw[:num_joints], calibration_pose, signs)
        residuals = offset_residuals(raw[:num_joints], calibration_pose, signs, offsets)
        echo(f"offset residuals (rad): {_flow_list(residuals)}")
        if np.any(residuals > np.pi / 8):
            echo("warning: a residual above pi/8 means the leader was not in the pose or a sign is wrong")
        gripper_open = gripper_closed = None
        if gripper_id is not None:
            gripper_open, gripper_closed = capture_gripper_range(bus.read_positions_rad, num_joints, ask)
    finally:
        bus.close()
    if joint_limits is None:
        echo("warning: no joint_limits given (--joint-limits-yaml); wrap_centers_rad are zeros, which is "
             "wrong for joints whose range is not centred on 0 (Panda joint 6 reaches past pi)")
        centers = [0.0] * num_joints
    else:
        if len(joint_limits) != num_joints:
            raise ValueError(f"joint_limits has {len(joint_limits)} rows, expected {num_joints}")
        centers = wrap_centers_from_joint_limits(joint_limits)
    return {
        "port": port,
        "baudrate": int(baudrate),
        "joint_ids": joint_ids,
        "joint_offsets_rad": [_rounded(offset) for offset in offsets],
        "joint_signs": [int(sign) for sign in signs],
        "wrap_centers_rad": [_rounded(center) for center in centers],
        "gripper_id": None if gripper_id is None else int(gripper_id),
        "gripper_open_rad": None if gripper_open is None else _rounded(gripper_open),
        "gripper_closed_rad": None if gripper_closed is None else _rounded(gripper_closed),
    }


def main(argv: Sequence[str] | None = None, *, ask: Ask = input, echo: Echo = print) -> int:
    return run(_parse_args(argv), None, ask=ask, echo=echo)


def run(args, ctx, *, ask: Ask = input, echo: Echo = print) -> int:
    args = validate_args(args, getattr(args, "_parser", None) or build_parser())
    joint_limits = None if args.joint_limits_yaml is None else load_joint_limits(args.joint_limits_yaml)
    options = calibrate_gello(
        port=args.port,
        joint_ids=args.joint_ids,
        gripper_id=args.gripper_id,
        baudrate=args.baudrate,
        calibration_pose=args.calibration_pose,
        joint_signs=args.joint_signs,
        joint_limits=joint_limits,
        skip_sign_check=args.skip_sign_check,
        ask=ask,
        echo=echo,
    )
    echo(format_yaml_block(**options))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
