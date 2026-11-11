"""Gripper feedback and motion commands."""
import math

from .command import Context, Result
from .errors import CliError, USAGE_EXIT_CODE


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


def gripper_command(args, connection, ctx: Context) -> tuple[dict, str]:
    gripper = connection.gripper
    if gripper is None:
        raise CliError("robot has no gripper")
    if args.action != "show":
        target = args.fraction if args.action == "move" else float(args.action == "open")
        ctx.confirm("proceed? [y/N] ", detail=(f"target gripper {args.action} {target:.4f}; "
                    f"max delta {abs(target - gripper.width):.4f} norm"))
        if args.action == "move":
            gripper.move(target)
        else:
            getattr(gripper, args.action)()
    data, text = gripper_feedback(gripper)
    return {"gripper": data}, text


class GripperCommand:
    name = "gripper"
    help = "read or command the gripper"

    def register(self, parser):
        actions = parser.add_subparsers(dest="action", required=True)
        for name in ("show", "open", "close", "move"):
            sub = actions.add_parser(name, allow_abbrev=False)
            if name == "move":
                sub.add_argument("fraction", type=float, metavar="F")

    def run(self, args, ctx):
        if args.action == "move" and (not math.isfinite(args.fraction) or not 0 <= args.fraction <= 1):
            raise CliError("gripper move F requires 0 <= F <= 1", code=USAGE_EXIT_CODE)
        with ctx.open_robot() as connection:
            data, text = gripper_command(args, connection, ctx)
            return Result({"command": args.command + " " + args.action, "path": connection.path, **data}, text)


GRIPPER = GripperCommand()
