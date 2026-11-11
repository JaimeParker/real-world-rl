"""Home configuration and motion commands."""
from __future__ import annotations

from .command import Context, Result
from .errors import CliError, USAGE_EXIT_CODE


def numbers(values) -> str:
    return "[" + ", ".join(f"{value:.4f}" for value in values) + "]"


def current_joints(connection) -> list[float]:
    from .preset_edit import joints
    value = connection.arm.read_state()["joint_pos"]
    values = value.tolist() if hasattr(value, "tolist") else list(value)
    return joints(values, connection.arm.num_joints, "joint_pos")


def resolve_home(preset, connection) -> tuple[list[float], str]:
    from .preset_edit import joints
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


def home_command(args, connection, preset, ctx: Context) -> tuple[dict, str]:
    current = current_joints(connection)
    if args.action == "set":
        limits = [("station joint_limits", connection.station_config.joint_limits)]
        arm_limits = connection.arm.safety_limits
        if arm_limits:
            limits.append(("arm joint_limits", arm_limits.get("joint_limits")))
        result = preset.save_home(current, limits, num_joints=connection.arm.num_joints)
        return result, (f"task.reset.home_joints: {result['old']} -> {numbers(result['home_joints'])}\n"
                        f"Previous file kept as {result['backup']}")
    home, source = resolve_home(preset, connection)
    delta, largest = delta_to(home, current)
    if args.action == "go":
        gripper_action = preset.reset.get("gripper")
        target = "home " + numbers(home)
        if connection.gripper is not None and gripper_action in ("open", "close"):
            width_target = 1.0 if gripper_action == "open" else 0.0
            width_delta = abs(width_target - connection.gripper.width)
            target += f"; gripper {gripper_action}, max width delta {width_delta:.4f} norm"
        ctx.confirm("proceed? [y/N] ", detail=f"target {target}; max delta {largest:.4f} rad")
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


class HomeCommand:
    name = "home"
    help = "show, save, or move to home (home phase only)"

    def register(self, parser):
        actions = parser.add_subparsers(dest="action", required=True)
        for name in ("show", "set", "go"):
            actions.add_parser(name, allow_abbrev=False)

    def run(self, args, ctx):
        from .preset_edit import Preset
        if args.action == "set" and args.preset is None:
            raise CliError("home set requires --preset", code=USAGE_EXIT_CODE)
        preset = Preset.load(args.preset)
        with ctx.open_robot() as connection:
            data, text = home_command(args, connection, preset, ctx)
            return Result({"command": args.command + " " + args.action, "path": connection.path, **data}, text)


HOME = HomeCommand()
