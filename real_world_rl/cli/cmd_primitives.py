"""Read state, stop and recover; device access only when invoked."""
from dataclasses import asdict, dataclass
from typing import Callable

from .command import Result
from .cmd_home import current_joints, numbers
from .cmd_gripper import gripper_feedback


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


def q(connection):
    values = current_joints(connection)
    return {"joint_pos": values}, "q (rad): " + numbers(values)


def tcp(connection):
    from .preset_edit import joints
    value = connection.arm.read_state()["tcp_pose"]
    values = value.tolist() if hasattr(value, "tolist") else list(value)
    pose = joints(values, 7, "tcp_pose")
    return {"tcp_pose": pose}, "tcp xyz (m): " + numbers(pose[:3]) + "; quat (xyzw): " + numbers(pose[3:])


def stop(connection):
    connection.arm.stop()
    return {"ok": True}, "stop: ok"


def recover(connection):
    connection.arm.recover()
    return {"ok": True}, "recover: ok"


@dataclass
class PrimitiveCommand:
    name: str
    operation: Callable
    help: str = ""
    parser_options = {"allow_abbrev": False}

    def register(self, parser):
        pass

    def run(self, args, ctx):
        with ctx.open_robot() as connection:
            data, text = self.operation(connection)
            return Result({"command": self.name, "path": connection.path, **data}, text)


STATUS = PrimitiveCommand("status", status)
Q = PrimitiveCommand("q", q)
TCP = PrimitiveCommand("tcp", tcp)
STOP = PrimitiveCommand("stop", stop)
RECOVER = PrimitiveCommand("recover", recover)
