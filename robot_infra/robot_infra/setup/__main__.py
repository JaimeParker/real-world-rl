"""``python -m robot_infra.setup``: detect the hardware, ask a few questions, calibrate the GELLO,
write the station yaml, check it with the doctor and print the next commands.

    python -m robot_infra.setup                       # interactive, probes the hardware
    python -m robot_infra.setup --mock                # canned hardware, simulated calibration
    python -m robot_infra.setup --from local/station.yaml [--only cameras|teleop|daemon]
    python -m robot_infra.setup --non-interactive --arms 1 --robot-ip 172.16.0.2 ...

Every question has a flag; per-arm flags take one value per arm (or one for all arms).
``--non-interactive`` fails on a question that has neither a flag nor a ``--from`` value."""
from __future__ import annotations

from robot_infra.cli.cmd_setup import build_parser
import argparse
import sys
from pathlib import Path
from typing import Sequence

from robot_infra.cli.errors import ABORTED_EXIT_CODE as _ABORTED_EXIT_CODE
from robot_infra.cli.errors import USAGE_EXIT_CODE as _USAGE_EXIT_CODE
from robot_infra.discovery.autoconfig import (
    STATION_FILENAME,
    STATION_SCOPES,
    default_station_config_dir,
    load_station,
    resolve_station_config_path,
)
from robot_infra.doctor.cli import (
    doctor_station,
    format_station_text,
    station_exit_code,
)
from robot_infra.setup.probes import ProbeOptions, discover_station, mock_station_probe
from robot_infra.setup.render import (
    check_rendered_station,
    render_station_yaml,
    write_station_yaml,
)
from robot_infra.setup.wizard import (
    Answers,
    Ask,
    Echo,
    Prompter,
    StationDraft,
    run_wizard,
)

BIMANUAL_ENV = "real_robot_bimanual"
ROBOT_PC_PLACEHOLDER = "<robot-pc>"
_WILDCARD_HOSTS = ("", "0.0.0.0", "::", "[::]")




def answers_from_args(args: argparse.Namespace, previous: StationDraft | None) -> Answers:
    return Answers(
        arms=args.arms,
        arm_names=args.arm_names,
        robot_ips=args.robot_ip,
        arm_backends=args.arm_backend,
        grippers=args.gripper,
        home_joints=args.home_joints,
        joint_limits=args.joint_limits,
        safety_boxes=args.safety_box,
        controls=args.control,
        max_delta_xyz_m=args.max_delta_xyz_m,
        max_delta_joint_rad=args.max_delta_joint_rad,
        cameras=args.cameras,
        teleops=args.teleop,
        gello_ports=args.gello_port,
        gello_baudrates=args.gello_baudrate,
        gello_joint_ids=args.gello_joint_ids,
        gello_gripper_ids=args.gello_gripper_id,
        interventions=args.intervention,
        event_devices=args.event_device,
        daemon_ports=args.daemon_port,
        calibrate=args.calibrate,
        allow_unprobed=args.allow_unprobed,
        non_interactive=args.non_interactive,
        mock=args.mock,
        only=args.only,
        previous=previous,
    )


def output_path(args: argparse.Namespace, prompt: Prompter) -> Path:
    """``--output``, else the ``--scope`` location, else asked (default: the ``--from`` file)."""
    if args.output:
        return Path(args.output).expanduser()
    if args.scope:
        return default_station_config_dir(args.scope) / STATION_FILENAME

    def parse(text: str) -> Path:
        if text in STATION_SCOPES:
            return default_station_config_dir(text) / STATION_FILENAME
        return Path(text).expanduser()

    return prompt.answer(
        "--scope/--output", None,
        "write the station to (local = ./local/station.yaml, user = ~/.config/robot_infra/station.yaml, or a path)",
        parse, prior=args.from_path, suggested="local",
    )


def _found_without_config(path: Path) -> bool:
    try:
        return resolve_station_config_path().resolve() == path.resolve()
    except FileNotFoundError:
        return False


def next_steps_text(draft: StationDraft, path: Path, *, found_without_config: bool) -> str:
    config_flag = "" if found_without_config else f" --config {path}"
    lines = ["next steps:"]
    if draft.multi_robot:
        lines.append(f"  serve every robot, one daemon process each: python -m robot_infra.server{config_flag}")
        lines.extend(f"    '{arm.key}' listens on {arm.bind}" for arm in draft.arms)
    else:
        lines.append(f"  serve the robot: python -m robot_infra.server{config_flag}  (listens on {draft.arms[0].bind})")
    lines.append(f"  recheck any time: python -m robot_infra.doctor{config_flag}")
    camera_names = [camera.name for arm in draft.arms for camera in arm.config.cameras]
    rgb = [f"    --obs.rgb {' '.join(camera_names)}"] if camera_names else []
    if draft.multi_robot:
        lines.append(f"  on the actor side (replace {ROBOT_PC_PLACEHOLDER} with this PC's address), pass:")
        lines.extend(_bimanual_actor_flags(draft))
        lines.extend(rgb)
        return "\n".join(lines)
    teleop = draft.arms[0].config.teleop
    actor = []
    if teleop is not None and teleop.backend == "gello":
        actor.append(f"    --teleop-device gello --teleop-options-yaml {path}")
    actor.extend(rgb)
    if actor:
        lines.append("  on the actor side, point the env at the daemon address and pass:")
        lines.extend(actor)
    return "\n".join(lines)


def _actor_address(bind: str) -> str:
    """Where the actor reaches a daemon: its bind address, a wildcard host replaced by a placeholder."""
    host, _, port = bind.rpartition(":")
    if host in _WILDCARD_HOSTS:
        host = ROBOT_PC_PLACEHOLDER
    return f"{host}:{port}"


def _bimanual_actor_flags(draft: StationDraft) -> list[str]:
    """The bimanual env's flags: the first arm is ``left``, the second ``right``; with a teleop on
    every arm, each daemon serves its own leader arm."""
    left, right = draft.arms
    flags = [
        f"    --env-backend {BIMANUAL_ENV}",
        f"    --{BIMANUAL_ENV}.left.remote {_actor_address(left.bind)}"
        f" --{BIMANUAL_ENV}.right.remote {_actor_address(right.bind)}",
    ]
    if (left.key, right.key) != ("left", "right"):
        flags[-1] += f"   # left = '{left.key}', right = '{right.key}'"
    if all(arm.config.teleop is not None for arm in draft.arms):
        flags.append("    --teleop-device bimanual --teleop-arm-device leader_arm")
    return flags


def main(argv: Sequence[str] | None = None, *, ask: Ask = input, echo: Echo = print) -> int:
    return run(build_parser().parse_args(argv), None, ask=ask, echo=echo)


def run(args, ctx, *, ask: Ask = input, echo: Echo = print) -> int:
    try:
        previous = StationDraft.from_station(load_station(args.from_path)) if args.from_path else None
        if args.mock:
            probe = mock_station_probe()
        else:
            probe = discover_station(
                ProbeOptions(
                    robot_ips=list(args.robot_ip or []),
                    scan_subnet=args.scan_subnet,
                    serial_ports=[*(args.serial_ports or []), *(args.gello_port or [])],
                    timeout_s=args.probe_timeout_s,
                )
            )
        echo(probe.format())
        draft = run_wizard(probe, answers_from_args(args, previous), ask, echo)
        text = render_station_yaml(draft)
        check_rendered_station(text, draft)
        path = write_station_yaml(
            text, path_override=output_path(args, Prompter(ask, echo, non_interactive=args.non_interactive)), echo=echo
        )
    except (EOFError, KeyboardInterrupt):
        print("aborted; nothing written", file=sys.stderr)
        return _ABORTED_EXIT_CODE
    except (ValueError, OSError) as error:
        print(f"error: {error}", file=sys.stderr)
        return _USAGE_EXIT_CODE
    echo(f"wrote {path}")
    echo("checking it with the doctor" + (" (--mock)" if args.mock else "") + ":")
    results = doctor_station(path, mock=args.mock)
    echo(format_station_text(results))
    echo(next_steps_text(draft, path, found_without_config=_found_without_config(path)))
    return station_exit_code(results)


if __name__ == "__main__":
    raise SystemExit(main())
