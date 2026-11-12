"""Lightweight setup registration; device imports wait until run."""
from __future__ import annotations

import argparse

from .options import add_station_argument

STATION_SCOPES = ("local", "user")
ARM_BACKENDS = ("franky", "serl_http", "fake")
GRIPPER_CHOICES = (*ARM_BACKENDS, "none")
TELEOP_CHOICES = ("gello", "spacemouse", "fake", "none")
SECTIONS = ("cameras", "teleop", "daemon")
MAX_ARMS = 2
FRANKA_NUM_JOINTS = 7
SUGGESTED_SAFETY_BOX = (0.3, -0.3, 0.05, 0.7, 0.3, 0.6)
DEFAULT_MAX_DELTA_XYZ_M = 0.05
DEFAULT_MAX_DELTA_JOINT_RAD = 0.04


def build_parser(parser=None) -> argparse.ArgumentParser:
    parser = parser if parser is not None else argparse.ArgumentParser(
        prog="rwrl setup",
        description='``rwrl setup``: detect the hardware, ask a few questions, calibrate the GELLO,\nwrite the station yaml, check it with the doctor and print the next commands.\n\n    rwrl setup                       # interactive, probes the hardware\n    rwrl setup --mock                # canned hardware, simulated calibration\n    rwrl setup --station local/station.yaml [--only cameras|teleop|daemon]\n    rwrl setup --non-interactive --arms 1 --robot-ip 172.16.0.2 ...\n\nEvery question has a flag; per-arm flags take one value per arm (or one for all arms).\n``--non-interactive`` fails on a question that has neither a flag nor a ``--station`` value.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    run = parser.add_argument_group("run")
    run.add_argument("--mock", action="store_true", help="canned probe results and a simulated GELLO; the doctor runs with --mock")
    run.add_argument("--non-interactive", action="store_true", help="never prompt; fail on a question without a flag or --station value")
    add_station_argument(run, alias="--from", dest="from_path", help="existing station yaml whose values are the defaults")
    run.add_argument("--only", choices=SECTIONS, default=None, help="rerun one section, keep the rest from --station")
    run.add_argument("--scope", choices=STATION_SCOPES, default=None,
                     help="write ./local/station.yaml (local) or ~/.config/robot_infra/station.yaml (user)")
    run.add_argument("--output", default=None, help="write to this path instead of a --scope location")
    probe = parser.add_argument_group("probes")
    probe.add_argument("--robot-ip", nargs="+", default=None, help="Franka IP per arm (also probed)")
    probe.add_argument("--scan-subnet", default=None, help="also probe every host of this CIDR (at most 256)")
    probe.add_argument("--port", dest="serial_ports", nargs="+", default=None, help="extra serial ports to probe for Dynamixel servos")
    probe.add_argument("--probe-timeout-s", type=float, default=0.5, help="TCP connect timeout per probed port")
    probe.add_argument("--allow-unprobed", action="store_true", help="accept an IP, camera serial or port the probe did not find")
    questions = parser.add_argument_group("answers (per-arm flags: one value per arm, or one for all)")
    questions.add_argument("--arms", type=int, choices=range(1, MAX_ARMS + 1), default=None)
    questions.add_argument("--arm-names", nargs="+", default=None, help="default: arm, or left right")
    questions.add_argument("--arm-backend", nargs="+", default=None, help=" | ".join(ARM_BACKENDS))
    questions.add_argument("--gripper", nargs="+", default=None, help=" | ".join(GRIPPER_CHOICES))
    questions.add_argument("--home-joints", nargs="+", default=None,
                           help="comma-separated rad values per arm, or none (write --home-joints=-0.1,... when the first is negative)")
    questions.add_argument("--joint-limits", nargs="+", default=None,
                           help=f"per arm: {2 * FRANKA_NUM_JOINTS} comma-separated rad values, lo,hi per joint "
                                "(default: Panda datasheet; write --joint-limits=-2.9,2.9,... as the first is negative)")
    questions.add_argument("--safety-box", nargs="+", default=None,
                           help="per arm: xmin,ymin,zmin,xmax,ymax,zmax in m, base frame "
                                f"(suggested: {','.join(map(str, SUGGESTED_SAFETY_BOX))})")
    questions.add_argument("--max-delta-xyz-m", nargs="+", default=None,
                           help=f"per arm: servo step cap of the TCP position, m (default {DEFAULT_MAX_DELTA_XYZ_M})")
    questions.add_argument("--max-delta-joint-rad", nargs="+", default=None,
                           help=f"per arm: servo step cap per joint, rad (default {DEFAULT_MAX_DELTA_JOINT_RAD})")
    questions.add_argument("--control", nargs="+", default=None,
                           help="optional per-arm impedance JSON/YAML mapping; null clears the profile; omitted preserves --station")
    questions.add_argument("--cameras", nargs="+", default=None, help="per arm: SERIAL[=NAME],... or none")
    questions.add_argument("--camera-resolution", nargs="+", default=None,
                           help="per arm: WIDTHxHEIGHT in pixels for that arm's cameras, e.g. 1280x720 "
                                "(unset: new cameras get the camera default, a --station camera keeps its value)")
    questions.add_argument("--camera-fps", nargs="+", default=None,
                           help="per arm: frames per second for that arm's cameras, a positive integer "
                                "(unset: new cameras get the camera default, a --station camera keeps its value)")
    questions.add_argument("--camera-depth", nargs="+", default=None,
                           help="per arm: yes | no, record the depth stream of that arm's cameras "
                                "(unset: new cameras get the camera default, a --station camera keeps its value)")
    questions.add_argument("--teleop", nargs="+", default=None, help=" | ".join(TELEOP_CHOICES))
    questions.add_argument("--gello-port", nargs="+", default=None)
    questions.add_argument("--gello-baudrate", nargs="+", default=None)
    questions.add_argument("--gello-joint-ids", nargs="+", default=None, help="comma-separated servo ids per arm")
    questions.add_argument("--gello-gripper-id", nargs="+", default=None, help="servo id per arm, or none")
    questions.add_argument("--intervention", nargs="+", default=None, help="always | toggle")
    questions.add_argument("--event-device", nargs="+", default=None, help="/dev/input/by-id/... per arm, or none")
    questions.add_argument("--daemon-port", nargs="+", default=None)
    questions.add_argument("--calibrate", action=argparse.BooleanOptionalAction, default=None,
                           help="calibrate every GELLO now (default: ask; --non-interactive: no)")
    return parser



def run(args, ctx):
    from robot_infra.setup.command import run as implementation
    return implementation(args, ctx)
