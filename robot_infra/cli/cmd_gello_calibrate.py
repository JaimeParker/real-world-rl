"""Lightweight gello-calibrate registration; device imports wait until run."""
from __future__ import annotations

import argparse



def build_parser(parser=None):
    parser = parser if parser is not None else argparse.ArgumentParser(
        prog="rwrl gello-calibrate",
        description="Calibrate a GELLO leader arm and print the robot yaml teleop: block",
    )
    parser.epilog = ("Standalone GELLO calibration prints a teleop YAML block. "
                     "setup --only teleop --calibrate runs the same calibration in the station "
                     "wizard and writes the station YAML.")
    parser.add_argument("--port", required=True, help="/dev/serial/by-id/... of the U2D2 / OpenRB-150")
    parser.add_argument("--joint-ids", type=int, nargs="+", required=True)
    parser.add_argument("--gripper-id", type=int, default=None)
    parser.add_argument("--baudrate", type=int, default=57600)
    parser.add_argument(
        "--calibration-pose",
        type=float,
        nargs="+",
        default=None,
        help="follower joint angles (rad) of the pose the leader is held in; default: the Franka "
        "pose 0 0 0 -pi/2 0 pi/2 0 for 7 joints",
    )
    parser.add_argument("--joint-signs", type=int, nargs="+", default=None, help="initial signs (default all +1)")
    parser.add_argument("--joint-limits-yaml", default=None, help="robot yaml whose joint_limits give the wrap centres")
    parser.add_argument("--skip-sign-check", action="store_true", help="take --joint-signs as they are")
    return parser



def run(args, ctx):
    from robot_infra.devices.backends.teleops.gello_calibrate import run as implementation
    return implementation(args, ctx)
