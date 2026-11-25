"""``python -m real_world_rl.shell.identity STATION ROBOT``: whether the daemon at the robot's
address serves that robot, as one JSON line. A separate process keeps gRPC out of the shell."""
from __future__ import annotations

import json
import sys


def probe(station: str, robot: str) -> dict:
    from real_world_rl.cli.connect import client_address, describe_daemon_identity
    from real_world_rl.common.robot_identity import expected_identity, identity_mismatches
    from real_world_rl.station import load_station
    selected = load_station(station).select(robot)
    actual = describe_daemon_identity(client_address(selected.bind), selected.token)
    return {"identity": actual, "mismatches": identity_mismatches(expected_identity(selected.config), actual)}


def main(argv=None) -> int:
    station, robot = sys.argv[1:] if argv is None else argv
    try:
        result = probe(station, robot)
    except Exception as error:  # reported in the status bar
        result = {"error": " ".join(str(error).split()) or type(error).__name__}
    print(json.dumps(result))
    return 0


if __name__ == "__main__":
    sys.exit(main())
