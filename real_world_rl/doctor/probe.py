"""Isolated, read-only device doctor: ``python -m real_world_rl.doctor.probe`` reads one robot
request as JSON on stdin and prints that robot's check rows as one JSON list on stdout. The rows
keep the device engine's fields (``layer``, ``name``, ``ok``, ...); ``rwrl doctor`` turns them into
its own findings. It never repairs or moves anything."""

from __future__ import annotations

import dataclasses
import json
import sys
from contextlib import redirect_stdout

from real_world_rl.cli.errors import SUCCESS_EXIT_CODE


def main():
    request = json.load(sys.stdin)
    with redirect_stdout(sys.stderr):
        from real_world_rl.devices import Robot
        from real_world_rl.doctor.checks import check_rows, run_doctor

        config = Robot.from_dict(request["config"]).config
        # Always participate in device ownership even if a legacy config disables claims.
        config = dataclasses.replace(config, claim=True)
        results = run_doctor(
            config,
            mock=bool(request.get("mock")),
            config_path=request.get("path"),
            serves_daemon=request.get("daemon", False),
        )
    print(json.dumps(check_rows(results), default=str))
    return SUCCESS_EXIT_CODE


if __name__ == "__main__":
    raise SystemExit(main())
