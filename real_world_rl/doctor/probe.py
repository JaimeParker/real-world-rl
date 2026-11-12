"""Isolated, read-only device doctor. Never accepts or executes repair callbacks."""

from __future__ import annotations

import dataclasses
import json
import sys
from contextlib import redirect_stdout

from .errors import SUCCESS_EXIT_CODE


def main():
    request = json.load(sys.stdin)
    with redirect_stdout(sys.stderr):
        from real_world_rl.devices import Robot
        from real_world_rl.doctor.checks import run_doctor

        config = Robot.from_dict(request["config"]).config
        # Always participate in device ownership even if a legacy config disables claims.
        config = dataclasses.replace(config, claim=True)
        results = run_doctor(
            config,
            mock=bool(request.get("mock")),
            fix=False,
            config_path=request.get("path"),
            serves_daemon=request.get("daemon", False),
        )
    payload = [
        {key: value for key, value in vars(row).items() if key != "autofix"}
        for row in results
    ]
    print(json.dumps(payload, default=str))
    return SUCCESS_EXIT_CODE


if __name__ == "__main__":
    raise SystemExit(main())
