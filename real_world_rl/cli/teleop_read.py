"""Stream live teleop samples: one line (or JSON object) per printed sample, then a summary."""
from __future__ import annotations

import json
import time

from .command import Result
from .errors import CliError, DEVICE_EXIT_CODE, SUCCESS_EXIT_CODE, USAGE_EXIT_CODE

READ_TIMEOUT_S = 1.0
_DECIMALS = 4


def _numbers(values) -> list[float]:
    return [round(float(value), _DECIMALS) for value in values]


def _action_name(action_space: str) -> str:
    return "q" if action_space.startswith("joint_") else "twist"


def sample_record(sample, elapsed_s: float, follower_q=None) -> dict:
    """Printable fields of one sample; ``delta`` (``joint_abs`` only) is leader minus follower, per joint."""
    action = _numbers(sample.action)
    record = {
        "t": round(elapsed_s, 3),
        "seq": sample.seq,
        _action_name(sample.action_space): action,
        "gripper": None if sample.gripper is None else round(float(sample.gripper), _DECIMALS),
        "intervening": bool(sample.intervening),
        "events": sorted(name for name, fired in sample.events.items() if fired),
    }
    if follower_q is not None:
        follower = _numbers(follower_q)
        record["follower_q"] = follower
        # Only absolute joint targets compare with the follower's joint positions.
        if sample.action_space == "joint_abs" and len(follower) == len(action):
            record["delta"] = [round(leader - arm, _DECIMALS) for leader, arm in zip(action, follower)]
    return record


def _text_value(value) -> str:
    if value is None:
        return "none"
    if isinstance(value, list):
        return "[" + ", ".join(str(item) for item in value) + "]"
    if isinstance(value, float):
        return f"{value:.3f}"
    return str(value)


def text_line(record: dict) -> str:
    return " ".join(f"{key}={_text_value(value)}" for key, value in record.items() if key != "seq")


def _health_failure(teleop) -> str | None:
    health = teleop.health()
    if health.ok:
        return None
    return f"teleop '{teleop.name}' is unhealthy: {health.detail or 'poll thread stopped or failing'}"


def read(args, ctx):
    from robot_infra.devices.errors import DeviceError
    from .connect import open_teleop

    if args.rate_hz <= 0:
        raise CliError("--rate-hz must be positive", code=USAGE_EXIT_CODE)
    if args.duration_s is not None and args.duration_s <= 0:
        raise CliError("--duration-s must be positive", code=USAGE_EXIT_CODE)
    period_s = 1.0 / args.rate_hz
    samples = 0
    events_seen: dict[str, int] = {}
    error = None
    with open_teleop(args, with_follower=args.with_follower) as connection:
        teleop = connection.teleop
        start = time.monotonic()
        deadline = None if args.duration_s is None else start + args.duration_s
        next_due = start
        try:
            error = _health_failure(teleop)
            while error is None and (deadline is None or time.monotonic() < deadline):
                sample = teleop.read(READ_TIMEOUT_S)
                follower_q = (None if connection.follower is None
                              else connection.follower.read_state()["joint_pos"])
                record = sample_record(sample, time.monotonic() - start, follower_q)
                samples += 1
                for name in record["events"]:
                    events_seen[name] = events_seen.get(name, 0) + 1
                line = json.dumps(record, allow_nan=False) if args.json else text_line(record)
                print(line, file=ctx.stdout, flush=True)
                error = _health_failure(teleop)
                next_due += period_s
                wait_s = next_due - time.monotonic()
                if deadline is not None:
                    wait_s = min(wait_s, deadline - time.monotonic())
                if error is None and wait_s > 0:
                    time.sleep(wait_s)
        except KeyboardInterrupt:
            pass
        except DeviceError as failure:
            error = str(failure).replace("\n", " ")
        summary = {"samples": samples, "duration_s": round(time.monotonic() - start, 3),
                   "events_seen": events_seen, "action_space": teleop.action_space,
                   "backend": connection.station_config.teleop.backend, "path": connection.path}
    if error is not None:
        summary["error"] = error
        print(f"error: {error}", file=ctx.stderr, flush=True)
    if args.json:
        print(json.dumps({"summary": summary}, allow_nan=False), file=ctx.stdout, flush=True)
    else:
        print(text_line(summary), file=ctx.stdout, flush=True)
    return Result({"summary": summary}, "", DEVICE_EXIT_CODE if error is not None else SUCCESS_EXIT_CODE,
                  rendered=True)
