"""Offline stop-trace assessment, including its acceptance exit code."""
import json
from pathlib import Path

from .command import Result
from .errors import (CliError, USAGE_EXIT_CODE, SUCCESS_EXIT_CODE,
                     STOP_TRACE_FAIL_EXIT_CODE, STOP_TRACE_BLOCKED_EXIT_CODE)


def inspect_stop(args, ctx):
    import hashlib
    from real_world_rl.devices.stop_trace import StopThresholds, evaluate_stop_trace
    try:
        source = Path(args.trace).read_bytes()
        thresholds = StopThresholds(**json.loads(args.thresholds)) if args.thresholds else StopThresholds()
        result = evaluate_stop_trace(json.loads(source), thresholds)
    except (OSError, ValueError, TypeError, KeyError) as error:
        raise CliError(f"invalid stop trace or thresholds: {error}", code=USAGE_EXIT_CODE) from error
    result.update(trace=str(Path(args.trace).resolve()), trace_sha256=hashlib.sha256(source).hexdigest())
    code = {"PASS": SUCCESS_EXIT_CODE, "FAIL": STOP_TRACE_FAIL_EXIT_CODE,
            "BLOCKED": STOP_TRACE_BLOCKED_EXIT_CODE}[result["status"]]
    return Result(result, json.dumps(result, indent=2), code)
