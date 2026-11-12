"""Torch-free arguments of the archive inspector, shared by its script and ``rwrl archive inspect``."""
from pathlib import Path

DRIFT_WINDOW_S = 10.0
SETTLE_SPEED_MM_S = 2.0
SETTLE_HOLD_S = 0.5


def add_inspect_arguments(parser) -> None:
    parser.add_argument("archive", type=Path)
    parser.add_argument("--csv", type=Path, help="new output CSV; source SQLite is opened read-only")
    parser.add_argument("--measurements", type=Path,
                        help="new measurements CSV with the test pages' header (segment, hold_window, settle rows)")
    parser.add_argument("--stop-time-s", type=float, action="append", default=[],
                        help="leader-stop wall time (date +%%s.%%N on the recording machine); repeatable")
    parser.add_argument("--drift-window-s", type=float, default=DRIFT_WINDOW_S,
                        help="hold-drift window from the start of each hold segment (default: %(default)s)")
    parser.add_argument("--settle-speed-mm-s", type=float, default=SETTLE_SPEED_MM_S,
                        help="TCP speed below which the arm counts as still (default: %(default)s)")
    parser.add_argument("--settle-hold-s", type=float, default=SETTLE_HOLD_S,
                        help="how long the speed must stay below that limit (default: %(default)s)")
