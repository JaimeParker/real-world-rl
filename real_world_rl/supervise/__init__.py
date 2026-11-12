"""Watchdogs for the single-process robot path.

- ``rwrl supervise [options] -- <command...>``: a separate supervisor process
  that kills the robot process group when its heartbeat goes silent (``supervisor.py``).
- ``Heartbeat``: the child side of that pipe (``heartbeat.py``).
- ``StepWatchdog``: the in-process first layer that calls ``arm.stop()`` when env steps stop
  (``inprocess.py``).
"""
from real_world_rl.supervise.heartbeat import (
    BEAT_BYTE,
    HEARTBEAT_FD_ENV,
    LONG_OPERATION_BYTE,
    Heartbeat,
    SupervisorLost,
    capture_heartbeats,
)
from real_world_rl.supervise.inprocess import StepWatchdog, StepWatchdogTripped

__all__ = [
    "BEAT_BYTE",
    "HEARTBEAT_FD_ENV",
    "LONG_OPERATION_BYTE",
    "Heartbeat",
    "StepWatchdog",
    "StepWatchdogTripped",
    "SupervisorLost",
    "capture_heartbeats",
]
