"""Runtime report destinations, separate from persisted environment configuration."""
from __future__ import annotations

import os
from dataclasses import dataclass
from typing import Optional


@dataclass
class RuntimeReportPaths:
    stop_trace: Optional[str] = None
    diagnostics: Optional[str] = None
    session_events: Optional[str] = None

    @classmethod
    def from_environ(cls) -> RuntimeReportPaths:
        return cls(
            stop_trace=os.environ.get("RWRL_STOP_TRACE_PATH"),
            diagnostics=os.environ.get("RWRL_DIAGNOSTICS_PATH"),
            session_events=os.environ.get("RWRL_SESSION_EVENTS_PATH"),
        )
