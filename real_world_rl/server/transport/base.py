"""Transport abstraction: a server exposes a ``RobotService``; a client calls its methods
by name. Implementations differ only in how ``codec`` bytes move."""
from __future__ import annotations

from abc import ABC, abstractmethod
from typing import TYPE_CHECKING, Any, Iterator

from real_world_rl.devices.errors import DeviceFault

if TYPE_CHECKING:
    from real_world_rl.server.service import RobotService

READ_ONLY_METHODS = frozenset(
    {
        "robot.describe",
        "robot.health",
        "robot.get_observation",
        "arm.read_state",
        "arm.last_cartesian_target",
        "arm.last_stop_trace",
        "arm.control_mode",
        "arm.serl_params",
        "arm.joint_trajectory_params",
        "arm.last_joint_reset",
        "arm.serl_auto_recovery",
        "arm.health",
        "arm.preflight",
        "gripper.width",
        "gripper.wait_idle",  # only waits; safe to retry and allowed during a normal stop
    }
)


def is_read_only(method: str) -> bool:
    """Read-only methods are safe to retry after a reconnect; commands are not."""
    if method in READ_ONLY_METHODS:
        return True
    parts = method.split(".")
    return len(parts) == 3 and parts[0] == "camera" and parts[2] == "read"


class TransportError(DeviceFault):
    kind = "transport"


class TransportUnavailable(TransportError):
    """The remote daemon cannot be reached over the transport."""

    kind = "transport_unavailable"


class Transport(ABC):
    """Server side."""

    @abstractmethod
    def serve(self, service: "RobotService", bind: str, *, token: str | None = None) -> None:
        """Serve until ``stop()`` is called (blocking)."""

    @abstractmethod
    def start(self, service: "RobotService", bind: str, *, token: str | None = None) -> None:
        """Serve in the background and return immediately."""

    @abstractmethod
    def stop(self, grace_s: float = 1.0) -> None: ...


class TransportClient(ABC):
    _last_rtt_ms: float | None = None

    @abstractmethod
    def open(self) -> None: ...

    @abstractmethod
    def close(self) -> None: ...

    @property
    @abstractmethod
    def is_open(self) -> bool: ...

    @abstractmethod
    def call(self, method: str, **kwargs: Any) -> Any:
        """Raise the remote ``DeviceError`` subclass re-typed, or ``TransportError``."""

    def call_with_deadline(self, method: str, deadline_s: float, **kwargs: Any) -> Any:
        """``call`` that may wait at least ``deadline_s`` (transports without deadlines ignore it)."""
        return self.call(method, **kwargs)

    @abstractmethod
    def subscribe(self, method: str, rate_hz: float) -> Iterator[Any]:
        """Yield ``call(method)`` results at ``rate_hz`` until the client is closed."""

    @property
    def last_rtt_ms(self) -> float | None:
        return self._last_rtt_ms

    def __enter__(self) -> "TransportClient":
        self.open()
        return self

    def __exit__(self, *exc_info: object) -> None:
        self.close()
