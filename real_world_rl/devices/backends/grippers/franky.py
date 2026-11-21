"""Franka Hand backend on franky: a libfranka gripper session independent of arm control.

A station gripper (``from_config``) commands the hand as HIL-SERL's ``franka_gripper_server``
does: close is a grasp to ``close_width_m`` (0.01 m) at ``speed_m_s`` (0.3 m/s) with
``grasp_force_n`` (130 N) and tolerances wide enough that closing on nothing succeeds, open
moves to ``open_width_m`` (default: fully open) at the same speed, and neither blocks the
caller. A worker thread runs one command at a time; while it runs, a newer command replaces any
queued one (the latest command wins), and a failed command raises from the next call.

franky's own ``*_async`` methods are not used: each waits for the previous command's future
before starting (``Gripper::setCurrentFuture``), so a toggle would block the env step.
"""
from __future__ import annotations

import logging
import threading
from types import ModuleType
from typing import TYPE_CHECKING, Any, Callable

import numpy as np

from real_world_rl.devices.backends.arms.franky import franky_connect_error, require_franky
from real_world_rl.devices.base import Connection
from real_world_rl.devices.errors import DeviceFault
from real_world_rl.devices.gripper import Gripper, GripperMode

if TYPE_CHECKING:
    from real_world_rl.devices.robot import RobotConfig

# Wide grasp tolerances make "close on nothing" a successful grasp instead of a failed command.
_GRASP_EPSILON_M = 1.0
# HIL-SERL franka_gripper_server.py: grasp(width 0.01, speed 0.3, force 130, epsilon 1) and
# move(0.09, 0.3); 0.09 is beyond the hand's stroke, i.e. fully open.
SERL_SPEED_M_S = 0.3
SERL_GRASP_FORCE_N = 130.0
SERL_CLOSE_WIDTH_M = 0.01
# How long disconnect waits for a running command to finish.
_WORKER_JOIN_TIMEOUT_S = 5.0
_FAULT_HINT = "check the hand in Desk (homing, errors) and that no other process holds the gripper"

logger = logging.getLogger(__name__)


class _CommandWorker:
    """Runs gripper commands on one thread: one at a time, the latest queued one wins, and the
    error of a failed command is kept for ``raise_error``."""

    def __init__(self, name: str) -> None:
        self._condition = threading.Condition()
        self._queued: tuple[str, Callable[[], None]] | None = None
        self._running = False
        self._error: BaseException | None = None
        self._stopping = False
        self._thread = threading.Thread(target=self._run, name=f"{name}-commands", daemon=True)
        self._thread.start()

    def submit(self, label: str, command: Callable[[], None]) -> None:
        with self._condition:
            if self._queued is not None:
                logger.debug("gripper command %s replaces queued %s", label, self._queued[0])
            self._queued = (label, command)
            self._condition.notify_all()

    def raise_error(self) -> None:
        with self._condition:
            error, self._error = self._error, None
        if error is not None:
            raise error

    def wait_idle(self, timeout_s: float | None = None) -> bool:
        with self._condition:
            return self._condition.wait_for(lambda: self._queued is None and not self._running, timeout_s)

    def stop(self) -> None:
        """Finish the running and queued command, then end the thread (bounded)."""
        with self._condition:
            self._stopping = True
            self._condition.notify_all()
        self._thread.join(_WORKER_JOIN_TIMEOUT_S)
        if self._thread.is_alive():
            logger.warning("Franka Hand command still running after %.0f s at disconnect",
                           _WORKER_JOIN_TIMEOUT_S)
        with self._condition:
            error, self._error = self._error, None
        if error is not None:
            logger.warning("Franka Hand command failed before disconnect: %s", error)

    def _run(self) -> None:
        while True:
            with self._condition:
                self._condition.wait_for(lambda: self._queued is not None or self._stopping)
                if self._queued is None:
                    return
                (_, command), self._queued = self._queued, None
                self._running = True
            try:
                command()
            except BaseException as error:  # kept for the next call
                with self._condition:
                    self._error = error
            finally:
                with self._condition:
                    self._running = False
                    self._condition.notify_all()


@Gripper.register("franky")
class FrankyGripper(Gripper):
    SDK = "franky"

    def __init__(
        self,
        name: str = "gripper",
        *,
        robot_ip: str,
        max_width_m: float = 0.08,
        mode: GripperMode = GripperMode.BINARY,
        binary_threshold: float = 0.5,
        speed: float = 0.05,
        force: float = 20.0,
        close_width_m: float = 0.0,
        open_width_m: float | None = None,
        asynchronous: bool = False,
        owner: Connection | None = None,
    ) -> None:
        """Constructed directly, commands block and use these conservative defaults;
        ``from_config`` (a station gripper) uses HIL-SERL's values and ``asynchronous=True``."""
        super().__init__(
            name,
            max_width_m=max_width_m,
            mode=mode,
            binary_threshold=binary_threshold,
            owner=owner,
        )
        self.robot_ip = robot_ip
        self.speed = speed
        self.force = force
        self.close_width_m = close_width_m
        self.open_width_m = open_width_m
        self.asynchronous = asynchronous
        self._worker: _CommandWorker | None = None

    @classmethod
    def from_config(
        cls, config: "RobotConfig", *, name: str = "gripper", owner: Connection | None = None
    ) -> "FrankyGripper":
        if owner is not None:
            raise ValueError(
                "franky gripper owns its own session; set gripper.shares_arm_session: false"
            )
        gripper = config.gripper

        def configured(value, default):
            return default if value is None else float(value)

        return cls(
            name,
            robot_ip=config.robot_ip,
            max_width_m=gripper.max_width_m,
            mode=GripperMode(gripper.mode),
            binary_threshold=gripper.binary_threshold,
            speed=configured(gripper.speed_m_s, SERL_SPEED_M_S),
            force=configured(gripper.grasp_force_n, SERL_GRASP_FORCE_N),
            close_width_m=configured(gripper.close_width_m, SERL_CLOSE_WIDTH_M),
            open_width_m=None if gripper.open_width_m is None else float(gripper.open_width_m),
            asynchronous=True,
            owner=owner,
        )

    @classmethod
    def require_sdk(cls) -> ModuleType:
        return require_franky(cls, cls.__name__)

    def _open(self) -> Any:
        self._forget_binary_command()
        franky = self.require_sdk()
        try:
            handle = franky.Gripper(self.robot_ip)
        except Exception as error:
            raise franky_connect_error(franky, error, self.robot_ip, self.name) from error
        if self.asynchronous:
            self._worker = _CommandWorker(self.name)
        return handle

    def _release(self, handle: Any) -> None:
        worker, self._worker = self._worker, None
        if worker is not None:
            worker.stop()

    def wait_idle(self, timeout_s: float | None = None) -> bool:
        """Wait until no command runs or is queued; True when idle (always, when blocking)."""
        return True if self._worker is None else self._worker.wait_idle(timeout_s)

    def _command(self, label: str, command: Callable[[], None]) -> None:
        """Run ``command`` now (blocking) or hand it to the worker; first raise a failed earlier
        command's error."""
        self._require_connected()
        self._raise_pending_error()
        if self._worker is None:
            command()
        else:
            self._worker.submit(label, command)

    def _raise_pending_error(self) -> None:
        if self._worker is None:
            return
        try:
            self._worker.raise_error()
        except DeviceFault:
            raise
        except Exception as error:
            raise DeviceFault(f"Franka Hand command failed: {error}", hint=_FAULT_HINT,
                              device=self.name) from error

    def open(self) -> None:
        self._require_connected()
        handle, speed, width = self._handle, self.speed, self.open_width_m
        if width is None:
            self._command("open", lambda: handle.open(speed))
        else:
            self._command("open", lambda: handle.move(width, speed))

    def close(self) -> None:
        self._require_connected()
        handle, franky = self._handle, self.require_sdk()
        width, speed, force = self.close_width_m, self.speed, self.force

        def grasp() -> None:
            try:
                handle.grasp(width, speed, force,
                             epsilon_inner=_GRASP_EPSILON_M, epsilon_outer=_GRASP_EPSILON_M)
            except franky.GripperException:
                pass  # nothing between the fingers: the hand still ends up closed
            except franky.CommandException as error:
                raise DeviceFault(
                    f"Franka Hand grasp command failed: {error}", hint=_FAULT_HINT, device=self.name,
                ) from error

        self._command("close", grasp)

    def move(self, width_norm: float) -> None:
        self._require_connected()
        handle, speed = self._handle, self.speed
        width = float(np.clip(width_norm, 0.0, 1.0)) * self._max_width_m
        self._command("move", lambda: handle.move(width, speed))

    @property
    def width(self) -> float:
        self._require_connected()
        self._raise_pending_error()
        return float(np.clip(self._handle.width / self._max_width_m, 0.0, 1.0))

    @property
    def is_grasped(self) -> bool | None:
        self._require_connected()
        return bool(self._handle.is_grasped)
