"""In-process first layer of the robot watchdog: a thread that stops the arm when steps stop.

It shares the process (and the GIL) with the control loop, so it covers a slow or stuck Python
caller and a step blocked inside a device call that releases the GIL, but not a frozen process
(SIGSTOP, a hung extension holding the GIL, a killed process). The separate supervisor process
(``python -m robot_infra.supervise``) is the layer for those.
"""
from __future__ import annotations

import threading
import time
from typing import Any, Callable, Optional


class StepWatchdogTripped(RuntimeError):
    """The arm was stopped because no step checked in within the watchdog timeout."""


class StepWatchdog:
    """Calls ``arm.stop()`` once when ``check_in()`` has not been called for ``timeout_s``.

    Inactive until ``activate()`` (call it when a reset has finished); ``deactivate()`` before
    anything that legitimately blocks (a reset motion). After a trip every ``check_in()`` raises
    ``StepWatchdogTripped`` until the next ``activate()``.
    """

    def __init__(self, arm: Any, timeout_s: float = 0.3, *, clock: Callable[[], float] = time.monotonic) -> None:
        if not timeout_s > 0:
            raise ValueError(f"timeout_s must be positive, got {timeout_s}")
        self._arm = arm
        self.timeout_s = float(timeout_s)
        self._clock = clock
        self._condition = threading.Condition()
        self._active = False
        self._closed = False
        self._last_check_in = 0.0
        self._trip_message: Optional[str] = None
        self.trip_count = 0
        self._thread = threading.Thread(target=self._watch, name="StepWatchdog", daemon=True)
        self._thread.start()

    @property
    def tripped(self) -> bool:
        return self._trip_message is not None

    @property
    def trip_message(self) -> Optional[str]:
        return self._trip_message

    def activate(self) -> None:
        """Start timing from now and clear an earlier trip."""
        with self._condition:
            self._trip_message = None
            self._last_check_in = self._clock()
            self._active = True
            self._condition.notify_all()

    def deactivate(self) -> None:
        with self._condition:
            self._active = False
            self._condition.notify_all()

    def check_in(self) -> None:
        """Mark the loop alive; raise ``StepWatchdogTripped`` if the arm was stopped meanwhile."""
        with self._condition:
            if self._trip_message is not None:
                raise StepWatchdogTripped(self._trip_message)
            self._last_check_in = self._clock()

    def close(self, timeout_s: float = 1.0) -> None:
        with self._condition:
            self._closed = True
            self._active = False
            self._condition.notify_all()
        if self._thread is not threading.current_thread():
            self._thread.join(timeout=timeout_s)

    def _watch(self) -> None:
        with self._condition:
            while not self._closed:
                if not self._active or self._trip_message is not None:
                    self._condition.wait()
                    continue
                silence = self._clock() - self._last_check_in
                if silence <= self.timeout_s:
                    self._condition.wait(timeout=self.timeout_s - silence)
                    continue
                self._active = False
                # Set before stopping, so a check-in racing with the stop already raises.
                self._trip_message = (
                    f"no env step for {silence:.3f}s (watchdog limit {self.timeout_s}s); "
                    "arm.stop() was called"
                )
                self.trip_count += 1
                self._condition.release()
                try:
                    stop_error = self._stop_arm()
                finally:
                    self._condition.acquire()
                if stop_error is not None and self._trip_message is not None:
                    self._trip_message += f" and raised {stop_error!r}"

    def _stop_arm(self) -> Optional[BaseException]:
        try:
            self._arm.stop()
        except Exception as error:
            return error
        return None
