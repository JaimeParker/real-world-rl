"""CLI failures with stable exit codes, independent of device imports."""


SUCCESS_EXIT_CODE = 0  # Successful command or accepted JSON workspace report.
FAILURE_EXIT_CODE = 1  # Generic runtime failure.
USAGE_EXIT_CODE = 2  # Invalid arguments/configuration or declined confirmation.
DEVICE_EXIT_CODE = 1  # A DeviceError ended the run (real-world-rl-deploy); a run failure, not usage.
DOCTOR_UNRESOLVED_EXIT_CODE = 1  # FAIL, BLOCKED or PENDING_ACTION remains.
STOP_TRACE_FAIL_EXIT_CODE = 3  # Stop trace fails acceptance.
STOP_TRACE_BLOCKED_EXIT_CODE = 4  # Stop trace lacks required evidence.
WORKSPACE_INVALID_EXIT_CODE = 5  # Invalid workspace pose, text mode only.
ABORTED_EXIT_CODE = 130  # Setup interrupted by the operator.
ESCALATED_EXIT_CODE = 1  # Supervisor had to escalate stopping the child.
LAUNCH_FAILURE_EXIT_CODE = 127  # Supervisor could not start the child.


class CliError(Exception):
    def __init__(self, message: str, *, code: int = FAILURE_EXIT_CODE, hint: str = "", required_fields=(),
                 payload: dict | None = None, diagnostic: str | None = None) -> None:
        super().__init__(message)
        self.message = message
        self.code = code
        self.hint = hint
        self.required_fields = tuple(required_fields)
        self.payload = payload
        self.diagnostic = diagnostic
