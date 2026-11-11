"""CLI failures with stable exit codes, independent of device imports."""


STOP_TRACE_FAIL_EXIT_CODE = 3
STOP_TRACE_BLOCKED_EXIT_CODE = 4
WORKSPACE_INVALID_EXIT_CODE = 5


class CliError(Exception):
    def __init__(self, message: str, *, code: int = 1, hint: str = "", required_fields=()) -> None:
        super().__init__(message)
        self.message = message
        self.code = code
        self.hint = hint
        self.required_fields = tuple(required_fields)
