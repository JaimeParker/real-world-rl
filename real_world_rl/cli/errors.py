"""CLI failures with stable exit codes, independent of device imports."""


class CliError(Exception):
    def __init__(self, message: str, *, code: int = 1, hint: str = "") -> None:
        super().__init__(message)
        self.message = message
        self.code = code
        self.hint = hint
