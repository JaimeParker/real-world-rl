"""Actionable migration errors for removed environment configuration."""
from __future__ import annotations

import sys
from collections.abc import Sequence

REMOVED_BACKEND = "franka_real"
REMOVAL_MESSAGE = (
    "env_backend franka_real was removed; use env_backend: real_robot with a "
    "serl_http (SERL bridge) or franky arm in the station yaml"
)


def check_backend_cli(argv: Sequence[str] | None = None) -> list[str]:
    """Reject obsolete backend flags and presets before the ordinary parser runs."""
    from rl_garden.common.effective_config import ConfigError, load_preset

    tokens = list(sys.argv[1:] if argv is None else argv)
    for index, token in enumerate(tokens):
        option, separator, inline = token.partition("=")
        option = option.replace("-", "_")
        value = inline if separator else tokens[index + 1] if index + 1 < len(tokens) else None
        if option.startswith(f"__{REMOVED_BACKEND}.") or (
            option == "__env_backend" and value == REMOVED_BACKEND
        ):
            raise ConfigError(REMOVAL_MESSAGE)
        if option == "__config" and value is not None:
            values = load_preset(value).values
            if values.get("env_backend") == REMOVED_BACKEND or REMOVED_BACKEND in values:
                raise ConfigError(REMOVAL_MESSAGE)
    return tokens
