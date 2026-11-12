"""Sim only: the old raw-twist SpaceMouse source (``device="spacemouse_twist"``) for the
ManiSkill teleop examples; see ``SpaceMouseTwistTeleopSource``. Real robots use
``device="spacemouse"`` (``real_world_rl.teleop.spacemouse.source``)."""
from __future__ import annotations

from typing import Any, Mapping, Optional, Sequence

from real_world_rl.teleop.spacemouse import (
    backend as _spacemouse_backend,  # registers "spacemouse"
)
from real_world_rl.devices.teleop import Teleop, TeleopConfig

from real_world_rl.action_contract import (
    LEGACY_ACTION_CONTRACT,
    get_env_action_contract,
)
from real_world_rl.teleop.core.contract import TeleopReading, TeleopSource
from real_world_rl.teleop.core.part import (
    DEFAULT_READ_TIMEOUT_S,
    TeleopPartSource,
)
from real_world_rl.teleop.core.registry import register_teleop_source

__all__ = ["SpaceMouseTwistTeleopSource"]

_EPISODE_END_BUTTON = 1


class _EpisodeEndOnButtonPress:
    """The device part, with the latched press of ``_EPISODE_END_BUTTON`` read as ``episode_end``;
    every other attribute is the part's."""

    def __init__(self, part: Any) -> None:
        self._part = part

    def __getattr__(self, name: str) -> Any:
        return getattr(self._part, name)

    def read(self, timeout_s: Optional[float] = None) -> Any:
        sample = self._part.read(timeout_s)
        if sample.events.get(_spacemouse_backend.BUTTON_NAMES[_EPISODE_END_BUTTON], False):
            sample.events["episode_end"] = True
        return sample


@register_teleop_source("spacemouse_twist")
class SpaceMouseTwistTeleopSource(TeleopSource):
    """Sim only (the ManiSkill teleop examples): raw twist, motion counts as intervention, button 0
    gripper, button 1 episode end, ``hold_s``. Not for real robots: a nudge of the puck takes over
    and every reading is the raw twist, with no command tracker; use ``device="spacemouse"`` there.

    A SpaceMouse on this machine, read by the device layer's ``SpaceMouseTeleop`` and mapped into
    the env action space by ``TeleopPartSource``. The twist is the raw puck reading (axis map
    ``[-y, x, z, -roll, -pitch, -yaw]`` of the pyspacemouse state, clipped to ``[-1, 1]``;
    ``invert_axes`` flips the sign of the listed output indices 0-5). The operator counts as
    intervening while the twist norm exceeds ``deadzone`` or a button is held, and ``hold_s``
    keeps it intervening that long after the last activity. Button 0 toggles the device's sticky
    gripper (open at start and after every ``reset()``, sent as ``+1`` open / ``-1`` closed); a
    press of button 1 raises ``episode_end``.

    Needs ``pyspacemouse>=2,<3``, imported when ``start()`` opens the device. ``close()``
    disconnects the device.
    """

    def __init__(
        self,
        *,
        env_contract: Optional[Mapping[str, Any]] = None,
        device_index: int = 0,
        deadzone: float = 1e-3,
        invert_axes: Sequence[int] = (),
        poll_interval_s: float = 0.005,
        hold_s: float = 0.0,
        read_timeout_s: float = DEFAULT_READ_TIMEOUT_S,
        device_table: Optional[str] = None,
    ) -> None:
        config = TeleopConfig(
            name="spacemouse",
            backend="spacemouse",
            hold_s=hold_s,
            options={
                "device_index": device_index,
                "deadzone": deadzone,
                "invert_axes": list(invert_axes),
                "poll_interval_s": poll_interval_s,
                **({} if device_table is None else {"device_table": str(device_table)}),
            },
        )
        self._part = Teleop.backend("spacemouse")(config, read_timeout_s=read_timeout_s)
        self._mapped = TeleopPartSource(
            _EpisodeEndOnButtonPress(self._part),
            env_contract=LEGACY_ACTION_CONTRACT if env_contract is None else env_contract,
            read_timeout_s=read_timeout_s,
        )

    @classmethod
    def from_env(cls, env: Any, **kwargs: Any) -> "SpaceMouseTwistTeleopSource":
        kwargs.setdefault("env_contract", get_env_action_contract(env))
        return cls(**kwargs)

    @property
    def action_mode(self) -> str:
        return self._mapped.action_mode

    @property
    def arm_action_dim(self) -> int:
        return self._mapped.arm_action_dim

    @property
    def has_gripper(self) -> bool:
        return self._mapped.has_gripper

    def start(self) -> None:
        self._part.connect()

    def poll(self) -> TeleopReading:
        return self._mapped.poll()

    def reset(self) -> None:
        self._mapped.reset()

    def close(self) -> None:
        self._part.disconnect()
