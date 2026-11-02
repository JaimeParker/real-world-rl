"""3Dconnexion SpaceMouse opened locally through the device layer."""
from __future__ import annotations

from typing import Any, Mapping, Optional, Sequence

from robot_infra.devices.backends.teleops import (
    spacemouse as _spacemouse_backend,  # noqa: F401  (registers "spacemouse")
)
from robot_infra.devices.teleop import Teleop, TeleopConfig

from rl_garden_real_world.action_contract import (
    LEGACY_ACTION_CONTRACT,
    get_env_action_contract,
)
from rl_garden_real_world.teleop.base import TeleopReading, TeleopSource
from rl_garden_real_world.teleop.part import DEFAULT_READ_TIMEOUT_S, TeleopPartSource
from rl_garden_real_world.teleop.registry import register_teleop_source


@register_teleop_source("spacemouse")
class SpaceMouseTeleopSource(TeleopSource):
    """A SpaceMouse on this machine, read by the device layer's ``SpaceMouseTeleop`` (a poll
    thread feeding ``read()``) and mapped into the env action space by ``TeleopPartSource``.

    Semantics are the device layer's: the puck drives a cartesian twist with axis map
    ``[-y, x, z, -roll, -pitch, -yaw]`` of the pyspacemouse state (the raw reading, clipped to
    ``[-1, 1]``; ``invert_axes`` flips the sign of the listed output indices 0-5); the operator counts as
    intervening while the twist norm exceeds ``deadzone`` or a button is held, and ``hold_s``
    keeps it intervening that long after the last activity; button 0 toggles the sticky gripper
    (open at start and after every ``reset()``, sent as ``+1`` open / ``-1`` closed); button 1
    raises ``episode_end``.

    Needs ``pyspacemouse>=2,<3``, imported when ``start()`` opens the device (the
    intervention wrapper calls it), so building the source works without the SDK and a missing
    SDK raises ``SdkMissing`` there. ``close()`` disconnects the device.
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
            },
        )
        self._part = Teleop.backend("spacemouse")(config, read_timeout_s=read_timeout_s)
        self._mapped = TeleopPartSource(
            self._part,
            env_contract=LEGACY_ACTION_CONTRACT if env_contract is None else env_contract,
            read_timeout_s=read_timeout_s,
        )

    @classmethod
    def from_env(cls, env: Any, **kwargs: Any) -> "SpaceMouseTeleopSource":
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
