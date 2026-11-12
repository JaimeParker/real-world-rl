"""The Pico as a raw EE twist, for the ManiSkill sim examples only (``device="pico_twist"``).

This is the former ``device="pico"``: ``EETwistTeleOpWrapper`` (``pico/bridge.py``) opens its ZMQ
subscriber at construction and turns hand-pose differences into a clipped twist in physical
units, which goes to the env as is (the sim examples normalise it to the controller's range).
Nothing limits how far that twist leads the robot, so it is not for real robots: there
``device="pico"`` drives the follower through the anchored pose tracker
(``real_world_rl.teleop.pico.source``).
"""
from __future__ import annotations

from typing import Any

from real_world_rl.teleop.core.contract import (
    TeleopReading,
    TeleopSource,
    reading_from_legacy_sample,
)
from real_world_rl.teleop.core.registry import register_teleop_source
from real_world_rl.teleop.pico.bridge import EETwistTeleOpWrapper

__all__ = ["PicoTwistTeleopSource"]

_ACTION_DIM = 7


@register_teleop_source("pico_twist")
class PicoTwistTeleopSource(TeleopSource):
    """Hand-pose twist (6-D) plus gripper, as ``EETwistTeleOpWrapper`` computes them.

    Declares ``cartesian_delta`` / 6 / gripper. ``kwargs`` go to ``EETwistTeleOpWrapper``
    (``zmq_url``, ``hand``, ``pos_scale``, ``rot_scale``, ``twist_limit``,
    ``intervention_threshold``); the ZMQ subscriber is opened at construction.
    """

    action_mode = "cartesian_delta"
    arm_action_dim = 6
    has_gripper = True
    DRIVES_ONE_ARM = True

    def __init__(self, **kwargs: Any) -> None:
        self._wrapper = EETwistTeleOpWrapper(device="pico", **kwargs)

    def poll(self) -> TeleopReading:
        return reading_from_legacy_sample(self._wrapper.poll(), action_dim=_ACTION_DIM)

    def reset(self, *, episode_end_pressed: bool = False) -> None:
        """``episode_end_pressed=True`` keeps a still-held episode-end button from firing again
        on the next poll (what recording scripts want right after they ended an episode)."""
        reset = getattr(self._wrapper, "reset", None)
        if not callable(reset):
            return
        if episode_end_pressed:
            reset(episode_end_pressed=True)
        else:
            reset()

    def close(self) -> None:
        close = getattr(self._wrapper, "close", None)
        if callable(close):
            close()
