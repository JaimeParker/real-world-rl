"""Continuous HIL RLPD for contracts without discrete grippers."""
from rl_garden.algorithms.rlpd import RLPD
from rl_garden.buffers.demo_intervention import DemoInterventionMixin


class ContinuousHilRLPD(DemoInterventionMixin, RLPD):
    _compatible_checkpoint_algorithms = ("ContinuousHilRLPD", "RLPD")
