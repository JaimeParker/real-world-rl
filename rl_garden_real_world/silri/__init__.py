"""SiLRI (HIL-RL) actor/learner loops. Sync goes through the HIL-SERL sync classes."""
from rl_garden_real_world.silri.actor_loop import SilriActorLoop
from rl_garden_real_world.silri.learner_loop import SilriLearnerLoop

__all__ = ["SilriActorLoop", "SilriLearnerLoop"]
