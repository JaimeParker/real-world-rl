"""Data contracts shared across envs, teleop, loops, demos, training and deploy.

``action``: the action-space contract; ``checkpoint``: keeping it across checkpoint loads;
``trajectory``: versioned archive records; ``outcomes``: episode outcomes.

Submodules are not imported here (``action``, ``checkpoint`` and ``trajectory`` import torch).
"""
