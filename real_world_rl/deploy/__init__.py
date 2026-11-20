"""Eval-only deployment on a real robot.

``checkpoint``: a trained rl-garden policy restored from a checkpoint.
``openpi``: an openpi policy served over websocket (``rwrl deploy-openpi``).

Submodules are not imported here, so importing one path does not pull in the others' dependencies.
"""
