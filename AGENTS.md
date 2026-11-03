# AGENTS.md

Operating guide for coding agents working on `real-world-rl`. Read
[README.md](README.md) for the methods, layout, installation and hardware guides.

## Layout

- This repository is the top level for real-robot work. `real_world_rl/`
  is the real-robot RL package; `robot_infra/` is the torch-free device layer
  (a second distribution, `rlgarden-robot-infra`, that never imports
  `rl_garden` or `real_world_rl`).
- Package layering: `real_world_rl/loops/` holds the actor, learner and sync
  classes (base modules `actor`, `learner`, `sync`, then one module per method);
  `algorithms/`, `policies/`, `envs/`, `teleop/` and `training/entrypoints/` are
  laid out as in rl-garden.
- `rl-garden/` is rl-garden as a git submodule, pinned to the commit the tests
  here pass against. Bump it deliberately: check out the new commit inside
  `rl-garden/`, run this repository's tests, then commit the new gitlink here
  on its own. Never leave the gitlink pointing at an untested commit.
- Inside `rl-garden/`, rl-garden's own [AGENTS.md](rl-garden/AGENTS.md) and its
  `.agents/` rules and runbooks apply. Changes to rl-garden are committed in the
  submodule first, then picked up here by bumping the gitlink.
- `real_world_rl` extends rl-garden only through the contract in
  `rl-garden/docs/guides/extending.md` (subclassing and the
  `rlgarden.env_backends` / `rlgarden.algorithms` entry points). Do not edit
  rl-garden to suit this package unless the change is a general rl-garden
  feature or fix.

## Tests

Run from this repository's root. No hardware or network is needed.

```bash
pip install -e rl-garden -e robot_infra -e .     # once, editable
python -m pytest tests robot_infra/tests -q

# without installing
PYTHONPATH=.:rl-garden:robot_infra python -m pytest tests robot_infra/tests -q
```

rl-garden's own suite runs inside the submodule, as its testing runbook
(`rl-garden/.agents/runbooks/testing.md`) describes.
