"""RLinf integration: importing ``rl_garden_real_world.integrations.rlinf.register`` registers
the ``RLGardenReal-v0`` Gymnasium id that RLinf's ``RealWorldEnv`` (``env_type: real``) builds
via ``gym.make("rl_garden_real_world.integrations.rlinf.register:RLGardenReal-v0", ...)``. Kept
import-free so the package does not pull gymnasium/robot_infra in until registration is
requested."""
