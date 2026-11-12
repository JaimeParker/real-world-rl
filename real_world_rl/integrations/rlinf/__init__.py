"""RLinf integration: importing ``real_world_rl.integrations.rlinf.register`` registers
the ``RLGardenReal-v0`` Gymnasium id that RLinf's ``RealWorldEnv`` (``env_type: real``) builds
via ``gym.make("real_world_rl.integrations.rlinf.register:RLGardenReal-v0", ...)``. Kept
import-free so the package does not pull gymnasium/the device layer in until registration is
requested."""
