"""Validated staged resets shared by single-arm and bimanual environments."""
from __future__ import annotations
import time
import numpy as np


class TaskRuntime:
    def __init__(self, config, arm_names):
        self.config, self.arm_names = config, arm_names

    def _plan(self, arm_name, env, rng):
        reset = self.config.arm_resets.get(arm_name, self.config.reset)
        plan = []
        if reset.lift_m:
            pose = np.asarray(env.robot.arm.read_state()["tcp_pose"], dtype=float).copy()
            pose[2] += reset.lift_m
            plan.append(("pose", pose))
        home = np.asarray(reset.home_joints if reset.home_joints is not None else env._home_joints(), dtype=float)
        plan.append(("joints", home))
        jitter = rng.uniform(-np.asarray(reset.xyz_jitter_m), np.asarray(reset.xyz_jitter_m))
        for waypoint in reset.waypoints:
            pose = np.asarray(waypoint, dtype=float).copy()
            pose[:3] += jitter
            plan.append(("pose", pose))
        if reset.gripper != "keep":
            if env.robot.gripper is None: raise ValueError("task reset gripper command requires a gripper")
            plan.append(("gripper", 1.0 if reset.gripper == "open" else 0.0))
        settle = env.config.reset_settle_s if reset.settle_s is None else reset.settle_s
        plan.append(("settle", settle))
        # Validate the entire sampled plan against both inherited and configured limits.
        for kind, target in plan:
            if kind == "joints":
                if target.shape != (env.config.num_joints,) or not np.isfinite(target).all():
                    raise ValueError("reset home_joints shape/values invalid")
                limits = env._joint_limits
                if limits is not None and (np.any(target < limits[:, 0]) or np.any(target > limits[:, 1])):
                    raise ValueError("reset home_joints outside joint limits")
            elif kind == "pose":
                if target.shape != (7,) or not np.isfinite(target).all() or abs(np.linalg.norm(target[3:])-1)>0.001:
                    raise ValueError("reset pose must be finite xyz + unit quaternion")
                box = env._effective_safety_box
                if box is not None and (np.any(target[:3] < box[:3]) or np.any(target[:3] > box[3:])):
                    raise ValueError("task reset waypoint outside safety_box")
        return plan

    @staticmethod
    def _run(env, command):
        kind, target = command
        if kind == "pose": env.robot.arm.move_to_pose(target, blocking=True)
        elif kind == "joints": env.robot.arm.move_to_joints(target, blocking=True)
        elif kind == "gripper": env._send_and_observe({"gripper": {"gripper": np.array([target])}})
        elif kind == "settle" and target: time.sleep(target)

    def reset(self, env, rng):
        arms = env.arms if hasattr(env, "arms") else {self.arm_names[0]: env}
        def every(call):
            if hasattr(env, "_on_every_arm"): return env._on_every_arm(call)
            return {self.arm_names[0]: call(self.arm_names[0], env)}
        try:
            every(lambda name, arm: arm._reset_begin())
            # All connections/metadata validation and all plans complete before any reset move.
            plans = {name: self._plan(name, arm, rng) for name, arm in arms.items()}
            # Phase alignment preserves lift -> home -> waypoints -> gripper -> settle barriers.
            phases = ["lift", "home", *[f"waypoint_{i}" for i in range(max(
                len(self.config.arm_resets.get(name, self.config.reset).waypoints) for name in arms))], "gripper", "settle"]
            aligned = {}
            for name, arm in arms.items():
                reset = self.config.arm_resets.get(name, self.config.reset)
                commands = iter(plans[name]); mapped = {}
                if reset.lift_m: mapped["lift"] = next(commands)
                mapped["home"] = next(commands)
                for i in range(len(reset.waypoints)): mapped[f"waypoint_{i}"] = next(commands)
                if reset.gripper != "keep": mapped["gripper"] = next(commands)
                mapped["settle"] = next(commands)
                aligned[name] = mapped
            for phase in phases:
                every(lambda name, arm: self._run(arm, aligned[name][phase]) if phase in aligned[name] else None)
            return every(lambda name, arm: arm._reset_finish())
        except BaseException:
            for arm in arms.values():
                try: arm.robot.arm.stop()
                except Exception: pass
            raise
