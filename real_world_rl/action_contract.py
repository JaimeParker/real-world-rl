"""Action-space contract shared by intervention, demo/snapshot loading and deploy.

A contract is the dict ``RealRobotEnv.action_contract`` exposes; the demo contract
(``DEMO_CONTRACT_KEYS``) is the subset persisted next to pkl transition files, so a
replayed transition can be checked against the env it is replayed into. Backends
without a contract (``franka_real``) yield ``None`` and keep their legacy behavior.

Action layout: an env drives ``num_arms`` arms named ``arm_names`` (one arm, ``["arm"]``,
by default), each with ``arm_action_dim`` arm dims. The action is the arm blocks in
``arm_names`` order followed, with ``gripper_in_action``, by one gripper dim per arm in the
same order: ``[arm_0, ..., arm_{n-1}, gripper_0, ..., gripper_{n-1}]``. So the last
``k = num_grippers_of(contract)`` dims are the grippers (``gripper_slice``) and the arm dims
are everything before them (``arm_block_slices``); consumers read the grippers as
``action[..., -k:]`` and the arms as ``action[..., :-k]``. A contract without ``num_arms`` /
``arm_names`` (older sidecars and checkpoints) is read as one arm named ``"arm"``.
"""
from __future__ import annotations

import json
import warnings
from pathlib import Path
from typing import Any, Mapping, Optional, Sequence

import numpy as np
import torch

# What a normalized action physically means: these must match for a replayed transition.
_PHYSICAL_KEYS = ("joint_limits", "xyz_scale_m", "rot_scale_rad", "joint_scale_rad", "state_fields")
_ARM_LAYOUT_KEYS = ("num_arms", "arm_names")
DEMO_CONTRACT_KEYS = (
    ("action_mode", "arm_action_dim", "gripper_in_action", "num_joints") + _ARM_LAYOUT_KEYS + _PHYSICAL_KEYS
)
_COMPARED_KEYS = ("action_mode", "arm_action_dim", "gripper_in_action") + _ARM_LAYOUT_KEYS + _PHYSICAL_KEYS
# Pre-contract convention: a 6-D cartesian twist plus a trailing gripper dimension.
LEGACY_ACTION_CONTRACT: Mapping[str, Any] = {
    "action_mode": "cartesian_delta",
    "arm_action_dim": 6,
    "gripper_in_action": True,
}
_SINGLE_ARM_NAMES = ("arm",)
_SIDECAR_SUFFIX = ".contract.json"


def get_env_action_contract(env: Any) -> Optional[dict]:
    """``env.action_contract`` through any wrapper chain, or ``None`` if the backend has none."""
    getter = getattr(env, "get_wrapper_attr", None)
    try:
        contract = getter("action_contract") if getter is not None else env.action_contract
    except AttributeError:
        return None
    return dict(contract) if isinstance(contract, Mapping) else None


def with_arm_layout(contract: Mapping[str, Any]) -> dict:
    """``contract`` with ``num_arms`` / ``arm_names`` filled in as one arm when it predates them."""
    return {"num_arms": 1, "arm_names": list(_SINGLE_ARM_NAMES), **contract}


def num_arms_of(contract: Mapping[str, Any]) -> int:
    return int(contract.get("num_arms", 1))


def num_grippers_of(contract: Mapping[str, Any]) -> int:
    """Gripper dims at the end of the action: one per arm with ``gripper_in_action``, else 0."""
    return num_arms_of(contract) if bool(contract["gripper_in_action"]) else 0


def action_dim_of(contract: Mapping[str, Any]) -> int:
    return num_arms_of(contract) * int(contract["arm_action_dim"]) + num_grippers_of(contract)


def arm_block_slices(contract: Mapping[str, Any]) -> list[slice]:
    """Each arm's block of the action, in ``arm_names`` order."""
    arm_dim = int(contract["arm_action_dim"])
    return [slice(index * arm_dim, (index + 1) * arm_dim) for index in range(num_arms_of(contract))]


def gripper_slice(contract: Mapping[str, Any]) -> slice:
    """The gripper dims (the last ``num_grippers_of(contract)``), one per arm in ``arm_names`` order."""
    action_dim = action_dim_of(contract)
    return slice(action_dim - num_grippers_of(contract), action_dim)


def env_num_grippers(env: Any) -> int:
    """``num_grippers_of`` ``env``'s contract; a contract-less backend has the legacy single
    trailing gripper."""
    return num_grippers_of(get_env_action_contract(env) or LEGACY_ACTION_CONTRACT)


def demo_contract_of(env: Any) -> Optional[dict]:
    """The persistable subset of ``env``'s contract, or ``None`` for contract-less backends."""
    contract = get_env_action_contract(env)
    if contract is None:
        return None
    return {key: contract[key] for key in DEMO_CONTRACT_KEYS if key in contract}


def describe_contract(contract: Mapping[str, Any]) -> str:
    num_arms = num_arms_of(contract)
    arms = (
        f", num_arms={num_arms}, arm_names={list(contract.get('arm_names', ()))!r}" if num_arms != 1 else ""
    )
    return (
        f"action_mode={contract['action_mode']!r}, arm_action_dim={contract['arm_action_dim']}, "
        f"gripper_in_action={contract['gripper_in_action']}{arms} "
        f"(action dim {action_dim_of(contract)})"
    )


def contract_sidecar_path(pkl_path: str | Path) -> Path:
    return Path(pkl_path).with_suffix(_SIDECAR_SUFFIX)


def write_contract_sidecar(pkl_path: str | Path, contract: Mapping[str, Any]) -> None:
    contract_sidecar_path(pkl_path).write_text(
        json.dumps({key: contract[key] for key in DEMO_CONTRACT_KEYS if key in contract}, sort_keys=True)
        + "\n",
        encoding="utf-8",
    )


def _contract_values_match(saved: Any, expected: Any) -> bool:
    if saved is None or expected is None:
        return saved is None and expected is None
    if isinstance(saved, str) or isinstance(expected, str):
        return saved == expected
    if isinstance(saved, (list, tuple)) and any(isinstance(item, str) for item in saved):
        return list(saved) == list(expected)  # state_fields (a JSON list vs the env's tuple)
    if isinstance(saved, bool) or isinstance(expected, bool):
        return saved == expected
    saved_array, expected_array = np.asarray(saved, dtype=np.float64), np.asarray(expected, dtype=np.float64)
    return saved_array.shape == expected_array.shape and bool(np.allclose(saved_array, expected_array))


def read_contract_sidecar(pkl_path: str | Path) -> Optional[dict]:
    path = contract_sidecar_path(pkl_path)
    if not path.exists():
        return None
    return json.loads(path.read_text(encoding="utf-8"))


def validate_transitions_against_contract(
    transitions: Sequence[Mapping[str, Any]],
    expected: Optional[Mapping[str, Any]],
    *,
    source: str,
    saved: Optional[Mapping[str, Any]] = None,
) -> None:
    """Raise ``ValueError`` if ``transitions`` were not recorded for the ``expected`` action space.

    ``saved`` is the sidecar metadata of ``source`` (``None`` for pkls written before contracts
    existed: only the action width is checked, and a warning notes the missing metadata). Keys
    the env contract has but an older sidecar lacks (limits, scales, state_fields) are skipped
    with a warning; a contract without ``num_arms`` / ``arm_names`` is read as one arm.
    """
    if expected is None:
        return
    expected = with_arm_layout(expected)
    if saved is not None:
        saved = with_arm_layout(saved)
        compared = [key for key in _COMPARED_KEYS if key in expected]
        missing = [key for key in compared if key not in saved]
        mismatched = [
            key
            for key in compared
            if key not in missing and not _contract_values_match(saved[key], expected[key])
        ]
        if mismatched:
            details = "; ".join(
                f"{key}: recorded={saved[key]!r}, env={expected[key]!r}" for key in mismatched
            )
            raise ValueError(
                f"{source}: recorded action contract does not match the env's "
                f"(differs in {mismatched}): {details}. recorded: {describe_contract(saved)}; "
                f"env: {describe_contract(expected)}"
            )
        if missing:
            warnings.warn(
                f"{source}'s action-contract metadata predates {missing}; those were not checked",
                stacklevel=2,
            )
    expected_dim = action_dim_of(expected)
    for index, transition in enumerate(transitions):
        actual_dim = int(torch.as_tensor(transition["action"]).shape[-1])
        if actual_dim != expected_dim:
            raise ValueError(
                f"{source}: transition {index} has action dim {actual_dim}, but the env expects "
                f"{expected_dim}: {describe_contract(expected)}"
            )
    if saved is None:
        warnings.warn(
            f"{source} has no action-contract metadata ({contract_sidecar_path(source).name} "
            f"missing); only the action dim ({expected_dim}) was checked, so a same-width but "
            "different action_mode would go unnoticed",
            stacklevel=2,
        )
