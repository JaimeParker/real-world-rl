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
import dataclasses
import hashlib
import warnings
from pathlib import Path
from typing import Any, Mapping, Optional, Sequence

import numpy as np
import torch

# What a normalized action physically means: these must match for a replayed transition.
_PHYSICAL_KEYS = ("joint_limits", "xyz_scale_m", "rot_scale_rad", "joint_scale_rad", "state_fields")
_ARM_LAYOUT_KEYS = ("num_arms", "arm_names")
DEMO_CONTRACT_KEYS = (
    ("action_mode", "arm_action_dim", "gripper_in_action", "num_joints") + _ARM_LAYOUT_KEYS + _PHYSICAL_KEYS + ("contract_version", "experiment", "effective_control")
)
_COMPARED_KEYS = ("action_mode", "arm_action_dim", "gripper_in_action") + _ARM_LAYOUT_KEYS + _PHYSICAL_KEYS + ("experiment",)
# Experiment entries that are saved and shown but never compared: the reward sources and the
# station facts the CLI owns. None of them changes what the policy sees or how its actions are
# interpreted; camera names and order are compared through ``observation`` (its entries and
# ``camera_order``) and ``image_size``.
_PROVENANCE_EXPERIMENT_KEYS = ("sources", "safety_box", "home_joints", "reset_settle_s", "max_episode_steps", "cameras")
# ``sources`` entries that change the action layout (which dims the operator drives): compared.
_COMPARED_SOURCE_KEYS = ("teleop_selected_parts", "teleop_idle_behavior")
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


def describe_contract(contract: Mapping[str, Any], *, with_provenance: bool = False) -> str:
    """One-line summary; ``with_provenance`` appends the experiment's ``experiment_provenance``."""
    num_arms = num_arms_of(contract)
    arms = (
        f", num_arms={num_arms}, arm_names={list(contract.get('arm_names', ()))!r}" if num_arms != 1 else ""
    )
    provenance = experiment_provenance(contract.get("experiment")) if with_provenance else {}
    return (
        f"action_mode={contract['action_mode']!r}, arm_action_dim={contract['arm_action_dim']}, "
        f"gripper_in_action={contract['gripper_in_action']}{arms} "
        f"(action dim {action_dim_of(contract)})"
        + (f", provenance (not compared)={provenance!r}" if provenance else "")
    )


def compared_experiment(spec: Any) -> Any:
    """The part of an experiment spec that must match between recording, training and
    deployment (per arm under ``arms``): everything except ``experiment_provenance``, with the
    action-layout ``sources`` entries kept under ``teleop``."""
    if not isinstance(spec, Mapping):
        return spec
    compared = {key: value for key, value in spec.items() if key not in _PROVENANCE_EXPERIMENT_KEYS}
    sources = spec.get("sources") or {}
    teleop = {key: sources[key] for key in _COMPARED_SOURCE_KEYS if key in sources}
    if teleop:
        compared["teleop"] = teleop
    if isinstance(spec.get("arms"), Mapping):
        compared["arms"] = {name: compared_experiment(arm) for name, arm in spec["arms"].items()}
    return compared


def experiment_provenance(spec: Any) -> dict:
    """The saved-but-never-compared part of an experiment spec (per arm under ``arms``)."""
    if not isinstance(spec, Mapping):
        return {}
    provenance = {key: spec[key] for key in _PROVENANCE_EXPERIMENT_KEYS if key in spec}
    arms = spec.get("arms")
    if isinstance(arms, Mapping):
        per_arm = {name: experiment_provenance(arm) for name, arm in arms.items()}
        if any(per_arm.values()):
            provenance["arms"] = per_arm
    return provenance


def _compared_contract(contract: Mapping[str, Any]) -> dict:
    """``contract`` with its ``experiment`` reduced to ``compared_experiment``."""
    if "experiment" not in contract:
        return dict(contract)
    return {**contract, "experiment": compared_experiment(contract["experiment"])}


def contract_sidecar_path(pkl_path: str | Path) -> Path:
    return Path(pkl_path).with_suffix(_SIDECAR_SUFFIX)


def write_contract_sidecar(pkl_path: str | Path, contract: Mapping[str, Any]) -> None:
    contract_sidecar_path(pkl_path).write_text(
        json.dumps(json_normalize({key: contract[key] for key in DEMO_CONTRACT_KEYS if key in contract}), sort_keys=True)
        + "\n",
        encoding="utf-8",
    )


def _contract_values_match(saved: Any, expected: Any) -> bool:
    if isinstance(saved, Mapping) and isinstance(expected, Mapping):
        saved, expected = semantic_experiment(saved), semantic_experiment(expected)
    if isinstance(saved, Mapping) or isinstance(expected, Mapping):
        return (isinstance(saved, Mapping) and isinstance(expected, Mapping)
                and set(saved) == set(expected)
                and all(_contract_values_match(saved[key], expected[key]) for key in saved))
    if isinstance(saved, (list, tuple)) and isinstance(expected, (list, tuple)):
        return len(saved) == len(expected) and all(_contract_values_match(a, b) for a, b in zip(saved, expected))
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
        saved_spec = saved.get("experiment", {})
        expected_spec = expected.get("experiment", {})
        if saved_spec and "observation" not in saved_spec and "observation" in expected_spec:
            if _requires_experiment_metadata(expected_spec):
                raise ValueError(f"{source}: legacy metadata cannot verify task/control/crop/depth observation processing")
            warnings.warn(f"{source}: legacy raw metadata has no final observation/source contract; checking configured default view", stacklevel=2)
            saved = {**saved, "experiment": {**saved_spec, "observation": expected_spec["observation"],
                                             "sources": expected_spec.get("sources", {})}}
        compared = [key for key in _COMPARED_KEYS if key in expected]
        missing = [key for key in compared if key not in saved]
        if "experiment" in missing and _requires_experiment_metadata(expected.get("experiment", {})):
            raise ValueError(f"{source}: legacy metadata cannot verify task/control/crop/depth experiment")
        saved_view, expected_view = _compared_contract(saved), _compared_contract(expected)
        mismatched = [
            key
            for key in compared
            if key not in missing and not _contract_values_match(saved_view[key], expected_view[key])
        ]
        if mismatched:
            details = "; ".join(
                f"{key}: recorded={saved_view[key]!r}, env={expected_view[key]!r}" for key in mismatched
            )
            raise ValueError(
                f"{source}: recorded action contract does not match the env's "
                f"(differs in {mismatched}): {details}. recorded: "
                f"{describe_contract(saved, with_provenance=True)}; "
                f"env: {describe_contract(expected, with_provenance=True)}"
            )
        if missing:
            warnings.warn(
                f"{source}'s action-contract metadata predates {missing}; those were not checked",
                stacklevel=2,
            )
    if saved is None and _requires_experiment_metadata(expected.get("experiment", {})):
        raise ValueError(f"{source}: missing metadata cannot verify task/control/crop/depth experiment")
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


def json_normalize(value):
    """Canonical JSON representation shared by configs, wire metadata and sidecars."""
    if dataclasses.is_dataclass(value): value = dataclasses.asdict(value)
    if isinstance(value, Mapping): return {str(key): json_normalize(item) for key, item in sorted(value.items())}
    if isinstance(value, (tuple, list)): return [json_normalize(item) for item in value]
    if isinstance(value, np.ndarray): return value.tolist()
    if isinstance(value, torch.Tensor): return value.detach().cpu().tolist()
    if isinstance(value, np.generic): return value.item()
    if isinstance(value, float) and not np.isfinite(value): return str(value)
    return value


def _requires_experiment_metadata(spec):
    """Whether ``spec`` changes the observation or action semantics, so that data without
    experiment metadata cannot be checked against it. Reward sources (a classifier) do not."""
    if not isinstance(spec, Mapping): return False
    task = spec.get("task", {})
    if task.get("name", "none") != "none" or spec.get("control") or spec.get("depth_cameras"):
        return True
    for preprocessing in spec.get("preprocess", {}).values():
        if any(value is not None for value in preprocessing.values()): return True
    return any(_requires_experiment_metadata(value) for value in spec.get("arms", {}).values())


def experiment_contract_of(env):
    contract = get_env_action_contract(env)
    if contract is None: return None
    return semantic_experiment(json_normalize({"schema_version": 2,
                           "action": {key: contract[key] for key in DEMO_CONTRACT_KEYS
                                      if key in contract and key not in ("experiment", "effective_control", "contract_version")},
                           "experiment": contract.get("experiment", {})}))


def experiment_hash(env):
    """Hash of the compared part of ``experiment_contract_of(env)``; provenance does not enter it."""
    contract = experiment_contract_of(env)
    if contract is None: return None
    compared = _compared_contract(contract)
    return hashlib.sha256(json.dumps(compared, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()


def effective_experiment_of(env):
    contract = experiment_contract_of(env)
    if contract is None: return None
    raw = get_env_action_contract(env) or {}
    return {**contract, "runtime_control": json_normalize(getattr(env, "runtime_control", {})),
            "provenance": json_normalize(raw.get("experiment", {}))}


def validate_experiment(env, saved):
    """Validate a saved requested contract now; enforce effective control before reset moves."""
    if saved is None:
        contract = get_env_action_contract(env) or {}
        if _requires_experiment_metadata(contract.get("experiment", {})):
            raise ValueError("saved experiment metadata is missing for task/control/crop/depth configuration")
        return
    expected = {key: value for key, value in saved.items() if key not in ("runtime_control", "provenance")}
    actual = experiment_contract_of(env)
    if actual is None:
        raise ValueError("saved experiment contract/hash differs from the constructed environment, which has none")
    saved_view, actual_view = _compared_contract(expected), _compared_contract(actual)
    if not _contract_values_match(saved_view, actual_view):
        differing = [
            f"{part}.{key}"
            for part in ("action", "experiment")
            for key in sorted(set(saved_view.get(part) or {}) | set(actual_view.get(part) or {}))
            if not _contract_values_match((saved_view.get(part) or {}).get(key), (actual_view.get(part) or {}).get(key))
        ]
        raise ValueError(
            f"saved experiment contract/hash differs from the constructed environment (differs in {differing}); "
            f"provenance (not compared): saved={experiment_provenance(expected.get('experiment'))!r}, "
            f"env={experiment_provenance(actual.get('experiment'))!r}"
        )
    metadata = saved.get("runtime_control", {})
    current = env
    while getattr(current, "env", None) is not None:
        current = current.env
    arms = current.arms if hasattr(current, "arms") else {"arm": current}
    for name, arm in arms.items():
        if metadata.get(name): arm._expected_control_metadata = metadata[name]


def save_experiment_metadata(env, path):
    """Atomically save requested identity and resolved runtime provenance beside artifacts."""
    import os
    import tempfile
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {"experiment_hash": experiment_hash(env), "experiment_contract": experiment_contract_of(env),
               "effective_experiment": effective_experiment_of(env)}
    descriptor, temporary = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    try:
        with os.fdopen(descriptor, "w") as stream:
            json.dump(json_normalize(payload), stream, sort_keys=True, allow_nan=False)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary): os.unlink(temporary)
    return payload


def semantic_experiment(value):
    """Model contents identify an experiment; filesystem locations remain provenance."""
    if isinstance(value, Mapping):
        return {key: semantic_experiment(item) for key, item in value.items()
                if key != "classifier_checkpoints" and not (key == "path" and "content_sha256" in value)}
    if isinstance(value, (tuple, list)): return [semantic_experiment(item) for item in value]
    return value
