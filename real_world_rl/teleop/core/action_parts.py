"""Named spans over the existing arms-then-grippers action layout."""
import numpy as np
from real_world_rl.action_contract import arm_block_slices, num_grippers_of, with_arm_layout, action_dim_of


def action_parts(contract):
    names = with_arm_layout(contract)["arm_names"]
    result = {f"{name}.arm": block for name, block in zip(names, arm_block_slices(contract))}
    count = num_grippers_of(contract)
    if count:
        start = action_dim_of(contract) - count
        result.update({f"{name}.gripper": slice(start + i, start + i + 1) for i, name in enumerate(names)})
    return result


def part_mask(contract, parts=None):
    layout = action_parts(contract)
    mask = np.zeros(action_dim_of(contract), dtype=bool)
    for part in layout if parts is None else parts:
        if part not in layout:
            raise ValueError(f"unknown action part {part!r}; available: {list(layout)}")
        mask[layout[part]] = True
    return mask
