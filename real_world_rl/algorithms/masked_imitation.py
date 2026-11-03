"""Human-dimension imitation objectives with legacy full-mask scaling."""
import torch


def human_mask(data, width=None):
    mask = getattr(data, "human_action_mask", None)
    if mask is None:
        mask = data.intervened.reshape(-1, 1).expand_as(data.actions)
    mask = mask.to(device=data.actions.device, dtype=torch.bool)
    return mask if width is None else mask[..., :width]


def masked_sum_mean(values, mask, *, reference_width=None):
    mask = mask.to(values.dtype)
    count = mask.sum(-1)
    width = values.shape[-1] if reference_width is None else reference_width
    per_row = (values * mask).sum(-1) * width / count.clamp_min(1)
    active = (count > 0).to(values.dtype)
    return (per_row * active).sum() / active.sum().clamp_min(1)


def marginal_gripper_ce(logits, labels, mask, num_grippers):
    from rl_garden.policies.rlpd_hybrid_policy import gripper_index_digits
    digits = gripper_index_digits(torch.arange(logits.shape[-1], device=logits.device), num_grippers)
    target = gripper_index_digits(labels, num_grippers)
    matches = ((digits[None] == target[:, None]) | ~mask[:, None]).all(-1)
    per_row = torch.logsumexp(logits, -1) - torch.logsumexp(logits.masked_fill(~matches, -torch.inf), -1)
    active = mask.any(-1).to(logits.dtype)
    return (per_row * active).sum() / active.sum().clamp_min(1)
