from __future__ import annotations

import torch


def compute_custom_reward_terms(env) -> dict[str, torch.Tensor]:
    """Return additional unscaled reward terms for this closed-chain USD task."""

    return {"custom_reward": torch.zeros(env.num_envs, dtype=torch.float, device=env.device)}
