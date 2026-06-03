from __future__ import annotations

import torch


def compute_custom_reward_terms(env) -> dict[str, torch.Tensor]:
    """返回此闭链 USD 任务的额外未缩放奖励项。"""

    return {"custom_reward": torch.zeros(env.num_envs, dtype=torch.float, device=env.device)}
