from __future__ import annotations

import torch


def compute_custom_reward_terms(env) -> dict[str, torch.Tensor]:
    """返回此闭链 USD 任务的额外**未缩放**奖励项。

    约定：返回值是「未乘权重」的原始项，权重由 cfg.reward_scales[key] 决定
    （见 env._get_rewards 的 custom 项循环）。某项不想生效时把权重设 0 即可。
    """

    terms: dict[str, torch.Tensor] = {
        "custom_reward": torch.zeros(env.num_envs, dtype=torch.float, device=env.device),
    }

    # --- 力矩惩罚 sum(τ²)（对应 todo.md：力矩/电流不足的「根因」修复）-------------
    # 评估发现策略学出「贴力矩上限硬走」（shank 关节 83~84% 时间 ≥90% 上限）。
    # 当前 reward_scales 无任何 torque penalty → PPO 没有省力矩的动机。
    # 加 sum(τ²) 惩罚给 PPO 一个「少费力」的梯度；权重在 reward_scales["torque"]，
    # 默认 0.0（opt-in，不改变现有训练），建议起步 -2e-4 并用 eval_plot.py 对比
    # near_limit_frac 后微调。
    #
    # 取 applied_torque（已按 effort_limit 截断的实际下发力矩，与 play.py 一致），
    # 只统计 12 个受控关节。防御性 nan_to_num + clamp，避免闭链 PhysX 偶发爆值
    # 把惩罚拉爆（与 env._get_rewards 中其它项一致的处理风格）。
    robot_data = env._robot.data
    tau = getattr(robot_data, "applied_torque", None)
    if tau is None:  # 兜底：极少数版本字段名差异
        tau = getattr(robot_data, "computed_torque", None)
    if tau is not None:
        tau = tau[:, env._actuated_joint_ids]
        tau = torch.nan_to_num(tau, nan=0.0, posinf=0.0, neginf=0.0)
        # 单关节 |τ| 上限按电机峰值 ~24 N·m 估，平方后 ~576；12 关节求和上限 ~7e3。
        torque_sq = torch.clamp(torch.sum(torch.square(tau), dim=1), 0.0, 1.0e4)
    else:
        torque_sq = torch.zeros(env.num_envs, dtype=torch.float, device=env.device)
    terms["torque"] = torque_sq

    return terms
