import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Random agent for StackForce closed-chain USD Isaac Lab environments.")
parser.add_argument("--disable_fabric", action="store_true", default=False)
parser.add_argument("--num_envs", type=int, default=None)
parser.add_argument("--task", type=str, default=None)
parser.add_argument("--num_steps", type=int, default=200, help="Number of steps to run. Use 0 to run until the window is closed.")
parser.add_argument("--motion_mode", choices=["gait", "held-random", "sine"], default="gait", help="Action source for visible actuator checks.")
parser.add_argument("--hold_steps", type=int, default=24, help="Steps to hold each random action. Higher values make closed-chain USD motion easier to see.")
parser.add_argument("--period_steps", type=int, default=120, help="Sine/gait period in environment steps. Larger values reduce contact impulses in visible closed-chain checks.")
parser.add_argument("--action_gain", type=float, default=0.1, help="Multiplier for random/sine/gait actions. Keep this small for effort-controlled closed-chain USD assets.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import gymnasium as gym
import torch

from isaaclab_tasks.utils import parse_env_cfg

import stackforce_simready_mos2026_2_closed_usd_closed_usd_lab.tasks  # noqa: F401


def make_gait_actions(env, step, phase_offsets):
    """Name-based visible motion for common quadruped closed-chain USD assets."""
    actions = torch.zeros(env.action_space.shape, device=env.unwrapped.device)
    names = list(getattr(env.unwrapped, "_actuated_joint_names", []))
    phase = (2.0 * 3.14159265 * step) / max(args_cli.period_steps, 1)
    sin_phase = torch.sin(torch.tensor(phase, device=env.unwrapped.device))
    cos_phase = torch.cos(torch.tensor(phase, device=env.unwrapped.device))
    matched = False
    for joint_id, name in enumerate(names):
        upper_name = name.upper()
        is_fr = "_FR" in upper_name or upper_name.endswith("FR") or upper_name.startswith("RF_")
        is_fl = "_FL" in upper_name or upper_name.endswith("FL") or upper_name.startswith("LF_")
        is_br = "_BR" in upper_name or upper_name.endswith("BR") or upper_name.startswith("RH_")
        is_bl = "_BL" in upper_name or upper_name.endswith("BL") or upper_name.startswith("LH_")
        leg_sign = 1.0 if is_fr or is_bl else -1.0 if is_fl or is_br else 0.0
        if leg_sign == 0.0:
            continue
        if "TRANSVERSALHIP" in upper_name:
            # Five-bar legs need same-leg front/back hip motors to move
            # together; independent random targets mostly fight the loop.
            actions[:, joint_id] = leg_sign * sin_phase
            matched = True
        elif "LATERALHIP" in upper_name:
            actions[:, joint_id] = 0.2 * leg_sign * cos_phase
            matched = True
        elif upper_name.endswith("_HFE") or "HIP" in upper_name:
            actions[:, joint_id] = leg_sign * sin_phase
            matched = True
        elif upper_name.endswith("_KFE") or "KNEE" in upper_name:
            actions[:, joint_id] = -0.7 * leg_sign * sin_phase
            matched = True
        elif upper_name.endswith("_HAA"):
            actions[:, joint_id] = 0.15 * leg_sign * cos_phase
            matched = True
    if matched:
        return actions
    return torch.sin(phase + phase_offsets).expand(env.action_space.shape)



def main():
    env_cfg = parse_env_cfg(
        args_cli.task, device=args_cli.device, num_envs=args_cli.num_envs, use_fabric=not args_cli.disable_fabric
    )
    env_cfg.action_clip = max(float(getattr(env_cfg, "action_clip", 1.0)), float(args_cli.action_gain))
    env = gym.make(args_cli.task, cfg=env_cfg)
    env.reset()
    steps = 0
    held_actions = torch.zeros(env.action_space.shape, device=env.unwrapped.device)
    phase_offsets = torch.linspace(0.0, 3.14159265, env.action_space.shape[-1], device=env.unwrapped.device).view(1, -1)
    while simulation_app.is_running():
        with torch.inference_mode():
            if args_cli.motion_mode == "gait":
                actions = make_gait_actions(env, steps, phase_offsets)
            elif args_cli.motion_mode == "sine":
                phase = (2.0 * 3.14159265 * steps) / max(args_cli.period_steps, 1)
                actions = torch.sin(phase + phase_offsets).expand(env.action_space.shape)
            else:
                if steps % max(args_cli.hold_steps, 1) == 0:
                    held_actions = 2 * torch.rand(env.action_space.shape, device=env.unwrapped.device) - 1
                actions = held_actions
            actions = args_cli.action_gain * actions
            env.step(actions)
        steps += 1
        if args_cli.num_steps > 0 and steps >= args_cli.num_steps:
            break
    print(f"RANDOM_AGENT_COMPLETED steps={steps} num_envs={env.unwrapped.num_envs}", flush=True)
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
