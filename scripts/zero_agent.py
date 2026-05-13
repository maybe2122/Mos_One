import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Zero agent for StackForce closed-chain USD Isaac Lab environments.")
parser.add_argument("--disable_fabric", action="store_true", default=False)
parser.add_argument("--num_envs", type=int, default=None)
parser.add_argument("--task", type=str, default=None)
parser.add_argument("--num_steps", type=int, default=200, help="Number of steps to run. Use 0 to run until the window is closed.")




AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import gymnasium as gym
import torch

from isaaclab_tasks.utils import parse_env_cfg

import stackforce_simready_mos2026_2_closed_usd_closed_usd_lab.tasks  # noqa: F401



def main():
    env_cfg = parse_env_cfg(
        args_cli.task, device=args_cli.device, num_envs=args_cli.num_envs, use_fabric=not args_cli.disable_fabric
    )
    
    env = gym.make(args_cli.task, cfg=env_cfg)
    env.reset()
    steps = 0
    
    
    while simulation_app.is_running():
        with torch.inference_mode():
            actions = torch.zeros(env.action_space.shape, device=env.unwrapped.device)
            env.step(actions)
        steps += 1
        if args_cli.num_steps > 0 and steps >= args_cli.num_steps:
            break
    print(f"ZERO_AGENT_COMPLETED steps={steps} num_envs={env.unwrapped.num_envs}", flush=True)
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
