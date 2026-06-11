# Play / visually validate a trained HIM checkpoint on the mos2026_2 env.
#
# Mirrors him/train.py's env construction (blind 45-dim actor frame + 270-dim
# history via HIMVecEnvAdapter), loads a HIMActorCritic checkpoint and runs the
# deterministic inference policy (act_inference: estimator vel+latent + actor).
#
#   GUI:      ../env_isaaclab/bin/python him/play.py                  # 自动挑最新 ckpt
#   headless: ../env_isaaclab/bin/python him/play.py --headless --num_steps 500
#   指定模型:  ../env_isaaclab/bin/python him/play.py --checkpoint him/logs/mos2026_2_him/<run>/model_1500.pt

import argparse
import os
import sys

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.dirname(_THIS_DIR)
sys.path.insert(0, _THIS_DIR)
sys.path.insert(0, os.path.join(_REPO_ROOT, "source", "mos_one"))

from isaaclab.app import AppLauncher  # noqa: E402

parser = argparse.ArgumentParser(description="Play a trained HIM policy on mos2026_2.")
parser.add_argument("--checkpoint", type=str, default=None,
                    help="HIM checkpoint (.pt)。不给则自动挑 him/logs 下最新的 model_*.pt。")
parser.add_argument("--num_envs", type=int, default=20, help="并行环境数（看效果 1~20 即可）。")
parser.add_argument("--num_steps", type=int, default=0,
                    help="跑多少控制步；0 = 不限（GUI 模式关窗结束）。headless 必须 > 0。")
parser.add_argument(
    "--terrain", type=str, default="flat", choices=["flat", "rough", "curriculum"],
    help="地形（应与训练时一致）。",
)
parser.add_argument("--actor_noise", action="store_true",
                    help="播放时也加 HIM actor 观测噪声（默认关，干净播放）。")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()

if args.headless and args.num_steps <= 0:
    parser.error("--headless 模式必须给 --num_steps > 0")

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

# ---- imports that require the running app ----
import torch  # noqa: E402

from mos_one.tasks.direct.mos2026_2_closed_usd.mos2026_2_closed_usd_env_cfg import (  # noqa: E402
    CURRICULUM_TERRAIN_CFG,
    ROUGH_TERRAIN_CFG,
)
from him_env_cfg import Mos20262ClosedUsdHIMEnvCfg  # noqa: E402
from him_env import Mos20262ClosedUsdHIMEnv  # noqa: E402
from adapter import HIMVecEnvAdapter  # noqa: E402
from him_cfg import get_him_train_cfg  # noqa: E402
from him_rl.modules import HIMActorCritic  # noqa: E402

LOG_ROOT = os.path.join(_THIS_DIR, "logs")


def find_latest_checkpoint() -> str | None:
    """him/logs 下最新的 model_*.pt（按 run 目录 mtime，再按迭代号）。"""
    candidates = []
    for root, _dirs, files in os.walk(LOG_ROOT):
        for f in files:
            if f.startswith("model_") and f.endswith(".pt"):
                candidates.append(os.path.join(root, f))
    if not candidates:
        return None

    def sort_key(p: str):
        stem = os.path.basename(p)[len("model_"):-len(".pt")]
        try:
            it = int(stem)
        except ValueError:
            it = -1
        return (os.path.getmtime(os.path.dirname(p)), it)

    return max(candidates, key=sort_key)


def main():
    device = "cuda:0"

    ckpt_path = args.checkpoint or find_latest_checkpoint()
    if ckpt_path is None:
        raise SystemExit(f"[HIM-play] him/logs 下没有任何 model_*.pt，请先训练或用 --checkpoint 指定。")
    print(f"[HIM-play] checkpoint: {ckpt_path}", flush=True)

    env_cfg = Mos20262ClosedUsdHIMEnvCfg()
    env_cfg.scene.num_envs = args.num_envs
    env_cfg.sim.device = device
    env_cfg.events = None                       # 播放不开域随机化（干净观看）
    env_cfg.him_actor_noise = bool(args.actor_noise)

    if args.terrain == "rough":
        env_cfg.terrain.terrain_type = "generator"
        env_cfg.terrain.terrain_generator = ROUGH_TERRAIN_CFG
        env_cfg.terrain.max_init_terrain_level = None
        env_cfg.terrain_curriculum_enabled = False
    elif args.terrain == "curriculum":
        env_cfg.terrain.terrain_type = "generator"
        env_cfg.terrain.terrain_generator = CURRICULUM_TERRAIN_CFG
        env_cfg.terrain.max_init_terrain_level = 0
        env_cfg.terrain_curriculum_enabled = True
    else:
        env_cfg.terrain.terrain_type = "plane"
        env_cfg.terrain.terrain_generator = None
        env_cfg.terrain_curriculum_enabled = False

    env = Mos20262ClosedUsdHIMEnv(cfg=env_cfg)
    vec_env = HIMVecEnvAdapter(env)

    # 与训练同款网络结构（him_cfg 的 policy 段），加载权重后取确定性推理策略。
    policy_cfg = get_him_train_cfg()["policy"]
    actor_critic = HIMActorCritic(
        vec_env.num_obs, vec_env.num_privileged_obs, vec_env.num_one_step_obs,
        vec_env.num_actions, **policy_cfg,
    ).to(device)
    ckpt = torch.load(ckpt_path, map_location=device, weights_only=False)
    actor_critic.load_state_dict(ckpt["model_state_dict"])
    actor_critic.eval()
    policy = actor_critic.act_inference
    print(f"[HIM-play] iter={ckpt.get('iter')} obs={vec_env.num_obs} "
          f"actions={vec_env.num_actions} num_envs={args.num_envs} terrain={args.terrain}", flush=True)

    obs, _ = vec_env.reset()
    obs = obs.to(device)
    step = 0
    ep_rew = torch.zeros(vec_env.num_envs, device=device)
    with torch.inference_mode():
        while simulation_app.is_running():
            actions = policy(obs)
            obs, _priv, rew, dones, _infos, _tids, _tobs = vec_env.step(actions)
            obs = obs.to(device)
            ep_rew += rew.to(device)
            done_ids = (dones > 0).nonzero(as_tuple=False).flatten()
            if done_ids.numel() > 0:
                print(f"[HIM-play] step {step}: {done_ids.numel()} env(s) reset, "
                      f"mean ep_rew={ep_rew[done_ids].mean().item():.2f}", flush=True)
                ep_rew[done_ids] = 0.0
            step += 1
            if args.num_steps > 0 and step >= args.num_steps:
                break

    print(f"HIM_PLAY_COMPLETED steps={step} num_envs={args.num_envs}", flush=True)
    env.close()
    simulation_app.close()


if __name__ == "__main__":
    main()
