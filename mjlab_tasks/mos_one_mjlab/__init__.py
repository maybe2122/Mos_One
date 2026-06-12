"""Register the mos2026_2 velocity task with mjlab.

Importing this package registers:
  Mjlab-Velocity-Flat-MosOne

Use via the wrapper scripts in scripts/mjlab/ (they put this package on
sys.path inside mjlab's uv environment).
"""

import torch

from mjlab.rl import vecenv_wrapper
from mjlab.tasks.registry import register_mjlab_task
from mjlab.tasks.velocity.rl import VelocityOnPolicyRunner

from .env_cfg import mos_one_flat_env_cfg
from .rl_cfg import mos_one_ppo_runner_cfg

# --- closed-chain NaN quarantine (paired with env_cfg's `nan_state` term) ----
# mjlab computes rewards BEFORE resetting terminated envs, and a blowup's
# first step can poison sensor-derived obs (contact forces) while qpos is
# still finite — i.e. before `nan_detection` fires. rsl_rl hard-aborts on any
# NaN, so sanitize both rewards and obs at the RL boundary; the offending env
# is terminated+reset by `nan_state` within a step. This mirrors the Isaac
# env's nan_to_num + invalid_state pattern (bad env contributes nothing,
# training continues).
_orig_step = vecenv_wrapper.RslRlVecEnvWrapper.step


def _nan_to_num(t: torch.Tensor) -> torch.Tensor:
  return torch.nan_to_num(t, nan=0.0, posinf=0.0, neginf=0.0)


def _nan_safe_step(self, actions):
  obs, rewards, dones, extras = _orig_step(self, actions)
  obs = obs.apply(_nan_to_num)
  rewards = _nan_to_num(rewards)
  return obs, rewards, dones, extras


vecenv_wrapper.RslRlVecEnvWrapper.step = _nan_safe_step

register_mjlab_task(
  task_id="Mjlab-Velocity-Flat-MosOne",
  env_cfg=mos_one_flat_env_cfg(),
  play_env_cfg=mos_one_flat_env_cfg(play=True),
  rl_cfg=mos_one_ppo_runner_cfg(),
  runner_cls=VelocityOnPolicyRunner,
)
