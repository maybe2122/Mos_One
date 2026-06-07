# Bridges the mos2026_2 HIM DirectRLEnv to the legged_gym VecEnv interface that
# HIMLoco's HIMOnPolicyRunner expects:
#   - attributes: num_obs, num_privileged_obs, num_one_step_obs, num_actions,
#     num_envs, device, episode_length_buf, max_episode_length, extras
#   - get_observations() / get_privileged_observations() / reset()
#   - step(actions) -> 7-tuple
#       (obs, priv, rew, dones, infos, termination_ids, termination_privileged_obs)
#
# The 6-step actor history is maintained here (newest-first), exactly like
# legged_gym's obs_buf, because the env emits one 45-dim frame per step.

import torch

from him_env_cfg import HISTORY_LENGTH, NUM_ONE_STEP_OBS, NUM_PRIVILEGED_OBS


class HIMVecEnvAdapter:
    def __init__(self, env):
        self.env = env  # Mos20262ClosedUsdHIMEnv
        self.num_one_step_obs = NUM_ONE_STEP_OBS
        self.num_obs = NUM_ONE_STEP_OBS * HISTORY_LENGTH          # 270
        self.num_privileged_obs = NUM_PRIVILEGED_OBS              # 51
        self.num_actions = env.cfg.action_space                   # 12
        self.num_envs = env.num_envs
        self.device = env.device
        self.extras = {}
        self.obs_buf = torch.zeros(self.num_envs, self.num_obs, device=self.device)
        self.critic_buf = torch.zeros(self.num_envs, self.num_privileged_obs, device=self.device)

    # episode_length_buf is reassigned by the runner (init_at_random_ep_len); proxy it.
    @property
    def episode_length_buf(self):
        return self.env.episode_length_buf

    @episode_length_buf.setter
    def episode_length_buf(self, value):
        self.env.episode_length_buf = value

    @property
    def max_episode_length(self):
        return self.env.max_episode_length

    def get_observations(self):
        return self.obs_buf

    def get_privileged_observations(self):
        return self.critic_buf

    def _push_history(self, current, done_ids):
        # newest block first; shift the rest back
        self.obs_buf = torch.cat((current, self.obs_buf[:, : -self.num_one_step_obs]), dim=1)
        if done_ids.numel() > 0:
            # fresh episode: fill history with the (post-reset) current frame
            self.obs_buf[done_ids] = current[done_ids].repeat(1, HISTORY_LENGTH)

    def reset(self, env_ids=None):
        obs_dict, _ = self.env.reset()
        current = obs_dict["policy"]
        self.obs_buf = current.repeat(1, HISTORY_LENGTH)
        self.critic_buf = obs_dict["critic"]
        return self.obs_buf, self.critic_buf

    def step(self, actions):
        obs_dict, rew, terminated, truncated, info = self.env.step(actions)
        dones = terminated | truncated
        done_ids = dones.nonzero(as_tuple=False).flatten()

        current = obs_dict["policy"]
        self.critic_buf = obs_dict["critic"]
        self._push_history(current, done_ids)

        # faithful pre-reset privileged obs for terminated envs (see him_env.py)
        pre = getattr(self.env, "_pre_reset_critic", None)
        if pre is not None and done_ids.numel() > 0:
            termination_privileged_obs = pre[done_ids].clone()
        else:
            termination_privileged_obs = self.critic_buf[done_ids].clone()

        self.extras = {"time_outs": truncated.to(torch.float)}
        if isinstance(info, dict) and "log" in info:
            self.extras["episode"] = info["log"]

        return (
            self.obs_buf,
            self.critic_buf,
            rew,
            dones.to(torch.long),
            self.extras,
            done_ids,
            termination_privileged_obs,
        )
