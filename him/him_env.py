# DirectRLEnv subclass of the mos2026_2 closed-chain env that emits HIM's
# observation layout instead of the default symmetric 45-dim obs.
#
# What changes vs. the base env:
#   - _get_observations returns BOTH a blind 45-dim actor obs (no base linear
#     velocity — HIM estimates it) and a privileged 51-dim critic obs
#     (= the 45 + true base_lin_vel(3) + disturbance(3) zeros).
#   - _reset_idx snapshots the pre-reset critic obs so the adapter can hand the
#     HIM runner a faithful `termination_privileged_obs` (DirectRLEnv, like the
#     manager-based env, resets terminated envs inside step() *before* computing
#     the returned observation).
# Everything else (rewards, terminations, robot, terrain, curriculum) is the
# base env's, untouched.

from __future__ import annotations

import torch

from mos_one.tasks.direct.mos2026_2_closed_usd.mos2026_2_closed_usd_env import (
    Mos20262ClosedUsdEnv,
)

from him_env_cfg import Mos20262ClosedUsdHIMEnvCfg


class Mos20262ClosedUsdHIMEnv(Mos20262ClosedUsdEnv):
    cfg: Mos20262ClosedUsdHIMEnvCfg

    _pre_reset_critic: torch.Tensor | None = None

    def __init__(self, cfg: Mos20262ClosedUsdHIMEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        # constant forward command broadcast to every env: [vx, vy, wz]
        cmd = (
            float(self.cfg.commanded_lin_vel_xy[0]),
            float(self.cfg.commanded_lin_vel_xy[1]),
            float(self.cfg.commanded_ang_vel_z),
        )
        self._him_command = torch.tensor(cmd, device=self.device).repeat(self.num_envs, 1)

    # ---- observation builders -------------------------------------------------
    def _him_actor_obs(self) -> torch.Tensor:
        """Blind 45-dim actor obs: [cmd, ang_vel, gravity, jpos_rel, jvel, action]."""
        jids = self._actuated_joint_ids
        ang_vel = self._robot.data.root_ang_vel_b
        gravity = self._robot.data.projected_gravity_b
        jpos_rel = (
            self._robot.data.joint_pos[:, jids]
            - self._robot.data.default_joint_pos[:, jids]
        )
        jvel = self._robot.data.joint_vel[:, jids]

        if getattr(self.cfg, "him_actor_noise", False):
            ang_vel = ang_vel + self._uniform_like(ang_vel, self.cfg.him_noise_ang_vel)
            gravity = gravity + self._uniform_like(gravity, self.cfg.him_noise_gravity)
            jpos_rel = jpos_rel + self._uniform_like(jpos_rel, self.cfg.him_noise_joint_pos)
            jvel = jvel + self._uniform_like(jvel, self.cfg.him_noise_joint_vel)

        obs = torch.cat([self._him_command, ang_vel, gravity, jpos_rel, jvel, self._actions], dim=-1)
        return torch.nan_to_num(obs, nan=0.0, posinf=0.0, neginf=0.0)

    def _him_critic_obs(self) -> torch.Tensor:
        """Privileged 51-dim critic obs: clean 45 + true base_lin_vel(3) + disturbance(3)."""
        jids = self._actuated_joint_ids
        jpos_rel = (
            self._robot.data.joint_pos[:, jids]
            - self._robot.data.default_joint_pos[:, jids]
        )
        disturbance = torch.zeros(self.num_envs, 3, device=self.device)
        obs = torch.cat(
            [
                self._him_command,                          # [0:3]
                self._robot.data.root_ang_vel_b,            # [3:6]
                self._robot.data.projected_gravity_b,       # [6:9]
                jpos_rel,                                   # [9:21]
                self._robot.data.joint_vel[:, jids],        # [21:33]
                self._actions,                              # [33:45]
                self._robot.data.root_lin_vel_b,            # [45:48]  <- HIM velocity target
                disturbance,                                # [48:51]
            ],
            dim=-1,
        )
        return torch.nan_to_num(obs, nan=0.0, posinf=0.0, neginf=0.0)

    @staticmethod
    def _uniform_like(x: torch.Tensor, half_width: float) -> torch.Tensor:
        return (torch.rand_like(x) * 2.0 - 1.0) * float(half_width)

    # ---- DirectRLEnv hooks ----------------------------------------------------
    def _get_observations(self) -> dict:
        policy = self._him_actor_obs()
        critic = self._him_critic_obs()
        # preserve the base env's per-step bookkeeping / debug viz
        self._previous_actions = self._actions.clone()
        if getattr(self.cfg, "show_velocity_arrows", True) and self.sim.has_gui():
            self._update_velocity_arrows()
        return {"policy": policy, "critic": critic}

    def _reset_idx(self, env_ids):
        # snapshot the true terminal critic obs before the reset overwrites state
        try:
            self._pre_reset_critic = self._him_critic_obs()
        except Exception:
            self._pre_reset_critic = None
        super()._reset_idx(env_ids)
