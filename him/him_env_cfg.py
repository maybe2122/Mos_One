# HIM (Hybrid Internal Model) variant of the mos2026_2 closed-chain env config.
#
# Reuses the entire base env (robot, actuators, terrain, rewards, terminations)
# unchanged — HIM only changes the *observation layout* and the learning
# algorithm. The actor becomes blind (no base linear velocity); a privileged
# critic + HIM estimator recover the velocity instead.
#
# Single-step observation layouts (the 6-step actor history is maintained in the
# adapter, legged_gym-style):
#   one_step_obs (45) = [cmd(3), base_ang_vel(3), proj_gravity(3),
#                        joint_pos_rel(12), joint_vel(12), last_action(12)]
#   critic_obs   (51) = one_step_obs(45) + base_lin_vel(3) + disturbance(3)
# The HIM estimator reads the true base velocity at critic_obs[:, 45:48].

from isaaclab.utils import configclass

from stackforce_mos.tasks.direct.mos2026_2_closed_usd.mos2026_2_closed_usd_env_cfg import (
    Mos20262ClosedUsdEnvCfg,
)

NUM_ONE_STEP_OBS = 45
NUM_PRIVILEGED_OBS = 51
HISTORY_LENGTH = 6


@configclass
class Mos20262ClosedUsdHIMEnvCfg(Mos20262ClosedUsdEnvCfg):
    """mos2026_2 env exposing HIM's 45 (blind actor) / 51 (privileged critic) layout."""

    # DirectRLEnv buffer sizes: env emits a single 45-dim actor frame per step
    # (the adapter stacks 6 of them into the 270-dim actor input) and a 51-dim
    # privileged critic frame.
    observation_space = NUM_ONE_STEP_OBS
    state_space = NUM_PRIVILEGED_OBS

    # HIM adds observation noise to the *actor* obs only (the critic stays clean
    # so the estimator's velocity target is exact). Scales mirror the Go2 HIM
    # port; set to False to train on clean obs.
    him_actor_noise = True
    # Per-term uniform noise half-widths, applied to the actor obs blocks that
    # have physical noise in HIMLoco. cmd / last_action get no noise.
    him_noise_ang_vel = 0.2
    him_noise_gravity = 0.05
    him_noise_joint_pos = 0.01
    him_noise_joint_vel = 1.5
