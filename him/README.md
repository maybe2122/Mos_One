# HIM on mos2026_2 (closed-chain quadruped)

Applies HIMLoco's **HIM (Hybrid Internal Model)** algorithm to the `mos2026_2`
closed-chain robot, reusing the project's existing DirectRLEnv (robot, actuators,
rewards, terminations, terrain) unchanged. HIM only swaps the **policy / critic /
algorithm** and the **observation layout**.

Same approach as the Go2 HIM port in `HIMLoco/himloco_isaaclab/`.

## What HIM changes vs. the stock PPO setup

| | Stock PPO (`scripts/rsl_rl/`) | HIM (`him/`) |
|---|---|---|
| actor obs | 45, **includes** `root_lin_vel` | 45, **blind** (no lin vel) + 6-step history → 270 |
| critic | symmetric (same 45) | privileged 51 = 45 + true `base_lin_vel(3)` + disturbance(3) |
| velocity | fed directly | **estimated** by HIM estimator from history |
| policy/algo | `ActorCritic` / `PPO` | `HIMActorCritic` / `HIMPPO` (estimation + swap/contrastive loss) |
| net | `[256,256,128]` | `[512,256,128]` |

PPO optimizer hyperparams (clip, entropy, lr, gamma, lam, kl, epochs, …) are
identical between the two — see `him_cfg.py`.

### Observation layouts
```
one_step_obs (45) = [cmd(3), base_ang_vel(3), proj_gravity(3),
                     joint_pos_rel(12), joint_vel(12), last_action(12)]
critic_obs   (51) = one_step_obs(45) + base_lin_vel(3) + disturbance(3)
```
`cmd` is the env's fixed command `(commanded_lin_vel_xy, commanded_ang_vel_z)`.
The actor input is 6 stacked `one_step_obs` frames (newest-first), maintained in
`adapter.py`. The estimator reads `base_lin_vel` at `critic_obs[:, 45:48]`.

## Files
- `him_env_cfg.py` — `Mos20262ClosedUsdHIMEnvCfg`: sets `observation_space=45`,
  `state_space=51`, actor-noise scales. Subclasses the base env cfg.
- `him_env.py` — `Mos20262ClosedUsdHIMEnv`: rebuilds obs into the HIM layout and
  snapshots the pre-reset critic obs (for a faithful `termination_privileged_obs`).
- `adapter.py` — `HIMVecEnvAdapter`: legged_gym VecEnv interface + 6-step history
  + the 7-tuple `step()` the HIM runner expects.
- `him_cfg.py` — HIM hyperparameters (matches HIMLoco / the Go2 port).
- `train.py` — launches Isaac Sim, builds env+adapter, runs `HIMOnPolicyRunner`.
- `play.py` — loads a HIM checkpoint and plays it deterministically
  (estimator vel+latent + actor, 干净 obs，无域随机化).
- `him_rl/` — HIMLoco's rsl_rl fork (vendored; pure Python).

## Run
Use the shared Isaac Lab env (`env_isaaclab`, the one with isaaclab + torch).
`mos_one` does not need to be pip-installed — `train.py` adds
`source/mos_one` to `sys.path`.

```bash
# GUI (few envs)
../env_isaaclab/bin/python him/train.py --num_envs 256

# headless (full)；务必显式给 --max_iterations（默认值大到不会自己停）
../env_isaaclab/bin/python him/train.py --headless --num_envs 4096 --max_iterations 1500

# rough / curriculum terrain
../env_isaaclab/bin/python him/train.py --headless --num_envs 4096 --terrain curriculum

# 域随机化（sim2real 建议开；与 scripts/rsl_rl/train.py 同一套 EventCfg：
# 摩擦/质量/Kp-Kd/关节零位/周期推搡。首次开请先小 env 冒烟）
../env_isaaclab/bin/python him/train.py --headless --num_envs 4096 \
    --max_iterations 1500 --domain_rand

# 关闭 HIM 自带的 actor 观测噪声（默认开；训干净基线用）
../env_isaaclab/bin/python him/train.py ... --no_actor_noise
```

> HIM 的观测噪声由 `him_actor_noise`（分块均匀噪声，默认开）控制；基类的
> `--obs_noise_std` 高斯噪声路径被 HIM 的 `_get_observations` 覆写绕开，对 HIM 无效。

## Play（验证训练效果）

```bash
# GUI：自动挑 him/logs 下最新 checkpoint
../env_isaaclab/bin/python him/play.py

# headless 验证（必须给 --num_steps）
../env_isaaclab/bin/python him/play.py --headless --num_steps 500 --num_envs 4

# 指定 checkpoint / 地形（应与训练一致）
../env_isaaclab/bin/python him/play.py \
    --checkpoint him/logs/mos2026_2_him/<run>/model_1500.pt --terrain curriculum
```

Logs (TensorBoard) go to `him/logs/<experiment_name>/<timestamp>_<run_name>/`.
Verified: trains on the RTX 5090 with `Loss/Estimation Loss` and `Loss/Swap Loss`
updating (proves real HIM, not plain PPO).

> Note: Isaac Sim block-buffers stdout when redirected to a file, so the
> per-iteration table may only appear at the end — check the TensorBoard logs and
> the saved `model_*.pt` checkpoints to confirm progress mid-run.
