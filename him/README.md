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
- `him_rl/` — HIMLoco's rsl_rl fork (vendored; pure Python).

## Run
Use the shared Isaac Lab env (`env_isaaclab`, the one with isaaclab + torch).
`mos_one` does not need to be pip-installed — `train.py` adds
`source/mos_one` to `sys.path`.

```bash
# GUI (few envs)
/home/maybe/code/rl/env_isaaclab/bin/python him/train.py --num_envs 256

# headless (full)
/home/maybe/code/rl/env_isaaclab/bin/python him/train.py --headless --num_envs 4096

# rough / curriculum terrain
/home/maybe/code/rl/env_isaaclab/bin/python him/train.py --headless --num_envs 4096 --terrain curriculum
```

Logs (TensorBoard) go to `him/logs/<experiment_name>/<timestamp>_<run_name>/`.
Verified: trains on the RTX 5090 with `Loss/Estimation Loss` and `Loss/Swap Loss`
updating (proves real HIM, not plain PPO).

> Note: Isaac Sim block-buffers stdout when redirected to a file, so the
> per-iteration table may only appear at the end — check the TensorBoard logs and
> the saved `model_*.pt` checkpoints to confirm progress mid-run.
