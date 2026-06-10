import gymnasium as gym

from . import agents


gym.register(
    id="MosOne-Mos20262ClosedUsd-ClosedUsd-v0",
    entry_point=f"{__name__}.mos2026_2_closed_usd_env:Mos20262ClosedUsdEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.mos2026_2_closed_usd_env_cfg:Mos20262ClosedUsdEnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:Mos20262ClosedUsdPPORunnerCfg",
    },
)
