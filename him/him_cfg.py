# HIM training hyperparameters for mos2026_2 — identical to HIMLoco's
# LeggedRobotCfgPPO config used for the Go2 HIM port (same algorithm,
# same network, same PPO schedule).

def get_him_train_cfg(experiment_name="mos2026_2_him", run_name="", num_steps_per_env=24,
                      max_iterations=200000, save_interval=50):
    return {
        "runner": {
            "policy_class_name": "HIMActorCritic",
            "algorithm_class_name": "HIMPPO",
            "num_steps_per_env": num_steps_per_env,
            "max_iterations": max_iterations,
            "save_interval": save_interval,
            "experiment_name": experiment_name,
            "run_name": run_name,
            "resume": False,
            "load_run": -1,
            "checkpoint": -1,
            "resume_path": None,
        },
        "algorithm": {
            "value_loss_coef": 1.0,
            "use_clipped_value_loss": True,
            "clip_param": 0.2,
            "entropy_coef": 0.01,
            "num_learning_epochs": 5,
            "num_mini_batches": 4,
            "learning_rate": 1.0e-3,
            "schedule": "adaptive",
            "gamma": 0.99,
            "lam": 0.95,
            "desired_kl": 0.01,
            "max_grad_norm": 1.0,
        },
        "policy": {
            "init_noise_std": 1.0,
            "actor_hidden_dims": [512, 256, 128],
            "critic_hidden_dims": [512, 256, 128],
            "activation": "elu",
        },
    }
