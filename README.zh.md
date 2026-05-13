# StackForce SimReady 闭链 USD Isaac Lab / Isaac Sim 导出工程

这个工程由 StackForce SimReady 导出，适用于已经在 Isaac Sim 中准备好的闭链 USD 机器人。

## 直接可复制的命令

```bash
conda activate <你自己的IsaacLab环境名称>
cd <exported_project>
python -m pip install -e source/stackforce_simready_mos2026_2_closed_usd_closed_usd_lab
python scripts/list_envs.py
python scripts/inspect_usd.py --headless
python scripts/zero_agent.py --task StackForce-Mos20262ClosedUsd-ClosedUsd-v0 --headless --num_envs 4 --num_steps 200
python scripts/random_agent.py --task StackForce-Mos20262ClosedUsd-ClosedUsd-v0 --headless --num_envs 4 --num_steps 200
python scripts/rsl_rl/train.py --task StackForce-Mos20262ClosedUsd-ClosedUsd-v0 --headless --num_envs 16 --max_iterations 20
```

闭链 USD 的随机动作会默认保持若干步，避免每帧高频随机在视觉上互相抵消。需要肉眼检查关节驱动时，可以使用正弦动作：

```bash
python scripts/random_agent.py --task StackForce-Mos20262ClosedUsd-ClosedUsd-v0 --num_envs 1 --num_steps 0 --motion_mode gait --action_gain 0.1
```

想打开 Isaac Sim 窗口时，去掉 `--headless`。想持续播放直到手动关闭窗口时，把 `--num_steps` 设为 `0`。

### 推荐 Isaac Lab / Isaac Sim 环境

本导出工程推荐使用下面这套已验证配置：

```text
Python 3.11
Isaac Sim 5.1.0
Isaac Lab v2.3.2 / pip 2.3.2.post1
Torch 2.7.0+cu128
Torchvision 0.22.0+cu128
LeggedGym-Ex 0.3.0 提供的 rsl_rl
```

导出包内已包含一键环境脚本：

```bash
chmod +x scripts/setup_stackforce_isaac_lab_sim_env.sh
./scripts/setup_stackforce_isaac_lab_sim_env.sh
```

脚本默认创建 `env_isaaclab`。如果你想改环境名：

```bash
ENV_NAME=my_isaaclab ./scripts/setup_stackforce_isaac_lab_sim_env.sh
```


如果是多 GPU 机器且窗口无法显示，请确认显示器连接在哪张 GPU 上：

```bash
nvidia-smi --query-gpu=index,pci.bus_id,name,display_active --format=csv,noheader
CUDA_VISIBLE_DEVICES=<display_active 为 Enabled 的 GPU index> python scripts/rsl_rl/play.py --task StackForce-Mos20262ClosedUsd-ClosedUsd-v0 --checkpoint <checkpoint.pt> --num_envs 1 --disable_resets
```

## 播放训练结果

```bash
find logs -name "*.pt" | sort | tail -n 1
python scripts/rsl_rl/play.py --task StackForce-Mos20262ClosedUsd-ClosedUsd-v0 --checkpoint <上一步找到的.pt文件> --num_envs 1 --disable_resets
```

## 闭链 USD 注意事项

闭链机器人不要再走 URDF converter。这个导出包直接使用：

```text
mos2026_2.usd
```

如果你移动整个工程，请在新目录重新运行：

```bash
python -m pip install -e source/stackforce_simready_mos2026_2_closed_usd_closed_usd_lab
```

## 自定义 reward

编辑：

```text
source/stackforce_simready_mos2026_2_closed_usd_closed_usd_lab/stackforce_simready_mos2026_2_closed_usd_closed_usd_lab/tasks/direct/mos2026_2_closed_usd/custom_rewards.py
```

然后在 `compute_custom_reward_terms(env)` 返回新的 reward tensor，并在：

```text
source/stackforce_simready_mos2026_2_closed_usd_closed_usd_lab/stackforce_simready_mos2026_2_closed_usd_closed_usd_lab/tasks/direct/mos2026_2_closed_usd/mos2026_2_closed_usd_env_cfg.py
```

把对应 `reward_scales` 改成非零值。

## 来源

- Robot: mos2026 2
- Source: user-uploaded USD package
- Imported from the unified URDF/USD upload path.
- Set the actuated joint names in the Mapping card before exporting a trainable Isaac Lab project.
