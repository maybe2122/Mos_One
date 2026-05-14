# StackForce SimReady — 闭链 USD · Isaac Lab / Isaac Sim 导出工程

> 由 **StackForce SimReady** 自动导出,针对已在 Isaac Sim 中装配完成的**闭链(closed-chain)USD** 机器人,开箱即用。

---

## 一分钟上手

```bash
# 1. 激活你已有的 Isaac Lab 环境
conda activate <你自己的IsaacLab环境名称>

# 2. 安装导出工程
cd <exported_project>
uv pip install -e source/stackforce_simready_mos2026_2_closed_usd_closed_usd_lab

# 3. 检查环境注册情况
python scripts/list_envs.py
python scripts/inspect_usd.py --headless

# 4. 用 zero / random agent 跑一遍,确认 USD 加载正常
python scripts/zero_agent.py   --task StackForce-Mos20262ClosedUsd-ClosedUsd-v0 --headless --num_envs 4 --num_steps 200
python scripts/random_agent.py --task StackForce-Mos20262ClosedUsd-ClosedUsd-v0 --headless --num_envs 4 --num_steps 200

# 5. 开始 PPO 训练(先跑 20 个 iteration 做冒烟验证)
python scripts/rsl_rl/train.py --task StackForce-Mos20262ClosedUsd-ClosedUsd-v0 --headless --num_envs 16 --max_iterations 20
```

> **常用开关**
> - 想打开 Isaac Sim 窗口看效果:去掉 `--headless`
> - 想一直播放直到手动关窗口:`--num_steps 0`

---

## 可视化调试关节驱动

闭链 USD 的随机动作默认会保持若干步,避免每帧高频随机在视觉上互相抵消。需要肉眼检查关节驱动时,可以用正弦步态:

```bash
python scripts/random_agent.py \
    --task StackForce-Mos20262ClosedUsd-ClosedUsd-v0 \
    --num_envs 1 --num_steps 0 \
    --motion_mode gait --action_gain 0.1
```

---

## 推荐 Isaac Lab / Isaac Sim 环境

下面是已验证可用的版本组合:

| 组件 | 版本 |
| --- | --- |
| Python | 3.11 |
| Isaac Sim | 5.1.0 |
| Isaac Lab | v2.3.2 / pip 2.3.2.post1 |
| Torch | 2.7.0+cu128 |
| Torchvision | 0.22.0+cu128 |
| rsl_rl | 由 LeggedGym-Ex 0.3.0 提供 |

### 一键安装脚本

```bash
chmod +x scripts/setup_stackforce_isaac_lab_sim_env.sh
./scripts/setup_stackforce_isaac_lab_sim_env.sh
```

脚本默认创建 `env_isaaclab`。要改环境名:

```bash
ENV_NAME=my_isaaclab ./scripts/setup_stackforce_isaac_lab_sim_env.sh
```

### 多 GPU 机器:窗口无法显示

先确认显示器接在哪张 GPU 上,再用对应 GPU 启动:

```bash
nvidia-smi --query-gpu=index,pci.bus_id,name,display_active --format=csv,noheader

CUDA_VISIBLE_DEVICES=<display_active 为 Enabled 的 GPU index> \
    python scripts/rsl_rl/play.py \
        --task StackForce-Mos20262ClosedUsd-ClosedUsd-v0 \
        --checkpoint <checkpoint.pt> \
        --num_envs 1 --disable_resets
```

---

## 播放训练结果

```bash
# 找到最新的 checkpoint
find logs -name "*.pt" | sort | tail -n 1

# 播放
python scripts/rsl_rl/play.py \
    --task StackForce-Mos20262ClosedUsd-ClosedUsd-v0 \
    --checkpoint <上一步找到的 .pt 文件> \
    --num_envs 1 --disable_resets
```

---

## 闭链 USD 注意事项

闭链机器人**不要**再走 URDF Converter。本导出包直接使用:

```text
mos2026_2.usd
```

> 如果你把整个工程移到新目录,请在新目录重新执行:
> ```bash
> python -m pip install -e source/stackforce_simready_mos2026_2_closed_usd_closed_usd_lab
> ```

---

## 自定义 Reward

**1. 编辑 reward 实现**

```text
source/stackforce_simready_mos2026_2_closed_usd_closed_usd_lab/
  stackforce_simready_mos2026_2_closed_usd_closed_usd_lab/
    tasks/direct/mos2026_2_closed_usd/custom_rewards.py
```

在 `compute_custom_reward_terms(env)` 中返回新的 reward tensor。

**2. 调整 reward 权重**

```text
source/stackforce_simready_mos2026_2_closed_usd_closed_usd_lab/
  stackforce_simready_mos2026_2_closed_usd_closed_usd_lab/
    tasks/direct/mos2026_2_closed_usd/mos2026_2_closed_usd_env_cfg.py
```

把对应的 `reward_scales` 改成非零值。

---

## 来源

- **Robot:** mos2026 2
- **Source:** user-uploaded USD package
- **Pipeline:** Imported from the unified URDF/USD upload path
- **Mapping:** 导出可训练 Isaac Lab 工程前,需在 Mapping card 中设置 actuated joint names
