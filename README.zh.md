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

## 训练脚本参数 (`scripts/rsl_rl/train.py`)

| 参数 | 类型 | 默认值 | 说明 |
| --- | --- | --- | --- |
| `--task` | str | 必填 | Gym 任务 ID,本工程为 `StackForce-Mos20262ClosedUsd-ClosedUsd-v0` |
| `--num_envs` | int | 任务默认 | 并行环境数。显存吃紧先用 16,稳定后 4096 提速 |
| `--max_iterations` | int | 任务默认 (1500) | PPO 学习迭代次数,冒烟可用 20 |
| `--seed` | int | 任务默认 | 随机种子 |
| `--run_name` | str | `""` | 给本次训练加后缀,出现在日志目录名里,方便对比实验 |
| `--checkpoint` | str | `None` | 续训用的 `.pt` 路径(目前由 RSL-RL 内部恢复) |
| `--agent` | str | `rsl_rl_cfg_entry_point` | Hydra agent 配置入口,通常不用改 |
| `--terrain` | str | `flat` | 地形:`flat` / `rough` / `curriculum`(详见下表) |

### `--terrain` 选项

| 取值 | 地形 | 课程学习 | 用途 |
| --- | --- | --- | --- |
| `flat` | 平地 | 关 | 默认,新策略起步、调奖励 |
| `rough` | 程序生成高度场 | 关 | 直接训练崎岖地形,难度恒定 |
| `curriculum` | 平地 → 崎岖 + 楼梯 | 开 | 所有 env 从第 0 行起步,跑稳后自动升级难度 |

### AppLauncher 透传参数

`AppLauncher.add_app_launcher_args(parser)` 会注入 Isaac Sim 的标准参数,常用如下:

| 参数 | 说明 |
| --- | --- |
| `--headless` | 不启用 GUI,训练务必开启 |
| `--device cuda:0` | 指定 GPU,多卡机器需要 |
| `--enable_cameras` | 启用相机渲染(占显存,默认关) |
| `--livestream {0,1,2}` | 推流到 Omniverse Streaming Client / WebRTC,适合服务器训练时远程查看 |

### 完整示例

```bash
# 1. 冒烟跑 20 个 iteration,小 env 验证管线
python scripts/rsl_rl/train.py \
    --task StackForce-Mos20262ClosedUsd-ClosedUsd-v0 \
    --headless --num_envs 16 --max_iterations 20

# 2. 正式训练,带 run_name 方便日志对比
python scripts/rsl_rl/train.py \
    --task StackForce-Mos20262ClosedUsd-ClosedUsd-v0 \
    --headless --num_envs 4096 --max_iterations 1500 \
    --seed 42 --run_name baseline_flat

# 3. 课程学习地形上训练
python scripts/rsl_rl/train.py \
    --task StackForce-Mos20262ClosedUsd-ClosedUsd-v0 \
    --headless --num_envs 4096 --terrain curriculum \
    --run_name curriculum_v1
```

> 日志目录:`logs/rsl_rl/<experiment_name>/<时间戳>[_<run_name>]/`,checkpoint 落在 `model_final.pt` 与 `model_*.pt`。

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

一键脚本会自动找出 `logs/` 下最新的 `.pt` 并启动 play.py:

```bash
# 默认: 20 envs + 关闭 reset
./scripts/rsl_rl/play_latest.sh

# 想改播放参数,直接在后面追加,会原样转发给 play.py
./scripts/rsl_rl/play_latest.sh --num_envs 1 --num_steps 0 --disable_resets

# 想换任务或日志目录,用环境变量覆盖
TASK=StackForce-Mos20262ClosedUsd-ClosedUsd-v0 LOGS_DIR=logs \
    ./scripts/rsl_rl/play_latest.sh
```

如果想手动选 checkpoint,也可以直接调用底层命令:

```bash
find logs -name "*.pt" | sort | tail -n 1
python scripts/rsl_rl/play.py \
    --task StackForce-Mos20262ClosedUsd-ClosedUsd-v0 \
    --checkpoint <上一步找到的 .pt 文件> \
    --num_envs 20 --disable_resets
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
