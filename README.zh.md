<div align="center">

# 🤖 StackForce SimReady

### 闭链 USD · Isaac Lab / Isaac Sim 导出工程

[![Isaac Sim](https://img.shields.io/badge/Isaac_Sim-5.1.0-76b900?style=for-the-badge&logo=nvidia&logoColor=white)](https://developer.nvidia.com/isaac-sim)
[![Isaac Lab](https://img.shields.io/badge/Isaac_Lab-v2.3.2-76b900?style=for-the-badge&logo=nvidia&logoColor=white)](https://isaac-sim.github.io/IsaacLab/)
[![Python](https://img.shields.io/badge/Python-3.11-3776ab?style=for-the-badge&logo=python&logoColor=white)](https://python.org)
[![PyTorch](https://img.shields.io/badge/PyTorch-2.7.0-ee4c2c?style=for-the-badge&logo=pytorch&logoColor=white)](https://pytorch.org)
[![License](https://img.shields.io/badge/License-Proprietary-blue?style=for-the-badge)](./NOTICE.closed-chain-source.md)

<br/>



[English](./README.en.md) ·
[快速开始](#-一分钟上手) ·
[训练参数](#-训练脚本参数) ·
[播放结果](#-播放训练结果) ·
[评估结果](#-评估训练结果) ·
[自定义奖励](#-自定义-reward)

</div>

---

## 📑 目录

- [🤖 StackForce SimReady](#-stackforce-simready)
    - [闭链 USD · Isaac Lab / Isaac Sim 导出工程](#闭链-usd--isaac-lab--isaac-sim-导出工程)
  - [📑 目录](#-目录)
  - [⚡ 一分钟上手](#-一分钟上手)
  - [🎯 训练脚本参数](#-训练脚本参数)
    - [🗺️ `--terrain` 选项](#️---terrain-选项)
    - [💡 完整示例](#-完整示例)
  - [🔍 可视化调试关节驱动](#-可视化调试关节驱动)
  - [🛠️ 推荐环境 \& 安装](#️-推荐环境--安装)
    - [已验证版本组合](#已验证版本组合)
    - [🚀 一键安装](#-一键安装)
  - [🎬 播放训练结果](#-播放训练结果)
  - [📊 评估训练结果](#-评估训练结果)
    - [⚙️ `eval.py` 参数](#️-evalpy-参数)
    - [📋 指标说明](#-指标说明)
    - [🧪 一键验证套件](#-一键验证套件)
  - [📦 闭链 USD 注意事项](#-闭链-usd-注意事项)
  - [🏆 自定义 Reward](#-自定义-reward)
    - [1️⃣ 编辑 Reward 实现](#1️⃣-编辑-reward-实现)
    - [2️⃣ 调整 Reward 权重](#2️⃣-调整-reward-权重)
  - [📌 来源](#-来源)

---

## ⚡ 一分钟上手

```bash
# 1️⃣ 激活你已有的 Isaac Lab 环境
conda activate <你自己的IsaacLab环境名称>

# 2️⃣ 安装导出工程
cd <exported_project>
uv pip install -e source/stackforce_mos

# 3️⃣ 检查环境注册情况
python scripts/list_envs.py
python scripts/inspect_usd.py --headless

# 4️⃣ 用 zero / random agent 跑一遍，确认 USD 加载正常
python scripts/zero_agent.py   --task StackForce-Mos20262ClosedUsd-ClosedUsd-v0 --headless --num_envs 4 --num_steps 200
python scripts/random_agent.py --task StackForce-Mos20262ClosedUsd-ClosedUsd-v0 --headless --num_envs 4 --num_steps 200

# 5️⃣ 开始 PPO 训练（先跑 20 个 iteration 做冒烟验证）
python scripts/rsl_rl/train.py \
    --task StackForce-Mos20262ClosedUsd-ClosedUsd-v0 \
    --headless --max_iterations 3000 \
    --terrain curriculum \
    --max_gpu_mem 10
```

> [!TIP]
> **常用开关**
> - 🖥️ 想打开 Isaac Sim 窗口看效果：去掉 `--headless`
> - 🔁 想一直播放直到手动关窗口：`--num_steps 0`

---

## 🎯 训练脚本参数

> `scripts/rsl_rl/train.py`

| 参数 | 类型 | 默认值 | 说明 |
|:---|:---:|:---:|:---|
| `--task` | `str` | **必填** | Gym 任务 ID，本工程为 `StackForce-Mos20262ClosedUsd-ClosedUsd-v0` |
| `--num_envs` | `int` | 任务默认 | 并行环境数。显存吃紧先用 `16`，稳定后 `4096` 提速 |
| `--max_iterations` | `int` | `1500` | PPO 学习迭代次数，冒烟可用 `20` |
| `--seed` | `int` | 任务默认 | 随机种子 |
| `--run_name` | `str` | `""` | 给本次训练加后缀，出现在日志目录名里，方便对比实验 |
| `--checkpoint` | `str` | `None` | 续训用的 `.pt` 路径（由 RSL-RL 内部恢复） |
| `--agent` | `str` | `rsl_rl_cfg_entry_point` | Hydra agent 配置入口，通常不用改 |
| `--terrain` | `str` | `flat` | 地形模式（详见下表） |
| `--max_gpu_mem` | `float` | `32.0` | 显存预算 (GB)。在 16/8 GB 卡上传入对应数值，PhysX 容量按比例缩放 |

### 🗺️ `--terrain` 选项

| 取值 | 地形 | 课程学习 | 用途 |
|:---:|:---|:---:|:---|
| `flat` | 🟩 平地 | ❌ | 默认，新策略起步、调奖励 |
| `rough` | 🏔️ 程序生成高度场 | ❌ | 直接训练崎岖地形，难度恒定 |
| `curriculum` | 📈 平地 → 崎岖 + 楼梯 | ✅ | 所有 env 从第 0 行起步，跑稳后自动升级难度 |

<details>
<summary><strong>🔧 AppLauncher 透传参数</strong></summary>

<br/>

`AppLauncher.add_app_launcher_args(parser)` 会注入 Isaac Sim 的标准参数，常用如下：

| 参数 | 说明 |
|:---|:---|
| `--headless` | 不启用 GUI，训练务必开启 |
| `--device cuda:0` | 指定 GPU，多卡机器需要 |
| `--enable_cameras` | 启用相机渲染（占显存，默认关） |
| `--livestream {0,1,2}` | 推流到 Omniverse Streaming Client / WebRTC，适合服务器训练时远程查看 |

</details>

### 💡 完整示例

```bash
# 1️⃣ 冒烟跑 20 个 iteration，小 env 验证管线
python scripts/rsl_rl/train.py \
    --task StackForce-Mos20262ClosedUsd-ClosedUsd-v0 \
    --headless --num_envs 16 --max_iterations 20

# 2️⃣ 正式训练，带 run_name 方便日志对比
python scripts/rsl_rl/train.py \
    --task StackForce-Mos20262ClosedUsd-ClosedUsd-v0 \
    --headless --num_envs 4096 --max_iterations 1500 \
    --seed 42 --run_name baseline_flat

# 3️⃣ 课程学习地形上训练
python scripts/rsl_rl/train.py \
    --task StackForce-Mos20262ClosedUsd-ClosedUsd-v0 \
    --headless --num_envs 4096 --terrain curriculum \
    --run_name curriculum_v1

# 4️⃣ 在 16GB / 8GB 显卡上训练：同时缩 PhysX 容量和 env 数
python scripts/rsl_rl/train.py \
    --task StackForce-Mos20262ClosedUsd-ClosedUsd-v0 \
    --headless --num_envs 2048 --max_gpu_mem 16 \
    --max_iterations 1500 --run_name gpu16

python scripts/rsl_rl/train.py \
    --task StackForce-Mos20262ClosedUsd-ClosedUsd-v0 \
    --headless --num_envs 1024 --max_gpu_mem 8 \
    --max_iterations 1500 --run_name gpu8
```

> [!NOTE]
> 📂 日志目录：`logs/rsl_rl/<experiment_name>/<时间戳>[_<run_name>]/`  
> 💾 Checkpoint 落在 `model_final.pt` 与 `model_*.pt`

---

## 🔍 可视化调试关节驱动

闭链 USD 的随机动作默认会保持若干步，避免每帧高频随机在视觉上互相抵消。

需要肉眼检查关节驱动时，可以用**正弦步态**：

```bash
python scripts/random_agent.py \
    --task StackForce-Mos20262ClosedUsd-ClosedUsd-v0 \
    --num_envs 1 --num_steps 0 \
    --motion_mode gait --action_gain 0.1
```

---

## 🛠️ 推荐环境 & 安装

### 已验证版本组合

| 组件 | 版本 |
|:---|:---|
| 🐍 Python | `3.11` |
| 🎮 Isaac Sim | `5.1.0` |
| 🧪 Isaac Lab | `v2.3.2` / pip `2.3.2.post1` |
| 🔥 PyTorch | `2.7.0+cu128` |
| 👁️ Torchvision | `0.22.0+cu128` |
| 🦿 rsl_rl | 由 LeggedGym-Ex `0.3.0` 提供 |

### 🚀 一键安装

```bash
chmod +x scripts/setup_stackforce_isaac_lab_sim_env.sh
./scripts/setup_stackforce_isaac_lab_sim_env.sh
```

脚本默认创建 `env_isaaclab` 环境。要改环境名：

```bash
ENV_NAME=my_isaaclab ./scripts/setup_stackforce_isaac_lab_sim_env.sh
```

<details>
<summary><strong>🖥️ 多 GPU 机器：窗口无法显示？</strong></summary>

<br/>

先确认显示器接在哪张 GPU 上，再用对应 GPU 启动：

```bash
nvidia-smi --query-gpu=index,pci.bus_id,name,display_active --format=csv,noheader

CUDA_VISIBLE_DEVICES=<display_active 为 Enabled 的 GPU index> \
    python scripts/rsl_rl/play.py \
        --task StackForce-Mos20262ClosedUsd-ClosedUsd-v0 \
        --checkpoint <checkpoint.pt> \
        --num_envs 1 --disable_resets
```

</details>

---

## 🎬 播放训练结果

一键脚本会自动找出 `logs/` 下最新的 `.pt` 并启动 play.py：

```bash
# 默认: 20 envs + 关闭 reset
./scripts/rsl_rl/play_latest.sh

# 想改播放参数，直接在后面追加（会原样转发给 play.py）
./scripts/rsl_rl/play_latest.sh --num_envs 1 --num_steps 0 --disable_resets

# 想换任务或日志目录，用环境变量覆盖
TASK=StackForce-Mos20262ClosedUsd-ClosedUsd-v0 LOGS_DIR=logs \
    ./scripts/rsl_rl/play_latest.sh
```

<details>
<summary><strong>🔎 手动选择 Checkpoint</strong></summary>

<br/>

```bash
# 查找最新的 checkpoint
find logs -name "*.pt" | sort | tail -n 1

# 播放指定 checkpoint
python scripts/rsl_rl/play.py \
    --task StackForce-Mos20262ClosedUsd-ClosedUsd-v0 \
    --checkpoint <上一步找到的 .pt 文件> \
    --num_envs 20 --disable_resets
```

</details>

---

## 📊 评估训练结果

> 训练 reward 高 ≠ 真实表现好。播放只是肉眼看，**定量验证**才能发现 reward hacking、步态异常、力矩不足、泛化差等问题。

`scripts/rsl_rl/eval.py` 以**多并行环境、关闭探索噪声**的方式跑策略，直接读取环境内部真实状态（速度、跌倒判定、12 个关节力矩、足端 body、地形原点）统计一整套指标，并写出 JSON；`eval_report.py` 把多个 JSON 汇总成验证表 + CSV。

```bash
# 平地基线：64 个环境跑 3000 步（=60s，约数十个回合）
python scripts/rsl_rl/eval.py \
    --headless --num_envs 64 --num_steps 3000 \
    --load_run 2026-06-07_22-11-43 --tag flat_default

# 结果默认写到 <checkpoint目录>/eval/<tag>.json
```

### ⚙️ `eval.py` 参数

| 参数 | 默认 | 说明 |
|:---|:---:|:---|
| `--load_run` | `.*` | 选择 `logs/rsl_rl/<exp>/` 下的训练 run（正则） |
| `--checkpoint` | `model_.*.pt` | checkpoint 文件名正则，或直接给 `.pt` 绝对路径 |
| `--num_envs` | `64` | 并行评估环境数，越多统计越稳 |
| `--num_steps` | `3000` | 控制步数（dt=0.02s；1000 步 = 一个完整回合） |
| `--tag` | `eval` | 结果标签，决定 JSON 文件名与报告分组 |
| `--terrain` | `flat` | `flat` / `rough` / `curriculum` |
| `--cmd_vx/--cmd_vy/--cmd_wz` | 训练值 | 覆盖速度/偏航指令（指令泛化测试） |
| `--friction` | — | 覆盖地面静/动摩擦（泛化测试） |
| `--mass_scale` | `1.0` | 按比例缩放机器人质量（负载测试，例 `1.1`=+10%） |
| `--obs_noise` | `0.0` | 观测高斯噪声标准差（传感器噪声测试） |
| `--action_delay` | `0` | 动作延迟步数（每步 20ms；sim-to-real 延迟测试） |
| `--push_interval_s` / `--push_vel` | `0` / `0.5` | 每隔 N 秒对 base 施加随机水平速度冲量（推搡测试） |

### 📋 指标说明

| 类别 | 指标 | 含义 |
|:---|:---|:---|
| 🟢 存活 | `success_rate` / `fall_rate` / `mean_survival_s` | 回合走完率 / 跌倒率 / 平均存活时间 |
| 🎯 跟踪 | `vx_rmse` / `vy_mae` / `yaw_mae` | 速度/偏航跟踪误差（指令 vs 实际） |
| 🧍 姿态 | `mean_base_height` / `mean_upright_err` | base 高度（目标 0.32）/ 倾斜误差 |
| 🔋 能耗 | `mean_power_w` / `mean_cot` | 平均机械功率 / 运输成本 CoT = E/(m·g·d) |
| ⚙️ 力矩 | per-joint `abs_mean` / `rms` / `abs_max` / `near_limit_frac` | 每关节力矩统计 + **近上限占比**（上限自动读 `effort_limit_sim`） |
| 🦿 步态 | `duty_factor` / `touchdown_hz` / `diag_inphase_rate` | 占空比 / 触地频率 / 对角同相位率（判断 trot vs bound） |
| 👣 打滑 | `mean_foot_slip` | 接触脚水平速度平方和 |

> [!NOTE]
> 本闭链 USD 没有接触传感器，步态指标用**足端 body 高度**近似接触判定。
> 指令不进 observation（固定单指令训练），所以指令泛化测试是“换指令重建环境跑一遍”。

### 🧪 一键验证套件

`eval_suite.sh` 按“仿真泛化 → sim-to-real、简单 → 复杂”的顺序跑完整验证表（指令 / 地形 / 摩擦 / 质量 / 推搡 / 噪声 / 延迟），最后自动汇总：

```bash
# 用法: eval_suite.sh [LOAD_RUN] [NUM_ENVS] [NUM_STEPS]
scripts/rsl_rl/eval_suite.sh 2026-06-07_22-11-43 64 3000

# 单独汇总已有 JSON（纯标准库，无需 Isaac，可在任意机器跑）
python scripts/rsl_rl/eval_report.py \
    logs/rsl_rl/mos2026_2_closed_usd/<RUN>/eval --csv table.csv
```

> [!TIP]
> 每个测试条件都是一次独立的 Isaac 进程（启动较慢但相互隔离、结果干净）。
> 只想跑其中几项时，把 `eval_suite.sh` 里不需要的行注释掉即可。

---

## 📦 闭链 USD 注意事项

> [!IMPORTANT]
> 闭链机器人 **不要** 再走 URDF Converter！  
> 本导出包直接使用 `mos2026_2.usd`

> [!WARNING]
> 如果你把整个工程移到新目录，请在新目录重新执行：
> ```bash
> python -m pip install -e source/stackforce_mos
> ```

---

## 🏆 自定义 Reward

### 1️⃣ 编辑 Reward 实现

```
source/stackforce_mos/
  └── stackforce_mos/
        └── tasks/direct/mos2026_2_closed_usd/
              └── 📄 custom_rewards.py
```

在 `compute_custom_reward_terms(env)` 中返回新的 reward tensor。

### 2️⃣ 调整 Reward 权重

```
source/stackforce_mos/
  └── stackforce_mos/
        └── tasks/direct/mos2026_2_closed_usd/
              └── 📄 mos2026_2_closed_usd_env_cfg.py
```

把对应的 `reward_scales` 改成非零值即可。

---

## 📌 来源

| 字段 | 详情 |
|:---|:---|
| 🤖 Robot | mos2026 2 |
| 📁 Source | User-uploaded USD package |
| 🔄 Pipeline | Imported from the unified URDF/USD upload path |
| 🗺️ Mapping | 导出可训练 Isaac Lab 工程前，需在 Mapping card 中设置 actuated joint names |

---

<div align="center">

**Built with ❤️ by [StackForce SimReady](https://stackforce.ai)**

</div>
