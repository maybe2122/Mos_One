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
  - [📈 SwanLab 实验跟踪](#-swanlab-实验跟踪)
  - [🔍 可视化调试关节驱动](#-可视化调试关节驱动)
  - [🛠️ 推荐环境 \& 安装](#️-推荐环境--安装)
    - [已验证版本组合](#已验证版本组合)
    - [🚀 一键安装](#-一键安装)
  - [🎬 播放训练结果](#-播放训练结果)
  - [📊 评估训练结果](#-评估训练结果)
    - [⚙️ `eval.py` 参数](#️-evalpy-参数)
    - [📋 指标说明](#-指标说明)
    - [🧪 一键验证套件](#-一键验证套件)
  - [🦿 控制栈：运动学 / 步态 / 动力学 / Sim2Real](#-控制栈运动学--步态--动力学--sim2real)
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

## 📈 SwanLab 实验跟踪

训练默认开启 [SwanLab](https://swanlab.cn)，把 rsl_rl 写进 TensorBoard 的所有标量（reward、loss、各项奖励分量等）**自动镜像**一份到 SwanLab，方便跨实验对比。`scripts/rsl_rl/train.py`（PPO）和 `him/train.py`（HIM）都支持，参数一致。

> [!IMPORTANT]
> 镜像通过 patch TensorBoard 的 `SummaryWriter`，所以 SwanLab 在建 runner 之前启动即可，**无需改动任何训练/奖励代码**。未安装 swanlab 时自动跳过，初始化失败也只回退到纯 TensorBoard，不影响训练。

### ⚙️ 相关参数

| 参数 | 默认值 | 说明 |
|:---|:---:|:---|
| `--no_swanlab` | 关闭开关 | 加上即彻底关闭 SwanLab（默认开启） |
| `--swanlab_project` | `stackforce-mos` | 项目名。PPO 与 HIM 用同一项目便于对比 |
| `--swanlab_mode` | `cloud` | 运行模式：`cloud` 上传云端 / `local` 本地看板 / `offline` 离线缓存 / `disabled` 关闭 |

### 🚀 首次使用

```bash
# 1️⃣ 安装（注意 swanlab 会顶坏 Isaac 的 wrapt/sentry-sdk 版本，装完务必降回）
pip install swanlab
pip install "wrapt==1.16.0" "sentry-sdk==1.43.0"

# 2️⃣ cloud 模式首次需登录一次（在 https://swanlab.cn 账户设置里拿 API Key）
swanlab login

# 3️⃣ 正常训练即可，标量会自动上传到云端 project=stackforce-mos
python scripts/rsl_rl/train.py \
    --task StackForce-Mos20262ClosedUsd-ClosedUsd-v0 \
    --headless --num_envs 4096 --run_name baseline_flat
```

### 💡 常见用法

```bash
# 不想联网 / 服务器无外网：本地看板模式
python scripts/rsl_rl/train.py ... --swanlab_mode local

# 完全关闭 SwanLab（只留 TensorBoard）
python scripts/rsl_rl/train.py ... --no_swanlab

# 自定义项目名
python scripts/rsl_rl/train.py ... --swanlab_project my-exp
```

> [!TIP]
> 已经跑完、只有 TensorBoard 日志、想离线转成 SwanLab 本地看板看？用 `tools/swanlab_convert_local.py`（绕开了 SwanLab 0.8.0 本地转换的登录 bug）：
> ```bash
> # 用装了 swanlab 的 Isaac venv 跑转换
> /home/maybe/code/rl/env_isaaclab/bin/python3 tools/swanlab_convert_local.py \
>     logs/rsl_rl/<experiment>/<时间戳> \
>     --out ./swanlog_local --project stackforce-mos
> # 再本地起看板
> /home/maybe/code/rl/env_isaaclab/bin/swanlab watch ./swanlog_local
> ```

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

## 🦿 控制栈：运动学 / 步态 / 动力学 / Sim2Real

> 纯 RL 端到端之外补齐的**经典运控层**与 **sim2real 必需项**——RL 上真机、传统控制
> baseline、执行器选型的基础。完整说明见 **[📄 doc/control_stack.md](./doc/control_stack.md)**，
> 动力学/选型见 **[📄 doc/dynamics_gear_ratio_analysis.md](./doc/dynamics_gear_ratio_analysis.md)**。

| 模块 | 文件 | 一键验证 |
|:---|:---|:---|
| 🦵 腿部 FK/IK（解析，4 腿向量化） | `deploy/common/kinematics.py` | `--selftest`（往返 1e-16） |
| 🐾 足端轨迹 trot 步态（摆线摆动 + IK） | `deploy/common/gait.py` | `--selftest` / `--demo` / `--linkage` |
| ⚙️ 动力学 + 减速比选型 | `deploy/common/dynamics.py` | `--plot` |
| 🎲 域随机化 / 观测噪声 | `EventCfg` + `train.py --domain_rand` | env_isaaclab 冒烟 √ |
| 🔋 力矩惩罚奖励（opt-in） | `custom_rewards.py` `sum(τ²)` | `reward_scales["torque"]` |
| 📦 Policy 导出 TorchScript+ONNX | `deploy/real/policy_export.py` | 数值一致性门 `<1e-4` |

```bash
# deploy/common/* 纯 numpy，无需 Isaac（用根目录 .venv）
.venv/bin/python deploy/common/kinematics.py --selftest
.venv/bin/python deploy/common/gait.py --demo          # → outputs/gait_demo/
.venv/bin/python deploy/common/dynamics.py --plot      # → outputs/dynamics/
```

> 💡 **减速比结论**：现用 6.33 已接近最优（余量平衡最优 N\*=6.16）；真机「力矩/电流不足」
> 根因不是减速比，而是 `effort_limit_sim` 卡需求线 + 驱动器电流上限/母线掉压。

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
