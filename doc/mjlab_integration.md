# 将 mjlab 集成进本项目（mos_one）

> 目标读者：本项目维护者。
> 适用栈：现有 **Isaac Sim 5.1.0 + Isaac Lab v2.3.2 + rsl_rl**（Python 3.11 / Torch 2.7 cu128，USD/闭链机器人）。
>
> ✅ **2026-06-11 已按 A 方案落地**：submodule 锁定 `0cdc5624`（v1.1.1-257），独立 uv 环境
> （Python 3.13 / torch 2.9.0+cu128 / mujoco 3.8.1 / warp 1.14.0），RTX 5090 上官方
> `Mjlab-Velocity-Flat-Unitree-Go1` 任务 3-iter 训练冒烟通过（~1.1s/iter）。
> **§4 的命令已全部换成实测验证过的真实命令**；§5–§6（接入本机器人）尚未开始。

---

## 0. 结论先行（TL;DR）

- **mjlab ≈ “把 Isaac Lab 的 manager 式 API 搬到 MuJoCo Warp 上”。** 它与本项目共享同一套设计哲学（manager-based env）和**同一套 RL 库 rsl_rl（PPO）**，因此**任务定义可大幅复用，训练/导出脚本几乎通用**。
- 集成要处理三个差异：
  1. **仿真器**：PhysX 5 → MuJoCo Warp（`mujoco_warp`）
  2. **资产格式**：USD/URDF → **MJCF**（MuJoCo XML）
  3. **依赖栈**：Isaac Sim(conda) → mjlab(uv)，**必须用独立环境**，不要混进 `env_isaaclab` 那个 conda env。
- **对本项目的价值**：
  - (a) **8GB RTX 4060 本地更友好** —— mjlab 不需要 Isaac Sim 那几十 GB 的安装，启动轻、显存占用低，适合本地快速迭代；
  - (b) **闭链原生支持** —— MuJoCo 用 equality 约束天然处理并联连杆膝关节（`thigh_motor_gear → link_a → link_b → shank` 这套环），比 PhysX 省心（项目里一路 `closed_usd` 命名都在跟这个较劲）；
  - (c) **sim-to-sim 验证** —— Isaac 训练 → mjlab 复核 → 现有 `third_party/rl_sar` 部署，降低 sim2real gap。

---

## 1. mjlab 是什么 / 与 Isaac Lab 的关系

| 维度 | Isaac Lab（现有） | mjlab |
|---|---|---|
| 仿真后端 | Isaac Sim / PhysX 5 | **MuJoCo Warp**（`mujoco` + `mujoco_warp`） |
| 资产格式 | USD（本项目还含 URDF/STL） | **MJCF**（MuJoCo XML）+ mesh |
| 并行方式 | PhysX GPU pipeline | Warp GPU kernels |
| Env 架构 | manager-based（Observation/Reward/Command/Event/Termination/Action Manager） | **同为 manager-based，API 高度相似** |
| RL 库 | **rsl_rl（RSL-RL PPO）** | **同样是 rsl_rl** |
| 安装 | pip + `isaacsim` + conda | **uv** + `mujoco` + `mujoco_warp` |
| 体量 | 数十 GB，重 | 轻量，消费级 GPU 友好 |
| 闭链 | PhysX 闭链支持有限、较麻烦 | **equality 约束原生支持** |

一句话：**迁移成本主要落在「资产转换」和「少量 API 适配」上，MDP 逻辑（reward/obs/command）和 PPO 训练基本可平移。**

---

## 2. 为什么要在本项目里集成它

1. **闭链膝关节**：MJCF 的 `<equality><connect>` / `<weld>` 能把断开的连杆端点重新闭合成四连杆环，作动器只驱动主动关节（髋/大腿电机），从动连杆交给约束求解器。这是 MuJoCo 相对 PhysX 的主场。
2. **本地训练成本低**：呼应「8GB 4060 vs 32GB 调优的 PhysX 容量上限」这一已知约束（见 `scripts/rsl_rl/train.py` 的 `--max_gpu_mem` / `--num_envs`）。mjlab 在小卡上更容易跑起来。
3. **跨仿真器鲁棒性**：把 Isaac 训出的策略放到 MuJoCo 里复核，是廉价且有效的 sim2real 前哨。

---

## 3. 集成方式选型

| 方案 | 做法 | 适用 | 评价 |
|---|---|---|---|
| **A. submodule + 独立 uv 环境** | `third_party/mjlab` 子模块 + 独立 venv | 需要读/改 mjlab 源码、锁版本 | ✅ **推荐**，与现有 `third_party/rl_sar` 模式一致 |
| B. 纯依赖安装 | `uv add mjlab` 或从 git 安装 | 只当库用、不动其源码 | 干净，但不便读源码；早期项目 PyPI 可能滞后 |
| C. 装进 `env_isaaclab` conda env | 直接在 Isaac 环境里 pip 装 | —— | ❌ **不推荐**，`torch/warp/mujoco` 与 `isaacsim` 的 pin 极可能冲突 |

> 选 **A** 的理由：你大概率需要参考 mjlab 的 manager 实现来移植任务，submodule 能让你随时读源码、按 commit 锁版本；独立 uv 环境则彻底隔离掉与 Isaac Sim 的依赖冲突。

---

## 4. 集成步骤（A 方案，✅ 已完成并实测）

### 4.1 添加 submodule（已做）

```bash
git submodule add https://github.com/mujocolab/mjlab.git third_party/mjlab
# 当前锁定：0cdc56246999409b83622764f5b38edb660cf16e（v1.1.1-257-g0cdc5624）
```

### 4.2 独立 uv 环境安装（已做，命令已验证）

```bash
cd third_party/mjlab
uv sync --extra cu128        # 仓库带 uv.lock；cu128 = CUDA 12.8 版 torch（5090 必需）
```

实测装出的关键版本：**Python 3.13、torch 2.9.0+cu128、mujoco 3.8.1、mujoco-warp、warp-lang 1.14.0**。
环境位于 `third_party/mjlab/.venv`，与 `env_isaaclab` 完全隔离。

```bash
# 验证 GPU 链路
uv run python -c "import torch; print(torch.cuda.is_available())"   # True
uv run list-envs                                                     # 列出全部 Mjlab-* 任务
```

### 4.3 官方任务冒烟（已通过）

入口是 pyproject 注册的 `train` / `play` / `demo` / `list-envs` 脚本（tyro CLI，嵌套参数）：

```bash
cd third_party/mjlab

# ✅ 实测：Go1 四足速度任务，64 env × 3 iter，~1.1s/iter（RTX 5090）
uv run train Mjlab-Velocity-Flat-Unitree-Go1 \
    --env.scene.num-envs 64 --agent.max-iterations 3 --agent.logger tensorboard

# 正式训练（默认 logger 是 wandb，需要先 wandb login；不想用就 --agent.logger tensorboard）
uv run train Mjlab-Velocity-Flat-Unitree-Go1 --env.scene.num-envs 4096

# 播放 / 用 zero agent 检查 MDP
uv run play Mjlab-Velocity-Flat-Unitree-Go1 --agent zero
```

> 注意：`train.py` 用 `CUDA_VISIBLE_DEVICES` 判断设备（空 = CPU），默认 `gpu_ids=[0]` 会自动设置。
> 训练日志写到**当前目录**的 `logs/rsl_rl/`——在 submodule 里跑完记得清掉，保持子模块干净。

---

## 5. 把机器人接进 mjlab（资产层）

### 5.1 URDF/USD → MJCF

- 复用现有 `assets/urdf/mos2026_v2/meshes/*.STL`，**不复制 mesh**，MJCF 用相对路径引用。
- 转换途径（任选）：
  - MuJoCo 官方 URDF 导入（`<compiler>` 直接吃 URDF / `mjcf` 解析）；
  - 手写 MJCF 骨架 + `<mesh>` 引用 STL（最可控，推荐用于闭链）；
  - 参考 **MuJoCo Menagerie** 的组织范式。
- 注意：**单位、惯量（inertial）、坐标系/axis 方向**逐项核对，URDF 与 MJCF 约定不完全一致。

### 5.2 闭链闭合（关键步骤）

URDF 是树结构，表达不了并联膝的环。转出 MJCF 后，按下述方式补回闭环：

```xml
<!-- 在 <equality> 段落里把断开的连杆端点重新连上，形成四连杆闭环 -->
<equality>
  <connect name="fl_knee_loop" body1="fl_shank_link_b" body2="fl_shank" anchor="..."/>
  <!-- 其余三条腿同理 -->
</equality>
```

- 作动器只挂在**主动关节**（髋外摆、大腿电机），从动连杆 `link_a/link_b` 由约束求解。
- 闭链稳定性靠 `solref/solimp`、惯量、`timestep` 调，先用简化模型把流程跑通再精修。

### 5.3 对齐关节符号 / 极性（容易踩坑）

把已知经验带过来，**别照抄 URDF 注释**：

- **前髋不是镜像关系**：两侧同时外摆时符号相反（fl < 0、fr > 0）；`init_state` 里“外撇”的注释是错的。
- MJCF 的 joint `axis` 方向、actuator 极性，按 `scripts/tools/measure_joint_signs.py`、`scripts/tools/abduct_hips.py` 的**实测结果**来设。

---

## 6. 把任务接进 mjlab（代码层）

因为两边都是 manager-based，迁移是「**按字段翻译**」而非重写：

| 本项目（Isaac Lab） | mjlab 对应（名称以源码为准） |
|---|---|
| `ObservationsCfg` | Observation manager cfg |
| `RewardsCfg` | Reward manager cfg |
| `CommandsCfg`（速度指令） | Command manager cfg |
| `EventCfg`（域随机化） | Event manager cfg |
| `TerminationsCfg` | Termination manager cfg |
| `ActionsCfg` | Action manager cfg |
| `rsl_rl` PPO agent cfg（`scripts/rsl_rl`） | **基本可直接搬**（注意 obs/action 维度一致） |

需逐项核对的差异点（以 mjlab 源码为准）：

- **接触/传感器读取**：PhysX contact → MuJoCo `contact`/`sensor`；
- **地形**：Isaac `TerrainImporter` → mjlab 地形方案（对应 train.py 的 `flat/rough/curriculum`）；
- **域随机化字段名**：Event 项参数命名差异。

**建议推进顺序**：平地 + 你的 MJCF 站立 → 速度跟踪 reward 跑通 → 逐步加域随机化 → 加地形。

---

## 7. 目录组织建议

```
mos_one/
├─ third_party/
│  ├─ rl_sar/              # 已有：部署 / sim2real
│  └─ mjlab/               # 新增 submodule
├─ source/.../tasks/       # 现有 Isaac Lab 任务（保持不动）
├─ mjlab_tasks/            # 新增：你的 mjlab 任务包（与 isaac 任务并列）
│  ├─ assets/mos2026.xml   # MJCF（相对路径引用现有 STL）
│  └─ velocity_env_cfg.py
└─ scripts/
   ├─ rsl_rl/              # 现有 Isaac 训练/play
   └─ mjlab/               # 新增 mjlab 训练/play 入口
```

- **资产复用**：MJCF 里 `<mesh file="../assets/urdf/mos2026_v2/meshes/fl_shank.STL"/>` 这样相对引用，避免重复拷贝。

---

## 8. 环境与依赖隔离（重要）

- `env_isaaclab`（conda，Isaac Sim）与 mjlab（uv venv）**两套环境分开**，互不污染。
- 4060 上 Warp 也吃显存：`num_envs` 先从 **1024 ~ 4096** 起步，与 Isaac 那边 `--max_gpu_mem` / `--num_envs` 的收缩思路一致。

---

## 9. 验证清单（里程碑）

- [x] mjlab 官方 demo 能训练（2026-06-11：Go1 velocity 3-iter 冒烟通过，RTX 5090）
- [x] 自己的机器人接进 mjlab（2026-06-12：`mjlab_tasks/mos_one_mjlab/`，复用
  `deploy/mujoco/assets/mos2026_2.xml`，spec_fn 程序化适配——删 floor/light/旧执行器、
  加 IMU 传感器组 + 足端 site；任务 `Mjlab-Velocity-Flat-MosOne`）
- [x] 平地速度跟踪任务冒烟跑通（2026-06-12：16 env × 2 iter，全 reward 项/终止/课程正常；
  收敛训练待跑）
- [ ] 关节 axis / actuator 极性与实测一致（measure_joint_signs）
- [ ] 正式训练收敛 + 步态质量评估
- [ ] sim2sim：Isaac 训出的 policy 在 mjlab 回放行为合理（注意两边 obs 布局不同，
  需要专门的回放适配层）
- [ ] 导出 policy → `third_party/rl_sar` 部署

---

## 10. 风险与注意

- 本文部分命令/模块名基于通用认知，**务必以 mjlab 仓库 README 与 pyproject 为准**（撰写时外网受限，未能拉取最新）。
- mjlab 较新，API 可能变动 → submodule **锁定 commit**。
- 闭链 MJCF 调参（约束、惯量）需要时间 → **先用简化模型跑通整条链路**，再回头精修。

---

## 参考

- mjlab：<https://github.com/mujocolab/mjlab>
- MuJoCo 官方文档（含 equality 约束、MJCF 语法）：<https://mujoco.readthedocs.io>
- MuJoCo Menagerie（资产组织范例）：<https://github.com/google-deepmind/mujoco_menagerie>
- rsl_rl（RSL-RL）：<https://github.com/leggedrobotics/rsl_rl>
