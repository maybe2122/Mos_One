# 🧰 tools/ — 工具脚本索引

> 与 `scripts/` 的分工：**`scripts/` 是 Isaac 运行时入口**（训练/播放/评估/调试），
> **`tools/` 是辅助工具**（资产转换、checkpoint 转换、实验管理、检查/可视化），按用途
> 分子目录。多数不需要 Isaac，需要的见各表"运行环境"列。

## 📂 分类

### `asset/` — USD / MJCF 资产处理

| 脚本 | 用途 | 运行环境 | 状态 |
|:---|:---|:---|:---|
| `usd_to_mjcf.py` | 闭链 USD → MuJoCo MJCF 转换（`deploy/mujoco/assets/` 的来源） | **env_isaaclab**（需 SimulationApp/pxr） | 常用 |
| `strip_embedded_ground.py` | 剥除 USD 内嵌的 GroundPlane/PhysicsScene（`replicate_physics` 克隆的前提） | env_isaaclab | ⚠️ 一次性，已执行过，留档备查 |
| `fix_fr_close_loop_path.py` | 修复 fr_close_loop 关节在 USD 里的错误路径 | env_isaaclab | ⚠️ 一次性，已执行过，留档备查 |

### `ckpt/` — Checkpoint / 模型转换

| 脚本 | 用途 | 运行环境 | 状态 |
|:---|:---|:---|:---|
| `convert_checkpoint_to_rsl_rl.py` | 旧版 `actor/critic_state_dict` 格式 → 新 rsl_rl `model_state_dict`（纯键名重映射，已验证 eval 逐位一致） | env_isaaclab（需 torch） | 常用 |

### `exp/` — 实验管理

| 脚本 | 用途 | 运行环境 | 状态 |
|:---|:---|:---|:---|
| `log_run.py` | 把一次训练登记进实验台账 `doc/experiments/EXPERIMENTS.md`（自动拉 git/run/seed） | 任意 python3（纯标准库） | 常用 |
| `swanlab_convert_local.py` | TensorBoard 日志 → SwanLab 本地看板（绕开 0.8.0 登录 bug） | env_isaaclab（需 swanlab） | 偶用 |

### `isaac/` — Isaac 运行时检查 / 可视化（需启动 Isaac Sim）

| 脚本 | 用途 | 运行环境 | 状态 |
|:---|:---|:---|:---|
| `inspect_joint_limits.py` | 打印 USD 里全部 PhysX 关节的限位/驱动参数 | env_isaaclab（headless） | 偶用 |
| `speed_viz_isaac.py` | 机身速度↔电机角速度实时可视化（HUD 条/滚动曲线/峰值表，`--speed` 设速度） | env_isaaclab（GUI） | 常用 |

## 💡 用法示例

```bash
# 实验台账（任意 python3）
python3 tools/exp/log_run.py --run-dir logs/rsl_rl/<exp>/<ts>_<run_name> \
    --title "..." --hypothesis "..." --changes "..." --conclusion "..." --dry-run

# 旧 checkpoint 转新格式
../env_isaaclab/bin/python tools/ckpt/convert_checkpoint_to_rsl_rl.py \
    <old_model.pt> -o <out_rslrl.pt>

# USD → MJCF（需 Isaac）
../env_isaaclab/bin/python tools/asset/usd_to_mjcf.py

# TensorBoard → SwanLab 本地看板
../env_isaaclab/bin/python tools/exp/swanlab_convert_local.py \
    logs/rsl_rl/<experiment>/<时间戳> --out ./swanlog_local --project mos_one-mos

# 速度↔电机角速度实时可视化（需 Isaac GUI；--ramp 扫速度）
../env_isaaclab/bin/python tools/isaac/speed_viz_isaac.py --speed 1.5
```

> 📌 新增工具请按上述分类放进对应子目录，并在本表登记（用途/运行环境/是否一次性）。
