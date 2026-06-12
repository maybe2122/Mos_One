# 🦿 mjlab_tasks — mos2026_2 的 mjlab 任务包

> 把本仓库的闭链四足接进 [mjlab](https://github.com/mujocolab/mjlab)（MuJoCo Warp 上的
> Isaac Lab 式训练框架）。背景与设计见 [`doc/mjlab_integration.md`](../doc/mjlab_integration.md)。

## 已注册任务

| Task ID | 说明 |
|:---|:---|
| `Mjlab-Velocity-Flat-MosOne` | 平地速度跟踪（指令条件化：vx/vy/wz 均匀采样 + 课程扩速），含域随机化（摩擦/质心/编码器偏置/推搡） |

## 文件

| 文件 | 内容 |
|:---|:---|
| `mos_one_mjlab/robot.py` | 机器人 EntityCfg：**复用** `deploy/mujoco/assets/mos2026_2.xml`（闭链 `<connect>` 已配好），spec_fn 程序化删 floor/light/旧执行器、加 IMU 传感器组 + 4 个足端 site（mesh 远端顶点算出）；执行器 GO-M8010-6（kp25/kd0.5/effort12/armature=J_rotor·N²，与 Isaac 对齐） |
| `mos_one_mjlab/env_cfg.py` | 平地速度任务 cfg（改自 mjlab Go1 flat）：pose reward **只约束 12 受控关节**（被动闭链关节不罚）、接触传感器用 shank mesh、动作 scale 与 Isaac 一致（hip 0.5 / 其他 0.8145） |
| `mos_one_mjlab/rl_cfg.py` | PPO 配置（同 mjlab Go1 日程） |

## 运行（在 mjlab 的 uv 环境里）

```bash
cd third_party/mjlab

# 训练（默认 logger 是 wandb；离线用 tensorboard）
uv run python ../../scripts/mjlab/train.py Mjlab-Velocity-Flat-MosOne \
    --env.scene.num-envs 4096 --agent.logger tensorboard

# 冒烟（已验证 ✓）
uv run python ../../scripts/mjlab/train.py Mjlab-Velocity-Flat-MosOne \
    --env.scene.num-envs 16 --agent.max-iterations 2 --agent.logger tensorboard

# zero-agent 查看 MDP / 播放 checkpoint（开 viewer，需显示）
uv run python ../../scripts/mjlab/play.py Mjlab-Velocity-Flat-MosOne --agent zero
```

> ⚠️ 训练日志写到**当前目录**（`third_party/mjlab/logs/`），跑完记得清理，保持子模块干净。

## 踩坑实录（闭链 × mjlab，都已修复在代码里）

| 症状 | 根因 | 修复位置 |
|:---|:---|:---|
| 4096 env 训练几步即 obs NaN | 闭链 `<connect>` solref=0.002 要求 dt≤1ms，mjlab 默认 dt=5ms 违反软约束稳定条件 | `robot.py` 钳制 connect timeconst≥0.004；`env_cfg.py` dt=0.002 + decimation=10 + solver 迭代 20 |
| 出生即爆炸（ep_len≈1.6 步、reward −1000+） | mjlab `CollisionCfg` 默认 contype=1/conaffinity=1 → 打开了几何重叠的闭链连杆**自碰撞** | `robot.py` 还原 contype=2/conaffinity=1（只碰地、不自碰） |
| 训练 ~25 iter 后偶发 NaN（约每数百万 env-steps 一次） | 闭链稀有失稳；爆炸第一步先污染接触力传感器（qpos 仍有限，`nan_detection` 当步抓不到），且 mjlab 在 reset **之前**算 reward；rsl_rl 对任何 NaN 硬中止 | `env_cfg.py` 加 `nan_state` 终止项（隔离+重置坏 env）+ `__init__.py` 在 RL 包装层对 **obs 和 reward** 统一 `nan_to_num`——与 Isaac env 的 `nan_to_num + invalid_state` 同语义 |

## 与 Isaac 训练栈的关系

- **几何/资产同源**：同一份 MJCF（`tools/asset/usd_to_mjcf.py` 产物），闭链 anchor 一致
- **执行器/动作对齐**：kp/kd/effort/action_scale 与 Isaac env 相同 → 便于 sim2sim 对比
- **MDP 不同**：mjlab 任务是指令条件化的（Isaac 现版指令不进 obs），obs 布局也不同
  （mjlab 含全部 28 关节 + 接触状态），两边的 checkpoint **不可互换**
