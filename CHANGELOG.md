# 更新日志 / CHANGELOG

> **工程里程碑时间线** —— 回答"代码现在能做什么了"。
> 配套：[`todo.md`](todo.md)（前瞻计划）· [`doc/experiments/EXPERIMENTS.md`](doc/experiments/EXPERIMENTS.md)（实验迭代台账）。
> 三者关系见 [`doc/version_tracking.md`](doc/version_tracking.md)。
>
> 版本号采用「能力里程碑」语义：`vX.Y-<能力关键词>`，在达成一个可演示的整机/管线能力时打 tag。
> 路线：**建模 → 仿真 → 控制 → RL 平地 → 地形泛化 → 动态/HIM → Sim2Real → 真机**

---

## [未发布] Unreleased

> 当前正在进行、尚未打 tag 的工作。达成一个里程碑能力后，把本节内容迁入新版本号。

### 新增
- 站起来过程关节/电机力矩分析与可视化 `deploy/mujoco/standup_torque.py`（`43980e4`）
- mjlab 集成说明文档 `doc/mjlab_integration.md`（`b21560e`）
- **版本记录系统**：CHANGELOG + 实验台账 + 自动追加脚本 `tools/exp/log_run.py`

### 变更（2026-06-12，仿真侧 sim2real 修复，16-env 冒烟通过，待重训验证）
- 训练 env 力矩惩罚默认开启：`reward_scales["torque"]` 0.0 → **-2e-4**（贴上限硬走 / 蹦跳步态的根因）
- `effort_limit_sim` 12 → **16 N·m**（12 卡深蹲/蹬地需求线零余量，见 `doc/dynamics_gear_ratio_analysis.md` 行动项）
- 执行器新增 **`armature=0.01`**（转子惯量×6.33² 反射；取自 menagerie unitree_go2 同款 GO-M8010-6 执行器，待真机辨识校准）
- `foot_contact_height_threshold` 0.07 → **0.15**（此前 `foot_slip` 奖励从未生效）
- `train.py --num_envs` 默认 16000 → **16384**（RTX 5090 32GB 实测 16384 可跑，~85k steps/s）

### 修复
- 重新登记 `rl_sar` 子模块（恢复 `.gitmodules` 与 gitlink）（`0c94d36`）

---

## [v0.5-control-stack] · 2026-06-09 —— 运控栈与 Sim2Real 基建

腿部运动学/步态/域随机化/导出一次性补齐，形成 sim→real 的完整链路骨架。

### 新增
- 腿部 FK/IK + trot 步态 + 域随机化 + 动力学选型 + ONNX 导出（`7ebd2d8`）
- 控制栈文档：FK/IK · 步态 · 动力学 · sim2real（`doc/control_stack.md`，`bf32141`）
- 动力学/减速比分析（`doc/dynamics_gear_ratio_analysis.md`）：估算各关节理论静态保持力矩

### 本轮实验结论（详见台账 EXP-0001 / EXP-0002）
- ONNX 导出 + 数值校验通过；域随机化冒烟通过；旧 checkpoint 格式转换器落地
- **速度扫描定论**：当前策略指令未进 obs，对 `--cmd_vx` 无响应 → 催生「指令条件化重训」计划

---

## [v0.4-deploy] · 2026-06-08 —— RL 推理部署 + 实验跟踪

### 新增
- 50 Hz RL 推理主控节点 `deploy/real/rl_deploy.py`（`01c5fdc`）
- `eval` 评估管线（`60e77ba`）
- SwanLab 实验跟踪接入，project=`mos_one-mos`（`f94ca6c`）
- sar_rl 适配进度 README（`86b750d`）

### 变更
- `mos2026_2.yaml` 注释中文化；忽略 CMake 构建产物（`bc2cf1b`）

---

## [v0.3-standup] · 2026-06-04 —— ✅ 真机站起来

四足真机已能从趴姿安全启动、撑起到站姿。这是第一个可演示的整机能力里程碑。

### 新增
- 前馈力矩 / 扭矩监控 / 角度跟踪 + 站立健壮性修复（`229a33a`）
- 站立保持时单关节微调（≤10°/1s 间隔），禁用坐下功能（`35e3ab2`）

### 已知瓶颈
- **力矩 / 电流不足**：站立维持/动态动作时电机输出达不到所需值（详见 `todo.md` 力矩专项）

---

## [v0.2-motor-gui] · 2026-05-30 —— 电机调试与站立标定工具

### 新增
- 网页端电机调试工具（`d612af4` / `ba3b709`）
- 四足站立标定与方向验证网页工具（`0eb68f3`）
- 完善电机调试网页与底层伺服：统一关节角显示、实时反馈、安全开关（`309ac89`）

---

## [v0.1-sim-setup] · 2026-05-15 —— Isaac Lab 接入与闭链 USD 环境

项目起点：把闭链 USD 资产接入 Isaac Lab，环境可加载、可跑 zero/random agent。

### 新增
- 接入 Isaac Lab，PPO 训练管线打通（`e409b90`）
- 闭链 USD 资产增强与环境配置 `Mos2026_2`（`bbfe587`）
- 环境配置完善；删除 USD 内嵌 ground（`ef093a8` / `3c964f5`）
- 参考论文归档（`497b986`）

---

<!--
新增一个里程碑版本时的模板：

## [vX.Y-能力关键词] · YYYY-MM-DD —— 一句话能力描述

### 新增
- ...（commit 短哈希）

### 变更 / 修复 / 已知瓶颈
- ...

打 tag：见 doc/version_tracking.md「补打历史里程碑 tag」
-->
