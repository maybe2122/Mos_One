# 🐾 Mos 开发流程

> 总体路线：**建模 → 仿真 → 控制 → RL 平地 → 地形泛化 → 动态/Hybrid Internal Model → Sim2Real → 真机**
>

---

## 📍 当前状态（更新于 2026-06-03）

✅ **四足真机已能站起来**（从趴姿安全启动 → 撑起到站姿，见 [站立标定/方向验证网页工具]）。

⚠️ **主要瓶颈：力矩 / 电流不足**
- 站立维持或动态动作时，电机输出力矩（电流）达不到所需值，表现为：撑起吃力 / 站姿下垂 / 抗扰差。
- 整机质量 **≈ 11.70 kg**（USD 标称：base 4.52 kg + 单腿 ≈ 1.80 kg ×4），站立时单腿需承担约 1/4 整机重量的静态保持力矩。
- 需要确认是「电机/减速比选型不够」还是「驱动器电流上限设置过低 / 供电不足」。

🔧 已有调试工具：电机调试网页（前馈力矩、扭矩监控、角度跟踪）、`scripts/rsl_rl/play.py`（力矩统计 CSV / 曲线 / TensorBoard 实时曲线）。

---

## 🎯 后续工作（力矩/电流不足专项）

> 目标：定位力矩/电流缺口的根因，先让站立稳定且有余量，再谈行走。

- [ ] **量化需求 vs 实际**
  - [ ] 用 `play.py` 力矩统计 + 电机网页扭矩监控，记录站立/支撑相各关节**实测力矩与电流**
  - [ ] 用整机质量(11.70 kg)和几何，估算各关节**理论静态保持力矩**，对比实测缺口
  - [ ] 确认仿真里 `play_torque_stats.csv` 的 |max| / rms 是否超过电机额定/峰值力矩
- [ ] **驱动链路排查（先软后硬）**
  - [ ] 检查驱动器**电流上限 / 力矩上限**参数是否被设低，逐步上调到电机额定
  - [ ] 检查**供电电压/电流能力**（电池/电源是否在大负载下掉压限流）
  - [ ] 核对**减速比 / 力矩常数 Kt** 标定值，确认下发力矩→实际力矩换算正确
- [ ] **控制侧补偿**
  - [ ] 站立加**重力前馈**（gravity compensation），减少纯 PD 的力矩负担
  - [ ] 复核站立目标姿态，避免大力臂的高耗能站姿
- [ ] **选型评估（若软调无法满足）**
  - [ ] 评估更高扭矩电机 / 更大减速比 / 更高母线电压的方案
  - [ ] 在仿真里对齐真实电机的 actuator 力矩-速度曲线，确认 policy 输出在可行域内
- [ ] **闭环验证**：上调后重测站立维持 + 抗扰（侧推），确认有力矩余量再推进行走

---

## 🧱 Phase 1：机械建模与仿真环境

📄 [sub/Structural_Design.md](sub/Structural_Design.md) ｜ [sub/urdf.md](sub/urdf.md)

- [x] 结构设计：Body / Legs / Joints / Sensors / Battery
- [ ] 重心、刚性、承载能力评估
- [x] SolidWorks → URDF / USD 导出
- [x] 关节类型 / link 层级 / 惯量参数 检查
- [x] 导入 Isaac Lab / MuJoCo，加载无报错
- [x] 重力、关节限制、无穿模/抖动 验证

---

## 🤖 Phase 2：基础控制（非 RL，先能动）

> 关键：先用传统控制让它「站起来 / 走一步」，作为 RL 的 baseline。

- [ ] 关节 PD 控制器：`τ = kp·(q*−q) + kd·(dq*−dq)`，每关节稳定无震荡
- [ ] 手写简单 gait（抬腿 → 前摆 → 落地），能走几步
- [ ] 状态观测打通：joint pos/vel、base orientation/vel

---

## 🧠 Phase 3：强化学习（平地步态）

📄 [sub/Isaac_Lab_env_create.md](sub/Isaac_Lab_env_create.md)

- [ ] 定义 RL 环境
  - obs: `[joint_pos, joint_vel, base_vel]`
  - action: `joint_target / torque / residual`
- [ ] 奖励函数：前进速度、姿态稳定（roll/pitch）、能耗 penalty
- [ ] PPO + GPU 并行训练
- [ ] 在平地收敛，学会前进 + 不摔倒

---

## 🏔️ Phase 4：地形泛化（Curriculum Learning）

- [ ] 难度递增：平地 → 不平地 → 楼梯 / 小障碍   curriculum 
- [ ] 地形观测：height map、foot contact
- [ ] 稳定性奖励增强：slip penalty、contact timing
- [ ] 同一 policy 跨地形保持稳定
> 参考论文：*Hybrid Internal Model: Learning Agile Legged Locomotion with Simulated Robot Response*
---

## ⚡ Phase 5：高级动态训练（Hybrid Internal Model 风格）

> 引入 simulated robot response，作为 internal model 反馈到 policy。

- [ ] 高阶步态：跳跃、快速转向、急停
- [ ] 奖励扩展：动态稳定性、敏捷性、能耗最优
- [ ] Sim2Real 预备：domain randomization（mass / friction / sensor noise）
- [ ] 加入控制延迟、actuator 动力学偏差

---

## 🔁 Phase 6：Sim2Real 与真实硬件

- [ ] 硬件实物：电机 / 编码器 / IMU / 控制器
- [ ] 实时控制：MCU 或工控机，1 kHz 控制循环
- [ ] 软件部署：policy 推理 + ROS 2 接口
- [ ] 上机测试：低速 → 加速 → 多地形，配急停保护

---

## 🔧 Phase 7：调试与优化

- [ ] 监控 reward 收敛 / 前进能力 / 摔倒次数
- [ ] 高阶任务：楼梯、跳跃、急转
- [ ] 反推调优：PD 基线、奖励项、动作空间
- [ ] 仿真 inertia 与实机标定对齐

---

## 🚀 最小可行版本（MVP）

```text
1. 导入仿真（Phase 1）
2. PD 让它能动（Phase 2）
3. RL 在平地学会向前走（Phase 3）
```

完成这三步就有一个能跑的雏形，其它阶段都是在这之上叠加。

---

## ⚠️ 常见坑

- ❌ 一上来做 Sim2Real → 必崩
- ❌ 没有 PD baseline → RL 学不会
- ❌ reward 太复杂 → 不收敛
- ❌ 模型 inertia 错 → 一切错
