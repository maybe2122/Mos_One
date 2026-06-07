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

### 📊 评估发现（2026-06-07，`eval.py` / `model_750.pt` 平地 1.0 m/s）

> 工具：`scripts/rsl_rl/{eval.py, eval_report.py, eval_plot.py}`，结果见 `logs/.../eval/`。
> 结论：策略**站立稳、速度跟踪近乎完美（0% 跌倒、vx 0.99/1.0）**，但**靠蹲低 + 小腿电机近乎满载**换来——这正是真机"力矩/电流不足"在仿真侧的同源表现。

- [ ] **加力矩惩罚项**（根因）：当前 `reward_scales` 无任何 torque penalty，PPO 没有省力矩动机 → 学出贴上限硬走。
  - 仿真实测：`fl/rl/rr_shank` 有 **83~84%** 时间 ≥90% 上限（12 N·m），`rl_thigh` 66%，整体近上限占比 40%，CoT 高达 **4.5**、功率 515 W。
  - 在 `custom_rewards.py` 加 `sum(τ²)` 惩罚并在 `reward_scales` 给小负权重，重训后用 `eval_plot.py` 对比近上限占比。
- [ ] **抬高站姿**：base 高度只有 **0.256**（目标 0.32，低 20%），蹲低直接抬高膝/踝力矩需求 → 调大 `base_height` 权重或核对目标可达性。
- [ ] **修接触阈值**：`foot_contact_height_threshold=0.07` 太低，shank body 中心始终高于阈值 → **`foot_slip` 奖励训练时大概率从未生效**，步态/打滑指标也全退化。抬到 ~0.15（评估时可用 `eval.py --foot_contact_height 0.15` 验证）。
- [ ] **速度-力矩扫描**：用 `eval.py --cmd_vx {0.3,0.6,1.0,1.3}` + `eval_plot.py` 看 `_compare.png`，确认力矩饱和随指令速度如何恶化，定出真机可行的速度上限。

---

## 🧩 代码审计缺口（更新于 2026-06-08）

> 基于对 `source/`、`him/`、`deploy/`、`motor_control/` 的实际代码核查（非路线图层面）。
> 区别于上面的力矩专项：这里是**结构性缺口**——不补，策略「能在仿真里跑」但「上不了真机」。
> 优先级总原则：先结硬件根因（力矩专项）→ 训练侧补 sim2real 必需项 → 定线（stock vs HIM）→ 导出+校验 → 真机控制器。

### A. 仿真训练侧（最致命：训练设定本身不为真机服务）

- [ ] **域随机化 / 扰动注入 = 0（最高优先级）**
  - 全仓 `grep` 不到任何 `EventTermCfg / randomize_rigid_body / push / apply_external_force`，连骨架都没有。
  - 质量、质心、摩擦、关节 friction/damping、Kp/Kd 全是单一标称值；观测零噪声（`_get_observations` 直接 `cat` 真值）；无 push 外推、无 actuator 延迟/带宽。
  - → 这种策略上真机基本必崩。需新增一套 `EventCfg`：随机 base/腿质量±质心、地面/足端摩擦、Kp/Kd、关节零位偏置、观测高斯噪声、周期性 push、控制延迟。
- [ ] **stock PPO 观测含特权信息 `root_lin_vel_b`（结构性不可部署）**
  - `observation_space=45` 前 3 维是机身线速度真值，真机**没有传感器直接给**（需 IMU+里程做状态估计）。
  - `state_space=0` → critic 也对称，无法用特权信息训练估计器。
  - → stock PPO 这条线结构上无法部署，必须二选一（见 §B 定线）。
- [ ] **无观测历史 / 时序建模**（stock）：单帧 MLP 对真机噪声很脆，缺历史去滤波/估速。HIM 有，stock 没有。
- [ ] **地形泛化基本未启用**：`terrain_curriculum_enabled=False`，默认 `plane`；rough/stairs 地形 cfg 写了但未纳入课程。对应 Phase 4 实质未做。

### B. 路线定线：stock PPO vs HIM（需先决策）

- [ ] **决策：放弃 stock PPO 转 HIM，还是给 stock 加非对称 critic + 速度估计器？**
  - `him/`（盲 actor + 6 步历史 270 维 + HIM 速度估计器 + 特权 critic 51 维）是 sim2real 正解，但**未见收敛模型/验证产物**。
  - [ ] 把 HIM 跑出第一个收敛模型；在「受限观测」MuJoCo 下验证估速精度（不再喂 `qvel` 真值）。

### C. Sim2Real 中间层（几乎空白）

- [ ] **无 policy 导出管线**：`play_mujoco.py` 靠 `torch.load` + 硬编码重建 `[256,256,128]` MLP，结构一变就静默错。
  - → 加 **ONNX / TorchScript 导出 + 数值一致性校验**（onnx 输出 == torch 输出），真机推理也复用。
- [ ] **MuJoCo 部署是「假」sim2real**：从 `data.qvel` 直接读机身速度喂 obs（继续用特权量），只验证了网络数值复现，没验证「真机拿不到的量怎么办」。
  - → 出一版「真机同款受限观测 + 估计器」的 MuJoCo 闭环。
- [ ] **缺真机 obs 构建 / 单位对齐文档**：IMU 系→机体系旋转；关节方向/零位（注意编码器掉电丢整圈的隐患）；关节顺序映射（`joint_map.default.json` 未与 obs 的 12 维顺序对账）。

### D. 实机部署侧（RL 与硬件之间断开）

- [ ] **`robot_web.py` 里没有任何 RL 推理**：真机现在只会 PD 站立，policy → 真机这条链完全没接（站起来 → 走起来 之间最大空洞）。
- [ ] **无实时控制回路**：robot_web 是网页指令式，不是定频实时控制器。
  - → 写独立的 **50 Hz 确定性控制进程**（与训练 decimation 对齐）：读 IMU+编码器 → 组 obs → policy → 下发力矩/位置。
- [ ] **无 IMU 接入与机身状态估计**：obs 需 ang_vel + projected_gravity（IMU 可给）+ lin_vel（必须估计）；真机侧暂无 IMU 读取代码。
- [ ] **RL 上机安全**：已有急停（estop），但缺「policy 输出限幅 / 看门狗 / 姿态超限自动软急停」。
  - → 先在悬吊/低增益下空跑验证 obs 正确，再落地。

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
