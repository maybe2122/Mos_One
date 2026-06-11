# deploy/common — 部署/控制通用工具

与 Isaac 无关、可独立运行的运动学/步态/动力学模块。只依赖 `numpy`（出图另需
`matplotlib`），仓库根目录的 `.venv` 已装齐：

```bash
# 统一用仓库根目录的 .venv 解释器（无 python 别名，torch 也不在里面，但本目录用不到）
.venv/bin/python deploy/common/<脚本>.py ...
```

## 模块一览（依赖自下而上）

| 模块 | 内容 | 自测 / 演示 |
|---|---|---|
| `kinematics.py` | 单腿/整机 FK、解析 IK、解析 Jacobian（足端速度 ↔ 关节角速度的核心映射） | `--selftest` |
| `gait.py` | 对角小跑（trot）足端轨迹生成器：支撑相贴地直线 + 摆动相摆线 | `--selftest`、`--demo` |
| `dynamics.py` | 静力/蹬地力矩需求、GO-M8010-6 T-N 曲线、减速比选型扫描 | 直接运行 / `--plot` |
| `speed_map.py` | **机身线速度 → 关节/电机角速度** 的解算与可视化（见下） | `--selftest` |

几何参数取自 `deploy/mujoco/assets/mos2026_2.xml`，电机规格取自
`deploy/real/config/mos2026_2.yaml`（GO-M8010-6 @24V，减速比 6.33）。

---

## 线速度 → 驱动电机转速：怎么跑

回答「机器人要走 v m/s，每个电机得转多快？反过来电机转速上限决定了能走多快？」。

物理链条：无打滑约束 `v = 步幅/(占空比·周期)` → 步态足端速度（解析求导）→
Jacobian 逆解关节角速度 → ×6.33 减速比得电机轴转速。两条参考线：训练 sim 软限
15 rad/s（关节侧），电机物理空载上限 30 rad/s。

### 1. 单点查询（控制台输出）

```bash
.venv/bin/python deploy/common/speed_map.py --speed 1.0
```

输出该速度下每条腿每个关节的峰值/RMS 角速度（关节侧 rad/s 与电机轴 rpm）、整机
峰值命中哪个关节、是否超训练软限/电机物理限。超限时还会自动反解「至少需要多大
步幅才可行」。

可调步态旋钮（同一速度可用 大步幅+低步频 或 小步幅+高步频 实现，答案不唯一）：

```bash
.venv/bin/python deploy/common/speed_map.py --speed 3.0 --step-length 0.18 --duty 0.5 --step-height 0.04
```

### 2. 速度扫描 + 可视化（出图/CSV）

```bash
.venv/bin/python deploy/common/speed_map.py --sweep            # 默认扫 0~4 m/s
.venv/bin/python deploy/common/speed_map.py --sweep --v-max 3  # 自定上限
```

输出到 `outputs/speed_map/`（matplotlib Agg 后端，无需显示器）：

- `speed_vs_motor_speed.png` — 机身速度 → 峰值/RMS 关节角速度曲线，左轴 rad/s、
  右轴电机 rpm，叠 15/30 rad/s 两条限位线并标注最大可行速度；
- `design_map_speed_steplength.png` — (机身速度 × 步幅) 二维设计图，等高线圈出
  可行域，用来选步态参数；
- `speed_sweep.csv` — 逐速度的峰值/RMS 关节角速度与电机 rad/s、rpm。

参考结论（默认步幅 10 cm、β=0.5）：≤ 训练软限的最大速度 ≈ **1.04 m/s**，
≤ 电机物理限 ≈ **2.02 m/s**；要更快需加大步幅（见设计图）。

### 3. 自测（公式/Jacobian/无打滑约束自洽验证）

```bash
.venv/bin/python deploy/common/speed_map.py --selftest
```

### 程序化调用

```python
from speed_map import gait_for_speed, cycle_kinematics, summarize, instant

g = gait_for_speed(1.5, step_length=0.12)      # 目标 1.5 m/s 的 trot 步态
res = cycle_kinematics(g)                       # 一个周期：q/qd/foot/电机轴角速度
print(summarize(res)["peak_motor_rpm"])
q, qd = instant(g, t=0.123)                     # 单时刻 (4,3)，供实时可视化逐帧调用
```

---

## 其他入口

```bash
.venv/bin/python deploy/common/kinematics.py --selftest   # FK/IK/Jacobian 往返验证
.venv/bin/python deploy/common/gait.py --demo             # 关节目标曲线/足端轨迹图 → outputs/gait_demo/
.venv/bin/python deploy/common/dynamics.py                # 力矩需求 + 减速比选型报告（控制台）
.venv/bin/python deploy/common/dynamics.py --plot         # 另出 T-N 曲线等图 → outputs/dynamics/
```

## ⚠️ 注意事项

- **闭链膝**：膝由平行四连杆驱动，本目录输出的是「等效膝关节」角度/角速度；shank
  电机轴看到的 ≈ 该值 × 连杆传动比（≈1，未标定，见 `kinematics.py` 文首与 todo §E）。
  hip/thigh 为直驱，映射干净。
- **约定角 ≠ 电机角**：所有模块产出「约定角」，下发真机/仿真前需做每关节仿射映射
  `q_motor = sign·q + offset`（零位/轴向见 `config/mos2026_2.yaml`）。
- `L2 = 0.16 m`（小腿长）是按站立几何估计的，XML 无 foot site，待标定。
