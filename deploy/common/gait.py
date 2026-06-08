"""mos2026_2 足端轨迹步态生成器（Phase 2 传统控制 baseline）。

思路（对应 todo.md Phase 2「足端轨迹 gait」）
---------------------------------------------
传统四足步态在「足端笛卡尔轨迹」里规划，再用 IK 转成关节目标——比在关节空间硬凑
正弦直观得多，也能直接对齐真机的步幅/步高/离地高度。本模块实现一个对角小跑（trot）：

  · 相位：FL+RR 同相，FR+RL 反相（差半个周期）——对角腿成对，trot 的定义。
  · 支撑相（stance, 占空比 β）：足端贴地，相对机身从 +L/2 向后扫到 −L/2，
    机身因此前进（足端不动、机身动）。
  · 摆动相（swing, 1−β）：足端抬起前摆，水平用摆线（cycloid）、竖直用 (1−cos)/2，
    保证离地/触地瞬间水平与竖直速度都为 0 → 平滑、少打滑。

足端目标在每条腿的「髋系」给出（x 前、y 左、z 上），y 固定为侧偏 d（外摆≈0，腿竖直），
经 `kinematics.leg_ik` 得到约定角 (q_ab, q_hip, q_knee)。

⚠️ 输出的是「约定角」。下发真机/仿真前需做每关节仿射映射 q_motor = sign·q + offset，
并处理闭链「shank 电机轴↔等效膝角」传动（见 kinematics.py 文首与 todo.md §E）。

自测：``python deploy/common/gait.py --selftest``
出图/CSV：``python deploy/common/gait.py --demo``
"""

from __future__ import annotations

import argparse
import math
import os
from dataclasses import dataclass

import numpy as np

from kinematics import LEG_NAMES, LEGS, leg_fk, leg_ik

# XML 里每个 hinge 的 range 都是 ±1.57 rad
JOINT_LIMIT = 1.57


@dataclass
class TrotGait:
    """对角小跑步态参数（单位：m / s）。"""

    # body_height 是髋系下足端深度。摆动顶点深度 = body_height − step_height，
    # 须保证此时 reach 不把膝顶到 ±1.57 限位（L1=0.18,L2=0.16 ⇒ 顶点深度 ≳ 0.25）。
    body_height: float = 0.30      # 站立足端深度（< L1+L2=0.34，对应 base 高度 ~0.31）
    step_length: float = 0.10      # 单步前后总位移
    step_height: float = 0.04      # 摆动相离地高度（顶点深度 0.26，膝角余量充足）
    period: float = 0.5            # 一个步态周期时长（s）
    duty: float = 0.5              # 支撑相占空比（trot 取 0.5）
    x_offset: float = 0.0          # 足端默认前后偏置（髋系 x）

    # 对角相位偏移：FL,FR,RL,RR
    phase_offset = {"fl": 0.0, "fr": 0.5, "rl": 0.5, "rr": 0.0}

    def foot_target(self, leg_name: str, t: float) -> np.ndarray:
        """给定腿名与时刻 t，返回髋系足端目标 (x, y, z)。"""
        leg = LEGS[leg_name]
        p = (t / self.period + self.phase_offset[leg_name]) % 1.0
        x0, z0, d = self.x_offset, -self.body_height, leg.d
        L, h, beta = self.step_length, self.step_height, self.duty

        if p < beta:  # 支撑相：+L/2 → −L/2，贴地
            s = p / beta
            x = x0 + L * (0.5 - s)
            z = z0
        else:         # 摆动相：摆线前摆 + (1−cos) 抬腿
            s = (p - beta) / (1.0 - beta)
            x = x0 + L * (-0.5 + (s - math.sin(2 * math.pi * s) / (2 * math.pi)))
            z = z0 + h * (1.0 - math.cos(2 * math.pi * s)) / 2.0
        return np.array([x, d, z])

    def joint_targets(self, t: float) -> dict[str, tuple[float, float, float]]:
        """时刻 t 的 4 腿约定角 {leg: (q_ab, q_hip, q_knee)}。"""
        out = {}
        for name in LEG_NAMES:
            foot = self.foot_target(name, t)
            a, hh, k = leg_ik(foot, LEGS[name], frame="hip", knee_sign=-1.0)
            out[name] = (float(a), float(hh), float(k))
        return out

    def rollout(self, n: int = 200):
        """返回 (times[n], feet[n,4,3], q[n,4,3])，覆盖一个完整周期。"""
        times = np.linspace(0.0, self.period, n, endpoint=False)
        feet = np.zeros((n, 4, 3))
        q = np.zeros((n, 4, 3))
        for ti, t in enumerate(times):
            for li, name in enumerate(LEG_NAMES):
                f = self.foot_target(name, t)
                feet[ti, li] = f
                a, hh, k = leg_ik(f, LEGS[name], frame="hip", knee_sign=-1.0)
                q[ti, li] = (a, hh, k)
        return times, feet, q


# --- 自测 ---------------------------------------------------------------------
def _selftest() -> int:
    ok = True
    gait = TrotGait()
    times, feet, q = gait.rollout(400)

    # 1) IK 可解且足端 FK 往返一致
    print("== 1. 步态 IK 足端往返 ==")
    max_rt = 0.0
    for li, name in enumerate(LEG_NAMES):
        foot_fk = np.stack(
            [leg_fk(q[k, li, 0], q[k, li, 1], q[k, li, 2], LEGS[name], frame="hip") for k in range(len(times))]
        )
        rt = float(np.max(np.linalg.norm(foot_fk - feet[:, li], axis=-1)))
        max_rt = max(max_rt, rt)
    print(f"  足端 FK(IK) 往返 max_err = {max_rt:.2e}")
    ok &= max_rt < 1e-9

    # 2) 关节角在限位内
    print("\n== 2. 关节限位（±1.57 rad）==")
    qmax = float(np.max(np.abs(q)))
    print(f"  |q|_max = {qmax:.3f} rad（限位 {JOINT_LIMIT}）")
    ok &= qmax < JOINT_LIMIT

    # 3) 轨迹连续（相邻采样足端位移、关节角增量有界）
    print("\n== 3. 轨迹连续性（周期首尾相接、无跳变）==")
    # 周期闭合：t=0 与 t=period 的足端目标应逐腿一致（相位 mod 1）
    close = max(
        float(np.linalg.norm(gait.foot_target(n, 0.0) - gait.foot_target(n, gait.period)))
        for n in LEG_NAMES
    )
    dfoot = float(np.max(np.linalg.norm(np.diff(feet, axis=0), axis=-1)))
    dq = float(np.max(np.abs(np.diff(q, axis=0))))
    print(f"  周期闭合误差 = {close:.2e}  相邻足端位移 = {dfoot*1000:.2f} mm/步  关节增量 = {math.degrees(dq):.2f}°")
    ok &= close < 1e-9 and dfoot < 0.01 and dq < 0.1  # 400 采样/周期下应很平滑

    # 4) 对角相位：t=0 时 FL/RR 在支撑、FR/RL 在摆动（或反之），z 应分两组
    print("\n== 4. 对角 trot 相位检查（t=0）==")
    z0 = {n: gait.foot_target(n, 0.0)[2] for n in LEG_NAMES}
    print("  " + "  ".join(f"{n}:z={z0[n]:.3f}" for n in LEG_NAMES))
    diag_ok = abs(z0["fl"] - z0["rr"]) < 1e-9 and abs(z0["fr"] - z0["rl"]) < 1e-9
    print(f"  对角同相（FL≈RR, FR≈RL）: {diag_ok}")
    ok &= diag_ok

    print("\n" + ("✅ 全部通过" if ok else "❌ 有用例失败"))
    return 0 if ok else 1


def _demo() -> int:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    out_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), "outputs", "gait_demo")
    os.makedirs(out_dir, exist_ok=True)

    gait = TrotGait()
    times, feet, q = gait.rollout(300)

    # 关节角曲线（matplotlib 默认字体无中文，标签用英文避免乱码）
    fig, axes = plt.subplots(2, 2, figsize=(11, 7), sharex=True)
    labels = {"ab": "q_ab (abduction)", "hip": "q_hip (thigh)", "knee": "q_knee"}
    for li, name in enumerate(LEG_NAMES):
        ax = axes[li // 2][li % 2]
        for j, key in enumerate(["ab", "hip", "knee"]):
            ax.plot(times, np.degrees(q[:, li, j]), label=labels[key])
        ax.set_title(f"leg {name.upper()}")
        ax.set_ylabel("deg")
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
    axes[1][0].set_xlabel("t (s)")
    axes[1][1].set_xlabel("t (s)")
    fig.suptitle("mos2026_2 trot gait: joint targets from analytic IK (convention angles)")
    fig.tight_layout()
    p1 = os.path.join(out_dir, "joint_targets.png")
    fig.savefig(p1, dpi=120)

    # 足端矢状面轨迹（x-z），看到典型的 "D" 形 trot 轨迹
    fig2, ax2 = plt.subplots(figsize=(7, 5))
    for li, name in enumerate(LEG_NAMES):
        ax2.plot(feet[:, li, 0], feet[:, li, 2], label=name.upper())
    ax2.set_xlabel("x forward (m)")
    ax2.set_ylabel("z up (m)")
    ax2.set_title("Foot sagittal trajectory (hip frame): flat stance + cycloid swing")
    ax2.axis("equal")
    ax2.grid(alpha=0.3)
    ax2.legend()
    fig2.tight_layout()
    p2 = os.path.join(out_dir, "foot_trajectory.png")
    fig2.savefig(p2, dpi=120)

    # CSV：time + 12 关节约定角（leg×[ab,hip,knee]）
    header = "t," + ",".join(f"{n}_{j}" for n in LEG_NAMES for j in ("ab", "hip", "knee"))
    data = np.concatenate([times[:, None], q.reshape(len(times), -1)], axis=1)
    p3 = os.path.join(out_dir, "joint_targets.csv")
    np.savetxt(p3, data, delimiter=",", header=header, comments="")

    # 二维连杆图：髋固定，只画大腿+小腿两段，叠画足端轨迹
    p4 = _plot_linkage("fl", out_dir)

    print("已输出：")
    for p in (p1, p2, p3, p4):
        print(f"  {p}")
    return 0


def _planar_joints(gait: TrotGait, leg_name: str, n: int = 240):
    """矢状面 2 连杆几何（hip 固定原点，q_ab=0）。
    返回 knee[n,2], foot[n,2]（列 = [x 前, z 上]）以及关节角。"""
    leg = LEGS[leg_name]
    L1, L2 = leg.l1, leg.l2
    times = np.linspace(0.0, gait.period, n, endpoint=False)
    q_hip = np.array([gait.joint_targets(t)[leg_name][1] for t in times])
    q_knee = np.array([gait.joint_targets(t)[leg_name][2] for t in times])
    knee = np.stack([L1 * np.sin(q_hip), -L1 * np.cos(q_hip)], axis=-1)
    foot = knee + np.stack(
        [L2 * np.sin(q_hip + q_knee), -L2 * np.cos(q_hip + q_knee)], axis=-1
    )
    return knee, foot, times


def _plot_linkage(leg_name: str, out_dir: str) -> str:
    """画矢状面（x-z）2 连杆姿态序列 + 足端轨迹。髋固定不动。"""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import cm

    gait = TrotGait()
    knee, foot, times = _planar_joints(gait, leg_name)

    fig, ax = plt.subplots(figsize=(7, 7))

    # 1) 多相位叠画连杆姿态（thigh: hip→knee, shank: knee→foot），颜色随相位渐变
    n_pose = 16
    idx = np.linspace(0, len(times) - 1, n_pose, dtype=int)
    colors = cm.viridis(np.linspace(0, 1, n_pose))
    for c, i in zip(colors, idx):
        kx, kz = knee[i]
        fx, fz = foot[i]
        ax.plot([0.0, kx], [0.0, kz], "-", color=c, lw=2.0, alpha=0.7)   # 大腿
        ax.plot([kx, fx], [kz, fz], "-", color=c, lw=2.0, alpha=0.7)     # 小腿
        ax.plot(kx, kz, "o", color=c, ms=4)                              # 膝
        ax.plot(fx, fz, ".", color=c, ms=6)                             # 足端

    # 2) 足端轨迹整圈曲线
    ax.plot(foot[:, 0], foot[:, 1], "k-", lw=1.2, label="foot trajectory")
    # 3) 髋（固定）
    ax.plot(0.0, 0.0, "rs", ms=10, label="hip (fixed)")
    ax.annotate("hip", (0, 0), textcoords="offset points", xytext=(8, 6), color="r")

    ax.set_xlabel("x forward (m)")
    ax.set_ylabel("z up (m)")
    ax.set_title(
        f"leg {leg_name.upper()} sagittal plane: thigh + shank only, hip fixed\n"
        "(link poses across one trot cycle, color = phase)"
    )
    ax.axis("equal")
    ax.grid(alpha=0.3)
    ax.legend(loc="upper right")
    sm = cm.ScalarMappable(cmap="viridis")
    sm.set_array([0, gait.period])
    fig.colorbar(sm, ax=ax, label="phase t (s)", fraction=0.046, pad=0.04)
    fig.tight_layout()
    path = os.path.join(out_dir, "leg_linkage.png")
    fig.savefig(path, dpi=120)
    return path


def main() -> int:
    p = argparse.ArgumentParser(description="mos2026_2 足端轨迹 trot 步态")
    p.add_argument("--selftest", action="store_true", help="运行步态自洽性自测")
    p.add_argument("--demo", action="store_true", help="输出关节曲线/足端轨迹图 + CSV + 连杆图")
    p.add_argument("--linkage", action="store_true", help="只画矢状面 2 连杆（大腿+小腿，髋固定）+ 足端轨迹")
    p.add_argument("--leg", type=str, default="fl", choices=list(LEG_NAMES), help="连杆图选哪条腿（默认 fl）")
    args = p.parse_args()
    if args.selftest:
        return _selftest()
    if args.demo:
        return _demo()
    if args.linkage:
        out_dir = os.path.join(
            os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), "outputs", "gait_demo"
        )
        os.makedirs(out_dir, exist_ok=True)
        path = _plot_linkage(args.leg, out_dir)
        print(f"已输出：{path}")
        return 0
    p.print_help()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
