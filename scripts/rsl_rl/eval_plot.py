"""把 eval.py 产出的 JSON 画成图（纯 matplotlib，不依赖 Isaac Lab / torch）。

生成两类图：
  1. 每个 JSON 一张「单关节力矩」图：|τ|均值 + rms 柱状 + |τ|峰值标记 + 力矩上限线，
     柱子按「近上限占比」着色（绿/橙/红），一眼看出哪些电机被钉在上限。
     同图副面板画每只脚的占空比。
  2. 多个 JSON 时额外一张「条件对比」图：成功率 / vx_RMSE / CoT / 力矩近上限占比
     四个关键指标在各测试条件下的对比，用来看泛化/鲁棒性退化。

用法：
  # 单个结果
  python scripts/rsl_rl/eval_plot.py logs/rsl_rl/mos2026_2_closed_usd/<RUN>/eval/eval.json

  # 整个 eval 目录（自动画对比图）
  python scripts/rsl_rl/eval_plot.py logs/rsl_rl/mos2026_2_closed_usd/<RUN>/eval

PNG 默认写到每个 JSON 同目录（<tag>_torque.png），对比图写到 eval 目录（_compare.png）。
"""

import argparse
import glob
import json
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib import font_manager  # noqa: E402


def _setup_cjk_font():
    """挑一个系统里能显示中文的字体，避免中文渲染成方框；找不到就算了。"""
    installed = {f.name for f in font_manager.fontManager.ttflist}
    for name in ("Noto Sans CJK SC", "Noto Sans CJK JP", "Source Han Sans SC",
                 "WenQuanYi Zen Hei", "WenQuanYi Micro Hei", "Microsoft YaHei",
                 "SimHei", "Droid Sans Fallback", "AR PL UMing CN"):
        if name in installed:
            plt.rcParams["font.sans-serif"] = [name]
            break
    plt.rcParams["axes.unicode_minus"] = False  # 负号正常显示


_setup_cjk_font()


def _color(near_frac):
    if near_frac >= 0.5:
        return "#d64545"      # 红：超过一半时间贴上限
    if near_frac >= 0.2:
        return "#e08a1e"      # 橙：经常接近上限
    return "#2a8a3e"          # 绿：裕度充足


def _load(paths):
    files = []
    for p in paths:
        if os.path.isdir(p):
            files += sorted(glob.glob(os.path.join(p, "*.json")))
        else:
            files.append(p)
    out = []
    for f in files:
        try:
            with open(f) as fh:
                out.append((f, json.load(fh)))
        except (OSError, json.JSONDecodeError) as exc:
            print(f"[WARN] 跳过 {f}: {exc}")
    return out


def plot_torque(path, result):
    m = result["metrics"]
    tag = result.get("condition", {}).get("tag", "eval")
    pj = m.get("per_joint_torque", [])
    if not pj:
        return None
    names = [j["joint"] for j in pj]
    mean = [j["abs_mean"] for j in pj]
    rms = [j["rms"] for j in pj]
    amax = [j["abs_max"] for j in pj]
    near = [j["near_limit_frac"] for j in pj]
    lim = m.get("torque_limit", 12.0)
    x = range(len(names))

    feet = m.get("per_foot", [])
    ncols = 2 if feet else 1
    fig, axes = plt.subplots(1, ncols, figsize=(8.5 * ncols, 5.2), squeeze=False)

    # --- 左：每关节力矩 ---
    ax = axes[0][0]
    colors = [_color(n) for n in near]
    ax.bar(x, mean, color=colors, label="|τ| 均值", zorder=3)
    ax.bar(x, rms, color="none", edgecolor="black", linewidth=0.8, label="rms", zorder=4)
    ax.scatter(list(x), amax, color="black", marker="v", s=28, label="|τ| 峰值", zorder=5)
    ax.axhline(lim, color="red", linestyle="--", linewidth=1.2, label=f"上限 {lim:g} N·m")
    ax.axhline(0.8 * lim, color="orange", linestyle=":", linewidth=1.0, label="80% 上限")
    for xi, n in zip(x, near):
        ax.text(xi, lim * 1.02, f"{n*100:.0f}%", ha="center", va="bottom", fontsize=7,
                color=_color(n), rotation=0)
    ax.set_xticks(list(x))
    ax.set_xticklabels(names, rotation=45, ha="right", fontsize=8)
    ax.set_ylabel("N·m")
    ax.set_ylim(0, lim * 1.15)
    ax.set_title(f"每关节力矩  tag={tag}  (柱顶%=近上限占比)", fontsize=10)
    ax.legend(fontsize=7, loc="lower right", ncol=2)
    ax.grid(True, axis="y", alpha=0.3)

    # --- 右：每脚占空比 ---
    if feet:
        ax2 = axes[0][1]
        fnames = [f["foot"] for f in feet]
        duty = [f["duty_factor"] for f in feet]
        td = [f.get("touchdown_hz", 0.0) for f in feet]
        fx = range(len(fnames))
        ax2.bar(fx, duty, color="#3a6ea5", zorder=3)
        for xi, (d, h) in zip(fx, zip(duty, td)):
            ax2.text(xi, d + 0.01, f"{d:.2f}\n{h:.1f}Hz", ha="center", va="bottom", fontsize=7)
        ax2.set_xticks(list(fx))
        ax2.set_xticklabels(fnames, rotation=30, ha="right", fontsize=8)
        ax2.set_ylabel("占空比 duty factor")
        ax2.set_ylim(0, 1.0)
        title = "每脚占空比 / 触地频率"
        if sum(duty) == 0:
            title += "  ⚠ 全 0：接触阈值过低，用 --foot_contact_height 调大"
        ax2.set_title(title, fontsize=10)
        ax2.grid(True, axis="y", alpha=0.3)

    fig.suptitle(
        f"成功率 {m.get('success_rate', float('nan'))*100:.0f}%  "
        f"vx {m.get('mean_speed_vx', float('nan')):.2f}/{result['condition'].get('cmd_vx', '?')}  "
        f"RMSE {m.get('vx_rmse', float('nan')):.3f}  "
        f"base高 {m.get('mean_base_height', float('nan')):.3f}/{m.get('base_height_target', '?')}  "
        f"CoT {m.get('mean_cot', float('nan')):.2f}  功率 {m.get('mean_power_w', float('nan')):.0f}W",
        fontsize=9, y=1.00,
    )
    fig.tight_layout()
    out = os.path.join(os.path.dirname(path), f"{tag}_torque.png")
    fig.savefig(out, dpi=120, bbox_inches="tight")
    plt.close(fig)
    return out


def plot_compare(results, out_dir):
    """多条件对比：4 个关键指标的横向条形图。"""
    rows = []
    for _, r in results:
        m = r["metrics"]
        rows.append((
            r.get("condition", {}).get("tag", "?"),
            m.get("success_rate", float("nan")),
            m.get("vx_rmse", float("nan")),
            m.get("mean_cot", float("nan")),
            m.get("torque_near_limit_frac_all", float("nan")),
        ))
    rows.sort(key=lambda t: t[0])
    tags = [t[0] for t in rows]
    y = range(len(tags))
    panels = [
        ("成功率", [t[1] * 100 for t in rows], "%", "#2a8a3e"),
        ("vx_RMSE", [t[2] for t in rows], "m/s", "#3a6ea5"),
        ("CoT", [t[3] for t in rows], "", "#e08a1e"),
        ("力矩近上限占比", [t[4] * 100 for t in rows], "%", "#d64545"),
    ]
    fig, axes = plt.subplots(1, 4, figsize=(15, max(3, 0.45 * len(tags) + 1.5)), squeeze=False)
    for ax, (title, vals, unit, color) in zip(axes[0], panels):
        ax.barh(list(y), vals, color=color, zorder=3)
        for yi, v in zip(y, vals):
            ax.text(v, yi, f" {v:.2f}{unit}", va="center", fontsize=7)
        ax.set_yticks(list(y))
        ax.set_yticklabels(tags, fontsize=8)
        ax.invert_yaxis()
        ax.set_title(title, fontsize=10)
        ax.grid(True, axis="x", alpha=0.3)
    fig.suptitle("条件对比（成功率越高越好；RMSE / CoT / 近上限占比越低越好）", fontsize=11)
    fig.tight_layout()
    out = os.path.join(out_dir, "_compare.png")
    fig.savefig(out, dpi=120, bbox_inches="tight")
    plt.close(fig)
    return out


def main():
    ap = argparse.ArgumentParser(description="Plot eval.py JSON outputs.")
    ap.add_argument("paths", nargs="+", help="JSON 文件或包含 *.json 的目录")
    args = ap.parse_args()
    results = _load(args.paths)
    if not results:
        print("没有找到任何评估 JSON。")
        return
    for path, r in results:
        out = plot_torque(path, r)
        if out:
            print(f"[INFO] 力矩图已保存: {out}")
    if len(results) > 1:
        # 对比图写到第一个 JSON 所在目录
        out_dir = os.path.dirname(results[0][0])
        print(f"[INFO] 对比图已保存: {plot_compare(results, out_dir)}")


if __name__ == "__main__":
    main()
