"""把 eval.py 产出的多个 JSON 汇总成一张验证表 + CSV。

纯标准库实现，**不依赖 Isaac Lab / torch**，可在任意 Python 环境直接运行。

用法：
  # 汇总某次训练 run 的 eval/ 目录
  python scripts/rsl_rl/eval_report.py logs/rsl_rl/mos2026_2_closed_usd/<RUN>/eval

  # 或显式给多个 json
  python scripts/rsl_rl/eval_report.py a.json b.json c.json --csv report.csv
"""

import argparse
import csv
import glob
import json
import os


# 验证表里展示的列（key, 表头, 格式化函数）
COLUMNS = [
    ("tag", "条件", lambda v: str(v)),
    ("success_rate", "成功率%", lambda v: f"{v*100:.1f}"),
    ("fall_rate", "跌倒率%", lambda v: f"{v*100:.1f}"),
    ("mean_survival_s", "存活s", lambda v: f"{v:.1f}"),
    ("vx_rmse", "vx_RMSE", lambda v: f"{v:.3f}"),
    ("yaw_mae", "yaw_MAE", lambda v: f"{v:.3f}"),
    ("mean_base_height", "base高", lambda v: f"{v:.3f}"),
    ("mean_cot", "CoT", lambda v: f"{v:.2f}"),
    ("mean_power_w", "功率W", lambda v: f"{v:.0f}"),
    ("torque_abs_max_all", "|τ|max", lambda v: f"{v:.1f}"),
    ("torque_near_limit_frac_all", "近上限%", lambda v: f"{v*100:.1f}"),
    ("mean_duty_factor", "占空比", lambda v: f"{v:.2f}"),
    ("diag_inphase_rate", "对角相位", lambda v: f"{v:.2f}"),
]


def _load(paths):
    files = []
    for p in paths:
        if os.path.isdir(p):
            files += sorted(glob.glob(os.path.join(p, "*.json")))
        else:
            files.append(p)
    rows = []
    for f in files:
        try:
            with open(f) as fh:
                rows.append(json.load(fh))
        except (OSError, json.JSONDecodeError) as exc:
            print(f"[WARN] 跳过 {f}: {exc}")
    return rows


def _flat(result):
    """把 condition.tag + metrics 摊平成一行 dict。"""
    row = {"tag": result.get("condition", {}).get("tag", "?")}
    row.update({k: v for k, v in result.get("metrics", {}).items() if not isinstance(v, (list, dict))})
    return row


def main():
    ap = argparse.ArgumentParser(description="Aggregate eval.py JSON outputs into a validation table.")
    ap.add_argument("paths", nargs="+", help="JSON 文件或包含 *.json 的目录")
    ap.add_argument("--csv", default=None, help="同时写出 CSV 路径")
    args = ap.parse_args()

    results = _load(args.paths)
    if not results:
        print("没有找到任何评估 JSON。")
        return
    rows = [_flat(r) for r in results]

    # --- 控制台表格 ---
    widths = []
    headers = [h for _, h, _ in COLUMNS]
    cells = []
    for row in rows:
        cell = []
        for key, _, fmt in COLUMNS:
            v = row.get(key)
            cell.append(fmt(v) if isinstance(v, (int, float)) else (str(v) if v is not None else "-"))
        cells.append(cell)
    for i, h in enumerate(headers):
        w = max(len(h), *(len(c[i]) for c in cells)) if cells else len(h)
        widths.append(w)

    def _fmt_row(vals):
        return "  ".join(s.ljust(widths[i]) for i, s in enumerate(vals))

    print("\n" + "=" * (sum(widths) + 2 * len(widths)))
    print(" 四足策略验证汇总表")
    print("=" * (sum(widths) + 2 * len(widths)))
    print(_fmt_row(headers))
    print("-" * (sum(widths) + 2 * len(widths)))
    for c in cells:
        print(_fmt_row(c))
    print("=" * (sum(widths) + 2 * len(widths)))

    # --- CSV：导出所有摊平字段 ---
    if args.csv:
        all_keys = []
        for row in rows:
            for k in row:
                if k not in all_keys:
                    all_keys.append(k)
        with open(args.csv, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=all_keys)
            w.writeheader()
            for row in rows:
                w.writerow(row)
        print(f"\n[INFO] CSV 已写出: {args.csv}")


if __name__ == "__main__":
    main()
