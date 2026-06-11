"""Append a new experiment entry to the ledger (doc/experiments/EXPERIMENTS.md).

Why this exists: SwanLab / TensorBoard capture the *metrics*, but not the
*narrative* — why you ran an experiment, what you changed relative to last
time, and what you concluded. That narrative is what you'll actually want when
you look back in three months. This script makes logging that narrative cheap
and link-complete: it auto-pulls what a machine can know (git commit, nearest
milestone tag, run name, seed / iterations from the training dir) so you only
hand-write the parts a machine can't (hypothesis, conclusion).

Typical use, right after a training run finishes:

    python tools/log_run.py \
        --run-dir logs/rsl_rl/mos_flat/2026-06-10_14-22-01_cmd_cond \
        --title "指令条件化重训 (obs 45->48)" \
        --hypothesis "指令进 obs 后策略能响应 cmd_vx，解锁速度-力矩扫描" \
        --changes "obs 45->48 (vx/vy/wz)；同步 rl_deploy._build_obs + policy_export.OBS_DIM" \
        --metrics "reward 18.2->19.1, near_limit_frac 0.81->0.62" \
        --conclusion "速度响应成立；shank 饱和仍偏高 -> 进 EXP 加力矩惩罚" \
        --swanlab-url https://swanlab.cn/@you/mos_one-mos/runs/xxxx

`--run-dir` is optional: when training ran on a remote box and the dir isn't
present locally, the script just skips the auto-extracted fields (you can pass
--seed / --iters manually). Use --dry-run to preview the entry without writing.

The ledger uses HTML marker comments (INDEX:ROWS / ENTRIES:START) as insertion
points; new entries go to the top (highest ID first).
"""

import argparse
import datetime as _dt
import os
import re
import subprocess
import sys

DEFAULT_LEDGER = os.path.join("doc", "experiments", "EXPERIMENTS.md")
SWANLAB_PROJECT = "mos_one-mos"

STATUS_ALIASES = {
    "done": "✅ 完成",
    "wip": "🔄 进行中",
    "planned": "📋 计划",
    "dropped": "❌ 放弃",
}


# --------------------------------------------------------------------------- #
# helpers: git / training-dir extraction (all degrade gracefully to None)
# --------------------------------------------------------------------------- #
def _run(cmd):
    try:
        out = subprocess.run(cmd, capture_output=True, text=True, check=False)
        return out.stdout.strip() if out.returncode == 0 else None
    except (OSError, ValueError):
        return None


def repo_root():
    top = _run(["git", "rev-parse", "--show-toplevel"])
    if top:
        return top
    # fallback: assume tools/ lives directly under the repo root
    return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def git_commit():
    h = _run(["git", "rev-parse", "--short", "HEAD"])
    if not h:
        return None
    dirty = _run(["git", "status", "--porcelain"])
    return f"{h}+dirty" if dirty else h


def git_nearest_tag():
    # most recent tag reachable from HEAD; None if no tags yet
    return _run(["git", "describe", "--tags", "--abbrev=0"])


def scan_param(run_dir, key):
    """Light line-scan for `key: value` in params/agent.yaml (no pyyaml dep)."""
    if not run_dir:
        return None
    path = os.path.join(run_dir, "params", "agent.yaml")
    if not os.path.isfile(path):
        return None
    pat = re.compile(rf"^\s*{re.escape(key)}\s*:\s*(\S+)")
    try:
        with open(path, encoding="utf-8") as fh:
            for line in fh:
                m = pat.match(line)
                if m:
                    return m.group(1).strip().strip("'\"")
    except OSError:
        return None
    return None


def next_exp_id(ledger_text):
    ids = [int(n) for n in re.findall(r"EXP-(\d{4})", ledger_text)]
    return max(ids) + 1 if ids else 1


# --------------------------------------------------------------------------- #
# rendering
# --------------------------------------------------------------------------- #
def render_entry(exp_id, args, auto):
    eid = f"EXP-{exp_id:04d}"
    status = STATUS_ALIASES.get(args.status, args.status)

    commit = auto["commit"] or "?"
    tag = auto["tag"]
    tag_str = f" ｜ 关联 `{tag}`" if tag else ""

    if args.swanlab_url:
        metric_link = f"[SwanLab run]({args.swanlab_url})"
    elif auto["run_name"]:
        metric_link = f"SwanLab project=`{SWANLAB_PROJECT}` exp=`{auto['run_name']}` _(补 run 链接)_"
    else:
        metric_link = f"SwanLab project=`{SWANLAB_PROJECT}` _(补 run 链接)_"

    env_bits = []
    if auto["seed"]:
        env_bits.append(f"seed={auto['seed']}")
    if auto["iters"]:
        env_bits.append(f"iters={auto['iters']}")
    env_line = args.env
    if env_bits:
        env_line = f"{env_line} ｜ " + " / ".join(env_bits)

    lines = [
        f"## {eid}",
        f"### {args.title} — {status} ({args.date})",
        "",
        f"- **假设**: {args.hypothesis or '_(待填)_'}",
        f"- **改动**: {args.changes or '_(待填)_'}",
        f"- **环境**: {env_line}",
        f"- **commit/tag**: `{commit}`{tag_str}",
    ]
    if auto["run_name"]:
        lines.append(f"- **训练目录**: `{args.run_dir}`")
    lines += [
        f"- **指标**: {metric_link} ｜ {args.metrics or '_(待填)_'}",
        f"- **结论**: {args.conclusion or '_(待填)_'}",
        f"- **真机**: {args.real or '未上机'}",
        "",
    ]
    return eid, "\n".join(lines)


def render_index_row(eid, args):
    status = STATUS_ALIASES.get(args.status, args.status)
    anchor = eid.lower()
    concl = (args.conclusion or "").splitlines()[0] if args.conclusion else "_(待填)_"
    if len(concl) > 40:
        concl = concl[:39] + "…"
    return f"| [{eid}](#{anchor}) | {args.date} | {args.title} | {status} | {concl} |"


def insert_after_marker(text, marker, payload):
    idx = text.find(marker)
    if idx == -1:
        raise SystemExit(f"[log_run] 台账缺少标记 {marker!r}，无法插入。")
    nl = text.find("\n", idx + len(marker))
    at = nl + 1 if nl != -1 else len(text)
    return text[:at] + payload + text[at:]


# --------------------------------------------------------------------------- #
def main():
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("--run-dir", help="训练输出目录 logs/rsl_rl/<exp>/<ts>_<run_name>/（可选；远端训练本地没有时省略）")
    p.add_argument("--title", required=True, help="一句话标题")
    p.add_argument("--hypothesis", help="假设：预期改 X 带来 Y（可证伪）")
    p.add_argument("--changes", help="相对上一版具体改了什么")
    p.add_argument("--metrics", help="关键数字（reward A->B 等），曲线交给 SwanLab")
    p.add_argument("--conclusion", help="结论 + 下一步指向")
    p.add_argument("--real", help="真机表现（默认：未上机）")
    p.add_argument("--swanlab-url", help="SwanLab run 链接")
    p.add_argument("--status", default="done",
                   help="done|wip|planned|dropped 或自定义（默认 done）")
    p.add_argument("--date", default=_dt.date.today().isoformat(), help="日期（默认今天）")
    p.add_argument("--env", default="env_isaaclab + RTX 5090", help="环境描述")
    p.add_argument("--seed", help="手填 seed（覆盖自动提取）")
    p.add_argument("--iters", help="手填 iterations（覆盖自动提取）")
    p.add_argument("--ledger", default=None, help=f"台账路径（默认 {DEFAULT_LEDGER}）")
    p.add_argument("--dry-run", action="store_true", help="只打印，不写入")
    args = p.parse_args()

    root = repo_root()
    ledger_path = args.ledger or os.path.join(root, DEFAULT_LEDGER)
    if not os.path.isfile(ledger_path):
        raise SystemExit(f"[log_run] 找不到台账：{ledger_path}")

    run_dir = args.run_dir
    if run_dir and not os.path.isdir(run_dir):
        print(f"[log_run] 注意：--run-dir 不存在（{run_dir}），跳过自动提取，仅记录路径字符串。",
              file=sys.stderr)

    auto = {
        "commit": git_commit(),
        "tag": git_nearest_tag(),
        "run_name": os.path.basename(run_dir.rstrip("/")) if run_dir else None,
        "seed": args.seed or scan_param(run_dir, "seed"),
        "iters": args.iters or scan_param(run_dir, "max_iterations"),
    }

    with open(ledger_path, encoding="utf-8") as fh:
        text = fh.read()

    exp_id = next_exp_id(text)
    eid, entry = render_entry(exp_id, args, auto)
    row = render_index_row(eid, args)

    if args.dry_run:
        print(f"[log_run] (dry-run) 下一个编号：{eid}\n")
        print("索引行：\n" + row + "\n")
        print("条目：\n" + entry)
        return 0

    text = insert_after_marker(text, "<!-- INDEX:ROWS -->", row + "\n")
    text = insert_after_marker(text, "<!-- ENTRIES:START -->", "\n" + entry)

    with open(ledger_path, "w", encoding="utf-8") as fh:
        fh.write(text)

    print(f"[log_run] 已写入 {eid} → {ledger_path}")
    if not auto["tag"]:
        print("[log_run] 提示：仓库还没有 tag，commit 未关联里程碑。见 doc/version_tracking.md 补打 tag。")
    if auto["run_name"] and not args.swanlab_url:
        print(f"[log_run] 提示：记得把 SwanLab run 链接补进 {eid}（exp=`{auto['run_name']}`）。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
