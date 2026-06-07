#!/usr/bin/env bash
# 四足策略完整验证套件：按"从简单到复杂、从仿真泛化到 sim-to-real"的顺序，
# 多次调用 eval.py 跑下面这张验证表的每一行，最后用 eval_report.py 汇总。
#
# 每个测试条件都是一次独立的 Isaac 进程（启动较慢但相互隔离、结果干净）。
#
# 用法：
#   scripts/rsl_rl/eval_suite.sh [LOAD_RUN] [NUM_ENVS] [NUM_STEPS]
# 例：
#   scripts/rsl_rl/eval_suite.sh 2026-06-07_22-11-43 64 3000
#   scripts/rsl_rl/eval_suite.sh                      # 用最新 run，默认 64 env / 3000 步
#
# 只想跑其中几项时，把不需要的行注释掉即可。
set -euo pipefail

LOAD_RUN="${1:-.*}"
NUM_ENVS="${2:-64}"
NUM_STEPS="${3:-3000}"

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PY="python ${ROOT}/scripts/rsl_rl/eval.py --headless --load_run ${LOAD_RUN} --num_envs ${NUM_ENVS} --num_steps ${NUM_STEPS}"

run() { echo; echo ">>> [$1]"; ${PY} --tag "$1" "${@:2}"; }

# === 1. 基线：平地 + 默认前进指令 ===
run flat_default

# === 2. 指令泛化（速度跟踪测试集）===
run cmd_vx0p5     --cmd_vx 0.5
run cmd_vx1p5     --cmd_vx 1.5
run cmd_side      --cmd_vx 0.5 --cmd_vy 0.2
run cmd_turn      --cmd_vx 0.0 --cmd_vy 0.0 --cmd_wz 1.0
run cmd_back      --cmd_vx -0.5 --cmd_wz -0.5

# === 3. 地形泛化 ===
run terrain_rough        --terrain rough
run terrain_curriculum   --terrain curriculum

# === 4. 摩擦泛化 ===
run friction_0p4   --friction 0.4
run friction_0p6   --friction 0.6
run friction_1p0   --friction 1.0

# === 5. 质量/负载 ===
run mass_p10   --mass_scale 1.10
run mass_p20   --mass_scale 1.20

# === 6. 鲁棒性：推搡 ===
run push_0p3   --push_interval_s 2.0 --push_vel 0.3
run push_0p6   --push_interval_s 2.0 --push_vel 0.6
run push_1p0   --push_interval_s 2.0 --push_vel 1.0

# === 7. Sim-to-real：观测噪声 ===
run noise_0p01   --obs_noise 0.01
run noise_0p05   --obs_noise 0.05
run noise_0p10   --obs_noise 0.10

# === 8. Sim-to-real：控制延迟（每步 20ms）===
run delay_20ms   --action_delay 1
run delay_40ms   --action_delay 2
run delay_100ms  --action_delay 5

# === 汇总 ===
echo
echo ">>> 汇总验证表"
# eval.py 默认把 JSON 写到 <checkpoint目录>/eval/，定位它来汇总。
EVAL_DIR="$(find "${ROOT}/logs/rsl_rl" -type d -name eval | sort | tail -1)"
if [ -n "${EVAL_DIR}" ]; then
  python "${ROOT}/scripts/rsl_rl/eval_report.py" "${EVAL_DIR}" --csv "${EVAL_DIR}/validation_table.csv"
fi
