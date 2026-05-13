#!/usr/bin/env bash
set -euo pipefail

# StackForce SimReady Isaac Lab / Isaac Sim environment bootstrap.
# Tested target: Isaac Sim 5.1.0, Isaac Lab v2.3.2, Python 3.11, Torch 2.7.0 cu128.
# Override defaults with: ENV_NAME=env_isaaclab INSTALL_ROOT=$HOME ./setup_stackforce_isaac_lab_sim_env.sh

ENV_NAME="${ENV_NAME:-env_isaaclab}"
INSTALL_ROOT="${INSTALL_ROOT:-$HOME}"
ISAACLAB_DIR="${ISAACLAB_DIR:-$INSTALL_ROOT/IsaacLab}"
LEGGED_GYM_EX_DIR="${LEGGED_GYM_EX_DIR:-$HOME/leggedgymex/LeggedGym-Ex}"
ISAACLAB_TAG="${ISAACLAB_TAG:-v2.3.2}"

if ! command -v conda >/dev/null 2>&1; then
  echo "conda was not found. Install Miniconda/Anaconda first, then rerun this script." >&2
  exit 1
fi

if ! command -v git >/dev/null 2>&1; then
  echo "git was not found. Install git first, then rerun this script." >&2
  exit 1
fi

CONDA_BASE="$(conda info --base)"
source "$CONDA_BASE/etc/profile.d/conda.sh"

if ! conda env list | awk '{print $1}' | grep -qx "$ENV_NAME"; then
  conda create -n "$ENV_NAME" python=3.11 -y
fi
conda activate "$ENV_NAME"

python -m pip install --upgrade pip setuptools wheel
python -m pip install "isaacsim[all,extscache]==5.1.0" --extra-index-url https://pypi.nvidia.com
python -m pip install -U torch==2.7.0 torchvision==0.22.0 --index-url https://download.pytorch.org/whl/cu128

if [ ! -d "$ISAACLAB_DIR/.git" ]; then
  mkdir -p "$(dirname "$ISAACLAB_DIR")"
  git clone https://github.com/isaac-sim/IsaacLab.git "$ISAACLAB_DIR"
fi

cd "$ISAACLAB_DIR"
git fetch --tags
git checkout "$ISAACLAB_TAG"
./isaaclab.sh --install
./isaaclab.sh --install rsl_rl || true

# SimReady has been validated with the rsl_rl package bundled by LeggedGym-Ex.
if [ ! -d "$LEGGED_GYM_EX_DIR/.git" ]; then
  mkdir -p "$(dirname "$LEGGED_GYM_EX_DIR")"
  git clone https://github.com/lupinjia/LeggedGym-Ex.git "$LEGGED_GYM_EX_DIR"
fi

python -m pip uninstall -y rsl-rl rsl-rl-lib >/dev/null 2>&1 || true
python -m pip install -e "$LEGGED_GYM_EX_DIR" --no-deps

python - <<'PY'
import importlib.metadata as md
import sys
import torch
import rsl_rl

print("StackForce Isaac Lab / Isaac Sim environment is ready.")
print("python:", sys.version.split()[0])
print("torch:", torch.__version__)
for name in ("isaacsim", "isaaclab", "isaaclab_rl", "isaaclab_tasks"):
    try:
        print(f"{name}:", md.version(name))
    except Exception as exc:
        print(f"{name}: not found ({exc})")
print("rsl_rl path:", rsl_rl.__file__)
PY

echo ""
echo "Next step for a SimReady export:"
echo "  conda activate $ENV_NAME"
echo "  cd <exported_project>"
echo "  python -m pip install -e source/<package_name>"
echo "  python scripts/list_envs.py"
echo "  python scripts/rsl_rl/train.py --task <TaskName> --headless --num_envs 64 --max_iterations 100"
