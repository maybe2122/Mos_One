"""Play / zero-agent check the mos2026_2 velocity task with mjlab.

Run from inside mjlab's uv environment (repo root):

    cd third_party/mjlab
    # MDP sanity check without a trained policy:
    uv run python ../../scripts/mjlab/play.py Mjlab-Velocity-Flat-MosOne --agent zero
    # Play a trained checkpoint:
    uv run python ../../scripts/mjlab/play.py Mjlab-Velocity-Flat-MosOne \
        --checkpoint-path <model.pt>
"""

import os
import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[2]
_TASKS_DIR = str(_REPO_ROOT / "mjlab_tasks")

sys.path.insert(0, _TASKS_DIR)
os.environ["PYTHONPATH"] = _TASKS_DIR + os.pathsep + os.environ.get("PYTHONPATH", "")

import mos_one_mjlab  # noqa: F401, E402  (registers Mjlab-Velocity-Flat-MosOne)
from mjlab.scripts.play import main  # noqa: E402

if __name__ == "__main__":
  main()
