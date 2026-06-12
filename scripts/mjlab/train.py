"""Train the mos2026_2 velocity task with mjlab.

Run from inside mjlab's uv environment (repo root):

    cd third_party/mjlab
    uv run python ../../scripts/mjlab/train.py Mjlab-Velocity-Flat-MosOne \
        --env.scene.num-envs 4096 --agent.logger tensorboard

This wrapper puts mjlab_tasks/ on sys.path (and PYTHONPATH for spawned
worker processes), imports mos_one_mjlab to register the task, then hands
off to mjlab's own train CLI.
"""

import os
import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[2]
_TASKS_DIR = str(_REPO_ROOT / "mjlab_tasks")

sys.path.insert(0, _TASKS_DIR)
# torchrunx workers are fresh processes; propagate the import path to them.
os.environ["PYTHONPATH"] = _TASKS_DIR + os.pathsep + os.environ.get("PYTHONPATH", "")

import mos_one_mjlab  # noqa: F401, E402  (registers Mjlab-Velocity-Flat-MosOne)
from mjlab.scripts.train import main  # noqa: E402

if __name__ == "__main__":
  main()
