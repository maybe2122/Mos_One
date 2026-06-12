"""Deploy a trained HIM checkpoint (him/train.py) onto the MuJoCo MJCF of mos2026_2.

Sim2sim counterpart of deploy/mujoco/play_mujoco.py, but for HIM:
  - obs is the *blind* 45-dim HIM frame [cmd(3), ang_vel_b(3), gravity_b(3),
    joint_pos_rel(12), joint_vel(12), last_clipped_action(12)] — base linear
    velocity is NOT observed, the HIM estimator recovers it from history.
  - a 6-frame newest-first history (270 dim) is maintained exactly like
    him/adapter.py: hist = [current, hist[:-45]]; on (re)set all 6 frames are
    filled with the current frame.
  - the full HIMActorCritic (him/him_rl) is rebuilt and act_inference is used:
    actor([current_45, estimator_vel(3), estimator_latent(16)]).
  - the action pipeline / 50 Hz control over 1000 Hz sim / MJCF handling are
    inherited from play_mujoco.MujocoDeployer unchanged.

Example (any python with torch + mujoco, e.g. env_isaaclab):
    # bare run — auto-picks the newest model_*.pt under him/logs
    ../env_isaaclab/bin/python deploy/mujoco/play_him_mujoco.py

    ../env_isaaclab/bin/python deploy/mujoco/play_him_mujoco.py \
        --checkpoint him/logs/mos2026_2_him/Jun12_23-09-51_/model_150.pt

    # headless validation over SSH
    ../env_isaaclab/bin/python deploy/mujoco/play_him_mujoco.py --headless \
        --duration 5 --checkpoint him/logs/mos2026_2_him/<run>/model_150.pt
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np
import torch

import mujoco
import mujoco.viewer

_THIS_DIR = Path(__file__).resolve().parent
REPO_ROOT = _THIS_DIR.parents[1]
sys.path.insert(0, str(_THIS_DIR))           # play_mujoco
sys.path.insert(0, str(REPO_ROOT / "him"))   # him_rl, him_cfg

from play_mujoco import (  # noqa: E402
    ACTION_CLIP,
    COMMANDED_ANG_VEL_Z,
    COMMANDED_LIN_VEL_XY,
    CONTROL_HZ,
    DEFAULT_JOINT_POS,
    DEFAULT_MJCF,
    SIM_HZ,
    MujocoDeployer,
)
from him_cfg import get_him_train_cfg  # noqa: E402
from him_rl.modules import HIMActorCritic  # noqa: E402

NUM_ONE_STEP_OBS = 45
HISTORY_LENGTH = 6
NUM_ACTOR_OBS = NUM_ONE_STEP_OBS * HISTORY_LENGTH  # 270
NUM_PRIVILEGED_OBS = 51
NUM_ACTIONS = 12

LOG_ROOT = REPO_ROOT / "him" / "logs"


def find_latest_checkpoint() -> Path | None:
    """Newest model_*.pt under him/logs (by run dir mtime, then iter number)."""
    candidates = list(LOG_ROOT.rglob("model_*.pt"))
    if not candidates:
        return None

    def sort_key(p: Path):
        try:
            it = int(p.stem.split("_")[1])
        except (IndexError, ValueError):
            it = -1
        return (p.parent.stat().st_mtime, it)

    return max(candidates, key=sort_key)


def load_him_policy(checkpoint_path: Path):
    """Rebuild HIMActorCritic with the training hyperparams and load the weights."""
    policy_cfg = get_him_train_cfg()["policy"]
    actor_critic = HIMActorCritic(
        NUM_ACTOR_OBS, NUM_PRIVILEGED_OBS, NUM_ONE_STEP_OBS, NUM_ACTIONS, **policy_cfg
    )
    ckpt = torch.load(checkpoint_path, map_location="cpu", weights_only=False)
    if "model_state_dict" not in ckpt:
        raise KeyError(
            f"checkpoint missing 'model_state_dict' (keys: {list(ckpt.keys())}); "
            "this script expects a HIM checkpoint from him/train.py"
        )
    actor_critic.load_state_dict(ckpt["model_state_dict"])
    actor_critic.eval()
    for p in actor_critic.parameters():
        p.requires_grad_(False)
    print(f"[info] HIM checkpoint loaded: iter={ckpt.get('iter')} {checkpoint_path}")
    return actor_critic


class HIMMujocoDeployer(MujocoDeployer):
    """MujocoDeployer with the HIM blind obs layout + 6-frame history."""

    def __init__(self, mjcf_path: Path, command: np.ndarray, payload_kg: float = 0.0):
        super().__init__(mjcf_path)
        self.command = command.astype(np.float32)  # [vx, vy, wz]
        self.obs_history = np.zeros(NUM_ACTOR_OBS, dtype=np.float32)
        if payload_kg > 0.0:
            # Mirror training's --payload: add mass to the base body and scale its
            # inertia by the mass ratio (same uniform-density assumption as Isaac
            # Lab's randomize_rigid_body_mass with recompute_inertia=True).
            m0 = float(self.model.body_mass[self.base_id])
            self.model.body_mass[self.base_id] = m0 + payload_kg
            self.model.body_inertia[self.base_id] *= (m0 + payload_kg) / m0
            print(f"[info] payload: base mass {m0:.2f} -> {m0 + payload_kg:.2f} kg")

    def reset(self) -> None:
        super().reset()
        # fill all 6 history frames with the post-reset frame (adapter.py behaviour)
        self.obs_history = np.tile(self.observe(), HISTORY_LENGTH)

    def observe(self) -> np.ndarray:
        # HIM one-step obs: cmd replaces base lin vel; the rest matches him_env.py.
        vel6 = np.zeros(6)
        mujoco.mj_objectVelocity(self.model, self.data, mujoco.mjtObj.mjOBJ_BODY,
                                 self.base_id, vel6, 1)  # 1 = local frame
        ang_body = vel6[:3]

        base_rot = self.data.xmat[self.base_id].reshape(3, 3)
        gravity_body = base_rot.T @ np.array([0.0, 0.0, -1.0])

        joint_pos_rel = self.data.qpos[self.qpos_idx] - DEFAULT_JOINT_POS
        joint_vel = self.data.qvel[self.qvel_idx]

        obs = np.concatenate([
            self.command, ang_body, gravity_body,
            joint_pos_rel, joint_vel, self.prev_actions,
        ]).astype(np.float32)
        return np.nan_to_num(obs, nan=0.0, posinf=0.0, neginf=0.0)

    def push_history(self) -> np.ndarray:
        """Shift in the current frame newest-first and return the 270-dim input."""
        current = self.observe()
        self.obs_history = np.concatenate(
            [current, self.obs_history[:-NUM_ONE_STEP_OBS]]
        )
        return self.obs_history


def run_loop(args) -> int:
    mjcf = Path(args.mjcf)
    if not mjcf.exists():
        print(f"[error] MJCF not found at {mjcf}. Run tools/asset/usd_to_mjcf.py first.",
              file=sys.stderr)
        return 1

    command = np.array([args.vx, args.vy, args.wz])
    deployer = HIMMujocoDeployer(mjcf, command, payload_kg=args.payload)
    deployer.reset()
    policy = load_him_policy(Path(args.checkpoint))
    print(f"[info] command = (vx={args.vx:+.2f}, vy={args.vy:+.2f}, wz={args.wz:+.2f})")

    decimation = SIM_HZ // CONTROL_HZ
    control_dt = 1.0 / CONTROL_HZ
    max_control_steps = int(args.duration * CONTROL_HZ) if args.duration > 0 else None

    def step_control(step_i: int) -> bool:
        obs_history = deployer.push_history()
        with torch.no_grad():
            action = policy.act_inference(
                torch.from_numpy(obs_history).unsqueeze(0)
            ).squeeze(0).numpy()
        deployer.apply_action(action)
        for _ in range(decimation):
            mujoco.mj_step(deployer.model, deployer.data)
        if not np.all(np.isfinite(deployer.data.qpos)):
            print(f"[warn] non-finite qpos at control step {step_i}; resetting.")
            deployer.reset()
            return False
        return True

    if args.headless:
        print(f"[info] headless run for {args.duration:.1f}s @ {CONTROL_HZ} Hz control")
        t0 = time.time()
        for k in range(max_control_steps or 0):
            step_control(k)
        elapsed = time.time() - t0
        base = deployer.data.qpos[:3]
        print(f"[info] {max_control_steps} steps in {elapsed:.2f}s "
              f"(realtime ratio {(max_control_steps * control_dt) / elapsed:.2f})")
        print(f"[info] base pos = ({base[0]:+.3f}, {base[1]:+.3f}, {base[2]:+.3f})")
        return 0

    with mujoco.viewer.launch_passive(deployer.model, deployer.data) as viewer:
        print("[info] viewer launched. press Esc to quit.")
        k = 0
        next_tick = time.time()
        while viewer.is_running():
            if max_control_steps is not None and k >= max_control_steps:
                break
            step_control(k)
            viewer.sync()
            k += 1
            next_tick += control_dt
            sleep = next_tick - time.time()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_tick = time.time()
    return 0


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--mjcf", default=str(DEFAULT_MJCF))
    ap.add_argument("--checkpoint", default=None,
                    help="HIM .pt from him/train.py (contains model_state_dict). "
                         "Defaults to the newest model_*.pt under him/logs.")
    ap.add_argument("--headless", action="store_true",
                    help="step without launching the viewer (useful over SSH).")
    ap.add_argument("--duration", type=float, default=0.0,
                    help="run for this many seconds; 0 = unlimited (viewer mode) "
                         "or required positive for headless mode.")
    ap.add_argument("--vx", type=float, default=float(COMMANDED_LIN_VEL_XY[0]),
                    help="commanded forward velocity (m/s), default = training cmd")
    ap.add_argument("--vy", type=float, default=float(COMMANDED_LIN_VEL_XY[1]),
                    help="commanded lateral velocity (m/s)")
    ap.add_argument("--wz", type=float, default=float(COMMANDED_ANG_VEL_Z),
                    help="commanded yaw rate (rad/s)")
    ap.add_argument("--payload", type=float, default=0.0,
                    help="extra base mass (kg) to mirror him/train.py --payload; "
                         "0 = stock MJCF. Use the value the checkpoint trained with.")
    return ap.parse_args()


if __name__ == "__main__":
    args = parse_args()
    if args.headless and args.duration <= 0:
        print("[error] --headless requires --duration > 0", file=sys.stderr)
        sys.exit(2)
    if args.checkpoint is None:
        latest = find_latest_checkpoint()
        if latest is None:
            print(f"[error] no model_*.pt found under {LOG_ROOT}; pass --checkpoint.",
                  file=sys.stderr)
            sys.exit(2)
        args.checkpoint = str(latest)
        print(f"[info] no --checkpoint given; using latest {latest}")
    sys.exit(run_loop(args))
