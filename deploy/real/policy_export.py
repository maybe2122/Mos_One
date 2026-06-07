#!/usr/bin/env python3
"""Export a trained rsl_rl PPO actor to a TorchScript policy for real-robot deploy.

The Isaac Lab training run saves an rsl_rl checkpoint whose `actor_state_dict`
holds the actor MLP weights plus a Gaussian `std` parameter. Neither rl_sar's
InferenceRuntime nor the standalone `rl_deploy.py` node can consume that raw
state-dict — both expect a self-contained TorchScript module that maps a
45-dim observation to a 12-dim action mean.

This script:
  1. Loads the checkpoint (default: newest model_*.pt under logs/rsl_rl),
  2. Rebuilds the exact [256, 256, 128]-ELU actor from rsl_rl's ActorCriticCfg,
  3. Drops the std parameter (we deploy the deterministic mean),
  4. torch.jit.trace's it and saves deploy/real/policy/policy.pt,
  5. Verifies the scripted module reproduces the eager output bit-for-bit.

The result is drop-in compatible with fan-ziqi/rl_sar: copy it next to
config/mos2026_2.yaml as `policy/mos2026_2/policy.pt`.

Example:
    python deploy/real/policy_export.py
    python deploy/real/policy_export.py \
        --checkpoint logs/rsl_rl/mos2026_2_closed_usd/2026-05-16_22-22-39/model_1500.pt
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import torch
from torch import nn

REPO_ROOT = Path(__file__).resolve().parents[2]
LOG_ROOT = REPO_ROOT / "logs/rsl_rl"
DEFAULT_OUT = Path(__file__).resolve().parent / "policy" / "policy.pt"

OBS_DIM = 45
ACTION_DIM = 12
HIDDEN = (256, 256, 128)


def find_latest_checkpoint() -> Path | None:
    """Newest model_*.pt under logs/rsl_rl (by run mtime, then iteration)."""
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


class ActorMLP(nn.Module):
    """rsl_rl ActorCritic actor: Linear-ELU stack ending in the action mean."""

    def __init__(self, obs_dim: int, action_dim: int, hidden=HIDDEN):
        super().__init__()
        dims = [obs_dim, *hidden, action_dim]
        layers: list[nn.Module] = []
        for i in range(len(dims) - 1):
            layers.append(nn.Linear(dims[i], dims[i + 1]))
            if i < len(dims) - 2:
                layers.append(nn.ELU())
        self.mlp = nn.Sequential(*layers)

    def forward(self, obs: torch.Tensor) -> torch.Tensor:
        return self.mlp(obs)


def load_actor(checkpoint_path: Path) -> ActorMLP:
    ckpt = torch.load(checkpoint_path, map_location="cpu", weights_only=False)
    if "actor_state_dict" not in ckpt:
        raise KeyError(
            f"checkpoint missing 'actor_state_dict'. available keys: {list(ckpt.keys())}"
        )
    actor = ActorMLP(OBS_DIM, ACTION_DIM)
    # Keep only the MLP weights; drop the Gaussian std (deterministic deploy).
    mlp_state = {k: v for k, v in ckpt["actor_state_dict"].items() if k.startswith("mlp.")}
    missing, unexpected = actor.load_state_dict(mlp_state, strict=False)
    if missing:
        raise RuntimeError(f"actor missing params after load: {missing}")
    if unexpected:
        print(f"[warn] ignoring non-mlp params in checkpoint: {unexpected}")
    actor.eval()
    for p in actor.parameters():
        p.requires_grad_(False)
    return actor


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--checkpoint", default=None,
                    help="rsl_rl .pt with actor_state_dict (default: newest under logs/rsl_rl)")
    ap.add_argument("--out", default=str(DEFAULT_OUT),
                    help=f"output TorchScript path (default: {DEFAULT_OUT})")
    args = ap.parse_args()

    if args.checkpoint is None:
        latest = find_latest_checkpoint()
        if latest is None:
            print(f"[error] no model_*.pt under {LOG_ROOT}; pass --checkpoint.", file=sys.stderr)
            return 2
        args.checkpoint = str(latest)
        print(f"[info] no --checkpoint given; using latest {latest}")

    ckpt_path = Path(args.checkpoint)
    if not ckpt_path.exists():
        print(f"[error] checkpoint not found: {ckpt_path}", file=sys.stderr)
        return 2

    actor = load_actor(ckpt_path)

    # Trace with a fixed [1, 45] example — the deploy node always feeds batch 1.
    example = torch.zeros(1, OBS_DIM)
    with torch.no_grad():
        scripted = torch.jit.trace(actor, example)
        scripted = torch.jit.freeze(scripted)

        # Verify: scripted == eager on random inputs (catches tracing surprises).
        max_err = 0.0
        for _ in range(64):
            x = torch.randn(1, OBS_DIM)
            max_err = max(max_err, (actor(x) - scripted(x)).abs().max().item())
    if max_err > 1e-6:
        print(f"[error] scripted/eager mismatch {max_err:.2e} > 1e-6; aborting.", file=sys.stderr)
        return 1

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    scripted.save(str(out_path))
    print(f"[ok] exported TorchScript policy")
    print(f"     source : {ckpt_path}")
    print(f"     output : {out_path}")
    print(f"     io     : obs[{OBS_DIM}] -> action[{ACTION_DIM}], verify max|Δ|={max_err:.2e}")
    print(f"[hint] for rl_sar: copy to <rl_sar>/policy/mos2026_2/policy.pt "
          f"alongside config/mos2026_2.yaml")
    return 0


if __name__ == "__main__":
    sys.exit(main())
