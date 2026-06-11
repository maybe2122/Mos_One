#!/usr/bin/env python3
"""Convert a legacy split-actor/critic rsl_rl checkpoint into the current
`model_state_dict` layout so the installed rsl_rl `OnPolicyRunner.load` accepts it.

Why this exists: the converged checkpoints under `logs/rsl_rl/.../model_*.pt`
were trained with an OLDER rsl_rl whose checkpoint stores the policy as separate
`actor_state_dict` / `critic_state_dict` (keys `mlp.N.*` + `distribution.std_param`).
The rsl_rl currently installed in env_isaaclab instead expects a single
`model_state_dict` with keys `actor.N.*` / `critic.N.*` / `std`, loaded
strict=True. So `eval.py` / `play.py` fail on the old files with
`KeyError: 'model_state_dict'`.

The two layouts are the same [256,256,128]-ELU ActorCritic with identical shapes
— only the key prefixes differ — so the conversion is a pure key remap:

    actor_state_dict["mlp.N.{w,b}"]            -> model_state_dict["actor.N.{w,b}"]
    actor_state_dict["distribution.std_param"] -> model_state_dict["std"]
    critic_state_dict["mlp.N.{w,b}"]           -> model_state_dict["critic.N.{w,b}"]

`optimizer_state_dict` / `iter` / `infos` are carried through unchanged (the new
runner loads the optimizer because ActorCritic.load_state_dict returns "resumed").

Example:
    python tools/ckpt/convert_checkpoint_to_rsl_rl.py \
        logs/rsl_rl/mos2026_2_closed_usd/2026-06-07_22-11-43/model_750.pt
    # -> writes ..._rslrl.pt next to it (or pass --out)
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import torch


def is_legacy(ckpt: dict) -> bool:
    return "actor_state_dict" in ckpt and "model_state_dict" not in ckpt


def remap_module(state: dict, src_prefix: str, dst_prefix: str) -> dict:
    """Rename keys `<src_prefix>N...` -> `<dst_prefix>N...`."""
    out = {}
    for k, v in state.items():
        if k.startswith(src_prefix):
            out[dst_prefix + k[len(src_prefix):]] = v
    return out


def convert(ckpt: dict) -> dict:
    actor = ckpt["actor_state_dict"]
    critic = ckpt["critic_state_dict"]

    model_state: dict = {}
    model_state.update(remap_module(actor, "mlp.", "actor."))
    model_state.update(remap_module(critic, "mlp.", "critic."))

    # Gaussian std: legacy "distribution.std_param" -> "std".
    if "distribution.std_param" in actor:
        model_state["std"] = actor["distribution.std_param"]
    elif "std" in actor:
        model_state["std"] = actor["std"]
    else:
        raise KeyError("legacy actor_state_dict missing std (distribution.std_param)")

    return {
        "model_state_dict": model_state,
        "optimizer_state_dict": ckpt.get("optimizer_state_dict", {}),
        "iter": ckpt.get("iter", 0),
        "infos": ckpt.get("infos", None),
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("checkpoint", help="legacy .pt with actor_state_dict/critic_state_dict")
    ap.add_argument("--out", default=None,
                    help="output path (default: <stem>_rslrl.pt next to input)")
    ap.add_argument("--force", action="store_true", help="overwrite existing output")
    args = ap.parse_args()

    src = Path(args.checkpoint)
    if not src.exists():
        print(f"[error] not found: {src}", file=sys.stderr)
        return 2

    ckpt = torch.load(src, map_location="cpu", weights_only=False)
    if not is_legacy(ckpt):
        if "model_state_dict" in ckpt:
            print(f"[skip] already new-format (has model_state_dict): {src}")
            return 0
        print(f"[error] unrecognized checkpoint keys: {list(ckpt.keys())}", file=sys.stderr)
        return 1

    out = Path(args.out) if args.out else src.with_name(src.stem + "_rslrl.pt")
    if out.exists() and not args.force:
        print(f"[error] output exists (use --force): {out}", file=sys.stderr)
        return 1

    new_ckpt = convert(ckpt)
    n_actor = sum(k.startswith("actor.") for k in new_ckpt["model_state_dict"])
    n_critic = sum(k.startswith("critic.") for k in new_ckpt["model_state_dict"])
    torch.save(new_ckpt, out)
    print(f"[ok] converted legacy checkpoint")
    print(f"     source : {src}")
    print(f"     output : {out}")
    print(f"     keys   : actor={n_actor} critic={n_critic} std=1, iter={new_ckpt['iter']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
