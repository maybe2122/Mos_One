"""One-shot fixer: move the misplaced fr_close_loop joint into /joints/.

The mos2026_2 closed-chain USD authors three of the four leg closure joints
under `/mos2026_2/joints/` (a Scope grouping all 31 other joints) but places
`fr_close_loop` directly under `/mos2026_2/` — the same level as the
articulation root. PhysX' env-clone path fix-up appears to miss this
out-of-place joint, producing at runtime:

    PhysicsUSD: CreateJoint - no bodies defined at body0 and body1,
                joint prim: /World/envs/env_0/Robot/fr_close_loop
    omni.physx.tensors.plugin: Failed to create simulation view:
                no active physics scene found

which cascades into the simulation-view backend never being created.

This script relocates the spec on disk so the asset matches the symmetric
layout of the other three legs. Run once from the repo root:

    /home/sz/code/rl/env_isaaclab/bin/python3 tools/fix_fr_close_loop_path.py
"""
from __future__ import annotations

from pathlib import Path

from pxr import Sdf

USD_PATH = (
    Path(__file__).resolve().parent.parent
    / "source/stackforce_simready_mos2026_2_closed_usd_closed_usd_lab"
    / "stackforce_simready_mos2026_2_closed_usd_closed_usd_lab"
    / "assets/robots/mos2026_2_closed_usd/usd/mos2026_2.usd"
)

SRC_PATH = Sdf.Path("/mos2026_2/fr_close_loop")
DST_PATH = Sdf.Path("/mos2026_2/joints/fr_close_loop")


def main() -> int:
    layer = Sdf.Layer.FindOrOpen(str(USD_PATH))
    if layer is None:
        raise SystemExit(f"could not open layer: {USD_PATH}")

    src_spec = layer.GetPrimAtPath(SRC_PATH)
    dst_parent = layer.GetPrimAtPath(DST_PATH.GetParentPath())
    if src_spec is None:
        print(f"[skip] {SRC_PATH} not present in {USD_PATH.name}; nothing to do.")
        return 0
    if dst_parent is None:
        raise SystemExit(f"target parent {DST_PATH.GetParentPath()} missing in layer")
    if layer.GetPrimAtPath(DST_PATH) is not None:
        raise SystemExit(f"destination {DST_PATH} already exists; refusing to overwrite")

    if not Sdf.CopySpec(layer, SRC_PATH, layer, DST_PATH):
        raise SystemExit(f"Sdf.CopySpec failed for {SRC_PATH} -> {DST_PATH}")
    parent_spec = layer.GetPrimAtPath(SRC_PATH.GetParentPath())
    del parent_spec.nameChildren[SRC_PATH.name]
    if layer.GetPrimAtPath(SRC_PATH) is not None:
        raise SystemExit(f"failed to remove original spec at {SRC_PATH}")

    layer.Save()
    print(f"[ok] moved {SRC_PATH} -> {DST_PATH} in {USD_PATH.name}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
