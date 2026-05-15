"""One-shot cleanup: permanently remove embedded GroundPlane prims from the
closed-chain robot USD asset(s).

Why: the upstream USD ships with a `GroundPlane` child under the robot root
that was used for authoring previews. At runtime Isaac Lab clones it into
every env, where it follows the robot's transform and conflicts with the
real terrain. The training env currently strips them via `SetActive(False)`
on every startup — this script removes them from disk so the strip pass is
no longer needed.

Run with the Isaac Sim Python (so the `pxr` / USD module is available):

    /isaac-sim/python.sh tools/strip_embedded_ground.py [--dry-run]

By default scans every .usd under source/.../assets/robots and writes the
cleaned files in-place. `--dry-run` only prints what would be deleted.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

try:
    from pxr import Usd
except ImportError:
    sys.stderr.write(
        "pxr.Usd not found. Run this script under Isaac Sim's Python, e.g.\n"
        "  /isaac-sim/python.sh tools/strip_embedded_ground.py\n"
    )
    sys.exit(1)


REPO_ROOT = Path(__file__).resolve().parents[1]
ASSET_ROOT = (
    REPO_ROOT
    / "source"
    / "stackforce_simready_mos2026_2_closed_usd_closed_usd_lab"
    / "stackforce_simready_mos2026_2_closed_usd_closed_usd_lab"
    / "assets"
    / "robots"
)

# Names (lower-cased) that count as "demo ground". Same set the runtime
# strip in mos2026_2_closed_usd_env.py looks for.
GROUND_NAMES = {"groundplane", "ground_plane"}


def find_ground_prims(stage: Usd.Stage) -> list[str]:
    matches: list[str] = []
    for prim in stage.TraverseAll():
        name = prim.GetName().lower()
        type_name = str(prim.GetTypeName())
        if name in GROUND_NAMES or (name.startswith("ground") and type_name == "Plane"):
            matches.append(str(prim.GetPath()))
    return matches


def clean_one(usd_path: Path, dry_run: bool) -> int:
    stage = Usd.Stage.Open(str(usd_path))
    if stage is None:
        print(f"  [SKIP] could not open {usd_path}")
        return 0
    paths = find_ground_prims(stage)
    if not paths:
        return 0
    print(f"  {usd_path.relative_to(REPO_ROOT)}: {len(paths)} prim(s)")
    for path in paths:
        print(f"    - {path}")
        if not dry_run:
            stage.RemovePrim(path)
    if not dry_run:
        stage.GetRootLayer().Save()
    return len(paths)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dry-run", action="store_true", help="Only list what would be removed.")
    parser.add_argument(
        "--path",
        type=Path,
        default=None,
        help="Specific .usd file to clean. Default: scan all under assets/robots.",
    )
    args = parser.parse_args()

    if args.path is not None:
        targets = [args.path.resolve()]
    else:
        if not ASSET_ROOT.exists():
            print(f"Asset root not found: {ASSET_ROOT}")
            return 1
        targets = sorted(p for p in ASSET_ROOT.rglob("*.usd"))

    print(f"Scanning {len(targets)} USD file(s){' (dry run)' if args.dry_run else ''}:")
    total = 0
    for usd_path in targets:
        total += clean_one(usd_path, args.dry_run)
    if total == 0:
        print("No embedded GroundPlane prims found — already clean.")
    else:
        verb = "Would remove" if args.dry_run else "Removed"
        print(f"{verb} {total} prim(s) across {len(targets)} file(s).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
