"""Print every PhysX joint in a USD file with its limits and drive params.

Usage:
    python scripts/tools/inspect_joint_limits.py [path/to/asset.usd]

Defaults to the closed-chain mos2026_2 asset if no path is given.
"""

from __future__ import annotations

import argparse
from pathlib import Path

# Isaac Sim ships its own USD libraries inside the extscache. We have to bring up
# a headless SimulationApp before importing `pxr` so those libraries land on
# sys.path.
from isaacsim import SimulationApp  # type: ignore

_app = SimulationApp({"headless": True})

from pxr import Usd, UsdPhysics  # noqa: E402


DEFAULT_USD = (
    Path(__file__).resolve().parents[2]
    / "source"
    / "stackforce_simready_mos2026_2_closed_usd_closed_usd_lab"
    / "stackforce_simready_mos2026_2_closed_usd_closed_usd_lab"
    / "assets"
    / "robots"
    / "mos2026_2_closed_usd"
    / "usd"
    / "mos2026_2.usd"
)


def _attr(prim, name, default=None):
    attr = prim.GetAttribute(name)
    if not attr or not attr.HasAuthoredValue():
        return default
    return attr.Get()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("usd", nargs="?", default=str(DEFAULT_USD))
    parser.add_argument("--out", default=None, help="Write the table to this file (in addition to stdout).")
    args = parser.parse_args()

    stage = Usd.Stage.Open(args.usd)
    if stage is None:
        raise SystemExit(f"failed to open USD: {args.usd}")

    rows = []
    for prim in stage.TraverseAll():
        type_name = prim.GetTypeName()
        if type_name not in {"PhysicsRevoluteJoint", "PhysicsPrismaticJoint", "PhysicsJoint", "PhysicsSphericalJoint", "PhysicsFixedJoint", "PhysicsDistanceJoint"}:
            continue
        name = prim.GetName()
        axis = _attr(prim, "physics:axis", "?")
        lo = _attr(prim, "physics:lowerLimit", None)
        hi = _attr(prim, "physics:upperLimit", None)
        # Drive params (only authored for actuated joints).
        if type_name == "PhysicsRevoluteJoint":
            drive_ns = "drive:angular"
        elif type_name == "PhysicsPrismaticJoint":
            drive_ns = "drive:linear"
        else:
            drive_ns = None
        stiffness = _attr(prim, f"{drive_ns}:physics:stiffness", None) if drive_ns else None
        damping = _attr(prim, f"{drive_ns}:physics:damping", None) if drive_ns else None
        max_force = _attr(prim, f"{drive_ns}:physics:maxForce", None) if drive_ns else None
        rows.append((name, type_name.replace("Physics", "").replace("Joint", ""), str(axis), lo, hi, stiffness, damping, max_force))

    if not rows:
        print(f"no joints found in {args.usd}")
        return

    header = ("name", "type", "axis", "lower", "upper", "stiffness", "damping", "maxForce")
    fmt = "{:<30} {:<10} {:<6} {:>10} {:>10} {:>10} {:>10} {:>10}"
    lines = [
        "",
        f"USD: {args.usd}",
        "Revolute limits are in DEGREES (USD convention). Prismatic limits are in scene units.",
        "",
        fmt.format(*header),
        "-" * 100,
    ]
    for row in rows:
        rendered = []
        for v in row:
            if v is None:
                rendered.append("-")
            elif isinstance(v, float):
                rendered.append(f"{v:.3g}")
            else:
                rendered.append(str(v))
        lines.append(fmt.format(*rendered))
    text = "\n".join(lines) + "\n"
    import sys
    sys.stdout.write(text)
    sys.stdout.flush()
    if args.out:
        Path(args.out).write_text(text)


if __name__ == "__main__":
    try:
        main()
    finally:
        _app.close()
