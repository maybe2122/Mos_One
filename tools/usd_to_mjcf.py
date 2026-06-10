"""Convert the mos2026_2 closed-chain USD asset to a MuJoCo MJCF.

Output layout:
    deploy/mujoco/assets/
        mos2026_2.xml          <- MJCF
        scene.xml              <- thin wrapper that includes mos2026_2.xml + floor/light

Strategy:
  - USD topology: 29 rigid bodies, 28 articulation joints (spanning tree),
    4 closure joints. Articulation root is /mos2026_2/base.
  - Bodies emitted under <worldbody> with a freejoint on `base`.
  - Each non-root body's pos/quat is computed from USD world transforms as
    inverse(parent_world) * child_world. Joint pos is in the child's local
    frame (USD already authors localPos1=(0,0,0) for every articulation
    joint, so this is just zero); joint axis is `localRot1 * unit_axis_USD`.
  - Collision geom per body is a single box derived from the AABB of the
    body's visual mesh (kept for fast, stable contacts). For visual fidelity
    the body's high-poly STL visual mesh is also exported (grid-decimated to
    ~1/7 the triangles) to deploy/mujoco/assets/meshes/<body>.obj and emitted
    as a non-colliding mesh geom; the collision box is then made transparent.
    Physics is therefore identical to the box-only converter — only the
    appearance changes, so existing trained policies stay valid.
  - 4 closure joints become <equality connect> constraints, anchored in
    body0's frame (USD's body0 == MJCF's body1 for the connect element).
  - 12 <position> actuators on the actuated joints with kp=stiffness=25,
    kv=damping=0.5, forcerange ±80 — matching Isaac Lab's ImplicitActuatorCfg.

Run:
    /home/sz/code/rl/env_isaaclab/bin/python3 tools/usd_to_mjcf.py
"""
from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path
from xml.dom import minidom

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

REPO_ROOT = Path(__file__).resolve().parents[1]
USD_DIR = (
    REPO_ROOT
    / "source/mos_one"
    / "mos_one"
    / "assets/robots/mos2026_2_closed_usd/usd"
)
MAIN_USD = USD_DIR / "mos2026_2.usd"
BASE_USD = USD_DIR / "configuration/mos2026_2_base.usd"

OUT_DIR = REPO_ROOT / "deploy/mujoco/assets"
OUT_DIR.mkdir(parents=True, exist_ok=True)
MESH_DIR = OUT_DIR / "meshes"

# Grid vertex-clustering cell size (metres) for decimating the high-poly STL
# visual meshes. 4 mm keeps the robot recognisable while cutting triangle count
# ~10x so the viewer loads fast. Purely visual — collision/physics are unchanged.
MESH_CELL_SIZE = 0.004

# Joint -> actuator metadata. Matches Isaac Lab's ImplicitActuatorCfg in
# mos2026_2_closed_usd_env_cfg.py. Note the FL/FR naming asymmetry already
# baked into the USD: FL uses "shank_link" / "shank_motor_gear", the other
# three legs use "*_shank_link_a" / "*_shank_motor".
ACTUATED_JOINTS = [
    "fl_hip", "fr_hip", "rl_hip", "rr_hip",
    "fl_thigh", "fr_thigh", "rl_thigh", "rr_thigh",
    "fl_shank_link", "fr_shank_link_a", "rl_shank_link_a", "rr_shank_link_a",
]
DEFAULT_JOINT_POS = {
    "fl_hip": 0.06, "fr_hip": 0.06, "rl_hip": -0.06, "rr_hip": -0.06,
    "fl_thigh": 0.0, "fr_thigh": 0.0, "rl_thigh": 0.0, "rr_thigh": 0.0,
    "fl_shank_link": 0.0, "fr_shank_link_a": 0.0,
    "rl_shank_link_a": 0.0, "rr_shank_link_a": 0.0,
}
ACTUATOR_KP = 25.0
ACTUATOR_KV = 0.5
ACTUATOR_FORCE_LIMIT = 80.0
INIT_HEIGHT = 0.35  # matches ArticulationCfg.InitialStateCfg.pos[2]


def usd_quat_to_wxyz(q: Gf.Quatf | Gf.Quatd) -> np.ndarray:
    return np.array([q.GetReal(), *q.GetImaginary()], dtype=float)


def quat_rotate(q_wxyz: np.ndarray, v: np.ndarray) -> np.ndarray:
    w, x, y, z = q_wxyz
    rot = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    return rot @ v


def matrix_to_pos_quat(m: Gf.Matrix4d) -> tuple[np.ndarray, np.ndarray]:
    pos = np.array(m.ExtractTranslation(), dtype=float)
    q = m.ExtractRotationQuat()
    return pos, usd_quat_to_wxyz(q)


def mul_quat(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ])


def conj_quat(q: np.ndarray) -> np.ndarray:
    return np.array([q[0], -q[1], -q[2], -q[3]])


def relative_transform(parent_xf: Gf.Matrix4d, child_xf: Gf.Matrix4d) -> tuple[np.ndarray, np.ndarray]:
    parent_pos, parent_quat = matrix_to_pos_quat(parent_xf)
    child_pos, child_quat = matrix_to_pos_quat(child_xf)
    delta_world = child_pos - parent_pos
    inv_parent_q = conj_quat(parent_quat)
    rel_pos = quat_rotate(inv_parent_q, delta_world)
    rel_quat = mul_quat(inv_parent_q, child_quat)
    return rel_pos, rel_quat


def axis_in_child_frame(axis_letter: str, local_rot1: Gf.Quatf) -> np.ndarray:
    unit = {"X": np.array([1.0, 0, 0]), "Y": np.array([0, 1.0, 0]), "Z": np.array([0, 0, 1.0])}[axis_letter]
    q = usd_quat_to_wxyz(local_rot1)
    return quat_rotate(q, unit)


def gather_bodies(stage: Usd.Stage) -> dict:
    bodies = {}
    for prim in stage.TraverseAll():
        schemas = {str(s) for s in prim.GetAppliedSchemas()}
        if "PhysicsRigidBodyAPI" not in schemas:
            continue
        mass = prim.GetAttribute("physics:mass").Get()
        com = prim.GetAttribute("physics:centerOfMass").Get()
        diag = prim.GetAttribute("physics:diagonalInertia").Get()
        pax = prim.GetAttribute("physics:principalAxes").Get()
        xf = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0)
        bodies[str(prim.GetPath())] = {
            "name": prim.GetName(),
            "world_xf": xf,
            "mass": float(mass) if mass is not None else 0.0,
            "com": np.array(com, dtype=float) if com is not None else np.zeros(3),
            "diag_inertia": np.array(diag, dtype=float) if diag is not None else np.full(3, 1e-4),
            "principal_axes": usd_quat_to_wxyz(pax) if pax is not None else np.array([1.0, 0, 0, 0]),
        }
    return bodies


def gather_joints(stage: Usd.Stage) -> tuple[list, list]:
    articulation = []
    closures = []
    for prim in stage.TraverseAll():
        tn = prim.GetTypeName()
        if tn != "PhysicsRevoluteJoint":
            continue
        j = UsdPhysics.Joint(prim)
        b0 = j.GetBody0Rel().GetTargets()
        b1 = j.GetBody1Rel().GetTargets()
        ex = prim.GetAttribute("physics:excludeFromArticulation")
        is_closure = bool(ex.Get()) if ex else False
        lo = prim.GetAttribute("physics:lowerLimit")
        hi = prim.GetAttribute("physics:upperLimit")
        info = {
            "name": prim.GetName(),
            "body0": str(b0[0]) if b0 else None,
            "body1": str(b1[0]) if b1 else None,
            "axis": prim.GetAttribute("physics:axis").Get(),
            "local_pos0": np.array(prim.GetAttribute("physics:localPos0").Get(), dtype=float),
            "local_pos1": np.array(prim.GetAttribute("physics:localPos1").Get(), dtype=float),
            "local_rot0": prim.GetAttribute("physics:localRot0").Get(),
            "local_rot1": prim.GetAttribute("physics:localRot1").Get(),
            "lo": np.deg2rad(lo.Get()) if lo and lo.Get() is not None and np.isfinite(lo.Get()) else None,
            "hi": np.deg2rad(hi.Get()) if hi and hi.Get() is not None and np.isfinite(hi.Get()) else None,
        }
        (closures if is_closure else articulation).append(info)
    return articulation, closures


def aabb_for_body(base_stage: Usd.Stage, body_name: str) -> tuple[np.ndarray, np.ndarray] | None:
    visuals_root = base_stage.GetPrimAtPath(f"/visuals/{body_name}")
    if not visuals_root.IsValid():
        return None
    mins = []
    maxs = []
    for desc in Usd.PrimRange(visuals_root):
        if desc.GetTypeName() != "Mesh":
            continue
        pts = desc.GetAttribute("points").Get()
        if not pts:
            continue
        arr = np.asarray(pts, dtype=float)
        mins.append(arr.min(0))
        maxs.append(arr.max(0))
    if not mins:
        return None
    return np.min(mins, axis=0), np.max(maxs, axis=0)


def load_visual_mesh(base_stage: Usd.Stage, body_name: str) -> tuple[np.ndarray, np.ndarray] | None:
    """Combine all Mesh prims under /visuals/{body_name} into (points, tris).

    Points are taken raw (no transform) — they are already authored in the
    body-local frame, the same convention `aabb_for_body` relies on, so the
    emitted mesh geom needs no pos/quat offset and lines up with the box.
    """
    root = base_stage.GetPrimAtPath(f"/visuals/{body_name}")
    if not root.IsValid():
        return None
    pts_list: list[np.ndarray] = []
    tri_list: list[np.ndarray] = []
    offset = 0
    for desc in Usd.PrimRange(root):
        if desc.GetTypeName() != "Mesh":
            continue
        mesh = UsdGeom.Mesh(desc)
        pts = mesh.GetPointsAttr().Get()
        counts = mesh.GetFaceVertexCountsAttr().Get()
        indices = mesh.GetFaceVertexIndicesAttr().Get()
        if not pts or not counts or not indices:
            continue
        pts = np.asarray(pts, dtype=np.float64)
        counts = np.asarray(counts)
        indices = np.asarray(indices)
        tris = []
        i = 0
        for c in counts:
            for k in range(1, c - 1):  # fan-triangulate arbitrary polygons
                tris.append((indices[i], indices[i + k], indices[i + k + 1]))
            i += c
        if not tris:
            continue
        tri_list.append(np.asarray(tris, dtype=np.int64) + offset)
        pts_list.append(pts)
        offset += len(pts)
    if not pts_list:
        return None
    return np.vstack(pts_list), np.vstack(tri_list)


def decimate_mesh(points: np.ndarray, tris: np.ndarray, cell: float) -> tuple[np.ndarray, np.ndarray]:
    """Grid vertex clustering: snap verts to a `cell`-sized grid, merge, drop
    degenerate triangles. Dependency-free and robust on dirty STL output."""
    keys = np.floor(points / cell).astype(np.int64)
    _, inverse = np.unique(keys, axis=0, return_inverse=True)
    inverse = inverse.ravel()
    n_new = int(inverse.max()) + 1
    new_pts = np.zeros((n_new, 3))
    counts = np.zeros(n_new)
    np.add.at(new_pts, inverse, points)
    np.add.at(counts, inverse, 1)
    new_pts /= counts[:, None]
    new_tris = inverse[tris]
    keep = (
        (new_tris[:, 0] != new_tris[:, 1])
        & (new_tris[:, 1] != new_tris[:, 2])
        & (new_tris[:, 0] != new_tris[:, 2])
    )
    return new_pts, new_tris[keep]


def write_obj(path: Path, points: np.ndarray, tris: np.ndarray) -> None:
    lines = [f"v {x:.6f} {y:.6f} {z:.6f}" for x, y, z in points]
    lines += [f"f {a + 1} {b + 1} {c + 1}" for a, b, c in tris]
    path.write_text("\n".join(lines) + "\n")


def export_visual_meshes(base_stage: Usd.Stage, body_names: list[str]) -> set[str]:
    """Decimate + write one OBJ per body that has a visual mesh.

    Returns the set of body names for which an OBJ was written (those get a
    mesh geom in the MJCF; others fall back to the AABB box only)."""
    MESH_DIR.mkdir(parents=True, exist_ok=True)
    written: set[str] = set()
    total_in = total_out = 0
    for name in body_names:
        loaded = load_visual_mesh(base_stage, name)
        if loaded is None:
            continue
        pts, tris = loaded
        dpts, dtris = decimate_mesh(pts, tris, MESH_CELL_SIZE)
        if len(dtris) == 0:
            continue
        write_obj(MESH_DIR / f"{name}.obj", dpts, dtris)
        written.add(name)
        total_in += len(tris)
        total_out += len(dtris)
    print(f"[info] exported {len(written)} meshes, {total_in} -> {total_out} tris "
          f"(cell {MESH_CELL_SIZE} m)")
    return written


def add_geoms(elem: ET.Element, body_name: str, aabb, has_mesh: bool) -> None:
    """Emit a collision box (kept identical to the original converter so physics
    is unchanged) plus, when available, a visual mesh geom. When a mesh is
    present the collision box is made transparent so only the mesh is seen."""
    if aabb is not None:
        bmin, bmax = aabb
        center = (bmin + bmax) / 2
        half = np.maximum((bmax - bmin) / 2, 1e-3)
        box_attrs = {
            "name": f"{body_name}_geom",
            "type": "box",
            "pos": fmt(center),
            "size": fmt(half),
            "class": "body_visual",
        }
        if has_mesh:
            box_attrs["rgba"] = "1 1 1 0"  # invisible; still collides
        ET.SubElement(elem, "geom", box_attrs)
    else:
        ET.SubElement(elem, "geom", {
            "name": f"{body_name}_geom",
            "type": "sphere",
            "size": "0.01",
            "class": "body_visual",
            **({"rgba": "1 1 1 0"} if has_mesh else {}),
        })
    if has_mesh:
        ET.SubElement(elem, "geom", {
            "name": f"{body_name}_visual",
            "type": "mesh",
            "mesh": f"{body_name}_mesh",
            "contype": "0",
            "conaffinity": "0",
            "group": "1",
            "rgba": "0.75 0.75 0.78 1",
        })


def build_tree(bodies: dict, articulation: list) -> tuple[str, dict]:
    parent_of = {}
    children_of = {}
    for j in articulation:
        parent_of[j["body1"]] = (j["body0"], j)
        children_of.setdefault(j["body0"], []).append((j["body1"], j))
    roots = [b for b in bodies if b not in parent_of]
    assert len(roots) == 1, f"expected single root, got {roots}"
    return roots[0], children_of


def fmt(arr: np.ndarray, prec: int = 6) -> str:
    return " ".join(f"{x:.{prec}f}" for x in arr)


def emit_body(parent_elem: ET.Element, body_path: str, parent_world_xf: Gf.Matrix4d | None,
              bodies: dict, children_of: dict, base_stage: Usd.Stage,
              joint_to_actuator: dict, is_root: bool = False) -> None:
    body_info = bodies[body_path]
    body_name = body_info["name"]
    world_xf = body_info["world_xf"]

    body_elem = ET.SubElement(parent_elem, "body", {"name": body_name})

    if is_root:
        body_elem.set("pos", fmt(np.array([0.0, 0.0, INIT_HEIGHT])))
        body_elem.set("quat", "1 0 0 0")
        ET.SubElement(body_elem, "freejoint", {"name": "root"})
    else:
        rel_pos, rel_quat = relative_transform(parent_world_xf, world_xf)
        body_elem.set("pos", fmt(rel_pos))
        body_elem.set("quat", fmt(rel_quat))
        # this body is reached via the joint stored in parent_of -> use it
        # we have children_of, so the joint info comes from the parent walker;
        # but it's easier to pull it back through children_of here:
        # actually we don't have that here directly — emit_body's caller passes joint
        pass

    # Inertial
    inertial = ET.SubElement(body_elem, "inertial", {
        "mass": f"{body_info['mass']:.6f}",
        "pos": fmt(body_info["com"]),
        "diaginertia": fmt(np.maximum(body_info["diag_inertia"], 1e-7)),
        "quat": fmt(body_info["principal_axes"]),
    })

    # Geom from AABB
    aabb = aabb_for_body(base_stage, body_name)
    if aabb is not None:
        bmin, bmax = aabb
        center = (bmin + bmax) / 2
        half = np.maximum((bmax - bmin) / 2, 1e-3)
        ET.SubElement(body_elem, "geom", {
            "name": f"{body_name}_geom",
            "type": "box",
            "pos": fmt(center),
            "size": fmt(half),
            "class": "body_visual",
        })
    else:
        # fallback small placeholder so MJCF compiles
        ET.SubElement(body_elem, "geom", {
            "name": f"{body_name}_geom",
            "type": "sphere",
            "size": "0.01",
            "class": "body_visual",
        })

    for child_path, joint in children_of.get(body_path, []):
        sub_body = bodies[child_path]
        sub_name = sub_body["name"]
        # We need to insert the body element first to get a handle, then add
        # the joint as its child after pos/quat are set.
        # Recurse — emit child body, but with the joint info passed in:
        child_elem = ET.SubElement(body_elem, "body", {"name": sub_name})
        rel_pos, rel_quat = relative_transform(world_xf, sub_body["world_xf"])
        child_elem.set("pos", fmt(rel_pos))
        child_elem.set("quat", fmt(rel_quat))

        axis = axis_in_child_frame(joint["axis"], joint["local_rot1"])
        joint_attrs = {
            "name": joint["name"],
            "type": "hinge",
            "pos": fmt(joint["local_pos1"]),
            "axis": fmt(axis),
            "damping": "0.05",
        }
        if joint["lo"] is not None and joint["hi"] is not None:
            joint_attrs["range"] = f"{joint['lo']:.6f} {joint['hi']:.6f}"
            joint_attrs["limited"] = "true"
        else:
            joint_attrs["limited"] = "false"
        ET.SubElement(child_elem, "joint", joint_attrs)

        # inertial
        ci = sub_body
        ET.SubElement(child_elem, "inertial", {
            "mass": f"{ci['mass']:.6f}",
            "pos": fmt(ci["com"]),
            "diaginertia": fmt(np.maximum(ci["diag_inertia"], 1e-7)),
            "quat": fmt(ci["principal_axes"]),
        })

        # geom from AABB
        sub_aabb = aabb_for_body(base_stage, sub_name)
        if sub_aabb is not None:
            bmin, bmax = sub_aabb
            center = (bmin + bmax) / 2
            half = np.maximum((bmax - bmin) / 2, 1e-3)
            ET.SubElement(child_elem, "geom", {
                "name": f"{sub_name}_geom",
                "type": "box",
                "pos": fmt(center),
                "size": fmt(half),
                "class": "body_visual",
            })
        else:
            ET.SubElement(child_elem, "geom", {
                "name": f"{sub_name}_geom",
                "type": "sphere",
                "size": "0.01",
                "class": "body_visual",
            })

        # recurse into grandchildren
        _emit_subtree(child_elem, child_path, sub_body["world_xf"], bodies, children_of, base_stage)


def _emit_subtree(parent_elem: ET.Element, body_path: str, body_world_xf: Gf.Matrix4d,
                  bodies: dict, children_of: dict, base_stage: Usd.Stage,
                  mesh_bodies: set[str]) -> None:
    for child_path, joint in children_of.get(body_path, []):
        sub_body = bodies[child_path]
        sub_name = sub_body["name"]
        child_elem = ET.SubElement(parent_elem, "body", {"name": sub_name})
        rel_pos, rel_quat = relative_transform(body_world_xf, sub_body["world_xf"])
        child_elem.set("pos", fmt(rel_pos))
        child_elem.set("quat", fmt(rel_quat))

        axis = axis_in_child_frame(joint["axis"], joint["local_rot1"])
        joint_attrs = {
            "name": joint["name"],
            "type": "hinge",
            "pos": fmt(joint["local_pos1"]),
            "axis": fmt(axis),
            "damping": "0.05",
        }
        if joint["lo"] is not None and joint["hi"] is not None:
            joint_attrs["range"] = f"{joint['lo']:.6f} {joint['hi']:.6f}"
            joint_attrs["limited"] = "true"
        else:
            joint_attrs["limited"] = "false"
        ET.SubElement(child_elem, "joint", joint_attrs)

        ET.SubElement(child_elem, "inertial", {
            "mass": f"{sub_body['mass']:.6f}",
            "pos": fmt(sub_body["com"]),
            "diaginertia": fmt(np.maximum(sub_body["diag_inertia"], 1e-7)),
            "quat": fmt(sub_body["principal_axes"]),
        })

        add_geoms(child_elem, sub_name, aabb_for_body(base_stage, sub_name),
                  sub_name in mesh_bodies)

        _emit_subtree(child_elem, child_path, sub_body["world_xf"], bodies,
                      children_of, base_stage, mesh_bodies)


def prettify(elem: ET.Element) -> str:
    rough = ET.tostring(elem, "utf-8")
    return minidom.parseString(rough).toprettyxml(indent="  ")


def main() -> int:
    # Stage with joints/bodies/masses
    stage = Usd.Stage.Open(str(MAIN_USD), load=Usd.Stage.LoadAll)
    base_stage = Usd.Stage.Open(str(BASE_USD), load=Usd.Stage.LoadAll)

    bodies = gather_bodies(stage)
    articulation, closures = gather_joints(stage)
    root_path, children_of = build_tree(bodies, articulation)
    print(f"[info] {len(bodies)} bodies, {len(articulation)} articulation joints, {len(closures)} closure joints")
    print(f"[info] root body: {root_path}")

    body_names = [bodies[p]["name"] for p in bodies]
    mesh_bodies = export_visual_meshes(base_stage, body_names)

    mujoco = ET.Element("mujoco", {"model": "mos2026_2"})
    ET.SubElement(mujoco, "compiler", {"angle": "radian", "autolimits": "true",
                                        "balanceinertia": "true", "meshdir": "meshes"})
    ET.SubElement(mujoco, "option", {"timestep": "0.005", "gravity": "0 0 -9.81", "iterations": "50", "solver": "Newton"})

    default = ET.SubElement(mujoco, "default")
    body_visual_default = ET.SubElement(default, "default", {"class": "body_visual"})
    ET.SubElement(body_visual_default, "geom", {"rgba": "0.55 0.55 0.6 1", "contype": "1", "conaffinity": "1", "friction": "1 0.1 0.01"})

    asset = ET.SubElement(mujoco, "asset")
    ET.SubElement(asset, "texture", {"name": "grid", "type": "2d", "builtin": "checker", "width": "256", "height": "256",
                                       "rgb1": "0.2 0.3 0.4", "rgb2": "0.3 0.4 0.5"})
    ET.SubElement(asset, "material", {"name": "grid", "texture": "grid", "texrepeat": "10 10", "reflectance": "0.2"})
    for name in sorted(mesh_bodies):
        ET.SubElement(asset, "mesh", {"name": f"{name}_mesh", "file": f"{name}.obj"})

    worldbody = ET.SubElement(mujoco, "worldbody")
    ET.SubElement(worldbody, "light", {"name": "spot", "pos": "0 0 3", "dir": "0 0 -1", "directional": "true"})
    ET.SubElement(worldbody, "geom", {"name": "floor", "type": "plane", "size": "20 20 0.1", "material": "grid", "friction": "1 0.1 0.01"})

    root_info = bodies[root_path]
    # Emit root body with freejoint
    root_elem = ET.SubElement(worldbody, "body", {
        "name": root_info["name"],
        "pos": fmt(np.array([0.0, 0.0, INIT_HEIGHT])),
        "quat": "1 0 0 0",
    })
    ET.SubElement(root_elem, "freejoint", {"name": "root"})
    ET.SubElement(root_elem, "inertial", {
        "mass": f"{root_info['mass']:.6f}",
        "pos": fmt(root_info["com"]),
        "diaginertia": fmt(np.maximum(root_info["diag_inertia"], 1e-7)),
        "quat": fmt(root_info["principal_axes"]),
    })
    add_geoms(root_elem, root_info["name"], aabb_for_body(base_stage, root_info["name"]),
              root_info["name"] in mesh_bodies)
    _emit_subtree(root_elem, root_path, root_info["world_xf"], bodies, children_of,
                  base_stage, mesh_bodies)

    # Equality constraints for closure joints
    equality = ET.SubElement(mujoco, "equality")
    for c in closures:
        body0_name = bodies[c["body0"]]["name"]
        body1_name = bodies[c["body1"]]["name"]
        # MuJoCo connect: anchor is in body1 frame (the FIRST element body1
        # attribute). Use USD's localPos0 expressed in USD body0's frame,
        # which corresponds to MJCF body1's frame here.
        anchor = c["local_pos0"]
        ET.SubElement(equality, "connect", {
            "name": c["name"],
            "body1": body0_name,
            "body2": body1_name,
            "anchor": fmt(anchor),
            "solref": "0.005 1",
        })

    # Actuators on the 12 driven joints
    actuator_root = ET.SubElement(mujoco, "actuator")
    for jname in ACTUATED_JOINTS:
        ET.SubElement(actuator_root, "position", {
            "name": f"act_{jname}",
            "joint": jname,
            "kp": str(ACTUATOR_KP),
            "kv": str(ACTUATOR_KV),
            "forcerange": f"{-ACTUATOR_FORCE_LIMIT} {ACTUATOR_FORCE_LIMIT}",
            "ctrlrange": "-3.14159 3.14159",
        })

    # Write MJCF
    out_path = OUT_DIR / "mos2026_2.xml"
    xml_str = prettify(mujoco)
    out_path.write_text(xml_str)
    print(f"[ok] wrote {out_path}  ({len(xml_str)} bytes)")

    # A thin wrapper scene with floor + the robot — convenient for play scripts
    scene_path = OUT_DIR / "scene.xml"
    scene_str = """<mujoco model="mos2026_2_scene">
  <include file="mos2026_2.xml"/>
</mujoco>
"""
    scene_path.write_text(scene_str)
    print(f"[ok] wrote {scene_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
