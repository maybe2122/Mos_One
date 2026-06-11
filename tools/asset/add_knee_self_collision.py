"""给 mos2026_2.xml 的大腿↔小腿加自碰撞（机械限位）。

问题
----
导出的 MJCF 自碰撞全关（contype=2/conaffinity=1，与 Isaac 训练
enabled_self_collisions=False 一致），蹲下/趴下时大腿杆视觉上直接穿过小腿杆。
不能简单给 thigh/shank 的整体凸包加 <pair>：两者在膝关节轴承处本来就嵌套，
整体凸包在任意姿态下都互相重叠 ~20 mm，会产生永久接触力。

方案
----
1. coacd 把每条腿的 thigh / shank 视觉 mesh 凸分解成 ≤8 个凸块，写到
   meshes/collision/{leg}_{part}_cvx{i}.obj；
2. 凸块以 contype=0 conaffinity=0 group=4 透明 geom 挂进对应 body —— 不参与
   任何隐式碰撞，只通过显式 <pair> 生效（viewer 里开 group 4 可看到碰撞件）；
3. 零位姿态（腿伸直，闭链一致）下逐对算 mj_geomDistance：间隙 > CLEARANCE 的
   thigh块×shank块 组合才生成 <pair>。轴承处常年嵌套的块被自动排除，杆身的块
   在深折叠时相撞 —— 等效真实机械限位。

效果：正常站立/行走姿态无任何新增接触（物理与训练一致）；只有折叠到杆身相碰
时接触才激活，大腿不再穿过小腿，趴下时腿架在限位上 —— 与真机一致。

运行（env_isaaclab，装了 mujoco/trimesh/coacd）：
  python tools/asset/add_knee_self_collision.py
幂等：检测到 XML 已有 knee-collision 标记就拒绝重复运行。
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
ASSETS = REPO_ROOT / "deploy/mujoco/assets"
XML = ASSETS / "mos2026_2.xml"
CVX_DIR = ASSETS / "meshes/collision"

LEGS = ["fl", "fr", "rl", "rr"]
PARTS = ["thigh", "shank"]
MAX_HULLS = 8           # coacd 每个 mesh 的凸块上限
COACD_THRESHOLD = 0.05  # coacd 凹度阈值（越小分得越细）
CLEARANCE = 0.003       # 零位姿态下间隙小于此值的块对不生成 pair（轴承嵌套件）
MARKER = "knee-collision (add_knee_self_collision.py)"
HIP_POS = {"fl": 0.15, "fr": 0.15, "rl": -0.15, "rr": -0.15}


def decompose() -> dict[str, int]:
    """coacd 分解 8 个 mesh，写 OBJ，返回 {f"{leg}_{part}": n_pieces}。"""
    import coacd
    import trimesh

    coacd.set_log_level("error")
    CVX_DIR.mkdir(parents=True, exist_ok=True)
    counts: dict[str, int] = {}
    for leg in LEGS:
        for part in PARTS:
            name = f"{leg}_{part}"
            mesh = trimesh.load(ASSETS / f"meshes/{name}.obj", force="mesh")
            parts = coacd.run_coacd(
                coacd.Mesh(np.asarray(mesh.vertices), np.asarray(mesh.faces)),
                threshold=COACD_THRESHOLD,
                max_convex_hull=MAX_HULLS,
                preprocess_resolution=50,
            )
            for i, (verts, faces) in enumerate(parts):
                piece = trimesh.Trimesh(verts, faces).convex_hull  # 去冗余顶点
                lines = [f"v {x:.6f} {y:.6f} {z:.6f}" for x, y, z in piece.vertices]
                lines += [f"f {a+1} {b+1} {c+1}" for a, b, c in piece.faces]
                (CVX_DIR / f"{name}_cvx{i}.obj").write_text("\n".join(lines) + "\n")
            counts[name] = len(parts)
            print(f"[coacd] {name}: {len(parts)} pieces")
    return counts


def piece_assets(counts) -> list[str]:
    out = []
    for name, n in counts.items():
        for i in range(n):
            out.append(f'    <mesh name="{name}_cvx{i}_mesh" file="collision/{name}_cvx{i}.obj"/>')
    return out


def piece_geom_lines(name: str, n: int, indent: str) -> list[str]:
    # contype/conaffinity 0：只经显式 <pair> 碰撞。group 4 默认不显示。
    return [
        f'{indent}<geom name="{name}_cvx{i}" type="mesh" mesh="{name}_cvx{i}_mesh" '
        f'contype="0" conaffinity="0" group="4" rgba="0.9 0.2 0.2 0.3"/>'
        for i in range(n)
    ]


def insert_into_xml(counts) -> str:
    """资产 + 凸块 geom 插入 XML 文本，返回新文本（pair 之后再补）。"""
    text = XML.read_text()
    if MARKER in text:
        print(f"[error] {XML} 已包含 {MARKER}，不重复插入", file=sys.stderr)
        sys.exit(1)

    text = text.replace("  </asset>",
                        f"    <!-- {MARKER}: convex pieces for thigh/shank mechanical-stop pairs -->\n"
                        + "\n".join(piece_assets(counts)) + "\n  </asset>")

    # 凸块 geom 插到对应 body 的 *_visual geom 行后面
    lines = text.split("\n")
    out = []
    for line in lines:
        out.append(line)
        for name, n in counts.items():
            if f'name="{name}_visual"' in line:
                indent = line[: len(line) - len(line.lstrip())]
                out.extend(piece_geom_lines(name, n, indent))
    return "\n".join(out)


def filter_pairs(xml_text: str, counts) -> list[tuple[str, str]]:
    """零位姿态（腿伸直，闭链一致）下，返回间隙 > CLEARANCE 的同腿 thigh块×shank块。"""
    import mujoco

    tmp = ASSETS / "_knee_tmp.xml"
    tmp.write_text(xml_text)
    try:
        m = mujoco.MjModel.from_xml_path(str(tmp))
    finally:
        tmp.unlink()
    d = mujoco.MjData(m)
    d.qpos[2] = 0.35
    for leg, v in HIP_POS.items():
        j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, f"{leg}_hip")
        d.qpos[m.jnt_qposadr[j]] = v
    mujoco.mj_forward(m, d)

    gid = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, n)
    fromto = np.zeros(6)
    pairs, dropped = [], 0
    for leg in LEGS:
        for i in range(counts[f"{leg}_thigh"]):
            for k in range(counts[f"{leg}_shank"]):
                a, b = f"{leg}_thigh_cvx{i}", f"{leg}_shank_cvx{k}"
                dist = mujoco.mj_geomDistance(m, d, gid(a), gid(b), 0.5, fromto)
                if dist > CLEARANCE:
                    pairs.append((a, b))
                else:
                    dropped += 1
    print(f"[pairs] kept {len(pairs)}, dropped {dropped} (bearing-area overlap at zero pose)")
    return pairs


def main() -> int:
    counts = decompose()
    xml_text = insert_into_xml(counts)
    pairs = filter_pairs(xml_text, counts)
    pair_lines = [f'    <pair geom1="{a}" geom2="{b}"/>' for a, b in pairs]
    contact_block = (f"  <!-- {MARKER}: thigh-shank mechanical stop -->\n"
                     "  <contact>\n" + "\n".join(pair_lines) + "\n  </contact>\n")
    xml_text = xml_text.replace("  <equality>", contact_block + "  <equality>")
    XML.write_text(xml_text)
    print(f"[done] {XML} updated: "
          f"{sum(counts.values())} collision pieces, {len(pairs)} contact pairs")
    return 0


if __name__ == "__main__":
    sys.exit(main())
