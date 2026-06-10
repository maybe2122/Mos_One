import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Inspect the embedded closed-chain USD asset.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd

from mos_one.tasks.direct.mos2026_2_closed_usd.mos2026_2_closed_usd_env_cfg import USD_PATH


def main():
    usd_path = str(USD_PATH)
    stage = Usd.Stage.Open(usd_path)
    if stage is None:
        raise RuntimeError(f"Failed to open USD: {usd_path}")
    roots = []
    joints = []
    driven = []
    bodies = []
    for prim in stage.TraverseAll():
        schemas = [str(item) for item in prim.GetAppliedSchemas()]
        attrs = [attr.GetName() for attr in prim.GetAttributes()]
        if "PhysicsArticulationRootAPI" in schemas or "PhysxArticulationAPI" in schemas:
            roots.append(str(prim.GetPath()))
        if "Joint" in prim.GetTypeName():
            joints.append(str(prim.GetPath()))
        if any("drive:" in attr.lower() for attr in attrs):
            driven.append(str(prim.GetPath()).split("/")[-1])
        if "PhysicsRigidBodyAPI" in schemas:
            bodies.append(str(prim.GetPath()))
    print(f"USD_PATH={usd_path}", flush=True)
    print(f"DEFAULT_PRIM={stage.GetDefaultPrim().GetPath() if stage.GetDefaultPrim() else None}", flush=True)
    print(f"ARTICULATION_ROOTS={roots}", flush=True)
    print(f"JOINT_COUNT={len(joints)}", flush=True)
    print(f"DRIVEN_JOINT_COUNT={len(driven)}", flush=True)
    print(f"DRIVEN_JOINTS={driven[:80]}", flush=True)
    print(f"BODY_COUNT={len(bodies)}", flush=True)


if __name__ == "__main__":
    main()
    simulation_app.close()
