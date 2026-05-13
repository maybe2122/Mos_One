from pathlib import Path

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg
from isaaclab.envs import DirectRLEnvCfg, ViewerCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import SimulationCfg
from isaaclab.terrains import TerrainImporterCfg
from isaaclab.utils import configclass


ASSET_DIR = Path(__file__).resolve().parents[3] / "assets" / "robots" / "mos2026_2_closed_usd" / "usd"
USD_PATH = ASSET_DIR / "mos2026_2.usd"


@configclass
class Mos20262ClosedUsdEnvCfg(DirectRLEnvCfg):
    episode_length_s = 20.0
    decimation = 4
    # Per-step joint-target deflection from the default pose, in radians.
    # 0.5 rad ≈ 29°; combined with `action_clip=1.5` the policy can reach
    # ±0.75 rad ≈ ±43° from the nominal stance.
    action_scale = 0.5
    action_control_mode = "position"
    action_space = 12
    observation_space = 45
    state_space = 0
    viewer = ViewerCfg(
        eye=(3.0, -4.0, 2.0),
        lookat=(0.0, 0.0, 0.45),
        origin_type="world",
        resolution=(1280, 720),
    )

    sim: SimulationCfg = SimulationCfg(
        dt=1 / 200,
        render_interval=decimation,
        physics_material=sim_utils.RigidBodyMaterialCfg(
            friction_combine_mode="multiply",
            restitution_combine_mode="multiply",
            static_friction=1.0,
            dynamic_friction=1.0,
            restitution=0.0,
        ),
    )

    terrain = TerrainImporterCfg(
        prim_path="/World/ground",
        terrain_type="plane",
        collision_group=-1,
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.28, 0.30, 0.32)),
        physics_material=sim_utils.RigidBodyMaterialCfg(
            friction_combine_mode="multiply",
            restitution_combine_mode="multiply",
            static_friction=1.0,
            dynamic_friction=1.0,
            restitution=0.0,
        ),
        debug_vis=False,
    )

    # Closed-chain USD assets may embed their own PhysicsScene. Replicating those
    # scene prims can trigger PhysX clone errors, so keep cloning conservative.
    scene: InteractiveSceneCfg = InteractiveSceneCfg(num_envs=128, env_spacing=4.0, replicate_physics=False)

    robot: ArticulationCfg = ArticulationCfg(
        prim_path="/World/envs/env_.*/Robot",
        spawn=sim_utils.UsdFileCfg(
            usd_path=str(USD_PATH),
            activate_contact_sensors=False,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=False,
                retain_accelerations=False,
                linear_damping=0.0,
                angular_damping=0.0,
                max_linear_velocity=20.0,
                max_angular_velocity=30.0,
                max_depenetration_velocity=1.0,
                solver_position_iteration_count=64,
                solver_velocity_iteration_count=4,
                sleep_threshold=0.0,
                stabilization_threshold=0.0001,
            ),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=False,
                solver_position_iteration_count=64,
                solver_velocity_iteration_count=4,
                sleep_threshold=0.0,
                stabilization_threshold=0.0001,
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, 0.35),
            rot=(1, 0, 0, 0),
            joint_pos={
                "fl_hip": 0.06, "fr_hip": 0.06, "rl_hip": -0.06, "rr_hip": -0.06,
                "fl_thigh": 0.0, "fr_thigh": 0.0, "rl_thigh": 0.0, "rr_thigh": 0.0,
                # FL leg's "shank_link_a" is named `fl_shank_link` (no _a) in the USD;
                # the other three legs use the `*_shank_link_a` convention.
                "fl_shank_link": 0.0, "fr_shank_link_a": 0.0, "rl_shank_link_a": 0.0, "rr_shank_link_a": 0.0,
            },
            joint_vel={},
        ),
        actuators={
            "main_joints": ImplicitActuatorCfg(
                joint_names_expr=[
                    "fl_hip", "fr_hip", "rl_hip", "rr_hip",
                    "fl_thigh", "fr_thigh", "rl_thigh", "rr_thigh",
                    "fl_shank_link", "fr_shank_link_a", "rl_shank_link_a", "rr_shank_link_a",
                ],
                stiffness=25.0,
                damping=0.5,
                effort_limit_sim=80.0,
                velocity_limit_sim=30.0,
            ),
        },
        soft_joint_pos_limit_factor=0.95,
    )

    actuated_joint_names = [
        "fl_hip", "fr_hip", "rl_hip", "rr_hip",
        "fl_thigh", "fr_thigh", "rl_thigh", "rr_thigh",
        "fl_shank_link", "fr_shank_link_a", "rl_shank_link_a", "rr_shank_link_a",
    ]
    projected_loop_joint_names = []
    auto_collision_from_visuals = False
    strip_embedded_ground_prims = False
    base_height_target = 0.32
    action_clip = 1.5
    visual_disable_resets = False
    commanded_lin_vel_xy = (1.0, 0.0)
    commanded_ang_vel_z = 0.0
    show_velocity_arrows = True
    fall_height_threshold = 0.15
    # cos(angle) threshold for "tipped over" detection on projected gravity.
    # 0.7 ≈ 45° tilt from upright; raise toward 1.0 to terminate sooner.
    fall_cos_threshold = 0.7
    reward_scales = {
        "alive": 0.5,
        "upright": 0.25,
        "base_height": -1.0,
        "lin_vel_z": -1.0,
        "ang_vel_xy": -0.05,
        "joint_vel": -0.0005,
        "action_rate": -0.01,
        "track_lin_vel_xy": 2.0,
        "track_ang_vel_z": 1.0,
        "custom_reward": 0.0,
    }
