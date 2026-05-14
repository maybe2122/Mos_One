from pathlib import Path

import isaaclab.sim as sim_utils
import isaaclab.terrains as terrain_gen
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg
from isaaclab.envs import DirectRLEnvCfg, ViewerCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import PhysxCfg, SimulationCfg
from isaaclab.terrains import TerrainGeneratorCfg, TerrainImporterCfg
from isaaclab.utils import configclass


ROUGH_TERRAIN_CFG = TerrainGeneratorCfg(
    # Each sub-terrain is 8x8 m. With 3x3 cells we cover 24x24 m, plenty of room
    # for 128 envs at env_spacing=4 to fit on the heightfield.
    size=(16.0, 16.0),
    border_width=10.0,
    num_rows=6,
    num_cols=6,
    horizontal_scale=0.1,
    vertical_scale=0.005,
    slope_threshold=0.75,
    use_cache=False,
    sub_terrains={
        # Gentle bumpy ground: 1-4 cm random height variation. Bump amplitude up
        # for harder terrains; this is a good first-pass for a fresh policy.
        "random_rough": terrain_gen.HfRandomUniformTerrainCfg(
            proportion=1.0,
            noise_range=(0.01, 0.04),
            noise_step=0.005,
            border_width=0.25,
        ),
    },
)


# Curriculum terrain: rows = difficulty levels (level 0 is easiest).
# `curriculum=True` makes the sub-terrains generate from difficulty 0..1
# along the rows, so every env starts on the flattest variant. The env's
# `_reset_idx` promotes/demotes envs between rows based on how far the
# policy walks per episode.
CURRICULUM_TERRAIN_CFG = TerrainGeneratorCfg(
    size=(8.0, 8.0),
    border_width=10.0,
    num_rows=6,
    num_cols=5,
    horizontal_scale=0.1,
    vertical_scale=0.005,
    slope_threshold=0.75,
    use_cache=False,
    curriculum=True,
    difficulty_range=(0.0, 1.0),
    sub_terrains={
        # Pure flat — independent of difficulty.
        "flat": terrain_gen.MeshPlaneTerrainCfg(proportion=0.2),
        # Bumpy ground: noise scales 0 → 8 cm with difficulty.
        "rough": terrain_gen.HfRandomUniformTerrainCfg(
            proportion=0.2,
            noise_range=(0.0, 0.08),
            noise_step=0.005,
            border_width=0.25,
        ),
        # Stairs up: step height scales 0 → 12 cm.
        "stairs_up": terrain_gen.MeshPyramidStairsTerrainCfg(
            proportion=0.2,
            step_height_range=(0.0, 0.12),
            step_width=0.32,
            platform_width=2.0,
            border_width=1.0,
        ),
        # Stairs down: same scaling but inverted.
        "stairs_down": terrain_gen.MeshInvertedPyramidStairsTerrainCfg(
            proportion=0.2,
            step_height_range=(0.0, 0.12),
            step_width=0.32,
            platform_width=2.0,
            border_width=1.0,
        ),
        # Discrete grid obstacles: cell height scales 0 → 6 cm.
        "random_grid": terrain_gen.MeshRandomGridTerrainCfg(
            proportion=0.2,
            grid_width=0.45,
            grid_height_range=(0.0, 0.06),
            platform_width=2.0,
        ),
    },
)


ASSET_DIR = Path(__file__).resolve().parents[3] / "assets" / "robots" / "mos2026_2_closed_usd" / "usd"
USD_PATH = ASSET_DIR / "mos2026_2.usd"


@configclass
class Mos20262ClosedUsdEnvCfg(DirectRLEnvCfg):
    episode_length_s = 20.0
    decimation = 4
    # Per-joint deflection coefficient (rad) relative to the default pose.
    # joint_target = default + action_scale * clamp(action, ±action_clip).
    # With action_clip=1.5:
    #   hip:           0.5    -> ±0.75 rad   ≈ ±43°
    #   thigh / shank: 0.8145 -> ±1.2217 rad ≈ ±70°
    # Tuple length and order must match `actuated_joint_names` below.
    action_scale = (
        0.5, 0.5, 0.5, 0.5,                      # fl_hip, fr_hip, rl_hip, rr_hip
        0.8145, 0.8145, 0.8145, 0.8145,          # fl_thigh, fr_thigh, rl_thigh, rr_thigh
        0.8145, 0.8145, 0.8145, 0.8145,          # *_shank_link*
    )
    # "position": actions are joint-angle deltas added to the default pose
    #             and sent via set_joint_position_target (closed-loop PD).
    # "effort":   actions are torques scaled by action_scale and applied via
    #             set_joint_effort_target (open-loop torque control).
    action_control_mode = "position"
    # Dimension of the policy action vector. Must equal len(actuated_joint_names)
    # — one scalar per actuated joint (4 hips + 4 thighs + 4 shanks = 12).
    action_space = 12
    # Dimension of the policy observation vector. Built in env._get_observations:
    #   root_lin_vel_b (3) + root_ang_vel_b (3) + projected_gravity_b (3)
    # + commanded_lin_vel_xy (2) + commanded_ang_vel_z (1)
    # + joint_pos - default_joint_pos (12) + joint_vel (12) + last_actions (12)
    # = 45. Update this number whenever the obs cat in env.py changes.
    observation_space = 45
    # Privileged "critic-only" observation size. 0 disables the asymmetric
    # critic; set >0 and emit a "critic" key in _get_observations to use it.
    state_space = 0
    # GUI camera pose for non-headless runs (also used by --livestream). `eye`
    # is the camera position, `lookat` is the target point, both in meters in
    # world space; `resolution` is the rendered window size.
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
        # Rough heightfield terrain + closed-chain articulation produce many
        # more broad-phase pairs than the PhysX defaults expect. Sized for a
        # 32 GB GPU — these caps reserve roughly 8-10 GB of VRAM for PhysX
        # broad-phase buffers, leaving the rest for the policy and rendering.
        physx=PhysxCfg(
            gpu_found_lost_pairs_capacity=2**25,
            gpu_found_lost_aggregate_pairs_capacity=2**29,
            gpu_total_aggregate_pairs_capacity=2**26,
            gpu_max_rigid_contact_count=2**24,
            gpu_max_rigid_patch_count=2**21,
        ),
    )

    # Default to a flat plane. Use `--terrain rough` on the train/play scripts
    # to switch to ROUGH_TERRAIN_CFG (or override `terrain` here in code).
    terrain = TerrainImporterCfg(
        prim_path="/World/ground",
        terrain_type="plane",
        max_init_terrain_level=None,
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
    scene: InteractiveSceneCfg = InteractiveSceneCfg(num_envs=4096, env_spacing=4.0, replicate_physics=False)

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
                effort_limit_sim=30.0,
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
    # When True, _reset_idx promotes envs to harder rows of the terrain
    # generator if they walked far enough during the episode, and demotes
    # envs that fell short. Requires terrain_type="generator" with
    # curriculum=True (e.g. CURRICULUM_TERRAIN_CFG).
    terrain_curriculum_enabled = False
    base_height_target = 0.32
    action_clip = 1.5
    visual_disable_resets = False
    commanded_lin_vel_xy = (1.0, 0.0)
    commanded_ang_vel_z = 0.0
    show_velocity_arrows = True
    # Raised from 0.15 so a robot that "lies down" (body around 0.10-0.18m)
    # terminates instead of farming alive/orientation reward forever.
    fall_height_threshold = 0.22
    # cos(angle) threshold for "tipped over" detection on projected gravity.
    # 0.85 ≈ 32° tilt from upright; raise toward 1.0 to terminate sooner.
    fall_cos_threshold = 0.85
    # Reward weights tuned to break the "stand still" local optimum:
    #   - `alive` is removed: a 0.5/step freebie was outweighing the
    #     small lin-vel tracking reward early in training.
    #   - `track_ang_vel_z` is reduced (was 1.0) because exp(0) gives a
    #     full payout for a robot that simply isn't rotating.
    #   - `base_height` and `flat_orientation` are strong L2 penalties so
    #     drooping or tipping is expensive (no exp saturation).
    #   - `track_lin_vel_xy` is bumped so forward motion dominates.
    #   - Penalties on joint_vel / action_rate / lin_vel_z / ang_vel_xy are
    #     held small so PPO can still freely jitter legs during exploration
    #     without the gait being immediately drowned out by penalties.
    reward_scales = {
        "alive": 0.0,
        "upright": 0.0,
        "flat_orientation": -2.5,
        "base_height": -10.0,
        "lin_vel_z": -0.5,
        "ang_vel_xy": -0.02,
        "joint_vel": -0.0001,
        "action_rate": -0.003,
        "track_lin_vel_xy": 4.0,
        "track_ang_vel_z": 0.5,
        "custom_reward": 0.0,
    }
    # Bandwidth of the exp() in the velocity-tracking reward. Wider band
    # means partial progress (e.g. moving at 0.4 m/s when commanded 1.0)
    # still earns a useful gradient toward the target instead of bottoming
    # out near 0 like a narrow Gaussian would.
    tracking_sigma = 1.0
