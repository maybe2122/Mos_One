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
    # 每个子地形为 8x8 m。3x3 网格覆盖 24x24 m，在 env_spacing=4 的情况下
    # 足以容纳 128 个环境实例。
    size=(16.0, 16.0),
    border_width=10.0,
    num_rows=6,
    num_cols=6,
    horizontal_scale=0.1,
    vertical_scale=0.005,
    slope_threshold=0.75,
    use_cache=False,
    sub_terrains={
        # 轻微凹凸地面：1-4 cm 随机高度变化。增大凸起幅度可提高地形难度；
        # 作为新策略的初始训练地形效果较好。
        "random_rough": terrain_gen.HfRandomUniformTerrainCfg(
            proportion=1.0,
            noise_range=(0.01, 0.04),
            noise_step=0.005,
            border_width=0.25,
        ),
    },
)


# 课程学习地形：行 = 难度等级（等级 0 最简单）。
# `curriculum=True` 使子地形沿行方向从难度 0 到 1 生成，
# 因此每个环境从最平坦的变体开始。环境的 `_reset_idx` 会根据
# 策略在每个回合中行走的距离来升级/降级环境所在行。
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
        # 纯平面 - 与难度无关。
        "flat": terrain_gen.MeshPlaneTerrainCfg(proportion=0.2),
        # 凹凸地面：噪声随难度从 0 缩放至 8 cm。
        "rough": terrain_gen.HfRandomUniformTerrainCfg(
            proportion=0.2,
            noise_range=(0.0, 0.08),
            noise_step=0.005,
            border_width=0.25,
        ),
        # 上楼梯：台阶高度随难度从 0 缩放至 12 cm。
        "stairs_up": terrain_gen.MeshPyramidStairsTerrainCfg(
            proportion=0.2,
            step_height_range=(0.0, 0.12),
            step_width=0.32,
            platform_width=2.0,
            border_width=1.0,
        ),
        # 下楼梯：同样的缩放但方向相反。
        "stairs_down": terrain_gen.MeshInvertedPyramidStairsTerrainCfg(
            proportion=0.2,
            step_height_range=(0.0, 0.12),
            step_width=0.32,
            platform_width=2.0,
            border_width=1.0,
        ),
        # 离散网格障碍：单元格高度随难度从 0 缩放至 6 cm。
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
    # 每个关节相对于默认姿态的偏转系数（弧度）。
    # joint_target = default + action_scale * clamp(action, ±action_clip)。
    # 当 action_clip=1.5 时：
    #   髀关节：       0.5    -> ±0.75 rad   ≈ ±43°
    #   大腿/小腿：    0.8145 -> ±1.2217 rad ≈ ±70°
    # 元组长度和顺序必须与下方的 `actuated_joint_names` 匹配。
    action_scale = (
        0.5, 0.5, 0.5, 0.5,                      # fl_hip, fr_hip, rl_hip, rr_hip
        0.8145, 0.8145, 0.8145, 0.8145,          # fl_thigh, fr_thigh, rl_thigh, rr_thigh
        0.8145, 0.8145, 0.8145, 0.8145,          # *_shank_link*
    )
    # "position"：动作为关节角度增量，叠加到默认姿态上，
    #              通过 set_joint_position_target 发送（闭环 PD 控制）。
    # "effort"：  动作为力矩，经 action_scale 缩放后通过
    #              set_joint_effort_target 施加（开环力矩控制）。
    action_control_mode = "position"
    # 策略动作向量维度。必须等于 len(actuated_joint_names)
    # — 每个驱动关节一个标量（4 髀 + 4 大腿 + 4 小腿 = 12）。
    action_space = 12
    # 策略观测向量维度。在 env._get_observations 中构建：
    #   root_lin_vel_b (3) + root_ang_vel_b (3) + projected_gravity_b (3)
    # + commanded_lin_vel_xy (2) + commanded_ang_vel_z (1)
    # + joint_pos - default_joint_pos (12) + joint_vel (12) + last_actions (12)
    # = 45。每当 env.py 中的 obs 拼接发生变化时需更新此值。
    observation_space = 45
    # 特权“仅评论家”观测维度。0 表示禁用非对称评论家；
    # 设为 >0 并在 _get_observations 中输出 "critic" 键即可启用。
    state_space = 0
    # 非无头模式运行时的 GUI 相机位姿（也用于 --livestream）。`eye`
    # 为相机位置，`lookat` 为目标点，均以米为单位在世界坐标系中；
    # `resolution` 为渲染窗口尺寸。
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
            static_friction=1.5,
            dynamic_friction=1.5,
            restitution=0.0,
        ),
        # 粗糙高度场地形 + 闭链关节会产生远超 PhysX 默认值的宽相位碰撞对。
        # 以下参数针对 32 GB GPU 调整 — 这些上限为 PhysX 宽相位缓冲区
        # 预留约 8-10 GB 显存，其余留给策略和渲染。
        physx=PhysxCfg(
            gpu_found_lost_pairs_capacity=2**25,
            gpu_found_lost_aggregate_pairs_capacity=2**29,
            gpu_total_aggregate_pairs_capacity=2**26,
            gpu_max_rigid_contact_count=2**24,
            gpu_max_rigid_patch_count=2**21,
        ),
    )

    # 默认使用平坦平面。在训练/播放脚本中使用 `--terrain rough`
    # 可切换到 ROUGH_TERRAIN_CFG（或在代码中直接覆盖 `terrain`）。
    terrain = TerrainImporterCfg(
        prim_path="/World/ground",
        terrain_type="plane",
        max_init_terrain_level=None,
        collision_group=-1,
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.28, 0.30, 0.32)),
        physics_material=sim_utils.RigidBodyMaterialCfg(
            friction_combine_mode="multiply",
            restitution_combine_mode="multiply",
            static_friction=1.5,
            dynamic_friction=1.5,
            restitution=0.0,
        ),
        debug_vis=False,
    )

    # 快速克隆路径：闭链 USD 物理只解析一次，然后复刻到其余 N-1 个 env 子树。
    # 启动时间近似与 num_envs 无关，单 env 显存占用大幅下降。
    # 之前设为 False 是因为上游 USD 里嵌了 PhysicsScene + GroundPlane，
    # 会让 replicate 路径报错；现在两者都已经被剥离
    # （见 tools/strip_embedded_ground.py；扫描 stage 已确认没有 PhysicsScene
    # 也没有内嵌灯光/相机），所以可以放心启用快路径。
    # 如果以后换上游 USD 而新 USD 又带了 scene prim，把这里改回 False 即可，
    # 否则启动时会看到 PhysX "clone failed" / 重复 PhysicsScene 报错。
    scene: InteractiveSceneCfg = InteractiveSceneCfg(num_envs=4096, env_spacing=4.0, replicate_physics=True)

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
                # FL 腿的 "shank_link_a" 在 USD 中命名为 `fl_shank_link`（无 _a 后缀）；
                # 其余三条腿使用 `*_shank_link_a` 命名约定。
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
    # 用来解析四个"脚"对应 body 的名字正则。
    # 这个闭链 USD 每条腿其实有 4 个 shank 相关 body
    # (`*_shank_link_a`, `*_shank_link_b`, `*_shank_motor_gear`, `*_shank`)，
    # 如果直接写 `.*shank.*` 会匹配到 16 个 body，把电机齿轮和被动平行连杆
    # 也算成"脚"，foot_slip 奖励会算到错误的物体上。
    # 这里只要每条腿"被驱动的主小腿"——也就是 actuated shank 关节的子 body。
    # FL 腿命名比较特殊（`fl_shank_link` 而不是 `fl_shank_link_a`），
    # 其他三条腿都遵循 `*_shank_link_a` 约定。用 ^...$ 精确匹配避免误中子串。
    foot_body_names_expr = [
        "^fl_shank_link_a$",
        "^fr_shank_link_a$",
        "^rl_shank_link_a$",
        "^rr_shank_link_a$",
    ]
    # 脚 body 相对地形原点的高度低于此阈值时，视为"接触地面"，
    # 才会参与 foot_slip 惩罚的统计。
    # 站立状态下本机器人小腿 body 中心大约离地 3-5 cm，所以 0.07 m 留一点余量。
    foot_contact_height_threshold = 0.07
    # 左右对称步态奖励用到的左/右两侧关节分组。
    # 惩罚的是两侧"偏离默认姿态的能量"之差（看大小不看符号），
    # 所以左腿向前迈、右腿向后摆这种正常交替步态不会被惩罚，
    # 只有"瘸腿/重心偏一侧"这种真正的不对称才会被扣分。
    # 两个列表的顺序要按腿/类型一一对应（hip/thigh/shank），
    # 这样配对的关节默认偏置相同，差值才有意义。
    left_leg_joint_names = [
        "fl_hip", "rl_hip",
        "fl_thigh", "rl_thigh",
        "fl_shank_link", "rl_shank_link_a",
    ]
    right_leg_joint_names = [
        "fr_hip", "rr_hip",
        "fr_thigh", "rr_thigh",
        "fr_shank_link_a", "rr_shank_link_a",
    ]
    # 闭环约束关节（每条腿用来"闭合"四连杆平行机构的那根 PhysX 关节）。
    # 这 4 个关节在 USD 里标了 `physics:excludeFromArticulation=True`，
    # 也就是说 PhysX 把它们当作独立约束求解，不放进缩减坐标的 articulation 树。
    # 如果不开 projection，它们的 anchor 会逐步漂移、solver 误差累积，
    # 最终爆掉——这就是 2026-05-15 跑训练时看到的 1e+28 奖励尖峰的根因。
    # env.py 里的 `_patch_projected_loop_joints` 会给下面每个名字开
    # physxJoint:enableProjection。
    #
    # 如果以后换 USD，把这个列表留空并把 `auto_detect_loop_joints` 设为 True，
    # env._setup_scene 启动时会把所有被 exclude 的关节名打印出来，照贴即可。
    projected_loop_joint_names = [
        "fl_close_loop",
        "fr_close_loop",
        "rl_close_loop",
        "rr_close_loop",
    ]
    auto_detect_loop_joints = True
    auto_collision_from_visuals = True
    # 原 USD 自带的 /mos2026_2/GroundPlane 已经通过
    # tools/strip_embedded_ground.py 永久从 USD 文件中删除，
    # 启动时不需要再扫一遍 stage 软关闭。
    # 如果以后换上游 USD 而新 USD 又带 demo ground plane，改回 True 即可。
    strip_embedded_ground_prims = False
    # 设为 True 时，_reset_idx 会根据每个回合走出的距离把走得远的 env
    # 升到更难的地形行、走得太近的 env 降级。
    # 需要 terrain_type="generator" 且地形 cfg 设 curriculum=True
    # （比如 CURRICULUM_TERRAIN_CFG）才生效。
    terrain_curriculum_enabled = False
    base_height_target = 0.32
    action_clip = 1.5
    visual_disable_resets = False
    commanded_lin_vel_xy = (1.0, 0.0)
    commanded_ang_vel_z = 0.0
    show_velocity_arrows = True
    # 从 0.15 抬到 0.22：避免机器人"趴下"后（base 高度 0.10-0.18 m 之间）
    # 还在持续薅 alive / orientation 奖励。低于这个高度就判终止。
    fall_height_threshold = 0.22
    # 投影重力 z 分量的 cos(angle) 阈值，用来判定"翻倒"。
    # 0.85 大约对应离竖直方向 32° 倾角；越接近 1.0 越早终止。
    fall_cos_threshold = 0.85
    # 这套 reward 权重是为了打破"原地不动"局部最优而调出来的：
    #   - 去掉 `alive`：之前 0.5/step 的白送奖励压过了训练初期微弱的速度跟踪奖励。
    #   - `track_ang_vel_z` 从 1.0 降到 0.5：原来不转身的机器人 exp(0) 就拿满分。
    #   - `flat_orientation` 是 L2 惩罚项（在 _get_rewards 里有 clamp 上限），
    #     翻倒代价高但不会被 exp 饱和掉。
    #   - `base_height` 在 _get_rewards 里 clamp(., 0, 4)；这里的权重决定 cap
    #     范围内的强度。之前是 -10 + 没有 clamp，叠加闭链 PhysX 爆掉时
    #     单步能炸到 1e+28。改成 -3 后最坏情况是 4 * 3 = 12（再乘 step_dt），
    #     既保留惩罚作用又不会主导整个 reward 预算。
    #   - `lin_vel_z` 之前 -0.5、lin_vel_z² 上限 100 → 单步最多 -50，
    #     完全盖过了 +4 的跟踪奖励，这是之前 reward 卡在 500-1100 的主因。
    #     现在 -0.05 + 上限 100 → 单步最多 -5，恰好。
    #   - `track_lin_vel_xy` 从 4.0 提到 6.0，让"往前走"明显比"站住"赚得多。
    #   - joint_vel / action_rate / ang_vel_xy 的惩罚都保持很小，
    #     这样 PPO 在探索时还能自由地抖动腿。
    reward_scales = {
        "alive": 0.0,
        "upright": 0.0,
        "flat_orientation": -2.5,
        "base_height": -3.0,
        "lin_vel_z": -0.05,
        "ang_vel_xy": -0.02,
        "joint_vel": -0.0001,
        "action_rate": -0.005,
        "track_lin_vel_xy": 6.0,
        "track_ang_vel_z": 0.5,
        # 足端打滑惩罚：脚 body 水平速度的平方，乘以一个"是否着地"的接触掩码后求和。
        # 没有这一项时 policy 容易学出"滑步"的步态——脚虽然踩着地，但实际上
        # 在身体下方持续滑动，从视觉上就是严重打滑。
        # -0.1 这个权重保证打滑代价能盖过 track_lin_vel_xy 拿到的收益
        # （≈ 6 * exp(-err)）：一只脚以 1 m/s 滑动时 vel² = 1，对应单步单脚 -0.1。
        "foot_slip": -0.1,
        # 左右对称步态奖励：基于"幅度"的版本，不会强求瞬时左==右
        # （那样会扼杀交替步态）。这里惩罚的是左侧关节偏离默认姿态的总能量
        # 和右侧总能量之差的平方，引导 policy 用对称的姿态去走路，
        # 而不是瘸腿/重心偏一侧。权重故意压得小，对应 doc/足端打滑问题.md
        # 里说的"修饰项"定位。
        "gait_symmetry": -0.05,
        "custom_reward": 0.0,
    }
    # 速度跟踪奖励里 exp() 的带宽。带宽越大，部分达成（比如指令 1.0 m/s
    # 实际只走 0.4 m/s）也能拿到有意义的梯度，引导策略向目标速度靠拢；
    # 带宽太小会像窄高斯一样在 0 附近直接饱和到 0。
    tracking_sigma = 1.0
