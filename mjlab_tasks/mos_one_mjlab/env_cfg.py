"""mos2026_2 flat-terrain velocity environment configuration for mjlab.

Adapted from mjlab's Unitree Go1 flat config (tasks/velocity/config/go1):
same MDP skeleton, robot-specific fields swapped for the mos2026_2
closed-chain quadruped. See doc/mjlab_integration.md §6.
"""

import math

from mjlab.envs import ManagerBasedRlEnvCfg
from mjlab.envs import mdp as envs_mdp
from mjlab.envs.mdp.actions import JointPositionActionCfg
from mjlab.managers import TerminationTermCfg
from mjlab.managers.scene_entity_config import SceneEntityCfg
from mjlab.sensor import (
  ContactMatch,
  ContactSensorCfg,
  ObjRef,
  RingPatternCfg,
  TerrainHeightSensorCfg,
)
from mjlab.tasks.velocity import mdp
from mjlab.tasks.velocity.mdp import UniformVelocityCommandCfg
from mjlab.tasks.velocity.velocity_env_cfg import make_velocity_env_cfg

from .robot import (
  ACTUATED_JOINTS,
  FOOT_GEOM_NAMES,
  MOS_ACTION_SCALE,
  get_mos_robot_cfg,
)

FOOT_SITE_NAMES = ("FL", "FR", "RL", "RR")


def mos_one_flat_env_cfg(play: bool = False) -> ManagerBasedRlEnvCfg:
  """mos2026_2 flat-terrain velocity tracking configuration."""
  cfg = make_velocity_env_cfg()

  # -- sim: flat-ground budget (mirrors go1 flat) -----------------------------
  cfg.sim.njmax = 300
  cfg.sim.mujoco.ccd_iterations = 50
  cfg.sim.contact_sensor_maxmatch = 64
  cfg.sim.nconmax = None
  # Closed-chain legs need a small timestep: the loop <connect> equalities
  # (solref clamped to 0.004 in robot.py) and the 0.005 contact solref both
  # require timeconst >= 2*dt. 500 Hz physics + decimation 10 keeps the
  # 50 Hz control rate. At mjlab's default dt=0.005 the loops go NaN at scale.
  cfg.sim.mujoco.timestep = 0.002
  cfg.decimation = 10
  # Closed loops + contact need more solver sweeps than an open-chain quadruped.
  cfg.sim.mujoco.iterations = 20

  # -- scene -------------------------------------------------------------------
  cfg.scene.entities = {"robot": get_mos_robot_cfg()}

  # Flat plane instead of the generator terrain.
  assert cfg.scene.terrain is not None
  cfg.scene.terrain.terrain_type = "plane"
  cfg.scene.terrain.terrain_generator = None

  # Sensors: drop the trunk raycast scan (rough-only); wire the per-foot
  # height scan to our shank-tip sites; add a feet<->terrain contact sensor.
  sensors = []
  for sensor in cfg.scene.sensors or ():
    if sensor.name == "terrain_scan":
      continue
    if sensor.name == "foot_height_scan":
      assert isinstance(sensor, TerrainHeightSensorCfg)
      sensor.frame = tuple(
        ObjRef(type="site", name=s, entity="robot") for s in FOOT_SITE_NAMES
      )
      sensor.pattern = RingPatternCfg.single_ring(radius=0.03, num_samples=4)
    sensors.append(sensor)
  feet_ground_cfg = ContactSensorCfg(
    name="feet_ground_contact",
    primary=ContactMatch(mode="geom", pattern=FOOT_GEOM_NAMES, entity="robot"),
    secondary=ContactMatch(mode="body", pattern="terrain"),
    fields=("found", "force"),
    reduce="netforce",
    num_slots=1,
    track_air_time=True,
  )
  cfg.scene.sensors = tuple(sensors) + (feet_ground_cfg,)

  # -- observations: no height scan on flat ------------------------------------
  del cfg.observations["actor"].terms["height_scan"]
  del cfg.observations["critic"].terms["height_scan"]

  # -- actions: residual joint targets with the Isaac-aligned per-joint scale --
  joint_pos_action = cfg.actions["joint_pos"]
  assert isinstance(joint_pos_action, JointPositionActionCfg)
  joint_pos_action.scale = MOS_ACTION_SCALE

  # -- events ------------------------------------------------------------------
  cfg.events["foot_friction"].params["asset_cfg"] = SceneEntityCfg(
    "robot", geom_names=FOOT_GEOM_NAMES
  )
  cfg.events["base_com"].params["asset_cfg"].body_names = ("base",)

  # -- rewards: robot-specific wiring -------------------------------------------
  cfg.rewards["upright"].params["asset_cfg"].body_names = ("base",)
  cfg.rewards["upright"].params.pop("terrain_sensor_names", None)
  cfg.rewards["body_ang_vel"].params["asset_cfg"].body_names = ("base",)

  # Posture stds: tight when standing, loose when walking/running. Constrain
  # only the 12 actuated joints — the passive closed-chain joints' positions
  # are dictated by the loop geometry and must not be penalized.
  cfg.rewards["pose"].params["asset_cfg"] = SceneEntityCfg(
    "robot", joint_names=ACTUATED_JOINTS
  )
  cfg.rewards["pose"].params["std_standing"] = {
    r"(fl|fr|rl|rr)_(hip|thigh)": 0.05,
    r"fl_shank_link|(fr|rl|rr)_shank_link_a": 0.1,
  }
  cfg.rewards["pose"].params["std_walking"] = {
    r"(fl|fr|rl|rr)_(hip|thigh)": 0.3,
    r"fl_shank_link|(fr|rl|rr)_shank_link_a": 0.6,
  }
  cfg.rewards["pose"].params["std_running"] = cfg.rewards["pose"].params[
    "std_walking"
  ]

  for reward_name in ("foot_clearance", "foot_slip"):
    cfg.rewards[reward_name].params["asset_cfg"].site_names = FOOT_SITE_NAMES

  cfg.rewards["body_ang_vel"].weight = 0.0
  cfg.rewards["angular_momentum"].weight = 0.0
  cfg.rewards["air_time"].weight = 0.0
  # Closed-chain shank can't lift as high as Go1's calf; relax clearance target.
  cfg.rewards["foot_clearance"].params["target_height"] = 0.06
  cfg.rewards["foot_swing_height"].params["target_height"] = 0.06

  # -- terminations: flat-ground set --------------------------------------------
  cfg.terminations.pop("out_of_terrain_bounds", None)
  cfg.terminations["fell_over"] = TerminationTermCfg(
    func=mdp.bad_orientation,
    params={"limit_angle": math.radians(70.0)},
  )
  # Closed-chain quarantine (same semantics as the Isaac env's invalid_state):
  # the loop constraints blow up to NaN roughly once per few-million env-steps;
  # terminate + reset the offending env instead of aborting the whole run.
  # The NaN reward it emits on that step is zeroed in __init__.py's
  # RslRlVecEnvWrapper.step patch (rewards are computed before the reset).
  cfg.terminations["nan_state"] = TerminationTermCfg(func=envs_mdp.nan_detection)

  # -- curriculum: no terrain levels on flat ------------------------------------
  cfg.curriculum.pop("terrain_levels", None)

  # -- viewer --------------------------------------------------------------------
  cfg.viewer.body_name = "base"
  cfg.viewer.distance = 1.5
  cfg.viewer.elevation = -10.0

  # -- play-mode overrides --------------------------------------------------------
  if play:
    cfg.episode_length_s = int(1e9)
    cfg.observations["actor"].enable_corruption = False
    cfg.events.pop("push_robot", None)
    cfg.curriculum = {}
    twist_cmd = cfg.commands["twist"]
    assert isinstance(twist_cmd, UniformVelocityCommandCfg)
    twist_cmd.ranges.lin_vel_x = (-1.0, 1.5)
    twist_cmd.ranges.ang_vel_z = (-0.7, 0.7)

  return cfg
