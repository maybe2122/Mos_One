from __future__ import annotations

import gymnasium as gym
import torch

import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation
from isaaclab.envs import DirectRLEnv
from isaaclab.markers import VisualizationMarkers
from isaaclab.markers.config import BLUE_ARROW_X_MARKER_CFG, GREEN_ARROW_X_MARKER_CFG
from isaaclab.utils.math import quat_from_euler_xyz, quat_mul
from isaacsim.core.utils.stage import get_current_stage
from pxr import Sdf, UsdPhysics

from .custom_rewards import compute_custom_reward_terms
from .mos2026_2_closed_usd_env_cfg import Mos20262ClosedUsdEnvCfg


class Mos20262ClosedUsdEnv(DirectRLEnv):
    cfg: Mos20262ClosedUsdEnvCfg

    def __init__(self, cfg: Mos20262ClosedUsdEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        self._actuated_joint_ids, self._actuated_joint_names = self._robot.find_joints(
            self.cfg.actuated_joint_names, preserve_order=True
        )
        if len(self._actuated_joint_ids) != gym.spaces.flatdim(self.single_action_space):
            raise RuntimeError(
                "Closed-chain USD actuator mismatch: "
                f"configured action_space={gym.spaces.flatdim(self.single_action_space)}, "
                f"matched_joints={len(self._actuated_joint_ids)}, "
                f"matched_names={self._actuated_joint_names}"
            )
        self._capture_usd_default_joint_state()
        self._actions = torch.zeros(self.num_envs, gym.spaces.flatdim(self.single_action_space), device=self.device)
        self._previous_actions = torch.zeros_like(self._actions)
        self._episode_sums = {
            key: torch.zeros(self.num_envs, dtype=torch.float, device=self.device)
            for key in self.cfg.reward_scales.keys()
        }
        self._cmd_vel_arrow = None
        self._actual_vel_arrow = None

    def _capture_usd_default_joint_state(self):
        # Closed-chain USD assets often rely on authored passive joint coordinates.
        # Isaac Lab's config defaults every joint to zero unless we explicitly preserve
        # the parsed PhysX state before the first reset.
        self._robot.update(0.0)
        joint_pos = self._robot.data.joint_pos.clone()
        # PhysX may not have populated joint state yet; replace NaN/Inf with the
        # existing default so we don't seed every reset with garbage.
        invalid = ~torch.isfinite(joint_pos)
        if invalid.any():
            joint_pos = torch.where(invalid, self._robot.data.default_joint_pos, joint_pos)
        joint_vel = torch.zeros_like(self._robot.data.default_joint_vel)
        self._robot.data.default_joint_pos[:] = joint_pos
        self._robot.data.default_joint_vel[:] = joint_vel

    def _setup_scene(self):
        self._robot = Articulation(self.cfg.robot)
        self._strip_embedded_ground_prims()
        self._patch_projected_loop_joints()
        self._patch_missing_rigid_body_collisions()
        self.scene.articulations["robot"] = self._robot
        self.cfg.terrain.num_envs = self.scene.cfg.num_envs
        self.cfg.terrain.env_spacing = self.scene.cfg.env_spacing
        self._terrain = self.cfg.terrain.class_type(self.cfg.terrain)
        self.scene.clone_environments(copy_from_source=False)
        self.scene.filter_collisions(global_prim_paths=[self.cfg.terrain.prim_path])
        light_cfg = sim_utils.DomeLightCfg(intensity=2400.0, color=(0.78, 0.82, 0.9))
        light_cfg.func("/World/Light", light_cfg)

    def _patch_projected_loop_joints(self):
        # Some closed-chain USD files model the closure joint as a regular
        # PhysX joint excluded from the reduced-coordinate articulation. Enable
        # projection on those closure joints so their anchors stay coincident.
        loop_joint_names = set(getattr(self.cfg, "projected_loop_joint_names", []))
        if not loop_joint_names:
            return
        stage = get_current_stage()
        patched = []
        for prim in stage.TraverseAll():
            if prim.GetName() not in loop_joint_names:
                continue
            prim.CreateAttribute("physxJoint:enableProjection", Sdf.ValueTypeNames.Bool).Set(True)
            prim.CreateAttribute("physxJoint:projectionLinearTolerance", Sdf.ValueTypeNames.Float).Set(0.002)
            prim.CreateAttribute("physxJoint:projectionAngularTolerance", Sdf.ValueTypeNames.Float).Set(0.05)
            patched.append(str(prim.GetPath()))
        patched_names = {path.rsplit("/", 1)[-1] for path in patched}
        missing = sorted(loop_joint_names - patched_names)
        if missing:
            raise RuntimeError(f"Missing projected closed-chain loop joints: {missing}; patched={patched}")

    def _strip_embedded_ground_prims(self):
        # Some shared USD examples include a demo GroundPlane inside the robot
        # asset. When Isaac Lab spawns the asset as an Articulation, that plane
        # moves with the robot root and can hide the real terrain or collide at
        # the robot spawn height. Remove those demo-only ground prims at runtime.
        if not getattr(self.cfg, "strip_embedded_ground_prims", False):
            return
        stage = get_current_stage()
        to_remove = []
        for prim in stage.TraverseAll():
            path = str(prim.GetPath())
            if not path.startswith("/World/envs/") or "/Robot/" not in path:
                continue
            name = prim.GetName().lower()
            if name in {"groundplane", "ground_plane"} or (name.startswith("ground") and prim.GetTypeName() == "Plane"):
                to_remove.append(prim.GetPath())
        for path in to_remove:
            prim = stage.GetPrimAtPath(path)
            if prim.IsValid():
                prim.SetActive(False)
        if to_remove:
            print(f"[StackForce] Deactivated embedded demo ground prims: {[str(path) for path in to_remove]}", flush=True)

    def _patch_missing_rigid_body_collisions(self):
        # Some USD examples contain rigid-body visual meshes but only partial
        # collision coverage. For visible validation and training, missing leg
        # colliders make the robot pass through the ground. Convert visual Mesh
        # descendants of collider-less rigid bodies into convex-hull colliders.
        if not getattr(self.cfg, "auto_collision_from_visuals", False):
            return
        stage = get_current_stage()
        patched = []
        for prim in stage.TraverseAll():
            schemas = {str(item) for item in prim.GetAppliedSchemas()}
            if "PhysicsRigidBodyAPI" not in schemas:
                continue
            descendants = [item for item in stage.TraverseAll() if item.GetPath().HasPrefix(prim.GetPath()) and item != prim]
            has_collision = any("PhysicsCollisionAPI" in {str(schema) for schema in item.GetAppliedSchemas()} for item in descendants)
            if has_collision:
                continue
            for item in descendants:
                if item.GetTypeName() != "Mesh":
                    continue
                UsdPhysics.CollisionAPI.Apply(item)
                mesh_collision = UsdPhysics.MeshCollisionAPI.Apply(item)
                mesh_collision.CreateApproximationAttr().Set("convexHull")
                patched.append(str(item.GetPath()))
        if patched:
            print(f"[StackForce] Added convex-hull collision to {len(patched)} visual meshes without colliders.", flush=True)

    def _pre_physics_step(self, actions: torch.Tensor):
        self._actions = torch.clamp(actions.clone(), -self.cfg.action_clip, self.cfg.action_clip)
        if self.cfg.action_control_mode == "effort":
            self._processed_actions = self.cfg.action_scale * self._actions
        else:
            default_pos = self._robot.data.default_joint_pos[:, self._actuated_joint_ids]
            self._processed_actions = self.cfg.action_scale * self._actions + default_pos

    def _apply_action(self):
        if self.cfg.action_control_mode == "effort":
            self._robot.set_joint_effort_target(self._processed_actions, joint_ids=self._actuated_joint_ids)
        else:
            self._robot.set_joint_position_target(self._processed_actions, joint_ids=self._actuated_joint_ids)

    def _ensure_velocity_arrows(self):
        if self._cmd_vel_arrow is not None:
            return
        cmd_cfg = GREEN_ARROW_X_MARKER_CFG.copy()
        cmd_cfg.prim_path = "/Visuals/Mos20262/velocity_command"
        cmd_cfg.markers["arrow"].scale = (0.4, 0.4, 0.4)
        self._cmd_vel_arrow = VisualizationMarkers(cmd_cfg)
        act_cfg = BLUE_ARROW_X_MARKER_CFG.copy()
        act_cfg.prim_path = "/Visuals/Mos20262/velocity_actual"
        act_cfg.markers["arrow"].scale = (0.4, 0.4, 0.4)
        self._actual_vel_arrow = VisualizationMarkers(act_cfg)

    def _xy_velocity_to_arrow(self, xy_velocity: torch.Tensor, marker: VisualizationMarkers):
        # `xy_velocity` is in the robot base frame. Return (scale, world-frame quat).
        default_scale = marker.cfg.markers["arrow"].scale
        scale = torch.tensor(default_scale, device=self.device).repeat(xy_velocity.shape[0], 1)
        scale[:, 0] *= torch.linalg.norm(xy_velocity, dim=1) * 3.0 + 1e-3
        heading = torch.atan2(xy_velocity[:, 1], xy_velocity[:, 0])
        zeros = torch.zeros_like(heading)
        arrow_quat = quat_from_euler_xyz(zeros, zeros, heading)
        base_quat_w = self._robot.data.root_quat_w
        arrow_quat = quat_mul(base_quat_w, arrow_quat)
        return scale, arrow_quat

    def _update_velocity_arrows(self):
        self._ensure_velocity_arrows()
        base_pos_w = self._robot.data.root_pos_w.clone()
        base_pos_w[:, 2] += 0.35
        cmd_xy = torch.tensor(self.cfg.commanded_lin_vel_xy, device=self.device).expand(self.num_envs, 2)
        cmd_scale, cmd_quat = self._xy_velocity_to_arrow(cmd_xy, self._cmd_vel_arrow)
        actual_xy = self._robot.data.root_lin_vel_b[:, :2]
        act_scale, act_quat = self._xy_velocity_to_arrow(actual_xy, self._actual_vel_arrow)
        # Offset the actual-velocity arrow slightly so the two don't visually overlap.
        act_pos = base_pos_w.clone()
        act_pos[:, 2] += 0.08
        self._cmd_vel_arrow.visualize(base_pos_w, cmd_quat, cmd_scale)
        self._actual_vel_arrow.visualize(act_pos, act_quat, act_scale)

    def _get_observations(self) -> dict:
        obs = torch.cat(
            [
                self._robot.data.root_lin_vel_b,
                self._robot.data.root_ang_vel_b,
                self._robot.data.projected_gravity_b,
                self._robot.data.joint_pos[:, self._actuated_joint_ids]
                - self._robot.data.default_joint_pos[:, self._actuated_joint_ids],
                self._robot.data.joint_vel[:, self._actuated_joint_ids],
                self._actions,
            ],
            dim=-1,
        )
        # Closed-chain physics can occasionally produce NaN/Inf in PhysX outputs.
        # Replace with 0 so rsl_rl's check_nan doesn't abort; the matching envs
        # are flagged for reset in `_get_dones`.
        obs = torch.nan_to_num(obs, nan=0.0, posinf=0.0, neginf=0.0)
        self._previous_actions = self._actions.clone()
        if getattr(self.cfg, "show_velocity_arrows", True) and self.sim.has_gui():
            self._update_velocity_arrows()
        return {"policy": obs}

    def _get_rewards(self) -> torch.Tensor:
        scales = self.cfg.reward_scales
        root_height = self._robot.data.root_pos_w[:, 2] - self._terrain.env_origins[:, 2]
        base_height_error = torch.square(root_height - self.cfg.base_height_target)
        upright_error = torch.sum(torch.square(self._robot.data.projected_gravity_b[:, :2]), dim=1)
        lin_vel_b = self._robot.data.root_lin_vel_b
        ang_vel_b = self._robot.data.root_ang_vel_b
        lin_vel_z = torch.square(lin_vel_b[:, 2])
        ang_vel_xy = torch.sum(torch.square(ang_vel_b[:, :2]), dim=1)
        joint_vel = torch.sum(torch.square(self._robot.data.joint_vel[:, self._actuated_joint_ids]), dim=1)
        action_rate = torch.sum(torch.square(self._actions - self._previous_actions), dim=1)
        # Constant forward locomotion command. Without a tracking term the
        # policy converges to the "stand still" optimum allowed by the other
        # shaping terms, so add explicit xy/yaw tracking rewards.
        cmd_xy = torch.tensor(self.cfg.commanded_lin_vel_xy, device=self.device).expand(self.num_envs, 2)
        lin_vel_xy_err = torch.sum(torch.square(cmd_xy - lin_vel_b[:, :2]), dim=1)
        ang_vel_yaw_err = torch.square(self.cfg.commanded_ang_vel_z - ang_vel_b[:, 2])
        rewards = {
            "alive": torch.ones(self.num_envs, device=self.device) * scales.get("alive", 0.0),
            "upright": torch.exp(-upright_error / 0.25) * scales.get("upright", 0.0),
            "base_height": base_height_error * scales.get("base_height", 0.0),
            "lin_vel_z": lin_vel_z * scales.get("lin_vel_z", 0.0),
            "ang_vel_xy": ang_vel_xy * scales.get("ang_vel_xy", 0.0),
            "joint_vel": joint_vel * scales.get("joint_vel", 0.0),
            "action_rate": action_rate * scales.get("action_rate", 0.0),
            "track_lin_vel_xy": torch.exp(-lin_vel_xy_err / 0.25) * scales.get("track_lin_vel_xy", 0.0),
            "track_ang_vel_z": torch.exp(-ang_vel_yaw_err / 0.25) * scales.get("track_ang_vel_z", 0.0),
        }
        for key, value in compute_custom_reward_terms(self).items():
            rewards[key] = rewards.get(key, torch.zeros_like(value)) + value * scales.get(key, 0.0)
        reward = torch.sum(torch.stack(list(rewards.values())), dim=0) * self.step_dt
        reward = torch.nan_to_num(reward, nan=0.0, posinf=0.0, neginf=0.0)
        for key, value in rewards.items():
            if key in self._episode_sums:
                self._episode_sums[key] += value * self.step_dt
        return reward

    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        time_out = self.episode_length_buf >= self.max_episode_length - 1
        if getattr(self.cfg, "visual_disable_resets", False):
            no_reset = torch.zeros_like(time_out)
            return no_reset, no_reset
        root_pos = self._robot.data.root_pos_w
        root_height = root_pos[:, 2] - self._terrain.env_origins[:, 2]
        died = root_height < self.cfg.fall_height_threshold
        # Orientation-based fall detection: when the robot is upright,
        # projected_gravity_b ≈ (0, 0, -1). After tipping past the configured
        # tilt threshold the z-component rises toward 0 — terminate so a
        # toppled robot doesn't continue accruing rewards on its side.
        gravity_z = self._robot.data.projected_gravity_b[:, 2]
        tipped_over = gravity_z > -self.cfg.fall_cos_threshold
        died = died | tipped_over
        # Treat NaN/Inf state as a terminal condition so the env is recycled
        # rather than silently propagating bad numbers into the policy.
        invalid_state = (
            ~torch.isfinite(root_pos).all(dim=-1)
            | ~torch.isfinite(self._robot.data.root_lin_vel_b).all(dim=-1)
            | ~torch.isfinite(self._robot.data.joint_pos).all(dim=-1)
            | ~torch.isfinite(self._robot.data.joint_vel).all(dim=-1)
        )
        died = died | invalid_state
        return died, time_out

    def _reset_idx(self, env_ids: torch.Tensor | None):
        if env_ids is None or len(env_ids) == self.num_envs:
            env_ids = self._robot._ALL_INDICES
        self._robot.reset(env_ids)
        super()._reset_idx(env_ids)
        if len(env_ids) == self.num_envs:
            self.episode_length_buf[:] = torch.randint_like(self.episode_length_buf, high=int(self.max_episode_length))
        self._actions[env_ids] = 0.0
        self._previous_actions[env_ids] = 0.0
        joint_pos = self._robot.data.default_joint_pos[env_ids]
        joint_vel = self._robot.data.default_joint_vel[env_ids]
        default_root_state = self._robot.data.default_root_state[env_ids]
        default_root_state[:, :3] += self._terrain.env_origins[env_ids]
        self._robot.write_root_pose_to_sim(default_root_state[:, :7], env_ids)
        self._robot.write_root_velocity_to_sim(default_root_state[:, 7:], env_ids)
        self._robot.write_joint_state_to_sim(joint_pos, joint_vel, None, env_ids)
        self._robot.set_joint_position_target(joint_pos, env_ids=env_ids)
        self._robot.set_joint_velocity_target(joint_vel, env_ids=env_ids)
        for key in self._episode_sums.keys():
            self._episode_sums[key][env_ids] = 0.0
