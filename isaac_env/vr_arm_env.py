# vr_arm_env.py
import torch
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import threading

import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation, ArticulationCfg
from isaaclab.envs import DirectRLEnv, DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import SimulationCfg
from isaaclab.utils import configclass
from isaaclab_assets import FRANKA_PANDA_CFG  # built-in Franka


# -------------------------------------------------------
# ROS2 listener — runs in background thread
# -------------------------------------------------------
class VRListener(Node):
    def __init__(self):
        super().__init__('vr_listener')
        self.latest_pose = None
        self._lock = threading.Lock()

        qos = rclpy.qos.QoSProfile(
            depth=10,
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT)

        self.create_subscription(
            PoseStamped, '/vr_pose', self._cb, qos)

    def _cb(self, msg):
        with self._lock:
            self.latest_pose = msg

    def get_pose(self):
        with self._lock:
            return self.latest_pose


# -------------------------------------------------------
# Isaac Lab Environment Config
# -------------------------------------------------------
@configclass
class VRArmEnvCfg(DirectRLEnvCfg):
    sim: SimulationCfg = SimulationCfg(dt=0.01)  # 100 Hz physics
    scene: InteractiveSceneCfg = InteractiveSceneCfg(
        num_envs=1, env_spacing=2.5)

    # Observation/action dims
    num_observations: int = 14  # 7 joint pos + 7 joint vel
    num_actions: int = 7        # 7 DOF Franka


# -------------------------------------------------------
# Isaac Lab Environment
# -------------------------------------------------------
class VRArmEnv(DirectRLEnv):
    cfg: VRArmEnvCfg

    def __init__(self, cfg, ros_listener: VRListener):
        super().__init__(cfg)
        self.ros_listener = ros_listener

    def _setup_scene(self):
        # Spawn Franka arm
        self.robot = Articulation(FRANKA_PANDA_CFG.replace(
            prim_path="/World/envs/env_.*/Robot"))
        self.scene.articulations["robot"] = self.robot

        # Ground plane + lighting
        sim_utils.spawn_ground_plane("/World/ground",
            sim_utils.GroundPlaneCfg())
        sim_utils.spawn_distant_light("/World/light",
            sim_utils.DistantLightCfg(intensity=3000.0))
        self.scene.clone_environments(copy_from_source=False)

    def _get_observations(self) -> dict:
        joint_pos = self.robot.data.joint_pos
        joint_vel = self.robot.data.joint_vel
        obs = torch.cat([joint_pos, joint_vel], dim=-1)
        return {"policy": obs}

    def _pre_physics_step(self, actions: torch.Tensor):
        self.actions = actions

    def _apply_action(self):
        self.robot.set_joint_position_target(self.actions)

    def _get_rewards(self) -> torch.Tensor:
        # Placeholder — not needed for teleoperation
        return torch.zeros(self.num_envs, device=self.device)

    def _get_dones(self):
        return (
            torch.zeros(self.num_envs, dtype=torch.bool, device=self.device),
            torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        )

    def _reset_idx(self, env_ids):
        super()._reset_idx(env_ids)
        self.robot.reset(env_ids)