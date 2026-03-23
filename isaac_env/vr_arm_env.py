import torch
import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation, ArticulationCfg
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.envs import DirectRLEnv, DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import SimulationCfg
from isaaclab.utils import configclass


ASEM_CFG = ArticulationCfg(
    prim_path="/World/envs/env_.*/Robot",
    spawn=sim_utils.UsdFileCfg(
        usd_path="/home/wukong/THEFENCEBOT/Simulation/ASEM_V2.SLDASM/usd/ASEM_V2.usd",
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.0),
        joint_pos={"j1": 0.0, "j2": 0.0, "j3": 0.0, "j4": 0.0, "j5": 0.0, "j6": 0.0}
    ),
    actuators={
        "all_joints": ImplicitActuatorCfg(
            joint_names_expr=["j1", "j2", "j3", "j4", "j5", "j6"],
            stiffness=100.0,
            damping=10.0,
        )
    }
)


@configclass
class ASEMEnvCfg(DirectRLEnvCfg):
    sim: SimulationCfg = SimulationCfg(dt=0.01)
    scene: InteractiveSceneCfg = InteractiveSceneCfg(num_envs=1, env_spacing=2.5)
    decimation: int = 2
    episode_length_s: float = 60.0
    num_observations: int = 12
    num_actions: int = 6
    observation_space: int = 12
    action_space: int = 6


class ASEMEnv(DirectRLEnv):
    cfg: ASEMEnvCfg

    def __init__(self, cfg):
        super().__init__(cfg)

    def _setup_scene(self):
        self.robot = Articulation(ASEM_CFG)
        self.scene.articulations["robot"] = self.robot
        sim_utils.spawn_ground_plane("/World/ground", sim_utils.GroundPlaneCfg())
        sim_utils.spawn_light("/World/light", sim_utils.DistantLightCfg(intensity=3000.0))
        self.scene.clone_environments(copy_from_source=False)

    def _get_observations(self):
        return {"policy": torch.cat([self.robot.data.joint_pos, self.robot.data.joint_vel], dim=-1)}

    def _pre_physics_step(self, actions: torch.Tensor):
        self.actions = actions

    def _apply_action(self):
        self.robot.set_joint_position_target(self.actions)

    def _get_rewards(self):
        return torch.zeros(self.num_envs, device=self.device)

    def _get_dones(self):
        return (
            torch.zeros(self.num_envs, dtype=torch.bool, device=self.device),
            torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        )

    def _reset_idx(self, env_ids):
        super()._reset_idx(env_ids)
        self.robot.reset(env_ids)
