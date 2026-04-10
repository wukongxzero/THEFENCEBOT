import torch
import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation, ArticulationCfg, RigidObject, RigidObjectCfg
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.envs import DirectRLEnv, DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import SimulationCfg
from isaaclab.sensors import ContactSensor, ContactSensorCfg
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
            stiffness=800.0,
            damping=40.0,
        )
    }
)

TARGET_CFG = RigidObjectCfg(
    prim_path="/World/envs/env_.*/Target",
    spawn=sim_utils.CuboidCfg(
        size=(0.05, 0.05, 0.05),  # 5cm cube
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=True,        # floats in place as a static target
            linear_damping=10.0,
            angular_damping=10.0,
        ),
        mass_props=sim_utils.MassPropertiesCfg(mass=0.1),
        collision_props=sim_utils.CollisionPropertiesCfg(),
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.2, 0.2)),  # red
    ),
    init_state=RigidObjectCfg.InitialStateCfg(
        pos=(0.5, 0.0, 0.6),   # centered in ASEM workspace
        rot=(1.0, 0.0, 0.0, 0.0),
    ),
)

CONTACT_SENSOR_CFG = ContactSensorCfg(
    prim_path="/World/envs/env_.*/Robot/link6",  # EE link
    update_period=0.0,   # every sim step
    history_length=3,
    debug_vis=False,
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

        self.target = RigidObject(TARGET_CFG)
        self.scene.rigid_objects["target"] = self.target

        #self.contact_sensor = ContactSensor(CONTACT_SENSOR_CFG)
        #self.scene.sensors["contact_sensor"] = self.contact_sensor

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
        self.target.reset(env_ids)

    def get_contact_forces(self):
        if not hasattr(self, 'contact_sensor'):
            return None
        return {
            "in_contact": self.contact_sensor.data.in_contact,
            "net_force": self.contact_sensor.data.net_forces_w,
            "force_history": self.contact_sensor.data.net_forces_w_history
        }