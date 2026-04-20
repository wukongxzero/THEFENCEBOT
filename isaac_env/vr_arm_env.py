import os
from pathlib import Path

import torch
import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation, ArticulationCfg, RigidObject, RigidObjectCfg
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.envs import DirectRLEnv, DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import SimulationCfg
from isaaclab.sensors import ContactSensor, ContactSensorCfg
from isaaclab.utils import configclass


# ---------------------------------------------------------------------------
# Resolve the ASEM USD path.
#
# Priority:
#   1. FENCEBOT_USD_PATH environment variable (set this on any new machine).
#   2. Repo-relative default: <repo_root>/Simulation/ASEM_V2.SLDASM/usd/ASEM_V2.usd
#
# The previous hardcoded /home/wukong/... path broke any machine that wasn't
# Pavan's laptop — including the grading environment.
# ---------------------------------------------------------------------------
_REPO_ROOT = Path(__file__).resolve().parent.parent
_DEFAULT_USD = _REPO_ROOT / "Simulation" / "ASEM_V2.SLDASM" / "usd" / "ASEM_V2.usd"
ASEM_USD_PATH = os.environ.get("FENCEBOT_USD_PATH", str(_DEFAULT_USD))


ASEM_CFG = ArticulationCfg(
    prim_path="/World/envs/env_.*/Robot",
    spawn=sim_utils.UsdFileCfg(
        usd_path=ASEM_USD_PATH,
        activate_contact_sensors=True,
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
        size=(0.12, 0.12, 0.12),  # 5cm cube
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
        pos=(0.500, -0.017, 0.600),
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

        # Contact sensor on the end-effector link — needed for sword-on-target
        # scoring. Data is accessed through get_contact_forces() below.
        self.contact_sensor = ContactSensor(CONTACT_SENSOR_CFG)
        self.scene.sensors["contact_sensor"] = self.contact_sensor

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

    def get_contact_forces(self, force_threshold: float = 0.5):
        """
        Return contact sensor state for the end-effector link.

        force_threshold: minimum force magnitude (N) to consider a hit
                         (filters out noise / numerical jitter).

        Returns None if the sensor isn't active, otherwise:
            {
                "in_contact":    (N,)   bool tensor
                "net_force":     (N, 3) force vector in world frame
                "force_history": (N, H, 3) recent force history
                "hit":           (N,)   bool — force magnitude above threshold
                "force_mag":     (N,)   float — ||net_force||
            }
        """
        if not hasattr(self, "contact_sensor"):
            return None

        data = self.contact_sensor.data
        net = data.net_forces_w
        # net shape is (num_envs, 1, 3) in DirectRLEnv; flatten the body dim.
        if net.dim() == 3:
            net = net.squeeze(1)
        force_mag = torch.linalg.norm(net, dim=-1)
        hit = force_mag > force_threshold

        return {
            "in_contact":    hit,
            "net_force":     net,
            "force_history": data.net_forces_w_history,
            "hit":           hit,
            "force_mag":     force_mag,
        }