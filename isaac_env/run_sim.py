import socket
import struct
import threading
import torch
from isaaclab.app import AppLauncher

launcher = AppLauncher({"headless": False})
simulation_app = launcher.app

from vr_arm_env import ASEMEnv, ASEMEnvCfg
from isaaclab.controllers import DifferentialIKController, DifferentialIKControllerCfg


class UDPListener:
    def __init__(self, port=5006):
        self.latest = None
        self._lock = threading.Lock()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', port))
        threading.Thread(target=self._loop, daemon=True).start()
        print(f"Isaac Lab listening on UDP port {port}")

    def _loop(self):
        while True:
            data, _ = self.sock.recvfrom(28)
            if len(data) == 28:
                with self._lock:
                    self.latest = struct.unpack('7f', data)

    def get_pose(self):
        with self._lock:
            return self.latest


def main():
    listener = UDPListener(port=5006)

    cfg = ASEMEnvCfg()
    env = ASEMEnv(cfg)
    env.reset()

    # Print all body positions at startup
    print("=== Body positions at rest ===")
    for i, name in enumerate(env.robot.body_names):
        pos = env.robot.data.body_pos_w[0, i, :].cpu().numpy()
        print(f"  Body {i} '{name}': [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]")
    print("==============================")

    ik_cfg = DifferentialIKControllerCfg(
        command_type="pose",        # fixed: was "position", needs orientation too
        use_relative_mode=False,
        ik_method="dls",
        ik_params={"lambda_val":0.1}
    )
    ik_controller = DifferentialIKController(
        ik_cfg, num_envs=1, device=env.device)

    ee_idx = env.robot.body_names.index("link6")
    print(f"End effector index: {ee_idx}")
    joint_ids = list(range(6))
    step_count = 0

    print("Sim running with IK")

    while simulation_app.is_running():
        pose = listener.get_pose()

        if pose is not None:
            x, y, z, qx, qy, qz, qw = pose

            ee_pos_target = torch.tensor([[x, y, z]], device=env.device)
            ee_quat_target = torch.tensor([[qw, qx, qy, qz]], device=env.device)  # IsaacLab expects (qw, qx, qy, qz)

            ik_controller.set_command(
                torch.cat([ee_pos_target, ee_quat_target], dim=-1))  # fixed: pass full pose

            joint_pos = env.robot.data.joint_pos[:, joint_ids]
            root_pos = env.robot.data.root_pos_w[:, :]
            root_quat = env.robot.data.root_quat_w[:, :]
            
            ee_pos_curr = env.robot.data.body_pos_w[:, ee_idx, :] - root_pos
            ee_quat_curr = env.robot.data.body_quat_w[:, ee_idx, :] 
            jacobian = env.robot.root_physx_view.get_jacobians()[:, ee_idx - 1, :6, :6]
            #delta = ik_controller.compute(
            #    ee_pos_curr, ee_quat_curr, jacobian, joint_pos)
            #delta = torch.clamp(delta,-0.1,0.1) #limit step size per iteration 


            #actions = torch.clamp(joint_pos + delta, -3.14, 3.14)  # ← clamp to joint limits
        
            actions = ik_controller.compute(
                ee_pos_curr, ee_quat_curr, jacobian, joint_pos)
            actions = torch.clamp(actions, -3.14, 3.14)
            step_count += 1

            # DEBUG — only print on first step
            if step_count == 1:
                jac = env.robot.root_physx_view.get_jacobians()
                print(f"Jacobian full shape: {jac.shape}")
                print(f"ee_idx: {ee_idx}")
                print(f"Using jacobian index: {ee_idx - 1}")
                print(f"EE pos at start: {env.robot.data.body_pos_w[0, ee_idx, :].cpu().numpy()}")

            if step_count % 50 == 0:
                actual = ee_pos_curr[0].cpu().numpy()
                print(f"Target: [{x:.3f}, {y:.3f}, {z:.3f}] | Actual: [{actual[0]:.3f}, {actual[1]:.3f}, {actual[2]:.3f}]")
        else:
            actions = torch.zeros(1, 6, device=env.device)

        env.step(actions)
        contact = env.get_contact_forces()
        if contact and contact["in_contact"].any():
            force = contact["net_force"][0].cpu().numpy()
            print(f"CONTACT — Force: [{force[0]:.3f}, {force[1]:.3f}, {force[2]:.3f}] N")


if __name__ == "__main__":
    main()