import socket
import struct
import threading
import torch
import csv
import time
from datetime import datetime
from pathlib import Path
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


class CSVLogger:
    def __init__(self, log_dir: str = "logs"):
        Path(log_dir).mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.path = Path(log_dir) / f"fencebot_{timestamp}.csv"

        self._file = open(self.path, "w", newline="")
        self._writer = csv.writer(self._file)

        # Header
        self._writer.writerow([
            "timestamp_s",
            "step",
            # Target EE pose from VR
            "target_x", "target_y", "target_z",
            "target_qx", "target_qy", "target_qz", "target_qw",
            # Actual EE position
            "ee_x", "ee_y", "ee_z",
            # Joint positions (6 DOF)
            "j1", "j2", "j3", "j4", "j5", "j6",
            # Contact
            "force_mag", "hit",
            "force_fx", "force_fy", "force_fz",
            # Hit counter
            "hit_counter",
        ])
        print(f"[CSVLogger] Logging to {self.path}")

    def write(self, step, pose, ee_pos, joint_pos, contact, hit_counter):
        ts = time.time()

        # VR target pose (may be None if no packet yet)
        if pose is not None:
            x, y, z, qx, qy, qz, qw = pose
        else:
            x, y, z, qx, qy, qz, qw = [float("nan")] * 7

        # EE actual position
        ex, ey, ez = ee_pos[0].cpu().numpy() if ee_pos is not None else [float("nan")] * 3

        # Joint positions
        jp = joint_pos[0].cpu().numpy().tolist() if joint_pos is not None else [float("nan")] * 6

        # Contact
        if contact is not None:
            force_mag = float(contact["force_mag"][0])
            hit = int(bool(contact["hit"][0]))
            fv = contact["net_force"][0].cpu().numpy()
            fx, fy, fz = float(fv[0]), float(fv[1]), float(fv[2])
        else:
            force_mag, hit, fx, fy, fz = 0.0, 0, 0.0, 0.0, 0.0

        self._writer.writerow([
            f"{ts:.4f}", step,
            f"{x:.4f}", f"{y:.4f}", f"{z:.4f}",
            f"{qx:.4f}", f"{qy:.4f}", f"{qz:.4f}", f"{qw:.4f}",
            f"{ex:.4f}", f"{ey:.4f}", f"{ez:.4f}",
            *[f"{v:.4f}" for v in jp],
            f"{force_mag:.4f}", hit,
            f"{fx:.4f}", f"{fy:.4f}", f"{fz:.4f}",
            hit_counter,
        ])

    def close(self):
        self._file.flush()
        self._file.close()
        print(f"[CSVLogger] Saved to {self.path}")


def main():
    listener = UDPListener(port=5006)
    logger = CSVLogger(log_dir="logs")

    cfg = ASEMEnvCfg()
    env = ASEMEnv(cfg)
    env.reset()

    print("=== Body positions at rest ===")
    for i, name in enumerate(env.robot.body_names):
        pos = env.robot.data.body_pos_w[0, i, :].cpu().numpy()
        print(f"  Body {i} '{name}': [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]")
    print("==============================")

    ik_cfg = DifferentialIKControllerCfg(
        command_type="pose",
        use_relative_mode=False,
        ik_method="dls",
        ik_params={"lambda_val": 0.1}
    )
    ik_controller = DifferentialIKController(ik_cfg, num_envs=1, device=env.device)

    ee_idx = env.robot.body_names.index("link6")
    print(f"End effector index: {ee_idx}")
    
    ee_world = env.robot.data.body_pos_w[0, ee_idx, :].cpu().numpy()
    print(f"EE world position: [{ee_world[0]:.3f}, {ee_world[1]:.3f}, {ee_world[2]:.3f}]")
    joint_ids = list(range(6))
    step_count = 0
    hit_counter = 0
    last_in_contact = False

    print("Sim running with IK + CSV logging")

    try:
        while simulation_app.is_running():
            pose = listener.get_pose()

            if pose is not None:
                x, y, z, qx, qy, qz, qw = pose

                ee_pos_target = torch.tensor([[x, y, z]], device=env.device)
                ee_quat_target = torch.tensor([[qw, qx, qy, qz]], device=env.device)

                ik_controller.set_command(
                    torch.cat([ee_pos_target, ee_quat_target], dim=-1))

                joint_pos = env.robot.data.joint_pos[:, joint_ids]
                root_pos = env.robot.data.root_pos_w[:, :]

                ee_pos_curr = env.robot.data.body_pos_w[:, ee_idx, :] - root_pos
                ee_quat_curr = env.robot.data.body_quat_w[:, ee_idx, :]
                jacobian = env.robot.root_physx_view.get_jacobians()[:, ee_idx - 1, :6, :6]

                actions = ik_controller.compute(
                    ee_pos_curr, ee_quat_curr, jacobian, joint_pos)
                actions = torch.clamp(actions, -3.14, 3.14)
                step_count += 1

                if step_count == 1:
                    jac = env.robot.root_physx_view.get_jacobians()
                    print(f"Jacobian full shape: {jac.shape}")
                    print(f"ee_idx: {ee_idx}")
                    print(f"Using jacobian index: {ee_idx - 1}")
                    print(f"EE pos at start: {env.robot.data.body_pos_w[0, ee_idx, :].cpu().numpy()}")
                    ee_world = env.robot.data.body_pos_w[0, ee_idx, :].cpu().numpy()
                    print(f"EE WORLD FRAME: [{ee_world[0]:.3f}, {ee_world[1]:.3f}, {ee_world[2]:.3f}]")
                if step_count % 50 == 0:
                    actual = ee_pos_curr[0].cpu().numpy()
                    print(f"Target: [{x:.3f}, {y:.3f}, {z:.3f}] | Actual: [{actual[0]:.3f}, {actual[1]:.3f}, {actual[2]:.3f}]")

            else:
                actions = torch.zeros(1, 6, device=env.device)
                ee_pos_curr = None
                joint_pos = None

            env.step(actions)
            ee_world = env.robot.data.body_pos_w[:, ee_idx, :]
            target_pos = ee_world.clone()
            target_pos[:, 0] += 0.02  # 2cm ahead of EE in X
            env.target.write_root_pose_to_sim(
                torch.cat([
                    target_pos,
                    torch.tensor([[1.0, 0.0, 0.0, 0.0]], device=env.device, dtype=torch.float)
                ], dim=-1)
            )
            contact = env.get_contact_forces()

            if contact is not None and bool(contact["hit"].any()):
                if not last_in_contact:
                    hit_counter += 1
                    force_mag = float(contact["force_mag"][0])
                    force_vec = contact["net_force"][0].cpu().numpy()
                    print(f"💥 HIT #{hit_counter} | |F|={force_mag:.2f} N | "
                        f"F=[{force_vec[0]:+.2f}, {force_vec[1]:+.2f}, {force_vec[2]:+.2f}]")
                last_in_contact = True
            else:
                last_in_contact = False 

            # Log every step
            logger.write(
                step=step_count,
                pose=pose,
                ee_pos=ee_pos_curr,
                joint_pos=joint_pos,
                contact=contact,
                hit_counter=hit_counter,
            )

    finally:
        logger.close()


if __name__ == "__main__":
    main()