# run_sim.py
import rclpy
import threading
import torch

from isaaclab.app import AppLauncher

# Launch Isaac Sim first — must happen before other Isaac imports
launcher = AppLauncher({"headless": False})
simulation_app = launcher.app

from vr_arm_env import VRArmEnv, VRArmEnvCfg, VRListener


def ros_spin(node):
    rclpy.spin(node)


def pose_to_joint_targets(pose, num_joints=7, device="cuda"):
    """
    Placeholder mapping: VR pose → joint targets.
    Replace this with IK later.
    """
    pos = pose.pose.position
    # For now: map XYZ to first 3 joints, rest stay at 0
    targets = torch.zeros(1, num_joints, device=device)
    targets[0, 0] = pos.x
    targets[0, 1] = pos.y
    targets[0, 2] = pos.z
    return targets


def main():
    rclpy.init()
    listener = VRListener()

    # Spin ROS2 in background thread
    ros_thread = threading.Thread(
        target=ros_spin, args=(listener,), daemon=True)
    ros_thread.start()

    # Create Isaac Lab env
    cfg = VRArmEnvCfg()
    env = VRArmEnv(cfg, ros_listener=listener)
    env.reset()

    print("Sim running — send VR UDP packets to control the arm")

    while simulation_app.is_running():
        pose = listener.get_pose()

        if pose is not None:
            actions = pose_to_joint_targets(pose, device=env.device)
        else:
            # Hold current position if no VR data
            actions = torch.zeros(1, 7, device=env.device)

        env.step(actions)

    env.close()
    rclpy.shutdown()


if __name__ == "__main__":
    main()