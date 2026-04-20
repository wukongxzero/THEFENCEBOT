# ROS2 launch file — brings up both FENCE-BOT nodes together.
#
# Run with:
#     ros2 launch vr_robot_sim fence_bot.launch.py
#
# This starts:
#   vr_udp_publisher  — listens on :5005 for VR packets, publishes /vr_pose
#   robot_controller  — subscribes /vr_pose, forwards to Isaac Lab on :5006
#
# Separately, you still need to launch:
#   - The Isaac Lab sim:       python3 isaac_env/run_sim.py
#   - Something producing VR:  Scripts/mock_vr_publisher.py  OR  Jordan's Unity rig

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="ROS2 log level (debug, info, warn, error)",
    )

    vr_udp_publisher = Node(
        package="vr_robot_sim",
        executable="vr_udp_publisher",
        name="vr_udp_publisher",
        output="screen",
        emulate_tty=True,
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )

    robot_controller = Node(
        package="vr_robot_sim",
        executable="robot_controller",
        name="robot_controller",
        output="screen",
        emulate_tty=True,
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )

    return LaunchDescription([
        log_level_arg,
        LogInfo(msg="Starting FENCE-BOT ROS2 bridge (UDP 5005 → /vr_pose → UDP 5006)"),
        vr_udp_publisher,
        robot_controller,
    ])
