# VR Teleoperated Fencing Robot 🤺

A real-time teleoperation system for controlling a robotic fencing arm using VR controllers. Built with ROS2, Unity/SteamVR, and inverse kinematics for precise sword control.

## 🎯 Project Overview

This project enables intuitive control of a robotic fencing arm through VR hand tracking, with future potential for autonomous fencing behaviors using machine learning.

**Current Status:** Teleoperation Phase  
**Timeline:** 2.5 months (10 weeks)  
**Future Goal:** Add autonomous fencing layer using behavior cloning/RL

---

## 🏗️ System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                        VR LAYER                              │
│  ┌─────────────────┐                                        │
│  │   VR Headset    │  (Visualization & immersion)           │
│  │  + Controllers  │  (Track sword hand position/rotation)  │
│  └────────┬────────┘                                        │
└───────────┼──────────────────────────────────────────────────┘
            │
            │ UDP Stream (Pose + Button States)
            │ Port: 8888
            ▼
┌─────────────────────────────────────────────────────────────┐
│                      ROS2 LAYER                              │
│                                                              │
│  ┌─────────────────┐                                        │
│  │  UDP Bridge     │  Receives VR data                      │
│  │     Node        │  Publishes: /vr/sword_pose             │
│  └────────┬────────┘                                        │
│           │                                                  │
│           ▼                                                  │
│  ┌─────────────────┐                                        │
│  │  Safety Monitor │  Workspace limits, velocity checks     │
│  │     Node        │  Emergency stop logic                  │
│  └────────┬────────┘                                        │
│           │                                                  │
│           ▼                                                  │
│  ┌─────────────────┐                                        │
│  │   IK Solver     │  MoveIt2 integration                   │
│  │   (MoveIt2)     │  Converts pose → joint angles          │
│  └────────┬────────┘                                        │
│           │                                                  │
│           ▼                                                  │
│  ┌─────────────────┐                                        │
│  │ Motor Control   │  Joint commands                        │
│  │     Node        │  Publishes: /joint_commands            │
│  └────────┬────────┘                                        │
│           │                                                  │
│  ┌───────┴────────┐                                         │
│  │  Data Logger   │  Records all topics to ROS bags         │
│  │     Node       │  For future ML training                 │
│  └────────────────┘                                         │
└───────────┼──────────────────────────────────────────────────┘
            │
            ▼
┌─────────────────────────────────────────────────────────────┐
│                    HARDWARE LAYER                            │
│  ┌─────────────────┐                                        │
│  │  Robot Arm      │  6/7-DOF manipulator                   │
│  │  + Sword Tool   │  End-effector with fencing sword       │
│  └─────────────────┘                                        │
└─────────────────────────────────────────────────────────────┘
```

---

## 📊 Data Flow Diagram

```
VR Controller Movement
        │
        ▼
   [Position, Rotation, Buttons]
        │
        ▼
    UDP Packet (Binary)
        │
        ▼
  ROS2 Bridge Node
        │
        ├──> geometry_msgs/PoseStamped → /vr/sword_pose
        │
        ▼
  Safety Monitor
        │
        ├──> Check workspace limits
        ├──> Check velocity limits
        ├──> Emergency stop if needed
        │
        ▼
  IK Solver (MoveIt2)
        │
        ├──> Compute joint angles
        ├──> Check for singularities
        │
        ▼
  Motor Control Node
        │
        ├──> trajectory_msgs/JointTrajectory
        │
        ▼
  Robot Hardware Interface
        │
        ▼
   Physical Robot Arm
```

---

## 🗂️ Project Structure

```
fencebot_ws/
├── src/
│   ├── fencebot_core/              # Main ROS2 package
│   │   ├── package.xml
│   │   ├── CMakeLists.txt
│   │   ├── include/
│   │   │   └── fencebot_core/
│   │   │       ├── udp_bridge_node.hpp
│   │   │       ├── ik_control_node.hpp
│   │   │       ├── safety_monitor_node.hpp
│   │   │       ├── motor_control_node.hpp
│   │   │       └── data_logger_node.hpp
│   │   ├── src/
│   │   │   ├── udp_bridge_node.cpp      # VR → ROS2 communication
│   │   │   ├── ik_control_node.cpp      # Inverse kinematics
│   │   │   ├── safety_monitor_node.cpp  # Safety systems
│   │   │   ├── motor_control_node.cpp   # Hardware interface
│   │   │   └── data_logger_node.cpp     # Recording for ML
│   │   ├── launch/
│   │   │   ├── teleoperation.launch.py # Main launch file
│   │   │   └── visualization.launch.py # RViz config
│   │   └── config/
│   │       ├── robot_params.yaml       # Robot configuration
│   │       ├── safety_limits.yaml      # Workspace/velocity limits
│   │       └── vr_mapping.yaml         # VR-to-robot coordinate mapping
│   │
│   ├── fencebot_description/       # Robot URDF/meshes
│   │   ├── urdf/
│   │   │   └── fencebot.urdf.xacro
│   │   ├── meshes/
│   │   └── launch/
│   │       └── display.launch.py
│   │
│   └── fencebot_moveit_config/     # MoveIt2 configuration
│       ├── config/
│       │   ├── joint_limits.yaml
│       │   ├── kinematics.yaml
│       │   └── moveit.rviz
│       └── launch/
│           └── moveit.launch.py
│
├── vr_app/                          # Unity VR application
│   ├── Assets/
│   │   ├── Scripts/
│   │   │   ├── VRController.cs          # VR tracking
│   │   │   ├── UDPSender.cs             # Network communication
│   │   │   └── RobotVisualizer.cs       # Ghost arm feedback
│   │   ├── Scenes/
│   │   │   └── FencingArena.unity
│   │   └── Prefabs/
│   └── ProjectSettings/
│
├── docs/
│   ├── setup_guide.md              # Detailed setup instructions
│   ├── calibration.md              # Calibration procedures
│   ├── safety.md                   # Safety protocols
│   └── troubleshooting.md          # Common issues
│
├── scripts/
│   ├── install_dependencies.sh     # Auto-install ROS2 packages
│   ├── calibrate_workspace.py      # Workspace calibration tool
│   └── emergency_stop.py           # Manual e-stop script
│
└── README.md
```

---

## 🛠️ Technology Stack

| Component | Technology | Purpose |
|-----------|-----------|---------|
| **VR Platform** | Unity + SteamVR | Hand tracking and visualization |
| **Middleware** | ROS2 Humble | Robot control and coordination |
| **IK Solver** | MoveIt2 | Inverse kinematics computation |
| **Communication** | UDP Sockets | Low-latency VR → ROS2 streaming |
| **Programming** | C++ 17/20 | ROS2 nodes (performance-critical) |
| **Hardware Interface** | ros2_control | Motor communication |
| **Visualization** | RViz2 | Robot state monitoring |

---

## 🚀 Quick Start

### Prerequisites

- Ubuntu 22.04 LTS
- ROS2 Humble
- VR Headset (Quest 3, Valve Index, etc.)
- Robot arm with ROS2 support

### Installation

```bash
# 1. Clone repository
git clone https://github.com/wukongxzero/THEFENCEBOT.git
cd fencebot

# 2. Install ROS2 dependencies
cd fencebot_ws
rosdep install --from-paths src -y --ignore-src

# 3. Install additional C++ libraries
sudo apt install -y \
    libeigen3-dev \
    libboost-all-dev \
    libasio-dev

# 4. Build workspace (C++ packages)
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# 5. Source workspace
source install/setup.bash

# 6. Install MoveIt2
sudo apt install ros-humble-moveit

# 7. Run installation script
./scripts/install_dependencies.sh
```

### Running the System

**Terminal 1: Launch ROS2 nodes**
```bash
source install/setup.bash
ros2 launch fencebot_core teleoperation.launch.py
```

**Terminal 2: Start visualization (optional)**
```bash
ros2 launch fencebot_core visualization.launch.py
```

**VR Headset: Launch Unity application**
- Open `vr_app` in Unity
- Build and run on your VR platform
- Ensure robot IP is configured correctly

---

## ⚙️ Configuration

### VR-to-Robot Coordinate Mapping

Edit `config/vr_mapping.yaml`:

```yaml
coordinate_transform:
  translation: [0.0, 0.0, 0.5]  # Offset between VR and robot origin
  rotation: [0.0, 0.0, 0.0]     # Euler angles (roll, pitch, yaw)
  scale: 1.0                     # Scale factor

control_sensitivity:
  position: 1.0
  rotation: 1.0
```

### Safety Limits

Edit `config/safety_limits.yaml`:

```yaml
workspace:
  x_min: -0.5
  x_max: 0.5
  y_min: -0.5
  y_max: 0.5
  z_min: 0.1
  z_max: 1.0

velocity_limits:
  max_linear: 0.5    # m/s
  max_angular: 1.0   # rad/s

emergency_stop:
  button: "trigger"  # VR controller button
  timeout: 5.0       # seconds
```

---

## 📡 ROS2 Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/vr/sword_pose` | geometry_msgs/PoseStamped | VR controller position/orientation |
| `/vr/buttons` | std_msgs/Bool[] | VR button states |
| `/joint_states` | sensor_msgs/JointState | Current robot joint angles |
| `/joint_commands` | trajectory_msgs/JointTrajectory | Target joint commands |
| `/safety/status` | std_msgs/String | Safety system status |
| `/emergency_stop` | std_msgs/Bool | Emergency stop trigger |

---

## 🎮 Controls

| VR Input | Action |
|----------|--------|
| **Grip Button** | Enable/disable robot control |
| **Trigger** | Emergency stop |
| **Joystick** | Fine position adjustment |
| **A/X Button** | Mode switching (future) |

---

## 🔧 Development Roadmap

### Phase 1: Teleoperation (Weeks 1-10) ✅ Current Phase
- [x] VR → UDP → ROS2 pipeline
- [ ] IK solver integration
- [ ] Safety systems
- [ ] User experience polish
- [ ] Data collection setup

### Phase 2: Autonomous Layer (Future)
- [ ] Behavior cloning from teleoperation data
- [ ] Simple reactive behaviors (block, parry)
- [ ] Mode switching (teleoperation ↔ autonomous)
- [ ] Reinforcement learning training

---

## 📊 Performance Metrics

**Target specifications:**
- **Latency:** <50ms (VR controller → robot movement)
- **Update Rate:** 100Hz minimum
- **Workspace:** 1m³ fencing zone
- **Safety Response:** <10ms emergency stop

**Monitoring:**
```bash
# Check topic rates
ros2 topic hz /vr/sword_pose

# Monitor latency
ros2 topic echo /diagnostics
```

---

## 🛡️ Safety

⚠️ **IMPORTANT SAFETY GUIDELINES**

1. **Always test in a clear workspace** with no people nearby
2. **Keep emergency stop accessible** at all times
3. **Start with reduced speed limits** (20% max velocity)
4. **Perform calibration** before each session
5. **Never bypass safety limits** in production code

See [docs/safety.md](docs/safety.md) for complete safety protocols.

---

## 🐛 Troubleshooting

### High Latency (>100ms)
- Check network connection (use wired ethernet)
- Verify ROS2 DDS settings
- Profile with `ros2 topic hz`

### Robot Not Responding
- Check emergency stop status
- Verify joint limits in config
- Test IK solver independently

### VR Tracking Issues
- Recalibrate VR play area
- Check coordinate transform in config
- Verify UDP packets are being sent

See [docs/troubleshooting.md](docs/troubleshooting.md) for more solutions.

---

## 📚 Resources

- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [MoveIt2 Tutorials](https://moveit.picknik.ai/main/index.html)
- [Unity SteamVR Plugin](https://valvesoftware.github.io/steamvr_unity_plugin/)
- [UDP Communication in Unity](https://wiki.unity3d.com/index.php/UDP_Communication)

---

## 🤝 Contributing

Contributions welcome! Please:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit changes (`git commit -m 'Add amazing feature'`)
4. Push to branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

---

## 📝 License

This project is licensed under the MIT License - see [LICENSE](LICENSE) file for details.

---

## 👥 Authors

- Your Name - [GitHub](https://github.com/yourusername)

---

## 🙏 Acknowledgments

- ROS2 community for excellent middleware
- MoveIt2 team for inverse kinematics solver
- Unity/SteamVR for VR framework

---

## 📧 Contact

Questions? Open an issue or contact: your.email@example.com

---

**Status:** 🚧 In Development (Phase 1: Teleoperation)  
**Last Updated:** February 2026