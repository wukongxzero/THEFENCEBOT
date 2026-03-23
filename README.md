# THEFENCEBOT 🤺

A real-time VR teleoperation system for controlling a custom 6-DOF robotic arm using VR controllers. Built with ROS2 Humble (C++), Isaac Lab, and a differential IK solver.

## 🎯 Project Overview

This project enables real-time control of the ASEM robotic arm through VR hand tracking. The VR headset streams pose data over UDP, which is received by ROS2 C++ nodes and forwarded to an Isaac Lab simulation where a differential IK solver computes joint angles for the arm.

**Current Status:** Simulation + IK Working  
**Future Goal:** Connect real VR headset, add gripper control, deploy to hardware

---

##  System Architecture

```
VR Headset
    │
    │  UDP packets (x, y, z, qx, qy, qz, qw)
    │  Port 5005
    ▼
C++ ROS2 Node: vr_udp_publisher
    │  Publishes → /vr_pose (geometry_msgs/PoseStamped)
    │  BEST_EFFORT QoS (matches UDP semantics)
    ▼
C++ ROS2 Node: robot_controller
    │  Subscribes to /vr_pose
    │  500Hz control loop
    │  Forwards pose → Port 5006
    ▼
Isaac Lab (Python 3.11)
    │  UDPListener on port 5006
    │  Differential IK solver (pinv method)
    │  ASEM robot USD asset
    ▼
Isaac Sim 5.1
    │  6-DOF ASEM arm simulated at 100Hz
    │  Physics: PhysX
    └  Viewer: RTX Real-Time
```

---

##  Project Structure

```
THEFENCEBOT/
├── src/
│   └── vr_robot_sim/               # ROS2 C++ package
│       ├── src/
│       │   ├── vr_udp_publisher.cpp  # UDP → /vr_pose topic
│       │   └── robot_controller.cpp  # Subscriber + control loop
│       ├── CMakeLists.txt
│       └── package.xml
│
├── isaac_env/                      # Isaac Lab simulation
│   ├── vr_arm_env.py               # ASEM env + robot config
│   └── run_sim.py                  # Launcher + IK controller
│
├── Simulation/
│   └── ASEM_V2.SLDASM/
│       ├── urdf/
│       │   └── ASEM_V2.SLDASM.urdf  # Robot URDF (6 revolute joints)
│       └── usd/
│           └── ASEM_V2.usd          # Converted USD for Isaac Lab
│
├── IsaacLab/                       # Isaac Lab install (v2.3.2)
├── docs/
├── scripts/
└── README.md
```

---

## 🛠️ Technology Stack

| Component | Technology |
|-----------|-----------|
| **Simulation** | Isaac Lab 2.3.2 + Isaac Sim 5.1 |
| **IK Solver** | Isaac Lab DifferentialIKController (pinv) |
| **Middleware** | ROS2 Humble |
| **ROS2 Nodes** | C++17 |
| **Communication** | UDP Sockets (port 5005 VR→ROS2, port 5006 ROS2→Isaac) |
| **Robot** | ASEM V2 — 6-DOF custom arm (j1–j6, all revolute) |
| **Physics** | PhysX via Isaac Sim |
| **Python** | 3.11 (Isaac Lab venv at /opt/isaaclab_data/.venv) |

---

## 🤖 Robot Specs (ASEM V2)

| Property | Value |
|----------|-------|
| DOF | 6 |
| Joint names | j1, j2, j3, j4, j5, j6 |
| Joint type | Revolute |
| Joint limits | -3.14 to 3.14 rad |
| End effector | link6 |
| Rest position (EE) | [0.573, -0.025, 0.582] m |
| Actuator stiffness | 100.0 |
| Actuator damping | 10.0 |

---

## 🚀 Setup

### Prerequisites

- Ubuntu 22.04
- NVIDIA GPU (8GB+ VRAM, driver 525+)
- CUDA 12.x
- Python 3.11
- ROS2 Humble

### Environment Setup

```bash
# 1. Activate Isaac Lab venv
source /opt/isaaclab_data/.venv/bin/activate

# 2. Source ROS2
source /opt/ros/humble/setup.bash

# 3. Source ROS2 workspace
cd ~/THEFENCEBOT/src/vr_robot_sim
source install/setup.bash
```

Add this alias to ~/.bashrc for convenience:
```bash
alias simenv='source /opt/ros/humble/setup.bash && source /opt/isaaclab_data/.venv/bin/activate'
```

---

## ▶️ Running the System

**Terminal 1 — UDP Publisher (VR → ROS2)**
```bash
source install/setup.bash
ros2 run vr_robot_sim vr_udp_publisher
```

**Terminal 2 — Robot Controller (ROS2 → Isaac Lab)**
```bash
source install/setup.bash
ros2 run vr_robot_sim robot_controller
```

**Terminal 3 — Isaac Lab Sim**
```bash
simenv
cd ~/THEFENCEBOT/isaac_env
~/THEFENCEBOT/IsaacLab/isaaclab.sh -p run_sim.py
```

**Terminal 4 — Test (without VR headset)**
```bash
python3 -c "
import socket, struct, time
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
while True:
    data = struct.pack('7f', 0.5, 0.0, 0.6, 0.0, 0.0, 0.0, 1.0)
    sock.sendto(data, ('127.0.0.1', 5006))
    time.sleep(0.02)
"
```

---

## 📡 ROS2 Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/vr_pose` | geometry_msgs/PoseStamped | VR controller pose (BEST_EFFORT QoS) |

---

## 📊 UDP Packet Format

Both ports use the same 7-float binary format (28 bytes):

```
struct Packet {
    float x, y, z;          // position (meters)
    float qx, qy, qz, qw;   // orientation (quaternion)
};
```

Port 5005: VR headset → vr_udp_publisher  
Port 5006: robot_controller → Isaac Lab

---

##  Current Progress

- [x] ROS2 C++ UDP publisher node
- [x] ROS2 C++ subscriber + control loop
- [x] Isaac Lab install (Python 3.11, /opt/isaaclab_data)
- [x] ASEM URDF → USD conversion
- [x] ASEM robot spawning in Isaac Sim
- [x] Differential IK solver working
- [x] End-effector tracking target positions (<1mm error on static targets)
- [ ] Forward UDP from robot_controller to port 5006
- [ ] Connect real VR headset
- [ ] Workspace scaling (VR space → robot workspace)
- [ ] Gripper control
- [ ] Deploy to real hardware

---

##  Known Issues / Notes

- ROS2 Humble uses Python 3.10, Isaac Lab uses Python 3.11 — they cannot share a Python process. Communication between them is done via UDP (port 5006) instead of rclpy.
- Isaac Lab venv must be activated before running isaaclab.sh
- Robot workspace center is ~[0.5, 0.0, 0.6] — send targets within this region for IK to converge
- IK diverges on fast-moving targets — keep target velocity smooth for teleoperation

---

##  License

MIT License

---

**Last Updated:** March 2026