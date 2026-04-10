Here's the updated README reflecting everything completed today:
markdown# THEFENCEBOT 🤺

A real-time teleoperation simulation system for a custom 6-DOF robotic arm. Pose commands are streamed over UDP through a ROS2 middleware layer to an Isaac Lab simulation where a differential IK solver computes and executes joint trajectories in real time.

## 🎯 Project Overview

THEFENCEBOT is a simulation-first teleoperation framework for the ASEM V2 robotic arm. The current implementation includes the Isaac Lab simulation layer and a fully functional ROS2 C++ middleware pipeline: receiving 6-DOF pose targets over UDP, routing through ROS2 nodes, solving inverse kinematics using a damped least squares controller, and driving the arm to track the target in real time.

The longer-term goal is to close the loop with a real VR headset and deploy to physical hardware.

**Current Status:** Isaac Lab simulation + IK tracking working. ROS2 middleware pipeline complete and validated. Target rigid body spawning in sim.

---

## System Architecture

### Implemented (Working)
VR Emulator / Fake VR Script
│
│  UDP packets (x, y, z, qx, qy, qz, qw) — 28 bytes
│  Port 5005
▼
ROS2 C++ Node: vr_udp_publisher
│  Publishes geometry_msgs/PoseStamped
│  Topic: /vr_pose (QoS: best_effort)
▼
ROS2 C++ Node: robot_controller
│  Subscribes to /vr_pose
│  Applies workspace scaling
│  Forwards 28-byte UDP packet
│  Port 5006
▼
Isaac Lab (Python 3.11)
│  UDPListener thread (non-blocking)
│  Differential IK solver (DLS, λ=0.1)
│  Joint position targets → PhysX
│  Red target cube at [0.5, 0.0, 0.6]
▼
Isaac Sim 5.1
│  6-DOF ASEM arm at 100Hz
└  RTX Real-Time viewer

### Planned (Not Yet Implemented)
VR Headset (Quest 3 / Valve Index)
│  UDP Port 5005
▼
ROS2 C++ Node: vr_udp_publisher  (existing)
▼
... (rest of pipeline same as above)

---

## Project Structure
THEFENCEBOT/
├── isaac_env/
│   ├── vr_arm_env.py        # ASEM DirectRLEnv + actuator config + target rigid body
│   └── run_sim.py           # UDP listener + IK controller loop
│
├── src/
│   └── vr_robot_sim/        # ROS2 C++ package
│       ├── src/
│       │   ├── vr_udp_publisher.cpp   # UDP 5005 → /vr_pose topic
│       │   └── robot_controller.cpp   # /vr_pose → UDP 5006
│       ├── CMakeLists.txt
│       └── package.xml
│
├── Simulation/
│   └── ASEM_V2.SLDASM/
│       ├── urdf/
│       │   └── ASEM_V2.SLDASM.urdf
│       └── usd/
│           └── ASEM_V2.usd
│
├── Math/
│   └── 6dof_math_mode.ipynb
│
├── IsaacLab/
└── README.md

---

## 🛠️ Technology Stack

| Component | Technology |
|-----------|------------|
| Simulation | Isaac Lab 2.3.2 + Isaac Sim 5.1 |
| IK Solver | DifferentialIKController — damped least squares |
| Physics | PhysX (via Isaac Sim) |
| Middleware | ROS2 Humble, C++17 |
| Communication | UDP sockets (28-byte float packets) |
| Robot | ASEM V2 — 6-DOF custom arm |
| Python | 3.11 (Isaac Lab venv) |

---

## 🤖 Robot Specs (ASEM V2)

| Property | Value |
|----------|-------|
| DOF | 6 |
| Joint names | j1, j2, j3, j4, j5, j6 |
| Joint type | Revolute |
| Joint limits | ±π rad |
| End effector | link6 |
| Rest position (EE) | [0.573, -0.025, 0.582] m |
| Actuator stiffness | 800.0 |
| Actuator damping | 40.0 |

---

## IK Solver

The IK uses Isaac Lab's `DifferentialIKController` configured as follows:

```python
DifferentialIKControllerCfg(
    command_type="pose",
    use_relative_mode=False,
    ik_method="dls",
    ik_params={"lambda_val": 0.1}
)
```

**Why DLS:** The ASEM arm encounters kinematic singularities at certain configurations. Damped least squares regularizes the solution, trading ~0.02–0.05m error near singularities for stability.

**Jacobian:** `get_jacobians()[:, ee_idx-1, :6, :6]` — Jacobian excludes base body, ee_idx offset by 1.

**Frame:** EE pose in robot root frame (`body_pos_w - root_pos_w`).

---

## 🚀 Setup

### Prerequisites

- Ubuntu 22.04
- NVIDIA GPU (8GB+ VRAM, driver 525+)
- CUDA 12.x
- Python 3.11
- Isaac Lab 2.3.2
- ROS2 Humble

### Build ROS2 Package

```bash
cd ~/THEFENCEBOT
colcon build --packages-select vr_robot_sim
source install/setup.bash
```

---

## ▶️ Running the Full Pipeline

**Terminal 1 — Isaac Lab Sim**
```bash
source /opt/isaaclab_data/.venv/bin/activate
cd ~/THEFENCEBOT/isaac_env
~/THEFENCEBOT/IsaacLab/isaaclab.sh -p run_sim.py
```

**Terminal 2 — ROS2 Nodes**
```bash
source ~/THEFENCEBOT/install/setup.bash
ros2 run vr_robot_sim vr_udp_publisher &
ros2 run vr_robot_sim robot_controller
```

**Terminal 3 — Fake VR Input (circle sweep)**
```bash
python3 -c "
import socket, struct, time, math
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
t = 0.0
while True:
    x = 0.5 + 0.1 * math.cos(t)
    y = 0.1 * math.sin(t)
    z = 0.6
    data = struct.pack('7f', x, y, z, 0.0, 0.0, 0.0, 1.0)
    sock.sendto(data, ('127.0.0.1', 5005))
    t += 0.005
    time.sleep(0.02)
"
```

---

## 📊 UDP Packet Format
[ x | y | z | qx | qy | qz | qw ]
4   4   4    4    4    4    4    = 28 bytes (7 × float32)

Position in meters. Quaternion `(qx, qy, qz, qw)` — reordered to `(qw, qx, qy, qz)` internally before passing to IK controller.

---

## Current Progress

- [x] ASEM URDF → USD conversion
- [x] ASEM arm spawning in Isaac Sim
- [x] UDP pose listener (threaded, non-blocking)
- [x] Differential IK solver — full pose (position + orientation)
- [x] DLS singularity handling
- [x] Real-time EE tracking via circle sweep test
- [x] ROS2 C++ vr_udp_publisher node
- [x] ROS2 C++ robot_controller node (forwards to Isaac Lab port 5006)
- [x] Full ROS2 ↔ Isaac Lab UDP bridge validated
- [x] Target rigid body (red cube) spawning in sim at [0.5, 0.0, 0.6]
- [ ] Contact sensor — in progress (activate_contact_sensors enabled, attribute name TBD)
- [ ] Data logging pipeline
- [ ] VR headset integration (Phase 2 — Ritesh)
- [ ] Workspace scaling (VR space → robot workspace)
- [ ] Safety monitor + E-stop
- [ ] Behavior cloning / LNN baseline (jordan)

---

## Known Issues / Notes

- ROS2 Humble (Python 3.10) and Isaac Lab (Python 3.11) cannot share a process — middleware communicates via UDP only
- Target positions should stay within ~0.1–0.15m of [0.5, 0.0, 0.6] for reliable IK convergence
- DLS tracking error near singularities is expected behavior, not a bug
- `ContactSensorData.in_contact` attribute name differs in Isaac Lab 2.3.2 — use `print(dir(contact_sensor.data))` to verify correct attribute name before enabling

---

## License

MIT License

---

**Last Updated:** April 2026