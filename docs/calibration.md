# FENCE-BOT 🤺

**A simulation-first teleoperation system for a custom 6-DOF robotic arm,** with a physics-informed learned dynamics model as a parallel research track. Built in Isaac Lab, connected to the outside world through a ROS2 middleware layer, instrumented to log contact events with the end-effector in real time.

Term project for *Advanced Robotics* at NYU Tandon (Spring 2026), Prof. Joo H. Kim.

---

## Table of Contents

- [The engineering problem](#the-engineering-problem)
- [Why simulation-first?](#why-simulation-first)
- [System Overview](#system-overview)
- [Architecture](#architecture)
- [Quick Start](#quick-start)
- [Repository Layout](#repository-layout)
- [Subsystem 1 — Real-Time Teleoperation Pipeline](#subsystem-1--real-time-teleoperation-pipeline)
- [Subsystem 2 — Learned Dynamics (LNN)](#subsystem-2--learned-dynamics-lnn)
- [Robot: ASEM V2](#robot-asem-v2)
- [Technology Stack](#technology-stack)
- [Development Status](#development-status)
- [Known Limitations](#known-limitations)
- [Team](#team)

---

## The engineering problem

Teleoperating a 6-DOF arm at fencing speeds compresses three distinct engineering problems into one tractable target:

1. **Real-time inverse kinematics** that stays numerically stable across a working volume — including close to kinematic singularities where naïve pseudoinverse solutions blow up.
2. **Low-latency, rate-decoupled middleware** between a VR pose source and a physics simulator, where the two processes can't share an interpreter and the VR side jitters.
3. **Instrumented contact detection** that distinguishes genuine impact events from numerical noise in the physics solver, cleanly enough to log per-hit without console spam.

Fencing is the domain, but the hard parts of the engineering are general: IK stability, clean middleware boundaries, and instrumentation discipline.

---

## Why simulation-first?

Everything in this repo runs in Isaac Lab against a CAD-modeled arm that does not (yet) exist as hardware. That's deliberate:

- **Iteration time.** A bad IK tuning costs seconds to discover in sim, not a weekend of rebuilding a driver board on physical hardware.
- **Safety.** The system can command unreachable poses, singular configurations, and high-speed impact motion without risking hardware or the person in the room. Software-level guards are documented in `docs/safety.md` — none of this is rated for a real robot, and the README is explicit about what's missing before it would be.
- **Reproducibility.** Any reviewer with Isaac Sim can run the demo end-to-end in under 15 minutes. Hardware demos don't have that property.
- **Parallel research.** The learned-dynamics track (Subsystem 2) requires ground-truth data from a known physical model. Sim is where that data comes from.

The pipeline is architected so that swapping the synthetic VR source for a real headset, or the Isaac Lab sim for a physical arm, changes one process each. Both transitions are future work; the interface shape is already correct.

---

## System Overview

FENCE-BOT is two loosely-coupled subsystems sharing one simulation domain:

**1. The live teleoperation pipeline** — a VR controller (real or synthetic) streams 6-DOF pose targets through a ROS2 middleware layer into Isaac Lab, where a damped least squares IK solver drives the 6-DOF ASEM arm. Contact between the end-effector and a target body is instrumented with a force-threshold sensor, logging "hit" events with force magnitudes.

**2. The offline learned dynamics model** — a symbolically-derived 2D rigid-body + pendulum collision simulator generates training data; a Lagrangian Neural Network learns the system's Lagrangian `L(q, q̇)` directly, with accelerations recovered through autograd-computed Euler-Lagrange:

```
M q̈ + C q̇ = ∂L/∂q
```

These are independent deliverables today. The integration path — replacing Isaac Lab's analytical contact model with the learned one, which would give a differentiable physics layer — is out of scope for this semester but the scaffolding is in place.

---

## Architecture

### Live teleoperation pipeline (implemented)

```
┌─────────────────┐              ┌──────────────────┐              ┌──────────────────┐              ┌──────────────────┐
│   VR source     │  UDP :5005   │ vr_udp_publisher │   /vr_pose   │ robot_controller │  UDP :5006   │   run_sim.py     │
│                 │  ──────────▶ │   (ROS2 C++)     │  ──────────▶ │   (ROS2 C++)     │  ──────────▶ │  (Isaac Lab)     │
│ • Mock VR (code)│  28 B packet │                  │  PoseStamped │                  │  28 B packet │                  │
│ • Real VR (TBD) │  (x,y,z +    │  UDP listener,   │  best-effort │  Subscriber +    │  (identical  │  UDP listener,   │
│                 │   quaternion)│  publisher       │  QoS         │  50 Hz forwarder │   format)    │  DLS IK,         │
│                 │              │                  │              │                  │              │  contact sensor, │
│                 │              │                  │              │                  │              │  hit counter     │
└─────────────────┘              └──────────────────┘              └──────────────────┘              └──────────────────┘
```

### Offline LNN pipeline (implemented)

```
┌───────────────────────────────────────┐       ┌──────────────────────────────┐       ┌──────────────────────────────┐
│ rigid_body_hits_pendulum.ipynb        │       │ pinn_training_dataset.csv    │       │ train_lnn_fixed.py           │
│                                       │       │                              │       │                              │
│ SymPy Lagrangian derivation →         │ ───▶  │ 711 rows of (q, q̇, q̈,      │ ───▶  │ Split by regime →            │
│ Euler-Lagrange eqns →                 │       │ is_colliding, params)        │       │ two LagrangianNets trained   │
│ scipy solve_ivp with event-triggered  │       │                              │       │ independently via            │
│ collision switching                   │       │                              │       │ torch.func + vmap            │
└───────────────────────────────────────┘       └──────────────────────────────┘       └──────────────────────────────┘
```

---

## Quick Start

```bash
# 1. Clone
git clone https://github.com/wukongxzero/FENCE-BOT.git && cd FENCE-BOT

# 2. One-shot install (apt + rosdep + pip + colcon build)
bash Scripts/install_dependencies.sh

# 3. Source ROS2 + the workspace overlay
source /opt/ros/humble/setup.bash
source install/setup.bash

# 4. Run three terminals:
# Terminal 1 — ROS2 bridge
ros2 launch vr_robot_sim fence_bot.launch.py

# Terminal 2 — Isaac Lab simulation
python3 isaac_env/run_sim.py

# Terminal 3 — synthetic VR input
python3 Scripts/mock_vr_publisher.py --pattern thrust
```

Expect the arm to begin tracking a sinusoidal thrust motion. When the end-effector reaches the red target cube at `(0.5, 0.0, 0.6)`, the sim console prints:

```
💥 HIT #1 | |F|=2.34 N | F=[+2.34, -0.01, +0.02]
```

For a detailed walkthrough, see **[docs/Setup_guide.md](docs/Setup_guide.md)**. For common failure modes, see **[docs/troubleshooting.md](docs/troubleshooting.md)**.

---

## Repository Layout

```
FENCE-BOT/
├── isaac_env/                        # Isaac Lab simulation layer
│   ├── vr_arm_env.py                 # ASEM DirectRLEnv, target cube, contact sensor
│   └── run_sim.py                    # Main control loop: UDP listen → IK → step → hit detect
│
├── src/vr_robot_sim/                 # ROS2 C++ package
│   ├── src/
│   │   ├── vr_udp_publisher.cpp      # UDP :5005 → /vr_pose topic
│   │   └── robot_controller.cpp      # /vr_pose → UDP :5006 @ 50 Hz
│   ├── launch/
│   │   └── fence_bot.launch.py       # One-command bringup of both nodes
│   ├── CMakeLists.txt
│   └── package.xml
│
├── Scripts/                          # Standalone tools
│   ├── mock_vr_publisher.py          # Fake VR source (6 motion patterns + CSV replay)
│   ├── emergency_stop.py             # Soft E-stop: floods publisher with hold pose
│   ├── calibrate_workspace.py        # 3D grid sweep → workspace_report.csv
│   └── install_dependencies.sh       # One-shot installer
│
├── NeuralNetwork/                    # Offline learned dynamics
│   ├── rigid_body_hits_pendulum.ipynb  # SymPy + solve_ivp data generator
│   ├── train_lnn_fixed.py            # Fixed LNN trainer (see LNN_FIX_NOTES.md)
│   ├── LNN_FIX_NOTES.md              # Bug analysis + empirical results
│   ├── lnn_freeflight.pth            # Trained LNN weights (free flight regime)
│   ├── lnn_contact.pth               # Trained LNN weights (contact regime)
│   ├── lnn_training_curves.png       # Training / test loss curves
│   ├── baseline_nn.pth               # Vanilla MLP baseline
│   └── pytorchInit.ipynb             # Early double-pendulum experiments
│
├── Math/
│   ├── 6dof_math_mode.ipynb          # Symbolic FK/DH derivation for ASEM
│   ├── DH_calculator.py
│   ├── urdf2dhparam.py               # Auto-extract DH from URDF
│   └── link.py
│
├── Simulation/                       # ASEM CAD + converted meshes
│   └── ASEM_V2.SLDASM/
│       ├── urdf/ASEM_V2.SLDASM.urdf
│       └── usd/ASEM_V2.usd
│
├── Resources/                        # DH diagrams, alternate URDFs
│
├── docs/                             # Setup, calibration, safety, debugging
│   ├── Setup_guide.md
│   ├── calibration.md
│   ├── safety.md
│   └── troubleshooting.md
│
├── vr_app/                           # Unity/SteamVR project (Phase 2)
│
└── Project_Proposal.md
```

---

## Subsystem 1 — Real-Time Teleoperation Pipeline

### Data flow

A VR source (real or mock) sends 28-byte UDP packets to `127.0.0.1:5005` at 60 Hz:

```
[x, y, z, qx, qy, qz, qw]     7 × float32, little-endian, base frame (meters, unit quat)
```

`vr_udp_publisher` (C++) unpacks these into `geometry_msgs/PoseStamped` and publishes on `/vr_pose` with best-effort QoS. `robot_controller` (C++) subscribes to the topic and — on a deterministic 20 ms timer, decoupled from the ROS callback rate — forwards the latest pose to `127.0.0.1:5006` as the same 28-byte struct. The deliberate rate decoupling absorbs jitter from the VR side and guarantees Isaac Lab sees a predictable 50 Hz stream.

### Isaac Lab side

`run_sim.py` launches Isaac Sim in windowed mode, spawns the ASEM arm + target cube + ground plane + distant light, attaches a contact sensor to `link6`, and enters the control loop:

1. Poll UDP `:5006` for the latest target pose (non-blocking, background thread)
2. Read current EE state: `body_pos_w[ee_idx]`, `body_quat_w[ee_idx]`, `root_pos_w` (for world→base)
3. Fetch the 6×6 Jacobian slice: `get_jacobians()[:, ee_idx-1, :6, :6]`
   *(ee_idx is offset by 1 because the base body has no actuating joint above it — a common Isaac Lab gotcha)*
4. Compute IK via `DifferentialIKController` with DLS:

   ```python
   DifferentialIKControllerCfg(
       command_type="pose",
       use_relative_mode=False,
       ik_method="dls",
       ik_params={"lambda_val": 0.1},
   )
   ```

5. Clamp output joint angles to `[-π, π]`
6. Step PhysX (`env.step(actions)`)
7. Poll contact sensor; on rising-edge of any contact exceeding 0.5 N, log a hit

### Why DLS (damped least squares) over pseudoinverse?

The ASEM V2 arm hits kinematic singularities in two easily-reached configurations: wrist-aligned (j5 ≈ 0) and elbow-extended (j3 ≈ ±π). Standard pseudoinverse IK produces joint velocities that diverge as the Jacobian becomes rank-deficient. DLS adds a damping term λI to the normal equations, trading off ~2–5 cm of tracking error near singularities for numerical stability. `λ=0.1` was chosen empirically from circle-sweep validation — lower values oscillated near singularities, higher values over-damped and lagged visibly.

### Mock VR source

For environments without a headset (grading, CI, headless demo, quick testing), `Scripts/mock_vr_publisher.py` emulates the VR side with six motion patterns:

| Pattern | Use case |
|---|---|
| `static`  | IK convergence validation at a fixed pose |
| `circle`  | XY sweep at fixed Z — the original pipeline validation test |
| `thrust`  | Sinusoidal +X advance/retreat — simulates a fencing thrust attack |
| `figure8` | XZ figure-8 — tests coupled axis motion |
| `square`  | Discrete waypoint transitions — tests transient response |
| `replay`  | CSV playback of recorded VR poses — deterministic repro |

The mock publisher is packet-compatible with the planned Unity/SteamVR rig; swapping from one to the other is a matter of starting a different Terminal 3 process.

### Contact detection

`vr_arm_env.py` attaches an Isaac Lab `ContactSensor` to `link6`. The wrapper `env.get_contact_forces(force_threshold=0.5)` returns per-environment booleans, net force vectors in world frame, force magnitudes, and contact history. `run_sim.py` uses rising-edge detection so hits print once per contact event rather than per simulation step — critical for readable logs during a live demo.

### Session logging

`run_sim.py` accepts an optional `--log` flag that writes a per-step CSV for the duration of the session. The file lands in `isaac_env/logs/session_<timestamp>.csv` by default; pass `--log <path>` to override.

```bash
python3 isaac_env/run_sim.py --log
```

**Columns:** `t, target_x, target_y, target_z, actual_x, actual_y, actual_z, error, in_contact, force_mag`

One row per simulation step when a VR pose is being received. Format is stable — downstream consumers can rely on the column order and naming. Generate a tracking-error plot + numeric summary from a session with:

```bash
python3 Scripts/plot_tracking.py isaac_env/logs/session_<timestamp>.csv
```

**Open question for downstream use:** the logged data is currently used only for sanity-checking IK tracking. Whether it can feed the LNN training pipeline (Subsystem 2) is an open design question — the two systems operate in different state spaces (3D Cartesian EE pose here vs. 2D generalized coordinates in the LNN domain) and a projection/reformulation would be needed before the data is directly usable as LNN training input.

---

## Subsystem 2 — Learned Dynamics (LNN)

### Motivation

Analytical contact dynamics — spring impulses, event-triggered ODE switching — work, but they don't generalize to geometries and regimes the derivation didn't anticipate. A data-driven dynamics model that takes state `(q, q̇)` and predicts acceleration `q̈` can, in principle, learn whatever the data contains. A vanilla MLP fits this end-to-end but throws away structural knowledge about physics (energy conservation, mass matrix positive-definiteness, coordinate-free formulation).

A Lagrangian Neural Network parameterizes a scalar `L(q, q̇)` and recovers dynamics through Euler-Lagrange:

```
M q̈ + C q̇ = ∂L/∂q
    where  M = ∂²L/∂q̇²      (mass matrix)
           C = ∂²L/(∂q̇ ∂q)  (Coriolis)
```

Autograd handles the differentiation; `torch.linalg.solve(M, rhs)` handles the inversion. The physics prior means the model can only represent *physically realizable* dynamics, regardless of how weird the data gets.

### Domain

A 2D rigid body (`x`, `y`, `φ`) colliding with a pendulum (`θ`). Collision is modeled analytically as a linear spring between the rigid body position and the pendulum tip, active only when they cross. Data generation in the notebook uses `scipy.integrate.solve_ivp` with event-triggered switching between `no_collision` and `collision` dynamics functions.

### Fixing the original implementation

The first LNN implementation (cell 16 of the notebook) had three latent bugs that were traced down while writing `train_lnn_fixed.py`. See **[NeuralNetwork/LNN_FIX_NOTES.md](NeuralNetwork/LNN_FIX_NOTES.md)** for the full writeup. Summary:

1. **StandardScaler applied independently to `q`, `q̇`, `q̈`** broke dimensional consistency of Euler-Lagrange. Fix: train in physical units, log-scale the loss plot.
2. **`is_colliding` as a discrete LNN input** corrupted the Hessian — a Lagrangian must be smooth in its inputs; a 0/1 switch produces undefined second derivatives. Fix: split the dataset and train two regime-specific LNNs.
3. **Per-sample Python loop over Jacobian/Hessian** was slow enough that training stopped at 50 epochs (before convergence). Fix: `torch.func.hessian` + `vmap` → ~10× speedup → can actually run 200+ epochs.

### Empirical results

30-epoch training run on the 711-sample dataset:

| Regime | Samples | Epoch 1 MSE | Epoch 30 MSE | Status |
|---|---|---|---|---|
| Free flight | 683 | 9.1×10⁵ | **1.12 train / 0.71 test** | Converges cleanly — 6 orders of magnitude |
| Contact     | 28  | 2.7×10⁴ | 370 train / 848 test           | Underfit — data-starved |

![Training curves](NeuralNetwork/lnn_training_curves.png)

The contact-regime plateau is a **data problem, not a code problem.** 28 collision samples is not enough to constrain any dynamics model regardless of architecture. The path forward is more data generation (varying initial conditions, external forces, spring stiffness) — straightforward but out of scope for this semester. This is an honest result and appears in the report as such.

The free-flight LNN **outperforms the vanilla MLP baseline** (cell 15 of the notebook) on held-out data by a meaningful margin, validating that the physics-informed prior helps when the dataset fairly covers the regime.

---

## Robot: ASEM V2

The ASEM V2 is a custom 6-DOF serial manipulator, CAD-modeled in SolidWorks and exported to URDF → USD for Isaac Lab.

| Property | Value |
|---|---|
| DOF | 6, all revolute |
| Joint names | `j1, j2, j3, j4, j5, j6` |
| Joint limits | ±π rad (soft-clamped in `run_sim.py`) |
| End effector | `link6` |
| Rest EE position (base frame) | `[0.573, -0.025, 0.582]` m |
| Workspace center used for targets | `(0.5, 0.0, 0.6)` |
| Reliable tracking radius | ~0.1–0.15 m from workspace center |
| Actuator stiffness / damping | 800.0 / 40.0 (implicit actuator model) |

DH parameters were derived symbolically in `Math/6dof_math_mode.ipynb` and validated against the URDF via `Math/urdf2dhparam.py`.

---

## Technology Stack

| Layer | Technology |
|---|---|
| Simulation | Isaac Lab 2.3.2 + Isaac Sim 5.1 |
| Physics | PhysX 5 (via Isaac Sim) |
| IK | `isaaclab.controllers.DifferentialIKController` (DLS) |
| Contact sensing | `isaaclab.sensors.ContactSensor` |
| Middleware | ROS2 Humble, C++17 |
| Transport | UDP (binary, 28-byte packets) |
| Learned dynamics | PyTorch 2.x, `torch.func` (vmap + hessian + jacrev) |
| Symbolic derivation | SymPy |
| ODE integration | `scipy.integrate.solve_ivp` with event detection |
| CAD → sim | SolidWorks → URDF → USD |

### Why the Python 3.10/3.11 UDP bridge instead of native ROS2 Python bindings?

ROS2 Humble ships with Python 3.10 bindings. Isaac Lab 2.3.2 ships with Python 3.11. `rclpy` from the Humble apt packages cannot be imported from the Isaac Lab venv, and rebuilding `rclpy` against 3.11 is a yak-shave not worth doing for a simulation project. UDP between two processes is the pragmatic solution: one binary protocol, two language stacks, zero coupling.

---

## Development Status

### Done

- [x] URDF → USD conversion for ASEM V2
- [x] Isaac Lab `DirectRLEnv` wrapping the arm, target cube, and contact sensor
- [x] UDP listener with non-blocking background thread
- [x] Differential IK (DLS) with full pose commands
- [x] Jacobian indexing verified via debug prints
- [x] World-frame → base-frame EE pose conversion
- [x] Circle sweep validation test
- [x] ROS2 C++ `vr_udp_publisher` node
- [x] ROS2 C++ `robot_controller` node with rate-decoupled forwarding
- [x] One-command bringup via `ros2 launch`
- [x] Target rigid body spawning with collision properties
- [x] Contact sensor wired into scene, force-threshold hit detection
- [x] Rising-edge hit counter for clean demo output
- [x] Mock VR publisher (6 patterns + CSV replay)
- [x] Soft E-stop, workspace calibration sweep, installer script
- [x] `FENCEBOT_USD_PATH` env var (portability fix)
- [x] LNN training (free-flight regime converges, contact regime data-starved)
- [x] Full setup, calibration, safety, and troubleshooting docs

### In progress / Phase 2

- [ ] Unity/SteamVR project wired to UDP :5005 (Jordan, Phase 2)
- [ ] Workspace clamping in `robot_controller.cpp` (safety gap)
- [ ] Data logging pipeline: record `(VR pose, joint state, contact force)` tuples to CSV
- [ ] Generate more collision data to fix LNN contact-regime underfitting

### Future work

- Velocity/acceleration rate limiting between successive IK commands
- Singularity-aware motion planning (condition-number monitoring + soft reject)
- Replace analytical collision with the learned LNN inside Jordan's simulator
- Differentiable contact dynamics via the LNN inside Isaac Lab
- Physical deployment on the real ASEM hardware

---

## Known Limitations

Honest section, worth reading before running anything in contact with real hardware.

- **Simulation-only.** Every safety guarantee discussed in `docs/safety.md` is a software-level guard in a sim. None of this is rated for use on a physical robot without significant additional engineering.
- **No workspace clamping.** `robot_controller.cpp` forwards any pose received, including nonsense. A glitched mock publisher or a runaway Unity update could command unreachable targets. Mitigation path documented in `docs/safety.md`.
- **No latency watchdog.** The `<10ms emergency stop` bullet from the original proposal was not implemented. The soft E-stop script is a convenience — it does nothing if the publisher or Isaac Lab is hung.
- **DLS over singularities is stable, not optimal.** DLS picks "a" solution near singularities; it doesn't pick a *good* one. For aggressive fencing motions this would matter; for the current demo it doesn't.
- **Contact LNN underfit.** 28 training samples, overfits. Framed honestly in the report as a data-quantity finding.
- **Reliable IK tracking only within ~0.1–0.15 m of `(0.5, 0.0, 0.6)`.** Outside that band, expect degraded convergence or occasional DLS oscillation.
- **Isaac Lab and ROS2 Humble cannot share a process** (Python version mismatch). UDP is the only bridge. Not changing this.

---

## Team

- **Pavan Kushal Velagaleti** — System architect; Isaac Lab environment; ROS2 middleware; IK + contact integration; tooling and documentation; LNN fix.
- **Jordan Irgang** — Symbolic Lagrangian derivation; 2D rigid-body + pendulum analytical simulator; event-triggered collision detection; initial Lagrangian Neural Network.

Course: *Advanced Robotics*, Prof. Joo H. Kim, NYU Tandon School of Engineering.

---

## License

MIT

---

**Last updated:** April 2026