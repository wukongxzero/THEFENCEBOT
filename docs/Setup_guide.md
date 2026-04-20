# FENCE-BOT Setup Guide

End-to-end instructions for bringing up the FENCE-BOT pipeline on a fresh
Ubuntu 22.04 machine.

## Pipeline overview

```
┌──────────────┐   UDP :5005   ┌───────────────────┐   /vr_pose   ┌──────────────────┐   UDP :5006   ┌────────────────┐
│  VR source   │ ───────────▶  │ vr_udp_publisher  │ ───────────▶ │ robot_controller │ ───────────▶  │  Isaac Lab sim │
│ (Unity / mock│               │   (ROS2 C++)      │  PoseStamped │   (ROS2 C++)     │               │  run_sim.py    │
│  publisher)  │               └───────────────────┘              └──────────────────┘               │  DLS IK → arm  │
└──────────────┘                                                                                       └────────────────┘
```

Four stages, three processes (the ROS2 bridge is two nodes started by one
launch file).

## Prerequisites

- Ubuntu 22.04
- ROS2 Humble installed and sourced (`source /opt/ros/humble/setup.bash`)
- NVIDIA Isaac Sim + Isaac Lab installed (separate NVIDIA installer, not
  covered by `install_dependencies.sh`)
- A CUDA-capable GPU for Isaac Lab
- Optional: Meta Quest / SteamVR-compatible headset + the Unity project in
  `vr_app/` for live teleoperation. Without hardware, use the mock publisher.

## First-time install

```bash
cd FENCE-BOT
bash Scripts/install_dependencies.sh
source install/setup.bash
```

If your ASEM USD file lives somewhere other than
`<repo>/Simulation/ASEM_V2.SLDASM/usd/ASEM_V2.usd`, export the override:

```bash
export FENCEBOT_USD_PATH=/absolute/path/to/ASEM_V2.usd
```

You probably want that in your `~/.bashrc`.

## Running the pipeline

Three terminals. Source `install/setup.bash` in each.

### Terminal 1 — ROS2 bridge

```bash
ros2 launch vr_robot_sim fence_bot.launch.py
```

You should see:
```
[vr_udp_publisher] Listening for VR data on UDP port 5005
[robot_controller] Robot controller ready, forwarding to Isaac Lab :5006
```

### Terminal 2 — Isaac Lab simulation

```bash
python3 isaac_env/run_sim.py
```

Wait for the scene to load. You should see the ASEM arm, a red target cube,
and `Sim running with IK` in the console.

### Terminal 3 — VR input (pick one)

**Mock input (no headset needed):**
```bash
# Pick a pattern: static | circle | thrust | figure8 | square | replay
python3 Scripts/mock_vr_publisher.py --pattern thrust
```

**Real VR headset:**
Launch the Unity project in `vr_app/` and start SteamVR. The Unity side is
responsible for sending the same 28-byte packet to UDP `:5005`.

## Verifying the pipeline

Signals that things are working end-to-end:

1. **ROS topic flowing.** In a fourth terminal:
   ```bash
   ros2 topic hz /vr_pose
   ```
   Should show ~60 Hz with the mock publisher.

2. **Isaac Lab log shows target tracking.** Every ~50 steps `run_sim.py`
   prints `Target: [...] | Actual: [...]`. Actual should track target within
   a few cm once the arm catches up.

3. **Contact sensor reports hits.** Aim the mock publisher at the target
   cube (default center `0.5, 0.0, 0.6`). You should see:
   ```
   💥 HIT #1 | |F|=2.34 N | F=[...]
   ```

## Shutting down

Ctrl+C each terminal. If the arm is doing something unexpected, run:

```bash
python3 Scripts/emergency_stop.py
```

from any terminal — it will flood the publisher with a hold pose. See
`docs/safety.md` for why this is a soft stop and not a replacement for a
hardware E-stop.

## Common first-run failures

| Symptom | Check |
|---|---|
| `ImportError: isaaclab` | Isaac Lab not installed or wrong conda env |
| USD fails to load | `FENCEBOT_USD_PATH` env var or repo layout |
| `ros2 launch` can't find the launch file | Did you `source install/setup.bash`? |
| `/vr_pose` silent under `ros2 topic hz` | Firewall blocking UDP 5005, or nothing publishing |
| Arm twitches but doesn't track | Jacobian index off by one — see `docs/troubleshooting.md` |
