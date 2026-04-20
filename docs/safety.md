# Safety

This is a simulation-only project. The safety surface below is specific to
that. Nothing here is rated for use on a real robot without significant
additional engineering.

## What actually enforces safety right now

| Layer | Where | What it protects against |
|---|---|---|
| Joint-angle clamp | `run_sim.py` → `torch.clamp(actions, -3.14, 3.14)` | IK spitting out joint angles beyond ±π |
| IK damping (DLS) | `run_sim.py` → `ik_params={"lambda_val": 0.1}` | Divergence near kinematic singularities |
| PhysX collision model | Isaac Lab | Arm passing through the target cube |
| Soft E-stop | `Scripts/emergency_stop.py` | Flooding the publisher with a hold pose |

That's it. Everything else listed as a "safety system" in the project
proposal is aspirational.

## What is NOT implemented (be honest about this in the report)

- **Workspace boundary clamping.** `robot_controller.cpp` forwards any pose
  received, including nonsense. A VR glitch or a buggy mock publisher can
  send the arm toward targets well outside reachable space. Mitigation:
  use `Scripts/calibrate_workspace.py` to characterize the reachable box,
  then add a clamp in `controlLoop()` before the `sendto()`. TODO.
- **Velocity and acceleration limits.** The IK controller emits desired
  joint positions; there is no rate limiter between successive targets, so
  a large jump in VR input produces a proportionally large joint step.
- **Singularity avoidance based on condition number.** DLS handles the
  numerical side (Jacobian near-singular doesn't blow up), but we don't
  monitor the Jacobian condition number and refuse/slow motion near
  singularities. DLS just picks "a" solution; it doesn't pick a good one.
- **Hardware E-stop.** There is no hardware to stop. The soft E-stop script
  is a convenience — if the publisher is hung or Isaac Lab is frozen, it
  does nothing. On a real arm this layer would be a physically wired cutoff
  independent of the control software.
- **Latency watchdog.** The `<10ms emergency stop` bullet in the proposal
  is not implemented. Adding it would mean a separate process subscribed to
  `/vr_pose` that watches for staleness and publishes a zero command if the
  stream drops for more than N ms.

## If this ever moves to hardware

Before running any of this on the real ASEM arm or any other physical
robot, you need at minimum:

1. A hardware E-stop cutting power to actuators, wired outside the control
   software.
2. A real workspace enforcement layer (Cartesian + joint-angle, with speed
   limits per joint) in the firmware / motor driver layer, not only in ROS.
3. Torque / current limits enforced by the driver.
4. A watchdog that commands zero velocity if the control stream drops.
5. A human bystander cutoff: an explicit "arm is hot" indicator, verified
   on every startup.

## Demo-day failure modes to watch for

- Arm swings fast toward `(0, 0, 0)` on startup because no VR packet has
  been received yet and `run_sim.py` falls through to `torch.zeros(1, 6)`.
  Expected — send a pose as soon as possible.
- `run_sim.py` crashes after mock publisher stops (it won't; the zero-
  fallback catches it). But it does freeze the arm visibly.
- Contact sensor reports false HITs during the initial articulation
  settling — ignore the first ~0.5 s of output, or raise the threshold in
  `get_contact_forces()`.
