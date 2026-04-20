# Troubleshooting

Failure modes we've actually hit during development, with the fix that
worked. Sorted by where in the pipeline the symptom shows up.

## Startup / build

### `colcon build` fails with "Could not find a package configuration file provided by geometry_msgs"
ROS2 env not sourced.
```bash
source /opt/ros/humble/setup.bash
```
Re-run `colcon build`.

### `ros2 launch vr_robot_sim fence_bot.launch.py` → "package 'vr_robot_sim' not found"
You forgot to source the workspace overlay after building.
```bash
source install/setup.bash
```

### Isaac Lab: `ImportError: cannot import name 'AppLauncher'`
You're in the wrong Python env. Isaac Lab runs inside its own isaac-sim
Python, not the system Python.

## VR / UDP side

### `/vr_pose` is empty, `ros2 topic hz /vr_pose` shows nothing
In order of likelihood:
1. Nothing is publishing to UDP 5005. Start `mock_vr_publisher.py` or
   Unity.
2. Unity is publishing but to the wrong host. Check the target IP in the
   Unity C# script.
3. Linux firewall is dropping UDP. `sudo ufw status`; if active,
   `sudo ufw allow 5005/udp`.
4. Publisher is running on a different machine than the ROS2 bridge and
   you're sending to `127.0.0.1`. Send to the bridge machine's LAN IP.

### Mock publisher reports "~30 Hz" when you asked for 60 Hz
Python `time.sleep()` precision is limited. If the host is loaded, actual
rate can drop. For the demo this doesn't matter; the controller re-samples
at 50 Hz anyway.

### `vr_udp_publisher` receives garbage floats
Packet size mismatch. The C++ struct and the Python `struct.pack` format
must agree. Current contract is 7 × float32, little-endian, 28 bytes. If
you extend the packet (e.g., add a trigger button), update BOTH sides AND
the contact sensor docs.

## Isaac Lab side

### Arm jitters violently, never settles
Jacobian slicing is off. Originally we had:
```python
jacobian = env.robot.root_physx_view.get_jacobians()[:, ee_idx, :6, :6]
```
The first time this was debugged, the fix was `[:, ee_idx - 1, :6, :6]` —
the Jacobian is indexed by joint, not by body, and there's an off-by-one
because the base is body 0 but has no joint above it.

If you change the URDF or rename `link6`, re-verify with the debug block
that prints on `step_count == 1`:
```python
print(f"Jacobian full shape: {jac.shape}")
print(f"ee_idx: {ee_idx}")
print(f"Using jacobian index: {ee_idx - 1}")
```

### Arm slowly drifts even with a static target
Integrator error from repeatedly applying IK deltas as absolute positions.
Current `run_sim.py` uses `use_relative_mode=False` with `set_command` on
a pose and directly consumes the output — this is the fixed version.
The deprecated block is left commented out in the code as a breadcrumb.

### `EE pos at start` prints as `(0, 0, 0)` or some tiny number
Base frame vs. world frame mixup. The controller expects targets in the
robot's base frame. In `run_sim.py`:
```python
ee_pos_curr = env.robot.data.body_pos_w[:, ee_idx, :] - root_pos
```
That subtraction is what converts world → base. If you drop it, the IK
thinks the EE is already at the target and does nothing.

### `ContactSensor` throws on startup
PhysX needs collision geometry on both bodies to register contacts. Check
that the USD has a collision mesh on `link6` and the target cube has
`collision_props=sim_utils.CollisionPropertiesCfg()` (it does in
`vr_arm_env.py`).

### HIT prints fire every frame even when nothing's touching
Residual articulation force above the threshold. Either let it settle 1
second before reacting, or raise the threshold:
```python
contact = env.get_contact_forces(force_threshold=2.0)
```

## Neural network side (Jordan's)

### LNN metrics diverge to inf or NaN
Most likely causes, in order:
1. Gradient clipping is set but too loose. Try `max_norm=0.5`.
2. Learning rate too high. LNNs are notoriously fragile; `lr=1e-4` is often
   better than the default `5e-4`.
3. Mass matrix `M` going singular early in training. The code falls back
   to `pinv(M)`, but repeated falls indicate the Lagrangian is
   pathological — simplify the network depth.

### Baseline MLP trains fine, LNN is 100× worse
Three likely suspects, unrelated to model capacity:
1. `StandardScaler` applied to both inputs and targets breaks
   Euler-Lagrange — the physics equations only hold in physical units.
   Train the LNN on unscaled data.
2. `is_colliding` being fed into the LNN. A Lagrangian should be a smooth
   function of (q, q̇); a discrete 0/1 switch isn't. Either train two LNNs
   (one per regime) or model the contact as a potential term, not a flag.
3. Per-sample Python loop for the Hessian is just slow, not wrong — but
   it means you can't easily do enough epochs. Switch to
   `torch.func.vmap` once the above are fixed.

## Performance

### Isaac Lab sim drops below 30 FPS
Turn off `debug_vis` on the contact sensor (already off in
`CONTACT_SENSOR_CFG`). Close other GPU consumers. Isaac Sim eats a lot of
VRAM idle.

### ROS2 nodes leak memory
Unlikely to matter in a 2-minute demo, but `vr_udp_publisher` allocates a
`PoseStamped` per received packet. At 60 Hz for 30 min that's ~100k
allocations. Not a leak, just pressure; fine for this scope.
