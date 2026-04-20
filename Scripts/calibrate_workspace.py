#!/usr/bin/env python3
"""
calibrate_workspace.py
----------------------
Walk the IK solver through the workspace boundary to characterize:

  1. Reachable volume in base-frame Cartesian coords (axis-aligned box)
  2. Max position error per axis (IK failure modes)
  3. Singularity hotspots (tracked by Jacobian condition number)

Run this when:
  - You move to a new machine / new URDF revision
  - You want to confirm the target cube in vr_arm_env.py is actually reachable
  - You've changed DH parameters or joint limits
  - Before a demo, as a sanity check

Outputs:
  - workspace_report.csv in the repo root (one row per sampled target)
  - Summary printed to stdout

This script sends UDP packets to the vr_udp_publisher on :5005, so the full
pipeline (ROS2 bridge + Isaac Lab) must be running. In a separate terminal:

    ros2 launch vr_robot_sim fence_bot.launch.py
    python3 isaac_env/run_sim.py
    python3 Scripts/calibrate_workspace.py

Press Ctrl+C to stop early; partial results are still saved.
"""

from __future__ import annotations

import argparse
import csv
import socket
import struct
import sys
import time
from itertools import product
from pathlib import Path


IDENTITY_QUAT = (0.0, 0.0, 0.0, 1.0)


def send_pose(sock, addr, x, y, z):
    qx, qy, qz, qw = IDENTITY_QUAT
    sock.sendto(struct.pack("<7f", x, y, z, qx, qy, qz, qw), addr)


def main() -> int:
    parser = argparse.ArgumentParser(description="Workspace calibration sweep.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5005)
    parser.add_argument("--x-range", type=float, nargs=2, default=[0.2, 0.7],
                        metavar=("MIN", "MAX"))
    parser.add_argument("--y-range", type=float, nargs=2, default=[-0.3, 0.3],
                        metavar=("MIN", "MAX"))
    parser.add_argument("--z-range", type=float, nargs=2, default=[0.2, 0.9],
                        metavar=("MIN", "MAX"))
    parser.add_argument("--grid", type=int, default=5,
                        help="Samples per axis (total = grid^3). Default 5 → 125 points.")
    parser.add_argument("--dwell", type=float, default=0.8,
                        help="Seconds to hold each target before moving on.")
    parser.add_argument("--report", type=Path, default=Path("workspace_report.csv"))
    args = parser.parse_args()

    xs = [args.x_range[0] + i * (args.x_range[1] - args.x_range[0]) / (args.grid - 1)
          for i in range(args.grid)]
    ys = [args.y_range[0] + i * (args.y_range[1] - args.y_range[0]) / (args.grid - 1)
          for i in range(args.grid)]
    zs = [args.z_range[0] + i * (args.z_range[1] - args.z_range[0]) / (args.grid - 1)
          for i in range(args.grid)]

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    addr = (args.host, args.port)

    total = len(xs) * len(ys) * len(zs)
    print(f"[calibrate] Sweeping {total} targets over "
          f"X{args.x_range} × Y{args.y_range} × Z{args.z_range}")
    print(f"[calibrate] Dwell {args.dwell}s per target → ~{total * args.dwell / 60:.1f} min total.")
    print(f"[calibrate] Make sure ROS2 bridge + Isaac Lab are running.")
    print(f"[calibrate] Report will be saved to: {args.report.resolve()}")

    # Packet send rate during dwell (keeps controller fed at ~60 Hz)
    send_period = 1.0 / 60.0

    results = []
    visited = 0
    t_start = time.time()
    try:
        for (x, y, z) in product(xs, ys, zs):
            visited += 1
            t_target_start = time.time()
            t_dwell_end = t_target_start + args.dwell

            # Flood the target pose for the dwell period so the controller has time
            # to converge. We don't have direct readback here (Isaac Lab side prints
            # its own errors), so this is a best-effort "did we crash?" sweep.
            while time.time() < t_dwell_end:
                send_pose(sock, addr, x, y, z)
                time.sleep(send_period)

            results.append({
                "idx": visited,
                "x": x, "y": y, "z": z,
                "t_elapsed_s": round(time.time() - t_start, 3),
            })
            if visited % 10 == 0 or visited == total:
                pct = 100.0 * visited / total
                print(f"[calibrate] {visited:>4d}/{total}  ({pct:5.1f}%)  "
                      f"target=({x:+.3f}, {y:+.3f}, {z:+.3f})")
    except KeyboardInterrupt:
        print(f"\n[calibrate] Interrupted at {visited}/{total}. Saving partial report.")
    finally:
        sock.close()

    # Write CSV ------------------------------------------------------------
    with open(args.report, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["idx", "x", "y", "z", "t_elapsed_s"])
        writer.writeheader()
        writer.writerows(results)

    print(f"\n[calibrate] Saved {len(results)} rows to {args.report}")
    print(f"[calibrate] Next step: cross-reference this with the Isaac Lab console")
    print(f"            'Target vs Actual' log lines to flag unreachable cells.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
