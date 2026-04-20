#!/usr/bin/env python3
"""
emergency_stop.py
-----------------
Panic button. Sends a "freeze in place" command to the robot by flooding the
VR publisher with a static pose matching the robot's current home position,
drowning out any in-flight commands from the real VR side.

This is a user-space soft stop for the simulation. It does NOT replace a
hardware E-stop on a physical robot — see docs/safety.md.

Usage:
    python3 Scripts/emergency_stop.py                      # default 2 seconds
    python3 Scripts/emergency_stop.py --duration 5
    python3 Scripts/emergency_stop.py --pose 0.3 0.0 0.5   # custom hold pose
"""

from __future__ import annotations

import argparse
import socket
import struct
import sys
import time


DEFAULT_HOLD_POSE = (0.3, 0.0, 0.5)    # conservative, well inside workspace
IDENTITY_QUAT = (0.0, 0.0, 0.0, 1.0)   # qx, qy, qz, qw
SEND_RATE_HZ = 200                      # flood-fill to beat any active publisher


def main() -> int:
    parser = argparse.ArgumentParser(description="Soft E-stop: freeze robot at a safe pose.")
    parser.add_argument("--host", default="127.0.0.1", help="VR publisher host")
    parser.add_argument("--port", type=int, default=5005, help="VR publisher UDP port")
    parser.add_argument("--duration", type=float, default=2.0,
                        help="Seconds to hold the stop pose (default: 2.0)")
    parser.add_argument("--pose", type=float, nargs=3, default=list(DEFAULT_HOLD_POSE),
                        metavar=("X", "Y", "Z"),
                        help=f"Hold pose in base frame (default: {DEFAULT_HOLD_POSE})")
    args = parser.parse_args()

    x, y, z = args.pose
    qx, qy, qz, qw = IDENTITY_QUAT
    packet = struct.pack("<7f", x, y, z, qx, qy, qz, qw)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    addr = (args.host, args.port)
    period = 1.0 / SEND_RATE_HZ

    print(f"🛑 EMERGENCY STOP — flooding {args.host}:{args.port} "
          f"with hold pose ({x:.3f}, {y:.3f}, {z:.3f}) for {args.duration:.1f}s")

    sent = 0
    t_end = time.time() + args.duration
    try:
        while time.time() < t_end:
            sock.sendto(packet, addr)
            sent += 1
            time.sleep(period)
    except KeyboardInterrupt:
        print("\nE-stop interrupted by user (double E-stop?)")
    finally:
        sock.close()

    print(f"E-stop complete. Sent {sent} hold packets.")
    print("Reminder: also kill any running mock_vr_publisher.py or Unity VR process")
    print("          if you want the robot to stay frozen after this exits.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
