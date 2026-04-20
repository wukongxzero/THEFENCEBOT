#!/usr/bin/env python3
"""
Mock VR Publisher
-----------------
Sends synthetic VR controller poses to the vr_udp_publisher ROS2 node.

Use this when you want to demo / test the full FENCE-BOT pipeline
without actually wearing a VR headset. Produces several motion patterns
useful for demos, IK validation, and recording deterministic test data.

Packet format (matches vr_udp_publisher.cpp Packet struct):
    7 × float32: x, y, z, qx, qy, qz, qw   (28 bytes, little-endian)

Usage:
    python3 mock_vr_publisher.py --pattern circle
    python3 mock_vr_publisher.py --pattern thrust --host 127.0.0.1 --port 5005
    python3 mock_vr_publisher.py --pattern replay --file recorded.csv

Patterns:
    static    Hold a fixed pose (handy for IK debugging)
    circle    Circular XY sweep at fixed Z (matches the validated sim test)
    thrust    Sinusoidal reach along +X, holding Y and Z (fencing thrust)
    figure8   Figure-8 in the XZ plane
    square    Discrete waypoint square in XY
    replay    Replay a CSV of recorded poses (cols: t,x,y,z,qx,qy,qz,qw)

Stop with Ctrl+C.
"""

from __future__ import annotations

import argparse
import csv
import math
import socket
import struct
import sys
import time
from pathlib import Path


# --- pose helpers ---------------------------------------------------------

def identity_quat() -> tuple[float, float, float, float]:
    """(qx, qy, qz, qw) — no rotation."""
    return (0.0, 0.0, 0.0, 1.0)


def quat_from_axis_angle(ax: float, ay: float, az: float, angle_rad: float):
    """Build a unit quaternion from an axis (normalized) and angle."""
    half = 0.5 * angle_rad
    s = math.sin(half)
    return (ax * s, ay * s, az * s, math.cos(half))


# --- motion patterns ------------------------------------------------------
# Each generator yields (x, y, z, qx, qy, qz, qw) given elapsed seconds.

def pattern_static(center):
    def gen(t):
        qx, qy, qz, qw = identity_quat()
        return (*center, qx, qy, qz, qw)
    return gen


def pattern_circle(center, radius=0.15, period=4.0):
    cx, cy, cz = center
    def gen(t):
        phase = 2.0 * math.pi * (t / period)
        x = cx + radius * math.cos(phase)
        y = cy + radius * math.sin(phase)
        z = cz
        qx, qy, qz, qw = identity_quat()
        return (x, y, z, qx, qy, qz, qw)
    return gen


def pattern_thrust(center, reach=0.20, period=1.5):
    """Sword thrust along +X, sinusoidal advance/retreat."""
    cx, cy, cz = center
    def gen(t):
        phase = 2.0 * math.pi * (t / period)
        # 0..reach..0: use (1 - cos)/2 so it dwells briefly at extremes
        x = cx + reach * (0.5 - 0.5 * math.cos(phase))
        qx, qy, qz, qw = identity_quat()
        return (x, cy, cz, qx, qy, qz, qw)
    return gen


def pattern_figure8(center, rx=0.12, rz=0.08, period=5.0):
    cx, cy, cz = center
    def gen(t):
        phase = 2.0 * math.pi * (t / period)
        x = cx + rx * math.sin(phase)
        z = cz + rz * math.sin(2.0 * phase)
        qx, qy, qz, qw = identity_quat()
        return (x, cy, z, qx, qy, qz, qw)
    return gen


def pattern_square(center, side=0.15, dwell=1.0):
    """Four-corner waypoint square, holding each corner for `dwell` seconds."""
    cx, cy, cz = center
    half = 0.5 * side
    corners = [
        (cx + half, cy + half, cz),
        (cx - half, cy + half, cz),
        (cx - half, cy - half, cz),
        (cx + half, cy - half, cz),
    ]

    def gen(t):
        idx = int((t // dwell) % len(corners))
        x, y, z = corners[idx]
        qx, qy, qz, qw = identity_quat()
        return (x, y, z, qx, qy, qz, qw)
    return gen


def pattern_replay(csv_path: Path, loop: bool = True):
    """Replay a CSV with columns t,x,y,z,qx,qy,qz,qw (t in seconds, monotonic)."""
    rows = []
    with open(csv_path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append((
                float(row["t"]),
                float(row["x"]), float(row["y"]), float(row["z"]),
                float(row["qx"]), float(row["qy"]), float(row["qz"]), float(row["qw"]),
            ))
    if not rows:
        raise ValueError(f"replay file {csv_path} is empty")
    total = rows[-1][0]

    def gen(t):
        u = (t % total) if loop else min(t, total)
        # Nearest-neighbor lookup; fine for ~50Hz playback
        lo, hi = 0, len(rows) - 1
        while lo < hi:
            mid = (lo + hi) // 2
            if rows[mid][0] < u:
                lo = mid + 1
            else:
                hi = mid
        return rows[lo][1:]
    return gen


PATTERNS = {
    "static":  pattern_static,
    "circle":  pattern_circle,
    "thrust":  pattern_thrust,
    "figure8": pattern_figure8,
    "square":  pattern_square,
}


# --- main loop ------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Mock VR controller → UDP publisher.")
    parser.add_argument("--host", default="127.0.0.1",
                        help="Target host running vr_udp_publisher (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=5005,
                        help="Target UDP port (default: 5005)")
    parser.add_argument("--pattern", default="circle",
                        choices=list(PATTERNS.keys()) + ["replay"],
                        help="Motion pattern to generate")
    parser.add_argument("--rate", type=float, default=60.0,
                        help="Publish rate in Hz (default: 60)")
    parser.add_argument("--center", type=float, nargs=3, default=[0.5, 0.0, 0.6],
                        metavar=("X", "Y", "Z"),
                        help="Workspace center (default: 0.5 0.0 0.6 — matches target cube)")
    parser.add_argument("--file", type=Path, default=None,
                        help="CSV file for --pattern replay")
    parser.add_argument("--duration", type=float, default=0.0,
                        help="Run for this many seconds (0 = forever)")
    parser.add_argument("--verbose", action="store_true", help="Print each packet")
    args = parser.parse_args()

    if args.pattern == "replay":
        if args.file is None:
            parser.error("--pattern replay requires --file")
        gen = pattern_replay(args.file)
    else:
        gen = PATTERNS[args.pattern](tuple(args.center))

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    addr = (args.host, args.port)
    period = 1.0 / args.rate

    print(f"[mock_vr] pattern={args.pattern} → udp://{args.host}:{args.port} @ {args.rate:.0f} Hz")
    if args.pattern != "replay":
        print(f"[mock_vr] workspace center = {tuple(args.center)}")
    print("[mock_vr] Ctrl+C to stop.")

    t0 = time.time()
    next_tick = t0
    n = 0
    try:
        while True:
            t = time.time() - t0
            if args.duration and t >= args.duration:
                break

            pose = gen(t)
            pkt = struct.pack("<7f", *pose)
            sock.sendto(pkt, addr)
            n += 1

            if args.verbose and n % 30 == 0:
                x, y, z, *_ = pose
                print(f"[mock_vr] t={t:5.2f}s  pose=({x:+.3f}, {y:+.3f}, {z:+.3f})")

            next_tick += period
            sleep_for = next_tick - time.time()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                # We're behind; don't try to catch up by bursting
                next_tick = time.time()
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()
        dt = time.time() - t0
        rate = n / dt if dt > 0 else 0.0
        print(f"\n[mock_vr] sent {n} packets in {dt:.2f}s (~{rate:.1f} Hz)")


if __name__ == "__main__":
    sys.exit(main())
