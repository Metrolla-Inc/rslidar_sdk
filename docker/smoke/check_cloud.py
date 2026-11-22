#!/usr/bin/env python3
"""Subscribe to the driver's point cloud (inside the image, as the driver's uid) and check it
against the fake sensor's reference (robosense_packets.py).

Usage: check_cloud.py --topic T --frame F --model RSHELIOS|RSM1 --stamp host|sensor
                      [--clock-offset S] [--clouds N] [--timeout S]
Prints a JSON report; exit 0 when every check passed, 1 otherwise.

Checks: the topic carries sensor_msgs/PointCloud2 with frame F; fields x, y, z, intensity are
float32 at 0, 4, 8, 12 (point_step 16); after the first cloud (which may be a partial frame),
every cloud is one whole frame and every point matches the reference within 1e-4 m (NaN where
there is no return); stamps are the host's receive time (host) or the fake sensor's clock
(sensor: host time + clock offset), each frame 0.1 s after the previous.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import struct
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2

sys.path.insert(0, str(Path(__file__).resolve().parent))
import robosense_packets as rp  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", required=True)
    ap.add_argument("--frame", required=True)
    ap.add_argument("--model", required=True, choices=sorted(rp.MODELS))
    ap.add_argument("--stamp", required=True, choices=("host", "sensor"))
    ap.add_argument("--clock-offset", type=float, default=0.0)
    ap.add_argument("--clouds", type=int, default=5)
    ap.add_argument("--timeout", type=float, default=60.0)
    args = ap.parse_args()

    expected_n = rp.MODELS[args.model]["points"]
    reference = rp.reference(args.model)
    got: list[tuple[float, PointCloud2]] = []

    rclpy.init()
    node = Node("rslidar_smoke_check")
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
    node.create_subscription(PointCloud2, args.topic, lambda m: got.append((time.time(), m)), qos)
    deadline = time.time() + args.timeout
    while time.time() < deadline and len(got) < args.clouds + 1:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()

    problems: list[str] = []
    report: dict = {"topic": args.topic, "received": len(got)}
    if len(got) < 2:
        problems.append(f"received {len(got)} clouds on {args.topic} in {args.timeout:.0f} s")
    whole = got[1:]  # the first cloud may be a partial frame
    stamps = []
    max_err = 0.0
    digest = None
    for arrival, msg in whole:
        n = msg.width * msg.height
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        stamps.append(stamp)
        fields = [(f.name, f.offset, f.datatype, f.count) for f in msg.fields]
        report.setdefault("frame_id", msg.header.frame_id)
        report.setdefault("layout", {"width": msg.width, "height": msg.height, "point_step": msg.point_step,
                                     "fields": fields, "is_dense": msg.is_dense})
        if msg.header.frame_id != args.frame:
            problems.append(f"frame_id {msg.header.frame_id!r}, expected {args.frame!r}")
        if fields[:4] != [("x", 0, 7, 1), ("y", 4, 7, 1), ("z", 8, 7, 1), ("intensity", 12, 7, 1)] \
                or msg.point_step != 16:
            problems.append(f"fields {fields} point_step {msg.point_step}")
            break
        if n != expected_n:
            problems.append(f"cloud of {n} points, expected one whole frame of {expected_n}")
            continue
        data = bytes(msg.data)
        if digest is None:
            digest = hashlib.sha256(data).hexdigest()
        bad = 0
        for i, (rx, ry, rz, ri) in enumerate(reference):
            x, y, z, inten = struct.unpack_from("<4f", data, i * 16)
            if math.isnan(rx):
                ok = math.isnan(x) and math.isnan(y) and math.isnan(z) and inten == 0
            else:
                err = max(abs(x - rx), abs(y - ry), abs(z - rz))
                max_err = max(max_err, err)
                ok = err <= 1e-4 and inten == ri
            if not ok:
                if bad == 0:
                    problems.append(f"point {i}: ({x}, {y}, {z}, {inten}), expected ({rx}, {ry}, {rz}, {ri})")
                bad += 1
        if bad:
            problems.append(f"{bad} of {n} points differ from the reference")
        want = arrival + (args.clock_offset if args.stamp == "sensor" else 0.0)
        # A cloud is published right after its frame ends; its stamp is the frame's first point.
        lag = want - stamp
        report.setdefault("stamp_lag_s", []).append(round(lag, 3))
        if not -0.05 <= lag <= 1.0:
            problems.append(f"stamp {stamp:.3f} is {lag:.3f} s before {'receive time' if args.stamp == 'host' else 'the sensor clock'}")
    gaps = [round(b - a, 4) for a, b in zip(stamps, stamps[1:])]
    report["stamp_gaps_s"] = gaps
    if any(abs(g - 0.1) > 0.02 for g in gaps):
        problems.append(f"frames not 0.1 s apart: {gaps}")
    report["max_error_m"] = max_err
    report["data_sha256"] = digest
    report["problems"] = problems
    print(json.dumps(report, indent=1))
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
