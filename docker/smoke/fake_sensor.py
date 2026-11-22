#!/usr/bin/env python3
"""A fake RoboSense sensor for the smoke test: sends DIFOP once a second to the device port and
MSOP at the sensor's real rate to the data port (robosense_packets.py: RS-Helios 1500 packets/s,
RS-LiDAR-M1 6300 packets/s), stamped with this host's clock plus --clock-offset, so a cloud
stamped by the sensor and one stamped by the host can be told apart.

Usage: fake_sensor.py --model RSHELIOS|RSM1 [--dest 127.0.0.1] [--data-port 6699]
                      [--device-port 7788] [--clock-offset SECONDS] [--seconds N]
Runs until killed (or for --seconds). Prints one line a second with the packets sent.
"""

from __future__ import annotations

import argparse
import socket
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import robosense_packets as rp  # noqa: E402

TS_OFFSET = {"RSHELIOS": 20, "RSM1": 10}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", required=True, choices=sorted(rp.MODELS))
    ap.add_argument("--dest", default="127.0.0.1")
    ap.add_argument("--data-port", type=int, default=6699)
    ap.add_argument("--device-port", type=int, default=7788)
    ap.add_argument("--clock-offset", type=float, default=0.0)
    ap.add_argument("--seconds", type=float, default=0.0)
    args = ap.parse_args()

    period = 1.0 / rp.MODELS[args.model]["rate"]
    template = [bytearray(p) for _, p in rp.frame_packets(args.model, 0.0)]
    rel = [rp.packet_time(args.model, bytes(p)) for p in template]  # offsets within the frame
    difop = rp.difop(args.model)
    at = TS_OFFSET[args.model]
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 4 << 20)
    data = (args.dest, args.data_port)
    device = (args.dest, args.device_port)

    start = time.time()
    next_frame = start
    last_difop = 0.0
    sent = 0
    last_report = start
    chunks = 10  # each frame in 10 bursts, 10 ms apart
    per_chunk = (len(template) + chunks - 1) // chunks
    while True:
        now = time.time()
        if args.seconds and now - start > args.seconds:
            return 0
        if now - last_difop >= 1.0:
            sock.sendto(difop, device)
            last_difop = now
        frame_start = next_frame + args.clock_offset
        for c in range(chunks):
            for i in range(c * per_chunk, min(len(template), (c + 1) * per_chunk)):
                pkt = template[i]
                pkt[at:at + 10] = rp._utc(frame_start + rel[i])
                sock.sendto(pkt, data)
                sent += 1
            delay = next_frame + (c + 1) * period / chunks - time.time()
            if delay > 0:
                time.sleep(delay)
        next_frame += period
        if time.time() - next_frame > 1.0:  # fell behind (slow runner): resynchronise
            next_frame = time.time()
        if time.time() - last_report >= 1.0:
            print(f"fake-sensor: {args.model} sent {sent} MSOP packets to {data}", flush=True)
            last_report = time.time()


if __name__ == "__main__":
    sys.exit(main())
