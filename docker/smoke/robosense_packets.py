"""Synthetic RoboSense packets and what rs_driver must make of them.

Two models, laid out as rs_driver's decoders read them (src/rs_driver/src/rs_driver/driver/decoder):

* RS-Helios (RSHELIOS, mechanical, 32 lasers): MSOP 1248 bytes = 42-byte header (id 55 AA 05 5A,
  UTC time at 20: 6 bytes seconds + 4 bytes microseconds, big-endian) + 12 blocks (FF EE, azimuth
  in 0.01 degree, 32 x (distance x 2.5 mm, intensity)) + 6 bytes; DIFOP 1248 bytes (id A5 FF 00 5A
  11 11 55 55, rpm at 8, FOV at 32, return mode at 300, vertical and horizontal calibration at 468
  and 564: per laser sign byte + 0.01 degree). 600 rpm, 0.2 degree steps: 1800 blocks = 150
  packets = 57600 points per rotation, 1500 packets a second.
* RS-LiDAR-M1 (RSM1, solid-state; the vendor's config.yaml model, so what the legacy image
  decodes): MSOP 1210 bytes = 32-byte header (id 55 AA 5A A5, packet sequence 1-630 at 4, UTC time
  at 10) + 25 blocks (time offset us, return, 5 x (distance x 5 mm, pitch, yaw: 0.01 degree + 32768,
  intensity, 2 bytes)) + 3; DIFOP 256 bytes (return mode at 44). 630 packets = 78750 points a
  frame, 10 frames a second.

The reference decoders follow the vendor's arithmetic (float tables of sin/cos at 0.01 degree,
float lens offsets) closely enough that every coordinate agrees within 1e-4 m; tests/unit checks
them against rs_driver itself (tests/decode).
"""

from __future__ import annotations

import math
import struct

# --- float32 helpers (the decoders compute in float) -------------------------------------------


def f32(x: float) -> float:
    return struct.unpack("<f", struct.pack("<f", x))[0]


def _trig(angle: int) -> tuple[float, float]:
    """Trigon::sin/cos: float table at 0.01 degree for -90.00..449.99, 0 outside."""
    if angle < -9000 or angle >= 45000:
        angle = 0
    rad = angle * 0.01 * math.pi / 180.0
    return f32(math.sin(rad)), f32(math.cos(rad))


def _utc(ts: float) -> bytes:
    sec = int(ts)
    usec = int(round((ts - sec) * 1e6))
    if usec >= 1000000:
        sec, usec = sec + 1, usec - 1000000
    return sec.to_bytes(6, "big") + usec.to_bytes(4, "big")


def _parse_utc(raw: bytes) -> float:
    return int.from_bytes(raw[:6], "big") + int.from_bytes(raw[6:10], "big") * 1e-6


# --- RS-Helios ----------------------------------------------------------------------------------

HELIOS_MSOP_ID = bytes([0x55, 0xAA, 0x05, 0x5A])
DIFOP_ID = bytes([0xA5, 0xFF, 0x00, 0x5A, 0x11, 0x11, 0x55, 0x55])
HELIOS_LASERS = 32
HELIOS_BLOCKS = 12
HELIOS_STEP = 20  # 0.2 degree per block at 600 rpm
HELIOS_PACKETS_PER_ROUND = 36000 // HELIOS_STEP // HELIOS_BLOCKS  # 150
HELIOS_POINTS_PER_ROUND = HELIOS_PACKETS_PER_ROUND * HELIOS_BLOCKS * HELIOS_LASERS  # 57600
HELIOS_BLOCK_DURATION = 55.56e-6
HELIOS_FIRING_US = (0.00, 1.73, 3.46, 5.19, 6.92, 8.65, 10.38, 12.11, 13.84, 15.57, 17.3, 19.03,
                    20.76, 22.49, 24.22, 25.95, 27.68, 29.41, 31.14, 32.87, 34.6, 36.33, 38.06,
                    39.79, 41.52, 43.25, 44.98, 46.71, 48.44, 50.17, 51.9, 53.63)
HELIOS_RX = f32(0.03498)
HELIOS_RZ = f32(0.0)
HELIOS_DIST_RES = f32(0.0025)

# Calibration the fake sensor reports: vertical -16.00 .. +15.00 degree, one per degree; a small
# horizontal offset per laser so the horizontal correction is exercised.
HELIOS_VERT = tuple(-1600 + 100 * i for i in range(HELIOS_LASERS))
HELIOS_HORIZ = tuple(((i % 5) - 2) * 7 for i in range(HELIOS_LASERS))


def helios_difop(rpm: int = 600) -> bytes:
    pkt = bytearray(1248)
    pkt[0:8] = DIFOP_ID
    pkt[8:10] = struct.pack(">H", rpm)
    pkt[32:36] = struct.pack(">HH", 0, 36000)  # FOV: full round
    pkt[300] = 0x04  # strongest return (single)
    for i in range(HELIOS_LASERS):
        for base, value in ((468, HELIOS_VERT[i]), (564, HELIOS_HORIZ[i])):
            pkt[base + 3 * i] = 1 if value < 0 else 0
            pkt[base + 3 * i + 1:base + 3 * i + 3] = struct.pack(">H", abs(value))
    pkt[1246:1248] = b"\x0f\xf0"
    return bytes(pkt)


def helios_distance(block: int, laser: int) -> int:
    """Raw distance (2.5 mm units) of a laser in the block at azimuth index `block` (0..1799):
    a deterministic pattern between 1 m and 120 m, with every 97th point at 0 (no return, NaN) and
    every 89th at 0.125 m, inside min_distance 0.2 m (also NaN)."""
    k = block * HELIOS_LASERS + laser
    if k % 97 == 13:
        return 0
    if k % 89 == 5:
        return 50
    return 400 + (block * 37 + laser * 113) % 47600


def helios_intensity(block: int, laser: int) -> int:
    return (block * 7 + laser * 11) % 256


def helios_msop(packet: int, ts: float) -> bytes:
    """MSOP packet number `packet` (0..149) of a rotation, stamped `ts` (sensor UTC)."""
    pkt = bytearray(1248)
    pkt[0:4] = HELIOS_MSOP_ID
    pkt[20:30] = _utc(ts)
    off = 42
    for b in range(HELIOS_BLOCKS):
        block = packet * HELIOS_BLOCKS + b
        pkt[off:off + 2] = b"\xff\xee"
        pkt[off + 2:off + 4] = struct.pack(">H", block * HELIOS_STEP)
        for laser in range(HELIOS_LASERS):
            o = off + 4 + 3 * laser
            pkt[o:o + 2] = struct.pack(">H", helios_distance(block, laser))
            pkt[o + 2] = helios_intensity(block, laser)
        off += 100
    pkt[1242:1248] = b"\x00\x00\x00\x00\x00\x00"
    return bytes(pkt)


def helios_reference(min_distance: float = 0.2, max_distance: float = 200.0):
    """The points (x, y, z, intensity; NaN coordinates and intensity 0 for no return) of one full
    rotation as rs_driver publishes it: block by block from azimuth 0, laser by laser."""
    chan_azis = [f32(f32(t) / f32(55.56)) for t in HELIOS_FIRING_US]
    lo, hi = f32(min_distance), f32(max_distance)
    points = []
    for block in range(36000 // HELIOS_STEP):
        az = block * HELIOS_STEP
        sin_h, cos_h = _trig(az)
        for laser in range(HELIOS_LASERS):
            dist = f32(helios_distance(block, laser) * HELIOS_DIST_RES)
            if not (lo <= dist <= hi):
                points.append((math.nan, math.nan, math.nan, 0.0))
                continue
            horiz = az + int(f32(HELIOS_STEP * chan_azis[laser]))
            final = horiz + HELIOS_HORIZ[laser]
            sin_v, cos_v = _trig(HELIOS_VERT[laser])
            sin_hf, cos_hf = _trig(final)
            sin_hh, cos_hh = _trig(horiz)
            x = dist * cos_v * cos_hf + HELIOS_RX * cos_hh
            y = -dist * cos_v * sin_hf - HELIOS_RX * sin_hh
            z = dist * sin_v + HELIOS_RZ
            points.append((f32(x), f32(y), f32(z), float(helios_intensity(block, laser))))
    return points


# --- RS-LiDAR-M1 --------------------------------------------------------------------------------

M1_MSOP_ID = bytes([0x55, 0xAA, 0x5A, 0xA5])
M1_PACKETS = 630
M1_BLOCKS = 25
M1_CHANNELS = 5
M1_POINTS_PER_FRAME = M1_PACKETS * M1_BLOCKS * M1_CHANNELS  # 78750
M1_DIST_RES = f32(0.005)


def m1_difop() -> bytes:
    pkt = bytearray(256)
    pkt[0:8] = DIFOP_ID
    pkt[9] = 10  # frame rate
    pkt[44] = 0x04  # single return
    return bytes(pkt)


def _m1_point(seq: int, block: int, chan: int) -> tuple[int, int, int, int]:
    """distance (5 mm), pitch, yaw (0.01 degree, without the 32768 offset), intensity."""
    k = ((seq - 1) * M1_BLOCKS + block) * M1_CHANNELS + chan
    dist = 0 if k % 101 == 7 else 200 + (k * 29) % 30000
    pitch = -1200 + (chan * 600) + (block % 5) * 3
    yaw = -6000 + ((seq - 1) * 19 + block * 7) % 12000
    return dist, pitch, yaw, (k * 13) % 256


def m1_msop(seq: int, ts: float) -> bytes:
    """MSOP packet with sequence number `seq` (1..630), stamped `ts` (sensor UTC)."""
    pkt = bytearray(1210)
    pkt[0:4] = M1_MSOP_ID
    pkt[4:6] = struct.pack(">H", seq)
    pkt[6:8] = b"\x00\x01"
    pkt[8] = 0x04
    pkt[10:20] = _utc(ts)
    off = 32
    for b in range(M1_BLOCKS):
        pkt[off] = b  # time offset, microseconds
        pkt[off + 1] = 1
        for c in range(M1_CHANNELS):
            dist, pitch, yaw, inten = _m1_point(seq, b, c)
            o = off + 2 + 9 * c
            pkt[o:o + 6] = struct.pack(">HHH", dist, pitch + 32768, yaw + 32768)
            pkt[o + 6] = inten
        off += 47
    return bytes(pkt)


def m1_reference(min_distance: float = 0.2, max_distance: float = 200.0):
    lo, hi = f32(min_distance), f32(max_distance)
    points = []
    for seq in range(1, M1_PACKETS + 1):
        for b in range(M1_BLOCKS):
            for c in range(M1_CHANNELS):
                raw, pitch, yaw, inten = _m1_point(seq, b, c)
                dist = f32(raw * M1_DIST_RES)
                if not (lo <= dist <= hi):
                    points.append((math.nan, math.nan, math.nan, 0.0))
                    continue
                sp, cp = _trig(pitch)
                sy, cy = _trig(yaw)
                points.append((f32(dist * cp * cy), f32(dist * cp * sy), f32(dist * sp), float(inten)))
    return points


# --- what a model needs -------------------------------------------------------------------------

MODELS = {
    "RSHELIOS": {"points": HELIOS_POINTS_PER_ROUND, "rate": 10.0},
    "RSM1": {"points": M1_POINTS_PER_FRAME, "rate": 10.0},
}


def frame_packets(model: str, frame_start: float):
    """(kind, bytes) of one frame starting at sensor time frame_start: kind 0 MSOP, 1 DIFOP."""
    if model == "RSHELIOS":
        dt = HELIOS_BLOCK_DURATION * HELIOS_BLOCKS
        return [(0, helios_msop(p, frame_start + p * dt)) for p in range(HELIOS_PACKETS_PER_ROUND)]
    if model == "RSM1":
        dt = 0.1 / M1_PACKETS
        return [(0, m1_msop(s, frame_start + (s - 1) * dt)) for s in range(1, M1_PACKETS + 1)]
    raise ValueError(model)


def difop(model: str) -> bytes:
    return helios_difop() if model == "RSHELIOS" else m1_difop()


def reference(model: str):
    return helios_reference() if model == "RSHELIOS" else m1_reference()


def packet_time(model: str, pkt: bytes) -> float:
    return _parse_utc(pkt[20:30] if model == "RSHELIOS" else pkt[10:20])
