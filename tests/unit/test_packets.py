"""The smoke test's synthetic packets (docker/smoke/robosense_packets.py) decode, in rs_driver
itself (tests/decode/decode_dump.cpp against the submodule), to exactly the reference points the
smoke test then expects from the running image. So a smoke failure means the image (UDP, ROS,
config), not the reference."""

from __future__ import annotations

import math
import struct
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "docker" / "smoke"))
sys.path.insert(0, str(ROOT / "tests" / "decode"))
import decode  # noqa: E402
import robosense_packets as rp  # noqa: E402


def write_packets(path: Path, packets) -> None:
    with path.open("wb") as f:
        for kind, data in packets:
            f.write(bytes([kind]) + struct.pack("<I", len(data)) + data)


def clouds(output: str):
    """[(header line, [(x, y, z, intensity, ring, ts)])] from decode_dump --points."""
    out = []
    for line in output.splitlines():
        if line.startswith("cloud "):
            out.append((line, []))
        elif line.startswith("p "):
            x, y, z, i, ring, ts = line.split()[1:]
            out[-1][1].append((float(x), float(y), float(z), float(i), int(ring), float(ts)))
    return out


def same(ref, got) -> bool:
    if math.isnan(ref[0]):
        return math.isnan(got[0]) and math.isnan(got[1]) and math.isnan(got[2]) and got[3] == 0
    return all(abs(a - b) <= 1e-4 for a, b in zip(ref[:3], got[:3])) and ref[3] == got[3]


@pytest.fixture(scope="module")
def exe(tmp_path_factory):
    if decode.compiler() is None:
        pytest.skip("needs g++")
    return decode.build(decode.SUBMODULE, tmp_path_factory.mktemp("dd"))


@pytest.mark.parametrize("model", sorted(rp.MODELS))
def test_reference_is_what_rs_driver_decodes(exe, model, tmp_path):
    start = 1700000000.25
    packets = [(1, rp.difop(model))]
    for frame in range(4):
        packets += rp.frame_packets(model, start + frame / rp.MODELS[model]["rate"])
    path = tmp_path / "packets.bin"
    write_packets(path, packets)
    got = clouds(decode.run(exe, "file", model, str(path), "0.2", "200", "--points"))
    full = [c for c in got if len(c[1]) == rp.MODELS[model]["points"]]
    assert len(full) >= 2, [c[0] for c in got]
    ref = rp.reference(model)
    for header, points in full:
        bad = [i for i, (a, b) in enumerate(zip(ref, points)) if not same(a, b)]
        assert not bad, f"{header}: {len(bad)} points differ, first {bad[0]}: {ref[bad[0]]} vs {points[bad[0]]}"
    # The stamp of a full frame is its first point (ts_first_point: true): a frame start.
    # (rs_driver stamps the very first frame 0: no first point was recorded before it.)
    stamps = sorted(float(h.split("ts=")[1].split()[0]) for h, _ in full)
    assert len([ts for ts in stamps if ts > 0]) >= 2
    for ts in [ts for ts in stamps if ts > 0]:
        k = (ts - start) * rp.MODELS[model]["rate"]
        assert abs(k - round(k)) < 1e-3, ts
    nans = sum(math.isnan(p[0]) for p in ref)
    assert 0 < nans < len(ref) // 20


def test_packet_sizes_and_ids():
    assert len(rp.helios_difop()) == 1248 and len(rp.helios_msop(0, 1.0)) == 1248
    assert len(rp.m1_difop()) == 256 and len(rp.m1_msop(1, 1.0)) == 1210
    assert rp.packet_time("RSHELIOS", rp.helios_msop(3, 1700000000.5)) == pytest.approx(1700000000.5)
    assert rp.packet_time("RSM1", rp.m1_msop(3, 1700000000.5)) == pytest.approx(1700000000.5)
