"""Build tests/decode/decode_dump.cpp against an rs_driver tree and run it.

Shared by the regression tests (rs_driver 1.5.18 against 1.5.20 on the same packets) and the unit
test of the smoke test's packet maker. Needs g++ and nothing else: the packets go through
rs_driver's LidarDriver with the RAW_PACKET input, no ROS, no pcap library.
"""

from __future__ import annotations

import os
import shutil
import subprocess
import tempfile
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
SOURCE = Path(__file__).resolve().parent / "decode_dump.cpp"
SUBMODULE = REPO / "src" / "rs_driver"
# The flags rslidar_sdk's CMakeLists.txt gives rs_driver (ENABLE_MODIFY_RECVBUF, ENABLE_DIFOP_PARSE),
# plus UNIT_TEST (constant parameters readable) and DISABLE_PCAP_PARSE (no libpcap needed).
FLAGS = ["-std=c++17", "-O2", "-DUNIT_TEST", "-DDISABLE_PCAP_PARSE", "-DENABLE_DIFOP_PARSE",
         "-DENABLE_MODIFY_RECVBUF"]

# Every model rs_driver 1.5.18 decodes, and the generator decode_dump has for it.
OLD_MODELS = ("RS16", "RS32", "RSBP", "RSAIRY", "RSHELIOS", "RSHELIOS_16P", "RS128", "RS80", "RS48",
              "RSP128", "RSP80", "RSP48", "RSM1", "RSM2", "RSM3", "RSE1", "RSMX", "RSM1_JUMBO")
SEEDS = (7, 1234)
DISTANCES = (("0.2", "200"), ("0", "0"))  # the vendor config's limits; 0/0 = each model's own
PACKETS = 1500


def compiler() -> str | None:
    return shutil.which(os.environ.get("CXX", "g++"))


def build(rs_driver: Path, out_dir: Path) -> Path:
    exe = out_dir / "decode_dump"
    cmd = [compiler(), *FLAGS, "-I", str(rs_driver / "src"), str(SOURCE), "-o", str(exe), "-lpthread"]
    subprocess.run(cmd, check=True, capture_output=True, text=True)
    return exe


def run(exe: Path, *args: str) -> str:
    proc = subprocess.run([str(exe), *args], check=True, capture_output=True, text=True, timeout=300)
    return proc.stdout


def model_report(exe: Path, model: str) -> str:
    """Every run of one model, without the line naming the rs_driver version."""
    parts = []
    for seed in SEEDS:
        for low, high in DISTANCES:
            out = run(exe, "synth", model, str(seed), str(PACKETS), low, high).splitlines()
            assert out[0].startswith("rs_driver "), out[0]
            parts.append(f"# {model} seed={seed} min={low} max={high}")
            parts.extend(out[1:])
    return "\n".join(parts) + "\n"


def extract(commit: str, dest: Path) -> Path:
    """The rs_driver tree at a commit (fetched into the submodule's object store if missing)."""
    if subprocess.run(["git", "-C", str(SUBMODULE), "cat-file", "-e", commit + "^{commit}"],
                      capture_output=True).returncode != 0:
        subprocess.run(["git", "-C", str(SUBMODULE), "fetch", "--quiet", "--depth=1", "origin", commit],
                       check=True)
    dest.mkdir(parents=True, exist_ok=True)
    archive = subprocess.run(["git", "-C", str(SUBMODULE), "archive", commit], check=True,
                             capture_output=True).stdout
    subprocess.run(["tar", "-x", "-C", str(dest)], input=archive, check=True)
    return dest


def tempdir() -> Path:
    return Path(tempfile.mkdtemp(prefix="rs-decode-"))
