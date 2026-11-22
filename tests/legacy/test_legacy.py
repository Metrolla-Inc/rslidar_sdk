"""Legacy mode stays exactly the released v1.5.18 image's behaviour.

No fleet pins this image today (balena-fleets, balena-fleets-metrollaprod, core_system_containers,
every branch, checked 22 Nov 2026); the released image is ghcr.io/metrolla-inc/rslidar_drivers:v1.5.18,
built by the tag push of this fork's own v1.5.18 tag (2205e8b).

Goldens: for each environment in cases/*.env, the record of running the entrypoint (exit code,
stdout, stderr, the exec'd command and its environment, which setup files were sourced) as produced
by the **v1.5.18 entrypoint** (ros_entrypoint-v1.5.18.sh, byte-identical to git tag v1.5.18 of this
fork). The current ros_entrypoint.sh must produce the same record.

REGEN=1 rewrites the goldens from the v1.5.18 script, never from the current one.
"""

from __future__ import annotations

import hashlib
import os
from pathlib import Path

import pytest

from harness import REPO, dump, read_case, run

HERE = Path(__file__).resolve().parent
CASES = sorted((HERE / "cases").glob("*.env"))
REFERENCE = HERE / "ros_entrypoint-v1.5.18.sh"
CURRENT = REPO / "ros_entrypoint.sh"
V1518_BLOB = "2592355cba3ed0a2e8a91ceadea6e4cb7a6f72b3"  # git rev-parse v1.5.18:ros_entrypoint.sh


def git_blob_id(data: bytes) -> str:
    return hashlib.sha1(b"blob %d\0" % len(data) + data).hexdigest()


def test_reference_is_the_v1_5_18_entrypoint():
    assert git_blob_id(REFERENCE.read_bytes()) == V1518_BLOB


def test_there_are_legacy_cases():
    assert len(CASES) >= 7


@pytest.mark.parametrize("case", CASES, ids=[c.stem for c in CASES])
def test_legacy_golden(case: Path):
    env, argv = read_case(case)
    golden = HERE / "golden" / (case.stem + ".json")
    reference = dump(run(REFERENCE, env, argv))
    if os.environ.get("REGEN"):
        golden.write_text(reference)
    expected = golden.read_text()
    assert reference == expected, "the v1.5.18 entrypoint no longer gives its golden (harness changed?)"
    assert dump(run(CURRENT, env, argv)) == expected, "legacy behaviour changed"


def test_image_default_runs_the_image_command():
    """No command: the image's CMD runs, with ROS and the workspace sourced."""
    env, argv = read_case(HERE / "cases" / "image-default-cmd.env")
    record = run(CURRENT, env, argv)
    assert record["exit"] == 0
    assert record["exec"]["argv"] == ["ros2", "launch", "rslidar_sdk", "start.py"]
    assert record["exec"]["env"]["SOURCED"] == "/opt/ros/jazzy:/ros2_ws/install"
