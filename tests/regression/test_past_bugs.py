"""Past bugs: each check passes on this checkout and fails on the released v1.5.18 files (this
fork's tag v1.5.18 = 2205e8b, read with git show; the regression job fetches full history)."""

from __future__ import annotations

import re
import subprocess
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]
RELEASED = "2205e8b1ef6994c7043b6c9ecdc5515c2cc1c6ed"  # fork tag v1.5.18 (not the vendor's v1.5.18)


def released(path: str) -> str:
    try:
        return subprocess.run(["git", "-C", str(REPO), "show", f"{RELEASED}:{path}"], check=True,
                              capture_output=True, text=True).stdout
    except subprocess.CalledProcessError:
        pytest.skip(f"{RELEASED[:7]} not in this clone (fetch full history)")


def current(path: str) -> str:
    return (REPO / path).read_text()


# 1. docker stop: SIGTERM did not end the node. rclcpp's SIGTERM handler shuts the ROS context down
#    but main() waits on a condition variable only sigHandler notifies, and only SIGINT was bound to
#    it; as PID 1 the node then sat until Docker's SIGKILL (exit 137).
def sigterm_bound(source: str) -> bool:
    main = source[source.index("int main("):]
    bind = main.find("signal(SIGTERM, sigHandler)")
    init = main.find("rclcpp::init(")
    return 0 <= bind < init


def test_sigterm_ends_the_node():
    assert sigterm_bound(current("node/rslidar_sdk_node.cpp"))
    assert not sigterm_bound(released("node/rslidar_sdk_node.cpp"))


# 2. Images were pushed for every push to main (and v* branches) as :main etc., from a moving
#    branch, and over an existing tag; actions were not pinned.
WORKFLOW = ".github/workflows/docker-build-push.yml"


def workflow_problems(text: str) -> list[str]:
    found = []
    on = text[text.index("\non:"):text.index("\npermissions:")]
    if "branches" in on:
        found.append("pushes images for branches")
    if "push: true" in text and "startsWith(github.ref, 'refs/tags/v')" not in text:
        found.append("pushes without a tag guard")
    if re.search(r"^\s+ref:", text, re.M):
        found.append("checks out something other than the tagged commit")
    if "already exists" not in text:
        found.append("may overwrite an existing image tag")
    for use in re.findall(r"uses:\s*(\S+)", text):
        if not re.fullmatch(r"[\w.-]+/[\w./-]+@[0-9a-f]{40}", use):
            found.append(f"action not pinned by sha: {use}")
    if "cache-to" in text:
        found.append("writes a registry cache tag")
    return found


def test_images_only_for_tags_from_the_tagged_commit():
    assert workflow_problems(current(WORKFLOW)) == []
    assert "pushes images for branches" in workflow_problems(released(WORKFLOW))


# 3. The Dockerfile cloned rs_driver's default branch when the submodule was missing, and
#    rslidar_msg unpinned: the image's code depended on the day it was built.
def dockerfile_problems(text: str) -> list[str]:
    found = []
    if "git clone https://github.com/RoboSense-LiDAR/rs_driver" in text:
        found.append("clones rs_driver's moving branch")
    if "rslidar_msg" in text and "RSLIDAR_MSG_COMMIT" not in text:
        found.append("rslidar_msg unpinned")
    return found


def test_dockerfile_builds_pinned_sources():
    assert dockerfile_problems(current("Dockerfile")) == []
    assert dockerfile_problems(released("Dockerfile")) == ["clones rs_driver's moving branch",
                                                           "rslidar_msg unpinned"]
