# Tests (Metrolla)

Three tiers, separately named jobs in `.github/workflows/ci.yml`. The vendor's own tests
(`src/rs_driver/test`, gtest) are not run by the Metrolla image build (as in v1.5.18).
`.github/workflows/vendor-watch-tests.yml` tests the weekly vendor comparison separately.

| Tier | Command | What it proves |
|---|---|---|
| unit | `python -m pytest tests/unit tests/legacy` (+ shellcheck, hadolint in CI; needs g++) | the contract (docker/rslidar_env.py): every variable, default, refusal, model name, the rendered config against the vendor's `config/config.yaml`, the exec line, the run/check/selfcheck commands; the legacy entrypoint is v1.5.18's plus the opt-in block; the smoke test's synthetic RS-Helios and M1 packets decode in rs_driver (built from the submodule) to exactly the reference points the smoke test expects |
| regression | `RS_DRIVER_OLD=<rs_driver 1.5.18 tree> python -m pytest tests/regression tests/legacy` | goldens: legacy entrypoint records for 7 environments, generated from the v1.5.18 entrypoint (`tests/legacy/ros_entrypoint-v1.5.18.sh`, git blob pinned); contract `check` and `check --json` output for 18 environments (accepted and refused); decoding of every model rs_driver 1.5.18 knew (18 models x 2 seeds x 2 distance settings, 1500 packets each, through rs_driver's LidarDriver) identical to 1.5.18, except the differences listed in docs/VENDOR_DIFF.md; with `RS_DRIVER_OLD` also that the 1.5.18 source still gives those goldens; past bugs (SIGTERM did not end the node; images pushed for branches, unpinned actions; Dockerfile cloned rs_driver's moving branch and rslidar_msg unpinned), each failing on the released v1.5.18 files |
| smoke | `docker/smoke/smoke.sh IMAGE [OLD_IMAGE]` (needs Docker and python3) | the real image: entrypoint/command/user/workdir unchanged, rs_driver 1.5.20 inside; legacy mode (no contract, the image's command and baked-in config, RSM1) publishes `/rslidar_points` frame `rslidar` from a fake M1 at 6300 packets/s, sensor-clock stamps, every point of five frames; with OLD_IMAGE the released image gives the same cloud data; contract `check` and refusals byte-identical to the goldens; contract as uid 97000, read-only root, tmpfs /tmp, cap-drop ALL, no-new-privileges, host network and IPC, from a fake RS-Helios at 1500 packets/s: PID 1 is the node reading the rendered file, clouds on the contract topic and frame, host stamps (and sensor stamps with TIME_MODE=sensor on ports 6700/7789), every point of five frames, three `docker stop` under load with exit 0 |

Run the Python tiers locally: `python -m venv .venv && .venv/bin/pip install pytest==9.0.1
pyyaml==6.0.3`, check out the submodule, then the commands above. For `RS_DRIVER_OLD`:
`git -C src/rs_driver archive cd358851ab65bf57fc7e321837be2a425305b298 | tar -x -C /tmp/rs_driver-1.5.18`.
`REGEN=1` rewrites goldens (legacy ones only from the v1.5.18 script; decode ones only for the
current rs_driver's listed differences and new models, never 1.5.18's).

Not covered here (needs hardware): see docs/METROLLA_DRIVER_CONTRACT.md, "Not proven".
