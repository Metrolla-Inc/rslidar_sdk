#!/usr/bin/env bash
# Smoke tier of the RoboSense driver image (TESTING.md). Needs Docker and python3 (no sudo: the
# fake sensor sends UDP to 127.0.0.1 and the driver listens on all addresses).
# Usage: docker/smoke/smoke.sh IMAGE [OLD_IMAGE]
#
#   1. Image facts: entrypoint, command, user and workdir are the released v1.5.18 image's.
#   2. Legacy mode: no command, no contract: the image's own command decodes a fake RS-LiDAR-M1
#      (the model in the vendor config.yaml the legacy image reads) on /rslidar_points, frame
#      rslidar, sensor-clock stamps, every point checked. With OLD_IMAGE (the released
#      rslidar_drivers:v1.5.18) the same run is compared with it.
#   3. Contract checks and refusals in the image: byte-identical to tests/regression/golden/contract.
#   4. Contract runs as the product runs it (uid 97000, read-only root, tmpfs /tmp, cap-drop ALL,
#      no-new-privileges, host network and IPC, no command) against a fake RS-Helios: the node is
#      PID 1 and reads the rendered file; clouds on the contract's topic and frame, host or sensor
#      stamps, every point; docker stop ends it with exit 0 (three times, under load).
set -euo pipefail

IMAGE=${1:?usage: smoke.sh IMAGE [OLD_IMAGE]}
OLD_IMAGE=${2:-}
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/../.." && pwd)
WORK=$(mktemp -d)
NAME=rslidar-smoke-$$
FAKE=
FAILED=0
CLOCK_OFFSET=-100000.0  # the fake sensor's clock runs this far behind the host's
HARDENED=(--user 97000:97000 --read-only --tmpfs /tmp --cap-drop ALL
          --security-opt no-new-privileges:true --network host --ipc host)
SUMMARY=${GITHUB_STEP_SUMMARY:-/dev/null}

fail() { echo "SMOKE FAIL: $*" >&2; FAILED=1; }
ok() { echo "ok: $*"; }
expect() { local msg=$1; shift; if "$@"; then ok "$msg"; else fail "$msg"; fi; }
stop_fake() {
    if [[ -n "$FAKE" ]]; then
        kill -TERM "$FAKE" 2>/dev/null || true
        wait "$FAKE" 2>/dev/null || true
        FAKE=
    fi
}
cleanup() {
    stop_fake
    docker rm -f "$NAME" >/dev/null 2>&1 || true
    rm -rf "$WORK"
}
trap cleanup EXIT

start_fake() {  # LABEL MODEL DATA_PORT DEVICE_PORT
    python3 "$HERE/fake_sensor.py" --model "$2" --data-port "$3" --device-port "$4" \
        --clock-offset "$CLOCK_OFFSET" --seconds 300 > "$WORK/$1.fake.log" 2>&1 &
    FAKE=$!
}

# check_cloud LABEL USER TOPIC FRAME MODEL STAMP: the cloud check, in the image, as USER.
# The checker runs as the driver's uid: with host IPC, Fast DDS delivers through shared memory,
# and a subscriber of another uid gets discovery but no data.
check_cloud() {
    local label=$1 user=$2 topic=$3 frame=$4 model=$5 stamp=$6
    set +e
    docker run --rm --network host --ipc host --user "$user" --read-only --tmpfs /tmp \
        -e ROS_HOME=/tmp/ros -e ROS_LOG_DIR=/tmp/ros/log ${CHECK_ENV:+-e "$CHECK_ENV"} \
        -v "$HERE:/smoke:ro" --entrypoint bash "$IMAGE" -c \
        "source /opt/ros/jazzy/setup.bash && exec python3 /smoke/check_cloud.py --topic '$topic' \
         --frame '$frame' --model '$model' --stamp '$stamp' --clock-offset '$CLOCK_OFFSET' \
         --clouds 5 --timeout 90" > "$WORK/$label.check.json" 2> "$WORK/$label.check.err"
    local code=$?
    set -e
    cat "$WORK/$label.check.json"
    if [[ $code -ne 0 ]]; then
        tail -20 "$WORK/$label.check.err" >&2
        show_logs "$label"
        return 1
    fi
}

show_logs() { echo "--- $1 log (tail)"; docker logs --tail 40 "$NAME" 2>&1 || true; }

# stop_exit: docker stop (SIGTERM, 10 s grace) and the container's exit code.
stop_exit() {
    docker stop -t 10 "$NAME" >/dev/null
    docker inspect -f '{{.State.ExitCode}}' "$NAME"
}

# ---------------------------------------------------------------------------------------------
echo "== 1. image facts"
facts=$(docker image inspect -f '{{json .Config.Entrypoint}} {{json .Config.Cmd}} user={{.Config.User}} workdir={{.Config.WorkingDir}}' "$IMAGE")
echo "$facts"
expect "entrypoint, command, user and workdir as released" \
    test "$facts" = '["/ros_entrypoint.sh"] ["ros2","launch","rslidar_sdk","start.py"] user= workdir=/ros2_ws'
expect "the image's /ros_entrypoint.sh is this checkout's" \
    cmp -s <(docker run --rm --entrypoint cat "$IMAGE" /ros_entrypoint.sh) "$REPO/ros_entrypoint.sh"
expect "the image's vendor config is this checkout's config/config.yaml" \
    cmp -s <(docker run --rm --entrypoint cat "$IMAGE" /ros2_ws/src/rslidar_sdk/config/config.yaml) "$REPO/config/config.yaml"
expect "built with rs_driver 1.5.20" docker run --rm --entrypoint grep "$IMAGE" -q \
    "RSLIDAR_VERSION_PATCH 20" /ros2_ws/src/rslidar_sdk/src/rs_driver/src/rs_driver/macro/version.hpp

# ---------------------------------------------------------------------------------------------
echo "== 2. legacy mode (no contract): the image's command, vendor config (RSM1, 6699/7788)"
legacy_run() {  # LABEL IMAGE
    start_fake "$1" RSM1 6699 7788
    docker run -d --name "$NAME" --network host --ipc host \
        -e ROS_LOCALHOST_ONLY=1 -e USE_MACADDRESS_NS=0 "$2" >/dev/null
    local result=0
    CHECK_ENV=ROS_LOCALHOST_ONLY=1 check_cloud "$1" 0:0 /rslidar_points rslidar RSM1 sensor || result=1
    docker logs "$NAME" > "$WORK/$1.log" 2>&1 || true
    echo "$1: docker stop exit code $(stop_exit) (informational: legacy runs ros2 launch as PID 1)"
    docker rm -f "$NAME" >/dev/null
    stop_fake
    return $result
}
expect "legacy: /rslidar_points, frame rslidar, sensor-clock stamps, every point of every frame" \
    legacy_run legacy "$IMAGE"
expect "legacy: the node read the vendor config" grep -q "/ros2_ws/src/rslidar_sdk/config/config.yaml" "$WORK/legacy.log"
if [[ -n "$OLD_IMAGE" ]]; then
    expect "released image: same check" legacy_run released "$OLD_IMAGE"
    new=$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["data_sha256"])' "$WORK/legacy.check.json")
    old=$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["data_sha256"])' "$WORK/released.check.json")
    expect "legacy cloud data identical to the released image's ($old)" test "$new" = "$old"
    echo "Legacy clouds compared with $OLD_IMAGE: data sha256 $old (new $new)." >> "$SUMMARY"
fi

# ---------------------------------------------------------------------------------------------
echo "== 3. contract check and refusals in the image (goldens)"
for case in "$REPO"/tests/regression/cases/*.env; do
    stem=$(basename "$case" .env)
    switch=(-e METROLLA_DRIVER_CONTRACT=1)
    grep -q '^METROLLA_DRIVER_CONTRACT=' "$case" && switch=()
    for json in "" "--json"; do
        golden="$REPO/tests/regression/golden/contract/$stem${json:+.json}.golden"
        set +e
        # shellcheck disable=SC2086  # $json is empty or one word
        docker run --rm "${HARDENED[@]}" "${switch[@]}" --env-file "$case" "$IMAGE" check $json \
            > "$WORK/out" 2> "$WORK/err"
        code=$?
        set -e
        { echo "exit: $code"; echo "--- stdout"; cat "$WORK/out"; echo "--- stderr"; cat "$WORK/err"; } > "$WORK/got"
        if cmp -s "$WORK/got" "$golden"; then ok "check $stem ${json}"; else
            fail "check $stem ${json}: differs from its golden"; diff "$golden" "$WORK/got" | head -20 || true; fi
    done
done
set +e
docker run --rm "${HARDENED[@]}" -e METROLLA_DRIVER_CONTRACT=1 "$IMAGE" > "$WORK/out" 2>&1
code=$?
set -e
expect "contract with nothing set: exit 1, refusing to start" \
    bash -c "[[ $code -eq 1 ]] && grep -q 'rslidar-driver: refusing to start: LIDAR_MODEL is not set' '$WORK/out'"

# ---------------------------------------------------------------------------------------------
echo "== 4. contract runs (uid 97000, read-only, tmpfs /tmp, cap-drop ALL, host network and IPC)"
contract_start() {  # extra -e args...
    docker run -d --name "$NAME" "${HARDENED[@]}" -e METROLLA_DRIVER_CONTRACT=1 \
        -e LIDAR_MODEL=rshelios -e SENSOR_IP=127.0.0.1 "$@" "$IMAGE" >/dev/null
}

start_fake host RSHELIOS 6699 7788
contract_start -e SENSOR_ID=east -e FRAME_ID=east_lidar
expect "contract (host time): /east/rslidar_points, frame east_lidar, host stamps, every point" \
    check_cloud host 97000:97000 /east/rslidar_points east_lidar RSHELIOS host
pid1=$(docker exec "$NAME" cat /proc/1/cmdline | tr '\0' ' ')
echo "PID 1: $pid1"
expect "the node is PID 1 with the rendered config" \
    test "$pid1" = "/ros2_ws/install/rslidar_sdk/lib/rslidar_sdk/rslidar_sdk_node --ros-args -r __ns:=/east -p config_path:=/tmp/rslidar/config.yaml "
docker run --rm "${HARDENED[@]}" -e METROLLA_DRIVER_CONTRACT=1 -e LIDAR_MODEL=rshelios \
    -e SENSOR_IP=127.0.0.1 -e SENSOR_ID=east -e FRAME_ID=east_lidar "$IMAGE" check | sed '$d' > "$WORK/expected.yaml"
expect "the file the node read is what check renders" \
    cmp -s <(docker exec "$NAME" cat /tmp/rslidar/config.yaml) "$WORK/expected.yaml"
expect "running as uid 97000" test "$(docker exec "$NAME" id -u)" = 97000
mem=$(docker stats --no-stream --format '{{.MemUsage}}' "$NAME")
echo "Contract RS-Helios at 1500 packets/s: memory $mem" | tee -a "$SUMMARY"
code=$(stop_exit)
expect "docker stop under load: exit 0 (got $code)" test "$code" = 0
docker logs "$NAME" > "$WORK/host.log" 2>&1 || true
expect "the node logged its clean stop" grep -q "RoboSense-LiDAR-Driver is stopping" "$WORK/host.log"
docker rm -f "$NAME" >/dev/null
for i in 1 2; do
    contract_start -e SENSOR_ID=east
    sleep 8
    code=$(stop_exit)
    expect "repeat stop $i under load: exit 0 (got $code)" test "$code" = 0
    docker rm -f "$NAME" >/dev/null
done
stop_fake

start_fake sensor RSHELIOS 6700 7789
contract_start -e DATA_PORT=6700 -e DEVICE_PORT=7789 -e TIME_MODE=sensor -e POINTCLOUD_TOPIC=/lidar/points
expect "contract (sensor time, ports 6700/7789, absolute topic): /lidar/points, frame rslidar, sensor-clock stamps" \
    check_cloud sensor 97000:97000 /lidar/points rslidar RSHELIOS sensor
code=$(stop_exit)
expect "docker stop: exit 0 (got $code)" test "$code" = 0
docker rm -f "$NAME" >/dev/null
stop_fake

if [[ $FAILED -ne 0 ]]; then
    echo "SMOKE: FAILED" >&2
    exit 1
fi
echo "SMOKE: all checks passed"
