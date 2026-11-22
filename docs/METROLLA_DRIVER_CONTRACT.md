# Metrolla driver contract (RoboSense image)

The image `ghcr.io/metrolla-inc/rslidar_drivers` (built from `Dockerfile` on `main`) takes its
settings from the environment the same way as the other Metrolla driver images (Seyond, Hesai,
LSLiDAR, Quanergy, Ouster), and refuses a value it does not understand.

The contract is **opt-in**. Without `METROLLA_DRIVER_CONTRACT` (or with it empty or `0`) the image
behaves as the released `v1.5.18` image: `ros_entrypoint.sh` is the v1.5.18 file with one block in
front of it (tests/legacy pins this byte for byte), and the image's entrypoint, command, user and
workdir are unchanged. No fleet runs this image today (balena-fleets, balena-fleets-metrollaprod
and core_system_containers, every branch, checked 22 Nov 2026). The driver inside is rs_driver
1.5.20 from this release on: see docs/VENDOR_DIFF.md for what that changes.

## Switch

| `METROLLA_DRIVER_CONTRACT` | Behaviour |
|---|---|
| unset, empty, `0` | legacy: the v1.5.18 entrypoint runs the command (default `ros2 launch rslidar_sdk start.py`, which reads the vendor's `config/config.yaml` baked into the image: RSM1, ports 6699/7788, `/rslidar_points`, frame `rslidar`, sensor clock) |
| `1` | contract: the environment below is checked, the driver's config rendered, the driver started |
| anything else | refused, exit 1 |

## Variables (with `METROLLA_DRIVER_CONTRACT=1`)

| Variable | Required | Accepted | Default | Becomes |
|---|---|---|---|---|
| `LIDAR_MODEL` | yes | one of the models below; any case, `_` or `-`, optional `robosense-` prefix; the driver's `rs` may be written `rs-`/`rslidar-` or left out before a letter (`rshelios`, `RS-Helios`, `helios`, `RSLIDAR-M1`, `m1`) | - | `lidar_type` |
| `SENSOR_IP` | yes | the sensor's IPv4 address | - | logged and written in the rendered file only: rs_driver listens on its ports and cannot filter by sender, so **each sensor on a host needs its own ports** |
| `SENSOR_ID` | no | `[a-z][a-z0-9_]{0,30}`, no `__` | none | ROS namespace `/<SENSOR_ID>` (default `/`) |
| `DATA_PORT` | no | 1024-65535 | `6699` | `msop_port` (point data, MSOP) |
| `DEVICE_PORT` | no | 1024-65535, not `DATA_PORT` | `7788` (`7766` for `rsemx`); **required when `DATA_PORT` is not 6699** | `difop_port` (device info and calibration, DIFOP) |
| `HOST_IP` | no | IPv4, optional `/1`-`/32`; not the sensor's address; with a prefix the sensor must be in the subnet | none | checked only; the driver listens on all addresses (a socket bound to one address would miss a sensor sending to broadcast) |
| `FRAME_ID` | no | `[A-Za-z_][A-Za-z0-9_]{0,63}` | `rslidar` | `ros_frame_id` (the cloud's `header.frame_id`) |
| `POINTCLOUD_TOPIC` | no | a ROS topic name, absolute or relative to the namespace | `rslidar_points` (`/rslidar_points`, or `/<SENSOR_ID>/rslidar_points`) | `ros_send_point_cloud_topic` |
| `TIME_MODE` | no | `host`, `sensor`, `gps`, `ptp` (any case); `ntp` refused | `host` | `use_lidar_clock`: host = false (stamps are the host's receive time of the frame's first packet, less one packet's duration); sensor/gps/ptp = true (stamps are the sensor's clock from the packets; the sensor must be set to that time source on its own web page: the driver never configures the sensor) |
| `PUBLISH_SCAN` | no | false values only | false | - (no laser scan output) |
| `RSLIDAR_RUN_DIR` | no | absolute path | `/tmp/rslidar` | where `config.yaml` is written |
| `DUAL_LIDAR` | no | false values only (empty = false) | false | - (one container per sensor) |

Why `DEVICE_PORT` is required with a moved `DATA_PORT`: rs_driver binds its ports with
`SO_REUSEADDR`, so two drivers on one host can both bind 7788 and the second silently takes the
other sensor's DIFOP packets, i.e. its calibration angles. A second sensor therefore needs its own
pair, set on the sensor and here.

Models (every `lidar_type` rs_driver 1.5.20 accepts; the vendor's product names from rs_driver's
README):

| `LIDAR_MODEL` | Product | Kind |
|---|---|---|
| `rs16` | RS-LiDAR-16 | mechanical |
| `rs32` | RS-LiDAR-32 | mechanical |
| `rsbp` | RS-Bpearl | mechanical |
| `rshelios` | RS-Helios | mechanical |
| `rshelios-16p` | RS-Helios-16P | mechanical |
| `rs128` | RS-Ruby-128 | mechanical |
| `rs80` | RS-Ruby-80 | mechanical |
| `rs48` | RS-Ruby-48 | mechanical |
| `rsp128` | RS-Ruby-Plus-128 | mechanical |
| `rsp80` | RS-Ruby-Plus-80 | mechanical |
| `rsp48` | RS-Ruby-Plus-48 | mechanical |
| `rsairy` | RS-LiDAR-AIRY | mechanical |
| `rsairylite-eth` | RS-LiDAR-AIRYLITE-ETH (new in 1.5.20) | mechanical |
| `rsfairy` | RS-LiDAR-FAIRY (new in 1.5.19) | mechanical |
| `rsm1` | RS-LiDAR-M1 | solid-state |
| `rsm1-jumbo` | RS-LiDAR-M1 sending jumbo packets | solid-state |
| `rsm2` | RS-LiDAR-M2 | solid-state |
| `rsm3` | RS-LiDAR-M3 | solid-state |
| `rse1` | RS-LiDAR-E1 | solid-state |
| `rsmx` | RS-LiDAR-MX | solid-state |
| `rsemx` | RS-LiDAR-EMX (new in 1.5.19; DIFOP default 7766, max distance 300 m) | solid-state |

Fixed in contract mode (the rest of the vendor's `config/config.yaml` is kept as it is):
`msg_source: 1` (UDP from the sensor), `send_packet_ros: false`, `send_point_cloud_ros: true`,
`imu_port: 0` (the image is built without IMU parsing), `max_distance` 200 m (300 m for `rsemx`, as
the vendor's comment says), `min_distance` 0.2 m, `dense_points: false` (NaN points kept, so a
mechanical model's cloud is organised: `height` x `width` = lasers x blocks), `ts_first_point: true`
(the stamp is the frame's first point), `start_angle` 0 / `end_angle` 360, `wait_for_difop` true
(rs_driver's default: no cloud until the sensor's calibration has arrived). Point type XYZI
(fields `x`, `y`, `z`, `intensity`, float32, point_step 16), as the v1.5.18 build. QoS: reliable,
keep last 100 (the vendor's `ros_queue_length`).

Refusals: every problem is one line on stderr starting `rslidar-driver: refusing to start:`, all
problems at once, exit 1. A legacy or vendor name (`SENSOR_TYPE`, `SENSOR_PORT`, `LIDAR_TYPE`,
`LIDAR_IP`, `MSOP_PORT`, `DIFOP_PORT`, `HOST_ADDRESS`, `ROS_FRAME_ID`, `USE_LIDAR_CLOCK`, ...) is
refused with the contract's name; an unknown variable under the contract's prefixes is refused with
the closest contract name. Other variables (`ROS_*`, `RMW_*`, `METROLLA_*`, ...) pass through.

Commands with the switch on: none, `run`, or the image's default command = run; `check` prints the
rendered file and the exec line; `check --json` the same as JSON; `selfcheck` renders every model
against the vendor file (run at image build); any other command is refused.

## How it runs

`/ros_entrypoint.sh` execs `/opt/metrolla/rslidar/contract-entrypoint.sh`, which sources ROS 2
Jazzy and the workspace and execs `python3 -I /opt/metrolla/rslidar/rslidar_env.py`. That checks
the environment, writes `<RSLIDAR_RUN_DIR>/config.yaml` (the vendor's
`/ros2_ws/src/rslidar_sdk/config/config.yaml` with the contract's keys replaced; each must be there
exactly once or the image build fails) and execs
`/ros2_ws/install/rslidar_sdk/lib/rslidar_sdk/rslidar_sdk_node --ros-args [-r __ns:=/<SENSOR_ID>] -p config_path:=<file>`.
The node is PID 1 and ends cleanly on SIGTERM, exit 0 (one vendor line added, below). No launch
file runs (the vendor's `start.py` only starts the same node).

## For the product (metrolla-edge generator)

```yaml
image: ghcr.io/metrolla-inc/rslidar_drivers:<tag from main after this change, pinned by digest>
user: "97000:97000"
read_only: true
tmpfs: [/tmp]
network_mode: host
ipc: host
cap_drop: [ALL]
security_opt: ["no-new-privileges:true"]
environment:
  METROLLA_DRIVER_CONTRACT: "1"
  LIDAR_MODEL: rshelios           # metrolla-edge model "robosense-<model>" (prefix optional)
  SENSOR_IP: 192.168.1.200
  SENSOR_ID: <sensors[].id>
  DATA_PORT: "6699"               # MSOP; a second sensor on the host needs other ports...
  DEVICE_PORT: "7788"             # ...and its own DIFOP port (required when DATA_PORT != 6699)
  FRAME_ID: <frame>               # optional
  POINTCLOUD_TOPIC: <topic>       # optional
  TIME_MODE: host                 # or ptp/gps/sensor when the sensor is set to that time source
# no command, no config file mount
```

Same uid for every ROS container on the host: with host IPC, Fast DDS (the image's default RMW)
delivers between processes through shared memory, and a subscriber running as another uid gets
discovery but no data (seen in the Ouster smoke test; this smoke test runs its subscriber as the
driver's uid).

Host firewall: allow UDP `DATA_PORT` and `DEVICE_PORT` in from the sensor. The driver sends
nothing to the sensor.

Sensor side (not done by the driver): RoboSense ships sensors at 192.168.1.200 sending to
192.168.1.102, MSOP 6699, DIFOP 7788 (the vendor's documentation, rs_driver doc/howto; not checked
on a sensor). The destination address and both ports are set on the sensor's web page.

## Vendor code change

One, in its own commit: `rslidar_sdk_node` binds SIGTERM to the same handler as SIGINT. Before,
rclcpp's SIGTERM handler shut ROS down but `main` kept waiting on a condition variable only that
handler notifies, so `docker stop` waited 10 s and killed the node (exit 137). It affects legacy
mode too: `docker stop` of a legacy container now ends the node at once. A SIGTERM during the
first moments of start-up, before `main` starts waiting, is still missed (vendor design; the node
is then killed after the grace period).

## Not proven (needs hardware)

Certification needs a real RoboSense sensor. Proven only with synthetic packets (CI): rs_driver
1.5.20 decodes every model 1.5.18 knew exactly as 1.5.18 did (tests/regression), and the image
publishes RS-Helios and RS-LiDAR-M1 clouds that match an independent decoder point for point
(TESTING.md). Not proven: any real sensor's packets (firmware variants, dual return, real
calibration, FOV blind zones, jumbo packets), the three models new in 1.5.19/1.5.20 beyond a
synthetic decode, PTP/GPS time on a sensor (whether its clock is UTC or TAI), packet loss at real
rates on a busy unit, the factory network settings above.
