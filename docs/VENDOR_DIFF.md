# Vendor update: rslidar_sdk v1.5.18 to v1.5.20, rs_driver 1.5.18 to 1.5.20

Taken on 22 Nov 2026 (D3 part B) by merging the vendor's tag `v1.5.20` (rslidar_sdk `8b4b4b7`,
which pins rs_driver `897b14d` = rs_driver v1.5.20) into `main`. Before: rslidar_sdk `7c4ea25`
(the vendor's v1.5.18) plus Metrolla's three commits, rs_driver `cd35885` (v1.5.18). These are the
vendor's current releases (newest tags on 22 Nov 2026; `.github/vendor-watch.json` records the new
base, and vendor-watch reports both up to date).

Note on tags: this fork's own tag `v1.5.18` is `2205e8b` (Metrolla's image release), not the
vendor's `v1.5.18` (`7c4ea25`). It is left where it is. A clone that fetches the vendor's tags gets
the vendor's `v1.5.19` and `v1.5.20` too; image tags for this fork must not reuse those names.

Why it is safe without a sensor: rs_driver only listens; it never writes to the sensor (unlike the
Ouster SDK update, which configures the sensor and refuses old firmware). The decoding of every
model 1.5.18 knew is proven unchanged on synthetic packets, and no fleet runs this image.

## Metrolla's changes on top of the vendor (kept)

`2205e8b` and its two parents: Dockerfile (ROS 2 Jazzy), `ros_entrypoint.sh`, image workflow,
`docker-compose.yml`, `launch/ros2_start.py`, `launch/humble_start.py` distro-agnostic, `start.py`
without rviz, C++17, `.gitmodules` URL to GitHub, README/CHANGELOG notes. The merge had one
conflict (CHANGELOG.md, both sides kept). None of the vendor's changes touch these files except
CHANGELOG and README.

## rslidar_sdk 7c4ea25 to 8b4b4b7

Documentation, `package.xml` (version 1.5.20, maintainer), and comments in `config/config.yaml`
(new model names, "DIFOP 7766 for RSEMX", "300 m for RSEMX"). No value in `config/config.yaml`
changed, no source file changed. So the legacy image still reads RSM1, 6699/7788, 0.2-200 m,
sensor clock, `/rslidar_points`, frame `rslidar`.

## rs_driver 1.5.18 to 1.5.20: every difference that can change output

| # | Change (file) | Output effect | Proof |
|---|---|---|---|
| 1 | Decoders of the 18 models 1.5.18 knew (decoder_*.hpp): only formatting, plus RSE1's own minimum distance 0.2 m to 0.0 m | none with the vendor config or the contract (min 0.2, max 200 set). Only when `min_distance` and `max_distance` are both 0 does RSE1 now keep points under 0.2 m | tests/regression/test_vendor_decode.py: every model, two packet seeds, limits 0.2/200 and 0/0, 1500 packets each through rs_driver's own LidarDriver: identical point-for-point hashes, cloud sizes, stamps and sequence numbers to 1.5.18 (goldens made with 1.5.18 before the merge, and re-made from the 1.5.18 source in CI); the two RSE1 0/0 runs are the only listed differences |
| 2 | New models RSFAIRY, RSEMX (1.5.19), RSAIRYLITE_ETH (1.5.20); `LidarType` enum renumbered (two values inserted after RSAIRY) | new `lidar_type` names accepted; the enum's numbers are internal (not in any packet, file or ROS message) | RSFAIRY and RSEMX decode synthetic packets (goldens); RSAIRYLITE_ETH is not exercised (no packet generator) |
| 3 | Packet input (input_sock_select.hpp): `recvmmsg` up to 16 packets per call instead of one `recvfrom`; packets no longer than the user/tail layers are dropped instead of queued empty | none for well-formed packets | smoke: 1500 packets/s RS-Helios and 6300 packets/s M1 over UDP, every point of every frame checked |
| 4 | Socket receive buffer: always set (the `ENABLE_MODIFY_RECVBUF` switch is gone) to at least 4 MiB instead of 106496 bytes | fewer drops in bursts. The kernel caps it at `net.core.rmem_max` (212992 by default) for a process without CAP_NET_ADMIN, so on a default host nothing changes | not measurable in CI; the log line "After setting: receive buffer size" shows the real value |
| 5 | Packet queue (sync_queue.hpp): a lock-free single-producer ring of 102400 entries replaces a mutex queue; it is emptied only when full (was: above 10240 packets) | under CPU starvation the driver can now fall up to ~68 s behind at 1500 packets/s (RS-Helios) before dropping, instead of ~7 s; clouds then arrive late (stamps stay correct in sensor time, host-time stamps are the decode time). The emptying on overflow runs in the producer thread while the consumer may read (vendor race, only on overflow) | not reproducible without starving the node; noted for the bench |
| 6 | Handle threads (lidar_driver_impl.hpp, rslidar_sdk's source_driver via SyncQueue): wait by polling with 1 ms sleeps instead of a condition variable; packets processed in batches of up to 50 | up to about 1 ms more latency per cloud; about 1000 wake-ups/s per waiting thread when idle | smoke: stamps and frame gaps checked (0.1 s apart) |
| 7 | Point clouds reserve 1.7 million points each (`MAX_POINT_CLOUD_SIZE`); a finer sine table (~4.3 MB per decoder); ~1.6 MB per queue | address space about 27 MB per cloud object (XYZI, 16 bytes a point), mostly never touched, so resident memory stays small: the whole contract container used 27 MiB with RS-Helios at 1500 packets/s (smoke, CI run 45027604718) | smoke prints the container's memory (job summary); a 128-laser model at full rate is not measured |
| 8 | `MAX_BLOCKS_PER_PKT` 12 to 16 (block_iterator.hpp) | array size only | (1) |
| 9 | QNX support (`__QNX__` only) | none on Linux | - |

Units, stamps, frames and field layout: unchanged (meters, seconds, `rslidar` frame, XYZI).

## Not proven (needs hardware)

Real packets of any model; the receive path under real packet loss and bursts (3-6); memory on a
small unit (7); whether a busy unit now publishes late clouds instead of dropping them (5).
