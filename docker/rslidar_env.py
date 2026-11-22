#!/usr/bin/env python3
"""Environment contract of the Metrolla RoboSense driver image (docs/METROLLA_DRIVER_CONTRACT.md).

Reads the container's environment, refuses to start on any value it does not understand, renders
the driver's config.yaml from the vendor's own config/config.yaml (only the keys the contract sets
are replaced; each must be in the vendor file exactly once) and (subcommand ``run``) execs
``rslidar_sdk_node`` with it, so signals reach the driver itself.

Standard library only: it runs in the image before the driver and in the unit and regression tests
outside the image, where there is no ROS.

Subcommands:
  run              validate, write the config file, exec the driver (the image's default)
  check [--json]   validate and print what ``run`` would write and exec; nothing is started
  selfcheck        render every model against the vendor file (run at image build)

Exit codes: 0 ok; 1 refused (a value this image does not understand, nowhere to write, or an
unknown command).
"""

from __future__ import annotations

import difflib
import ipaddress
import json
import os
import re
import sys
from dataclasses import dataclass, field
from typing import Mapping

PROG = "rslidar-driver"
EXIT_OK = 0
EXIT_REFUSED = 1

WORKSPACE = "/ros2_ws"
DRIVER_NODE = WORKSPACE + "/install/rslidar_sdk/lib/rslidar_sdk/rslidar_sdk_node"
# The file the legacy image reads (the node's compiled-in PROJECT_PATH + /config/config.yaml).
VENDOR_CONFIG = WORKSPACE + "/src/rslidar_sdk/config/config.yaml"
DEFAULT_RUN_DIR = "/tmp/rslidar"
DEFAULT_ROS_HOME = "/tmp/ros"

DEFAULT_DATA_PORT = 6699  # vendor default msop_port (config/config.yaml, rs_driver RSInputParam)
DEFAULT_DEVICE_PORT = 7788  # vendor default difop_port
MIN_PORT = 1024  # the driver runs as a non-root uid with no capability: no port below 1024
MAX_PORT = 65535
DEFAULT_FRAME_ID = "rslidar"  # vendor default ros_frame_id
DEFAULT_TOPIC = "rslidar_points"  # vendor default /rslidar_points, relative so it follows SENSOR_ID

TRUE_WORDS = ("1", "true", "yes", "on")
FALSE_WORDS = ("0", "false", "no", "off")
TIME_MODES = ("host", "sensor", "gps", "ptp")


# --------------------------------------------------------------------------------------------
# Models: every lidar_type rs_driver 1.5.20 accepts (driver_param.hpp, strToLidarType), with the
# vendor's product name (rs_driver README.md, "Supported LiDARs").
# --------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class Model:
    lidar_type: str  # the driver's lidar_type
    label: str  # the vendor's product name
    kind: str  # mechanical or solid-state (MEMS/flash), for messages
    difop_port: int = DEFAULT_DEVICE_PORT  # vendor default device (DIFOP) port
    max_distance: float = 200.0  # the vendor config's max_distance for this model

    @property
    def key(self) -> str:
        return self.lidar_type.lower().replace("_", "-")


MODELS_LIST = (
    Model("RS16", "RS-LiDAR-16", "mechanical"),
    Model("RS32", "RS-LiDAR-32", "mechanical"),
    Model("RSBP", "RS-Bpearl", "mechanical"),
    Model("RSHELIOS", "RS-Helios", "mechanical"),
    Model("RSHELIOS_16P", "RS-Helios-16P", "mechanical"),
    Model("RS128", "RS-Ruby-128", "mechanical"),
    Model("RS80", "RS-Ruby-80", "mechanical"),
    Model("RS48", "RS-Ruby-48", "mechanical"),
    Model("RSP128", "RS-Ruby-Plus-128", "mechanical"),
    Model("RSP80", "RS-Ruby-Plus-80", "mechanical"),
    Model("RSP48", "RS-Ruby-Plus-48", "mechanical"),
    Model("RSAIRY", "RS-LiDAR-AIRY", "mechanical"),
    Model("RSAIRYLITE_ETH", "RS-LiDAR-AIRYLITE-ETH", "mechanical"),
    Model("RSFAIRY", "RS-LiDAR-FAIRY", "mechanical"),
    Model("RSM1", "RS-LiDAR-M1", "solid-state"),
    Model("RSM1_JUMBO", "RS-LiDAR-M1 (jumbo packets)", "solid-state"),
    Model("RSM2", "RS-LiDAR-M2", "solid-state"),
    Model("RSM3", "RS-LiDAR-M3", "solid-state"),
    Model("RSE1", "RS-LiDAR-E1", "solid-state"),
    Model("RSMX", "RS-LiDAR-MX", "solid-state"),
    # config/config.yaml: "For RSEMX, use the port num of DIFOP2 ... Default 7766", "300 for RSEMX".
    Model("RSEMX", "RS-LiDAR-EMX", "solid-state", difop_port=7766, max_distance=300.0),
)
MODELS = {m.key: m for m in MODELS_LIST}


def normalise_model(value: str) -> str | None:
    """LIDAR_MODEL to a model key: any case, '_' or '-', optional 'robosense-' prefix; the
    driver's 'rs' may be written 'rs-' or 'rslidar-' (rs-helios, rslidar-m1) or left out where a
    letter follows it (helios, m1, bp, airy)."""
    key = value.lower().replace("_", "-")
    if key.startswith("robosense-"):
        key = key[len("robosense-"):]
    if key.startswith("rs-"):  # the vendor's product names are written RS-Helios, RS-LiDAR-M1
        key = "rs" + key[len("rs-"):]
    if key.startswith("rslidar-"):
        key = "rs" + key[len("rslidar-"):]
    if key in MODELS:
        return key
    if re.fullmatch(r"[a-z].*", key) and "rs" + key in MODELS:
        return "rs" + key
    return None


def model_list() -> str:
    return ", ".join(m.key for m in MODELS_LIST)


# --------------------------------------------------------------------------------------------
# Environment variables.
# --------------------------------------------------------------------------------------------

REQUIRED = ("LIDAR_MODEL", "SENSOR_IP")
OPTIONAL = ("SENSOR_ID", "DATA_PORT", "DEVICE_PORT", "HOST_IP", "FRAME_ID", "POINTCLOUD_TOPIC",
            "TIME_MODE", "PUBLISH_SCAN", "RSLIDAR_RUN_DIR")
COMPAT = ("DUAL_LIDAR",)  # read only to refuse a dual-lidar request; one container per sensor
KNOWN = REQUIRED + OPTIONAL + COMPAT
# Names the vendor, the legacy fleets' .env files or other drivers use for the same thing.
ALIASES = {
    "LIDAR_TYPE": "LIDAR_MODEL", "SENSOR_TYPE": "LIDAR_MODEL", "SENSOR_MODEL": "LIDAR_MODEL",
    "LIDAR_IP": "SENSOR_IP", "DEVICE_IP": "SENSOR_IP",
    "MSOP_PORT": "DATA_PORT", "SENSOR_PORT": "DATA_PORT", "LIDAR_PORT": "DATA_PORT",
    "DIFOP_PORT": "DEVICE_PORT", "HOST_ADDRESS": "HOST_IP", "ROS_FRAME_ID": "FRAME_ID",
    "ROS_SEND_POINT_CLOUD_TOPIC": "POINTCLOUD_TOPIC", "USE_LIDAR_CLOCK": "TIME_MODE",
    "TIMESTAMP_MODE": "TIME_MODE", "CONFIG_PATH": "RSLIDAR_RUN_DIR",
}
# Any other variable that starts with one of these is refused as a probable typo. METROLLA_ is not
# among them: the stack may pass its own METROLLA_* variables to every service.
STRICT_PREFIXES = ("RSLIDAR_", "LIDAR_", "SENSOR_", "DATA_", "DEVICE_", "HOST_", "TIME_", "FRAME_",
                   "POINTCLOUD_", "PUBLISH_", "MSOP_", "DIFOP_", "USE_LIDAR", "TIMESTAMP_",
                   "ROS_FRAME", "ROS_SEND", "CONFIG_PATH")

ACCEPTED = {
    "LIDAR_MODEL": "a RoboSense model such as rshelios, rs128 or rsm1 (the full list is in the "
                   "message when the name is unknown)",
    "SENSOR_IP": "the sensor's own IPv4 address, for example 192.168.1.200",
    "SENSOR_ID": "a lowercase letter then up to 30 lowercase letters, digits or single '_'",
    "DATA_PORT": f"a UDP port {MIN_PORT}-{MAX_PORT} (default {DEFAULT_DATA_PORT})",
    "DEVICE_PORT": f"a UDP port {MIN_PORT}-{MAX_PORT} other than DATA_PORT (default "
                   f"{DEFAULT_DEVICE_PORT}, 7766 for rsemx)",
    "HOST_IP": "the host's IPv4 address on the sensor network, optionally with /1-/32",
    "FRAME_ID": f"letters, digits and '_', not starting with a digit (default {DEFAULT_FRAME_ID})",
    "POINTCLOUD_TOPIC": f"a ROS topic name (default {DEFAULT_TOPIC})",
    "TIME_MODE": "host, sensor, gps or ptp (default host)",
    "PUBLISH_SCAN": "a false value (0, false, no, off): this driver has no laser scan output",
    "RSLIDAR_RUN_DIR": f"an absolute directory path (default {DEFAULT_RUN_DIR})",
}

_PORT_RE = re.compile(r"[1-9][0-9]{0,4}")
_OCTET = r"(?:25[0-5]|2[0-4][0-9]|1[0-9][0-9]|[1-9]?[0-9])"
_IPV4_RE = re.compile(rf"{_OCTET}\.{_OCTET}\.{_OCTET}\.{_OCTET}")
_PREFIX_RE = re.compile(r"[1-9]|[12][0-9]|3[0-2]")
_SENSOR_ID_RE = re.compile(r"[a-z][a-z0-9_]{0,30}")  # metrolla-edge deployment schema, sensors[].id
_FRAME_RE = re.compile(r"[A-Za-z_][A-Za-z0-9_]{0,63}")
_TOPIC_RE = re.compile(r"/?[A-Za-z][A-Za-z0-9_]*(?:/[A-Za-z][A-Za-z0-9_]*)*")


@dataclass
class Settings:
    model: Model
    sensor_ip: str
    sensor_id: str | None
    data_port: int
    device_port: int
    host_ip: str | None
    frame_id: str
    topic: str
    time_mode: str
    run_dir: str
    sources: dict[str, str] = field(default_factory=dict)  # variable -> "env" or "default"

    @property
    def namespace(self) -> str:
        return "/" + self.sensor_id if self.sensor_id else "/"

    @property
    def resolved_topic(self) -> str:
        if self.topic.startswith("/"):
            return self.topic
        return self.namespace.rstrip("/") + "/" + self.topic

    @property
    def config_path(self) -> str:
        return self.run_dir.rstrip("/") + "/config.yaml"


class Refused(Exception):
    def __init__(self, problems: list[str]):
        super().__init__("\n".join(problems))
        self.problems = problems


def shown(value: str) -> str:
    """A value as it appears in a message: quoted, escaped, cut short."""
    text = value if len(value) <= 64 else value[:61] + "..."
    if text.isprintable() and text.isascii():
        return "'" + text + "'"
    return ascii(text)


def _ipv4(name: str, value: str, problems: list[str], what: str) -> ipaddress.IPv4Address | None:
    if not _IPV4_RE.fullmatch(value):
        problems.append(f"{name}={shown(value)} is not an IPv4 address; accepted: {what} in dotted "
                        "decimal without leading zeros, for example 192.168.1.200")
        return None
    address = ipaddress.IPv4Address(value)
    if address.is_unspecified or address.is_multicast or address.is_reserved or \
            address == ipaddress.IPv4Address("255.255.255.255"):
        problems.append(f"{name}={shown(value)} is not a unicast address; accepted: {what}, not "
                        "0.0.0.0, multicast (224-239.x.x.x), 240.x.x.x and above or broadcast")
        return None
    return address


def _port(name: str, value: str, problems: list[str], default_text: str) -> int | None:
    if not _PORT_RE.fullmatch(value) or not MIN_PORT <= int(value) <= MAX_PORT:
        problems.append(f"{name}={shown(value)} is not a usable UDP port; accepted: a whole number "
                        f"{MIN_PORT}-{MAX_PORT} without leading zeros ({default_text})")
        return None
    return int(value)


def _false_only(name: str, value: str, problems: list[str], why: str) -> None:
    word = value.lower()
    if word in FALSE_WORDS:
        return
    if word in TRUE_WORDS:
        problems.append(f"{name}={shown(value)} is not supported by this image; accepted: unset or "
                        f"a false value ({', '.join(FALSE_WORDS)}). {why}")
        return
    problems.append(f"{name}={shown(value)} is not a boolean; accepted: unset or a false value "
                    f"({', '.join(FALSE_WORDS)}, any case). {why}")


def parse(env: Mapping[str, str]) -> Settings:
    """Validate the environment. Raises Refused with one line per problem."""
    problems: list[str] = []
    sources: dict[str, str] = {}

    def get(name: str) -> str | None:
        value = env.get(name)
        if value is None:
            sources[name] = "default"
            return None
        sources[name] = "env"
        return value

    # Empty values: a template that rendered nothing must not fall back to a default silently.
    for name in REQUIRED + OPTIONAL:  # an empty DUAL_LIDAR means false, as in the Seyond image
        if name in env and env[name] == "":
            problems.append(f"{name} is set but empty; accepted: {ACCEPTED[name]}"
                            + ("" if name in REQUIRED else ", or unset it for the default"))
    present = {k: v for k, v in env.items() if k in KNOWN and v != ""}

    # LIDAR_MODEL
    model = None
    raw = present.get("LIDAR_MODEL")
    if "LIDAR_MODEL" not in env:
        problems.append("LIDAR_MODEL is not set; accepted (any case, '_' or '-', optional "
                        f"'robosense-' prefix, 'rs' optional before a letter): {model_list()}")
    elif raw is not None:
        key = normalise_model(raw)
        if key is None:
            problems.append(f"LIDAR_MODEL={shown(raw)} is not a RoboSense model this image knows; "
                            "accepted (any case, '_' or '-', optional 'robosense-' prefix, 'rs' "
                            f"optional before a letter): {model_list()}")
        else:
            model = MODELS[key]
    get("LIDAR_MODEL")

    # SENSOR_IP
    sensor = None
    if "SENSOR_IP" not in env:
        problems.append("SENSOR_IP is not set; accepted: the sensor's own IPv4 address, for example "
                        "192.168.1.200 (the RoboSense factory address)")
    elif "SENSOR_IP" in present:
        sensor = _ipv4("SENSOR_IP", present["SENSOR_IP"], problems, "the sensor's own IPv4 address")
    get("SENSOR_IP")

    # SENSOR_ID
    sensor_id = None
    value = get("SENSOR_ID")
    if value:
        if not _SENSOR_ID_RE.fullmatch(value) or "__" in value:
            problems.append(f"SENSOR_ID={shown(value)} is not a sensor id; accepted: 1-31 characters, "
                            "a lowercase letter then lowercase letters, digits or single '_' (it "
                            "becomes the node's ROS namespace)")
        else:
            sensor_id = value

    # DATA_PORT / DEVICE_PORT. The sensor sends MSOP (points) and DIFOP (device and calibration)
    # packets to two ports set on the sensor. The driver cannot tell senders apart, so a second
    # sensor needs its own pair: a moved DATA_PORT with the default DEVICE_PORT would share one
    # DIFOP port between two sensors (and calibrate one with the other's angles), so it is refused.
    data_port = DEFAULT_DATA_PORT
    value = get("DATA_PORT")
    data_ok = True
    if value:
        port = _port("DATA_PORT", value, problems, f"default {DEFAULT_DATA_PORT}")
        data_ok = port is not None
        data_port = port if port is not None else data_port
    device_port = None
    value = get("DEVICE_PORT")
    if value:
        device_port = _port("DEVICE_PORT", value, problems,
                            f"default {DEFAULT_DEVICE_PORT}, 7766 for rsemx; not DATA_PORT")
    elif "DEVICE_PORT" not in env and data_ok:
        if data_port != DEFAULT_DATA_PORT:
            problems.append(f"DEVICE_PORT is not set and DATA_PORT={data_port} is not the default "
                            f"{DEFAULT_DATA_PORT}; accepted: set DEVICE_PORT too, to the DIFOP port "
                            "this sensor is set to send to (each sensor on a host needs its own "
                            "DATA_PORT and DEVICE_PORT)")
        elif model is not None:
            device_port = model.difop_port
    if data_ok and device_port is not None and device_port == data_port:
        problems.append(f"DEVICE_PORT={device_port} is the same as DATA_PORT; accepted: a different "
                        "port, the sensor sends data (MSOP) and device (DIFOP) packets to two ports")

    # HOST_IP
    host_ip = None
    value = get("HOST_IP")
    if value:
        address, _, prefix = value.partition("/")
        if "/" in value and not _PREFIX_RE.fullmatch(prefix):
            problems.append(f"HOST_IP={shown(value)} has no usable prefix length; accepted: the "
                            "host's IPv4 address on the sensor network, optionally with /1-/32, for "
                            "example 192.168.1.102/24")
        else:
            host = _ipv4("HOST_IP", address, problems,
                         "the host's IPv4 address on the sensor network (optionally with /1-/32)")
            if host is not None:
                host_ip = value
                if sensor is not None and host == sensor:
                    problems.append(f"HOST_IP={shown(value)} is the sensor's own address; accepted: "
                                    "the host's address on the sensor network")
                    host_ip = None
                elif sensor is not None and prefix:
                    network = ipaddress.IPv4Interface(value).network
                    if sensor not in network:
                        problems.append(f"HOST_IP={shown(value)}: the sensor at {sensor} is not in "
                                        f"the host's subnet {network}; accepted: a host address in "
                                        "the sensor's subnet")
                        host_ip = None

    # FRAME_ID
    frame_id = DEFAULT_FRAME_ID
    value = get("FRAME_ID")
    if value:
        if not _FRAME_RE.fullmatch(value):
            problems.append(f"FRAME_ID={shown(value)} is not a frame id; accepted: 1-64 characters, "
                            "letters, digits and '_', not starting with a digit (default "
                            f"{DEFAULT_FRAME_ID})")
        else:
            frame_id = value

    # POINTCLOUD_TOPIC
    topic = DEFAULT_TOPIC
    value = get("POINTCLOUD_TOPIC")
    if value:
        if not _TOPIC_RE.fullmatch(value) or "__" in value or len(value) > 200:
            problems.append(f"POINTCLOUD_TOPIC={shown(value)} is not a ROS topic name; accepted: "
                            "'/'-separated tokens of letters, digits and single '_', each starting "
                            "with a letter, absolute (/x) or relative to the SENSOR_ID namespace "
                            f"(default {DEFAULT_TOPIC})")
        else:
            topic = value

    # TIME_MODE
    time_mode = "host"
    value = get("TIME_MODE")
    if value:
        word = value.lower()
        if word == "ntp":
            problems.append(f"TIME_MODE={shown(value)} is not supported by this image; accepted: "
                            "host (the default), or sensor, gps or ptp (stamp with the sensor's own "
                            "clock); RoboSense sensors take time from GPS (PPS and NMEA) or PTP, "
                            "set on the sensor, not from NTP")
        elif word not in TIME_MODES:
            problems.append(f"TIME_MODE={shown(value)} is not a time mode; accepted, any case: host "
                            "(stamp with the host's receive time, the default), sensor, gps or ptp "
                            "(stamp with the sensor's own clock; the sensor must be set to that "
                            "time source on its web page)")
        else:
            time_mode = word

    # PUBLISH_SCAN: the driver has no LaserScan output.
    value = get("PUBLISH_SCAN")
    if value:
        _false_only("PUBLISH_SCAN", value, problems,
                    "rslidar_sdk publishes point clouds only, no laser scan")

    # RSLIDAR_RUN_DIR
    run_dir = DEFAULT_RUN_DIR
    value = get("RSLIDAR_RUN_DIR")
    if value:
        if not value.startswith("/") or not re.fullmatch(r"/[A-Za-z0-9_./-]*", value) or \
                "/../" in value + "/":
            problems.append(f"RSLIDAR_RUN_DIR={shown(value)} is not an absolute directory path; "
                            "accepted: an absolute path of letters, digits, '_', '.', '-' and '/' "
                            f"(default {DEFAULT_RUN_DIR})")
        else:
            run_dir = value

    # DUAL_LIDAR (the Seyond image's flag)
    value = env.get("DUAL_LIDAR")
    if value:
        _false_only("DUAL_LIDAR", value, problems, "Run one container per RoboSense sensor")

    # Unknown variables under this image's prefixes.
    for name in sorted(env):
        if name in KNOWN or not name.startswith(STRICT_PREFIXES):
            continue
        close = [ALIASES[name]] if name in ALIASES else difflib.get_close_matches(name, KNOWN, n=1,
                                                                                    cutoff=0.6)
        hint = f" (did you mean {close[0]}?)" if close else ""
        problems.append(f"{name} is not a variable this image knows{hint}; accepted: "
                        f"{', '.join(KNOWN)}")

    if problems:
        raise Refused(problems)
    assert model is not None and sensor is not None and device_port is not None
    return Settings(model=model, sensor_ip=str(sensor), sensor_id=sensor_id, data_port=data_port,
                    device_port=device_port, host_ip=host_ip, frame_id=frame_id, topic=topic,
                    time_mode=time_mode, run_dir=run_dir, sources=sources)


# --------------------------------------------------------------------------------------------
# Rendering: the vendor's config/config.yaml with the contract's keys replaced.
# --------------------------------------------------------------------------------------------


def contract_values(s: Settings) -> list[tuple[str, str, str]]:
    """(key, YAML value, why) for every key of the vendor file the contract sets, in file order."""
    return [
        ("msg_source", "1", "online sensor (UDP)"),
        ("send_packet_ros", "false", "no packet topic"),
        ("send_point_cloud_ros", "true", "point clouds on ROS 2"),
        ("lidar_type", s.model.lidar_type, f"LIDAR_MODEL {s.model.key} = {s.model.label}"),
        ("msop_port", str(s.data_port), "DATA_PORT"),
        ("difop_port", str(s.device_port), "DEVICE_PORT"),
        ("imu_port", "0", "no IMU (the image is built without IMU parsing)"),
        ("max_distance", repr(s.model.max_distance), "the vendor's limit for this model"),
        ("use_lidar_clock", "false" if s.time_mode == "host" else "true", f"TIME_MODE {s.time_mode}"),
        ("ros_frame_id", json.dumps(s.frame_id), "FRAME_ID"),
        ("ros_send_point_cloud_topic", json.dumps(s.topic), "POINTCLOUD_TOPIC"),
    ]


def _key_re(key: str) -> re.Pattern:
    return re.compile(rf"^([ \t]*(?:- )?{re.escape(key)}:)[ \t]*[^#\n]*?[ \t]*(#.*)?$", re.M)


def render(s: Settings, vendor: str) -> str:
    """The vendor file with the contract's keys replaced. Raises ValueError when a key is missing
    from the vendor file or there more than once (the image build runs selfcheck for this)."""
    text = vendor
    for key, value, why in contract_values(s):
        pattern = _key_re(key)
        found = pattern.findall(text)
        if len(found) != 1:
            raise ValueError(f"the vendor config has {len(found)} '{key}:' lines, expected 1")
        # The vendor's comment stays; what the contract set and why is in the header.
        text = pattern.sub(lambda m: f"{m.group(1)} {value}" + (f"  {m.group(2)}" if m.group(2) else ""),
                           text)
    head = [
        "# Rendered from the environment by the rslidar-driver image entrypoint; do not edit.",
        f"# LIDAR_MODEL {s.model.key} = {s.model.label} ({s.model.kind}, lidar_type "
        f"{s.model.lidar_type}); SENSOR_IP {s.sensor_ip} (the driver listens on its ports for any "
        "sender; it does not filter by address)",
        f"# TIME_MODE {s.time_mode}: "
        + ("points are stamped with the host's receive time" if s.time_mode == "host" else
           "points are stamped with the sensor's own clock; the sensor must be set to this time "
           "source"),
    ]
    if s.host_ip:
        head.append(f"# HOST_IP {s.host_ip}: checked against SENSOR_IP; the driver listens on all "
                    "addresses")
    head.append(f"# topic {s.resolved_topic}, frame {s.frame_id}")
    return "\n".join(head) + "\n" + text


def node_argv(s: Settings) -> list[str]:
    argv = [DRIVER_NODE, "--ros-args"]
    if s.sensor_id:
        argv += ["-r", f"__ns:={s.namespace}"]
    argv += ["-p", f"config_path:={s.config_path}"]
    return argv


def describe(s: Settings) -> dict:
    return {
        "model": {"key": s.model.key, "label": s.model.label, "lidar_type": s.model.lidar_type},
        "sensor_ip": s.sensor_ip,
        "data_port": s.data_port,
        "device_port": s.device_port,
        "topic": s.resolved_topic,
        "frame_id": s.frame_id,
        "time_mode": s.time_mode,
        "config_file": s.config_path,
        "config": {k: v for k, v, _ in contract_values(s)},
        "argv": node_argv(s),
        "sources": dict(sorted(s.sources.items())),
    }


# --------------------------------------------------------------------------------------------
# Command line.
# --------------------------------------------------------------------------------------------


def refuse(problems: list[str], as_json: bool = False) -> int:
    if as_json:
        print(json.dumps({"refused": problems}, indent=2))
    for problem in problems:
        print(f"{PROG}: refusing to start: {problem}", file=sys.stderr)
    return EXIT_REFUSED


def read_vendor() -> str:
    with open(VENDOR_CONFIG, encoding="utf-8") as handle:
        return handle.read()


def write_config(s: Settings, text: str) -> None:
    os.makedirs(s.run_dir, mode=0o700, exist_ok=True)
    tmp = s.config_path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as handle:
        handle.write(text)
    os.replace(tmp, s.config_path)


def exec_env(env: Mapping[str, str]) -> dict[str, str]:
    """The driver's environment: ROS_HOME and ROS_LOG_DIR default under /tmp, so an arbitrary
    uid with a read-only root filesystem and a tmpfs on /tmp can log."""
    out = dict(env)
    out.setdefault("ROS_HOME", DEFAULT_ROS_HOME)
    out.setdefault("ROS_LOG_DIR", out["ROS_HOME"].rstrip("/") + "/log")
    return out


def _settings_and_text(env: Mapping[str, str], as_json: bool = False):
    try:
        s = parse(env)
    except Refused as exc:
        return None, None, refuse(exc.problems, as_json)
    try:
        text = render(s, read_vendor())
    except (OSError, ValueError) as exc:
        return None, None, refuse([f"cannot render the driver config from the vendor file "
                                   f"({exc}); a broken image"], as_json)
    return s, text, EXIT_OK


def cmd_run(env: Mapping[str, str]) -> int:
    s, text, code = _settings_and_text(env)
    if s is None:
        return code
    child_env = exec_env(env)
    try:
        write_config(s, text)
        os.makedirs(child_env["ROS_LOG_DIR"], exist_ok=True)
    except OSError as exc:
        return refuse([f"cannot write {exc.filename or s.run_dir} ({exc.strerror}) as uid "
                       f"{os.getuid()}; accepted: a writable /tmp, for example docker run "
                       "--tmpfs /tmp or compose 'tmpfs: [/tmp]' with a read-only root filesystem "
                       "(or point RSLIDAR_RUN_DIR and ROS_LOG_DIR at a writable volume)"])
    print(f"{PROG}: RoboSense {s.model.label} at {s.sensor_ip}, data port {s.data_port}, device "
          f"port {s.device_port}, topic {s.resolved_topic}, frame {s.frame_id}, time "
          f"{s.time_mode}, config {s.config_path}", file=sys.stderr, flush=True)
    argv = node_argv(s)
    try:
        os.execve(argv[0], argv, child_env)
    except OSError as exc:
        print(f"{PROG}: cannot start the driver {argv[0]}: {exc.strerror} (a broken image)",
              file=sys.stderr)
    return EXIT_REFUSED  # reached only when execve failed


def cmd_check(env: Mapping[str, str], as_json: bool) -> int:
    s, text, code = _settings_and_text(env, as_json)
    if s is None:
        return code
    if as_json:
        out = describe(s)
        out["config_yaml"] = text
        print(json.dumps(out, indent=2))
    else:
        sys.stdout.write(text)
        print("# exec: " + " ".join(node_argv(s)))
    return EXIT_OK


def cmd_selfcheck(env: Mapping[str, str]) -> int:
    """Every model renders against the vendor file in this image, and (when PyYAML is there, as
    in the image) the rendered file parses and holds the contract's values."""
    vendor = read_vendor()
    try:
        import yaml  # noqa: PLC0415 - optional, present in the ROS image
    except ImportError:
        yaml = None
    for model in MODELS_LIST:
        s = parse({"LIDAR_MODEL": model.key, "SENSOR_IP": "192.168.1.200"})
        text = render(s, vendor)
        if yaml is not None:
            doc = yaml.safe_load(text)
            drv = doc["lidar"][0]["driver"]
            ros = doc["lidar"][0]["ros"]
            assert len(doc["lidar"]) == 1
            assert doc["common"]["msg_source"] == 1 and doc["common"]["send_packet_ros"] is False
            assert drv["lidar_type"] == model.lidar_type and drv["msop_port"] == DEFAULT_DATA_PORT
            assert drv["difop_port"] == model.difop_port and drv["use_lidar_clock"] is False
            assert ros["ros_frame_id"] == DEFAULT_FRAME_ID
            assert ros["ros_send_point_cloud_topic"] == DEFAULT_TOPIC
    print(f"{PROG}: selfcheck ok: {len(MODELS_LIST)} models render against {VENDOR_CONFIG}"
          + ("" if yaml is not None else " (YAML not parsed: no PyYAML)"))
    return EXIT_OK


USAGE = (f"usage: {PROG} [run | check [--json] | selfcheck]  (configuration from the environment; "
         "see docs/METROLLA_DRIVER_CONTRACT.md)")


# The image's CMD (Dockerfile): with the switch on it means "run", so an image started without a
# command runs the contract.
IMAGE_CMD = ["ros2", "launch", "rslidar_sdk", "start.py"]


def main(argv: list[str], env: Mapping[str, str]) -> int:
    if not argv or argv == ["run"] or argv == IMAGE_CMD:
        return cmd_run(env)
    if argv[0] == "check" and argv[1:] in ([], ["--json"]):
        return cmd_check(env, argv[1:] == ["--json"])
    if argv == ["selfcheck"]:
        return cmd_selfcheck(env)
    if argv[0] in ("-h", "--help", "help"):
        print(USAGE)
        return EXIT_OK
    print(f"{PROG}: unknown command {shown(' '.join(argv))}; {USAGE}. To run something else in "
          "this image, override the entrypoint", file=sys.stderr)
    return EXIT_REFUSED


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:], os.environ))
