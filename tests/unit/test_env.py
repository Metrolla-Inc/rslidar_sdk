"""The contract (docker/rslidar_env.py): variables, defaults, refusals, the rendered config and
the exec line. Rendered against this repository's config/config.yaml, the file the image uses."""

from __future__ import annotations

import json
import os

import pytest
import yaml

from contract_run import REPO, rslidar_env as r, run

BASE = {"LIDAR_MODEL": "rshelios", "SENSOR_IP": "192.168.1.200"}


def settings(**extra):
    env = dict(BASE)
    env.update(extra)
    return r.parse(env)


def problems(**extra) -> list[str]:
    env = dict(BASE)
    env.update(extra)
    env = {k: v for k, v in env.items() if v is not None}
    with pytest.raises(r.Refused) as exc:
        r.parse(env)
    return exc.value.problems


def rendered(**extra) -> dict:
    s = settings(**extra)
    return yaml.safe_load(r.render(s, (REPO / "config" / "config.yaml").read_text()))


# --- models ----------------------------------------------------------------------------------

def test_every_driver_lidar_type_is_a_model():
    """Every type rs_driver's strToLidarType accepts (the submodule in this checkout)."""
    header = (REPO / "src/rs_driver/src/rs_driver/driver/driver_param.hpp").read_text()
    table = header[header.index("strToLidarType"):]
    table = table[:table.index("};")]
    import re
    types = set(re.findall(r'\{"(RS[A-Z0-9_]+)"', table))
    assert types == {m.lidar_type for m in r.MODELS_LIST}


@pytest.mark.parametrize("value,lidar_type", [
    ("rshelios", "RSHELIOS"), ("RSHELIOS", "RSHELIOS"), ("helios", "RSHELIOS"),
    ("RS-Helios", "RSHELIOS"), ("robosense-rshelios", "RSHELIOS"), ("robosense-helios", "RSHELIOS"),
    ("rshelios_16p", "RSHELIOS_16P"), ("Helios-16P", "RSHELIOS_16P"), ("rs128", "RS128"),
    ("RS-LiDAR-M1", "RSM1"), ("m1", "RSM1"), ("rsm1-jumbo", "RSM1_JUMBO"), ("bp", "RSBP"),
    ("rsairylite_eth", "RSAIRYLITE_ETH"), ("emx", "RSEMX"), ("rs16", "RS16"),
])
def test_model_names(value, lidar_type):
    assert settings(LIDAR_MODEL=value).model.lidar_type == lidar_type


@pytest.mark.parametrize("value", ["128", "16", "helios32", "ouster-os1", "robosense", "rs", "bpearl",
                                   "rs_helios_x", "hesai-xt32"])
def test_unknown_models_refused_with_the_list(value):
    (p,) = problems(LIDAR_MODEL=value)
    assert p.startswith(f"LIDAR_MODEL='{value}' is not a RoboSense model") and "rshelios" in p


def test_emx_defaults():
    s = settings(LIDAR_MODEL="rsemx")
    assert s.device_port == 7766 and s.model.max_distance == 300.0
    assert rendered(LIDAR_MODEL="rsemx")["lidar"][0]["driver"]["max_distance"] == 300.0


# --- required, empty, unknown ------------------------------------------------------------------

def test_nothing_set():
    with pytest.raises(r.Refused) as exc:
        r.parse({})
    assert [p.split(" ")[0] for p in exc.value.problems] == ["LIDAR_MODEL", "SENSOR_IP"]


@pytest.mark.parametrize("name", r.REQUIRED + r.OPTIONAL)
def test_empty_values_refused(name):
    env = dict(BASE)
    env[name] = ""
    with pytest.raises(r.Refused) as exc:
        r.parse(env)
    assert any(p.startswith(f"{name} is set but empty") for p in exc.value.problems)


def test_empty_dual_lidar_is_false():
    settings(DUAL_LIDAR="")


@pytest.mark.parametrize("name,hint", sorted(r.ALIASES.items()))
def test_legacy_and_vendor_names_refused_with_the_contract_name(name, hint):
    (p,) = problems(**{name: "x"})
    assert p.startswith(f"{name} is not a variable this image knows (did you mean {hint}?)")


@pytest.mark.parametrize("name", ["SENSOR_IPP", "LIDAR_MODLE", "DATA_PROT", "RSLIDAR_RUNDIR",
                                  "FRAME_IDS", "TIME_MOD"])
def test_typos_refused(name):
    (p,) = problems(**{name: "x"})
    assert p.startswith(f"{name} is not a variable this image knows (did you mean ")


def test_other_variables_pass():
    settings(ROS_DOMAIN_ID="3", RMW_IMPLEMENTATION="rmw_fastrtps_cpp", METROLLA_UNIT="x",
             ROS_LOCALHOST_ONLY="1", PATH="/bin")


# --- values ------------------------------------------------------------------------------------

@pytest.mark.parametrize("value", ["192.168.1", "192.168.001.200", "0.0.0.0", "224.0.0.1",
                                   "255.255.255.255", "lidar.local", "240.0.0.1", " 192.168.1.200"])
def test_bad_sensor_ip(value):
    (p,) = problems(SENSOR_IP=value)
    assert p.startswith(f"SENSOR_IP={r.shown(value)}")


def test_ports_default():
    s = settings()
    assert (s.data_port, s.device_port) == (6699, 7788)


def test_ports_set():
    s = settings(DATA_PORT="6700", DEVICE_PORT="7789")
    assert (s.data_port, s.device_port) == (6700, 7789)
    drv = rendered(DATA_PORT="6700", DEVICE_PORT="7789")["lidar"][0]["driver"]
    assert (drv["msop_port"], drv["difop_port"]) == (6700, 7789)


def test_moved_data_port_needs_a_device_port():
    (p,) = problems(DATA_PORT="6700")
    assert p.startswith("DEVICE_PORT is not set and DATA_PORT=6700 is not the default 6699")


def test_default_data_port_written_out_keeps_the_default_device_port():
    assert settings(DATA_PORT="6699").device_port == 7788


@pytest.mark.parametrize("value", ["0", "80", "1023", "65536", "06699", "6699.0", "x"])
def test_bad_ports(value):
    p = problems(DATA_PORT=value, DEVICE_PORT="7788")
    assert p[0].startswith(f"DATA_PORT={r.shown(value)} is not a usable UDP port")
    p = problems(DEVICE_PORT=value)
    assert p[0].startswith(f"DEVICE_PORT={r.shown(value)} is not a usable UDP port")


def test_same_ports_refused():
    (p,) = problems(DATA_PORT="7000", DEVICE_PORT="7000")
    assert p.startswith("DEVICE_PORT=7000 is the same as DATA_PORT")


def test_host_ip():
    assert settings(HOST_IP="192.168.1.102").host_ip == "192.168.1.102"
    assert settings(HOST_IP="192.168.1.102/24").host_ip == "192.168.1.102/24"
    assert problems(HOST_IP="192.168.1.200")[0].startswith("HOST_IP='192.168.1.200' is the sensor's")
    assert "not in the host's subnet 10.0.0.0/24" in problems(HOST_IP="10.0.0.2/24")[0]
    assert "no usable prefix length" in problems(HOST_IP="192.168.1.102/33")[0]
    # HOST_IP is checked, not used for binding: broadcast-destination sensors keep working.
    assert "host_address" not in rendered(HOST_IP="192.168.1.102/24")["lidar"][0]["driver"]


def test_frame_and_topic_defaults():
    s = settings()
    assert (s.frame_id, s.resolved_topic) == ("rslidar", "/rslidar_points")
    s = settings(SENSOR_ID="east")
    assert (s.frame_id, s.resolved_topic) == ("rslidar", "/east/rslidar_points")
    s = settings(SENSOR_ID="east", POINTCLOUD_TOPIC="/lidar/points", FRAME_ID="east_lidar")
    assert (s.frame_id, s.resolved_topic) == ("east_lidar", "/lidar/points")
    ros = rendered(SENSOR_ID="east", POINTCLOUD_TOPIC="points", FRAME_ID="true")["lidar"][0]["ros"]
    assert ros["ros_send_point_cloud_topic"] == "points" and ros["ros_frame_id"] == "true"


@pytest.mark.parametrize("name,value", [("SENSOR_ID", "East"), ("SENSOR_ID", "a__b"), ("SENSOR_ID", "1a"),
                                        ("FRAME_ID", "1frame"), ("FRAME_ID", "a/b"), ("FRAME_ID", "a b"),
                                        ("POINTCLOUD_TOPIC", "a//b"), ("POINTCLOUD_TOPIC", "~/x"),
                                        ("POINTCLOUD_TOPIC", "a__b"), ("POINTCLOUD_TOPIC", "1x"),
                                        ("RSLIDAR_RUN_DIR", "tmp/x"), ("RSLIDAR_RUN_DIR", "/tmp/../etc")])
def test_bad_names(name, value):
    (p,) = problems(**{name: value})
    assert p.startswith(f"{name}={r.shown(value)}")


@pytest.mark.parametrize("value,clock", [("host", False), ("HOST", False), ("sensor", True),
                                         ("gps", True), ("PTP", True)])
def test_time_modes(value, clock):
    assert rendered(TIME_MODE=value)["lidar"][0]["driver"]["use_lidar_clock"] is clock


def test_time_mode_default_host():
    assert rendered()["lidar"][0]["driver"]["use_lidar_clock"] is False


@pytest.mark.parametrize("value", ["ntp", "NTP", "utc", "lidar", "1"])
def test_bad_time_modes(value):
    (p,) = problems(TIME_MODE=value)
    assert p.startswith(f"TIME_MODE={r.shown(value)}")


@pytest.mark.parametrize("name", ["PUBLISH_SCAN", "DUAL_LIDAR"])
def test_false_only_flags(name):
    for value in ("0", "false", "No", "OFF"):
        settings(**{name: value})
    for value in ("1", "true", "maybe"):
        (p,) = problems(**{name: value})
        assert p.startswith(f"{name}={r.shown(value)}")


def test_all_problems_at_once():
    p = problems(LIDAR_MODEL="x", SENSOR_IP="y", DATA_PORT="1", TIME_MODE="z", SENSOR_PORT="1")
    assert len(p) == 5


# --- rendering ----------------------------------------------------------------------------------

def test_render_keeps_the_vendor_file_except_the_contract_keys():
    vendor = (REPO / "config" / "config.yaml").read_text()
    out = r.render(settings(), vendor)
    body = out[out.index("common:"):]
    changed = [(a, b) for a, b in zip(vendor.splitlines(), body.splitlines()) if a != b]
    keys = {a.split(":")[0].strip().lstrip("- ") for a, _ in changed}
    assert keys <= {k for k, _, _ in r.contract_values(settings())}
    assert len(vendor.splitlines()) == len(body.splitlines())


def test_render_values():
    doc = rendered(LIDAR_MODEL="rs128", DATA_PORT="6800", DEVICE_PORT="7800", TIME_MODE="ptp",
                   FRAME_ID="west", POINTCLOUD_TOPIC="/west/points")
    common, drv, ros = doc["common"], doc["lidar"][0]["driver"], doc["lidar"][0]["ros"]
    assert common == {"msg_source": 1, "send_packet_ros": False, "send_point_cloud_ros": True}
    assert len(doc["lidar"]) == 1
    assert drv["lidar_type"] == "RS128" and drv["msop_port"] == 6800 and drv["difop_port"] == 7800
    assert drv["imu_port"] == 0 and drv["use_lidar_clock"] is True
    assert drv["min_distance"] == 0.2 and drv["max_distance"] == 200.0
    assert drv["dense_points"] is False and drv["ts_first_point"] is True
    assert ros["ros_frame_id"] == "west" and ros["ros_send_point_cloud_topic"] == "/west/points"


def test_render_refuses_a_missing_or_doubled_key():
    vendor = (REPO / "config" / "config.yaml").read_text()
    with pytest.raises(ValueError, match="0 'difop_port:' lines"):
        r.render(settings(), vendor.replace("difop_port:", "difop_prt:"))
    with pytest.raises(ValueError, match="2 'msop_port:' lines"):
        r.render(settings(), vendor + "      msop_port: 1\n")


def test_selfcheck_every_model():
    code, out, _ = run(["selfcheck"], {})
    assert code == 0 and "21 models" in out


def test_exec_line():
    assert r.node_argv(settings()) == [r.DRIVER_NODE, "--ros-args", "-p",
                                       "config_path:=/tmp/rslidar/config.yaml"]
    assert r.node_argv(settings(SENSOR_ID="east", RSLIDAR_RUN_DIR="/run/x")) == [
        r.DRIVER_NODE, "--ros-args", "-r", "__ns:=/east", "-p", "config_path:=/run/x/config.yaml"]


# --- command line --------------------------------------------------------------------------------

def test_check_and_json():
    code, out, err = run(["check"], dict(BASE))
    assert code == 0 and err == "" and out.splitlines()[-1].startswith("# exec: ")
    code, out, err = run(["check", "--json"], dict(BASE))
    doc = json.loads(out)
    assert code == 0 and doc["topic"] == "/rslidar_points" and doc["device_port"] == 7788
    assert yaml.safe_load(doc["config_yaml"])["lidar"][0]["driver"]["lidar_type"] == "RSHELIOS"


def test_refusal_output():
    code, out, err = run(["check", "--json"], {"SENSOR_IP": "x"})
    assert code == 1 and len(json.loads(out)["refused"]) == 2
    assert all(line.startswith("rslidar-driver: refusing to start: ") for line in err.splitlines())


@pytest.mark.parametrize("argv", [["start"], ["check", "--yaml"], ["bash"], ["ros2", "launch", "x"]])
def test_unknown_commands_refused(argv):
    code, _, err = run(argv, dict(BASE))
    assert code == 1 and "unknown command" in err


def test_run_writes_the_config_and_execs(tmp_path, monkeypatch):
    calls = []

    def fake_exec(path, argv, env):
        calls.append((path, argv, env))
        raise OSError(2, "stopped here by the test")

    monkeypatch.setattr(os, "execve", fake_exec)
    env = dict(BASE, RSLIDAR_RUN_DIR=str(tmp_path / "run"), ROS_HOME=str(tmp_path / "ros"))
    for argv in ([], ["run"], ["ros2", "launch", "rslidar_sdk", "start.py"]):
        code, _, err = run(argv, env)
        assert code == 1 and "cannot start the driver" in err
    path, argv, child = calls[-1]
    assert path == r.DRIVER_NODE and argv[-1] == f"config_path:={tmp_path}/run/config.yaml"
    assert child["ROS_LOG_DIR"] == f"{tmp_path}/ros/log" and os.path.isdir(child["ROS_LOG_DIR"])
    text = (tmp_path / "run" / "config.yaml").read_text()
    assert yaml.safe_load(text)["lidar"][0]["driver"]["lidar_type"] == "RSHELIOS"
    assert len(calls) == 3


def test_run_refuses_an_unwritable_dir(tmp_path):
    ro = tmp_path / "ro"
    ro.mkdir()
    ro.chmod(0o500)
    try:
        if os.access(ro, os.W_OK):
            pytest.skip("running as root")
        code, _, err = run([], dict(BASE, RSLIDAR_RUN_DIR=str(ro / "x")))
        assert code == 1 and "cannot write" in err and "tmpfs" in err
    finally:
        ro.chmod(0o700)
