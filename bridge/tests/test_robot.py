import asyncio
from pathlib import Path

import pytest
from bridge_fakes import HARDWARE_YAML, FakeRosbridge, lucy_config

from huri_lucy_bridge.robot import DiscoveryError, discover, parse_hardware_config
from huri_lucy_bridge.rosbridge import RosbridgeClient

FIXTURES = Path(__file__).parent / "fixtures"


def test_controllers_and_joints_are_read_as_lucy_reads_them():
    robot = parse_hardware_config(HARDWARE_YAML, "testbot_urdf", "default")
    assert (robot.package, robot.name, robot.config_name) == (
        "testbot_urdf",
        "testbot",
        "default",
    )
    # One controller per board that has one, joints in virtual_pin order.
    assert list(robot.controllers) == ["head_controller", "arm_controller"]
    arm = robot.controllers["arm_controller"]
    assert arm.topic == "/arm_controller/joint_trajectory"
    assert arm.joints == ("shoulder_joint", "elbow_joint")
    assert [c["id"] for c in robot.cameras] == ["cam"]


def test_limits_and_rest_are_in_urdf_radians():
    joints = parse_hardware_config(HARDWARE_YAML).joints
    # joint = (servo - offset) * direction * scale
    shoulder = joints["shoulder_joint"]
    assert shoulder.limits == pytest.approx((-1.5708, 1.5708), abs=1e-4)
    assert shoulder.rest == pytest.approx(0.0)
    assert shoulder.controller == "arm_controller"
    assert not shoulder.enabled
    jaw = joints["jaw_joint"]
    assert jaw.limits == pytest.approx((0.0, 1.0))
    assert jaw.clamp(2.0) == 1.0 and jaw.clamp(-1.0) == 0.0


def test_the_real_robots_parse():
    inmoov = parse_hardware_config((FIXTURES / "inmoov_urdf.yaml").read_text())
    assert set(inmoov.controllers) == {
        "left_arm_controller",
        "right_arm_controller",
        "torso_head_controller",
    }
    assert len(inmoov.joints) == 30
    rothead = inmoov.joints["i01.head.rothead_link_joint"]
    assert rothead.limits == pytest.approx((-1.5708, 1.5708), abs=1e-4)
    so_arm = parse_hardware_config((FIXTURES / "so_arm101_urdf.yaml").read_text())
    assert so_arm.controllers["so_arm_controller"].joints == (
        "Rotation",
        "Pitch",
        "Elbow",
        "Wrist_Pitch",
        "Wrist_Roll",
        "Jaw",
    )


def test_a_broken_config_is_a_discovery_error():
    with pytest.raises(DiscoveryError, match="not YAML"):
        parse_hardware_config("boards: [")
    with pytest.raises(DiscoveryError, match="mapping"):
        parse_hardware_config("- a list")


def test_discovery_asks_lucy_for_its_active_config():
    async def main(services):
        async with FakeRosbridge(services=services) as server:
            async with RosbridgeClient(server.url) as ros:
                robot = await discover(ros)
            return robot, server.ops

    robot, ops = asyncio.run(main(lucy_config(HARDWARE_YAML, "testbot_urdf")))
    assert robot.package == "testbot_urdf"
    assert len(robot.joints) == 3
    (call,) = ops
    assert (call["service"], call["type"]) == ("/config/get", "lucy_msgs/srv/GetConfig")
    assert call["args"] == {"robot_package": "", "config_name": ""}


def test_discovery_fails_without_config_services_or_config():
    with pytest.raises(DiscoveryError, match="/config/get"):
        asyncio.run(_discover({}))

    def missing(_):
        return {"success": False, "message": "Configuration not found"}

    with pytest.raises(DiscoveryError, match="not found"):
        asyncio.run(_discover({"/config/get": missing}))


async def _discover(services):
    async with FakeRosbridge(services=services) as server:
        async with RosbridgeClient(server.url) as ros:
            return await discover(ros)
