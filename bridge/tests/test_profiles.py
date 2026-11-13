import logging
from pathlib import Path

import pytest
from bridge_fakes import HARDWARE_YAML

from huri_lucy_bridge.gestures import SMPLX_JOINTS
from huri_lucy_bridge.profiles import (
    GESTURES,
    SPEECH_MOTION,
    ProfileError,
    packaged_profiles,
    parse_profile,
    resolve,
    select_profile,
)
from huri_lucy_bridge.robot import parse_hardware_config

FIXTURES = Path(__file__).parent / "fixtures"


def robot(name):
    return parse_hardware_config((FIXTURES / f"{name}.yaml").read_text(), name)


def test_the_robots_lucy_ships_have_a_profile():
    assert {"inmoov_urdf", "inmoov_i2_urdf", "so_arm101_urdf"} <= set(
        packaged_profiles()
    )


@pytest.mark.parametrize("package", ["inmoov_urdf", "so_arm101_urdf"])
def test_each_packaged_profile_fits_its_real_robot(package, caplog):
    # Every joint a profile names exists in the robot Lucy describes.
    real = robot(package)
    with caplog.at_level(logging.WARNING, "huri_lucy_bridge.profiles"):
        caps = resolve(select_profile(real), real)
    assert caplog.text == ""
    assert caps.names == {GESTURES, SPEECH_MOTION}
    assert len(caps.gestures) == len(packaged_profiles()[package].gestures)


def test_the_inmoov_profile_moves_the_jaw_and_the_upper_body():
    caps = resolve(select_profile(robot("inmoov_urdf")), robot("inmoov_urdf"))
    speech, jaw = caps.speech
    assert jaw.name == "i01.head.jaw_link_joint"
    moved = {joint.name for _, joint in caps.gestures}
    assert "i01.head.rothead_link_joint" in moved
    assert "right_elbow_x_link_joint" in moved


TESTBOT = """
version: 1
robot_packages: [testbot_urdf]
gestures:
  max_speed: 2.0
  joints:
    elbow_joint: {sources: [right_elbow.y], scale: 1.0}
    knee_joint: {sources: [right_knee.x], scale: 1.0, max_speed: 0.5}
speech: {joint: mouth_joint, open: 0.2}
"""


def test_joints_the_robot_lacks_are_left_out(caplog):
    import yaml

    profile = parse_profile(yaml.safe_load(TESTBOT), "testbot.yaml")
    testbot = parse_hardware_config(HARDWARE_YAML, "testbot_urdf")
    with caplog.at_level(logging.WARNING, "huri_lucy_bridge.profiles"):
        caps = resolve(profile, testbot)
    assert [joint.name for _, joint in caps.gestures] == ["elbow_joint"]
    assert caps.speech is None  # no mouth_joint: no speech motion
    assert caps.names == {GESTURES}
    assert "knee_joint" in caplog.text and "mouth_joint" in caplog.text
    (gesture,) = [g for g, _ in caps.gestures]
    assert gesture.sources == ((SMPLX_JOINTS.index("right_elbow"), 1),)
    assert gesture.max_speed == 2.0 and gesture.ref == 0.0


def test_no_profile_and_no_robot_mean_no_capability():
    testbot = parse_hardware_config(HARDWARE_YAML, "testbot_urdf")
    assert select_profile(testbot) is None  # no packaged profile
    assert resolve(None, testbot).names == frozenset()
    assert resolve(select_profile(robot("inmoov_urdf")), None).names == frozenset()
    assert select_profile(robot("inmoov_urdf"), "none") is None


def test_an_override_file_wins(tmp_path, caplog):
    path = tmp_path / "mine.yaml"
    path.write_text(TESTBOT)
    with caplog.at_level(logging.WARNING, "huri_lucy_bridge.profiles"):
        profile = select_profile(robot("inmoov_urdf"), str(path))
    assert profile.robot_packages == ("testbot_urdf",)
    assert "not inmoov_urdf" in caplog.text


@pytest.mark.parametrize(
    "raw, error",
    [
        ({"version": 2, "robot_packages": ["a"]}, "version"),
        ({"version": 1, "robot_packages": []}, "robot_packages"),
        ({"version": 1, "robot_packages": ["a"], "gesture": {}}, "unknown keys"),
        (
            {
                "version": 1,
                "robot_packages": ["a"],
                "gestures": {"joints": {"j": {"sources": ["elbow.y"], "scale": 1}}},
            },
            "SMPL-X",
        ),
        (
            {
                "version": 1,
                "robot_packages": ["a"],
                "gestures": {"joints": {"j": {"sources": ["right_elbow.w"]}}},
            },
            "SMPL-X",
        ),
        (
            {
                "version": 1,
                "robot_packages": ["a"],
                "gestures": {"joints": {"j": {"sources": ["right_elbow.y"]}}},
            },
            "'scale' must be a number",
        ),
        (
            {"version": 1, "robot_packages": ["a"], "speech": {"open": 0.1}},
            "joint must name",
        ),
    ],
)
def test_a_broken_profile_is_refused(raw, error):
    with pytest.raises(ProfileError, match=error):
        parse_profile(raw)
