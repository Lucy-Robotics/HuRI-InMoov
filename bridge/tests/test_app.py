"""The bridge between a fake rosbridge and HuRI's real /session endpoint: the
session handshake and its checks, the binary framing both ways, the routes,
the heartbeat and the reconnection."""

import asyncio
from pathlib import Path

import numpy as np
import pytest
from bridge_fakes import (
    HARDWARE_YAML,
    FakeRosbridge,
    huri_server,
    lucy_config,
    wait_until,
)
from huri_core.core.client import HuRISessionError
from huri_core.core.events import BytesEvent
from huri_core.core.module import Module
from huri_events.audio import Audio
from huri_events.motion import Motion
from omegaconf import OmegaConf

from huri_lucy_bridge.app import (
    HURI_TO_ROS,
    Outbound,
    build,
    connect,
    discover_robot,
    run,
    serve,
)
from huri_lucy_bridge.audio import AUDIO_STAMPED
from huri_lucy_bridge.presence import HEARTBEAT_TOPIC
from huri_lucy_bridge.profiles import GESTURES
from huri_lucy_bridge.robot import parse_hardware_config


class Echo(Module):
    """Stands in for vad..tts: says back every microphone frame it hears."""

    input_type = "huri.audio.in"
    output_type = "huri.audio.out"
    input_event = BytesEvent
    output_event = Audio

    async def process(self, data: BytesEvent) -> Audio:
        pcm = np.frombuffer(data.data, "<i2").astype(np.float32) / 32768
        return Audio(data=pcm, sample_rate=16000)


def config(huri_url, rosbridge_url, **lucy):
    return {
        "user_id": "bridge-test",
        "huri_url": huri_url,
        "send": ["huri.audio.in"],
        "subscribe": ["huri.audio.out"],
        "modules": {"echo": {"name": "echo"}},
        "lucy": {
            "rosbridge_url": rosbridge_url,
            "heartbeat_s": 0.05,
            "discovery_timeout_s": 0,
            **lucy,
        },
    }


def panel_mic(value, samples=1024):
    """A /mic_audio message as the control panel's AudioBridge publishes it."""

    return {
        "header": {"frame_id": "control_panel_mic", "stamp": {"sec": 1, "nanosec": 0}},
        "audio": {
            "info": {"format": 8, "channels": 1, "rate": 16000, "chunk": samples},
            "audio_data": {
                "float32_data": [],
                "int32_data": [],
                "int16_data": [value] * samples,
                "int8_data": [],
                "uint8_data": [],
            },
        },
    }


def test_the_robot_hears_and_speaks_through_a_real_huri_session():
    async def main():
        async with FakeRosbridge() as ros, huri_server({"echo": Echo}) as url:
            bridge = build(config(url, ros.url))
            stop = asyncio.Event()
            session = asyncio.create_task(run(bridge, stop))
            await wait_until(lambda: ros.subscribed("/mic_audio"))
            await ros.publish("/mic_audio", panel_mic(8192))
            await wait_until(lambda: len(ros.published("/audio")) == 2)
            await wait_until(lambda: ros.published(HEARTBEAT_TOPIC))
            stop.set()
            await asyncio.wait_for(session, timeout=5)
            return bridge, ros

    bridge, ros = asyncio.run(main())
    advertised = {op["topic"]: op["type"] for op in ros.ops if op["op"] == "advertise"}
    assert advertised == {
        HEARTBEAT_TOPIC: "std_msgs/msg/String",
        "/audio": AUDIO_STAMPED,
    }
    subscribe = next(op for op in ros.ops if op["op"] == "subscribe")
    assert (subscribe["topic"], subscribe["type"]) == ("/mic_audio", AUDIO_STAMPED)
    # 1024 samples -> two 30 ms frames (64 samples wait for more) -> two replies.
    speech = ros.published("/audio")
    assert [m["audio"]["info"]["chunk"] for m in speech] == [480, 480]
    assert all(m["audio"]["info"]["rate"] == 16000 for m in speech)
    assert all(m["header"]["frame_id"] == "huri_tts" for m in speech)
    pcm = np.concatenate([m["audio"]["audio_data"]["int16_data"] for m in speech])
    assert np.all(pcm == 8192)
    assert {m["data"] for m in ros.published(HEARTBEAT_TOPIC)} == {bridge.client_id}
    assert bridge.client_id.startswith("huri_bridge_")


def test_run_fails_when_rosbridge_goes_away():
    async def main():
        async with FakeRosbridge() as ros, huri_server({"echo": Echo}) as url:
            bridge = build(config(url, ros.url))
            session = asyncio.create_task(run(bridge))
            await wait_until(lambda: ros.connections)
            await ros.connections[0].close()
            await asyncio.wait_for(session, timeout=5)

    with pytest.raises(ConnectionError, match="rosbridge closed"):
        asyncio.run(main())


def test_serve_reconnects_under_the_same_client_id():
    async def main():
        async with FakeRosbridge() as ros, huri_server({"echo": Echo}) as url:
            stop = asyncio.Event()
            serving = asyncio.create_task(
                serve(config(url, ros.url, reconnect_s=0.05), stop)
            )
            # Each attempt: a discovery connection, then the bridge's.
            await wait_until(lambda: ros.published(HEARTBEAT_TOPIC, connection=1))
            await ros.connections[1].close()
            await wait_until(lambda: ros.published(HEARTBEAT_TOPIC, connection=3))
            # The new connection subscribes again, and bridges again.
            await wait_until(lambda: len(ros.subscribed("/mic_audio")) == 2)
            await ros.publish("/mic_audio", panel_mic(100, samples=480))
            await wait_until(lambda: ros.published("/audio", connection=3))
            stop.set()
            await asyncio.wait_for(serving, timeout=5)
            return ros

    ros = asyncio.run(main())
    ids = {m["data"] for m in ros.published(HEARTBEAT_TOPIC)}
    assert len(ids) == 1
    # Lucy described no robot (no config services): voice only, each time.
    assert [op["op"] for op in ros.ops if op["connection"] in (0, 2)] == [
        "call_service",
        "call_service",
    ]


def test_a_refused_session_is_not_retried_and_never_reaches_the_robot():
    async def main():
        async with FakeRosbridge() as ros, huri_server({}) as url:
            try:
                await serve(config(url, ros.url, reconnect_s=0.05), asyncio.Event())
            finally:
                # Discovery asked Lucy, but the bridge never registered.
                assert {op["op"] for op in ros.ops} == {"call_service"}

    with pytest.raises(HuRISessionError):
        asyncio.run(main())


def test_every_huri_topic_needs_a_route():
    raw = config("ws://h", "ws://r")
    raw["subscribe"] = ["huri.audio.out", "huri.transcript"]
    with pytest.raises(ValueError, match="no route for \\['huri.transcript'\\]"):
        build(raw)


def test_a_route_is_on_only_for_its_declared_topic():
    raw = config("ws://h", "ws://r", client_id="lucy_left_ear")
    raw["subscribe"] = []  # a mute robot
    bridge = build(raw)
    assert [r.huri_topic for r in bridge.inbound] == ["huri.audio.in"]
    assert bridge.outbound == []
    assert list(bridge.ros._adverts) == [HEARTBEAT_TOPIC]
    assert bridge.client_id.startswith("lucy_left_ear_")


def test_audio_settings_reach_the_routes():
    raw = config(
        "ws://h",
        "ws://r",
        audio={
            "mic_topic": "/robot/mic",
            "speaker_topic": "/robot/speaker",
            "sample_rate": 8000,
            "frame_duration": 0.020,
            "speaker_chunk": 256,
        },
    )
    bridge = build(raw)
    (mic,) = bridge.inbound
    (speaker,) = bridge.outbound
    assert (mic.ros_topic, mic.sample_rate, mic.frame_size) == ("/robot/mic", 8000, 160)
    assert (speaker.ros_topic, speaker.chunk) == ("/robot/speaker", 256)


def test_the_example_config_builds():
    path = Path(__file__).parents[1] / "config" / "bridge_example.yaml"
    raw = OmegaConf.to_container(OmegaConf.load(path), resolve=True)
    bridge = build({**raw, "user_id": "example"})
    assert bridge.ros.url == "ws://localhost:9090"
    assert bridge.config.modules["vad"].args["block_duration"] == 0.030
    assert bridge.config.modules["stt"].args["sample_rate"] == 16000
    assert [r.ros_topic for r in bridge.inbound] == ["/mic_audio"]
    assert [r.ros_topic for r in bridge.outbound] == ["/audio"]


class Mover(Module):
    """Stands in for mov: gestures for the speech it hears."""

    input_type = "huri.audio.out"
    output_type = "huri.motion"
    input_event = Audio
    output_event = Motion

    async def process(self, data: Audio) -> Motion:
        t = 3
        z = np.zeros((t, 165), np.float32)
        return Motion(z, np.zeros((t, 100), np.float32), np.zeros((t, 3)), 30, 0.0)


class Gestures:
    """A route needing the gestures capability (the motion route to come)."""

    huri_topic = "huri.motion"
    event_cls = Motion
    ros_topic = "/gestures"
    ros_type = "std_msgs/msg/String"

    def __init__(self, capabilities):
        self.joints = [joint.name for _, joint in capabilities.gestures]

    def convert(self, event):
        return [{"data": ",".join(self.joints)}]


@pytest.fixture
def gesture_route(monkeypatch):
    monkeypatch.setitem(
        HURI_TO_ROS,
        "huri.motion",
        Outbound(lambda settings, caps: Gestures(caps), requires=GESTURES),
    )


def gesture_config(url, ros_url):
    raw = config(url, ros_url)
    raw["subscribe"] = ["huri.audio.out", "huri.motion"]
    raw["modules"] = {"echo": {"name": "echo"}, "mov": {"name": "mover"}}
    return raw


def test_the_session_follows_the_robot_lucy_runs(gesture_route):
    """A robot without a profile (testbot) carries voice only: huri.motion
    and the mover that fed it leave the session, checked by HuRI's real
    session endpoint and GET /modules."""

    async def main():
        services = lucy_config(HARDWARE_YAML, "testbot_urdf")
        async with (
            FakeRosbridge(services=services) as ros,
            huri_server({"echo": Echo, "mover": Mover}) as url,
        ):
            bridge = await connect(gesture_config(url, ros.url))
            stop = asyncio.Event()
            session = asyncio.create_task(run(bridge, stop))
            await wait_until(lambda: ros.subscribed("/mic_audio"))
            await ros.publish("/mic_audio", panel_mic(1, samples=480))
            await wait_until(lambda: ros.published("/audio"))
            stop.set()
            await asyncio.wait_for(session, timeout=5)
            return bridge, ros

    bridge, ros = asyncio.run(main())
    assert bridge.robot.package == "testbot_urdf"
    assert bridge.capabilities.names == frozenset()
    assert bridge.dropped == ["huri.motion"]
    assert bridge.config.subscribe == ["huri.audio.out"]
    assert set(bridge.config.modules) == {"echo"}
    assert ros.published("/gestures") == []


def test_a_capable_robot_keeps_its_route(gesture_route):
    inmoov = parse_hardware_config(
        (Path(__file__).parent / "fixtures" / "inmoov_urdf.yaml").read_text(),
        "inmoov_urdf",
    )
    bridge = build(gesture_config("ws://h", "ws://r"), robot=inmoov)
    assert GESTURES in bridge.capabilities.names
    assert bridge.dropped == []
    assert bridge.config.subscribe == ["huri.audio.out", "huri.motion"]
    _, gestures = bridge.outbound
    assert "i01.head.rothead_link_joint" in gestures.joints
    # An undiscovered robot (no config services) does not get it.
    assert build(gesture_config("ws://h", "ws://r")).dropped == ["huri.motion"]


def test_discovery_waits_for_lucys_config_services():
    """rosbridge is up before the config services in Lucy's bringup."""

    async def main():
        async with FakeRosbridge() as ros:
            later = lucy_config(HARDWARE_YAML, "testbot_urdf")
            asyncio.get_running_loop().call_later(
                0.3, lambda: ros.services.update(later)
            )
            robot = await discover_robot(ros.url, patience=5)
            return robot, ros

    robot, ros = asyncio.run(main())
    assert robot.package == "testbot_urdf"
    assert len([op for op in ros.ops if op["op"] == "call_service"]) >= 2
