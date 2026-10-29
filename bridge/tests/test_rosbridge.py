import asyncio
import logging

import pytest
from bridge_fakes import FakeRosbridge, wait_until

from huri_lucy_bridge.rosbridge import RosbridgeClient

AUDIO = "audio_common_msgs/msg/AudioStamped"


def test_declarations_are_sent_on_connect_and_publishes_go_out():
    async def main():
        async with FakeRosbridge() as server:
            ros = RosbridgeClient(server.url)
            ros.advertise("/audio", AUDIO)
            ros.subscribe("/mic_audio", AUDIO, _ignore)
            async with ros:
                await ros.publish("/audio", {"x": 1})
                await wait_until(lambda: server.published("/audio"))
            return server

    server = asyncio.run(main())
    assert server.op_names() == [
        ("advertise", "/audio"),
        ("subscribe", "/mic_audio"),
        ("publish", "/audio"),
    ]
    advertise, subscribe = server.ops[:2]
    assert advertise["type"] == subscribe["type"] == AUDIO
    assert server.published("/audio") == [{"x": 1}]


def test_messages_reach_handlers_in_order_despite_a_failing_one():
    async def main():
        async with FakeRosbridge() as server:
            ros = RosbridgeClient(server.url)
            received = []

            async def record(msg):
                received.append(msg["n"])

            async def fail(msg):
                raise RuntimeError("boom")

            ros.subscribe("/t", "std_msgs/msg/Int32", fail)
            ros.subscribe("/t", "std_msgs/msg/Int32", record)
            async with ros:
                await wait_until(lambda: server.subscribed("/t"))
                for n in range(20):
                    await server.publish("/t", {"n": n})
                await wait_until(lambda: len(received) == 20)
            return received

    assert asyncio.run(main()) == list(range(20))


def test_rosbridge_errors_are_logged(caplog):
    async def main():
        async with FakeRosbridge(unknown_types={AUDIO}) as server:
            ros = RosbridgeClient(server.url)
            ros.advertise("/audio", AUDIO)
            with caplog.at_level(logging.ERROR, "huri_lucy_bridge.rosbridge"):
                async with ros:
                    await wait_until(lambda: "manifest" in caplog.text)

    asyncio.run(main())
    assert AUDIO in caplog.text


def test_declarations_are_checked():
    ros = RosbridgeClient("ws://unused")
    ros.advertise("/a", "std_msgs/msg/String")
    ros.advertise("/a", "std_msgs/msg/String")
    with pytest.raises(ValueError, match="already advertised"):
        ros.advertise("/a", "std_msgs/msg/Int32")
    ros.subscribe("/b", "std_msgs/msg/String", _ignore)
    with pytest.raises(ValueError, match="already subscribed"):
        ros.subscribe("/b", "std_msgs/msg/Int32", _ignore)
    with pytest.raises(RuntimeError, match="not connected"):
        asyncio.run(ros.publish("/a", {}))

    async def unadvertised():
        async with FakeRosbridge() as server:
            async with RosbridgeClient(server.url) as connected:
                await connected.publish("/nope", {})

    with pytest.raises(ValueError, match="not advertised"):
        asyncio.run(unadvertised())


async def _ignore(_):
    return None
