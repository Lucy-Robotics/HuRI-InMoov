"""huri-lucy-bridge: a Lucy robot as a HuRI client.

    huri-lucy-bridge --config bridge/config/bridge_example.yaml

Like the Lucy control panel, the bridge is a rosbridge client of the robot
(no ROS install) and registers in Lucy's client registry by heartbeat. It
forwards ROS topics to HuRI and HuRI topics to ROS, one route per HuRI topic:

    /mic_audio (AudioStamped) --> huri.audio.in    (send)
    /audio     (AudioStamped) <-- huri.audio.out   (subscribe)

The config is a HuRI client YAML (huri_url, modules, send, subscribe) plus a
`lucy:` section the server never reads:

    lucy:
      rosbridge_url: ws://localhost:9090
      client_id: huri_bridge    # prefix; a timestamp and a random suffix follow
      heartbeat_s: 1.0
      reconnect_s: 3.0          # 0: exit when HuRI or rosbridge goes away
      audio: {mic_topic: /mic_audio, speaker_topic: /audio,
              sample_rate: 16000, frame_duration: 0.030, speaker_chunk: 1024}

Every topic of `send` and `subscribe` needs a route; a route whose HuRI topic
is in neither is off (drop huri.audio.out from subscribe: the robot is mute).
"""

import argparse
import asyncio
import logging
import signal
import sys
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Mapping, Optional

import websockets
from huri_core.core.client import HuRIClient, HuRISessionError, load_raw_client_config
from huri_core.core.dataclasses.config import ClientConfig

from huri_lucy_bridge.audio import HuriToSpeaker, MicToHuri
from huri_lucy_bridge.presence import HEARTBEAT_TOPIC, STRING, heartbeat, new_client_id
from huri_lucy_bridge.rosbridge import RosbridgeClient
from huri_lucy_bridge.routes import HuriToRos, RosToHuri

logger = logging.getLogger("huri_lucy_bridge")

# What a dropped connection raises, against which reconnecting may help (not a
# refused session or a broken config).
CONNECTION_ERRORS = (
    OSError,
    asyncio.TimeoutError,
    websockets.ConnectionClosed,
    websockets.InvalidHandshake,
)


def _mic(settings: Mapping[str, Any]) -> RosToHuri:
    audio = settings.get("audio") or {}
    return MicToHuri(
        ros_topic=audio.get("mic_topic", "/mic_audio"),
        sample_rate=int(audio.get("sample_rate", 16000)),
        frame_duration=float(audio.get("frame_duration", 0.030)),
    )


def _speaker(settings: Mapping[str, Any]) -> HuriToRos:
    audio = settings.get("audio") or {}
    return HuriToSpeaker(
        ros_topic=audio.get("speaker_topic", "/audio"),
        chunk=int(audio.get("speaker_chunk", 1024)),
    )


# HuRI topic -> route factory, by direction.
ROS_TO_HURI: Dict[str, Callable[[Mapping[str, Any]], RosToHuri]] = {
    "huri.audio.in": _mic,
}
HURI_TO_ROS: Dict[str, Callable[[Mapping[str, Any]], HuriToRos]] = {
    "huri.audio.out": _speaker,
}


@dataclass
class Bridge:
    config: ClientConfig
    settings: Dict[str, Any]
    client_id: str
    client: HuRIClient
    ros: RosbridgeClient
    inbound: List[RosToHuri] = field(default_factory=list)
    outbound: List[HuriToRos] = field(default_factory=list)


def build(raw: Mapping[str, Any], client_id: Optional[str] = None) -> Bridge:
    """Both clients and their routes, from a loaded client config; nothing
    connected."""

    config = ClientConfig.from_dict(dict(raw))
    settings = dict(raw.get("lucy") or {})
    unrouted = sorted(set(config.send) - set(ROS_TO_HURI)) + sorted(
        set(config.subscribe) - set(HURI_TO_ROS)
    )
    if unrouted:
        raise ValueError(
            f"huri-lucy-bridge has no route for {unrouted}: it sends "
            f"{sorted(ROS_TO_HURI)} and subscribes to {sorted(HURI_TO_ROS)}"
        )
    client = HuRIClient.from_config(config)
    ros = RosbridgeClient(settings.get("rosbridge_url", "ws://localhost:9090"))
    ros.advertise(HEARTBEAT_TOPIC, STRING)
    bridge = Bridge(
        config,
        settings,
        client_id or new_client_id(settings.get("client_id", "huri_bridge")),
        client,
        ros,
    )

    for topic in config.send:
        inbound = ROS_TO_HURI[topic](settings)
        bridge.inbound.append(inbound)

        async def forward(msg: Dict[str, Any], route: RosToHuri = inbound) -> None:
            for event in route.convert(msg):
                await client.send(route.huri_topic, event)

        ros.subscribe(inbound.ros_topic, inbound.ros_type, forward)

    for topic in config.subscribe:
        outbound = HURI_TO_ROS[topic](settings)
        bridge.outbound.append(outbound)
        ros.advertise(outbound.ros_topic, outbound.ros_type)

        async def push(event: Any, route: HuriToRos = outbound) -> None:
            for msg in route.convert(event):
                await ros.publish(route.ros_topic, msg)

        client.on(outbound.huri_topic, outbound.event_cls, push)

    return bridge


async def run(bridge: Bridge, stop: Optional[asyncio.Event] = None) -> None:
    """Bridge until `stop` is set; raises ConnectionError when HuRI or
    rosbridge closes its connection.

    HuRI connects first: a refused session fails before the robot sees the
    bridge, and microphone frames never reach a client not yet connected."""

    stop = stop or asyncio.Event()
    async with bridge.client, bridge.ros:
        logger.info(
            "bridging %s and %s as %s",
            bridge.client.url,
            bridge.ros.url,
            bridge.client_id,
        )
        beat = asyncio.create_task(
            heartbeat(
                bridge.ros,
                bridge.client_id,
                float(bridge.settings.get("heartbeat_s", 1.0)),
            )
        )
        huri_closed = asyncio.create_task(bridge.client.wait_closed())
        ros_closed = asyncio.create_task(bridge.ros.wait_closed())
        stopped = asyncio.create_task(stop.wait())
        tasks = {beat, huri_closed, ros_closed, stopped}
        try:
            done, _ = await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
        finally:
            for task in tasks:
                task.cancel()
            await asyncio.gather(*tasks, return_exceptions=True)
        if stopped in done:
            return
        if beat in done and beat.exception() is not None:
            raise ConnectionError(f"heartbeat failed: {beat.exception()!r}")
        side = "HuRI" if huri_closed in done else "rosbridge"
        raise ConnectionError(f"{side} closed the connection")


async def serve(raw: Mapping[str, Any], stop: asyncio.Event) -> None:
    """run() again after `lucy.reconnect_s` each time a connection drops,
    under the same client id, with fresh routes; until `stop`."""

    reconnect = float((raw.get("lucy") or {}).get("reconnect_s", 3.0))
    client_id: Optional[str] = None
    while not stop.is_set():
        bridge = build(raw, client_id)
        client_id = bridge.client_id
        try:
            await run(bridge, stop)
            return
        except CONNECTION_ERRORS as e:
            if reconnect <= 0:
                raise
            logger.warning("%s; reconnecting in %.1f s", e, reconnect)
        try:
            await asyncio.wait_for(stop.wait(), reconnect)
        except asyncio.TimeoutError:
            pass


async def _main(args: argparse.Namespace) -> None:
    raw = load_raw_client_config(args.config)
    build(raw)  # a broken config fails now, not at the first reconnection
    loop = asyncio.get_running_loop()
    stop = asyncio.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, stop.set)
    await serve(raw, stop)


def main(argv: Optional[List[str]] = None) -> None:
    parser = argparse.ArgumentParser(
        prog="huri-lucy-bridge",
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--config", required=True, help="client YAML with lucy:")
    parser.add_argument("--log-level", default="INFO")
    args = parser.parse_args(argv)
    logging.basicConfig(
        level=args.log_level.upper(),
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    try:
        asyncio.run(_main(args))
    except HuRISessionError as e:
        sys.exit(f"huri-lucy-bridge: HuRI refused the session: {e}")
    except (ValueError, *CONNECTION_ERRORS) as e:
        sys.exit(f"huri-lucy-bridge: {e}")


if __name__ == "__main__":
    main()
