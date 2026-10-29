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
      profile: <path> | none    # default: the packaged one of the robot
      discovery_timeout_s: 30.0 # wait for Lucy's config services, then voice only
      audio: {mic_topic: /mic_audio, speaker_topic: /audio,
              sample_rate: 16000, frame_duration: 0.030, speaker_chunk: 1024}

Nothing in it names the robot. On each connection the bridge asks Lucy which
robot it runs (/config/get: its robot package, controllers, joints) and picks
that robot's embodiment profile (profiles.py), which says what the robot can do
(gestures, speech motion). The YAML describes the richest session: a route
needing a capability the robot lacks is dropped from it, with the modules that
only fed it; voice works on every robot.

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
from huri_lucy_bridge.profiles import (
    NO_PROFILE,
    Capabilities,
    load_profile,
    resolve,
    select_profile,
)
from huri_lucy_bridge.robot import DiscoveryError, Robot, discover
from huri_lucy_bridge.rosbridge import RosbridgeClient
from huri_lucy_bridge.routes import HuriToRos, RosToHuri
from huri_lucy_bridge.session import ModuleDetails, fetch_module_details, without

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


def _speaker(settings: Mapping[str, Any], _: Capabilities) -> HuriToRos:
    audio = settings.get("audio") or {}
    return HuriToSpeaker(
        ros_topic=audio.get("speaker_topic", "/audio"),
        chunk=int(audio.get("speaker_chunk", 1024)),
    )


@dataclass(frozen=True)
class Outbound:
    """A HuRI -> ROS route: its factory, and the capability the robot needs
    for it (None: every robot can, like playing speech)."""

    factory: Callable[[Mapping[str, Any], Capabilities], HuriToRos]
    requires: Optional[str] = None


# HuRI topic -> route, by direction. Sensing (ROS -> HuRI) needs no
# capability; acting on the robot (HuRI -> ROS) may.
ROS_TO_HURI: Dict[str, Callable[[Mapping[str, Any]], RosToHuri]] = {
    "huri.audio.in": _mic,
}
HURI_TO_ROS: Dict[str, Outbound] = {
    "huri.audio.out": Outbound(_speaker),
}


@dataclass
class Bridge:
    config: ClientConfig  # the session as sent: without what the robot cannot use
    settings: Dict[str, Any]
    client_id: str
    client: HuRIClient
    ros: RosbridgeClient
    robot: Optional[Robot] = None
    capabilities: Capabilities = field(default_factory=Capabilities)
    dropped: List[str] = field(default_factory=list)  # HuRI topics left out
    inbound: List[RosToHuri] = field(default_factory=list)
    outbound: List[HuriToRos] = field(default_factory=list)


def build(
    raw: Mapping[str, Any],
    client_id: Optional[str] = None,
    robot: Optional[Robot] = None,
    module_details: Optional[ModuleDetails] = None,
) -> Bridge:
    """Both clients and their routes, from a loaded client config and the
    robot Lucy described (None: undiscovered, voice only); nothing connected.

    `module_details` (the server's GET /modules) lets the session drop the
    modules that only fed a dropped route."""

    full = ClientConfig.from_dict(dict(raw))
    settings = dict(raw.get("lucy") or {})
    unrouted = sorted(set(full.send) - set(ROS_TO_HURI)) + sorted(
        set(full.subscribe) - set(HURI_TO_ROS)
    )
    if unrouted:
        raise ValueError(
            f"huri-lucy-bridge has no route for {unrouted}: it sends "
            f"{sorted(ROS_TO_HURI)} and subscribes to {sorted(HURI_TO_ROS)}"
        )
    profile = select_profile(robot, settings.get("profile")) if robot else None
    capabilities = resolve(profile, robot)
    if profile is not None:
        logger.info(
            "profile %s: %s",
            profile.source,
            ", ".join(sorted(capabilities.names)) or "no capability for this robot",
        )
    dropped = [
        topic
        for topic in full.subscribe
        if HURI_TO_ROS[topic].requires not in (None, *capabilities.names)
    ]
    for topic in dropped:
        logger.info(
            "%s needs %s, which %s cannot do: left out of the session",
            topic,
            HURI_TO_ROS[topic].requires,
            robot.package if robot else "an undiscovered robot",
        )
    config = ClientConfig.from_dict(
        without(raw, dropped, module_details) if dropped else dict(raw)
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
        robot,
        capabilities,
        dropped,
    )

    for topic in config.send:
        inbound = ROS_TO_HURI[topic](settings)
        bridge.inbound.append(inbound)

        async def forward(msg: Dict[str, Any], route: RosToHuri = inbound) -> None:
            for event in route.convert(msg):
                await client.send(route.huri_topic, event)

        ros.subscribe(inbound.ros_topic, inbound.ros_type, forward)

    for topic in config.subscribe:
        outbound = HURI_TO_ROS[topic].factory(settings, capabilities)
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


async def discover_robot(rosbridge_url: str, patience: float = 30.0) -> Optional[Robot]:
    """The robot Lucy runs, over a connection of its own; None when Lucy does
    not describe it within `patience` s (no config services): the bridge then
    carries voice only. Lucy's bringup starts rosbridge before its config
    services, so a bridge started with it asks again until then. A connection
    that fails raises, to be retried."""

    loop = asyncio.get_running_loop()
    deadline = loop.time() + patience
    async with RosbridgeClient(rosbridge_url) as ros:
        while True:
            try:
                return await discover(ros)
            except DiscoveryError as e:
                if loop.time() >= deadline:
                    logger.warning("%s: voice only", e)
                    return None
                logger.info("%s; asking again", e)
            await asyncio.sleep(min(1.0, max(0.0, deadline - loop.time())))


async def connect(raw: Mapping[str, Any], client_id: Optional[str] = None) -> Bridge:
    """build() for the robot Lucy runs now, and the server HuRI runs now."""

    settings = raw.get("lucy") or {}
    robot = await discover_robot(
        settings.get("rosbridge_url", "ws://localhost:9090"),
        float(settings.get("discovery_timeout_s", 30.0)),
    )
    details = await asyncio.to_thread(fetch_module_details, raw.get("huri_url", ""))
    return build(raw, client_id, robot, details)


async def serve(raw: Mapping[str, Any], stop: asyncio.Event) -> None:
    """run() again after `lucy.reconnect_s` each time a connection drops,
    under the same client id, with fresh routes for the robot Lucy runs then
    (it may have changed); until `stop`."""

    reconnect = float((raw.get("lucy") or {}).get("reconnect_s", 3.0))
    client_id: Optional[str] = None
    while not stop.is_set():
        try:
            bridge = await connect(raw, client_id)
            client_id = bridge.client_id
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
    profile = (raw.get("lucy") or {}).get("profile")
    if profile and profile != NO_PROFILE:
        load_profile(profile)
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
