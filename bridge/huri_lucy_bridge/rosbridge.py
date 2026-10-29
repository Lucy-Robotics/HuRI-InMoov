"""A minimal asyncio client of the rosbridge v2 protocol: what the Lucy control
panel does with roslib, in Python and without a ROS install.

    ros = RosbridgeClient("ws://localhost:9090")
    ros.subscribe("/mic_audio", "audio_common_msgs/msg/AudioStamped", on_mic)
    ros.advertise("/audio", "audio_common_msgs/msg/AudioStamped")
    async with ros:
        await ros.publish("/audio", {...})
        await ros.wait_closed()

Messages are plain dicts in rosbridge's JSON form (a ROS 2 stamp is
{"sec", "nanosec"}; uint8[] arrays arrive base64-encoded). Handlers of one
topic run one at a time, in arrival order; different topics run concurrently;
a failing handler is logged and does not stop the others. The errors rosbridge
reports (`status` ops: an unknown message type, a type mismatch) are logged.
"""

import asyncio
import itertools
import json
import logging
from dataclasses import dataclass
from typing import Any, Awaitable, Callable, Dict, List, Mapping, Optional

import websockets

logger = logging.getLogger("huri_lucy_bridge.rosbridge")

Message = Dict[str, Any]
Handler = Callable[[Message], Awaitable[None]]

_STATUS_LEVELS = {
    "error": logging.ERROR,
    "warning": logging.WARNING,
    "info": logging.INFO,
}


@dataclass
class _Subscription:
    msg_type: str
    handlers: List[Handler]
    queue: "asyncio.Queue[Message]"


class RosbridgeClient:
    """Connection to a rosbridge server (rosbridge_websocket, port 9090 in
    Lucy's bringup).

    Subscriptions and advertisements are declared before connecting and sent
    on connect, like the session of a HuRIClient.
    """

    def __init__(self, url: str, connect_kwargs: Optional[Mapping[str, Any]] = None):
        self.url = url
        # Audio messages in JSON exceed the library's 1 MiB default.
        self.connect_kwargs: Dict[str, Any] = {"max_size": None}
        self.connect_kwargs.update(connect_kwargs or {})
        self._subs: Dict[str, _Subscription] = {}
        self._adverts: Dict[str, str] = {}
        self._ws: Optional[websockets.ClientConnection] = None
        self._tasks: List[asyncio.Task] = []
        self._ids = itertools.count(1)

    # --- configuration ----------------------------------------------------

    def subscribe(self, topic: str, msg_type: str, handler: Handler) -> None:
        """Call `handler(msg)` for every message of `topic`."""

        if self._ws is not None:
            raise RuntimeError("subscribe before connecting")
        sub = self._subs.get(topic)
        if sub is None:
            self._subs[topic] = _Subscription(msg_type, [handler], asyncio.Queue())
        elif sub.msg_type != msg_type:
            raise ValueError(
                f"{topic!r} is already subscribed as {sub.msg_type}, not {msg_type}"
            )
        else:
            sub.handlers.append(handler)

    def advertise(self, topic: str, msg_type: str) -> None:
        """Declare a topic this client publishes."""

        if self._ws is not None:
            raise RuntimeError("advertise before connecting")
        known = self._adverts.setdefault(topic, msg_type)
        if known != msg_type:
            raise ValueError(f"{topic!r} is already advertised as {known}")

    # --- connection ---------------------------------------------------------

    async def connect(self) -> "RosbridgeClient":
        ws = await websockets.connect(self.url, **self.connect_kwargs)
        try:
            for topic, msg_type in self._adverts.items():
                await ws.send(self._op("advertise", topic=topic, type=msg_type))
            for topic, sub in self._subs.items():
                await ws.send(self._op("subscribe", topic=topic, type=sub.msg_type))
        except BaseException:
            await ws.close()
            raise
        self._ws = ws
        self._tasks = [asyncio.create_task(self._receive_loop(ws))]
        self._tasks += [
            asyncio.create_task(self._dispatch_loop(topic, sub))
            for topic, sub in self._subs.items()
        ]
        return self

    async def close(self) -> None:
        for task in self._tasks:
            task.cancel()
        await asyncio.gather(*self._tasks, return_exceptions=True)
        self._tasks = []
        if self._ws is not None:
            await self._ws.close()
            self._ws = None

    async def __aenter__(self) -> "RosbridgeClient":
        return await self.connect()

    async def __aexit__(self, *_) -> None:
        await self.close()

    async def wait_closed(self) -> None:
        """Until rosbridge closes the connection."""

        if self._ws is not None:
            await self._ws.wait_closed()

    # --- traffic -----------------------------------------------------------

    async def publish(self, topic: str, msg: Mapping[str, Any]) -> None:
        if self._ws is None:
            raise RuntimeError("not connected")
        if topic not in self._adverts:
            raise ValueError(f"{topic!r} is not advertised: advertise() it first")
        await self._ws.send(json.dumps({"op": "publish", "topic": topic, "msg": msg}))

    def _op(self, op: str, **fields: Any) -> str:
        return json.dumps({"op": op, "id": f"{op}:{next(self._ids)}", **fields})

    async def _receive_loop(self, ws: websockets.ClientConnection) -> None:
        try:
            async for message in ws:
                try:
                    frame = json.loads(message)
                except ValueError:
                    logger.warning("dropping a frame that is not JSON")
                    continue
                op = frame.get("op")
                if op == "publish":
                    sub = self._subs.get(frame.get("topic", ""))
                    if sub is not None:
                        sub.queue.put_nowait(frame.get("msg") or {})
                elif op == "status":
                    level = _STATUS_LEVELS.get(frame.get("level"), logging.INFO)
                    logger.log(level, "rosbridge: %s", frame.get("msg"))
        except websockets.ConnectionClosed:
            pass

    async def _dispatch_loop(self, topic: str, sub: _Subscription) -> None:
        while True:
            msg = await sub.queue.get()
            for handler in sub.handlers:
                try:
                    await handler(msg)
                except Exception:
                    logger.exception("handler for %r failed", topic)
