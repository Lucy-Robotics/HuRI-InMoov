"""A fake rosbridge (the v2 ops the bridge uses, over a local websocket
server) and HuRI's real session endpoint, in-process."""

import asyncio
import contextlib
import json
from typing import Any, AsyncIterator, Callable, Dict, List, Optional, Set, Type

import websockets
from huri_core.core.huri import HuRI
from huri_core.core.module import Module
from huri_core.core.plugins import load_events

# @serve.ingress subclasses HuRI; its direct base runs without Serve.
HuRIServer = HuRI.func_or_class.__mro__[1]


class FakeRosbridge:
    """Records the client's ops; publishes to its subscriptions on demand.

    `unknown_types` are message types this rosbridge cannot import: it answers
    their advertise or subscribe with an error status, as rosbridge does.
    `services` answer call_service ops: name -> callable(args) -> values (a
    raised exception is a failed call); a service not there fails, like a
    service nobody advertises."""

    def __init__(
        self,
        unknown_types: Set[str] = frozenset(),
        services: Optional[Dict[str, Callable[[Dict[str, Any]], Any]]] = None,
    ):
        self.unknown_types = set(unknown_types)
        self.services = dict(services or {})
        self.ops: List[Dict[str, Any]] = []
        self.connections: List[Any] = []
        self.port = 0
        self._server: Any = None

    @property
    def url(self) -> str:
        return f"ws://127.0.0.1:{self.port}"

    async def __aenter__(self) -> "FakeRosbridge":
        self._server = await websockets.serve(self._handle, "127.0.0.1", self.port)
        self.port = self._server.sockets[0].getsockname()[1]
        return self

    async def __aexit__(self, *_) -> None:
        self._server.close()
        await self._server.wait_closed()

    async def _handle(self, ws) -> None:
        self.connections.append(ws)
        async for message in ws:
            op = json.loads(message)
            op["connection"] = len(self.connections) - 1
            self.ops.append(op)
            if op["op"] == "call_service":
                await ws.send(json.dumps(self._call(op)))
            if op["op"] in ("advertise", "subscribe"):
                if op["type"] in self.unknown_types:
                    await ws.send(
                        json.dumps(
                            {
                                "op": "status",
                                "level": "error",
                                "id": op.get("id"),
                                "msg": f"Unable to load the manifest for "
                                f"{op['type']}",
                            }
                        )
                    )

    def _call(self, op: Dict[str, Any]) -> Dict[str, Any]:
        reply = {"op": "service_response", "id": op.get("id"), "service": op["service"]}
        service = self.services.get(op["service"])
        if service is None:
            return {**reply, "result": False, "values": "Service does not exist"}
        try:
            return {**reply, "result": True, "values": service(op.get("args", {}))}
        except Exception as e:
            return {**reply, "result": False, "values": str(e)}

    def subscribed(self, topic: str) -> List[Any]:
        """Connections that subscribed to `topic`."""

        return [
            self.connections[op["connection"]]
            for op in self.ops
            if op["op"] == "subscribe" and op["topic"] == topic
        ]

    async def publish(self, topic: str, msg: Dict[str, Any]) -> None:
        frame = json.dumps({"op": "publish", "topic": topic, "msg": msg})
        for ws in self.subscribed(topic):
            try:
                await ws.send(frame)
            except websockets.ConnectionClosed:
                pass  # a client that already went away

    def published(self, topic: str, connection: Optional[int] = None) -> List[Any]:
        return [
            op["msg"]
            for op in self.ops
            if op["op"] == "publish"
            and op["topic"] == topic
            and connection in (None, op["connection"])
        ]

    def op_names(self, connection: int = 0) -> List[tuple]:
        return [
            (op["op"], op.get("topic"))
            for op in self.ops
            if op["connection"] == connection
        ]


class FastAPISocket:
    """The FastAPI WebSocket methods HuRI.run_session uses, over a
    `websockets` server connection."""

    def __init__(self, ws):
        self.ws = ws

    async def accept(self):
        pass

    async def receive_json(self):
        return json.loads(await self.ws.recv())

    async def receive(self):
        try:
            message = await self.ws.recv()
        except websockets.ConnectionClosed:
            return {"type": "websocket.disconnect"}
        key = "bytes" if isinstance(message, bytes) else "text"
        return {"type": "websocket.receive", key: message}

    async def send_json(self, message):
        await self.ws.send(json.dumps(message))

    async def send_bytes(self, payload):
        await self.ws.send(payload)

    async def close(self, code=None, reason=None):
        await self.ws.close(code or 1000, reason or "")


async def wait_until(predicate: Callable[[], bool], timeout: float = 5.0) -> None:
    async def poll() -> None:
        while not predicate():
            await asyncio.sleep(0.01)

    await asyncio.wait_for(poll(), timeout)


@contextlib.asynccontextmanager
async def huri_server(modules: Dict[str, Type[Module]]) -> AsyncIterator[str]:
    """HuRI's /session endpoint serving `modules` (no Ray: the ingress class
    runs in-process, behind a plain websocket server); yields its URL."""

    server = HuRIServer(modules, {}, load_events(), [])

    async def serve(ws) -> None:
        await server.run_session(FastAPISocket(ws))

    def http(connection, request):
        if request.path == "/modules":  # GET /modules, as FastAPI serves it
            return connection.respond(200, json.dumps(server.modules_info))
        return None

    async with websockets.serve(
        serve, "127.0.0.1", 0, process_request=http
    ) as ws_server:
        port = ws_server.sockets[0].getsockname()[1]
        yield f"ws://127.0.0.1:{port}/session"


def lucy_config(yaml_text: str, package: str) -> Dict[str, Callable]:
    """The services of Lucy's config pipeline, serving one hardware YAML."""

    def get(args: Dict[str, Any]) -> Dict[str, Any]:
        return {
            "success": True,
            "message": "ok",
            "robot_package": package,
            "config_name": "default",
            "config_yaml": yaml_text,
            "flashed_config_name": "",
            "flashed_at": "",
        }

    return {"/config/get": get}


# Two boards, as Lucy's robot packages lay them out (servo radians; the left
# board's actuators out of virtual_pin order on purpose).
HARDWARE_YAML = """
version: 1
robot_name: testbot
cameras:
  - id: cam
    topic: /cam/jpg
boards:
  head_board:
    controller: {name: head_controller}
  arm_board:
    controller: {name: arm_controller}
  sensor_board: {}
actuators:
  - {id: elbow, urdf_joint: elbow_joint, board: arm_board, virtual_pin: 1,
     offset: 0, direction: 1, scale: 1, limit_min: 0, limit_max: 2.5, default: 0.5}
  - {id: shoulder, urdf_joint: shoulder_joint, board: arm_board, virtual_pin: 0,
     offset: 1.5708, direction: -1, scale: 1, limit_min: 0, limit_max: 3.14159,
     default: 1.5708, enabled: false}
  - {id: jaw, urdf_joint: jaw_joint, board: head_board, virtual_pin: 0,
     offset: 0, direction: 1, scale: 2, limit_min: 0, limit_max: 0.5, default: 0}
"""
