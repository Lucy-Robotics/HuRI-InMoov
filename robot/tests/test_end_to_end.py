"""huri-inmoov-robot against HuRI's real /session endpoint, with the inmoov
module loaded: the handshake and its checks (check_session), the actuation
opt-in, the server's Sender binary framing, and the robot's decoding and
playback. No Ray cluster: the ingress class runs in-process, behind a plain
websocket server (the huri-core tests do the same with a fake socket)."""

import asyncio
import json

import numpy as np
import pytest
import websockets
from huri_core.core.compat import check_actuation
from huri_core.core.events import BytesEvent
from huri_core.core.huri import HuRI
from huri_core.core.module import Module
from huri_core.core.plugins import find_manifest, load_events, load_modules
from huri_events.motion import Motion

from huri_events_inmoov import ServoAngles
from huri_inmoov.retarget import SMPLX_JOINTS
from huri_inmoov_robot.app import build, run

# @serve.ingress subclasses HuRI; its direct base runs without Serve.
Server = HuRI.func_or_class.__mro__[1]


class StubMotion(Module):
    """Stands in for mov: any audio chunk -> one second of a head turn."""

    input_type = "huri.audio.in"
    output_type = "huri.motion"
    input_event = BytesEvent
    output_event = Motion

    async def process(self, data: BytesEvent) -> Motion:
        t = 30
        poses = np.zeros((t, 165), np.float32)
        poses[:, SMPLX_JOINTS.index("neck") * 3 + 1] = np.radians(30)
        return Motion(poses, np.zeros((t, 100), np.float32), np.zeros((t, 3)), 30, 0.0)


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


def test_the_robot_moves_through_a_real_huri_session():
    modules = load_modules(["inmoov"])
    # The server's startup check: inmoov needs the operator's opt-in.
    manifests = {"inmoov": find_manifest("inmoov")}
    assert check_actuation(modules, manifests, opted_in=["inmoov"]) == ["inmoov"]
    modules["stub"] = StubMotion
    server = Server(modules, {}, load_events(), ["inmoov"])

    async def main():
        async def serve(ws):
            await server.run_session(FastAPISocket(ws))

        async with websockets.serve(serve, "127.0.0.1", 0) as ws_server:
            port = ws_server.sockets[0].getsockname()[1]
            robot = build(
                {
                    "user_id": "robot-e2e",
                    "huri_url": f"ws://127.0.0.1:{port}/session",
                    "send": ["huri.audio.in"],
                    "subscribe": ["inmoov.servo_angles"],
                    "modules": {
                        "mov": {"name": "stub"},
                        "inmoov": {"name": "inmoov"},
                    },
                    "robot": {"driver": {"type": "fake"}, "audio": {"mic": False}},
                }
            )
            received = []
            robot.client.on(
                "inmoov.servo_angles",
                ServoAngles,
                lambda event: _record(received, event),
            )
            stop = asyncio.Event()
            session = asyncio.create_task(run(robot, stop))
            for _ in range(500):
                if robot.client._ws is not None:
                    break
                await asyncio.sleep(0.01)
            await robot.client.send("huri.audio.in", BytesEvent(data=b"\0\0" * 480))
            for _ in range(500):
                if received and robot.player.queue.empty():
                    break
                await asyncio.sleep(0.01)
            await asyncio.sleep(0.2)
            stop.set()
            await asyncio.wait_for(session, timeout=10)
            return robot, received

    robot, received = asyncio.run(main())
    (angles,) = received
    assert angles.angles.shape[0] == 30 and angles.fps == 30
    rothead = [w["head.rothead"] for w in robot.driver.writes if "head.rothead" in w]
    assert max(rothead) > 91.0  # it turned the head...
    assert rothead[-1] == pytest.approx(90.0)  # ...and parked at rest
    assert robot.driver.closed


async def _record(received, event):
    received.append(event)
