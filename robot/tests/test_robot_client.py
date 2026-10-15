"""The robot client: serial protocol, playback, and a session against a fake
HuRI server (a websocket speaking the session protocol)."""

import asyncio
import json

import numpy as np
import pytest
import websockets
from huri_core.core.client import encode

from huri_events_inmoov import SERVO_NAMES, ServoAngles
from huri_inmoov_robot.app import build, make_driver, run
from huri_inmoov_robot.drivers import FakeDriver, SerialDriver
from huri_inmoov_robot.limits import DEFAULT_LIMITS
from huri_inmoov_robot.player import Player, park
from huri_inmoov_robot.safety import SafetyController

# --- serial line protocol -----------------------------------------------------


def test_serial_driver_line_protocol():
    driver = SerialDriver("loop://", channels={"head.neck": 0, "head.rothead": 7})
    driver.write({"head.neck": 92.54, "head.rothead": 10.0, "head.jaw": 12.0})
    driver.detach()
    driver.attach()
    sent = driver.serial.read(100).decode()
    assert sent == "S0:92.5\nS7:10.0\nD\nA\n"  # jaw has no channel: not sent
    driver.close()


def test_make_driver():
    assert isinstance(make_driver({}), FakeDriver)
    robot = {"driver": {"type": "serial", "port": "loop://"}, "channels": {}}
    assert isinstance(make_driver(robot), SerialDriver)
    assert isinstance(make_driver(robot, fake=True), FakeDriver)
    with pytest.raises(ValueError, match="port"):
        make_driver({"driver": {"type": "serial"}})
    with pytest.raises(ValueError, match="serial or fake"):
        make_driver({"driver": {"type": "can"}})


# --- playback -----------------------------------------------------------------


def _event(angle, n_frames=3, fps=50):
    angles = np.array([[DEFAULT_LIMITS[n].rest for n in SERVO_NAMES]] * n_frames)
    angles[:, SERVO_NAMES.index("head.rothead")] = angle
    return ServoAngles(angles.astype(np.float32), fps=fps)


def test_player_plays_frames_at_their_fps():
    driver = FakeDriver()
    ctl = SafetyController(driver, DEFAULT_LIMITS)
    slept = []

    async def sleep(seconds):
        slept.append(seconds)

    async def main():
        player = Player(ctl, sleep=sleep)
        await player.enqueue(_event(100.0, n_frames=4, fps=50))
        task = asyncio.create_task(player.run())
        while not player.queue.empty():
            await asyncio.sleep(0)
        task.cancel()

    asyncio.run(main())
    assert slept == [0.02] * 4
    assert all(set(w) <= set(SERVO_NAMES) for w in driver.writes)


def test_player_drops_a_late_backlog():
    ctl = SafetyController(FakeDriver(), DEFAULT_LIMITS)

    async def main():
        player = Player(ctl, max_backlog_s=1.0)
        await player.enqueue(_event(100.0, n_frames=80, fps=50))
        return player

    player = asyncio.run(main())
    assert player.queue.qsize() == 50 and player.dropped == 30


def test_player_ignores_frames_while_estopped():
    ctl = SafetyController(FakeDriver(), DEFAULT_LIMITS)
    ctl.estop()

    async def main():
        player = Player(ctl)
        await player.enqueue(_event(100.0))
        return player

    assert asyncio.run(main()).queue.empty()


def test_park_walks_back_to_rest():
    driver = FakeDriver()
    ctl = SafetyController(driver, DEFAULT_LIMITS)
    ctl.position["head.rothead"] = 100.0
    assert asyncio.run(park(ctl, timeout=2.0, period=0.005))
    assert driver.writes[-1]["head.rothead"] == pytest.approx(90.0)


# --- config -------------------------------------------------------------------


def _raw(**extra):
    raw = {
        "user_id": "robot-test",
        "huri_url": "ws://127.0.0.1:1/session",
        "send": [],
        "subscribe": ["inmoov.servo_angles"],
        "modules": {"inmoov": {"name": "inmoov"}},
        "robot": {"driver": {"type": "fake"}, "watchdog_ms": 200},
    }
    raw.update(extra)
    return raw


def test_build_checks_the_topics():
    robot = build(_raw())
    assert robot.controller.watchdog_s == pytest.approx(0.2)
    assert robot.client.handshake()["subscribe"] == ["inmoov.servo_angles"]
    with pytest.raises(ValueError, match="must subscribe"):
        build(_raw(subscribe=[]))
    with pytest.raises(ValueError, match="handles no"):
        build(_raw(subscribe=["inmoov.servo_angles", "huri.token"]))


def test_example_config_builds(monkeypatch, tmp_path):
    from pathlib import Path

    from huri_core.core.client import load_raw_client_config

    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path))
    monkeypatch.setenv("HOME", str(tmp_path))
    path = Path(__file__).resolve().parents[1] / "config" / "robot_example.yaml"
    raw = load_raw_client_config(str(path))
    raw["robot"]["audio"]["speaker"] = False
    robot = build(raw, fake=True)
    assert isinstance(robot.driver, FakeDriver)
    assert robot.controller.limits["head.neck"].min == 40
    assert robot.client.handshake()["modules"]["inmoov"]["name"] == "inmoov"


# --- a session ----------------------------------------------------------------


def test_session_moves_the_robot_then_parks_it():
    """Fake server: accepts the session, sends one chunk, closes."""

    handshakes = []

    async def serve(ws):
        handshakes.append(json.loads(await ws.recv()))
        await ws.send(json.dumps({"type": "session_init"}))
        await ws.send(encode("inmoov.servo_angles", _event(110.0, n_frames=10)))
        await asyncio.sleep(0.4)
        await ws.close()

    async def main():
        async with websockets.serve(serve, "127.0.0.1", 0) as server:
            port = server.sockets[0].getsockname()[1]
            robot = build(_raw(huri_url=f"ws://127.0.0.1:{port}/session"))
            await asyncio.wait_for(run(robot), timeout=10)
            return robot

    robot = asyncio.run(main())
    assert handshakes[0]["subscribe"] == ["inmoov.servo_angles"]
    assert handshakes[0]["modules"]["inmoov"]["name"] == "inmoov"
    driver = robot.driver
    assert isinstance(driver, FakeDriver) and driver.closed
    rothead = [w["head.rothead"] for w in driver.writes if "head.rothead" in w]
    assert max(rothead) > 91.0  # it moved towards 110...
    assert rothead[-1] == pytest.approx(90.0)  # ...and parked at rest
    assert robot.controller.at_rest()
