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
from huri_inmoov_robot.drivers import DriverFault, FakeDriver, SerialDriver
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


class StuckPort:
    """A serial port whose writes time out (the board stopped reading)."""

    def __init__(self):
        self.writes = 0

    def write(self, data):
        import serial

        self.writes += 1
        raise serial.SerialTimeoutException("Write timeout")

    def close(self):
        pass


def test_a_stuck_port_latches_the_estop_without_more_writes():
    port = StuckPort()
    driver = SerialDriver("loop://", channels={"head.rothead": 0}, serial=port)
    with pytest.raises(DriverFault):
        driver.write({"head.rothead": 95.0})
    ctl = SafetyController(driver, DEFAULT_LIMITS, estop_action="detach")
    ctl.position["head.rothead"] = 100.0
    ctl.go_rest(now=ctl.clock() + 0.05)
    assert ctl.estopped and "driver fault" in ctl.estop_reason
    writes = port.writes
    ctl.go_rest()
    assert not ctl.command({"head.rothead": 120.0})
    assert port.writes == writes  # nothing more sent, not even a detach


def test_estop_latches_even_if_detach_fails():
    class Broken(FakeDriver):
        def detach(self):
            raise DriverFault("stuck")

    ctl = SafetyController(Broken(), DEFAULT_LIMITS, estop_action="detach")
    ctl.estop("test")
    assert ctl.estopped and not ctl.command({"head.rothead": 120.0})


def test_serial_driver_sets_a_write_timeout():
    driver = SerialDriver("loop://", channels={}, write_timeout=0.03)
    assert driver.serial.write_timeout == pytest.approx(0.03)
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


def _event(angle, n_frames=3, fps=50, pts=0.0):
    angles = np.array([[DEFAULT_LIMITS[n].rest for n in SERVO_NAMES]] * n_frames)
    angles[:, SERVO_NAMES.index("head.rothead")] = angle
    return ServoAngles(angles.astype(np.float32), fps=fps, pts=pts)


class FakeTime:
    """The player's clock and sleep: sleeping advances the clock."""

    def __init__(self):
        self.t = 100.0
        self.slept = []

    def __call__(self):
        return self.t

    async def sleep(self, seconds):
        self.slept.append(seconds)
        self.t += seconds


def _play(player, *events, between=None):
    """Enqueue the events (calling `between(i)` before event i) and play
    until the queue is empty."""

    async def main():
        task = asyncio.create_task(player.run())
        for i, event in enumerate(events):
            if between is not None:
                between(i)
            await player.enqueue(event)
            for _ in range(1000):
                if player.queue.empty():
                    break
                await asyncio.sleep(0)
        await asyncio.sleep(0)
        task.cancel()

    asyncio.run(main())


def _player(driver=None, **kwargs):
    time = FakeTime()
    ctl = SafetyController(driver or FakeDriver(), DEFAULT_LIMITS, clock=time)
    return Player(ctl, sleep=time.sleep, clock=time, **kwargs), time


def test_player_plays_frames_at_their_fps():
    driver = FakeDriver()
    player, time = _player(driver)
    _play(player, _event(100.0, n_frames=4, fps=50))
    assert time.slept == pytest.approx([0.02] * 3)  # frame 0 is due at once
    assert player.dropped == 0
    assert all(set(w) <= set(SERVO_NAMES) for w in driver.writes)


def test_a_long_event_into_an_empty_queue_keeps_all_its_frames():
    """Motion arriving faster than real time is early, not late."""

    async def main():
        player, _ = _player()
        await player.enqueue(_event(100.0, n_frames=150, fps=30))
        await player.enqueue(_event(100.0, n_frames=150, fps=30, pts=5.0))
        return player

    player = asyncio.run(main())
    assert player.queue.qsize() == 300 and player.dropped == player.overflow == 0


def test_the_queue_is_bounded_only_to_cap_memory():
    async def main():
        player, _ = _player(max_backlog_s=1.0)
        await player.enqueue(_event(100.0, n_frames=80, fps=50))
        return player

    player = asyncio.run(main())
    assert player.queue.qsize() == 50 and player.overflow == 30
    assert player.dropped == 0


def test_frames_follow_pts_and_only_late_ones_are_dropped():
    player, time = _player(late_tolerance_s=0.1)

    def between(i):
        if i == 1:
            time.t += 1.0  # the second chunk comes 1 s after its due time

    # Utterance: [0, 0.5) then [0.5, 1.0) at 10 fps.
    _play(
        player,
        _event(100.0, n_frames=5, fps=10, pts=0.0),
        _event(100.0, n_frames=5, fps=10, pts=0.5),
        between=between,
    )
    # Chunk 2 due at 0.5-0.9 s, played from 1.4 s: every frame is late.
    assert player.dropped == 5
    assert time.slept == pytest.approx([0.1] * 4)


def test_a_new_utterance_re_anchors_the_clock():
    player, time = _player(late_tolerance_s=0.1)

    def between(i):
        if i == 1:
            time.t += 10.0  # a long pause, then pts goes back to 0

    _play(
        player,
        _event(100.0, n_frames=3, fps=10, pts=0.0),
        _event(100.0, n_frames=3, fps=10, pts=0.0),
        between=between,
    )
    assert player.dropped == 0
    assert time.slept == pytest.approx([0.1] * 4)


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


def test_speaker_is_muted_without_sounddevice(monkeypatch, caplog):
    import huri_inmoov_robot.audio as audio

    monkeypatch.setattr(audio, "sounddevice_available", lambda: False)
    raw = _raw(subscribe=["inmoov.servo_angles", "huri.audio.out"])
    robot = build(raw)
    assert "speaker off" in caplog.text
    handlers = robot.client._subs["huri.audio.out"].handlers
    assert [h.__name__ for h in handlers] == ["_mute"]


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


def test_a_dead_motion_task_ends_the_session_and_closes_the_driver(caplog):
    """The driver raises (USB cable out): run() leaves, logs, closes."""

    class Unplugged(FakeDriver):
        def write(self, angles):
            raise OSError("device disconnected")

    async def serve(ws):
        await ws.recv()
        await ws.send(json.dumps({"type": "session_init"}))
        await ws.send(encode("inmoov.servo_angles", _event(110.0, n_frames=10)))
        await ws.wait_closed()  # the server keeps the session open

    async def main():
        async with websockets.serve(serve, "127.0.0.1", 0) as server:
            port = server.sockets[0].getsockname()[1]
            robot = build(_raw(huri_url=f"ws://127.0.0.1:{port}/session"))
            robot.driver = robot.controller.driver = Unplugged()
            await asyncio.wait_for(run(robot), timeout=10)
            return robot

    robot = asyncio.run(main())
    assert robot.driver.closed
    assert robot.controller.estopped
    assert "motion task ended" in caplog.text
    assert "device disconnected" in caplog.text
