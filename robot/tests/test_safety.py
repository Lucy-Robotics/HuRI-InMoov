"""Robot-side safety with FakeDriver: limits, speed, watchdog, e-stop."""

import asyncio
import io
import os
import signal

import pytest

from huri_inmoov_robot.drivers import FakeDriver
from huri_inmoov_robot.estop import (
    handle_key,
    install_signal_estop,
    start_keyboard_estop,
)
from huri_inmoov_robot.limits import DEFAULT_LIMITS, ServoLimit, load_limits
from huri_inmoov_robot.safety import SafetyController

NECK = ServoLimit(min=40, max=140, rest=90, max_speed=100)  # 100 deg/s
LIMITS = {"head.neck": NECK, "head.rothead": ServoLimit(20, 160, 90, 50)}


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


@pytest.fixture
def clock():
    return Clock()


def controller(clock, **kwargs):
    driver = FakeDriver()
    kwargs.setdefault("max_dt", 1.0)
    return driver, SafetyController(driver, LIMITS, clock=clock, **kwargs)


def advance(ctl, clock, targets, seconds, period=0.02):
    """Send `targets` every `period` for `seconds`."""

    for _ in range(round(seconds / period)):
        clock.t += period
        ctl.command(targets)


# --- hard limits --------------------------------------------------------------


def test_targets_are_clamped_to_hard_limits(clock):
    driver, ctl = controller(clock)
    advance(ctl, clock, {"head.neck": 500.0, "head.rothead": -50.0}, seconds=5)
    assert ctl.position["head.neck"] == 140
    assert ctl.position["head.rothead"] == 20
    for write in driver.writes:
        for name, angle in write.items():
            assert LIMITS[name].min <= angle <= LIMITS[name].max


def test_unknown_and_non_finite_targets_are_ignored(clock):
    driver, ctl = controller(clock)
    clock.t = 0.1
    ctl.command({"leftArm.bicep": 50.0, "head.neck": float("nan")})
    assert driver.writes == []
    assert ctl.position["head.neck"] == 90


def test_robot_limits_override_and_validate():
    limits = load_limits({"head.neck": {"min": 50, "max": 130}})
    assert (limits["head.neck"].min, limits["head.neck"].max) == (50, 130)
    assert limits["head.rothead"] == DEFAULT_LIMITS["head.rothead"]
    with pytest.raises(ValueError, match="unknown servo"):
        load_limits({"head.nose": {"min": 0}})
    with pytest.raises(ValueError, match="unknown keys"):
        load_limits({"head.neck": {"speed": 3}})
    with pytest.raises(ValueError, match="min <= rest <= max"):
        load_limits({"head.neck": {"min": 100}})


# --- max speed ----------------------------------------------------------------


def test_speed_is_limited(clock):
    driver, ctl = controller(clock)
    advance(ctl, clock, {"head.neck": 140.0}, seconds=0.2)
    assert ctl.position["head.neck"] == pytest.approx(90 + 100 * 0.2)
    previous = 90.0
    for write in driver.writes:
        assert abs(write["head.neck"] - previous) <= 100 * 0.02 + 1e-9
        previous = write["head.neck"]


def test_a_pause_never_allows_a_jump(clock):
    driver, ctl = controller(clock, max_dt=0.1)
    clock.t = 10.0  # long silence, then a far target
    ctl.command({"head.neck": 140.0})
    assert ctl.position["head.neck"] == pytest.approx(90 + 100 * 0.1)


# --- watchdog -----------------------------------------------------------------


def test_watchdog_walks_back_to_rest(clock):
    driver, ctl = controller(clock, watchdog_s=0.5, on_timeout="rest")
    advance(ctl, clock, {"head.neck": 140.0}, seconds=1.0)
    assert ctl.position["head.neck"] == 140
    clock.t += 0.4
    ctl.tick()
    assert not ctl.timed_out and ctl.position["head.neck"] == 140
    clock.t += 0.2
    ctl.tick()
    assert ctl.timed_out
    n = len(driver.writes)
    for _ in range(100):
        clock.t += 0.02
        ctl.tick()
    assert ctl.at_rest()
    previous = 140.0
    for write in driver.writes[n:]:
        assert abs(write["head.neck"] - previous) <= 100 * 0.02 + 1e-9
        previous = write["head.neck"]
    assert previous == 90


def test_watchdog_hold_stops_sending(clock):
    driver, ctl = controller(clock, watchdog_s=0.5, on_timeout="hold")
    advance(ctl, clock, {"head.neck": 120.0}, seconds=0.5)
    n = len(driver.commands)
    for _ in range(50):
        clock.t += 0.02
        ctl.tick()
    assert ctl.timed_out
    assert len(driver.commands) == n
    assert ctl.position["head.neck"] == 120


def test_watchdog_is_idle_before_the_first_frame(clock):
    driver, ctl = controller(clock, watchdog_s=0.5)
    clock.t = 5.0
    ctl.tick()
    assert not ctl.timed_out and driver.commands == []


def test_frames_clear_the_watchdog(clock):
    driver, ctl = controller(clock, watchdog_s=0.5)
    advance(ctl, clock, {"head.neck": 100.0}, seconds=0.2)
    clock.t += 1.0
    ctl.tick()
    assert ctl.timed_out
    advance(ctl, clock, {"head.neck": 100.0}, seconds=0.04)
    assert not ctl.timed_out


# --- e-stop -------------------------------------------------------------------


def test_estop_hold_refuses_frames_until_reset(clock):
    driver, ctl = controller(clock, estop_action="hold")
    advance(ctl, clock, {"head.neck": 100.0}, seconds=0.2)
    ctl.estop("test")
    n = len(driver.commands)
    clock.t += 0.02
    assert ctl.command({"head.neck": 140.0}) is False
    clock.t += 5.0
    ctl.tick()  # the watchdog does not move an e-stopped robot either
    ctl.go_rest()
    assert len(driver.commands) == n
    ctl.reset()
    clock.t += 0.02
    assert ctl.command({"head.neck": 140.0}) is True
    assert len(driver.commands) == n + 1


def test_estop_detach_removes_torque_and_reset_restores_it(clock):
    driver, ctl = controller(clock, estop_action="detach")
    ctl.estop("test")
    ctl.estop("again")  # latched: once
    assert driver.commands == [("detach", None)] and not driver.attached
    ctl.reset()
    assert driver.commands[-1] == ("attach", None) and driver.attached


def test_reset_does_not_jump(clock):
    """After an e-stop, the speed limit restarts from the reset time."""

    driver, ctl = controller(clock, max_dt=1.0)
    ctl.estop()
    clock.t = 30.0
    ctl.reset()
    clock.t += 0.02
    ctl.command({"head.neck": 140.0})
    assert ctl.position["head.neck"] == pytest.approx(90 + 100 * 0.02)


def test_bad_actions_are_refused():
    with pytest.raises(ValueError):
        SafetyController(FakeDriver(), LIMITS, on_timeout="explode")
    with pytest.raises(ValueError):
        SafetyController(FakeDriver(), LIMITS, estop_action="explode")


@pytest.mark.parametrize("key", ["\n", "e\n", "STOP\n"])
def test_keyboard_stops(clock, key):
    _, ctl = controller(clock)
    handle_key(ctl, key)
    assert ctl.estopped and ctl.estop_reason == "keyboard"
    handle_key(ctl, "r\n")
    assert not ctl.estopped


def test_keyboard_thread_reads_the_stream(clock):
    _, ctl = controller(clock)

    async def main():
        start_keyboard_estop(asyncio.get_running_loop(), ctl, io.StringIO("e\n"))
        for _ in range(100):
            if ctl.estopped:
                return
            await asyncio.sleep(0.01)

    asyncio.run(main())
    assert ctl.estopped


def test_signal_stops(clock):
    _, ctl = controller(clock)

    async def main():
        loop = asyncio.get_running_loop()
        install_signal_estop(loop, ctl)
        try:
            os.kill(os.getpid(), signal.SIGUSR1)
            for _ in range(100):
                if ctl.estopped:
                    return
                await asyncio.sleep(0.01)
        finally:
            loop.remove_signal_handler(signal.SIGUSR1)

    asyncio.run(main())
    assert ctl.estopped and "SIGUSR1" in ctl.estop_reason
