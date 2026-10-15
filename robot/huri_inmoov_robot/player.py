"""Plays ServoAngles frames at their fps through the SafetyController."""

import asyncio
import logging
from typing import Awaitable, Callable, Dict, Tuple

from huri_events_inmoov import ServoAngles
from huri_inmoov_robot.safety import SafetyController

logger = logging.getLogger("huri_inmoov_robot.player")

Sleep = Callable[[float], Awaitable[None]]


class Player:
    """Frames wait in a queue and are played one per 1/fps.

    The queue holds at most `max_backlog_s` seconds of frames: beyond that the
    robot is late, and the oldest frames are dropped rather than played late.
    TODO: schedule frames on `pts` against the audio playback clock.
    """

    def __init__(
        self,
        controller: SafetyController,
        max_backlog_s: float = 2.0,
        sleep: Sleep = asyncio.sleep,
    ):
        self.controller = controller
        self.max_backlog_s = max_backlog_s
        self.sleep = sleep
        self.queue: "asyncio.Queue[Tuple[Dict[str, float], float]]" = asyncio.Queue()
        self.dropped = 0

    async def enqueue(self, event: ServoAngles) -> None:
        """The inmoov.servo_angles handler."""

        if self.controller.estopped:
            return
        period = 1.0 / event.fps
        for i in range(event.angles.shape[0]):
            self.queue.put_nowait((event.frame(i), period))
        max_frames = max(1, int(self.max_backlog_s * event.fps))
        while self.queue.qsize() > max_frames:
            self.queue.get_nowait()
            self.dropped += 1
        if self.dropped:
            logger.debug("dropped %d late frames so far", self.dropped)

    def clear(self) -> None:
        while not self.queue.empty():
            self.queue.get_nowait()

    async def run(self) -> None:
        while True:
            frame, period = await self.queue.get()
            if not self.controller.command(frame):
                self.clear()
                continue
            await self.sleep(period)


async def watchdog_loop(controller: SafetyController, period: float = 0.02) -> None:
    while True:
        controller.tick()
        await asyncio.sleep(period)


async def park(
    controller: SafetyController, timeout: float = 5.0, period: float = 0.02
) -> bool:
    """Walk back to rest at the speed limit (shutdown); True once there."""

    loop = asyncio.get_running_loop()
    deadline = loop.time() + timeout
    while not controller.estopped and not controller.at_rest():
        if loop.time() > deadline:
            logger.warning("not at rest after %.1fs: stopping anyway", timeout)
            return False
        controller.go_rest()
        await asyncio.sleep(period)
    return controller.at_rest()
