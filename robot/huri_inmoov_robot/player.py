"""Plays ServoAngles frames on their pts through the SafetyController."""

import asyncio
import logging
import time
from typing import Awaitable, Callable, Dict, Optional, Tuple

from huri_events_inmoov import ServoAngles
from huri_inmoov_robot.safety import SafetyController

logger = logging.getLogger("huri_inmoov_robot.player")

Sleep = Callable[[float], Awaitable[None]]
Clock = Callable[[], float]

# A queued frame: its angles, the utterance it belongs to, and its time in
# that utterance (pts + i / fps, seconds).
Frame = Tuple[Dict[str, float], int, float]


class Player:
    """Frames wait in a queue and are played at their presentation time.

    Frame i of an event is due at `pts + i / fps` seconds of its utterance.
    The utterance's clock is anchored when its first frame plays (that frame
    is due "now"); every later frame is scheduled against that anchor. A new
    utterance starts when an event's pts goes back (to 0, or before the end
    of the previous event), and re-anchors the clock.

    A frame is dropped only when it is late: its due time already passed by
    more than `late_tolerance_s` (the robot fell behind, or the rest of an
    utterance came after its time). Frames arriving early (motion produced
    faster than real time) are kept: the queue is bounded by `max_backlog_s`
    only to cap memory.

    TODO: anchor on the speaker's playback position instead of the first
    frame's wall-clock time, to follow the audio exactly.
    """

    def __init__(
        self,
        controller: SafetyController,
        max_backlog_s: float = 60.0,
        late_tolerance_s: float = 0.2,
        sleep: Sleep = asyncio.sleep,
        clock: Clock = time.monotonic,
    ):
        self.controller = controller
        self.max_backlog_s = max_backlog_s
        self.late_tolerance_s = late_tolerance_s
        self.sleep = sleep
        self.clock = clock
        self.queue: "asyncio.Queue[Frame]" = asyncio.Queue()
        self.dropped = 0  # late frames
        self.overflow = 0  # frames beyond max_backlog_s
        self._utterance = 0
        self._last_end: Optional[float] = None  # pts after the last event queued
        self._playing: Optional[int] = None  # utterance whose clock is anchored
        self._anchor = 0.0  # clock time of pts 0 of that utterance

    async def enqueue(self, event: ServoAngles) -> None:
        """The inmoov.servo_angles handler."""

        if self.controller.estopped:
            return
        n_frames = event.angles.shape[0]
        if n_frames == 0:
            return
        period = 1.0 / event.fps
        if (
            self._last_end is None
            or event.pts <= 0
            or event.pts < self._last_end - period / 2
        ):
            self._utterance += 1
        self._last_end = event.pts + n_frames * period
        for i in range(n_frames):
            self.queue.put_nowait(
                (event.frame(i), self._utterance, event.pts + i * period)
            )
        max_frames = max(1, int(self.max_backlog_s * event.fps))
        overflow = 0
        while self.queue.qsize() > max_frames:
            self.queue.get_nowait()
            overflow += 1
        if overflow:
            self.overflow += overflow
            logger.warning(
                "more than %.0fs of motion queued: dropped %d frames",
                self.max_backlog_s,
                overflow,
            )

    def clear(self) -> None:
        while not self.queue.empty():
            self.queue.get_nowait()
        self._playing = None
        self._last_end = None

    async def run(self) -> None:
        while True:
            frame, utterance, t = await self.queue.get()
            now = self.clock()
            if utterance != self._playing:
                self._playing = utterance
                self._anchor = now - t
            due = self._anchor + t
            if now - due > self.late_tolerance_s:
                self.dropped += 1
                logger.debug("dropped a frame %.3fs late", now - due)
                continue
            if due > now:
                await self.sleep(due - now)
            if not self.controller.command(frame):
                self.clear()


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
