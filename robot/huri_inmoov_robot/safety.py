"""Robot-side safety, independent of the server.

Every angle goes through a SafetyController before the driver:

- hard limits: each target is clamped to the servo's [min, max];
- max speed: a servo moves at most max_speed * dt per command (dt the real
  time since the last one, capped at max_dt so a pause never allows a jump);
- watchdog: no frame for `watchdog_s` -> "hold" (stop where it is) or "rest"
  (walk back to rest at the speed limit);
- e-stop: latched; nothing moves until reset(). "hold" stops sending angles
  (servos keep their torque), "detach" removes the torque (an InMoov's arms
  then fall: only when that is safe for your build);
- driver fault (DriverFault, e.g. a stuck serial port): latches the e-stop
  without trying any more writes.

The robot is assumed at rest when the controller starts (the Arduino sketch
attaches every servo at its rest angle).
"""

import logging
import math
import time
from typing import Callable, Dict, Mapping, Optional

from huri_inmoov_robot.drivers import DriverFault, ServoDriver
from huri_inmoov_robot.limits import ServoLimit

logger = logging.getLogger("huri_inmoov_robot.safety")

TIMEOUT_ACTIONS = ("hold", "rest")
ESTOP_ACTIONS = ("hold", "detach")


class SafetyController:
    def __init__(
        self,
        driver: ServoDriver,
        limits: Mapping[str, ServoLimit],
        *,
        watchdog_s: float = 1.5,
        on_timeout: str = "rest",
        estop_action: str = "hold",
        max_dt: float = 0.1,
        clock: Callable[[], float] = time.monotonic,
    ):
        if on_timeout not in TIMEOUT_ACTIONS:
            raise ValueError(f"on_timeout must be one of {TIMEOUT_ACTIONS}")
        if estop_action not in ESTOP_ACTIONS:
            raise ValueError(f"estop_action must be one of {ESTOP_ACTIONS}")
        self.driver = driver
        self.limits = dict(limits)
        self.watchdog_s = watchdog_s
        self.on_timeout = on_timeout
        self.estop_action = estop_action
        self.max_dt = max_dt
        self.clock = clock
        self.position: Dict[str, float] = {n: lim.rest for n, lim in limits.items()}
        self.estopped = False
        self.estop_reason = ""
        self.timed_out = False
        self._last_frame: Optional[float] = None
        self._last_move = clock()
        self._ignored: set = set()

    # --- motion -------------------------------------------------------------

    def command(
        self, targets: Mapping[str, float], now: Optional[float] = None
    ) -> bool:
        """One frame from the server. False when refused (e-stop)."""

        if self.estopped:
            return False
        now = self.clock() if now is None else now
        self._last_frame = now
        if self.timed_out:
            logger.info("frames again: watchdog cleared")
            self.timed_out = False
        self._move(targets, now)
        return True

    def tick(self, now: Optional[float] = None) -> None:
        """Run the watchdog; call it often (the app does, at 50 Hz)."""

        now = self.clock() if now is None else now
        if self.estopped:
            self._last_move = now
            return
        if self._last_frame is None or now - self._last_frame <= self.watchdog_s:
            return
        if not self.timed_out:
            self.timed_out = True
            logger.warning(
                "no frame for %.0f ms: %s", 1000 * self.watchdog_s, self.on_timeout
            )
            # The silence is not time to move in: start walking from now.
            self._last_move = now
            return
        if self.on_timeout == "rest":
            self._move(self.rest_targets(), now)
        else:
            self._last_move = now

    def go_rest(self, now: Optional[float] = None) -> None:
        """One step towards rest at the speed limit (shutdown parking)."""

        if not self.estopped:
            self._move(self.rest_targets(), self.clock() if now is None else now)

    def rest_targets(self) -> Dict[str, float]:
        return {n: lim.rest for n, lim in self.limits.items()}

    def at_rest(self, tolerance: float = 1e-6) -> bool:
        return all(
            abs(self.position[n] - lim.rest) <= tolerance
            for n, lim in self.limits.items()
        )

    def _move(self, targets: Mapping[str, float], now: float) -> None:
        dt = min(max(now - self._last_move, 0.0), self.max_dt)
        self._last_move = now
        out: Dict[str, float] = {}
        for name, target in targets.items():
            limit = self.limits.get(name)
            if limit is None:
                if name not in self._ignored:
                    self._ignored.add(name)
                    logger.warning("servo %s has no limits: ignored", name)
                continue
            if not math.isfinite(target):
                logger.warning("servo %s: non-finite angle %r ignored", name, target)
                continue
            target = min(max(target, limit.min), limit.max)
            current = self.position[name]
            step = limit.max_speed * dt
            new = current + min(max(target - current, -step), step)
            if new != current:
                self.position[name] = new
                out[name] = new
        if out:
            try:
                self.driver.write(out)
            except DriverFault as e:
                self.fault(e)

    # --- e-stop -------------------------------------------------------------

    def estop(self, reason: str = "e-stop") -> None:
        """Stop now, and stay stopped until reset()."""

        if self.estopped:
            return
        # Latched before touching the driver: a stuck port cannot keep the
        # robot out of the e-stop state.
        self.estopped = True
        self.estop_reason = reason
        logger.critical("E-STOP (%s): %s", reason, self.estop_action)
        if self.estop_action == "detach":
            try:
                self.driver.detach()
            except Exception:
                logger.exception("e-stop: detach failed")

    def fault(self, error: BaseException) -> None:
        """The driver failed: latch the e-stop, write nothing more."""

        if self.estopped:
            return
        self.estopped = True
        self.estop_reason = f"driver fault: {error}"
        logger.critical("E-STOP (driver fault): %s", error)

    def reset(self) -> None:
        """Leave the e-stop. After "detach" the servos get their torque back
        at their last commanded angle: check the robot before resetting."""

        if not self.estopped:
            return
        if self.estop_action == "detach":
            self.driver.attach()
        self.estopped = False
        self.estop_reason = ""
        self._last_frame = None
        self.timed_out = False
        self._last_move = self.clock()
        logger.warning("e-stop reset")
