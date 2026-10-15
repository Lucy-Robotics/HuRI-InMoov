"""Servo drivers: where the robot's angles end up.

SerialDriver line protocol (ASCII, one command per line, ``\\n``-terminated),
implemented by arduino/huri_inmoov_servos/huri_inmoov_servos.ino:

    S<id>:<deg>   move servo <id> (0-based channel, see robot.channels) to <deg>
                  degrees, one decimal, e.g. ``S3:92.5``
    D             detach every servo (no torque: the arms fall under gravity)
    A             attach every servo again (at its last commanded angle)

The board answers nothing; it ignores a line it does not parse.

A write that does not complete within `write_timeout` (the board stopped
reading: hung sketch, full USB buffer) raises DriverFault instead of blocking
the event loop, where the software e-stop runs.
"""

import logging
from typing import Any, Dict, List, Mapping, Optional, Protocol, Tuple

logger = logging.getLogger("huri_inmoov_robot.drivers")


class DriverFault(RuntimeError):
    """The driver cannot command the servos any more (e.g. a stuck port)."""


class ServoDriver(Protocol):
    def write(self, angles: Mapping[str, float]) -> None:
        """Move the named servos to these angles (degrees), now."""

    def detach(self) -> None:
        """Remove torque from every servo."""

    def attach(self) -> None:
        """Give torque back to every servo."""

    def close(self) -> None: ...


class FakeDriver:
    """Records every command, for tests and dry runs (--fake)."""

    def __init__(self) -> None:
        self.commands: List[Tuple[str, Any]] = []
        self.position: Dict[str, float] = {}
        self.attached = True
        self.closed = False

    def write(self, angles: Mapping[str, float]) -> None:
        self.commands.append(("write", dict(angles)))
        self.position.update(angles)

    def detach(self) -> None:
        self.commands.append(("detach", None))
        self.attached = False

    def attach(self) -> None:
        self.commands.append(("attach", None))
        self.attached = True

    def close(self) -> None:
        self.closed = True

    @property
    def writes(self) -> List[Dict[str, float]]:
        return [args for kind, args in self.commands if kind == "write"]


class SerialDriver:
    """An Arduino running huri_inmoov_servos.ino, over a serial port.

    :port: serial port (/dev/ttyACM0) or any pyserial URL (loop:// in tests).
    :channels: servo name -> channel id on the board; servos not listed are
        not driven (logged once).
    :write_timeout: seconds a write may block before DriverFault.
    """

    def __init__(
        self,
        port: str,
        channels: Mapping[str, int],
        baudrate: int = 115200,
        serial: Optional[Any] = None,
        write_timeout: float = 0.05,
    ):
        if serial is None:
            import serial as pyserial

            serial = pyserial.serial_for_url(
                port, baudrate=baudrate, timeout=0, write_timeout=write_timeout
            )
        self.serial = serial
        self.channels = dict(channels)
        self._unmapped: set = set()

    def _send(self, line: str) -> None:
        from serial import SerialTimeoutException

        try:
            self.serial.write(line.encode("ascii"))
        except SerialTimeoutException as e:
            raise DriverFault(f"serial write timed out: {e}") from e

    def write(self, angles: Mapping[str, float]) -> None:
        lines = []
        for name, angle in angles.items():
            channel = self.channels.get(name)
            if channel is None:
                if name not in self._unmapped:
                    self._unmapped.add(name)
                    logger.info("servo %s has no channel: not driven", name)
                continue
            lines.append(f"S{channel}:{angle:.1f}\n")
        if lines:
            self._send("".join(lines))

    def detach(self) -> None:
        self._send("D\n")

    def attach(self) -> None:
        self._send("A\n")

    def close(self) -> None:
        self.serial.close()
