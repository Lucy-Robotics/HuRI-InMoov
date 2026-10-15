"""E-stop triggers: a signal (SIGUSR1 by default) and the keyboard.

    kill -USR1 <pid>          e-stop from another process (a hardware button
                              daemon, a remote shell)
    <Enter> / e / stop        e-stop from the terminal
    r / reset                 leave the e-stop

A software e-stop is no substitute for a hardware one: wire a physical switch
cutting the servos' power supply.
"""

import asyncio
import logging
import signal
import threading
from typing import Iterable, TextIO

from huri_inmoov_robot.safety import SafetyController

logger = logging.getLogger("huri_inmoov_robot.estop")

STOP_WORDS = {"", "e", "s", "stop", "estop"}
RESET_WORDS = {"r", "reset"}


def handle_key(controller: SafetyController, line: str) -> None:
    word = line.strip().lower()
    if word in STOP_WORDS:
        controller.estop("keyboard")
    elif word in RESET_WORDS:
        controller.reset()
    else:
        logger.info("unknown key %r: <Enter> e-stops, 'r' resets", word)


def install_signal_estop(
    loop: asyncio.AbstractEventLoop,
    controller: SafetyController,
    signals: Iterable[signal.Signals] = (signal.SIGUSR1,),
) -> None:
    for sig in signals:
        loop.add_signal_handler(sig, controller.estop, f"signal {sig.name}")


def start_keyboard_estop(
    loop: asyncio.AbstractEventLoop, controller: SafetyController, stream: TextIO
) -> threading.Thread:
    """Reads `stream` lines in a daemon thread (stdin blocks), handling each in
    the event loop. Stops at end of file."""

    def read() -> None:
        for line in iter(stream.readline, ""):
            loop.call_soon_threadsafe(handle_key, controller, line)

    thread = threading.Thread(target=read, name="estop-keyboard", daemon=True)
    thread.start()
    return thread
