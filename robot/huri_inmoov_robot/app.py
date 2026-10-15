"""huri-inmoov-robot: the InMoov robot as a HuRI client.

    huri-inmoov-robot --config robot/config/robot_example.yaml [--fake]

The config is a HuRI client YAML (huri_url, modules, send, subscribe) plus a
`robot:` section the server never reads:

    robot:
      driver: {type: serial, port: /dev/ttyACM0, baudrate: 115200,
               write_timeout: 0.05}                          # or {type: fake}
      channels: {head.neck: 0, head.rothead: 1, ...}   # servo -> board channel
      watchdog_ms: 1500         # longer than a pause between sentences
      on_timeout: rest          # or hold
      estop_action: hold        # or detach
      limits: {head.neck: {min: 40, max: 140}}        # overrides of DEFAULT_LIMITS
      audio: {mic: true, speaker: true, sample_rate: 16000, frame_duration: 0.030}
"""

import argparse
import asyncio
import logging
import signal
import sys
from dataclasses import dataclass, field
from typing import Any, Dict, List, Mapping, Optional

from huri_core.core.client import HuRIClient, load_raw_client_config
from huri_core.core.dataclasses.config import ClientConfig
from huri_events.audio import Audio

from huri_events_inmoov import ServoAngles
from huri_inmoov_robot.drivers import FakeDriver, SerialDriver, ServoDriver
from huri_inmoov_robot.estop import install_signal_estop, start_keyboard_estop
from huri_inmoov_robot.limits import load_limits
from huri_inmoov_robot.player import Player, park, watchdog_loop
from huri_inmoov_robot.safety import SafetyController

logger = logging.getLogger("huri_inmoov_robot")

ANGLES_TOPIC = "inmoov.servo_angles"
AUDIO_IN_TOPIC = "huri.audio.in"
AUDIO_OUT_TOPIC = "huri.audio.out"
HANDLED = {ANGLES_TOPIC, AUDIO_OUT_TOPIC}


def make_driver(robot: Mapping[str, Any], fake: bool = False) -> ServoDriver:
    spec = dict(robot.get("driver", {"type": "fake"}))
    kind = "fake" if fake else spec.pop("type", "serial")
    if kind == "fake":
        return FakeDriver()
    if kind == "serial":
        if "port" not in spec:
            raise ValueError("robot.driver: a serial driver needs a port")
        return SerialDriver(
            spec["port"],
            channels=robot.get("channels", {}),
            baudrate=int(spec.get("baudrate", 115200)),
            write_timeout=float(spec.get("write_timeout", 0.05)),
        )
    raise ValueError(f"robot.driver.type must be serial or fake, got {kind!r}")


@dataclass
class Robot:
    config: ClientConfig
    settings: Dict[str, Any]
    driver: ServoDriver
    controller: SafetyController
    player: Player
    client: HuRIClient
    tasks: List["asyncio.Task[Any]"] = field(default_factory=list)


def build(raw: Mapping[str, Any], fake: bool = False) -> Robot:
    """Everything but the connection, from a loaded client config."""

    config = ClientConfig.from_dict(dict(raw))
    settings = dict(raw.get("robot") or {})
    if ANGLES_TOPIC not in config.subscribe:
        raise ValueError(f"the config must subscribe to {ANGLES_TOPIC}")
    unhandled = sorted(set(config.subscribe) - HANDLED)
    if unhandled:
        raise ValueError(f"huri-inmoov-robot handles no {unhandled}: unsubscribe")
    driver = make_driver(settings, fake)
    controller = SafetyController(
        driver,
        load_limits(settings.get("limits", {})),
        watchdog_s=float(settings.get("watchdog_ms", 1500)) / 1000,
        on_timeout=settings.get("on_timeout", "rest"),
        estop_action=settings.get("estop_action", "hold"),
    )
    player = Player(controller)
    client = HuRIClient.from_config(config)
    client.on(ANGLES_TOPIC, ServoAngles, player.enqueue)
    audio = settings.get("audio", {})
    speaker = AUDIO_OUT_TOPIC in config.subscribe and audio.get("speaker", True)
    if speaker:
        from huri_inmoov_robot.audio import Speaker, sounddevice_available

        if not sounddevice_available():
            logger.warning("no sounddevice: speaker off ([audio] extra)")
            speaker = False
        else:
            client.on(AUDIO_OUT_TOPIC, Audio, Speaker().play)
    if AUDIO_OUT_TOPIC in config.subscribe and not speaker:

        async def _mute(_: Audio) -> None:
            return None

        client.on(AUDIO_OUT_TOPIC, Audio, _mute)
    return Robot(config, settings, driver, controller, player, client)


async def run(robot: Robot, stop: Optional[asyncio.Event] = None) -> None:
    """Connect, play until the server closes the session or `stop` is set,
    then park the robot at rest.

    The motion tasks (player, watchdog) dying ends the session too: a robot
    without its watchdog must not keep talking and listening. Their error is
    logged and the e-stop latched."""

    stop = stop or asyncio.Event()
    audio = robot.settings.get("audio", {})
    try:
        async with robot.client:
            motion = {
                asyncio.create_task(robot.player.run()),
                asyncio.create_task(watchdog_loop(robot.controller)),
            }
            robot.tasks = list(motion)
            if AUDIO_IN_TOPIC in robot.config.send and audio.get("mic", True):
                from huri_inmoov_robot.audio import (
                    send_microphone,
                    sounddevice_available,
                )

                if not sounddevice_available():
                    logger.warning("no sounddevice: microphone off ([audio] extra)")
                else:
                    robot.tasks.append(
                        asyncio.create_task(
                            send_microphone(
                                robot.client,
                                AUDIO_IN_TOPIC,
                                int(audio.get("sample_rate", 16000)),
                                float(audio.get("frame_duration", 0.030)),
                            )
                        )
                    )
            closed = asyncio.create_task(robot.client.wait_closed())
            stopped = asyncio.create_task(stop.wait())
            robot.tasks += [closed, stopped]
            done, _ = await asyncio.wait(
                {closed, stopped, *motion}, return_when=asyncio.FIRST_COMPLETED
            )
            for task in done & motion:
                error = None if task.cancelled() else task.exception()
                logger.critical(
                    "motion task ended: %r; leaving the session",
                    error,
                    exc_info=error,
                )
                try:
                    robot.controller.estop("motion task died")
                except Exception:
                    logger.exception("e-stop failed")
    finally:
        for task in robot.tasks:
            task.cancel()
        await asyncio.gather(*robot.tasks, return_exceptions=True)
        robot.player.clear()
        try:
            await park(robot.controller)
        except Exception:
            logger.exception("parking failed")
        finally:
            try:
                robot.driver.close()
            except Exception:
                logger.exception("closing the driver failed")


async def _main(args: argparse.Namespace) -> None:
    robot = build(load_raw_client_config(args.config), fake=args.fake)
    loop = asyncio.get_running_loop()
    stop = asyncio.Event()
    install_signal_estop(loop, robot.controller)
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, stop.set)
    if not args.no_keyboard and sys.stdin.isatty():
        start_keyboard_estop(loop, robot.controller, sys.stdin)
        print("<Enter> e-stops the robot, 'r' + <Enter> resets, Ctrl+C quits.")
    await run(robot, stop)


def main(argv: Optional[List[str]] = None) -> None:
    parser = argparse.ArgumentParser(
        prog="huri-inmoov-robot",
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--config", required=True, help="client YAML with robot:")
    parser.add_argument(
        "--fake", action="store_true", help="record servo commands, drive nothing"
    )
    parser.add_argument("--no-keyboard", action="store_true", help="no keyboard e-stop")
    parser.add_argument("--log-level", default="INFO")
    args = parser.parse_args(argv)
    logging.basicConfig(
        level=args.log_level.upper(),
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    asyncio.run(_main(args))


if __name__ == "__main__":
    main()
