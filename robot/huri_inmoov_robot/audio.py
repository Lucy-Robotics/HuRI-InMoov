"""Optional robot audio: microphone to huri.audio.in, huri.audio.out to the
speaker. Needs sounddevice (the `audio` extra); the robot runs without it."""

import asyncio
import logging

import numpy as np
from huri_core.core.client import HuRIClient
from huri_core.core.events import BytesEvent
from huri_events.audio import Audio

logger = logging.getLogger("huri_inmoov_robot.audio")


def sounddevice_available() -> bool:
    """sounddevice and its PortAudio library are there (the [audio] extra)."""

    try:
        import sounddevice  # noqa: F401
    except (ImportError, OSError):  # OSError: no PortAudio library
        return False
    return True


async def send_microphone(
    client: HuRIClient,
    topic: str = "huri.audio.in",
    sample_rate: int = 16000,
    frame_duration: float = 0.030,
) -> None:
    """Streams 16-bit mono microphone frames until cancelled (the format the
    curated vad module expects)."""

    import sounddevice as sd

    loop = asyncio.get_running_loop()
    queue: "asyncio.Queue[np.ndarray]" = asyncio.Queue()

    def callback(indata: np.ndarray, frames, time, status) -> None:
        if status:
            logger.warning("microphone: %s", status)
        loop.call_soon_threadsafe(queue.put_nowait, indata.copy())

    with sd.InputStream(
        samplerate=sample_rate,
        channels=1,
        dtype="int16",
        callback=callback,
        blocksize=int(sample_rate * frame_duration),
    ):
        while True:
            chunk = await queue.get()
            await client.send(topic, BytesEvent(data=chunk.tobytes()))


class Speaker:
    """Plays huri.audio.out chunks in order (float32 mono), one stream per
    sample rate."""

    def __init__(self) -> None:
        self._stream = None
        self._rate = 0

    async def play(self, audio: Audio) -> None:
        import sounddevice as sd

        if self._stream is None or self._rate != audio.sample_rate:
            self.close()
            self._stream = sd.OutputStream(
                samplerate=audio.sample_rate, channels=1, dtype="float32"
            )
            self._stream.start()
            self._rate = audio.sample_rate
        samples = np.asarray(audio.data, dtype=np.float32).reshape(-1, 1)
        if samples.size:
            await asyncio.to_thread(self._stream.write, samples)

    def close(self) -> None:
        if self._stream is not None:
            self._stream.close()
            self._stream = None
