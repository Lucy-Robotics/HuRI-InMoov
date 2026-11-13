"""Audio routes between Lucy's audio topics and HuRI's voice pipeline.

Lucy (lucy_ros_packages, lucy_control_panel) carries audio as
audio_common_msgs/msg/AudioStamped on two topics:

    /mic_audio   microphone: the panel publishes the browser's mic there
                 (16 kHz mono int16, 1024 samples per message), the robot's
                 audio_capturer_node its own (48 kHz stereo int16)
    /audio       playback: the robot's audio_player_node and the panel's
                 speakers play it (each resamples to its device)

HuRI's vad takes huri.audio.in as raw int16 mono PCM at its sample rate, in
frames of exactly its block_duration (webrtcvad: 10, 20 or 30 ms), and the tts
gives huri.audio.out as float32 mono at the engine's rate. The routes convert
between the two: format, channels (averaged to mono), sample rate (linear
interpolation, no anti-aliasing filter: publish 16 kHz when the source can)
and framing.
"""

import base64
import time
from typing import Any, Callable, Dict, List, Mapping, Optional, Tuple

import numpy as np
from huri_core.core.events import BytesEvent
from huri_events.audio import Audio

AUDIO_STAMPED = "audio_common_msgs/msg/AudioStamped"

# PortAudio sample formats, as AudioInfo.format carries them.
PA_FLOAT32, PA_INT32, PA_INT16, PA_INT8, PA_UINT8 = 1, 2, 8, 16, 32

# format -> (AudioData field, dtype, full scale, zero)
_FORMATS: Dict[int, Tuple[str, Any, float, float]] = {
    PA_FLOAT32: ("float32_data", np.float32, 1.0, 0.0),
    PA_INT32: ("int32_data", np.int32, 2.0**31, 0.0),
    PA_INT16: ("int16_data", np.int16, 2.0**15, 0.0),
    PA_INT8: ("int8_data", np.int8, 2.0**7, 0.0),
    PA_UINT8: ("uint8_data", np.uint8, 2.0**7, 128.0),
}


def mono_samples(msg: Mapping[str, Any]) -> Tuple[np.ndarray, int]:
    """(float32 mono samples in [-1, 1], sample rate) of an AudioStamped.

    The samples are read from the field of `info.format`, or else from the
    first non-empty field (the panel's player is as lenient); interleaved
    channels are averaged."""

    audio = msg.get("audio") or {}
    info = audio.get("info") or {}
    data = audio.get("audio_data") or {}
    rate = int(info.get("rate") or 0)
    if rate <= 0:
        raise ValueError(f"AudioStamped without a sample rate: {info!r}")
    declared = int(info.get("format") or PA_INT16)
    if declared not in _FORMATS:
        raise ValueError(f"unsupported PortAudio format {declared}")
    order = [declared] + [f for f in _FORMATS if f != declared]
    samples = np.zeros(0, np.float32)
    for fmt in order:
        field, dtype, scale, zero = _FORMATS[fmt]
        raw = data.get(field)
        if raw is None or len(raw) == 0:
            continue
        if isinstance(raw, str):  # rosbridge sends uint8[] as base64
            values = np.frombuffer(base64.b64decode(raw), np.uint8)
        else:
            values = np.asarray(raw, dtype)
        samples = (values.astype(np.float32) - zero) / scale
        break
    channels = max(1, int(info.get("channels") or 1))
    if channels > 1:
        frames = len(samples) // channels
        samples = samples[: frames * channels].reshape(frames, channels).mean(axis=1)
    return samples.astype(np.float32, copy=False), rate


def to_int16(samples: np.ndarray) -> np.ndarray:
    """Little-endian int16 PCM of float samples in [-1, 1] (clipped)."""

    clipped = np.clip(np.asarray(samples, np.float32), -1.0, 1.0)
    return np.round(clipped * 32767).astype("<i2")


class LinearResampler:
    """Streaming linear interpolation from `src` to `dst` Hz: chunk after
    chunk, without a seam (the last sample of a chunk is kept for the next)."""

    def __init__(self, src: int, dst: int):
        self.src = src
        self.dst = dst
        self._step = src / dst
        self._pos = 0.0  # of the next output, in input samples from _prev
        self._prev: Optional[np.ndarray] = None

    def __call__(self, samples: np.ndarray) -> np.ndarray:
        if self.src == self.dst or len(samples) == 0:
            return samples
        x = samples if self._prev is None else np.concatenate([self._prev, samples])
        last = len(x) - 1
        count = int(np.ceil((last - self._pos) / self._step)) if last > self._pos else 0
        t = self._pos + self._step * np.arange(count)
        out = np.interp(t, np.arange(len(x)), x).astype(np.float32)
        self._pos += self._step * count - last
        self._prev = x[-1:]
        return out


class MicToHuri:
    """The ROS microphone -> huri.audio.in: int16 mono PCM at `sample_rate`,
    in frames of `frame_duration` (the vad module's sample_rate and
    block_duration)."""

    ros_type = AUDIO_STAMPED

    def __init__(
        self,
        ros_topic: str = "/mic_audio",
        huri_topic: str = "huri.audio.in",
        sample_rate: int = 16000,
        frame_duration: float = 0.030,
    ):
        self.ros_topic = ros_topic
        self.huri_topic = huri_topic
        self.sample_rate = sample_rate
        self.frame_size = round(sample_rate * frame_duration)
        if self.frame_size <= 0:
            raise ValueError("sample_rate * frame_duration must be at least 1 sample")
        self._resampler: Optional[LinearResampler] = None
        self._pending = np.zeros(0, "<i2")

    def convert(self, msg: Mapping[str, Any]) -> List[BytesEvent]:
        samples, rate = mono_samples(msg)
        if self._resampler is None or self._resampler.src != rate:
            self._resampler = LinearResampler(rate, self.sample_rate)
        pcm = np.concatenate([self._pending, to_int16(self._resampler(samples))])
        whole = len(pcm) // self.frame_size * self.frame_size
        self._pending = pcm[whole:]
        frames = pcm[:whole].reshape(-1, self.frame_size)
        return [BytesEvent(data=frame.tobytes()) for frame in frames]


def audio_stamped(
    pcm: np.ndarray, rate: int, frame_id: str, stamp: float
) -> Dict[str, Any]:
    """AudioStamped (rosbridge JSON) of int16 mono samples."""

    sec = int(stamp)
    return {
        "header": {
            "stamp": {"sec": sec, "nanosec": int((stamp - sec) * 1e9)},
            "frame_id": frame_id,
        },
        "audio": {
            "info": {
                "format": PA_INT16,
                "channels": 1,
                "rate": rate,
                "chunk": len(pcm),
            },
            "audio_data": {
                "float32_data": [],
                "int32_data": [],
                "int16_data": pcm.tolist(),
                "int8_data": [],
                "uint8_data": [],
            },
        },
    }


class HuriToSpeaker:
    """huri.audio.out -> the ROS playback topic: int16 mono at the TTS rate,
    `chunk` samples per message (int16 keeps rosbridge's JSON small).

    The stamps follow the audio, not the publish time: chunk after chunk, and
    never before now. audio_player_node reads a stamp gap longer than the
    audio as a pause (and flushes its resampler); the TTS runs faster than
    real time, so publish times would not tell."""

    event_cls = Audio
    ros_type = AUDIO_STAMPED

    def __init__(
        self,
        ros_topic: str = "/audio",
        huri_topic: str = "huri.audio.out",
        chunk: int = 1024,
        frame_id: str = "huri_tts",
        clock: Callable[[], float] = time.time,
    ):
        if chunk <= 0:
            raise ValueError("chunk must be positive")
        self.ros_topic = ros_topic
        self.huri_topic = huri_topic
        self.chunk = chunk
        self.frame_id = frame_id
        self._clock = clock
        self._next_stamp = 0.0

    def convert(self, audio: Audio) -> List[Dict[str, Any]]:
        rate = int(audio.sample_rate)
        if rate <= 0:
            raise ValueError(f"huri.audio.out without a sample rate: {rate}")
        pcm = to_int16(np.asarray(audio.data, np.float32).reshape(-1))
        start = max(self._clock(), self._next_stamp)
        self._next_stamp = start + len(pcm) / rate
        return [
            audio_stamped(
                pcm[i : i + self.chunk], rate, self.frame_id, start + i / rate
            )
            for i in range(0, len(pcm), self.chunk)
        ]
