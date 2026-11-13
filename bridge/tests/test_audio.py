import base64

import numpy as np
import pytest
from huri_events.audio import Audio

from huri_lucy_bridge.audio import (
    PA_FLOAT32,
    PA_INT16,
    PA_UINT8,
    HuriToSpeaker,
    LinearResampler,
    MicToHuri,
    mono_samples,
    to_int16,
)


def stamped(samples, rate=16000, channels=1, fmt=PA_INT16, field="int16_data"):
    data = {
        "float32_data": [],
        "int32_data": [],
        "int16_data": [],
        "int8_data": [],
        "uint8_data": [],
    }
    data[field] = samples
    return {
        "header": {"stamp": {"sec": 0, "nanosec": 0}, "frame_id": "mic"},
        "audio": {
            "info": {
                "format": fmt,
                "channels": channels,
                "rate": rate,
                "chunk": len(samples) // channels,
            },
            "audio_data": data,
        },
    }


def test_int16_samples_are_scaled_to_unit_floats():
    samples, rate = mono_samples(stamped([0, 16384, -32768]))
    assert rate == 16000
    assert samples.dtype == np.float32
    np.testing.assert_allclose(samples, [0.0, 0.5, -1.0])


def test_interleaved_channels_are_averaged_and_a_partial_frame_dropped():
    # L, R, L, R, L: the robot's capturer publishes stereo.
    samples, _ = mono_samples(stamped([16384, 0, -16384, -16384, 999], channels=2))
    np.testing.assert_allclose(samples, [0.25, -0.5])


def test_float32_and_base64_uint8_samples():
    floats, _ = mono_samples(
        stamped([0.5, -0.25], fmt=PA_FLOAT32, field="float32_data")
    )
    np.testing.assert_allclose(floats, [0.5, -0.25])
    raw = base64.b64encode(bytes([128, 192, 64])).decode()
    uint8, _ = mono_samples(stamped(raw, fmt=PA_UINT8, field="uint8_data"))
    np.testing.assert_allclose(uint8, [0.0, 0.5, -0.5])


def test_samples_come_from_the_filled_field_whatever_the_format_says():
    samples, _ = mono_samples(stamped([0.5], fmt=PA_INT16, field="float32_data"))
    np.testing.assert_allclose(samples, [0.5])


def test_a_message_without_rate_or_with_an_unknown_format_is_refused():
    with pytest.raises(ValueError, match="sample rate"):
        mono_samples(stamped([0], rate=0))
    with pytest.raises(ValueError, match="format"):
        mono_samples(stamped([0], fmt=4))


def test_to_int16_clips():
    np.testing.assert_array_equal(
        to_int16(np.array([0.0, 1.0, -1.0, 2.0, -2.0])),
        [0, 32767, -32767, 32767, -32767],
    )


def test_streaming_resampling_matches_resampling_at_once():
    signal = np.sin(np.linspace(0, 40, 4801)).astype(np.float32)
    whole = LinearResampler(48000, 16000)(signal)
    resampler = LinearResampler(48000, 16000)
    chunks = [resampler(c) for c in np.array_split(signal, [7, 1024, 1025, 3000])]
    streamed = np.concatenate(chunks)
    assert len(whole) == 1600  # the sample at 4800 waits for the next chunk
    np.testing.assert_allclose(streamed, whole, atol=1e-6)


def test_resampling_keeps_the_pitch():
    t48 = np.arange(48000) / 48000
    out = LinearResampler(48000, 16000)(np.sin(2 * np.pi * 440 * t48))
    t16 = np.arange(len(out)) / 16000
    np.testing.assert_allclose(out, np.sin(2 * np.pi * 440 * t16), atol=1e-2)
    up = LinearResampler(16000, 22050)(np.zeros(1600, np.float32))
    assert abs(len(up) - 2205) <= 1


def test_the_panel_mic_is_reframed_losslessly():
    # The panel: 16 kHz mono int16, 1024 samples per message.
    route = MicToHuri(sample_rate=16000, frame_duration=0.030)
    first = np.arange(1024, dtype=np.int16)
    second = np.arange(1024, 2048, dtype=np.int16)
    frames = route.convert(stamped(first.tolist()))
    frames += route.convert(stamped(second.tolist()))
    assert [len(f.data) for f in frames] == [960] * 4  # 480 samples, 30 ms
    got = np.frombuffer(b"".join(f.data for f in frames), "<i2")
    np.testing.assert_array_equal(got, np.arange(1920))  # 128 still pending


def test_the_robot_mic_is_downmixed_and_resampled():
    # audio_capturer_node: 48 kHz stereo int16, 1024 frames per message.
    route = MicToHuri(sample_rate=16000, frame_duration=0.020)
    stereo = np.full(2048, 8192, dtype=np.int16)
    frames = []
    for _ in range(3):
        frames += route.convert(stamped(stereo.tolist(), rate=48000, channels=2))
    assert len(frames) == 3  # 3072 frames at 48 kHz = 1024 at 16 kHz -> 3 x 320
    assert all(len(f.data) == 640 for f in frames)
    pcm = np.frombuffer(frames[0].data, "<i2")
    assert np.all(np.abs(pcm - 8192) <= 1)


def test_speech_is_published_in_chunks_stamped_along_the_audio():
    now = [1000.0]
    route = HuriToSpeaker(chunk=1024, clock=lambda: now[0])
    data = np.linspace(-1, 1, 2500, dtype=np.float32)
    msgs = route.convert(Audio(data=data, sample_rate=22050))
    infos = [m["audio"]["info"] for m in msgs]
    assert [i["chunk"] for i in infos] == [1024, 1024, 452]
    assert all(
        i["format"] == PA_INT16 and i["channels"] == 1 and i["rate"] == 22050
        for i in infos
    )
    pcm = np.concatenate([m["audio"]["audio_data"]["int16_data"] for m in msgs])
    np.testing.assert_array_equal(pcm, to_int16(data))
    assert all(len(m["audio"]["audio_data"]["float32_data"]) == 0 for m in msgs)

    def stamp(msg):
        s = msg["header"]["stamp"]
        return s["sec"] + s["nanosec"] / 1e9

    assert [stamp(m) for m in msgs] == pytest.approx(
        [1000.0, 1000 + 1024 / 22050, 1000 + 2048 / 22050], abs=1e-6
    )
    # The next chunk of the same utterance follows, though published at once...
    (follow,) = route.convert(Audio(data=np.zeros(10, np.float32), sample_rate=22050))
    assert stamp(follow) == pytest.approx(1000 + 2500 / 22050, abs=1e-6)
    # ...and speech after a pause starts now.
    now[0] = 1010.0
    (later,) = route.convert(Audio(data=np.zeros(10, np.float32), sample_rate=22050))
    assert stamp(later) == pytest.approx(1010.0)


def test_an_empty_end_of_speech_publishes_nothing():
    route = HuriToSpeaker()
    assert route.convert(Audio(np.zeros(0, np.float32), 22050, end=True)) == []
