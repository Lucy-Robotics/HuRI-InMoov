import struct

import numpy as np
import pytest
from huri_core.core.client import decode, encode
from huri_core.core.plugins import load_events

from huri_events_inmoov import (
    SERVO_LAYOUT_VERSION,
    SERVO_LAYOUTS,
    SERVO_NAMES,
    ServoAngles,
)


def _angles(n_frames=3):
    rng = np.random.default_rng(0)
    return rng.uniform(0, 180, (n_frames, len(SERVO_NAMES))).astype(np.float32)


def test_wire_round_trip():
    event = ServoAngles(_angles(), fps=25, pts=1.25)
    back = ServoAngles.from_wire(event.to_wire())
    np.testing.assert_array_equal(back.angles, event.angles)
    assert (back.fps, back.pts, back.layout) == (25, 1.25, SERVO_LAYOUT_VERSION)
    assert back.names == SERVO_NAMES


def test_round_trip_through_a_binary_frame():
    event = ServoAngles(_angles(2), fps=30, pts=0.5)
    frame = encode("inmoov.servo_angles", event)
    assert isinstance(frame, bytes)
    topic, data = decode(frame)
    assert topic == "inmoov.servo_angles"
    np.testing.assert_array_equal(ServoAngles.from_wire(data).angles, event.angles)


def test_wire_layout_is_the_documented_one():
    angles = _angles(1)
    wire = ServoAngles(angles, fps=30, pts=2.0).to_wire()
    header = struct.pack(">dIIII", 2.0, 30, 1, len(SERVO_NAMES), 1)
    assert wire[:24] == header
    assert wire[24:] == angles.astype("<f4").tobytes()
    assert len(wire) == 24 + 4 * len(SERVO_NAMES)


def test_empty_chunk_round_trips():
    back = ServoAngles.from_wire(ServoAngles(np.zeros((0, len(SERVO_NAMES)))).to_wire())
    assert back.angles.shape == (0, len(SERVO_NAMES))


def test_frame_gives_named_angles():
    event = ServoAngles(_angles(2))
    frame = event.frame(1)
    assert list(frame) == list(SERVO_NAMES)
    assert frame["head.neck"] == pytest.approx(float(event.angles[1, 0]))


@pytest.mark.parametrize(
    "mutate, message",
    [
        (lambda w: w[:10], "truncated"),
        (lambda w: w[:-4], "expected"),
        (lambda w: w + b"\0\0\0\0", "expected"),
        (lambda w: w[:20] + struct.pack(">I", 99) + w[24:], "unknown servo layout"),
        (lambda w: w[:16] + struct.pack(">I", 3) + w[20:], "servos"),
    ],
)
def test_bad_frames_are_refused(mutate, message):
    wire = ServoAngles(_angles(2)).to_wire()
    with pytest.raises(ValueError, match=message):
        ServoAngles.from_wire(mutate(wire))


def test_bad_shapes_are_refused():
    with pytest.raises(ValueError, match="angles must be"):
        ServoAngles(np.zeros((2, 3)))
    with pytest.raises(ValueError, match="unknown servo layout"):
        ServoAngles(_angles(), layout=0)


def test_layouts_are_frozen():
    """A released layout never changes: a new servo set is a new version."""

    assert SERVO_LAYOUT_VERSION == 1
    assert len(SERVO_LAYOUTS[1]) == 24 and len(set(SERVO_LAYOUTS[1])) == 24
    assert SERVO_LAYOUTS[1][:4] == (
        "head.neck",
        "head.rothead",
        "head.jaw",
        "torso.midStom",
    )


def test_entry_point_registers_the_topic():
    assert load_events()["inmoov.servo_angles"] is ServoAngles
