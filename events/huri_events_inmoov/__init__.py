"""Events of the InMoov integration: the ``inmoov.*`` topics.

Light on purpose (huri-core and numpy): the robot-side client installs this,
not the module that computes the angles.
"""

import struct
from dataclasses import dataclass
from typing import Dict, Tuple

import numpy as np
from huri_core.core.events import EventData

__all__ = [
    "SERVO_LAYOUT_VERSION",
    "SERVO_LAYOUTS",
    "SERVO_NAMES",
    "ServoAngles",
]

# The servos an angle row holds, in order. A layout never changes once
# released: a new servo set is a new layout version, so a robot client and a
# server of different releases either agree on the names or refuse the frame.
# Names follow MyRobotLab's InMoov2 services (<part>.<servo>).
SERVO_LAYOUTS: Dict[int, Tuple[str, ...]] = {
    1: (
        "head.neck",
        "head.rothead",
        "head.jaw",
        "torso.midStom",
        "leftArm.omoplate",
        "leftArm.shoulder",
        "leftArm.rotate",
        "leftArm.bicep",
        "rightArm.omoplate",
        "rightArm.shoulder",
        "rightArm.rotate",
        "rightArm.bicep",
        "leftHand.thumb",
        "leftHand.index",
        "leftHand.majeure",
        "leftHand.ringFinger",
        "leftHand.pinky",
        "leftHand.wrist",
        "rightHand.thumb",
        "rightHand.index",
        "rightHand.majeure",
        "rightHand.ringFinger",
        "rightHand.pinky",
        "rightHand.wrist",
    ),
}
SERVO_LAYOUT_VERSION = 1
SERVO_NAMES = SERVO_LAYOUTS[SERVO_LAYOUT_VERSION]

# pts (f64), fps (u32), frame count (u32), servo count (u32), layout (u32).
_HEADER = struct.Struct(">dIIII")
# Angles: little-endian float32, whatever the host's byte order.
_ANGLE = np.dtype("<f4")


@dataclass(eq=False)
class ServoAngles(EventData[bytes]):
    """InMoov servo angles in degrees, frame by frame, paired with the speech.

    ``angles[i, j]`` is the angle of servo ``names[j]`` at frame ``i``, the
    frame shown at ``pts + i / fps`` seconds (``pts`` matches the Motion and
    Audio it comes from).

    Wire (binary, 24-byte header, big-endian)::

        pts      f64  presentation timestamp of frame 0, seconds
        fps      u32  frames per second
        n_frames u32
        n_servos u32  must equal len(SERVO_LAYOUTS[layout])
        layout   u32  servo layout version (the names, in order)

    then ``n_frames * n_servos`` little-endian float32 angles, row-major
    (frame by frame). The servo names are not sent: they are fixed by the
    layout version, and a decoder refuses a layout it does not know.
    """

    angles: np.ndarray  # (n_frames, n_servos) degrees
    fps: int = 30
    pts: float = 0.0
    layout: int = SERVO_LAYOUT_VERSION

    def __post_init__(self) -> None:
        names = SERVO_LAYOUTS.get(self.layout)
        if names is None:
            raise ValueError(
                f"unknown servo layout {self.layout} "
                f"(known: {sorted(SERVO_LAYOUTS)}): upgrade huri-events-inmoov"
            )
        self.angles = np.asarray(self.angles, dtype=np.float32)
        if self.angles.ndim != 2 or self.angles.shape[1] != len(names):
            raise ValueError(
                f"angles must be (n_frames, {len(names)}) for layout "
                f"{self.layout}, got {self.angles.shape}"
            )
        if self.fps <= 0:
            raise ValueError(f"fps must be positive, got {self.fps}")

    @property
    def names(self) -> Tuple[str, ...]:
        return SERVO_LAYOUTS[self.layout]

    def frame(self, i: int) -> Dict[str, float]:
        """Frame ``i`` as {servo name: degrees}."""

        return {n: float(a) for n, a in zip(self.names, self.angles[i])}

    def to_wire(self) -> bytes:
        n_frames, n_servos = self.angles.shape
        header = _HEADER.pack(self.pts, self.fps, n_frames, n_servos, self.layout)
        return header + self.angles.astype(_ANGLE).tobytes()

    @classmethod
    def from_wire(cls, data: bytes) -> "ServoAngles":
        if len(data) < _HEADER.size:
            raise ValueError(f"truncated servo angles header: {len(data)} bytes")
        pts, fps, n_frames, n_servos, layout = _HEADER.unpack_from(data)
        names = SERVO_LAYOUTS.get(layout)
        if names is None:
            raise ValueError(
                f"unknown servo layout {layout} (known: {sorted(SERVO_LAYOUTS)})"
            )
        if n_servos != len(names):
            raise ValueError(
                f"layout {layout} has {len(names)} servos, the frame says {n_servos}"
            )
        expected = _HEADER.size + n_frames * n_servos * _ANGLE.itemsize
        if len(data) != expected:
            raise ValueError(
                f"servo angles frame is {len(data)} bytes, expected {expected} "
                f"for {n_frames} frames of {n_servos} servos"
            )
        body = np.frombuffer(data, dtype=_ANGLE, offset=_HEADER.size)
        angles = body.reshape(n_frames, n_servos).astype(np.float32)
        return cls(angles=angles, fps=fps, pts=pts, layout=layout)

    def summarize(self) -> str:
        cls = type(self).__name__
        return (
            f"{cls}(frames={self.angles.shape[0]}, servos={self.angles.shape[1]}, "
            f"fps={self.fps}, pts={self.pts:.3f}s, layout={self.layout})"
        )
