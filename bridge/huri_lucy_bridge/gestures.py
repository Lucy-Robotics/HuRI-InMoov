"""SMPL-X poses (huri.motion, from HuRI's mov) -> a robot's URDF joints.

Pure numpy. Each joint a profile retargets follows a sum of SMPL-X joint
components (the component of a joint's axis-angle vector on one axis, a
first-order Euler angle: exact for a rotation about a single axis):

    angle = rest + scale * (sum of the sources, radians - ref)
          -> clamped to the joint's limits
          -> moved at most max_speed / fps from the previous frame

`rest` and the limits are the robot's own, from Lucy's hardware config; the
profile only says how the human motion maps onto the joint.
"""

from dataclasses import dataclass
from typing import Sequence, Tuple

import numpy as np

from huri_lucy_bridge.robot import Joint

# SMPL-X joint order: 55 joints, 3 axis-angle values each in Motion.poses.
SMPLX_JOINTS: Tuple[str, ...] = (
    "pelvis",
    "left_hip",
    "right_hip",
    "spine1",
    "left_knee",
    "right_knee",
    "spine2",
    "left_ankle",
    "right_ankle",
    "spine3",
    "left_foot",
    "right_foot",
    "neck",
    "left_collar",
    "right_collar",
    "head",
    "left_shoulder",
    "right_shoulder",
    "left_elbow",
    "right_elbow",
    "left_wrist",
    "right_wrist",
    "jaw",
    "left_eye_smplhf",
    "right_eye_smplhf",
    *(
        f"{side}_{finger}{i}"
        for side in ("left", "right")
        for finger in ("index", "middle", "pinky", "ring", "thumb")
        for i in (1, 2, 3)
    ),
)
assert len(SMPLX_JOINTS) == 55
AXES = ("x", "y", "z")


@dataclass(frozen=True)
class GestureJoint:
    """How one URDF joint follows the SMPL-X pose (a profile's gestures)."""

    joint: str
    sources: Tuple[Tuple[int, int], ...]  # (SMPL-X joint index, axis index)
    scale: float
    ref: float  # radians of the sources mapping to the joint's rest
    max_speed: float  # rad/s


class Retargeter:
    """Stateful SMPL-X -> joint angles: remembers the last angles, so the speed
    limit holds across successive Motion chunks. Starts at rest."""

    def __init__(self, mapping: Sequence[Tuple[GestureJoint, Joint]]):
        self.joints = [joint.name for _, joint in mapping]
        n = len(mapping)
        self._weights = np.zeros((165, n), dtype=np.float64)
        for j, (gesture, _) in enumerate(mapping):
            for source, axis in gesture.sources:
                self._weights[source * 3 + axis, j] += 1.0
        self._scale = np.array([g.scale for g, _ in mapping])
        self._ref = np.array([g.ref for g, _ in mapping])
        self._speed = np.array([g.max_speed for g, _ in mapping])
        self.rest = np.array([joint.rest for _, joint in mapping])
        self._min = np.array([joint.limits[0] for _, joint in mapping])
        self._max = np.array([joint.limits[1] for _, joint in mapping])
        self.last = self.rest.copy()

    def targets(self, poses: np.ndarray) -> np.ndarray:
        """(t, 165) axis-angle radians -> (t, n_joints) clamped radians, no
        speed limit."""

        poses = np.asarray(poses, dtype=np.float64)
        if poses.ndim != 2 or poses.shape[1] != 165:
            raise ValueError(f"poses must be (t, 165), got {poses.shape}")
        angles = self.rest + self._scale * (poses @ self._weights - self._ref)
        return np.clip(angles, self._min, self._max)

    def __call__(self, poses: np.ndarray, fps: int) -> np.ndarray:
        """Targets, each frame moved at most max_speed / fps from the last."""

        if fps <= 0:
            raise ValueError(f"fps must be positive, got {fps}")
        targets = self.targets(poses)
        step = self._speed / fps
        out = np.empty_like(targets)
        last = self.last
        for i, target in enumerate(targets):
            # A non-finite target (NaN in the pose) holds that joint for this
            # frame, instead of poisoning `last` for the rest of the session.
            target = np.where(np.isfinite(target), target, last)
            last = last + np.clip(target - last, -step, step)
            out[i] = last
        self.last = last
        return out

    def reset(self) -> None:
        self.last = self.rest.copy()
