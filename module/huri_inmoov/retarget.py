"""SMPL-X axis-angle poses -> InMoov servo angles (degrees).

Pure numpy, no HuRI: the joints config (joints.toml) says which SMPL-X joint
components drive each servo; `Retargeter` applies it frame by frame, clamps to
each servo's range and limits its speed between frames.
"""

from dataclasses import dataclass
from importlib import resources
from pathlib import Path
from typing import Any, List, Mapping, Optional, Sequence, Tuple, Union

import numpy as np
from huri_core.core.manifest import loads_toml

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
_AXES = {"x": 0, "y": 1, "z": 2}

DEFAULT_CONFIG = "joints.toml"


@dataclass(frozen=True)
class ServoMap:
    """How one servo follows the SMPL-X pose (see joints.toml)."""

    name: str
    sources: Tuple[Tuple[int, int], ...]  # (joint index, axis index)
    scale: float
    ref: float
    rest: float
    min: float
    max: float
    max_speed: float  # deg/s


class JointsConfigError(ValueError):
    pass


def _joint_index(joint: Any, where: str) -> int:
    if isinstance(joint, bool):
        raise JointsConfigError(f"{where}: joint must be a name or an index")
    if isinstance(joint, int):
        if not 0 <= joint < len(SMPLX_JOINTS):
            raise JointsConfigError(f"{where}: joint index {joint} not in 0-54")
        return joint
    if isinstance(joint, str) and joint in SMPLX_JOINTS:
        return SMPLX_JOINTS.index(joint)
    raise JointsConfigError(f"{where}: unknown SMPL-X joint {joint!r}")


def _number(raw: Mapping[str, Any], key: str, where: str, default=None) -> float:
    value = raw.get(key, default)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise JointsConfigError(f"{where}: '{key}' must be a number, got {value!r}")
    return float(value)


_SERVO_KEYS = {"sources", "scale", "ref", "rest", "min", "max", "max_speed"}


def parse_config(raw: Mapping[str, Any], servo_names: Sequence[str]) -> List[ServoMap]:
    """The servo maps of a decoded joints config, in `servo_names` order."""

    defaults = raw.get("defaults", {})
    default_speed = _number(defaults, "max_speed", "[defaults]", 90.0)
    servos = raw.get("servos", {})
    unknown = set(servos) - set(servo_names)
    if unknown:
        raise JointsConfigError(f"servos not in the layout: {sorted(unknown)}")
    missing = [n for n in servo_names if n not in servos]
    if missing:
        raise JointsConfigError(f"servos missing from the config: {missing}")

    maps = []
    for name in servo_names:
        entry = servos[name]
        where = f'[servos."{name}"]'
        extra = set(entry) - _SERVO_KEYS
        if extra:
            raise JointsConfigError(f"{where}: unknown keys {sorted(extra)}")
        sources = []
        for i, src in enumerate(entry.get("sources", [])):
            axis = src.get("axis")
            if axis not in _AXES:
                raise JointsConfigError(f"{where} source {i}: axis must be x, y or z")
            sources.append(
                (_joint_index(src.get("joint"), f"{where} source {i}"), _AXES[axis])
            )
        servo = ServoMap(
            name=name,
            sources=tuple(sources),
            scale=_number(entry, "scale", where, 1.0),
            ref=_number(entry, "ref", where, 0.0),
            rest=_number(entry, "rest", where),
            min=_number(entry, "min", where),
            max=_number(entry, "max", where),
            max_speed=_number(entry, "max_speed", where, default_speed),
        )
        if not servo.min <= servo.rest <= servo.max:
            raise JointsConfigError(f"{where}: needs min <= rest <= max")
        if servo.max_speed <= 0:
            raise JointsConfigError(f"{where}: max_speed must be positive")
        maps.append(servo)
    return maps


def load_config(
    path: Optional[Union[str, Path]], servo_names: Sequence[str]
) -> List[ServoMap]:
    """A joints config file, or the packaged default when `path` is None."""

    if path is None:
        text = resources.files(__package__).joinpath(DEFAULT_CONFIG).read_text()
    else:
        text = Path(path).read_text()
    return parse_config(loads_toml(text), servo_names)


class Retargeter:
    """Stateful SMPL-X -> servo angles: remembers the last angles sent, so the
    speed limit holds across successive Motion chunks. Starts at rest."""

    def __init__(self, servos: Sequence[ServoMap]):
        self.servos = list(servos)
        n = len(self.servos)
        self._weights = np.zeros((165, n), dtype=np.float64)
        for j, servo in enumerate(self.servos):
            for joint, axis in servo.sources:
                self._weights[joint * 3 + axis, j] += 1.0
        self._scale = np.array([s.scale for s in self.servos])
        self._ref = np.array([s.ref for s in self.servos])
        self.rest = np.array([s.rest for s in self.servos])
        self._min = np.array([s.min for s in self.servos])
        self._max = np.array([s.max for s in self.servos])
        self._speed = np.array([s.max_speed for s in self.servos])
        self.last = self.rest.copy()

    def targets(self, poses: np.ndarray) -> np.ndarray:
        """(t, 165) axis-angle radians -> (t, n_servos) clamped degrees, no
        speed limit."""

        poses = np.asarray(poses, dtype=np.float64)
        if poses.ndim != 2 or poses.shape[1] != 165:
            raise ValueError(f"poses must be (t, 165), got {poses.shape}")
        source_deg = np.degrees(poses) @ self._weights
        angles = self.rest + self._scale * (source_deg - self._ref)
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
            last = last + np.clip(target - last, -step, step)
            out[i] = last
        self.last = last
        return out

    def reset(self) -> None:
        self.last = self.rest.copy()
