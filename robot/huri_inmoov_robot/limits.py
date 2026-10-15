"""The robot's own servo limits: enforced on the robot whatever the server sends.

Deliberately a separate copy of the ranges, not read from the server's joints
config: a wrong or hostile server configuration must not widen what the robot
does. Override them per robot in the `robot.limits` section of the config.
"""

from dataclasses import dataclass, replace
from typing import Any, Dict, Mapping

from huri_events_inmoov import SERVO_NAMES


@dataclass(frozen=True)
class ServoLimit:
    min: float  # degrees, hard limit
    max: float
    rest: float  # where the watchdog and shutdown bring the servo back
    max_speed: float  # deg/s

    def __post_init__(self) -> None:
        if not self.min <= self.rest <= self.max:
            raise ValueError(f"needs min <= rest <= max, got {self}")
        if self.max_speed <= 0:
            raise ValueError(f"max_speed must be positive, got {self.max_speed}")


# Starting points from MyRobotLab's InMoov2 defaults (see the README):
# calibrate them on your robot. max_speed is conservative on purpose.
_HEAD = ServoLimit(20, 160, 90, 60)
_FINGER = ServoLimit(2, 170, 2, 120)
_WRIST = ServoLimit(20, 160, 90, 60)
DEFAULT_LIMITS: Dict[str, ServoLimit] = {
    "head.neck": _HEAD,
    "head.rothead": _HEAD,
    "head.jaw": ServoLimit(10, 25, 10, 60),
    "torso.midStom": ServoLimit(60, 120, 90, 30),
    **{
        f"{side}Arm.{servo}": limit
        for side in ("left", "right")
        for servo, limit in (
            ("omoplate", ServoLimit(10, 80, 10, 45)),
            ("shoulder", ServoLimit(0, 180, 30, 60)),
            ("rotate", ServoLimit(40, 180, 90, 60)),
            ("bicep", ServoLimit(5, 90, 5, 60)),
        )
    },
    **{
        f"{side}Hand.{finger}": _FINGER
        for side in ("left", "right")
        for finger in ("thumb", "index", "majeure", "ringFinger", "pinky")
    },
    "leftHand.wrist": _WRIST,
    "rightHand.wrist": _WRIST,
}
assert set(DEFAULT_LIMITS) == set(SERVO_NAMES)


def load_limits(overrides: Mapping[str, Mapping[str, Any]]) -> Dict[str, ServoLimit]:
    """DEFAULT_LIMITS with per-servo overrides ({name: {min, max, rest,
    max_speed}}, any subset of the keys)."""

    limits = dict(DEFAULT_LIMITS)
    for name, fields in (overrides or {}).items():
        if name not in limits:
            raise ValueError(f"robot.limits: unknown servo {name!r}")
        unknown = set(fields) - {"min", "max", "rest", "max_speed"}
        if unknown:
            raise ValueError(f"robot.limits.{name}: unknown keys {sorted(unknown)}")
        limits[name] = replace(limits[name], **{k: float(v) for k, v in fields.items()})
    return limits
