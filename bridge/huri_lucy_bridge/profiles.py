"""Embodiment profiles: the only robot-specific part of the bridge, as data.

HuRI's output is robot-agnostic (speech, SMPL-X human motion); a profile says
what it does on one robot model, by URDF joint name, in radians:

    version: 1
    robot_packages: [inmoov_urdf, inmoov_i2_urdf]   # Lucy robot packages
    gestures:            # optional: huri.motion (SMPL-X) -> joints
      max_speed: 1.5     # rad/s, the default of the joints below
      joints:
        i01.head.rothead_link_joint:
          sources: [neck.y, head.y]   # SMPL-X joint.axis, summed
          scale: 1.0                  # rad of joint per rad of sources
          ref: 0.0                    # rad of sources mapping to the rest
          max_speed: 1.0
    speech:              # optional: a joint that moves while the robot speaks
      joint: i01.head.jaw_link_joint
      open: 0.3          # rad from rest at full voice

Each section is a capability (`gestures`, `speech_motion`) a route can need.
Rests and limits are not in the profile: they are the robot's, from Lucy. A
section resolves against the robot Lucy describes: its joints the robot lacks
are dropped (with a warning), so one profile serves the variants of a model,
and a section with no joint left is no capability.

The profiles of the robots Lucy ships are packaged (huri_lucy_bridge/profiles/)
and picked by robot package; `lucy.profile` in the bridge's YAML names another
file, or `none`. In time they belong in the robot packages themselves.
"""

import logging
from dataclasses import dataclass, field
from functools import lru_cache
from importlib import resources
from pathlib import Path
from typing import Any, Dict, FrozenSet, List, Mapping, Optional, Tuple

import yaml

from huri_lucy_bridge.gestures import AXES, SMPLX_JOINTS, GestureJoint
from huri_lucy_bridge.robot import Joint, Robot

logger = logging.getLogger("huri_lucy_bridge.profiles")

GESTURES = "gestures"
SPEECH_MOTION = "speech_motion"
NO_PROFILE = "none"


class ProfileError(ValueError):
    pass


@dataclass(frozen=True)
class Speech:
    joint: str
    open: float  # rad from rest at full voice


@dataclass(frozen=True)
class Profile:
    robot_packages: Tuple[str, ...]
    gestures: Tuple[GestureJoint, ...] = ()
    speech: Optional[Speech] = None
    source: str = ""


@dataclass
class Capabilities:
    """A profile resolved against a robot: what this robot can do."""

    gestures: List[Tuple[GestureJoint, Joint]] = field(default_factory=list)
    speech: Optional[Tuple[Speech, Joint]] = None

    @property
    def names(self) -> FrozenSet[str]:
        return frozenset(
            name
            for name, present in (
                (GESTURES, self.gestures),
                (SPEECH_MOTION, self.speech),
            )
            if present
        )


def _keys(raw: Any, allowed: set, where: str) -> Mapping[str, Any]:
    if not isinstance(raw, dict):
        raise ProfileError(f"{where}: must be a mapping")
    unknown = sorted(set(raw) - allowed)
    if unknown:
        raise ProfileError(f"{where}: unknown keys {unknown}")
    return raw


def _number(raw: Mapping[str, Any], key: str, where: str, default=None) -> float:
    value = raw.get(key, default)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ProfileError(f"{where}: '{key}' must be a number, got {value!r}")
    return float(value)


def _source(text: Any, where: str) -> Tuple[int, int]:
    joint, _, axis = str(text).rpartition(".")
    if joint not in SMPLX_JOINTS or axis not in AXES:
        raise ProfileError(
            f"{where}: source {text!r} is not <SMPL-X joint>.<x|y|z> "
            f"(joints: {', '.join(SMPLX_JOINTS[:25])}, ...)"
        )
    return SMPLX_JOINTS.index(joint), AXES.index(axis)


def parse_profile(raw: Any, source: str = "") -> Profile:
    where = source or "profile"
    raw = _keys(raw, {"version", "robot_packages", "gestures", "speech"}, where)
    if raw.get("version") != 1:
        raise ProfileError(f"{where}: version must be 1")
    packages = raw.get("robot_packages")
    if (
        not isinstance(packages, list)
        or not packages
        or not all(isinstance(p, str) and p for p in packages)
    ):
        raise ProfileError(f"{where}: robot_packages must list robot packages")

    gestures: List[GestureJoint] = []
    if raw.get("gestures") is not None:
        section = _keys(raw["gestures"], {"max_speed", "joints"}, f"{where}: gestures")
        default_speed = _number(section, "max_speed", f"{where}: gestures", 1.5)
        joints = section.get("joints") or {}
        if not isinstance(joints, dict) or not joints:
            raise ProfileError(f"{where}: gestures.joints must map joints")
        for name, spec in joints.items():
            at = f"{where}: gestures.joints.{name}"
            spec = _keys(spec, {"sources", "scale", "ref", "max_speed"}, at)
            sources = spec.get("sources")
            if not isinstance(sources, list) or not sources:
                raise ProfileError(f"{at}: sources must list SMPL-X joint.axis")
            speed = _number(spec, "max_speed", at, default_speed)
            if speed <= 0:
                raise ProfileError(f"{at}: max_speed must be positive")
            gestures.append(
                GestureJoint(
                    joint=str(name),
                    sources=tuple(_source(s, at) for s in sources),
                    scale=_number(spec, "scale", at),
                    ref=_number(spec, "ref", at, 0.0),
                    max_speed=speed,
                )
            )

    speech = None
    if raw.get("speech") is not None:
        at = f"{where}: speech"
        section = _keys(raw["speech"], {"joint", "open"}, at)
        if not isinstance(section.get("joint"), str) or not section["joint"]:
            raise ProfileError(f"{at}: joint must name a URDF joint")
        speech = Speech(section["joint"], _number(section, "open", at))

    return Profile(tuple(packages), tuple(gestures), speech, source)


def load_profile(path: str) -> Profile:
    try:
        raw = yaml.safe_load(Path(path).read_text())
    except (OSError, yaml.YAMLError) as e:
        raise ProfileError(f"{path}: {e}") from e
    return parse_profile(raw, str(path))


@lru_cache(maxsize=None)
def packaged_profiles() -> Dict[str, Profile]:
    """The packaged profiles, by robot package."""

    by_package: Dict[str, Profile] = {}
    for entry in sorted(
        resources.files("huri_lucy_bridge").joinpath("profiles").iterdir(),
        key=lambda e: e.name,
    ):
        if not entry.name.endswith(".yaml"):
            continue
        profile = parse_profile(yaml.safe_load(entry.read_text()), entry.name)
        for package in profile.robot_packages:
            if package in by_package:
                raise ProfileError(
                    f"{entry.name}: {package} already has "
                    f"{by_package[package].source}"
                )
            by_package[package] = profile
    return by_package


def select_profile(robot: Robot, override: Optional[str] = None) -> Optional[Profile]:
    """The profile of `robot`: `override` (a path, or `none`), else the
    packaged one of its robot package, else none."""

    if override == NO_PROFILE:
        return None
    if override:
        profile = load_profile(override)
        if robot.package not in profile.robot_packages:
            logger.warning(
                "%s is for %s, not %s: used as configured",
                override,
                ", ".join(profile.robot_packages),
                robot.package,
            )
        return profile
    packaged = packaged_profiles().get(robot.package)
    if packaged is None:
        logger.info("no embodiment profile for %s: voice only", robot.package)
    return packaged


def resolve(profile: Optional[Profile], robot: Optional[Robot]) -> Capabilities:
    """What `robot` can do with `profile`: its sections, minus the joints the
    robot lacks."""

    if profile is None or robot is None:
        return Capabilities()

    def joint(name: str, section: str) -> Optional[Joint]:
        found = robot.joints.get(name)
        if found is None:
            logger.warning(
                "%s: %s has no joint %s: left out of %s",
                profile.source,
                robot.package,
                name,
                section,
            )
        return found

    caps = Capabilities()
    for gesture in profile.gestures:
        found = joint(gesture.joint, GESTURES)
        if found is not None:
            caps.gestures.append((gesture, found))
    if profile.speech is not None:
        found = joint(profile.speech.joint, SPEECH_MOTION)
        if found is not None:
            caps.speech = (profile.speech, found)
    return caps
