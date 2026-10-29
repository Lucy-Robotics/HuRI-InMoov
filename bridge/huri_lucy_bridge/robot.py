"""The robot, as Lucy describes it: never configured in the bridge.

Lucy's config services (lucy_config_pipeline) serve the active hardware config
of whatever robot package the stack runs (inmoov_urdf, inmoov_i2_urdf,
so_arm101_urdf...) on /config/get. It is the same YAML the control panel and
lucy_cli read, parsed the same way:

- one JointTrajectory controller per board (`boards.<id>.controller.name`),
  on /<name>/joint_trajectory, its joints the board's actuators in
  `virtual_pin` order (the order the controller expects);
- per actuator, its URDF joint, limits and default (rest) in servo radians,
  and the servo -> URDF mapping: joint = (servo - offset) * direction * scale.

The bridge works in URDF radians: the space of the robot model, which a
rewired or recalibrated servo does not change.
"""

import asyncio
import logging
from dataclasses import dataclass, field
from typing import Any, Dict, List, Mapping, Tuple

import yaml

from huri_lucy_bridge.rosbridge import RosbridgeClient, ServiceError

logger = logging.getLogger("huri_lucy_bridge.robot")

CONFIG_SERVICE = "/config/get"
GET_CONFIG = "lucy_msgs/srv/GetConfig"


class DiscoveryError(RuntimeError):
    """Lucy did not describe its robot (no config services, no active config,
    an unreadable YAML)."""


@dataclass(frozen=True)
class Joint:
    """An actuated URDF joint, in URDF radians."""

    name: str
    controller: str
    limits: Tuple[float, float]
    rest: float
    enabled: bool = True

    def clamp(self, value: float) -> float:
        return min(max(value, self.limits[0]), self.limits[1])


@dataclass(frozen=True)
class Controller:
    name: str
    topic: str
    joints: Tuple[str, ...]  # in the order the controller expects


@dataclass
class Robot:
    package: str  # the robot package, e.g. inmoov_urdf: the key of its profile
    name: str  # robot_name of the hardware YAML (InMoov i1 and i2 share one)
    controllers: Dict[str, Controller] = field(default_factory=dict)
    joints: Dict[str, Joint] = field(default_factory=dict)
    cameras: List[Dict[str, Any]] = field(default_factory=list)
    config_name: str = ""


def _number(raw: Mapping[str, Any], key: str, default: float) -> float:
    value = raw.get(key, default)
    try:
        number = float(value)
    except (TypeError, ValueError):
        return default
    return number if number == number else default  # NaN


def _to_urdf(servo: float, actuator: Mapping[str, Any]) -> float:
    direction = _number(actuator, "direction", 1.0) or 1.0
    scale = _number(actuator, "scale", 1.0) or 1.0
    return (servo - _number(actuator, "offset", 0.0)) * direction * scale


def parse_hardware_config(text: str, package: str = "", config_name: str = "") -> Robot:
    """The Robot of a Lucy hardware YAML (config/hardware/active.yaml)."""

    try:
        doc = yaml.safe_load(text)
    except yaml.YAMLError as e:
        raise DiscoveryError(f"the hardware config is not YAML: {e}") from e
    if not isinstance(doc, dict):
        raise DiscoveryError("the hardware config is not a YAML mapping")
    boards = doc.get("boards") or {}
    actuators = [a for a in doc.get("actuators") or [] if isinstance(a, dict)]
    robot = Robot(
        package=package or str(doc.get("robot_name", "")),
        name=str(doc.get("robot_name", "")),
        cameras=[c for c in doc.get("cameras") or [] if isinstance(c, dict)],
        config_name=config_name,
    )
    for board_id, board in boards.items() if isinstance(boards, dict) else ():
        controller = (
            (board or {}).get("controller") if isinstance(board, dict) else None
        )
        name = str((controller or {}).get("name", "")).strip()
        if not name:
            continue
        rows = sorted(
            (a for a in actuators if a.get("board") == board_id),
            key=lambda a: _number(a, "virtual_pin", 0.0),
        )
        joints: List[str] = []
        for row in rows:
            joint = str(row.get("urdf_joint") or "").strip()
            if not joint:
                continue
            joints.append(joint)
            low = _to_urdf(_number(row, "limit_min", 0.0), row)
            high = _to_urdf(_number(row, "limit_max", 0.0), row)
            robot.joints[joint] = Joint(
                name=joint,
                controller=name,
                limits=(min(low, high), max(low, high)),
                rest=_to_urdf(_number(row, "default", 0.0), row),
                enabled=bool(row.get("enabled", True)),
            )
        if joints:
            robot.controllers[name] = Controller(
                name, f"/{name}/joint_trajectory", tuple(joints)
            )
    return robot


async def discover(ros: RosbridgeClient, timeout: float = 5.0) -> Robot:
    """The robot Lucy runs, from its active hardware config."""

    try:
        values = await ros.call_service(
            CONFIG_SERVICE,
            GET_CONFIG,
            {"robot_package": "", "config_name": ""},
            timeout=timeout,
        )
    except ServiceError as e:  # its message names the service
        raise DiscoveryError(str(e)) from e
    except asyncio.TimeoutError as e:
        raise DiscoveryError(f"{CONFIG_SERVICE}: no response in {timeout} s") from e
    if not values.get("success", False):
        raise DiscoveryError(f"{CONFIG_SERVICE}: {values.get('message', '')}")
    robot = parse_hardware_config(
        values.get("config_yaml", ""),
        values.get("robot_package", ""),
        values.get("config_name", ""),
    )
    logger.info(
        "robot %s (%s, config %r): %d joints on %s",
        robot.package,
        robot.name,
        robot.config_name,
        len(robot.joints),
        ", ".join(robot.controllers) or "no controller",
    )
    return robot
