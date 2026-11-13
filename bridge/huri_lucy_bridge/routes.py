"""Routes: how one ROS topic becomes one HuRI topic, or the reverse.

A route converts a message to zero or more messages of the other side (an
audio route regroups samples into frames of another size). Routes may keep
state between messages; a bridge builds new ones for each connection.
"""

from typing import Any, Dict, Mapping, Protocol, Sequence, Type

from huri_core.core.events import EventData


class RosToHuri(Protocol):
    """A ROS topic the bridge subscribes to, sent to HuRI."""

    ros_topic: str
    ros_type: str
    huri_topic: str

    def convert(self, msg: Mapping[str, Any]) -> Sequence[EventData]: ...


class HuriToRos(Protocol):
    """A HuRI topic the bridge subscribes to, published on ROS."""

    huri_topic: str
    event_cls: Type[EventData]
    ros_topic: str
    ros_type: str

    def convert(self, event: Any) -> Sequence[Dict[str, Any]]: ...
