"""Presence in Lucy's client registry (lucy_config_pipeline's
client_registry_node), the way the control panel and lucy_cli register.

A client counts as connected while it pulses its id on
/lucy/client_heartbeat about once a second; the registry prunes it three
seconds after the beats stop. The bridge never takes control (/lucy/control):
it moves no joint, and taking control would turn the panel read-only.
"""

import asyncio
import secrets
import time

from huri_lucy_bridge.rosbridge import RosbridgeClient

HEARTBEAT_TOPIC = "/lucy/client_heartbeat"
STRING = "std_msgs/msg/String"


def new_client_id(prefix: str = "huri_bridge") -> str:
    """Unique and descriptive, as Lucy asks: huri_bridge_1678886400_a3f7."""

    return f"{prefix}_{int(time.time())}_{secrets.token_hex(2)}"


async def heartbeat(ros: RosbridgeClient, client_id: str, period: float = 1.0) -> None:
    """Pulses `client_id` until cancelled. `ros` must advertise HEARTBEAT_TOPIC."""

    while True:
        await ros.publish(HEARTBEAT_TOPIC, {"data": client_id})
        await asyncio.sleep(period)
