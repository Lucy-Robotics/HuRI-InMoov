import logging
from typing import ClassVar, List, Optional

import numpy as np
from huri_core.core.module import Module
from huri_events.motion import Motion

from huri_events_inmoov import SERVO_LAYOUT_VERSION, SERVO_NAMES, ServoAngles
from huri_inmoov.retarget import (
    JOINTS_ENV,
    Retargeter,
    ServoMap,
    load_operator_config,
)

logger = logging.getLogger("huri_inmoov")


class InMoov(Module):
    """Retargets SMPL-X gestures to InMoov servo angles.

    input: huri.motion (Motion)
    output: inmoov.servo_angles (ServoAngles)

    Each Motion chunk becomes one ServoAngles with the same frame count, fps
    and pts. Angles are clamped to each servo's range and their speed limited
    between frames (and across chunks: the module remembers the last angles of
    the session, starting at rest).

    Moves hardware: the server loads it only if its operator lists `inmoov` in
    HURI_ACTUATION_MODULES as well as HURI_MODULES. The robot enforces its own
    limits too (huri-inmoov-robot): never rely on this module alone.

    The robot's calibration (a joints config, TOML: see
    huri_inmoov/joints.toml, the default) is the server operator's choice,
    never a client's: the path in $HURI_INMOOV_JOINTS, read once when the
    server imports the module (it fails to start if that file is unreadable
    or invalid). Takes no argument.
    """

    input_type = "huri.motion"
    output_type = "inmoov.servo_angles"
    input_event = Motion
    output_event = ServoAngles
    actuation = True

    # Resolved at import, on the server: shared by every session.
    servos: ClassVar[List[ServoMap]] = load_operator_config(SERVO_NAMES)

    def __init__(self, joints: Optional[str] = None):
        if joints is not None:
            raise ValueError(f"joints is set by the server operator ({JOINTS_ENV})")
        self.retarget = Retargeter(self.servos)

    async def process(self, data: Motion) -> Optional[ServoAngles]:
        if data.poses.shape[0] == 0:
            return None
        if not np.isfinite(data.poses).any():
            logger.warning("dropped a Motion chunk with no finite pose value")
            return None
        angles = self.retarget(data.poses, data.fps)
        return ServoAngles(
            angles=angles, fps=data.fps, pts=data.pts, layout=SERVO_LAYOUT_VERSION
        )
