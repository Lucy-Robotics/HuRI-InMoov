from typing import Optional

from huri_core.core.module import Module
from huri_events.motion import Motion

from huri_events_inmoov import SERVO_LAYOUT_VERSION, SERVO_NAMES, ServoAngles
from huri_inmoov.retarget import Retargeter, load_config


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

    :joints: path of a joints config (TOML, see huri_inmoov/joints.toml, the
        default); a robot's calibration lives there.
    """

    input_type = "huri.motion"
    output_type = "inmoov.servo_angles"
    input_event = Motion
    output_event = ServoAngles
    actuation = True

    def __init__(self, joints: Optional[str] = None):
        self.retarget = Retargeter(load_config(joints, SERVO_NAMES))

    async def process(self, data: Motion) -> Optional[ServoAngles]:
        if data.poses.shape[0] == 0:
            return None
        angles = self.retarget(data.poses, data.fps)
        return ServoAngles(
            angles=angles, fps=data.fps, pts=data.pts, layout=SERVO_LAYOUT_VERSION
        )
