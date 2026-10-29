import numpy as np
import pytest

from huri_lucy_bridge.gestures import SMPLX_JOINTS, GestureJoint, Retargeter
from huri_lucy_bridge.robot import Joint

NECK_Y = SMPLX_JOINTS.index("neck") * 3 + 1
HEAD_Y = SMPLX_JOINTS.index("head") * 3 + 1


def retargeter(scale=1.0, ref=0.0, max_speed=100.0, rest=0.1, limits=(-1.0, 1.0)):
    gesture = GestureJoint(
        joint="yaw",
        sources=((SMPLX_JOINTS.index("neck"), 1), (SMPLX_JOINTS.index("head"), 1)),
        scale=scale,
        ref=ref,
        max_speed=max_speed,
    )
    return Retargeter([(gesture, Joint("yaw", "head_controller", limits, rest))])


def poses(neck_y=0.0, head_y=0.0, frames=1):
    out = np.zeros((frames, 165))
    out[:, NECK_Y] = neck_y
    out[:, HEAD_Y] = head_y
    return out


def test_the_zero_pose_is_the_robots_rest():
    assert retargeter().targets(poses())[0, 0] == pytest.approx(0.1)


def test_sources_sum_scale_and_ref():
    r = retargeter(scale=-2.0, ref=0.1)
    # rest + scale * (0.2 + 0.1 - ref)
    assert r.targets(poses(0.2, 0.1))[0, 0] == pytest.approx(0.1 - 2.0 * 0.2)


def test_the_robots_limits_clamp():
    assert retargeter(scale=10.0).targets(poses(1.0))[0, 0] == pytest.approx(1.0)
    assert retargeter(scale=-10.0).targets(poses(1.0))[0, 0] == pytest.approx(-1.0)


def test_speed_is_limited_across_chunks():
    r = retargeter(max_speed=0.3)  # 0.01 rad per frame at 30 fps
    first = r(poses(0.5, frames=3), fps=30)
    assert first[:, 0] == pytest.approx([0.11, 0.12, 0.13])
    second = r(poses(0.5, frames=1), fps=30)
    assert second[0, 0] == pytest.approx(0.14)
    r.reset()
    assert r.last == pytest.approx([0.1])


def test_a_nan_pose_holds_the_joint():
    r = retargeter()
    out = r(poses(np.nan, frames=2), fps=30)
    assert out[:, 0] == pytest.approx([0.1, 0.1])


def test_poses_and_fps_are_checked():
    with pytest.raises(ValueError, match="165"):
        retargeter().targets(np.zeros((1, 3)))
    with pytest.raises(ValueError, match="fps"):
        retargeter()(poses(), fps=0)
