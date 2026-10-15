"""The module alone: no server, no Ray (huri_core.testing)."""

from pathlib import Path

import numpy as np
import pytest
from huri_core.core.compat import (
    CompatibilityError,
    check_actuation,
    check_manifests,
    check_modules,
)
from huri_core.core.manifest import load_manifest
from huri_core.core.plugins import find_manifest, load_events, load_modules
from huri_core.testing import ModuleHarness
from huri_events.motion import Motion

from huri_events_inmoov import SERVO_NAMES, ServoAngles
from huri_inmoov.module import InMoov
from huri_inmoov.retarget import (
    SMPLX_JOINTS,
    JointsConfigError,
    load_config,
    parse_config,
)

PACKAGE = Path(__file__).resolve().parents[1] / "huri_inmoov"
SERVOS = load_config(None, SERVO_NAMES)
REST = np.array([s.rest for s in SERVOS])
MIN = np.array([s.min for s in SERVOS])
MAX = np.array([s.max for s in SERVOS])


def motion(poses, fps=30, pts=0.0):
    poses = np.asarray(poses, dtype=np.float32)
    t = poses.shape[0]
    return Motion(poses, np.zeros((t, 100), np.float32), np.zeros((t, 3)), fps, pts)


def pose(t=1, **joints):
    """(t, 165) poses with joint=(x, y, z) degrees set on every frame."""

    poses = np.zeros((t, 165))
    for name, xyz in joints.items():
        i = SMPLX_JOINTS.index(name)
        poses[:, 3 * i : 3 * i + 3] = np.radians(xyz)
    return poses


def idx(name):
    return SERVO_NAMES.index(name)


def run(*events, **args):
    out = ModuleHarness(InMoov, **args).run(*events)
    assert all(isinstance(e, ServoAngles) for e in out)
    return out


def test_zero_pose_gives_rest_where_ref_is_zero():
    (out,) = run(motion(np.zeros((5, 165))))
    zero_ref = [i for i, s in enumerate(SERVOS) if s.ref == 0]
    np.testing.assert_allclose(out.angles[:, zero_ref], np.tile(REST[zero_ref], (5, 1)))


def test_reference_pose_gives_rest_everywhere():
    """The pose each servo's `ref` describes (arms down) is the robot's rest."""

    poses = np.zeros((4, 165))
    for servo in SERVOS:
        for joint, axis in servo.sources[:1]:
            poses[:, joint * 3 + axis] = np.radians(servo.ref)
    (out,) = run(motion(poses))
    np.testing.assert_allclose(out.angles, np.tile(REST, (4, 1)), atol=1e-4)


def test_output_shape_fps_and_pts_pass_through():
    (out,) = run(motion(np.zeros((7, 165)), fps=25, pts=3.5))
    assert out.angles.shape == (7, len(SERVO_NAMES))
    assert (out.fps, out.pts) == (25, 3.5)
    assert out.names == SERVO_NAMES


def test_empty_motion_emits_nothing():
    assert run(motion(np.zeros((0, 165)))) == []


def test_limits_are_respected_for_any_pose():
    rng = np.random.default_rng(1)
    poses = rng.uniform(-np.pi, np.pi, (300, 165))
    (out,) = run(motion(poses, fps=1))  # 1 fps: the speed limit barely binds
    assert np.all(out.angles >= MIN - 1e-4)
    assert np.all(out.angles <= MAX + 1e-4)


def test_head_follows_the_neck():
    module = InMoov()
    targets = module.retarget.targets(pose(neck=(10, 20, 0)))
    assert targets[0, idx("head.neck")] == pytest.approx(80.0)  # scale -1
    assert targets[0, idx("head.rothead")] == pytest.approx(110.0)


def test_fingers_stay_at_rest():
    (out,) = run(motion(np.full((3, 165), 0.5)))
    for name in SERVO_NAMES:
        if "Hand." in name:
            assert np.all(out.angles[:, idx(name)] == SERVOS[idx(name)].rest), name


def test_speed_is_limited_per_frame():
    fps = 30
    (out,) = run(motion(pose(60, neck=(0, 60, 0)), fps=fps))
    rothead = out.angles[:, idx("head.rothead")]
    step = SERVOS[idx("head.rothead")].max_speed / fps
    deltas = np.diff(np.concatenate([[REST[idx("head.rothead")]], rothead]))
    assert np.all(np.abs(deltas) <= step + 1e-4)
    assert rothead[0] == pytest.approx(90 + step)
    assert rothead[-1] == pytest.approx(150.0)  # reached the target in the end


def test_speed_limit_holds_across_chunks():
    module = ModuleHarness(InMoov)
    first = module.run(motion(pose(1, neck=(0, 60, 0))))[0]
    second = module.run(motion(pose(1, neck=(0, -60, 0))))[0]
    a = first.angles[0, idx("head.rothead")]
    b = second.angles[0, idx("head.rothead")]
    assert abs(b - a) <= SERVOS[idx("head.rothead")].max_speed / 30 + 1e-4


def test_custom_joints_config(tmp_path):
    text = (
        (PACKAGE / "joints.toml")
        .read_text()
        .replace("[defaults]\nmax_speed = 90.0", "[defaults]\nmax_speed = 1e6")
    )
    path = tmp_path / "joints.toml"
    path.write_text(text)
    (out,) = run(motion(pose(1, neck=(0, 60, 0))), joints=str(path))
    assert out.angles[0, idx("head.rothead")] == pytest.approx(150.0)


def _neck(**fields):
    return lambda c: c["servos"]["head.neck"].update(**fields)


@pytest.mark.parametrize(
    "mutate, message",
    [
        (lambda c: c["servos"].pop("head.neck"), "missing"),
        (lambda c: c["servos"].update({"head.nose": {}}), "not in"),
        (_neck(rest=200.0), "min <= rest"),
        (_neck(sources=[{"joint": "tail", "axis": "x"}]), "unknown SMPL-X"),
        (_neck(sources=[{"joint": 99, "axis": "x"}]), "not in 0-54"),
        (_neck(sources=[{"joint": 12, "axis": "w"}]), "axis"),
        (_neck(speed=3), "unknown keys"),
        (_neck(max_speed=0), "positive"),
        (_neck(min="low"), "must be a number"),
    ],
)
def test_bad_configs_are_refused(mutate, message):
    from huri_core.core.manifest import loads_toml

    raw = loads_toml((PACKAGE / "joints.toml").read_text())
    mutate(raw)
    with pytest.raises(JointsConfigError, match=message):
        parse_config(raw, SERVO_NAMES)


def test_manifest_matches_the_module():
    manifest = load_manifest(PACKAGE / "huri_module.toml")
    assert manifest.name == "inmoov"
    assert (manifest.input.topic, manifest.output.topic) == (
        InMoov.input_type,
        InMoov.output_type,
    )
    for port, cls in (
        (manifest.input, InMoov.input_event),
        (manifest.output, InMoov.output_event),
    ):
        assert port.event == f"{cls.__module__}:{cls.__qualname__}"
    assert manifest.actuation and InMoov.actuation


def test_installed_module_passes_the_server_checks():
    """What the server verifies at startup: entry point, events, manifest."""

    modules = load_modules(["inmoov"])
    events = load_events()
    check_modules(modules, events)
    manifest = find_manifest("inmoov")
    assert manifest is not None
    check_manifests(modules, {"inmoov": manifest}, events)


def test_server_refuses_it_without_actuation_opt_in():
    modules = load_modules(["inmoov"])
    manifests = {"inmoov": find_manifest("inmoov")}
    with pytest.raises(CompatibilityError, match="HURI_ACTUATION_MODULES"):
        check_actuation(modules, manifests, opted_in=[])
    assert check_actuation(modules, manifests, opted_in=["inmoov"]) == ["inmoov"]
