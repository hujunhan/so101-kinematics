import json
from pathlib import Path

import numpy as np
import pytest
from so101_core.safety import SafetyLayer

from so101_kinematics.drawing import CENTER_Y, CENTER_Z, RADIUS, X_PLANE, preflight, shape_ticks
from so101_kinematics.models import HI, LO, PinArm


@pytest.fixture(scope="module")
def arm():
    return PinArm()


def test_constants_match_the_s2_result():
    res = json.loads((Path(__file__).parent.parent / "results" / "s2_drawing.json").read_text())
    yc, zc, r = res["circle_center_radius_m"]
    assert (CENTER_Y, CENTER_Z) == pytest.approx((yc, zc), abs=1e-9)
    assert RADIUS == pytest.approx(r, abs=1e-9)
    assert res["plane"]["x_m"] == X_PLANE


@pytest.mark.parametrize("shape", ["circle", "square"])
@pytest.mark.parametrize("speed", [0.02, 0.05])
def test_planned_commands_pass_the_safety_layer_unchanged(arm, shape, speed):
    ticks = shape_ticks(arm, shape, speed, 30.0, grip=0.0)
    assert ticks.shape[1] == 6
    assert preflight(ticks, SafetyLayer()) == (0, 0)
    # and stay well inside the limits: the shapes were sized for a 10 deg margin
    q = ticks[:, :5]
    assert np.degrees(np.minimum(q - LO, HI - q).min()) > 10.0 - 1e-6


def test_the_tool_stays_on_the_plane_and_horizontal(arm):
    ticks = shape_ticks(arm, "square", 0.05, 30.0, grip=0.0)
    yaws = []
    for q in ticks[::10]:
        T = arm.fk(q[:5])
        assert T[0, 3] == pytest.approx(X_PLANE, abs=1e-3)
        approach = T[:3, 2]
        assert approach[2] == pytest.approx(0.0, abs=2e-3)  # horizontal (pitch 0)
        yaws.append(np.degrees(np.arctan2(approach[1], approach[0])))
    # 5 joints: the yaw is not controlled, it follows shoulder_pan, so off the centre line the tool is not
    # perpendicular to the plane (about 9 deg at the square's corners)
    assert 5.0 < max(np.abs(yaws)) < 20.0


def test_preflight_reports_a_path_the_layer_would_change(arm):
    ticks = shape_ticks(arm, "circle", 0.02, 30.0, grip=0.0)
    ticks = ticks.copy()
    ticks[len(ticks) // 2 :, 1] = np.radians(110)  # beyond the shoulder_lift limit
    clipped, capped = preflight(ticks, SafetyLayer())
    assert clipped > 0 and capped > 0


def test_shape_ticks_refuses_a_path_with_a_bad_point(arm, monkeypatch):
    import so101_kinematics.drawing as d

    real = d.track

    def one_bad(*a, **k):
        qs, m = real(*a, **k)
        return qs, {**m, "not_converged": 1}

    monkeypatch.setattr(d, "track", one_bad)
    with pytest.raises(ValueError, match="not safe to send"):
        shape_ticks(arm, "circle", 0.02, 30.0, grip=0.0)


@pytest.mark.parametrize("shape", ["circle", "square"])
def test_shapes_are_closed_and_drawn_at_the_requested_speed(arm, shape):
    from so101_kinematics.drawing import make_shapes

    path = make_shapes(CENTER_Y, CENTER_Z, RADIUS)[shape]
    assert np.allclose(path[0], path[-1])
    ticks = shape_ticks(arm, shape, 0.02, 30.0, grip=0.0)
    tips = np.array([arm.fk(q[:5])[1:3, 3] for q in ticks])
    assert np.allclose(tips[0], tips[-1], atol=1e-3)  # back where it started
    step = np.linalg.norm(np.diff(tips, axis=0), axis=1)
    assert np.median(step) == pytest.approx(0.02 / 30.0, rel=0.05)  # 2 cm/s at 30 Hz
