import numpy as np
import pinocchio as pin
import pytest

from so101_kinematics.ik import pitch_of, roll_of, solve_ik, solve_ik_clamp, wrap
from so101_kinematics.models import HI, LO, N, PinArm


@pytest.fixture(scope="module")
def arm():
    return PinArm()


def geometric_pitch_roll(arm: PinArm, q):
    """Pitch / roll straight from the URDF geometry, independent of the joint-sum formulas."""
    full = arm._full(q)
    pin.forwardKinematics(arm.model, arm.data, full)
    pin.framesForwardKinematics(arm.model, arm.data, full)
    normal = arm.data.oMi[arm.model.getJointId("wrist_flex")].rotation[:, 2]  # all flex axes are parallel
    R = arm.data.oMf[arm.fid].rotation
    radial = np.cross(normal, [0.0, 0.0, 1.0])
    radial /= np.linalg.norm(radial)
    approach = R[:, 2]
    pitch = np.arctan2(approach[2], approach @ radial)
    n_tool = R.T @ normal  # arm-plane normal in the tool frame
    roll = np.arctan2(n_tool[1], n_tool[0]) - np.pi / 2
    return pitch, roll, abs(approach @ normal)


def test_pitch_roll_formulas_match_urdf_geometry(arm):
    rng = np.random.default_rng(1)
    for _ in range(100):
        q = rng.uniform(LO, HI)
        pitch, roll, out_of_plane = geometric_pitch_roll(arm, q)
        # URDF numbers are rounded, so the axes are parallel / in-plane only to about 1e-5 rad
        assert out_of_plane < 1e-4  # the approach axis lies in the arm's vertical plane
        assert abs(wrap(pitch - pitch_of(q))) < 1e-4
        assert abs(wrap(roll - roll_of(q))) < 1e-4


def test_straight_down_is_minus_90_deg_and_zero_pose_is_horizontal(arm):
    assert pitch_of(np.zeros(N)) == 0.0
    q = np.array([0.0, np.pi / 4, np.pi / 4, 0.0, 0.0])  # lift + elbow + wrist_flex = 90 deg
    assert np.isclose(pitch_of(q), -np.pi / 2)
    assert np.isclose(geometric_pitch_roll(arm, q)[0], -np.pi / 2, atol=1e-4)


@pytest.mark.parametrize("solver", [solve_ik, solve_ik_clamp])
def test_solutions_always_inside_the_limits(arm, solver):
    rng = np.random.default_rng(2)
    for _ in range(20):
        q_t = rng.uniform(LO, HI)
        r = solver(arm, arm.fk(q_t)[:3, 3], pitch_of(q_t), roll_of(q_t), np.zeros(N))
        assert np.all(r.q >= LO - 1e-12) and np.all(r.q <= HI + 1e-12)


def test_reachable_target_is_reached_from_a_nearby_start(arm):
    q_t = np.deg2rad([20, -30, 40, 10, 15])
    r = solve_ik(arm, arm.fk(q_t)[:3, 3], pitch_of(q_t), roll_of(q_t), q_t + 0.2)
    assert r.converged
    assert r.pos_err < 0.002 and abs(r.pitch_err) < np.radians(0.5) and abs(r.roll_err) < np.radians(0.5)


def test_unreachable_pose_gives_a_compromise_not_garbage(arm):
    far = arm.fk(np.zeros(N))[:3, 3] + np.array([0.5, 0.0, 0.0])  # beyond the arm's reach
    r = solve_ik(arm, far, -np.pi / 2, 0.0, np.zeros(N))
    assert not r.converged
    assert np.all(r.q >= LO - 1e-12) and np.all(r.q <= HI + 1e-12)
    assert np.isfinite(r.pos_err) and r.pos_err < 0.5  # moved toward the target, did not run away
