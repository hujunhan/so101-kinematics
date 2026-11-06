import numpy as np

from so101_kinematics.models import HI, LO, LegacyArm, N, PinArm


def test_legacy_fk_matches_pinocchio():
    legacy, ref = LegacyArm(), PinArm()
    rng = np.random.default_rng(1)
    for _ in range(50):
        q = rng.uniform(LO, HI)
        assert np.allclose(legacy.fk(q), ref.fk(q), atol=1e-9)


def test_pinocchio_jacobian_matches_finite_difference():
    ref = PinArm()
    q = np.random.default_rng(2).uniform(LO, HI)
    J = ref.jacobian(q)[:3]
    eps = 1e-6
    for i in range(N):
        dq = np.zeros(N)
        dq[i] = eps
        num = (ref.fk(q + dq)[:3, 3] - ref.fk(q - dq)[:3, 3]) / (2 * eps)
        assert np.allclose(J[:, i], num, atol=1e-6)


def test_reference_dls_reaches_reachable_target():
    ref = PinArm()
    target = ref.fk(np.deg2rad([20, -30, 40, 10, 0]))[:3, 3]
    q = ref.ik_dls_position(target, np.zeros(N))
    assert np.linalg.norm(ref.fk(q)[:3, 3] - target) < 0.002
