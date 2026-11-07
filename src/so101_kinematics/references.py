"""Reference solvers for the same task as `ik.solve_ik`, used only to compare against.

- `solve_ik_slsqp`: scipy SLSQP on the weighted task cost with the joint limits as bounds (the "classic"
  constrained answer; what arm_control's ik_scipy does, but on position + pitch + roll).
- `PinkSolver`: Pink (differential IK on Pinocchio, a QP with configuration limits at every step),
  driven in a loop until it converges. It takes a full 6-D target pose.
"""

import numpy as np
import pinocchio as pin
from scipy.optimize import minimize
from so101_core.model import ARM_JOINTS

from so101_kinematics.ik import PITCH_ROW, ROLL_ROW, TOL_ANG, IKResult, pitch_of, roll_of, wrap
from so101_kinematics.models import HI, LO, TOOL_FRAME, N, PinArm


def _finish(arm, q, target_xyz, pitch, roll, it, tol_pos, tol_ang) -> IKResult:
    pos_err = float(np.linalg.norm(target_xyz - arm.fk(q)[:3, 3]))
    pe, re = wrap(pitch - pitch_of(q)), wrap(roll - roll_of(q))
    ok = pos_err < tol_pos and abs(pe) < tol_ang and abs(re) < tol_ang
    return IKResult(q, pos_err, pe, re, it, ok, bool(np.any((q <= LO + 1e-9) | (q >= HI - 1e-9))))


def solve_ik_slsqp(
    arm: PinArm,
    target_xyz,
    pitch: float,
    roll: float,
    q0,
    *,
    w_pos: float = 1.0,
    w_ori: float = 0.05,
    tol_pos: float = 0.002,
    tol_ang: float = TOL_ANG,
    max_iter: int = 100,
) -> IKResult:
    target_xyz = np.asarray(target_xyz, dtype=float)

    def cost_and_grad(q):
        pos, Jp = arm.pos_jac(q)
        ang = [w_ori * (pitch - pitch_of(q)), w_ori * (roll - roll_of(q))]
        e = np.concatenate([w_pos * (target_xyz - pos), ang])
        J = np.vstack([w_pos * Jp, w_ori * PITCH_ROW, w_ori * ROLL_ROW])
        return 0.5 * e @ e, -J.T @ e

    res = minimize(
        cost_and_grad,
        np.clip(np.asarray(q0, dtype=float), LO, HI),
        jac=True,
        method="SLSQP",
        bounds=list(zip(LO, HI, strict=True)),
        options={"maxiter": max_iter, "ftol": 1e-14},
    )
    return _finish(arm, np.clip(res.x, LO, HI), target_xyz, pitch, roll, res.nit, tol_pos, tol_ang)


class PinkSolver:
    """Pink on the 5 arm joints (the gripper joint is locked out of the model)."""

    def __init__(self, arm: PinArm, w_pos: float = 1.0, w_ori: float = 0.05, damping: float = 1e-4):
        import pink
        from pink.tasks import FrameTask

        self.arm, self.pink, self.damping = arm, pink, damping
        arm_ids = {arm.model.getJointId(n) for n in arm.model.names if n in ARM_JOINTS}
        lock = [j for j in range(1, arm.model.njoints) if j not in arm_ids]
        self.model = pin.buildReducedModel(arm.model, lock, pin.neutral(arm.model))
        self.data = self.model.createData()
        assert self.model.nq == N, self.model.nq
        self.task = FrameTask(TOOL_FRAME, position_cost=w_pos, orientation_cost=w_ori)

    def solve(
        self,
        target_pose: np.ndarray,
        pitch: float,
        roll: float,
        q0,
        *,
        tol_pos: float = 0.002,
        tol_ang: float = TOL_ANG,
        max_iter: int = 100,
        dt: float = 1.0,
    ) -> IKResult:
        target_xyz = target_pose[:3, 3]
        self.task.set_target(pin.SE3(target_pose))
        conf = self.pink.Configuration(self.model, self.data, np.clip(np.asarray(q0, dtype=float), LO, HI))
        for it in range(max_iter + 1):
            q = np.array(conf.q)
            res = _finish(self.arm, q, target_xyz, pitch, roll, it, tol_pos, tol_ang)
            if res.converged or it == max_iter:
                return res
            v = self.pink.solve_ik(
                conf, [self.task], dt, solver="quadprog", damping=self.damping, safety_break=False
            )
            conf.integrate_inplace(v, dt)
