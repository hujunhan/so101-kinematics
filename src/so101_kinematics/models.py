"""Two kinematics backends for the same SO-101 URDF (vendored in so101-core).

- `LegacyArm`: my 2023 arm_control (URDF parser, numeric and sympy FK, Jacobian / SLSQP IK).
- `PinArm`: Pinocchio, the reference.

Both work on the 5 arm joints (rad, so101_core.ARM_JOINTS order). The tool frame is the URDF's
`gripper_frame_link`, a fixed frame between the jaws.
"""

import contextlib
import io

import numpy as np
import pinocchio as pin
from loguru import logger
from so101_core.model import ARM_JOINTS, URDF_PATH, urdf_limits

TOOL_FRAME = "gripper_frame_link"
N = len(ARM_JOINTS)
LO, HI = (a[:N] for a in urdf_limits())


class LegacyArm:
    """arm_control on the SO-101 chain: 5 revolute joints + the fixed gripper_frame_joint."""

    def __init__(self):
        from arm_control.robot import Robot
        from arm_control.urdf import URDF

        logger.remove()  # arm_control logs every parsed element
        urdf = URDF()
        with contextlib.redirect_stderr(io.StringIO()):  # warns about the fixed joint's <axis>
            urdf.get_urdf_parameters(str(URDF_PATH))
        names = [j["name"] for j in urdf.joint_list]
        assert names[:N] == list(ARM_JOINTS), names  # chain order matches so101_core
        self.robot = Robot()
        self.robot.joint_list = urdf.joint_list
        self.n_chain = len(urdf.joint_list)  # 6: the fixed tool joint is part of the chain
        self.robot.jacobian_xyz(self.robot.forward_kine_sympy()[-1])

    def _pad(self, q: np.ndarray) -> np.ndarray:
        out = np.zeros(self.n_chain)
        out[:N] = q
        return out

    def fk(self, q: np.ndarray) -> np.ndarray:
        """4x4 tool pose, numeric FK."""
        return self.robot.forward_kine(list(self._pad(q)))[-1]

    def ik_jacobian(self, target_xyz, q0, method="inverse", alpha=0.5, tol=0.002, max_iter=500):
        q = self._pad(q0)
        self.robot.ik_jacobian(
            q, np.asarray(target_xyz), method=method, threshold=tol, max_iteration=max_iter, alpha=alpha
        )  # updates q in place
        return q[:N]

    def ik_scipy(self, target_xyz, q0, max_iter=100):
        with contextlib.redirect_stdout(io.StringIO()):  # it prints its progress
            hist = self.robot.ik_scipy(self._pad(q0), np.asarray(target_xyz), max_iteration=max_iter)
        return np.asarray(hist[-1])[:N]  # it returns its evaluation history; the last one is the result


class PinArm:
    def __init__(self):
        self.model = pin.buildModelFromUrdf(str(URDF_PATH))
        self.data = self.model.createData()
        self.fid = self.model.getFrameId(TOOL_FRAME)
        self.idx = np.array([self.model.joints[self.model.getJointId(j)].idx_q for j in ARM_JOINTS])

    def _full(self, q: np.ndarray) -> np.ndarray:
        full = pin.neutral(self.model)
        full[self.idx] = q
        return full

    def fk(self, q: np.ndarray) -> np.ndarray:
        pin.framesForwardKinematics(self.model, self.data, self._full(q))
        return self.data.oMf[self.fid].homogeneous.copy()

    def jacobian(self, q: np.ndarray) -> np.ndarray:
        """6x5 tool Jacobian in the world-aligned frame (rows: linear xyz, angular xyz)."""
        full = self._full(q)
        pin.computeJointJacobians(self.model, self.data, full)
        pin.framesForwardKinematics(self.model, self.data, full)
        J = pin.getFrameJacobian(self.model, self.data, self.fid, pin.LOCAL_WORLD_ALIGNED)
        return J[:, self.idx]

    def ik_dls_position(self, target_xyz, q0, damping=1e-3, tol=0.002, max_iter=200):
        """Textbook damped-least-squares IK on position only, no joint limits (reference)."""
        q = np.array(q0, dtype=float)
        for _ in range(max_iter):
            err = np.asarray(target_xyz) - self.fk(q)[:3, 3]
            if np.linalg.norm(err) < tol:
                break
            J = self.jacobian(q)[:3]
            q = q + J.T @ np.linalg.solve(J @ J.T + damping**2 * np.eye(3), err)
        return q
