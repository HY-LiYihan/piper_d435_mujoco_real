from __future__ import annotations

from dataclasses import dataclass
import numpy as np
from ..api.types import Pose
from ..errors import IKError


@dataclass
class IKResult:
    joints: np.ndarray
    success: bool
    position_error: float
    orientation_error: float
    iterations: int
    message: str = ""


def _quat_to_matrix(q: tuple[float, float, float, float]) -> np.ndarray:
    w, x, y, z = np.asarray(q, dtype=float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class NumericalIK:
    """Damped least-squares IK driven by a MuJoCo model."""

    def __init__(self, model, data, joint_ids: np.ndarray, body_id: int,
                 lower: np.ndarray, upper: np.ndarray, damping: float = 1e-3):
        self.model, self.data = model, data
        self.joint_ids, self.body_id = np.asarray(joint_ids), body_id
        self.lower, self.upper, self.damping = lower, upper, damping

    def solve(self, target: Pose, seed: np.ndarray | None = None,
              enforce_limits: bool = True, max_iterations: int = 200,
              tolerance: float = 1e-3) -> IKResult:
        q = np.array(self.data.qpos[:6] if seed is None else seed, dtype=float).copy()
        if q.shape != (6,):
            raise ValueError("IK seed must contain six joints")
        target_pos = np.asarray(target.position, dtype=float)
        target_rot = _quat_to_matrix(target.quaternion)
        import mujoco
        pos_err = ori_err = float("inf")
        for iteration in range(1, max_iterations + 1):
            self.data.qpos[:6] = q
            mujoco.mj_forward(self.model, self.data)
            cur_pos = self.data.xpos[self.body_id].copy()
            cur_rot = self.data.xmat[self.body_id].reshape(3, 3)
            dp = target_pos - cur_pos
            # Small-angle orientation error in world coordinates.
            do = 0.5 * (np.cross(cur_rot[:, 0], target_rot[:, 0]) +
                        np.cross(cur_rot[:, 1], target_rot[:, 1]) +
                        np.cross(cur_rot[:, 2], target_rot[:, 2]))
            err = np.concatenate((dp, do))
            pos_err, ori_err = float(np.linalg.norm(dp)), float(np.linalg.norm(do))
            if pos_err <= tolerance and ori_err <= tolerance:
                return IKResult(q.copy(), True, pos_err, ori_err, iteration, "converged")
            jacp = np.zeros((3, self.model.nv))
            jacr = np.zeros((3, self.model.nv))
            mujoco.mj_jacBody(self.model, self.data, jacp, jacr, self.body_id)
            jac = np.vstack((jacp[:, :6], jacr[:, :6]))
            step = jac.T @ np.linalg.solve(jac @ jac.T + self.damping ** 2 * np.eye(6), err)
            q += step
            if enforce_limits:
                q = np.clip(q, self.lower, self.upper)
        return IKResult(q, False, pos_err, ori_err, max_iterations, "iteration limit reached")
