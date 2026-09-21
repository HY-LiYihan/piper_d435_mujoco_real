from __future__ import annotations

from pathlib import Path
import time
import numpy as np
from ..api.types import JointState, Pose, RobotState
from ..errors import BackendUnavailableError, IKError, NotConnectedError
from ..kinematics.ik import NumericalIK


DEFAULT_MODEL = Path(__file__).parents[3] / "vendor/piper_isaac_sim/piper_description/mujoco_model/piper_description.xml"


class MujocoBackend:
    def __init__(self, model_path: str | Path = DEFAULT_MODEL, realtime: bool = False, **_: object):
        self.model_path = Path(model_path)
        self.realtime = realtime
        self.model = self.data = self.ik = None
        self._connected = False
        self._last = time.monotonic()

    def connect(self) -> None:
        try:
            import mujoco
        except ImportError as exc:
            raise BackendUnavailableError("Install piper-control[mujoco] to use MuJoCo") from exc
        if not self.model_path.exists():
            raise BackendUnavailableError(f"MuJoCo model not found: {self.model_path}")
        self.model = mujoco.MjModel.from_xml_path(str(self.model_path))
        self.data = mujoco.MjData(self.model)
        joint_ids = np.array([self.model.joint(f"joint{i+1}").id for i in range(6)])
        lower = self.model.jnt_range[joint_ids, 0].copy()
        upper = self.model.jnt_range[joint_ids, 1].copy()
        body_id = self.model.body("link6").id
        self.ik = NumericalIK(self.model, self.data, joint_ids, body_id, lower, upper)
        mujoco.mj_forward(self.model, self.data)
        self._connected = True

    def _require(self):
        if not self._connected or self.model is None or self.data is None:
            raise NotConnectedError("MuJoCo backend is not connected")

    def disconnect(self) -> None:
        self._connected = False
        self.model = self.data = self.ik = None

    def _step(self, steps: int = 1) -> None:
        import mujoco
        for _ in range(steps):
            mujoco.mj_step(self.model, self.data)
        if self.realtime:
            elapsed = time.monotonic() - self._last
            delay = max(0.0, self.model.opt.timestep * steps - elapsed)
            if delay:
                time.sleep(delay)
        self._last = time.monotonic()

    def state(self) -> RobotState:
        self._require()
        import mujoco
        mujoco.mj_forward(self.model, self.data)
        body = self.model.body("link6").id
        quat = self.data.xquat[body]
        pose = Pose(tuple(self.data.xpos[body]), tuple(quat))
        return RobotState(True, False, JointState(self.data.qpos[:6], self.data.qvel[:6], float(self.data.qpos[6])), pose)

    def move_joints(self, joints) -> None:
        self._require()
        q = np.asarray(joints, dtype=float)
        if q.shape != (6,):
            raise ValueError("move_joints requires six joint values in radians")
        lower, upper = self.model.jnt_range[:6, 0], self.model.jnt_range[:6, 1]
        self.data.ctrl[:6] = np.clip(q, lower, upper)
        self._step(10)

    def move_p(self, pose: Pose) -> None:
        self._require()
        result = self.ik.solve(pose, seed=self.data.qpos[:6])
        if not result.success:
            raise IKError(f"MuJoCo IK failed: {result.message}; position={result.position_error:.6g}")
        self.move_joints(result.joints)

    def gripper(self, width: float, effort: float | None = None) -> None:
        self._require()
        if not 0.0 <= width <= 0.07:
            raise ValueError("gripper width must be between 0 and 0.07 metres")
        self.data.ctrl[6] = width / 2.0
        self.data.ctrl[7] = -width / 2.0
        self._step(10)

    def stop(self) -> None:
        self._require()
        self.data.ctrl[:] = self.data.qpos[: self.model.nu]
