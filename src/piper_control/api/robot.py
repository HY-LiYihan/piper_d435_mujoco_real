from __future__ import annotations

from typing import Any, Sequence
from .protocols import RobotBackend
from .types import Pose, RobotState
from ..backends.mujoco import MujocoBackend
from ..backends.real import RealBackend


class PiperRobot:
    def __init__(self, backend: RobotBackend):
        self._backend = backend

    @classmethod
    def connect(cls, backend: str = "mujoco", config: dict[str, Any] | None = None) -> "PiperRobot":
        config = config or {}
        if backend == "mujoco":
            impl = MujocoBackend(**config)
        elif backend == "real":
            impl = RealBackend(**config)
        else:
            raise ValueError(f"unknown backend: {backend}")
        impl.connect()
        return cls(impl)

    def disconnect(self) -> None:
        self._backend.disconnect()

    def state(self) -> RobotState:
        return self._backend.state()

    def move_joints(self, joints: Sequence[float]) -> None:
        self._backend.move_joints(joints)

    def move_p(self, pose: Pose) -> None:
        self._backend.move_p(pose)

    def gripper(self, width: float, effort: float | None = None) -> None:
        self._backend.gripper(width, effort)

    def stop(self) -> None:
        self._backend.stop()
