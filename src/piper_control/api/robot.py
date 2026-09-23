from __future__ import annotations

from typing import Any, Sequence
from .protocols import RobotBackend
from .types import Pose, RobotState
from ..backends.mujoco import MujocoBackend
from ..backends.real import RealBackend
from ..errors import BackendUnavailableError
from ..scene import SceneClient


class PiperRobot:
    def __init__(self, backend: RobotBackend):
        self._backend = backend

    @classmethod
    def connect(cls, backend: str = "mujoco", config: dict[str, Any] | None = None) -> "PiperRobot":
        config = dict(config or {})
        requested_scene = None
        if config.get("scene") is not None:
            if backend != "mujoco":
                raise ValueError("scene is only supported by the MuJoCo backend")
            from ..backends.scene_builder import validate_scene
            requested_scene = validate_scene(config["scene"])
            config["scene"] = requested_scene
        if backend == "mujoco":
            socket_path = config.pop("socket_path", None)
            scene = SceneClient(socket_path=socket_path)
            try:
                scene.connect()
            except BackendUnavailableError:
                # No shared scene is running: fall back to a private in-process
                # simulation so scripts and tests keep working standalone.
                scene.disconnect()
                impl = MujocoBackend(**config)
                impl.connect()
                return cls(impl)
            if requested_scene is not None:
                try:
                    current_scene = scene.scene_info()["scene_path"]
                    if current_scene != str(requested_scene):
                        raise ValueError(f"Running MuJoCo scene is {current_scene}, requested {requested_scene}; restart the GUI with --scene")
                except Exception:
                    scene.disconnect()
                    raise
            return cls(scene)
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

    def step(self, steps: int = 1) -> None:
        """Advance a standalone MuJoCo simulation explicitly."""
        method = getattr(self._backend, "step", None)
        if method is None:
            raise NotImplementedError("Explicit stepping is only available on a standalone MuJoCo backend")
        method(steps)

    def wait_until_idle(self, timeout: float = 10.0) -> None:
        """Wait for measured simulation motion to settle, not just command acceptance."""
        method = getattr(self._backend, "wait_until_idle", None)
        if method is None:
            raise NotImplementedError("Waiting for motion is not implemented by this backend")
        method(timeout)
