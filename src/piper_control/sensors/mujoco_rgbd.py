from __future__ import annotations

import time
import numpy as np
from .frame import CameraIntrinsics, RGBDFrame
from ..errors import BackendUnavailableError


class MujocoRGBDCamera:
    def __init__(self, model, data, camera="wrist_camera", width=1280, height=720, frame_id="wrist_camera"):
        self.model, self.data, self.camera = model, data, camera
        self.width, self.height, self.frame_id = width, height, frame_id
        self._renderer = None

    def connect(self):
        try:
            import mujoco
            # The upstream XML uses MuJoCo's 640x480 default offscreen buffer.
            self.model.vis.global_.offwidth = max(self.model.vis.global_.offwidth, self.width)
            self.model.vis.global_.offheight = max(self.model.vis.global_.offheight, self.height)
            self._renderer = mujoco.Renderer(self.model, height=self.height, width=self.width)
        except ImportError as exc:
            raise BackendUnavailableError("Install piper-control[mujoco] for MuJoCo RGB-D") from exc

    def disconnect(self):
        if self._renderer is not None:
            self._renderer.close()
        self._renderer = None

    def read(self) -> RGBDFrame:
        if self._renderer is None:
            raise RuntimeError("camera is not connected")
        import mujoco
        self._renderer.update_scene(self.data, camera=self.camera)
        self._renderer.enable_depth_rendering()
        depth = self._renderer.render().copy()
        self._renderer.disable_depth_rendering()
        # Renderer depth is normalised; retain float32 and expose scale as metres per unit.
        depth = np.asarray(depth, dtype=np.float32)
        self._renderer.update_scene(self.data, camera=self.camera)
        color = np.asarray(self._renderer.render(), dtype=np.uint8)
        if self.camera == -1:
            fovy = np.deg2rad(45.0)
        else:
            cam_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, self.camera)
            if cam_id < 0:
                raise ValueError(f"MuJoCo camera not found: {self.camera}")
            fovy = np.deg2rad(self.model.cam_fovy[cam_id])
        fy = self.height / (2 * np.tan(fovy / 2))
        fx = fy
        intr = CameraIntrinsics(self.width, self.height, fx, fy, self.width / 2, self.height / 2)
        return RGBDFrame(color, depth, time.time(), self.frame_id, intr, 1.0)
