import pytest

pytest.importorskip("mujoco")

from piper_control.backends.mujoco import MujocoBackend
from piper_control.sensors.mujoco_rgbd import MujocoRGBDCamera


def test_mujoco_rgbd_contract():
    backend = MujocoBackend()
    backend.connect()
    camera = MujocoRGBDCamera(backend.model, backend.data)
    camera.connect()
    try:
        frame = camera.read()
        assert frame.color.shape == (720, 1280, 3)
        assert frame.depth.shape == (720, 1280)
    finally:
        camera.disconnect()
        backend.disconnect()
