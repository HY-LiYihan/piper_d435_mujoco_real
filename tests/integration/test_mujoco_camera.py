import pytest

pytest.importorskip("mujoco")

from piper_control.backends.mujoco import MujocoBackend
from piper_control.sensors.mujoco_rgbd import MujocoRGBDCamera


def test_mujoco_rgbd_contract():
    backend = MujocoBackend()
    backend.connect()
    assert backend.model.ncam == 1
    assert backend.model.camera("wrist_camera").id == 0
    stand_mesh = backend.model.mesh("wrist_camera_stand").id
    start = backend.model.mesh_vertadr[stand_mesh]
    count = backend.model.mesh_vertnum[stand_mesh]
    vertices = backend.model.mesh_vert[start:start + count]
    assert max(vertices.max(axis=0) - vertices.min(axis=0)) < 0.15
    camera = MujocoRGBDCamera(backend.model, backend.data)
    camera.connect()
    try:
        frame = camera.read()
        assert frame.color.shape == (720, 1280, 3)
        assert frame.depth.shape == (720, 1280)
    finally:
        camera.disconnect()
        backend.disconnect()
