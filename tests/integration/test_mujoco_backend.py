import numpy as np
import pytest

mujoco = pytest.importorskip("mujoco")

from piper_control.backends.mujoco import MujocoBackend


def test_mujoco_model_loads_and_moves():
    backend = MujocoBackend()
    backend.connect()
    try:
        assert backend.model.nq == 8
        backend.move_joints([0, 0.5, -0.5, 0, 0, 0])
        assert np.isfinite(backend.state().joints.positions).all()
        backend.gripper(0.02)
    finally:
        backend.disconnect()
