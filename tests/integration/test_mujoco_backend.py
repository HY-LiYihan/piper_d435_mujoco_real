import numpy as np
import pytest

mujoco = pytest.importorskip("mujoco")

from piper_control.backends.mujoco import MujocoBackend


def test_mujoco_model_loads_and_moves():
    backend = MujocoBackend()
    backend.connect()
    try:
        assert backend.model.nq == 8
        target = [0, 0.5, -0.5, 0, 0, 0]
        backend.move_joints(target)
        assert np.max(np.abs(backend.state().joints.positions - target)) < 0.02
        assert np.isfinite(backend.state().joints.positions).all()
        backend.gripper(0.02)
        assert backend.state().joints.gripper == pytest.approx(0.02)
    finally:
        backend.disconnect()
