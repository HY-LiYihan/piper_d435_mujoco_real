from unittest.mock import patch
import time

import numpy as np
import pytest

from piper_control.backends.piper_model import ASSET_ROOT
from piper_control.backends.real import RealBackend
from piper_control.errors import BackendUnavailableError
from piper_control.kinematics.ik import PinocchioIK


def test_real_backend_import_is_lazy():
    backend = RealBackend()
    with patch.dict("sys.modules", {"piper_sdk": None}):
        try:
            backend.connect()
        except Exception as exc:
            assert "piper_sdk" in str(exc)


def test_real_backend_state_includes_feedback_pose():
    class JointFeedback:
        joint_1 = 1000
        joint_2 = 0
        joint_3 = 0
        joint_4 = 0
        joint_5 = 0
        joint_6 = 0

    class Message:
        time_stamp = time.time()
        joint_state = JointFeedback()

    backend = RealBackend()
    backend._sdk = type("Sdk", (), {"GetArmJointMsgs": lambda self: Message()})()
    backend._fk = PinocchioIK(ASSET_ROOT / "piper/urdf/piper_description.urdf")
    backend._connected = True
    state = backend.state()
    assert state.pose is not None
    np.testing.assert_allclose(state.joints.positions[0], np.deg2rad(1.0))
    np.testing.assert_allclose(state.pose.position, backend._fk.forward(state.joints.positions).position)


def test_real_backend_rejects_missing_feedback():
    backend = RealBackend()
    backend._sdk = type("Sdk", (), {"GetArmJointMsgs": lambda self: type("Message", (), {"time_stamp": 0})()})()
    backend._connected = True
    with pytest.raises(BackendUnavailableError, match="fresh"):
        backend.state()
