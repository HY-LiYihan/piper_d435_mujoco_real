from unittest.mock import patch
from piper_control.backends.real import RealBackend


def test_real_backend_import_is_lazy():
    backend = RealBackend()
    with patch.dict("sys.modules", {"piper_sdk": None}):
        try:
            backend.connect()
        except Exception as exc:
            assert "piper_sdk" in str(exc)
