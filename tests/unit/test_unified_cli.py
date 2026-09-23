from pathlib import Path

from typer.testing import CliRunner
import pytest

from piper_control import cli
from piper_control.cli import app
from piper_control.errors import BackendUnavailableError
from piper_control.scene import SceneClient, SceneServer


runner = CliRunner()


def test_root_launches_gui_for_selected_robot(monkeypatch):
    calls = []
    monkeypatch.setattr(cli, "_run_scene_host", lambda *args, **kwargs: calls.append((args, kwargs)))
    assert runner.invoke(app, ["--backend", "mujoco"]).exit_code == 0
    assert calls[-1] == ((0.0,), {"scene": None})
    assert runner.invoke(app, ["--backend", "mujoco", "--robot", "franka_fr3"]).exit_code == 0
    assert calls[-1] == ((0.0,), {"scene": None, "robot": "franka_fr3"})


def test_no_gui_hosts_scene_without_viewer(monkeypatch):
    calls = []
    monkeypatch.setattr("piper_control.mujoco_gui.run_host", lambda **kwargs: calls.append(kwargs))
    result = runner.invoke(app, ["--backend", "mujoco", "--robot", "franka_fr3", "--no-gui"])
    assert result.exit_code == 0, result.output
    assert calls == [{"scene": None, "robot": "franka_fr3", "gui": False}]


def test_real_backend_requires_explicit_piper_without_touching_hardware(monkeypatch):
    monkeypatch.setattr(cli, "_robot", lambda *args, **kwargs: 1 / 0)
    missing = runner.invoke(app, ["--backend", "real", "state"])
    assert missing.exit_code == 2
    assert "requires --robot piper" in missing.output
    unsupported = runner.invoke(app, ["--backend", "real", "--robot", "franka_fr3", "state"])
    assert unsupported.exit_code == 2
    assert "not implemented" in unsupported.output


def test_command_global_options_and_conflicts(monkeypatch):
    calls = []
    class FakeRobot:
        def state(self):
            return "fr3 state"

        def disconnect(self):
            pass

    monkeypatch.setattr(cli, "_robot", lambda backend, can_name, robot: calls.append((backend, robot)) or FakeRobot())
    selected = runner.invoke(app, ["--backend", "mujoco", "--robot", "franka_fr3", "state"])
    assert selected.exit_code == 0, selected.output
    assert calls == [("mujoco", "franka_fr3")]
    conflict = runner.invoke(app, ["--robot", "franka_fr3", "state", "--robot", "piper"])
    assert conflict.exit_code == 2
    assert "conflicting --robot" in conflict.output


def test_auto_selection_and_ambiguity(monkeypatch, tmp_path):
    class FakeBackend:
        scene_path = None

    fr3_socket = Path("/tmp") / f"fr3-select-{id(tmp_path)}.sock"
    piper_socket = Path("/tmp") / f"piper-select-{id(tmp_path)}.sock"
    monkeypatch.setenv("FR3_SCENE_SOCKET", str(fr3_socket))
    monkeypatch.setenv("PIPER_SCENE_SOCKET", str(piper_socket))
    fr3_server = SceneServer(FakeBackend(), robot="franka_fr3")
    fr3_server.start()
    calls = []
    class FakeRobot:
        def state(self):
            return "selected"

        def disconnect(self):
            pass

    monkeypatch.setattr(cli, "_robot", lambda backend, can_name, robot: calls.append(robot) or FakeRobot())
    try:
        result = runner.invoke(app, ["state"])
        assert result.exit_code == 0, result.output
        assert calls == ["franka_fr3"]
        piper_server = SceneServer(FakeBackend(), robot="piper")
        piper_server.start()
        try:
            ambiguous = runner.invoke(app, ["state"])
            assert ambiguous.exit_code == 2
            assert "Both Piper and FR3" in ambiguous.output
            explicit = runner.invoke(app, ["--robot", "franka_fr3", "state"])
            assert explicit.exit_code == 0, explicit.output
        finally:
            piper_server.stop()
    finally:
        fr3_server.stop()
    fallback = runner.invoke(app, ["state"])
    assert fallback.exit_code == 0, fallback.output
    assert calls[-1] == "piper"


def test_duplicate_host_cannot_unlink_running_scene(tmp_path):
    class FakeBackend:
        scene_path = None

    socket_path = Path("/tmp") / f"scene-duplicate-{id(tmp_path)}.sock"
    first = SceneServer(FakeBackend(), socket_path=socket_path)
    first.start()
    second = SceneServer(FakeBackend(), socket_path=socket_path)
    try:
        with pytest.raises(BackendUnavailableError, match="already running"):
            second.start()
        second.stop()
        client = SceneClient(socket_path=socket_path)
        client.connect()
        try:
            assert client.scene_info()["robot"] == "piper"
        finally:
            client.disconnect()
    finally:
        first.stop()
