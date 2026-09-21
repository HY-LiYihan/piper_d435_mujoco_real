from __future__ import annotations

import json
from typing import Annotated
import typer
from .api.robot import PiperRobot
from .api.types import Pose

app = typer.Typer(help="Unified Piper real-robot and MuJoCo control CLI")


def _robot(backend: str, can_name: str):
    return PiperRobot.connect(backend, {"can_name": can_name} if backend == "real" else {})


@app.command()
def doctor():
    """Report optional runtime dependencies and pinned asset paths."""
    checks = {}
    for name in ("numpy", "typer", "mujoco", "pyrealsense2", "can"):
        try:
            __import__(name)
            checks[name] = "available"
        except ImportError:
            checks[name] = "missing"
    typer.echo(json.dumps(checks, indent=2))


@app.command()
def state(backend: str = "mujoco", can_name: str = "can0"):
    robot = _robot(backend, can_name)
    try:
        typer.echo(robot.state())
    finally:
        robot.disconnect()


@app.command("move-joints")
def move_joints(joints: Annotated[list[float], typer.Argument(help="Six joint angles in radians")],
                backend: str = "mujoco", can_name: str = "can0"):
    robot = _robot(backend, can_name)
    try:
        robot.move_joints(joints)
    finally:
        robot.disconnect()


@app.command("move-p")
def move_p(x: float, y: float, z: float, qw: float = 1.0, qx: float = 0.0,
           qy: float = 0.0, qz: float = 0.0, backend: str = "mujoco", can_name: str = "can0"):
    robot = _robot(backend, can_name)
    try:
        robot.move_p(Pose((x, y, z), (qw, qx, qy, qz)))
    finally:
        robot.disconnect()


@app.command()
def gripper(width: float, effort: float | None = None, backend: str = "mujoco", can_name: str = "can0"):
    robot = _robot(backend, can_name)
    try:
        robot.gripper(width, effort)
    finally:
        robot.disconnect()


@app.command()
def stop(backend: str = "mujoco", can_name: str = "can0"):
    robot = _robot(backend, can_name)
    try:
        robot.stop()
    finally:
        robot.disconnect()


@app.command()
def run(backend: str = "mujoco", can_name: str = "can0", steps: int = 0):
    """Start a backend and optionally advance a MuJoCo simulation."""
    robot = _robot(backend, can_name)
    try:
        if backend == "mujoco" and steps > 0:
            impl = robot._backend
            impl._step(steps)
        typer.echo(robot.state())
    finally:
        robot.disconnect()


@app.command()
def camera(backend: str = "real", can_name: str = "can0"):
    """Read one 1280x720 RGB-D frame and print its metadata."""
    if backend != "real":
        raise typer.BadParameter("camera command currently supports backend=real")
    from .sensors.realsense import RealSenseCamera
    cam = RealSenseCamera()
    cam.connect()
    try:
        frame = cam.read()
        typer.echo(json.dumps({"shape": frame.color.shape, "depth_shape": frame.depth.shape,
                               "frame_id": frame.frame_id, "depth_scale": frame.depth_scale}))
    finally:
        cam.disconnect()


if __name__ == "__main__":
    app()
