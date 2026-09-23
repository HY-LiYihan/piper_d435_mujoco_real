from __future__ import annotations

import json
import os
from pathlib import Path
import subprocess
import sys
from typing import Annotated
import typer
from .api.robot import PiperRobot
from .api.types import Pose

app = typer.Typer(help="Piper (default) and Franka FR3 control and D435i RGB-D CLI")


def _robot_name(robot: str) -> str:
    selected = "piper" if robot == "pepper" else robot
    if selected not in ("piper", "franka_fr3"):
        raise typer.BadParameter("choose piper or franka_fr3", param_hint="--robot")
    return selected


def _robot(backend: str, can_name: str, robot: str = "piper"):
    return PiperRobot.connect(backend, {"can_name": can_name} if backend == "real" else {},
                              robot=_robot_name(robot))


def _run_scene_host(duration: float, scene: Path | None = None, robot: str = "piper") -> None:
    """Host the shared MuJoCo scene; on macOS re-exec through mjpython for the viewer."""
    if sys.platform == "darwin" and not os.environ.get("PIPER_MUJOCO_GUI_REEXEC"):
        mjpython = Path(sys.executable).with_name("mjpython")
        if not mjpython.is_file():
            raise RuntimeError("Install piper-control[mujoco] in this Python environment to provide mjpython")
        environment = os.environ.copy()
        environment["PIPER_MUJOCO_GUI_REEXEC"] = "1"
        source_root = str(Path(__file__).resolve().parents[1])
        environment["PYTHONPATH"] = source_root + os.pathsep + environment.get("PYTHONPATH", "")
        command = [str(mjpython), "-m", "piper_control.mujoco_gui", "--duration", str(duration)]
        if robot != "piper":
            command.extend(["--robot", robot])
        if scene is not None:
            command.extend(["--scene", str(scene)])
        subprocess.run(command, env=environment, check=True)
        return
    from .mujoco_gui import run_host
    run_host(duration, scene=scene, robot=robot)


def _pose_dict(pose: Pose) -> dict[str, list[float]]:
    return {
        "position_m": [float(value) for value in pose.position],
        "quaternion_wxyz": [float(value) for value in pose.quaternion],
    }


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
def state(backend: str = "mujoco", can_name: str = "can0", robot: str = "piper"):
    instance = _robot(backend, can_name, robot)
    try:
        typer.echo(instance.state())
    finally:
        instance.disconnect()


@app.command()
def pose(backend: str = "mujoco", can_name: str = "can0", robot: str = "piper"):
    """Print the current end-effector pose as JSON."""
    instance = _robot(backend, can_name, robot)
    try:
        current = instance.state().pose
        if current is None:
            raise typer.BadParameter("backend did not return an end-effector pose")
        typer.echo(json.dumps(_pose_dict(current), indent=2))
    finally:
        instance.disconnect()


@app.command("move-joints")
def move_joints(
    j1: float = typer.Option(..., "--j1", help="Joint 1 in radians"),
    j2: float = typer.Option(..., "--j2", help="Joint 2 in radians"),
    j3: float = typer.Option(..., "--j3", help="Joint 3 in radians"),
    j4: float = typer.Option(..., "--j4", help="Joint 4 in radians"),
    j5: float = typer.Option(..., "--j5", help="Joint 5 in radians"),
    j6: float = typer.Option(..., "--j6", help="Joint 6 in radians"),
    j7: float | None = typer.Option(None, "--j7", help="Joint 7, required only for franka_fr3"),
    backend: str = "mujoco", can_name: str = "can0", robot: str = "piper"):
    selected = _robot_name(robot)
    if selected == "franka_fr3" and j7 is None:
        raise typer.BadParameter("--j7 is required for franka_fr3", param_hint="--j7")
    if selected == "piper" and j7 is not None:
        raise typer.BadParameter("--j7 is only valid for franka_fr3", param_hint="--j7")
    instance = _robot(backend, can_name, selected)
    try:
        joints = [j1, j2, j3, j4, j5, j6]
        if j7 is not None:
            joints.append(j7)
        instance.move_joints(joints)
        if backend == "mujoco":
            instance.wait_until_idle()
        typer.echo(json.dumps({"joints_rad": instance.state().joints.positions.tolist()}, indent=2))
    finally:
        instance.disconnect()


@app.command("move-p")
def move_p(
           x: float = typer.Option(..., "--x", help="X position in metres"),
           y: float = typer.Option(..., "--y", help="Y position in metres"),
           z: float = typer.Option(..., "--z", help="Z position in metres"),
           qw: float | None = typer.Option(None, "--qw"), qx: float | None = typer.Option(None, "--qx"),
           qy: float | None = typer.Option(None, "--qy"), qz: float | None = typer.Option(None, "--qz"),
           backend: str = "mujoco", can_name: str = "can0", robot: str = "piper"):
    instance = _robot(backend, can_name, robot)
    try:
        quaternion = (qw, qx, qy, qz)
        if all(value is None for value in quaternion):
            current = instance.state().pose
            if current is None:
                raise typer.BadParameter("backend did not return a pose for orientation hold")
            quaternion = current.quaternion
        elif any(value is None for value in quaternion):
            raise typer.BadParameter("provide all four quaternion options or none")
        instance.move_p(Pose((x, y, z), tuple(float(value) for value in quaternion)))
        if backend == "mujoco":
            instance.wait_until_idle()
        current = instance.state().pose
        if current is not None:
            typer.echo(json.dumps(_pose_dict(current), indent=2))
    finally:
        instance.disconnect()


@app.command()
def gripper(width: float, effort: float | None = None, backend: str = "mujoco", can_name: str = "can0", robot: str = "piper"):
    instance = _robot(backend, can_name, robot)
    try:
        instance.gripper(width, effort)
        if backend == "mujoco":
            instance.wait_until_idle()
        typer.echo(json.dumps({"gripper_width_m": instance.state().joints.gripper}, indent=2))
    finally:
        instance.disconnect()


@app.command()
def stop(backend: str = "mujoco", can_name: str = "can0", robot: str = "piper"):
    instance = _robot(backend, can_name, robot)
    try:
        instance.stop()
    finally:
        instance.disconnect()


@app.command()
def run(backend: str = "mujoco", can_name: str = "can0", steps: int = 0,
        gui: bool = False, duration: float = 0.0,
        scene: Annotated[Path | None, typer.Option("--scene", help="MuJoCo scene XML with the selected robot's mount pose")] = None,
        robot: str = "piper"):
    """Own the shared MuJoCo scene (--gui) or report its current state."""
    selected = _robot_name(robot)
    if scene is not None:
        if backend != "mujoco":
            raise typer.BadParameter("--scene is only supported by the MuJoCo backend", param_hint="--scene")
        if selected == "franka_fr3":
            from .fr3.scene_builder import validate_scene
        else:
            from .backends.scene_builder import validate_scene
        try:
            scene = validate_scene(scene)
        except ValueError as exc:
            raise typer.BadParameter(str(exc), param_hint="--scene") from exc
    if gui:
        if backend != "mujoco":
            raise typer.BadParameter("--gui is only supported by the MuJoCo backend")
        if selected == "piper":
            _run_scene_host(duration, scene=scene)
        else:
            _run_scene_host(duration, scene=scene, robot=selected)
        return
    if backend == "mujoco" and (steps > 0 or scene is not None):
        # Standalone stepping keeps the documented --steps mode working
        # without requiring a running GUI process.
        if selected == "franka_fr3":
            from .fr3.mujoco import MujocoBackend
        else:
            from .backends.mujoco import MujocoBackend
        impl = MujocoBackend(realtime=True, scene=scene)
        impl.connect()
        try:
            impl._step(steps)
            typer.echo(impl.state())
        finally:
            impl.disconnect()
        return
    instance = _robot(backend, can_name, selected)
    try:
        typer.echo(instance.state())
    finally:
        instance.disconnect()


@app.command()
def camera(backend: str = "mujoco", can_name: str = "can0",
           rgb_out: Path = Path("wrist_rgb.png"), depth_out: Path = Path("wrist_depth.npy"),
           robot: str = "piper"):
    """Capture one aligned 1280x720 wrist RGB-D frame."""
    selected = _robot_name(robot)
    instance = None
    if backend == "real":
        from .sensors.realsense import RealSenseCamera
        cam = RealSenseCamera()
    elif backend == "mujoco":
        instance = _robot("mujoco", can_name, selected)
        impl = instance._backend
        if hasattr(impl, "read"):
            cam = impl  # shared-scene client renders from the running scene
        else:
            from .sensors.mujoco_rgbd import MujocoRGBDCamera
            cam = MujocoRGBDCamera(impl.model, impl.data,
                                  camera="d435i_check" if selected == "franka_fr3" else "d435i_color_optical_camera")
    else:
        raise typer.BadParameter(f"unknown backend: {backend}")
    cam.connect()
    try:
        frame = cam.read()
        import numpy as np
        try:
            from PIL import Image
        except ImportError as exc:
            raise RuntimeError("Install piper-control[camera] to save PNG images") from exc
        rgb_out.parent.mkdir(parents=True, exist_ok=True)
        depth_out.parent.mkdir(parents=True, exist_ok=True)
        Image.fromarray(frame.color).save(rgb_out)
        np.save(depth_out, frame.depth)
        typer.echo(json.dumps({
            "rgb": str(rgb_out.resolve()),
            "depth": str(depth_out.resolve()),
            "color_shape": list(frame.color.shape),
            "depth_shape": list(frame.depth.shape),
            "frame_id": frame.frame_id,
            "depth_scale": frame.depth_scale,
        }, indent=2))
    finally:
        cam.disconnect()
        if instance is not None:
            instance.disconnect()


if __name__ == "__main__":
    app()
