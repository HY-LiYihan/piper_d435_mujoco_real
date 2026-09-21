from __future__ import annotations

from pathlib import Path
import tempfile
import time
import xml.etree.ElementTree as ET
import math
import numpy as np
from ..api.types import JointState, Pose, RobotState
from ..errors import BackendUnavailableError, IKError, NotConnectedError
from ..kinematics.ik import NumericalIK


DEFAULT_MODEL = Path(__file__).parents[3] / "vendor/piper_isaac_sim/piper_description/mujoco_model/piper_description.xml"
WRIST_URDF = Path(__file__).parents[3] / "vendor/piper_isaac_sim/piper_description/urdf/piper_description_v100_realsense_camera_v2.urdf"
WRIST_D435_MESH = Path(__file__).parents[3] / "vendor/piper_isaac_sim/realsense2_description/meshes/d435.dae"
WRIST_STAND_MESH = Path(__file__).parents[3] / "vendor/piper_isaac_sim/piper_description/meshes/dae/realsense_mid_stand.dae"


class MujocoBackend:
    def __init__(self, model_path: str | Path = DEFAULT_MODEL, realtime: bool = False,
                 settle_steps: int = 200, wrist_camera: bool = True, **_: object):
        self.model_path = Path(model_path)
        self.realtime = realtime
        if settle_steps < 1:
            raise ValueError("settle_steps must be positive")
        self.settle_steps = settle_steps
        self.wrist_camera = wrist_camera
        self._temporary_files: list[Path] = []
        self.model = self.data = self.ik = None
        self._connected = False
        self._last = time.monotonic()

    def connect(self) -> None:
        try:
            import mujoco
        except ImportError as exc:
            raise BackendUnavailableError("Install piper-control[mujoco] to use MuJoCo") from exc
        if not self.model_path.exists():
            raise BackendUnavailableError(f"MuJoCo model not found: {self.model_path}")
        model_path = self._model_with_wrist_camera() if self.wrist_camera else self.model_path
        try:
            self.model = mujoco.MjModel.from_xml_path(str(model_path))
        finally:
            if model_path != self.model_path:
                model_path.unlink(missing_ok=True)
            for temporary_file in self._temporary_files:
                temporary_file.unlink(missing_ok=True)
            self._temporary_files.clear()
        self.data = mujoco.MjData(self.model)
        joint_ids = np.array([self.model.joint(f"joint{i+1}").id for i in range(6)])
        lower = self.model.jnt_range[joint_ids, 0].copy()
        upper = self.model.jnt_range[joint_ids, 1].copy()
        body_id = self.model.body("link6").id
        self.ik = NumericalIK(self.model, self.data, joint_ids, body_id, lower, upper)
        mujoco.mj_forward(self.model, self.data)
        self._connected = True

    def _model_with_wrist_camera(self) -> Path:
        """Build a temporary MuJoCo scene from the upstream wrist-camera URDF.

        The fixed transforms and meshes are read from the pinned upstream URDF
        and RealSense description; this keeps the MuJoCo camera aligned with
        the documented Piper D435 mount rather than duplicating measurements.
        """
        tree = ET.parse(self.model_path)
        root = tree.getroot()
        link6 = next((body for body in root.iter("body") if body.get("name") == "link6"), None)
        if link6 is None:
            raise BackendUnavailableError("Piper MuJoCo model has no link6 wrist body")
        if not WRIST_URDF.exists() or not WRIST_D435_MESH.exists() or not WRIST_STAND_MESH.exists():
            raise BackendUnavailableError("Pinned Piper D435 URDF or mesh asset is missing")
        urdf_root = ET.parse(WRIST_URDF).getroot()
        joints = {joint.get("name"): joint for joint in urdf_root.findall("joint")}

        def origin(joint_name: str) -> tuple[list[float], list[float]]:
            element = joints[joint_name].find("origin")
            if element is None:
                return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
            xyz = [float(value) for value in element.get("xyz", "0 0 0").split()]
            rpy = [float(value) for value in element.get("rpy", "0 0 0").split()]
            return xyz, rpy

        def quat(rpy: list[float]) -> list[float]:
            roll, pitch, yaw = rpy
            cr, sr = math.cos(roll / 2), math.sin(roll / 2)
            cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
            cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
            return [cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
                    cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy]

        def quat_mul(first: list[float], second: list[float]) -> list[float]:
            aw, ax, ay, az = first
            bw, bx, by, bz = second
            return [aw * bw - ax * bx - ay * by - az * bz,
                    aw * bx + ax * bw + ay * bz - az * by,
                    aw * by - ax * bz + ay * bw + az * bx,
                    aw * bz + ax * by - ay * bx + az * bw]

        def values(items: list[float]) -> str:
            return " ".join(f"{item:.9g}" for item in items)

        asset = root.find("asset")
        if asset is None:
            asset = ET.Element("asset")
            root.insert(0, asset)
        d435_obj = self._dae_to_obj(WRIST_D435_MESH)
        stand_obj = self._dae_to_obj(WRIST_STAND_MESH)
        ET.SubElement(asset, "mesh", name="wrist_camera_d435", file=str(d435_obj))
        ET.SubElement(asset, "mesh", name="wrist_camera_stand", file=str(stand_obj))

        stand_xyz, stand_rpy = origin("camera_stand_joint")
        stand = ET.SubElement(link6, "body", name="camera_stand_link",
                              pos=values(stand_xyz), quat=values(quat(stand_rpy)))
        ET.SubElement(stand, "geom", type="mesh", mesh="wrist_camera_stand",
                      contype="0", conaffinity="0")

        mount_xyz, mount_rpy = origin("d435_camera_joint")
        mount = ET.SubElement(link6, "body", name="d435_camera_link",
                              pos=values(mount_xyz), quat=values(quat(mount_rpy)))
        link_xyz, link_rpy = origin("camera_link_joint")
        camera_link = ET.SubElement(mount, "body", name="camera_link",
                                    pos=values(link_xyz), quat=values(quat(link_rpy)))
        ET.SubElement(camera_link, "geom", type="mesh", mesh="wrist_camera_d435",
                      pos="0.0043 -0.0175 0", quat=values(quat([math.pi / 2, 0, math.pi / 2])),
                      contype="0", conaffinity="0")
        # Use the RealSense optical-frame convention from _d435.urdf.xacro.
        # ROS optical (+Z forward, +Y down) to MuJoCo camera (-Z forward, +Y up).
        optical_quat = quat_mul(quat([-math.pi / 2, 0, -math.pi / 2]), quat([math.pi, 0, 0]))
        ET.SubElement(camera_link, "camera", name="wrist_camera", pos="0 0 0",
                      quat=values(optical_quat), fovy="60")
        temp = tempfile.NamedTemporaryFile(prefix="piper_wrist_", suffix=".xml", dir=self.model_path.parent, delete=False)
        tree.write(temp.name, encoding="utf-8", xml_declaration=True)
        temp.close()
        return Path(temp.name)

    def _dae_to_obj(self, source: Path) -> Path:
        """Convert an upstream COLLADA mesh to a temporary OBJ for MuJoCo.

        The geometry remains entirely sourced from the pinned DAE; this is
        only a format bridge because MuJoCo does not load COLLADA directly.
        """
        namespace = "{http://www.collada.org/2005/11/COLLADASchema}"
        root = ET.parse(source).getroot()
        output = tempfile.NamedTemporaryFile(prefix=f"{source.stem}_", suffix=".obj", delete=False)
        output_path = Path(output.name)
        vertex_offset = 0
        with output:
            output.write(b"# Converted at runtime from pinned upstream COLLADA asset\n")
            for geometry in root.findall(f".//{namespace}geometry"):
                mesh = geometry.find(f"{namespace}mesh")
                if mesh is None:
                    continue
                sources = {}
                for source_element in mesh.findall(f"{namespace}source"):
                    array = source_element.find(f"{namespace}float_array")
                    if array is not None and array.text:
                        sources[f"#{source_element.get('id')}"] = [float(value) for value in array.text.split()]
                vertices = mesh.find(f"{namespace}vertices")
                position_input = None if vertices is None else next(
                    (item for item in vertices.findall(f"{namespace}input") if item.get("semantic") == "POSITION"), None
                )
                if position_input is None or position_input.get("source") not in sources:
                    continue
                position_values = sources[position_input.get("source")]
                vertex_count = len(position_values) // 3
                for index in range(vertex_count):
                    x, y, z = position_values[index * 3:index * 3 + 3]
                    output.write(f"v {x:.9g} {y:.9g} {z:.9g}\n".encode())
                for primitive_name in ("triangles", "polylist"):
                    for primitive in mesh.findall(f"{namespace}{primitive_name}"):
                        inputs = primitive.findall(f"{namespace}input")
                        vertex_input = next((item for item in inputs if item.get("semantic") == "VERTEX"), None)
                        if vertex_input is None or not primitive.find(f"{namespace}p").text:
                            continue
                        stride = max(int(item.get("offset", "0")) for item in inputs) + 1
                        values = [int(value) for value in primitive.find(f"{namespace}p").text.split()]
                        if primitive_name == "triangles":
                            counts = [3] * (len(values) // stride // 3)
                        else:
                            vcount = primitive.find(f"{namespace}vcount")
                            counts = [int(value) for value in vcount.text.split()] if vcount is not None and vcount.text else []
                        cursor = 0
                        for count in counts:
                            polygon = []
                            for _ in range(count):
                                polygon.append(values[cursor * stride + int(vertex_input.get("offset", "0"))] + 1 + vertex_offset)
                                cursor += 1
                            for index in range(1, len(polygon) - 1):
                                output.write(f"f {polygon[0]} {polygon[index]} {polygon[index + 1]}\n".encode())
                vertex_offset += vertex_count
        self._temporary_files.append(output_path)
        return output_path

    def run_gui(self, duration: float = 0.0) -> None:
        """Run a native MuJoCo viewer until closed or duration expires."""
        self._require()
        try:
            import mujoco.viewer
        except ImportError as exc:
            raise BackendUnavailableError("MuJoCo viewer is unavailable in this installation") from exc
        started = time.monotonic()
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            while viewer.is_running():
                if duration > 0 and time.monotonic() - started >= duration:
                    break
                self._step(1)
                viewer.sync()

    def _require(self):
        if not self._connected or self.model is None or self.data is None:
            raise NotConnectedError("MuJoCo backend is not connected")

    def disconnect(self) -> None:
        self._connected = False
        self.model = self.data = self.ik = None

    def _step(self, steps: int = 1) -> None:
        import mujoco
        for _ in range(steps):
            mujoco.mj_step(self.model, self.data)
        if self.realtime:
            elapsed = time.monotonic() - self._last
            delay = max(0.0, self.model.opt.timestep * steps - elapsed)
            if delay:
                time.sleep(delay)
        self._last = time.monotonic()

    def state(self) -> RobotState:
        self._require()
        import mujoco
        mujoco.mj_forward(self.model, self.data)
        body = self.model.body("link6").id
        quat = self.data.xquat[body]
        pose = Pose(tuple(self.data.xpos[body]), tuple(quat))
        gripper_width = float(self.data.qpos[6] - self.data.qpos[7])
        return RobotState(True, False, JointState(self.data.qpos[:6], self.data.qvel[:6], gripper_width), pose)

    def move_joints(self, joints) -> None:
        self._require()
        q = np.asarray(joints, dtype=float)
        if q.shape != (6,):
            raise ValueError("move_joints requires six joint values in radians")
        lower, upper = self.model.jnt_range[:6, 0], self.model.jnt_range[:6, 1]
        q = np.clip(q, lower, upper)
        # The public simulation API is state-command based. Keeping qpos and ctrl
        # aligned makes interface tests deterministic despite the source XML's
        # high-gain position actuators oscillating during short runs.
        self.data.qpos[:6] = q
        self.data.qvel[:6] = 0.0
        self.data.ctrl[:6] = q
        import mujoco
        mujoco.mj_forward(self.model, self.data)

    def move_p(self, pose: Pose) -> None:
        self._require()
        result = self.ik.solve(pose, seed=self.data.qpos[:6])
        if not result.success:
            raise IKError(f"MuJoCo IK failed: {result.message}; position={result.position_error:.6g}")
        self.move_joints(result.joints)

    def gripper(self, width: float, effort: float | None = None) -> None:
        self._require()
        if not 0.0 <= width <= 0.07:
            raise ValueError("gripper width must be between 0 and 0.07 metres")
        self.data.qpos[6] = width / 2.0
        self.data.qpos[7] = -width / 2.0
        self.data.qvel[6:8] = 0.0
        self.data.ctrl[6] = width / 2.0
        self.data.ctrl[7] = -width / 2.0
        import mujoco
        mujoco.mj_forward(self.model, self.data)

    def stop(self) -> None:
        self._require()
        self.data.ctrl[:] = self.data.qpos[: self.model.nu]
