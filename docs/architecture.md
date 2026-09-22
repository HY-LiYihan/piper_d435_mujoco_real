# Architecture

`PiperRobot` exposes one typed facade. A backend implements robot lifecycle, state, joint/cartesian motion, gripper and stop.

The real backend delegates Cartesian `move_p` to the pinned Piper SDK/firmware. The MuJoCo backend solves Cartesian targets with damped least-squares IK using MuJoCo Jacobians, then drives the model position actuators.

`backends/piper_model.py` builds MJCF at startup from the pinned
`agx_arm_urdf/piper/urdf/piper_with_gripper_description.xacro` and its base URDF.
It expands this concrete include-only Xacro without ROS. Transforms, joint limits,
inertial tensors and collision STL meshes come from that repository. The same STL
surfaces are rendered; COLLADA materials are not imported. Fixed bodies including
`flange_link` and `gripper_base` are retained, and the end-effector remains `link6`.
The massless virtual `gripper` driver is eliminated; its +/-0.5 mimic relationship
becomes a MuJoCo equality between the two physical finger joints. This keeps eight
simulation coordinates and eight position actuators, with a 0.10 m total opening.
Joint/actuator addresses in the backend are resolved by name.

Simulation-specific position gains, damping, effort limits and the implicitfast
integrator are configured in the adapter. Effort/control limits follow the URDF;
gains and damping are local tuning, not manufacturer controller parameters.
The public state-command API sets positions immediately; explicit physics stepping
can have gravity-induced servo error. No collision-aware motion planning is added.

The old `piper_isaac_sim` repository is read only for `d435.dae` and
`realsense_mid_stand.dae` on the default path. Camera attachment transforms stay
in `backends/mujoco.py`; they retain the previous nominal mounting calibration.
No old arm XML/URDF or arm/gripper mesh is used. Explicit legacy MJCF overrides
remain supported via `model_path`.

Both camera providers return `RGBDFrame`: RGB is `uint8 HxWx3`, depth is `float32 HxW`, and metadata contains resolution, intrinsics, timestamp, frame id and depth scale. RealSense uses `pyrealsense2`; simulation uses the official D435i color-optical pose for both color and color-aligned depth rendering.

The public unit conventions are metres, radians and `(w, x, y, z)` quaternions. ROS 2 is intentionally not a first-stage dependency.

The upstream Isaac asset repository contains DAE paths that differ only by letter case. On case-insensitive macOS filesystems that submodule can appear dirty immediately after checkout, so `.gitmodules` ignores submodule worktree dirt while the pinned gitlink SHA remains authoritative.
