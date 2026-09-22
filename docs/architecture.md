# Architecture

`PiperRobot` exposes one typed facade. A backend implements robot lifecycle, state, joint/cartesian motion, gripper and stop.

The real backend delegates Cartesian `move_p` to the pinned Piper SDK/firmware. The MuJoCo backend solves Cartesian targets with damped least-squares IK using MuJoCo Jacobians, then drives the model position actuators.

Both camera providers return `RGBDFrame`: RGB is `uint8 HxWx3`, depth is `float32 HxW`, and metadata contains resolution, intrinsics, timestamp, frame id and depth scale. RealSense uses `pyrealsense2`; simulation uses the official D435i color-optical pose for both color and color-aligned depth rendering.

The public unit conventions are metres, radians and `(w, x, y, z)` quaternions. ROS 2 is intentionally not a first-stage dependency.

The upstream Isaac asset repository contains DAE paths that differ only by letter case. On case-insensitive macOS filesystems that submodule can appear dirty immediately after checkout, so `.gitmodules` ignores submodule worktree dirt while the pinned gitlink SHA remains authoritative.
