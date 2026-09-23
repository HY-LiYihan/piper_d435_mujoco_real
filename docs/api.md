# API contract

`Pose.position` uses metres and `Pose.quaternion` uses `(w, x, y, z)`. `PiperRobot.move_joints` takes six radians. `gripper` takes opening width in metres. `state()` returns six arm joints, gripper state, optional end-effector pose, connection status and timestamps.

MuJoCo motion commands are nonblocking setpoint commands. `move_p` solves IK
with Pinocchio using the current measured arm configuration as its seed, then
writes the resulting joint position targets to MuJoCo position-actuator controls.
`move_joints` and `gripper` also update controls only. None of these commands
writes the live `qpos` or `qvel`. Pinocchio operates on an independent model;
failed IK leaves both the existing controls and measured state unchanged.

The GUI advances physics continuously. Standalone scripts call `robot.step(n)`
to advance physics explicitly, or `robot.wait_until_idle(timeout=10)` to step
until measured position errors and velocities settle. The standalone timeout
is in simulation seconds; shared-scene waiting polls the GUI using wall time.
CLI motion commands wait for settling before printing measured results.
Timeout raises `TimeoutError`; it does not force completion or cancel the target.
`state().moving` reflects both tracking error and measured velocity. `stop()`
sets holding targets to the current measured positions; it does not erase
velocity, so physical deceleration is still required.

The default simulation uses the official `agx_arm_urdf` Piper model and reports
the `link6` pose. Its gripper accepts 0–0.10 m total opening; the two physical
fingers move by +/- half that width. The real backend retains its existing
0–0.07 m command limit and SDK Cartesian control. The model migration does not
change hardware limits or replace firmware IK on the real backend.
