# API contract

`Pose.position` uses metres and `Pose.quaternion` uses `(w, x, y, z)`. `PiperRobot.move_joints` takes six radians. `gripper` takes opening width in metres. `state()` returns six arm joints, gripper state, optional end-effector pose, connection status and timestamps.

The MuJoCo backend uses deterministic state-command semantics for the public API: target `qpos` and actuator controls are updated together, then MuJoCo forward kinematics is refreshed. This keeps tests independent of the source XML's high-gain actuator transient; `_step()` remains available for explicit dynamics experiments.
