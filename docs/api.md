# API contract

`Pose.position` uses metres and `Pose.quaternion` uses `(w, x, y, z)`. `PiperRobot.move_joints` takes six radians. `gripper` takes opening width in metres. `state()` returns six arm joints, gripper state, optional end-effector pose, connection status and timestamps.

The MuJoCo backend uses deterministic state-command semantics for the public API: target `qpos` and actuator controls are updated together, then MuJoCo forward kinematics is refreshed. `_step()` remains available for explicit dynamics experiments.

The default simulation uses the official `agx_arm_urdf` Piper model and reports
the `link6` pose. Its gripper accepts 0–0.10 m total opening; the two physical
fingers move by +/- half that width. The real backend retains its existing
0–0.07 m command limit and SDK Cartesian control. The model migration does not
change hardware limits or install a new IK solver.
