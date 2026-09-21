# API contract

`Pose.position` uses metres and `Pose.quaternion` uses `(w, x, y, z)`. `PiperRobot.move_joints` takes six radians. `gripper` takes opening width in metres. `state()` returns six arm joints, gripper state, optional end-effector pose, connection status and timestamps.
