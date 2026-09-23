# FR3 + D435i wrist-camera URDF

`urdf/fr3_d435i_wrist_camera.urdf` is a single-file, directly loadable model. It includes a 7-DOF FR3-style chain, the supplied `models/FR3_D435i_mount.stl`, and D435i color/depth/IMU frames. The STL is authored in millimetres, so the URDF applies `scale="0.001 0.001 0.001"`.

For a real FR3 description, include `urdf/fr3_d435i_wrist_camera.xacro` from your robot launch file and instantiate:

```xml
<xacro:include filename="$(find fr3_d435i_wrist_camera_mount)/urdf/fr3_d435i_wrist_camera.xacro"/>
<xacro:fr3_d435i_wrist_camera parent_link="franka_hand" mesh_filename="$(find fr3_d435i_wrist_camera_mount)/models/FR3_D435i_mount.stl"/>
```

The supplied mount is attached to the official FR3 hand frame; the official hand and finger links remain in the model. The camera joint uses the CAD measurement `(72.566, 31.620, 13.559) mm`, with the camera lowered 5 mm below the CNC/gripper underside, and a `+25 deg` in-plane yaw (`rpy="0 0 0.436332313"`). The D435 visual uses the RealSense description mesh, while its collision remains a simple box.
