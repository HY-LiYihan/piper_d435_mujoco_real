# Piper Control

统一的 Piper 机械臂 Python API、MuJoCo 仿真、Piper 真机和 D435i RGB-D 接口。

## 上游版本

| 组件 | URL | 固定版本 |
| --- | --- | --- |
| Piper SDK | https://github.com/agilexrobotics/piper_sdk | `0.6.2`, `c9e8a28174e71eeaac448593cb65f8ab258a92fe` |
| Piper Isaac assets | https://github.com/agilexrobotics/piper_isaac_sim | master, `8e1f88fdb7afca49c40e9a0c1c01cc588e86f0d2` |

Isaac assets 作为 submodule 保留；当前 MVP 使用其中的 MuJoCo XML 和 mesh，不要求 Isaac Sim。

## 安装

```bash
pip install -e ".[mujoco,dev]"
pip install -e ".[real,camera]"  # 需要真机/RealSense 时
```

## 使用

```python
from piper_control import PiperRobot, Pose

robot = PiperRobot.connect("mujoco")
robot.move_p(Pose((0.30, 0.0, 0.30), (0.0, 0.0, 1.0, 0.0)))
print(robot.state())
robot.stop()
robot.disconnect()
```

## 常用仿真命令

打开 MuJoCo GUI（窗口关闭前持续运行）：

```bash
piper run --backend mujoco --gui
```

读取当前末端位姿（位置 m，四元数顺序 wxyz）：

```bash
piper pose --backend mujoco
```

移动到指定位置；不提供四元数时保持当前姿态：

```bash
piper move-p --backend mujoco --x 0.055 --y 0.0 --z 0.203
```

指定完整四元数：

```bash
piper move-p --backend mujoco --x 0.055 --y 0.0 --z 0.203 \
  --qw 1.0 --qx 0.0 --qy 0.0 --qz 0.0
```

控制夹爪，单位为米：

```bash
piper gripper 0.02 --backend mujoco
```

获取腕部 D435 RGB-D。RGB 为 PNG，depth 为米制 `float32` NumPy 文件，分辨率固定 1280x720：

```bash
piper camera --backend mujoco \
  --rgb-out wrist_rgb.png --depth-out wrist_depth.npy
```

关节命令使用弧度，并采用选项形式以支持负数：

```bash
piper move-joints --backend mujoco \
  --j1 0.1 --j2 0.2 --j3 -0.2 --j4 0 --j5 0 --j6 0
```

MuJoCo 使用官方普通 Piper 带夹爪 XML 作为机械臂主体，在运行时附加官方 Isaac 资源中的打印支架和 D435 外壳。D435i 的 RGB、Depth、双红外和 IMU 坐标链采用上游 `realsense2_description` 的 nominal extrinsics；机械臂本体的关节、夹爪和 FK 不会因相机附加物改变。

## 开发

```bash
pytest
git submodule update --init --recursive
```
