# Piper Control

统一的 Piper 机械臂 Python API、MuJoCo 仿真、Piper 真机和 D435i RGB-D 接口。

## 上游版本

| 组件 | URL | 固定版本 |
| --- | --- | --- |
| Piper SDK | https://github.com/agilexrobotics/piper_sdk | `0.6.2`, `c9e8a28174e71eeaac448593cb65f8ab258a92fe` |
| Piper 官方 URDF / 夹爪 / meshes | https://github.com/agilexrobotics/agx_arm_urdf | `f6642ce0d7872c686f29c99e9e10cd23d1d49313` |
| D435 外壳和支架资源 | https://github.com/agilexrobotics/piper_isaac_sim | `8e1f88fdb7afca49c40e9a0c1c01cc588e86f0d2` |

两个模型仓库作为 submodule 保留。机械臂与夹爪使用 `agx_arm_urdf/piper`；旧 Isaac 仓库仅提供 D435 外壳和打印支架，不要求 Isaac Sim 或 ROS。

## 安装

```bash
git submodule update --init --recursive
python -m pip install -e ".[mujoco,dev]"
pip install -e ".[real,camera]"  # 需要真机/RealSense 时
```

## 使用

```python
from piper_control import PiperRobot

robot = PiperRobot.connect("mujoco")
robot.move_joints([0.2, 0.8, -1.2, 0.2, -0.3, 0.4])
robot.wait_until_idle()
target = robot.state().pose
robot.move_joints([0.22, 0.82, -1.22, 0.22, -0.32, 0.42])
robot.wait_until_idle()
robot.move_p(target)
robot.wait_until_idle()
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
piper move-p --backend mujoco --x 0.0561352 --y 0.0 --z 0.2131783
```

指定完整四元数：

```bash
piper move-p --backend mujoco --x 0.0561352 --y 0.0 --z 0.2131783 \
  --qw -0.73727734 --qx 0.0 --qy -0.67559020 --qz 0.0
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

MuJoCo 启动时读取 `vendor/agx_arm_urdf/piper/urdf/piper_with_gripper_description.xacro` 及其引用的 `piper_description.urdf`，转换为仿真模型。关节坐标、限位、质量和惯量来自这些文件，显示和碰撞使用新仓库的 STL 网格，末端固定为 `link6`。法兰与夹爪结构来自新 Xacro；仿真夹爪开口范围为 0–0.10 m，两侧手指通过等式约束同步运动。

IK 使用 Pinocchio（安装包名 `pin`），读取同一官方机械臂 URDF，求解末端 `link6` 的关节目标。`move_p`、`move_joints` 和 `gripper` 仅下发执行器控制目标，不改写实际关节位置或速度。GUI 持续推进物理仿真；独立脚本可调用 `robot.step(n)` 或 `robot.wait_until_idle()`。CLI 运动命令会等待实际运动完成后返回；超时会报错，不会强制关节到位。`stop()` 下发当前位置保持目标，通过执行器减速。

位置执行器、阻尼和经执行器限力的重力补偿由本项目配置。D435 外壳及打印支架仍使用旧 Isaac 仓库的两个 DAE 文件；相机安装变换和 nominal extrinsics 保留在本项目代码中，未做实机重新标定。真机 SDK/固件 IK 控制路径与原有 0–0.07 m 夹爪限制暂不改变。已经运行的 GUI/共享场景需重启才能加载新模型和控制逻辑。macOS GUI 使用当前 Python 环境旁的 `mjpython`，请在同一环境安装依赖。

## 开发

```bash
pytest
git submodule update --init --recursive
```
