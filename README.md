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

不传 `--scene` 时使用 MuJoCo 示例常见的蓝色渐变天空和棋盘地面。指定其他场景：

```bash
piper run --backend mujoco --gui --scene scenes/tabletop.xml
# 无窗口运行，也支持 --scene：
piper run --backend mujoco --steps 100 --scene scenes/tabletop.xml
```

`--scene` 仅用于 `piper run` 的 MuJoCo 后端，真机后端会拒绝该参数。场景采用原生 MJCF XML；加载机器人和资源前，会检查主 XML 的 `<worldbody>` 下是否有且只有一个空的固定挂载节点：

```xml
<body name="piper_mount" pos="-0.3 0 0.75" quat="1 0 0 0"/>
```

`pos` 必填，是基座在世界中的位置，单位米；`quat` 是 wxyz 四元数，可省略以使用单位朝向。该节点不能包含关节、子节点或其他属性，不能嵌在其他 body/frame 中，也不能放在 include 文件内。缺少节点、缺少位置、非有限数值或无效四元数都会报错，不会静默放到原点。其余场景内容可使用 MuJoCo 的 include、相对资源路径、静态物体和可运动物体；机器人及其夹爪、支架、相机由程序整体挂载。可复制 `scenes/tabletop.xml` 修改桌子、物体、灯光和安装位姿。切换场景需要关闭原 GUI 后重新启动，后续控制和相机命令连接这个运行中的场景。

MuJoCo 的 `pose` 输出和 `move-p` 输入统一使用**场景世界坐标**；后端自动转换成 Pinocchio 模型坐标后求解 IK，因此移动或旋转基座不需要修改目标求解器。场景碰撞会影响实际运动，但 IK 本身不提供避障路径。环境通过 MuJoCo 3.2.7+ 的模型装配接口加载；机械臂继续采用 0.002 s 步长、implicitfast 积分器和原有控制参数。

读取当前末端位姿（位置 m，四元数顺序 wxyz；MuJoCo 下为世界坐标）：

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

MuJoCo 启动时读取 `vendor/agx_arm_urdf/piper/urdf/piper_with_gripper_description.xacro` 及其引用的 `piper_description.urdf`，转换为仿真模型。关节坐标、限位、质量和惯量来自这些文件，末端固定为 `link6`。显示使用官方 `visual` 引用的 DAE 网格，保留部件变换、法线和材质颜色；转换器按颜色分组生成内嵌 MJCF 网格，无需额外依赖或手工生成资源。显示网格位于 group 1，关闭碰撞且质量为零；碰撞继续使用原有 STL，位于默认隐藏的 group 3（可在查看器中打开检查）。上游 DAE 没有图片贴图，本次恢复的是原有几何和材质颜色。法兰与夹爪结构来自新 Xacro；仿真夹爪开口范围为 0–0.10 m，两侧手指通过等式约束同步运动。

IK 使用 Pinocchio（安装包名 `pin`），读取同一官方机械臂 URDF，求解末端 `link6` 的关节目标。`move_p`、`move_joints` 和 `gripper` 仅下发执行器控制目标，不改写实际关节位置或速度。GUI 持续推进物理仿真；独立脚本可调用 `robot.step(n)` 或 `robot.wait_until_idle()`。CLI 运动命令会等待实际运动完成后返回；超时会报错，不会强制关节到位。`stop()` 下发当前位置保持目标，通过执行器减速。

位置执行器、阻尼和经执行器限力的重力补偿由本项目配置。D435 外壳及打印支架仍使用旧 Isaac 仓库的两个 DAE 文件，外壳显示为带高光的银色，打印支架为黑色；材质设置位于 `backends/mujoco.py`，属于本地外观配置。默认照明适当调亮以显示深色机械臂细节。相机安装变换和 nominal extrinsics 保留在本项目代码中，未做实机重新标定。真机 SDK/固件 IK 控制路径与原有 0–0.07 m 夹爪限制暂不改变。已经运行的 GUI/共享场景需重启才能加载新模型和控制逻辑。macOS GUI 使用当前 Python 环境旁的 `mjpython`，请在同一环境安装依赖。

## 开发

```bash
pytest
git submodule update --init --recursive
```
