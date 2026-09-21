# Piper Control

统一的 Piper 机械臂 Python API、MuJoCo 仿真、Piper 真机和 D435/D435i RGB-D 接口。

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

CLI：`piper doctor`, `piper state`, `piper move-joints`, `piper move-p`, `piper gripper`, `piper stop`。

## 开发

```bash
pytest
git submodule update --init --recursive
```
