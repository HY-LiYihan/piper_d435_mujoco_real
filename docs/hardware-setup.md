# Hardware setup

On a new Linux control PC, after checking out this repository (including submodules), install the renamed package in a Python 3.10+ environment and verify its entry points without connecting to hardware:

```bash
python3 -m venv .venv
. .venv/bin/activate
python -m pip install -e .
python -c 'from robot_control import Robot; print(Robot.__name__)'
robot_control --help
```

For an existing environment upgraded from the previous name, run `python -m pip uninstall piper-control` before the new installation to remove the old `piper` command. Do not uninstall `piper_sdk`: it is the Piper hardware dependency, not the application name.

Install `robot-control[real,camera]` for Piper plus D435i on Linux x86_64, configure the CAN interface using the scripts in `vendor/piper_sdk`, and connect the camera. For FR3-only operation, start with `python -m pip install -e .` and install matching `pylibfranka` / `libfranka` on the Linux control PC separately. The direct backend follows the reference client's 0.13.5 API: do not substitute a newer PyPI `pylibfranka` release without validating both its API and matching `libfranka` ABI. Neither binding nor shared library is bundled with this repository; successful project installation alone does not verify FR3 hardware readiness. The Python import is `from robot_control import Robot`; the CLI is `robot_control` (there is no `piper` CLI entry point). The default camera stream is aligned 1280x720 RGB/depth at 30 FPS.

Dependency checks: Python 3.10 with Pinocchio 3.8 and Python 3.11 with Pinocchio 3.9 both passed the complete offline test suite. The `mujoco`, `real`, `camera`, and `dev` extras resolve together for Linux x86_64 on both Python versions; RealSense capture and FCI motion still need on-machine hardware verification. `pyrealsense2` is not part of the macOS validation environment.

Use `robot_control doctor` before connecting. Piper's real backend defaults to `can0`; override with `--can-name` after the subcommand. Always specify `--backend real` and the robot type before real-arm commands. FR3 uses a local Linux `pylibfranka`/matching `libfranka` installation and connects directly to `FRANKA_ROBOT_IP` (default `192.168.1.6`), without an HTTP service; start with `robot_control --backend real --robot franka_fr3 state`. CLI movement requires a typed confirmation. FR3 hardware motion has not been tested; keep the workspace clear and the physical emergency stop accessible. Software `stop` is not an emergency stop.

Piper starts with the configured initial joint target `[0°, 30°, -45°, 0°, 60°, 0°]`. MuJoCo places the simulated arm there immediately. The Piper real backend sends the same target through MOVE J after connecting and waits for measured feedback to be within 0.02 rad; keep the workspace clear before any Piper real or twin command.

For local simulation, use `robot_control --backend mujoco` to open the GUI, or add `--robot franka_fr3` for FR3. The wrist RGB-D command is `robot_control camera --rgb-out wrist_rgb.png --depth-out wrist_depth.npy`; with one scene running it targets that scene automatically and prints `T_base_color_optical` extrinsics. On real Piper hardware, use `robot_control --backend real --robot piper camera` to capture RGB-D and compute the same extrinsics from joint feedback and the shared URDF. Standalone RealSense capture without arm extrinsics is `robot_control --backend real camera --no-extrinsics`.
