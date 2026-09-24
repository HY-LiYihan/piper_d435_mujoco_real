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

## Ubuntu FR3 twin deployment

Use an Ubuntu control PC with Python 3.10 or 3.11, an active graphical session for the MuJoCo window, network connectivity to the FR3, and an enabled FCI. Replace the repository path and IP below with your actual values. Install the repository containing `backends/franka_direct.py` (the `piper_d435_mujoco_real` remote), not a different `robot_control` repository that lacks FR3 real control.

```bash
sudo apt update
sudo apt install -y git python3-venv python3-dev
git clone https://github.com/HY-LiYihan/piper_d435_mujoco_real.git
cd piper_d435_mujoco_real
git submodule update --init --recursive
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -e '.[mujoco,dev]'
python -m pytest -q
robot_control --help
```

The repository **does not provide** `pylibfranka` or `libfranka`. Before connecting, install a binding/shared-library pair compatible with the existing `FrankaDirectBackend` API and the FR3's robot-system release. Do not blindly install the latest `pylibfranka` from PyPI: API/ABI compatibility is not established. Once you have the matching wheel and shared library from your FR3 setup, install/verify in the same venv:

```bash
python -m pip install /path/to/compatible/pylibfranka-*.whl
# Install the matching libfranka.so.0.13 under /usr/local/lib (or use your provider's instructions).
sudo ldconfig
python -c 'import pylibfranka; print("pylibfranka import OK")'
python -c 'from robot_control.backends.franka_direct import _bindings; print("FCI bindings:", _bindings().__name__)'
export FRANKA_ROBOT_IP=192.168.1.6  # set this to the real robot address
ping -c 3 "$FRANKA_ROBOT_IP"
```

Check the actual Python and shared-library compatibility **before** connecting; successful import does not validate hardware motion. For FIFO motion scheduling configure Linux real-time permissions for your user as required by the FR3 installation; `FRANKA_RT_PRIORITY=0` is for diagnostics only. In the first terminal (with the venv active), start the read-only mirror:

```bash
robot_control --backend twin --robot franka_fr3
# If no graphical desktop is available: robot_control --backend twin --robot franka_fr3 --no-gui
```

In a second terminal on the **same PC** (activate the same venv and set the same IP), read state without opening another FCI connection:

```bash
cd ~/piper_d435_mujoco_real
source .venv/bin/activate
export FRANKA_ROBOT_IP=192.168.1.6
robot_control --backend real --robot franka_fr3 state
robot_control --backend real --robot franka_fr3 pose
```

Movement is real and requires the CLI confirmation; nothing is sent to the arm by starting the mirror. The twin uses a local socket (`FR3_TWIN_SOCKET`, default `/tmp/fr3_twin.sock`) and never accepts remote connections. Running movement commands requires inspecting the physical workspace and emergency stop first. The real FR3 and twin paths have only been tested with mock FCI hardware, not on a physical arm.
