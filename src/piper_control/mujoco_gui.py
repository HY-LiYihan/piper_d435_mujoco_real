from __future__ import annotations

import argparse

from .backends.mujoco import MujocoBackend


def main() -> None:
    parser = argparse.ArgumentParser(description="Open the Piper MuJoCo viewer")
    parser.add_argument("--duration", type=float, default=0.0, help="Seconds; 0 keeps the window open")
    args = parser.parse_args()
    backend = MujocoBackend(realtime=True)
    backend.connect()
    try:
        backend.run_gui(args.duration)
    finally:
        backend.disconnect()


if __name__ == "__main__":
    main()
