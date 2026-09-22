from __future__ import annotations

import argparse

from .backends.mujoco import MujocoBackend
from .scene import SceneServer


def run_host(duration: float = 0.0) -> None:
    """Own the shared MuJoCo scene: host the viewer and serve CLI commands."""
    backend = MujocoBackend(realtime=True)
    backend.connect()
    server = SceneServer(backend)
    server.start()
    try:
        backend.run_gui(duration, lock=server.lock)
    finally:
        server.stop()
        backend.disconnect()


def main() -> None:
    parser = argparse.ArgumentParser(description="Open the Piper MuJoCo viewer and serve the shared scene")
    parser.add_argument("--duration", type=float, default=0.0, help="Seconds; 0 keeps the window open")
    args = parser.parse_args()
    run_host(args.duration)


if __name__ == "__main__":
    main()
