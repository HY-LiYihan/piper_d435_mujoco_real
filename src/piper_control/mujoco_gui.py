from __future__ import annotations

import argparse
from pathlib import Path

from .backends.mujoco import MujocoBackend
from .scene import SceneServer


def run_host(duration: float = 0.0, scene: Path | None = None) -> None:
    """Own the shared MuJoCo scene: host the viewer and serve CLI commands."""
    backend = MujocoBackend(realtime=True, scene=scene)
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
    parser.add_argument("--scene", type=Path, help="MuJoCo scene XML containing piper_mount")
    args = parser.parse_args()
    run_host(args.duration, scene=args.scene)


if __name__ == "__main__":
    main()
