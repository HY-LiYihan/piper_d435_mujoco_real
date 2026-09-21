from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional
import time
import numpy as np


@dataclass(frozen=True)
class Pose:
    """Cartesian pose. Position is metres; quaternion is w, x, y, z."""

    position: tuple[float, float, float]
    quaternion: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)

    def __post_init__(self) -> None:
        if len(self.position) != 3 or len(self.quaternion) != 4:
            raise ValueError("position must have 3 and quaternion must have 4 values")
        norm = sum(x * x for x in self.quaternion) ** 0.5
        if norm < 1e-9:
            raise ValueError("quaternion must be non-zero")


@dataclass
class JointState:
    positions: np.ndarray
    velocities: np.ndarray = field(default_factory=lambda: np.zeros(6, dtype=float))
    gripper: float = 0.0
    timestamp: float = field(default_factory=time.time)

    def __post_init__(self) -> None:
        self.positions = np.asarray(self.positions, dtype=float).reshape(-1)
        self.velocities = np.asarray(self.velocities, dtype=float).reshape(-1)
        if self.positions.size != 6:
            raise ValueError("Piper requires six arm joint positions")
        if self.velocities.size != 6:
            raise ValueError("Piper requires six arm joint velocities")


@dataclass
class RobotState:
    connected: bool
    moving: bool
    joints: JointState
    pose: Optional[Pose] = None
    error: Optional[str] = None
    timestamp: float = field(default_factory=time.time)
