"""Simulated forward-facing camera with depth sensing."""

import math
import time
from dataclasses import dataclass, field

from .drone import DroneState
from .environment import Environment


@dataclass
class CameraFrame:
    """A single camera frame with depth data across horizontal FOV."""
    depths: list[float]           # distance per ray sector
    obstacle_types: list[str]     # obstacle type per sector ("" if none)
    obstacle_heights: list[float] # height of detected obstacle per sector
    fov_h: float                  # horizontal FOV in degrees
    num_rays: int
    max_range: float
    drone_yaw: float
    drone_z: float
    timestamp: float = field(default_factory=time.time)

    def to_dict(self):
        return {
            "depths": [round(d, 1) for d in self.depths],
            "obstacle_types": self.obstacle_types,
            "obstacle_heights": [round(h, 1) for h in self.obstacle_heights],
            "fov_h": self.fov_h,
            "num_rays": self.num_rays,
            "max_range": self.max_range,
            "drone_yaw": round(self.drone_yaw, 1),
            "drone_z": round(self.drone_z, 1),
            "timestamp": self.timestamp,
        }


class SimulatedCamera:
    """Forward-facing camera that produces depth scans of the environment."""

    def __init__(self, fov_h: float = 70.0, num_rays: int = 48,
                 max_range: float = 500.0):
        self.fov_h = fov_h
        self.num_rays = num_rays
        self.max_range = max_range

    def capture(self, state: DroneState, environment: Environment) -> CameraFrame:
        """Capture a frame from the drone's current position and heading."""
        depths = []
        obstacle_types = []
        obstacle_heights = []

        yaw_rad = math.radians(state.yaw)
        half_fov = math.radians(self.fov_h / 2)

        for i in range(self.num_rays):
            frac = (i / max(self.num_rays - 1, 1)) - 0.5  # -0.5 to 0.5
            ray_angle = yaw_rad + frac * 2 * half_fov

            dist, obs = environment.ray_cast(
                state.x, state.y, state.z, ray_angle, self.max_range
            )

            depths.append(dist)
            if obs:
                obstacle_types.append(obs.obstacle_type.value)
                obstacle_heights.append(obs.height)
            else:
                obstacle_types.append("")
                obstacle_heights.append(0.0)

        return CameraFrame(
            depths=depths,
            obstacle_types=obstacle_types,
            obstacle_heights=obstacle_heights,
            fov_h=self.fov_h,
            num_rays=self.num_rays,
            max_range=self.max_range,
            drone_yaw=state.yaw,
            drone_z=state.z,
        )
