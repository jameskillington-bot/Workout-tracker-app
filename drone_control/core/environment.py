"""Simulated environment with obstacles for the drone to navigate."""

import math
from dataclasses import dataclass
from enum import Enum


class ObstacleType(Enum):
    BUILDING = "building"
    TREE = "tree"
    WALL = "wall"
    PILLAR = "pillar"


@dataclass
class Obstacle:
    """An obstacle in the environment."""
    x: float            # center x (cm)
    y: float            # center y (cm)
    z_base: float       # base altitude (cm), usually 0
    width: float        # extent in x (cm), or diameter for cylindrical
    depth: float        # extent in y (cm), ignored for cylindrical
    height: float       # vertical extent (cm)
    obstacle_type: ObstacleType = ObstacleType.BUILDING
    is_cylindrical: bool = False

    def to_dict(self):
        return {
            "x": self.x, "y": self.y, "z_base": self.z_base,
            "width": self.width, "depth": self.depth, "height": self.height,
            "type": self.obstacle_type.value,
            "is_cylindrical": self.is_cylindrical,
        }

    def overlaps_altitude(self, z: float, margin: float = 30.0) -> bool:
        """Check if altitude z is within the obstacle's vertical range."""
        return (z + margin) >= self.z_base and (z - margin) <= (self.z_base + self.height)

    def ray_intersect_2d(self, ox: float, oy: float, dx: float, dy: float,
                         max_range: float) -> float | None:
        """2D ray intersection. Returns distance or None.
        Ray origin (ox, oy), unit direction (dx, dy)."""
        if self.is_cylindrical:
            return self._ray_circle_intersect(ox, oy, dx, dy, max_range)
        return self._ray_aabb_intersect(ox, oy, dx, dy, max_range)

    def _ray_aabb_intersect(self, ox, oy, dx, dy, max_range):
        half_w = self.width / 2
        half_d = self.depth / 2
        x_min, x_max = self.x - half_w, self.x + half_w
        y_min, y_max = self.y - half_d, self.y + half_d

        EPS = 1e-9

        if abs(dx) < EPS:
            if ox < x_min or ox > x_max:
                return None
            t_min_x, t_max_x = -1e18, 1e18
        else:
            t1 = (x_min - ox) / dx
            t2 = (x_max - ox) / dx
            t_min_x, t_max_x = min(t1, t2), max(t1, t2)

        if abs(dy) < EPS:
            if oy < y_min or oy > y_max:
                return None
            t_min_y, t_max_y = -1e18, 1e18
        else:
            t1 = (y_min - oy) / dy
            t2 = (y_max - oy) / dy
            t_min_y, t_max_y = min(t1, t2), max(t1, t2)

        t_enter = max(t_min_x, t_min_y)
        t_exit = min(t_max_x, t_max_y)

        if t_enter > t_exit or t_exit < 0:
            return None

        t = t_enter if t_enter >= 0 else t_exit
        if t > max_range or t < 0:
            return None
        return t

    def _ray_circle_intersect(self, ox, oy, dx, dy, max_range):
        radius = self.width / 2
        fx = ox - self.x
        fy = oy - self.y

        a = dx * dx + dy * dy
        b = 2 * (fx * dx + fy * dy)
        c = fx * fx + fy * fy - radius * radius

        discriminant = b * b - 4 * a * c
        if discriminant < 0:
            return None

        sqrt_disc = math.sqrt(discriminant)
        t1 = (-b - sqrt_disc) / (2 * a)
        t2 = (-b + sqrt_disc) / (2 * a)

        t = t1 if t1 >= 0 else t2
        if t < 0 or t > max_range:
            return None
        return t


class Environment:
    """Simulated world with obstacles."""

    def __init__(self, obstacles: list[Obstacle] | None = None):
        self.obstacles = obstacles or []

    def add_obstacle(self, obstacle: Obstacle):
        self.obstacles.append(obstacle)

    def ray_cast(self, ox: float, oy: float, oz: float,
                 angle_rad: float, max_range: float = 500.0):
        """Cast a ray from (ox, oy) at angle_rad.
        Returns (distance, obstacle_or_None)."""
        dx = math.cos(angle_rad)
        dy = math.sin(angle_rad)

        nearest_dist = max_range
        nearest_obs = None

        for obs in self.obstacles:
            if not obs.overlaps_altitude(oz):
                continue
            dist = obs.ray_intersect_2d(ox, oy, dx, dy, max_range)
            if dist is not None and dist < nearest_dist:
                nearest_dist = dist
                nearest_obs = obs

        return nearest_dist, nearest_obs

    def check_collision(self, x: float, y: float, z: float,
                        radius: float = 20.0) -> bool:
        """Check if a sphere at (x,y,z) collides with any obstacle."""
        for obs in self.obstacles:
            if not obs.overlaps_altitude(z, margin=radius):
                continue
            if obs.is_cylindrical:
                dist = math.sqrt((x - obs.x) ** 2 + (y - obs.y) ** 2)
                if dist < (obs.width / 2 + radius):
                    return True
            else:
                half_w = obs.width / 2 + radius
                half_d = obs.depth / 2 + radius
                if abs(x - obs.x) < half_w and abs(y - obs.y) < half_d:
                    return True
        return False

    def get_obstacles_dict(self) -> list[dict]:
        return [obs.to_dict() for obs in self.obstacles]

    @staticmethod
    def default_environment():
        """Create a default environment with various obstacles."""
        env = Environment()

        # Buildings
        env.add_obstacle(Obstacle(180, 200, 0, 80, 80, 200, ObstacleType.BUILDING))
        env.add_obstacle(Obstacle(-150, 120, 0, 100, 60, 180, ObstacleType.BUILDING))
        env.add_obstacle(Obstacle(-200, -180, 0, 70, 90, 160, ObstacleType.BUILDING))
        env.add_obstacle(Obstacle(100, -200, 0, 60, 60, 140, ObstacleType.BUILDING))

        # Trees (cylindrical)
        env.add_obstacle(Obstacle(60, 280, 0, 50, 50, 250, ObstacleType.TREE, True))
        env.add_obstacle(Obstacle(-80, -100, 0, 40, 40, 200, ObstacleType.TREE, True))
        env.add_obstacle(Obstacle(250, 80, 0, 45, 45, 220, ObstacleType.TREE, True))
        env.add_obstacle(Obstacle(-250, 50, 0, 35, 35, 180, ObstacleType.TREE, True))

        # Walls
        env.add_obstacle(Obstacle(0, 380, 0, 300, 20, 150, ObstacleType.WALL))
        env.add_obstacle(Obstacle(300, 0, 0, 20, 250, 170, ObstacleType.WALL))

        # Pillars (cylindrical)
        env.add_obstacle(Obstacle(-30, 150, 0, 25, 25, 300, ObstacleType.PILLAR, True))
        env.add_obstacle(Obstacle(150, -80, 0, 20, 20, 280, ObstacleType.PILLAR, True))

        return env
