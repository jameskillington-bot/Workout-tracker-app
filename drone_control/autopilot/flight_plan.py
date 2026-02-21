"""Autonomous flight planning — waypoints and pre-programmed routines."""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass, field
from enum import Enum

from ..core.drone import Drone


class WaypointStatus(Enum):
    PENDING = "pending"
    ACTIVE = "active"
    REACHED = "reached"
    SKIPPED = "skipped"


@dataclass
class Waypoint:
    x: int  # cm
    y: int  # cm
    z: int  # cm (altitude)
    speed: int = 50  # cm/s
    hover_time: float = 0.0  # seconds to hover after reaching
    action: str | None = None  # optional action name at waypoint
    status: WaypointStatus = WaypointStatus.PENDING

    def to_dict(self):
        return {
            "x": self.x, "y": self.y, "z": self.z,
            "speed": self.speed, "hover_time": self.hover_time,
            "action": self.action, "status": self.status.value,
        }


class FlightPlanStatus(Enum):
    IDLE = "idle"
    RUNNING = "running"
    PAUSED = "paused"
    COMPLETED = "completed"
    ABORTED = "aborted"


@dataclass
class FlightPlan:
    """A sequence of waypoints the drone will fly through."""
    name: str
    waypoints: list[Waypoint] = field(default_factory=list)
    loop: bool = False
    status: FlightPlanStatus = FlightPlanStatus.IDLE
    current_index: int = 0

    def add_waypoint(self, x: int, y: int, z: int, speed: int = 50,
                     hover_time: float = 0.0, action: str | None = None):
        self.waypoints.append(Waypoint(x, y, z, speed, hover_time, action))

    def to_dict(self):
        return {
            "name": self.name,
            "status": self.status.value,
            "current_index": self.current_index,
            "loop": self.loop,
            "waypoints": [w.to_dict() for w in self.waypoints],
        }


# -- Pre-built routines ---------------------------------------------------

def square_routine(size_cm: int = 200, altitude: int = 100,
                   speed: int = 40) -> FlightPlan:
    """Fly a square pattern."""
    plan = FlightPlan(name="Square")
    half = size_cm // 2
    corners = [
        (half, half, altitude),
        (-half, half, altitude),
        (-half, -half, altitude),
        (half, -half, altitude),
    ]
    for x, y, z in corners:
        plan.add_waypoint(x, y, z, speed=speed, hover_time=1.0)
    return plan


def circle_routine(radius_cm: int = 150, altitude: int = 100,
                   points: int = 12, speed: int = 30) -> FlightPlan:
    """Fly a circular pattern."""
    plan = FlightPlan(name="Circle")
    for i in range(points):
        angle = 2 * math.pi * i / points
        x = int(radius_cm * math.cos(angle))
        y = int(radius_cm * math.sin(angle))
        plan.add_waypoint(x, y, altitude, speed=speed)
    return plan


def figure_eight_routine(radius_cm: int = 100, altitude: int = 100,
                         points: int = 16, speed: int = 30) -> FlightPlan:
    """Fly a figure-eight pattern."""
    plan = FlightPlan(name="Figure-8")
    for i in range(points):
        t = 2 * math.pi * i / points
        x = int(radius_cm * math.sin(t))
        y = int(radius_cm * math.sin(t) * math.cos(t))
        plan.add_waypoint(x, y, altitude, speed=speed)
    return plan


def survey_grid_routine(width_cm: int = 300, height_cm: int = 300,
                        spacing_cm: int = 100, altitude: int = 120,
                        speed: int = 35) -> FlightPlan:
    """Fly a lawn-mower survey grid pattern."""
    plan = FlightPlan(name="Survey Grid")
    rows = height_cm // spacing_cm + 1
    left_x = -width_cm // 2
    right_x = width_cm // 2
    start_y = -height_cm // 2

    for row in range(rows):
        y = start_y + row * spacing_cm
        if row % 2 == 0:
            plan.add_waypoint(left_x, y, altitude, speed=speed)
            plan.add_waypoint(right_x, y, altitude, speed=speed)
        else:
            plan.add_waypoint(right_x, y, altitude, speed=speed)
            plan.add_waypoint(left_x, y, altitude, speed=speed)
    return plan


BUILTIN_ROUTINES = {
    "square": square_routine,
    "circle": circle_routine,
    "figure_eight": figure_eight_routine,
    "survey_grid": survey_grid_routine,
}


# -- Autopilot executor ---------------------------------------------------

class Autopilot:
    """Executes a flight plan on a drone in a background thread."""

    def __init__(self, drone: Drone):
        self._drone = drone
        self._plan: FlightPlan | None = None
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()

    @property
    def plan(self) -> FlightPlan | None:
        return self._plan

    def load_plan(self, plan: FlightPlan):
        if self._plan and self._plan.status == FlightPlanStatus.RUNNING:
            raise RuntimeError("A flight plan is already running — abort it first")
        self._plan = plan
        self._plan.status = FlightPlanStatus.IDLE
        self._plan.current_index = 0
        for wp in self._plan.waypoints:
            wp.status = WaypointStatus.PENDING

    def start(self):
        if not self._plan:
            raise RuntimeError("No flight plan loaded")
        if self._plan.status == FlightPlanStatus.RUNNING:
            return

        self._stop_event.clear()
        self._plan.status = FlightPlanStatus.RUNNING
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def pause(self):
        if self._plan and self._plan.status == FlightPlanStatus.RUNNING:
            self._plan.status = FlightPlanStatus.PAUSED

    def resume(self):
        if self._plan and self._plan.status == FlightPlanStatus.PAUSED:
            self._plan.status = FlightPlanStatus.RUNNING

    def abort(self):
        if self._plan:
            self._plan.status = FlightPlanStatus.ABORTED
        self._stop_event.set()

    def _run(self):
        plan = self._plan
        while True:
            if self._stop_event.is_set():
                break

            if plan.status == FlightPlanStatus.PAUSED:
                time.sleep(0.2)
                continue

            if plan.status != FlightPlanStatus.RUNNING:
                break

            idx = plan.current_index
            if idx >= len(plan.waypoints):
                if plan.loop:
                    plan.current_index = 0
                    for wp in plan.waypoints:
                        wp.status = WaypointStatus.PENDING
                    continue
                else:
                    plan.status = FlightPlanStatus.COMPLETED
                    break

            wp = plan.waypoints[idx]
            wp.status = WaypointStatus.ACTIVE

            success = self._drone.go_to(wp.x, wp.y, wp.z, wp.speed)

            if self._stop_event.is_set():
                break

            if success:
                wp.status = WaypointStatus.REACHED
                if wp.hover_time > 0:
                    time.sleep(wp.hover_time)
            else:
                wp.status = WaypointStatus.SKIPPED

            plan.current_index = idx + 1
