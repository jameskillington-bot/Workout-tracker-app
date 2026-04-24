"""Autonomous navigation with camera-based obstacle avoidance."""

import math
import threading
import time

from ..core.drone import Drone, DroneState
from ..core.camera import SimulatedCamera, CameraFrame
from ..core.environment import Environment


class AutonomousNavigator:
    """Navigates toward a destination while avoiding obstacles detected by the camera."""

    SAFETY_DISTANCE = 100.0   # cm — begin steering around obstacles
    CRITICAL_DISTANCE = 50.0  # cm — hard avoidance / reverse
    CRUISE_SPEED = 45         # RC forward value while cruising
    ARRIVAL_RADIUS = 35.0     # cm — "close enough" to destination
    NAV_HZ = 10               # navigation loop frequency

    def __init__(self, drone: Drone, environment: Environment,
                 camera: SimulatedCamera):
        self._drone = drone
        self._environment = environment
        self._camera = camera

        self._destination = None   # (x, y, z) or None
        self._active = False
        self._thread = None
        self._lock = threading.Lock()
        self._last_frame: CameraFrame | None = None
        self._status = "idle"
        self._avoidance_action = "none"
        self._reached = False

    # -- public API -----------------------------------------------------------

    @property
    def last_frame(self) -> CameraFrame | None:
        return self._last_frame

    @property
    def active(self) -> bool:
        return self._active

    def set_destination(self, x: float, y: float, z: float):
        with self._lock:
            self._destination = (x, y, z)
            self._reached = False
            self._status = "destination_set"

    def start(self):
        if self._active:
            return
        if not self._destination:
            raise RuntimeError("No destination set")
        self._active = True
        self._reached = False
        self._status = "navigating"
        self._thread = threading.Thread(target=self._nav_loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._active = False
        self._status = "stopped"
        self._drone.send_rc(0, 0, 0, 0)

    def get_status(self) -> dict:
        state = self._drone.get_state()
        dest = self._destination
        dist_to_goal = 0.0
        if dest:
            dist_to_goal = math.sqrt(
                (state.x - dest[0]) ** 2 +
                (state.y - dest[1]) ** 2 +
                (state.z - dest[2]) ** 2
            )
        return {
            "status": self._status,
            "active": self._active,
            "destination": (
                {"x": dest[0], "y": dest[1], "z": dest[2]} if dest else None
            ),
            "distance_to_goal": round(dist_to_goal, 1),
            "avoidance_action": self._avoidance_action,
            "reached": self._reached,
        }

    # -- navigation loop ------------------------------------------------------

    def _nav_loop(self):
        interval = 1.0 / self.NAV_HZ

        while self._active:
            state = self._drone.get_state()

            if not state.is_flying:
                time.sleep(interval)
                continue

            # Capture camera frame
            frame = self._camera.capture(state, self._environment)
            self._last_frame = frame

            dest = self._destination
            if not dest:
                time.sleep(interval)
                continue

            # Check arrival
            dist_to_goal = math.sqrt(
                (state.x - dest[0]) ** 2 +
                (state.y - dest[1]) ** 2 +
                (state.z - dest[2]) ** 2
            )
            if dist_to_goal < self.ARRIVAL_RADIUS:
                self._drone.send_rc(0, 0, 0, 0)
                self._status = "reached"
                self._reached = True
                self._avoidance_action = "none"
                self._active = False
                break

            # Desired heading to goal
            goal_bearing = math.degrees(
                math.atan2(dest[1] - state.y, dest[0] - state.x)
            ) % 360

            # Compute avoidance-aware RC commands
            yaw_cmd, fwd_cmd, alt_cmd = self._compute_commands(
                frame, state, goal_bearing, dest[2]
            )
            self._drone.send_rc(0, fwd_cmd, alt_cmd, yaw_cmd)
            time.sleep(interval)

        # Ensure motors idle when loop exits
        self._drone.send_rc(0, 0, 0, 0)

    # -- obstacle avoidance logic ---------------------------------------------

    def _compute_commands(self, frame: CameraFrame, state: DroneState,
                          goal_bearing: float, goal_alt: float):
        """Return (yaw_cmd, fwd_cmd, alt_cmd) RC values."""

        depths = frame.depths
        n = frame.num_rays

        # Split rays into 5 equal zones: far-left, left, center, right, far-right
        zone_size = n // 5
        zones = []
        for z in range(5):
            start = z * zone_size
            end = start + zone_size if z < 4 else n
            zone_depths = depths[start:end]
            zones.append(min(zone_depths))

        center_clear = zones[2]
        left_clear = min(zones[0], zones[1])
        right_clear = min(zones[3], zones[4])
        best_left = max(zones[0], zones[1])
        best_right = max(zones[3], zones[4])

        # Heading error (signed, -180..180)
        heading_error = (goal_bearing - state.yaw + 180) % 360 - 180

        # Altitude correction toward goal altitude
        alt_error = goal_alt - state.z
        alt_cmd = int(max(-30, min(30, alt_error * 0.5)))

        # Default: steer toward goal at cruise speed
        yaw_cmd = int(max(-60, min(60, heading_error * 0.8)))
        fwd_cmd = self.CRUISE_SPEED

        if center_clear < self.CRITICAL_DISTANCE:
            # --- CRITICAL: obstacle very close — back up and turn hard ---
            fwd_cmd = -20
            self._avoidance_action = "critical"
            if best_left > best_right:
                yaw_cmd = -70
            else:
                yaw_cmd = 70

        elif center_clear < self.SAFETY_DISTANCE:
            # --- CAUTION: obstacle ahead — slow down and steer around ---
            ratio = center_clear / self.SAFETY_DISTANCE
            fwd_cmd = max(10, int(self.CRUISE_SPEED * ratio))
            self._avoidance_action = "avoiding"

            # Pick the side that is both clearer AND closer to the goal
            if heading_error < 0:  # goal is to the left
                if best_left > self.CRITICAL_DISTANCE:
                    yaw_cmd = -50
                elif best_right > self.CRITICAL_DISTANCE:
                    yaw_cmd = 50
                else:
                    yaw_cmd = -50 if best_left >= best_right else 50
            else:  # goal is to the right
                if best_right > self.CRITICAL_DISTANCE:
                    yaw_cmd = 50
                elif best_left > self.CRITICAL_DISTANCE:
                    yaw_cmd = -50
                else:
                    yaw_cmd = 50 if best_right >= best_left else -50

        else:
            # --- CLEAR: cruise toward goal ---
            self._avoidance_action = "clear"

            # Gentle nudge away from peripheral obstacles
            if zones[0] < self.SAFETY_DISTANCE * 0.6:
                yaw_cmd = max(yaw_cmd, 15)
            if zones[4] < self.SAFETY_DISTANCE * 0.6:
                yaw_cmd = min(yaw_cmd, -15)

        return yaw_cmd, fwd_cmd, alt_cmd
