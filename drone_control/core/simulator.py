"""Simulated drone for testing without hardware."""

import math
import threading
import time

from .drone import Drone, DroneState


class SimulatedDrone(Drone):
    """A virtual drone that simulates physics and state."""

    MOVE_SPEED = 50  # cm/s default
    ASCEND_SPEED = 40  # cm/s
    ROTATE_SPEED = 90  # degrees/s

    def __init__(self):
        self._state = DroneState()
        self._speed = self.MOVE_SPEED
        self._lock = threading.Lock()
        self._rc_thread = None
        self._rc_active = False
        self._rc_values = [0, 0, 0, 0]
        self._flight_start = None
        self._log = []

    def _add_log(self, msg: str):
        entry = f"[{time.strftime('%H:%M:%S')}] {msg}"
        self._log.append(entry)
        if len(self._log) > 200:
            self._log = self._log[-200:]

    def get_log(self) -> list[str]:
        return list(self._log)

    def connect(self) -> bool:
        with self._lock:
            self._state.is_connected = True
            self._state.battery = 100
            self._state.temperature = 25.0
            self._add_log("Connected to simulated drone")
        return True

    def disconnect(self):
        self.emergency_stop()
        with self._lock:
            self._state.is_connected = False
            self._add_log("Disconnected")

    def takeoff(self) -> bool:
        with self._lock:
            if not self._state.is_connected or self._state.is_flying:
                return False
            self._state.is_flying = True
            self._state.z = 80.0  # hover at 80cm
            self._flight_start = time.time()
            self._add_log("Takeoff — hovering at 80cm")
        self._start_rc_loop()
        return True

    def land(self) -> bool:
        self._stop_rc_loop()
        with self._lock:
            if not self._state.is_flying:
                return False
            self._state.is_flying = False
            self._state.z = 0.0
            self._state.speed = 0.0
            if self._flight_start:
                self._state.flight_time += time.time() - self._flight_start
                self._flight_start = None
            self._add_log("Landed")
        return True

    def emergency_stop(self):
        self._stop_rc_loop()
        with self._lock:
            self._state.is_flying = False
            self._state.z = 0.0
            self._state.speed = 0.0
            if self._flight_start:
                self._state.flight_time += time.time() - self._flight_start
                self._flight_start = None
            self._add_log("EMERGENCY STOP")

    def move(self, direction: str, distance_cm: int) -> bool:
        with self._lock:
            if not self._state.is_flying:
                return False

        rad = math.radians(self._state.yaw)
        dx, dy, dz = 0.0, 0.0, 0.0

        if direction == "forward":
            dx = distance_cm * math.cos(rad)
            dy = distance_cm * math.sin(rad)
        elif direction == "back":
            dx = -distance_cm * math.cos(rad)
            dy = -distance_cm * math.sin(rad)
        elif direction == "left":
            dx = distance_cm * math.sin(rad)
            dy = -distance_cm * math.cos(rad)
        elif direction == "right":
            dx = -distance_cm * math.sin(rad)
            dy = distance_cm * math.cos(rad)
        elif direction == "up":
            dz = distance_cm
        elif direction == "down":
            dz = -distance_cm
        else:
            return False

        duration = distance_cm / self._speed
        time.sleep(min(duration, 2.0))  # cap simulated delay

        with self._lock:
            self._state.x += dx
            self._state.y += dy
            self._state.z = max(0, self._state.z + dz)
            self._drain_battery(duration)
            self._add_log(f"Move {direction} {distance_cm}cm")
        return True

    def rotate(self, degrees: int) -> bool:
        with self._lock:
            if not self._state.is_flying:
                return False
            self._state.yaw = (self._state.yaw + degrees) % 360
            self._add_log(f"Rotate {degrees}°  → yaw={self._state.yaw:.0f}°")
        duration = abs(degrees) / self.ROTATE_SPEED
        time.sleep(min(duration, 2.0))
        return True

    def set_speed(self, speed_cm_s: int) -> bool:
        self._speed = max(10, min(100, speed_cm_s))
        self._add_log(f"Speed set to {self._speed} cm/s")
        return True

    def send_rc(self, left_right: int, forward_back: int,
                up_down: int, yaw: int):
        self._rc_values = [
            max(-100, min(100, left_right)),
            max(-100, min(100, forward_back)),
            max(-100, min(100, up_down)),
            max(-100, min(100, yaw)),
        ]

    def get_state(self) -> DroneState:
        with self._lock:
            if self._flight_start and self._state.is_flying:
                self._state.flight_time_current = (
                    time.time() - self._flight_start
                )
            self._state.timestamp = time.time()
            return DroneState(**{
                k: getattr(self._state, k)
                for k in DroneState.__dataclass_fields__
            })

    def go_to(self, x: int, y: int, z: int, speed: int) -> bool:
        with self._lock:
            if not self._state.is_flying:
                return False
            sx, sy, sz = self._state.x, self._state.y, self._state.z

        dist = math.sqrt((x - sx) ** 2 + (y - sy) ** 2 + (z - sz) ** 2)
        if dist < 1:
            return True

        duration = dist / max(speed, 10)
        steps = max(int(duration * 10), 1)
        dt = duration / steps

        for i in range(1, steps + 1):
            frac = i / steps
            time.sleep(min(dt, 0.5))
            with self._lock:
                self._state.x = sx + (x - sx) * frac
                self._state.y = sy + (y - sy) * frac
                self._state.z = max(0, sz + (z - sz) * frac)
                self._state.speed = speed
                self._drain_battery(dt)

        self._add_log(f"Go to ({x}, {y}, {z}) at {speed} cm/s")
        return True

    # -- internal helpers --

    def _drain_battery(self, seconds: float):
        """Simulate battery drain (~0.5% per second of flight)."""
        self._state.battery = max(0, self._state.battery - seconds * 0.5)
        self._state.temperature = min(45, self._state.temperature + seconds * 0.1)

    def _start_rc_loop(self):
        if self._rc_active:
            return
        self._rc_active = True
        self._rc_thread = threading.Thread(target=self._rc_loop, daemon=True)
        self._rc_thread.start()

    def _stop_rc_loop(self):
        self._rc_active = False
        self._rc_values = [0, 0, 0, 0]

    def _rc_loop(self):
        """Apply RC stick inputs at ~20 Hz."""
        interval = 0.05
        while self._rc_active:
            lr, fb, ud, yaw_rate = self._rc_values
            with self._lock:
                if not self._state.is_flying:
                    break
                rad = math.radians(self._state.yaw)
                scale = self._speed * interval / 100.0

                # forward/back and left/right relative to yaw
                self._state.x += (fb * math.cos(rad) - lr * math.sin(rad)) * scale
                self._state.y += (fb * math.sin(rad) + lr * math.cos(rad)) * scale
                self._state.z = max(0, self._state.z + ud * scale)
                self._state.yaw = (self._state.yaw + yaw_rate * 0.9 * interval) % 360
                self._state.speed = math.sqrt(lr**2 + fb**2 + ud**2) * self._speed / 100
                self._drain_battery(interval)
            time.sleep(interval)
