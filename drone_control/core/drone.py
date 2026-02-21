"""Abstract drone interface defining the control API."""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
import time


@dataclass
class DroneState:
    """Current state of the drone."""
    x: float = 0.0          # cm from origin
    y: float = 0.0          # cm from origin
    z: float = 0.0          # cm (altitude)
    yaw: float = 0.0        # degrees (0-360)
    speed: float = 0.0      # cm/s
    battery: int = 100       # percent
    is_flying: bool = False
    is_connected: bool = False
    flight_time: float = 0.0  # seconds
    temperature: float = 25.0  # celsius
    timestamp: float = field(default_factory=time.time)

    def to_dict(self):
        return {
            "x": round(self.x, 1),
            "y": round(self.y, 1),
            "z": round(self.z, 1),
            "yaw": round(self.yaw, 1),
            "speed": round(self.speed, 1),
            "battery": self.battery,
            "is_flying": self.is_flying,
            "is_connected": self.is_connected,
            "flight_time": round(self.flight_time, 1),
            "temperature": round(self.temperature, 1),
        }


class Drone(ABC):
    """Abstract base class for all drone drivers."""

    @abstractmethod
    def connect(self) -> bool:
        """Connect to the drone. Returns True on success."""

    @abstractmethod
    def disconnect(self):
        """Disconnect from the drone."""

    @abstractmethod
    def takeoff(self) -> bool:
        """Command the drone to take off. Returns True on success."""

    @abstractmethod
    def land(self) -> bool:
        """Command the drone to land. Returns True on success."""

    @abstractmethod
    def emergency_stop(self):
        """Immediately stop all motors."""

    @abstractmethod
    def move(self, direction: str, distance_cm: int) -> bool:
        """Move in a direction ('up','down','left','right','forward','back')."""

    @abstractmethod
    def rotate(self, degrees: int) -> bool:
        """Rotate clockwise by degrees (negative for counter-clockwise)."""

    @abstractmethod
    def set_speed(self, speed_cm_s: int) -> bool:
        """Set movement speed in cm/s."""

    @abstractmethod
    def send_rc(self, left_right: int, forward_back: int,
                up_down: int, yaw: int):
        """Send RC-style stick values (-100 to 100 each)."""

    @abstractmethod
    def get_state(self) -> DroneState:
        """Return current drone state."""

    @abstractmethod
    def go_to(self, x: int, y: int, z: int, speed: int) -> bool:
        """Fly to coordinates (x, y, z) at given speed."""
