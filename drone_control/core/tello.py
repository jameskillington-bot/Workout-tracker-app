"""Driver for DJI Tello drone over UDP."""

import socket
import threading
import time

from .drone import Drone, DroneState


class TelloDrone(Drone):
    """Controls a real DJI Tello drone via its UDP SDK."""

    TELLO_IP = "192.168.10.1"
    CMD_PORT = 8889
    STATE_PORT = 8890
    TIMEOUT = 10  # seconds

    def __init__(self, local_ip: str = "0.0.0.0"):
        self._local_ip = local_ip
        self._cmd_socket = None
        self._state_socket = None
        self._state = DroneState()
        self._lock = threading.Lock()
        self._state_thread = None
        self._running = False

    def connect(self) -> bool:
        try:
            self._cmd_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._cmd_socket.bind((self._local_ip, self.CMD_PORT))
            self._cmd_socket.settimeout(self.TIMEOUT)

            self._state_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._state_socket.bind((self._local_ip, self.STATE_PORT))
            self._state_socket.settimeout(2)

            resp = self._send_command("command")
            if resp is None:
                return False

            self._running = True
            self._state_thread = threading.Thread(
                target=self._state_listener, daemon=True
            )
            self._state_thread.start()

            with self._lock:
                self._state.is_connected = True
            return True
        except OSError:
            return False

    def disconnect(self):
        self._running = False
        with self._lock:
            self._state.is_connected = False
        if self._cmd_socket:
            self._cmd_socket.close()
        if self._state_socket:
            self._state_socket.close()

    def takeoff(self) -> bool:
        resp = self._send_command("takeoff")
        if resp and "ok" in resp.lower():
            with self._lock:
                self._state.is_flying = True
            return True
        return False

    def land(self) -> bool:
        resp = self._send_command("land")
        if resp and "ok" in resp.lower():
            with self._lock:
                self._state.is_flying = False
            return True
        return False

    def emergency_stop(self):
        self._send_command("emergency")
        with self._lock:
            self._state.is_flying = False

    def move(self, direction: str, distance_cm: int) -> bool:
        valid = {"up", "down", "left", "right", "forward", "back"}
        if direction not in valid:
            return False
        distance_cm = max(20, min(500, distance_cm))
        resp = self._send_command(f"{direction} {distance_cm}")
        return resp is not None and "ok" in resp.lower()

    def rotate(self, degrees: int) -> bool:
        if degrees >= 0:
            resp = self._send_command(f"cw {abs(degrees)}")
        else:
            resp = self._send_command(f"ccw {abs(degrees)}")
        return resp is not None and "ok" in resp.lower()

    def set_speed(self, speed_cm_s: int) -> bool:
        speed_cm_s = max(10, min(100, speed_cm_s))
        resp = self._send_command(f"speed {speed_cm_s}")
        return resp is not None and "ok" in resp.lower()

    def send_rc(self, left_right: int, forward_back: int,
                up_down: int, yaw: int):
        cmd = f"rc {left_right} {forward_back} {up_down} {yaw}"
        self._send_command(cmd, wait_response=False)

    def get_state(self) -> DroneState:
        with self._lock:
            self._state.timestamp = time.time()
            return DroneState(**{
                k: getattr(self._state, k)
                for k in DroneState.__dataclass_fields__
            })

    def go_to(self, x: int, y: int, z: int, speed: int) -> bool:
        speed = max(10, min(100, speed))
        resp = self._send_command(f"go {x} {y} {z} {speed}")
        return resp is not None and "ok" in resp.lower()

    # -- internal --

    def _send_command(self, cmd: str, wait_response: bool = True) -> str | None:
        try:
            self._cmd_socket.sendto(
                cmd.encode("utf-8"),
                (self.TELLO_IP, self.CMD_PORT),
            )
            if not wait_response:
                return "ok"
            data, _ = self._cmd_socket.recvfrom(1024)
            return data.decode("utf-8", errors="replace")
        except (OSError, TimeoutError):
            return None

    def _state_listener(self):
        """Listen for state telemetry packets from the Tello."""
        while self._running:
            try:
                data, _ = self._state_socket.recvfrom(1024)
                self._parse_state(data.decode("utf-8", errors="replace"))
            except (OSError, TimeoutError):
                continue

    def _parse_state(self, raw: str):
        """Parse Tello state string like 'pitch:0;roll:0;yaw:0;...'"""
        fields = {}
        for pair in raw.strip().rstrip(";").split(";"):
            if ":" in pair:
                k, v = pair.split(":", 1)
                fields[k.strip()] = v.strip()

        with self._lock:
            if "bat" in fields:
                self._state.battery = int(fields["bat"])
            if "h" in fields:
                self._state.z = float(fields["h"])
            if "yaw" in fields:
                self._state.yaw = float(fields["yaw"]) % 360
            if "time" in fields:
                self._state.flight_time = float(fields["time"])
            if "temph" in fields:
                self._state.temperature = float(fields["temph"])
