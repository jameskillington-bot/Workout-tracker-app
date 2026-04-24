"""Flask web server bridging the frontend dashboard to the drone backend."""

from __future__ import annotations

import json
import os
from pathlib import Path

from flask import Flask, jsonify, request, send_from_directory

from ..core.drone import Drone
from ..core.simulator import SimulatedDrone
from ..core.tello import TelloDrone
from ..core.environment import Environment
from ..core.camera import SimulatedCamera
from ..autopilot.flight_plan import (
    Autopilot, FlightPlan, BUILTIN_ROUTINES,
)
from ..autopilot.obstacle_avoidance import AutonomousNavigator

STATIC_DIR = Path(__file__).parent / "static"

app = Flask(__name__, static_folder=str(STATIC_DIR))

# -- Global state (initialised in create_app) -----------------------------
drone: Drone | None = None
autopilot: Autopilot | None = None
environment: Environment | None = None
camera: SimulatedCamera | None = None
navigator: AutonomousNavigator | None = None


def create_app(use_simulator: bool = True) -> Flask:
    global drone, autopilot, environment, camera, navigator

    if use_simulator:
        drone = SimulatedDrone()
    else:
        drone = TelloDrone()

    autopilot = Autopilot(drone)

    # Simulated environment + camera for obstacle avoidance
    environment = Environment.default_environment()
    camera = SimulatedCamera(fov_h=70.0, num_rays=48, max_range=500.0)
    navigator = AutonomousNavigator(drone, environment, camera)

    return app


# -- Static files ---------------------------------------------------------

@app.route("/")
def index():
    return send_from_directory(str(STATIC_DIR), "index.html")


@app.route("/static/<path:filename>")
def static_files(filename):
    return send_from_directory(str(STATIC_DIR), filename)


# -- Connection -----------------------------------------------------------

@app.route("/api/connect", methods=["POST"])
def api_connect():
    ok = drone.connect()
    return jsonify({"success": ok})


@app.route("/api/disconnect", methods=["POST"])
def api_disconnect():
    drone.disconnect()
    return jsonify({"success": True})


# -- Basic flight commands ------------------------------------------------

@app.route("/api/takeoff", methods=["POST"])
def api_takeoff():
    ok = drone.takeoff()
    return jsonify({"success": ok})


@app.route("/api/land", methods=["POST"])
def api_land():
    ok = drone.land()
    return jsonify({"success": ok})


@app.route("/api/emergency", methods=["POST"])
def api_emergency():
    drone.emergency_stop()
    return jsonify({"success": True})


@app.route("/api/move", methods=["POST"])
def api_move():
    data = request.get_json(force=True)
    direction = data.get("direction", "")
    distance = int(data.get("distance", 50))
    ok = drone.move(direction, distance)
    return jsonify({"success": ok})


@app.route("/api/rotate", methods=["POST"])
def api_rotate():
    data = request.get_json(force=True)
    degrees = int(data.get("degrees", 90))
    ok = drone.rotate(degrees)
    return jsonify({"success": ok})


@app.route("/api/speed", methods=["POST"])
def api_speed():
    data = request.get_json(force=True)
    speed = int(data.get("speed", 50))
    ok = drone.set_speed(speed)
    return jsonify({"success": ok})


@app.route("/api/rc", methods=["POST"])
def api_rc():
    data = request.get_json(force=True)
    drone.send_rc(
        int(data.get("left_right", 0)),
        int(data.get("forward_back", 0)),
        int(data.get("up_down", 0)),
        int(data.get("yaw", 0)),
    )
    return jsonify({"success": True})


# -- Telemetry -----------------------------------------------------------

@app.route("/api/state")
def api_state():
    state = drone.get_state()
    result = state.to_dict()
    if hasattr(drone, "get_log"):
        result["log"] = drone.get_log()[-30:]
    return jsonify(result)


# -- Autopilot -----------------------------------------------------------

@app.route("/api/routines")
def api_routines():
    return jsonify({"routines": list(BUILTIN_ROUTINES.keys())})


@app.route("/api/autopilot/load", methods=["POST"])
def api_autopilot_load():
    data = request.get_json(force=True)
    name = data.get("routine", "square")
    params = data.get("params", {})

    if name in BUILTIN_ROUTINES:
        plan = BUILTIN_ROUTINES[name](**params)
    else:
        # Custom waypoint list
        plan = FlightPlan(name=name)
        for wp in data.get("waypoints", []):
            plan.add_waypoint(
                int(wp["x"]), int(wp["y"]), int(wp["z"]),
                speed=int(wp.get("speed", 50)),
                hover_time=float(wp.get("hover_time", 0)),
            )

    loop = data.get("loop", False)
    plan.loop = loop
    autopilot.load_plan(plan)
    return jsonify({"success": True, "plan": plan.to_dict()})


@app.route("/api/autopilot/start", methods=["POST"])
def api_autopilot_start():
    try:
        autopilot.start()
        return jsonify({"success": True})
    except RuntimeError as e:
        return jsonify({"success": False, "error": str(e)}), 400


@app.route("/api/autopilot/pause", methods=["POST"])
def api_autopilot_pause():
    autopilot.pause()
    return jsonify({"success": True})


@app.route("/api/autopilot/resume", methods=["POST"])
def api_autopilot_resume():
    autopilot.resume()
    return jsonify({"success": True})


@app.route("/api/autopilot/abort", methods=["POST"])
def api_autopilot_abort():
    autopilot.abort()
    return jsonify({"success": True})


@app.route("/api/autopilot/status")
def api_autopilot_status():
    plan = autopilot.plan
    if plan:
        return jsonify(plan.to_dict())
    return jsonify({"status": "idle", "waypoints": []})


# -- Environment ----------------------------------------------------------

@app.route("/api/environment")
def api_environment():
    return jsonify({"obstacles": environment.get_obstacles_dict()})


# -- Camera ---------------------------------------------------------------

@app.route("/api/camera")
def api_camera():
    """Return the latest camera depth frame."""
    frame = navigator.last_frame
    if frame:
        return jsonify(frame.to_dict())
    # Capture a fresh frame if none yet
    state = drone.get_state()
    fresh = camera.capture(state, environment)
    return jsonify(fresh.to_dict())


# -- Autonomous navigation -----------------------------------------------

@app.route("/api/autonomous/destination", methods=["POST"])
def api_autonomous_destination():
    data = request.get_json(force=True)
    x = float(data.get("x", 0))
    y = float(data.get("y", 0))
    z = float(data.get("z", 80))
    navigator.set_destination(x, y, z)
    return jsonify({"success": True, "destination": {"x": x, "y": y, "z": z}})


@app.route("/api/autonomous/start", methods=["POST"])
def api_autonomous_start():
    try:
        navigator.start()
        return jsonify({"success": True})
    except RuntimeError as e:
        return jsonify({"success": False, "error": str(e)}), 400


@app.route("/api/autonomous/stop", methods=["POST"])
def api_autonomous_stop():
    navigator.stop()
    return jsonify({"success": True})


@app.route("/api/autonomous/status")
def api_autonomous_status():
    return jsonify(navigator.get_status())
