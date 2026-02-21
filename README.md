# Drone Control App

A Python-based control and autopilot application for the **DJI Tello** drone, with a web dashboard for manual piloting and autonomous flight routines. Designed as an alternative to the Tello Go app with added autopilot capabilities.

## Features

- **Web Dashboard** — real-time telemetry, position map, and flight log
- **Manual Control** — on-screen D-pad, altitude/yaw buttons, and keyboard shortcuts (WASD + QE)
- **Autopilot** — pre-built flight routines (square, circle, figure-8, survey grid) and custom waypoint missions
- **Simulator Mode** — test everything without a physical drone
- **Tello SDK** — communicates directly with the Tello over its UDP command protocol (the same interface Tello Go uses)

## Quick Start

```bash
# Install dependencies
pip install -r requirements.txt

# Run in simulator mode (no drone needed)
python run.py

# Run with a real Tello drone
python run.py --real
```

Then open **http://localhost:5000** in your browser.

## Connecting to a Real Tello

1. Power on your Tello drone
2. Connect your computer to the Tello's Wi-Fi network (e.g., `TELLO-XXXXXX`)
3. Run `python run.py --real`
4. Click **Connect** in the dashboard

This uses the same UDP SDK (port 8889) that the official Tello Go app uses.

## Controls

| Key | Action |
|-----|--------|
| W/A/S/D | Forward / Left / Back / Right |
| Q / E | Rotate left / right |
| R / F | Up / Down |
| T | Takeoff |
| L | Land |
| Space | Emergency stop |

## Autopilot Routines

| Routine | Description |
|---------|-------------|
| Square | Fly a square pattern around the origin |
| Circle | Fly a circular path |
| Figure-8 | Fly a figure-eight pattern |
| Survey Grid | Lawn-mower grid for area coverage |

Load a routine from the sidebar, click **Start**, and the drone will fly the pattern autonomously. You can **Pause** or **Abort** at any time.

## Project Structure

```
drone_control/
  core/
    drone.py          # Abstract drone interface
    simulator.py      # Simulated drone (for testing)
    tello.py          # DJI Tello UDP driver
  autopilot/
    flight_plan.py    # Waypoints, routines, and autopilot executor
  web/
    server.py         # Flask API server
    static/
      index.html      # Dashboard UI
run.py                # Entry point
requirements.txt      # Python dependencies
```

## CLI Options

```
python run.py [--real] [--host HOST] [--port PORT]

  --real          Connect to a real Tello (default: simulator)
  --host HOST     Bind address (default: 0.0.0.0)
  --port PORT     Web server port (default: 5000)
```
