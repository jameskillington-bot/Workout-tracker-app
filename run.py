#!/usr/bin/env python3
"""Entry point for the Drone Control application.

Usage:
    python run.py                  # Simulator mode (no drone needed)
    python run.py --real           # Connect to a real DJI Tello drone
    python run.py --port 8080      # Custom port
"""

import argparse

from drone_control.web.server import create_app


def main():
    parser = argparse.ArgumentParser(description="Drone Control Dashboard")
    parser.add_argument(
        "--real", action="store_true",
        help="Connect to a real Tello drone instead of the simulator",
    )
    parser.add_argument(
        "--host", default="0.0.0.0",
        help="Host to bind the web server (default: 0.0.0.0)",
    )
    parser.add_argument(
        "--port", type=int, default=5000,
        help="Port for the web server (default: 5000)",
    )
    args = parser.parse_args()

    use_simulator = not args.real
    app = create_app(use_simulator=use_simulator)

    mode = "REAL TELLO" if args.real else "SIMULATOR"
    print(f"\n  Drone Control Dashboard")
    print(f"  Mode: {mode}")
    print(f"  Open http://localhost:{args.port} in your browser\n")

    app.run(host=args.host, port=args.port, debug=False, threaded=True)


if __name__ == "__main__":
    main()
