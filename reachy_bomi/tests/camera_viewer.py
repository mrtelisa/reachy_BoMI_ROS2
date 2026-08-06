#!/usr/bin/env python3
"""Connect to Reachy2 and show a live feed from one of its cameras --
nothing else, no robot motion. Meant to run as its own OS process, spawned
by reachy_control.py (see there) so a camera's network round-trip
(Camera.get_frame) never sits inside reachy_control.py's own loops.

Usage:
    python3 camera_viewer.py [robot_ip] [--camera head|torso]
    Q, ESC, or closing the window = quit.

    --camera head   Head/teleop camera, LEFT eye (default) -- used while
                     driving the base.
    --camera torso  Torso depth camera's RGB, LEFT view -- used as a
                     look-before-you-grasp checkpoint.
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from reachy2_sdk import ReachySDK

import safety
import stream

DEFAULT_ROBOT_IP = "192.168.0.104"
WINDOW_NAMES = {
    "head": "Reachy - Head Camera (LEFT)",
    "torso": "Reachy - Torso Camera (LEFT)",
}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("robot_ip", nargs="?", default=DEFAULT_ROBOT_IP,
                        help=f"IP address of the Reachy robot (default: {DEFAULT_ROBOT_IP})")
    parser.add_argument("--camera", choices=["head", "torso"], default="head",
                        help="Which camera to stream (default: head)")
    cli_args = parser.parse_args()

    reachy = ReachySDK(host=cli_args.robot_ip)
    if reachy.cameras is None:
        print(f"[ERROR] No camera service reported by the robot at '{cli_args.robot_ip}'")
        reachy.disconnect()
        sys.exit(1)

    camera = reachy.cameras.teleop if cli_args.camera == "head" else reachy.cameras.depth
    if camera is None:
        print(f"[ERROR] No {cli_args.camera} camera reported by the robot at '{cli_args.robot_ip}'")
        reachy.disconnect()
        sys.exit(1)

    try:
        stream.stream_blocking(camera, WINDOW_NAMES[cli_args.camera], safety.quit_requested)
    finally:
        reachy.disconnect()


if __name__ == "__main__":
    main()
