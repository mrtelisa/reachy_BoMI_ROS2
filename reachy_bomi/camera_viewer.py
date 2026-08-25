#!/usr/bin/env python3
"""Connect to Reachy2 and show a live feed from its head/teleop camera's
LEFT eye. Meant to run as its own OS process, spawned during Control so the 
camera's network round-trip never sits inside the main file cursor/velocity-timed loop.

Usage:
    python3 camera_viewer.py [robot_ip]
    Q, ESC, or closing the window = quit.
"""

import argparse
import sys

from reachy2_sdk import ReachySDK

import safety
import stream

DEFAULT_ROBOT_IP = "10.186.13.148"
WINDOW_NAME = "Reachy - Head Camera (LEFT)"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("robot_ip", nargs="?", default=DEFAULT_ROBOT_IP,
                        help=f"IP address of the Reachy robot (default: {DEFAULT_ROBOT_IP})")
    cli_args = parser.parse_args()

    reachy = ReachySDK(host=cli_args.robot_ip)
    if reachy.cameras is None or reachy.cameras.teleop is None:
        print(f"[ERROR] No head/teleop camera reported by the robot at '{cli_args.robot_ip}'")
        reachy.disconnect()
        sys.exit(1)

    try:
        stream.stream_blocking(reachy.cameras.teleop, WINDOW_NAME, safety.quit_requested)
    finally:
        reachy.disconnect()


if __name__ == "__main__":
    main()
