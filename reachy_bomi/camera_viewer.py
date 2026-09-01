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

import cv2
from reachy2_sdk import ReachySDK

import safety
import stream

DEFAULT_ROBOT_IP = "130.251.6.85"
WINDOW_NAME_HEAD = "Reachy - Head Camera (LEFT)"
WINDOW_NAME_TORSO = "Reachy - Torso Camera (LEFT)"

# Screen position (top-left corner) -- kept in sync with reachy_control.MAP_WINDOW_POS
# so this window lands beside the cursor map instead of spawning on top of it.
WINDOW_POS = (660, 60)



def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("robot_ip", nargs="?", default=DEFAULT_ROBOT_IP,
                        help=f"IP address of the Reachy robot (default: {DEFAULT_ROBOT_IP})")
    parser.add_argument("--camera", choices=["teleop", "torso"], default="teleop",
                        help="Which camera to stream: 'teleop' (head) or 'torso' (chest/depth). "
                             "Default: teleop")
    cli_args = parser.parse_args()

    if cli_args.camera == "torso":
        window_name, cam_label = WINDOW_NAME_TORSO, "torso/depth"
    else:
        window_name, cam_label = WINDOW_NAME_HEAD, "head/teleop"

    reachy = ReachySDK(host=cli_args.robot_ip)
    camera = reachy.cameras.depth if cli_args.camera == "torso" else reachy.cameras.teleop
    if reachy.cameras is None or camera is None:
        print(f"[ERROR] No {cam_label} camera reported by the robot at '{cli_args.robot_ip}'")
        reachy.disconnect()
        sys.exit(1)

    cv2.namedWindow(window_name)
    cv2.moveWindow(window_name, *WINDOW_POS)

    try:
        stream.stream_blocking(camera, window_name, safety.quit_requested)
    finally:
        reachy.disconnect()


if __name__ == "__main__":
    main()
