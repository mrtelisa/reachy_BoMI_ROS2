#!/usr/bin/env python3
"""Camera-streaming primitives shared by reachy_detection.py and
reachy_control.py -- parameterized (window name, camera view, quit check)
rather than hardcoding either caller's own window-naming/quit-key
conventions, so this module doesn't need to import either of them."""

from typing import Callable

import cv2
from reachy2_sdk.media.camera import CameraView, DepthCamera


def show_frame(depth_cam: DepthCamera, window_name: str, view: CameraView = CameraView.LEFT) -> None:
    """One non-blocking grab+show of a Reachy camera view. Meant to be
    called once per iteration of an already-running cv2 loop (which already
    pumps cv2.waitKey for its own windows) -- not run in its own loop."""
    result = depth_cam.get_frame(view=view)
    if result is not None:
        frame, _timestamp = result
        cv2.imshow(window_name, frame)


def stream_blocking(
    depth_cam: DepthCamera, window_name: str, quit_requested: Callable[[int, str], bool],
    view: CameraView = CameraView.LEFT,
) -> None:
    """Live feed in its own loop, no detection, until quit_requested(key,
    window_name) is True (e.g. Q/ESC pressed, or the window closed).
    quit_requested is injected so this module doesn't need to know the
    caller's own quit-key convention."""
    print("\n=== LIVE RGB STREAM (no detection) ===  Q = quit")
    while True:
        result = depth_cam.get_frame(view=view)
        if result is None:
            continue
        frame, _timestamp = result
        cv2.imshow(window_name, frame)

        key = cv2.waitKey(1) & 0xFF
        if quit_requested(key, window_name):
            break
