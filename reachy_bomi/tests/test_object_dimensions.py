#!/usr/bin/env python3
"""
Diagnostic tool for the camera/depth size estimation, independent of the
BoMI/teleop pipeline: the robot's arms and mobile base are never powered on
(reachy.turn_on() is never called) -- only the depth camera is used, so
Reachy just sits still the whole time.

Live YOLO detection re-runs every frame (unlike the normal object-selection
UI, which freezes on one captured snapshot), so the feed keeps updating.
Hover the mouse over any detected box for HOVER_HOLD_SECONDS and its class
name plus estimated width/height (via the same point-cloud pipeline used
for grasping: bbox crop + 10-frame depth fusion) print to the terminal.
The stream then just keeps going -- select as many objects as you want,
one after another, no confirm dialog.

Dependencies:
    pip install reachy2-sdk opencv-python numpy ultralytics matplotlib

Usage:
    python3 test_object_dimensions.py [robot_ip]
    Options:
        --yolo-model PATH    YOLOv8 weights (.pt). Default: yolov8n.pt
        --conf FLOAT         Minimum YOLO detection confidence. Default: 0.5

    Q, ESC, or closing the window with the X = quit.
"""

import argparse
import os
import sys
import time
from typing import Tuple

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from reachy2_sdk import ReachySDK
from ultralytics import YOLO

import bomi_detection as grasp
import reachy_grasp

DEFAULT_ROBOT_IP = "192.168.0.121"


def _box_eccentricity_deg(K, box) -> Tuple[float, float]:
    """Angular offset of the box center from the optical axis (0,0 = dead
    center), horizontal/vertical. Lets us correlate a size error with how
    far off to the side/top/bottom the object was, instead of guessing from
    the video."""
    x1, y1, x2, y2 = box
    u, v = (x1 + x2) / 2.0, (y1 + y2) / 2.0
    fx, cx = K[0, 0], K[0, 2]
    fy, cy = K[1, 1], K[1, 2]
    return float(np.degrees(np.arctan2(u - cx, fx))), float(np.degrees(np.arctan2(v - cy, fy)))


def _measure_and_print(depth_cam, class_name: str, box) -> None:
    """Runs the same pipeline used for grasping (bbox crop + 10-frame median
    fusion -> point cloud for position, PCA-based width/height), with its 4
    diagnostic point-cloud plots (show=True) -- non-blocking, so they don't
    interrupt the live stream."""
    result = grasp._build_object_point_cloud(depth_cam, class_name, box, show=False)
    if result is None:
        print(f"[{class_name}] no valid depth points, cannot estimate size")
        return
    point_cloud, width_m, height_m = result
    if point_cloud.shape[0] == 0:
        print(f"[{class_name}] no valid depth points, cannot estimate size")
        return

    _, shape = reachy_grasp.point_cloud_to_grasp_input(point_cloud, class_name)
    print(f"[{class_name}]  shape={shape:<8}  width={width_m * 100:5.1f} cm  "
          f"height={height_m * 100:5.1f} cm  ({len(point_cloud)} points)")

    params = depth_cam.get_parameters(view=grasp.CameraView.LEFT)
    if params is not None:
        _, _, _, _, K, _, _ = params
        ecc_x_deg, ecc_y_deg = _box_eccentricity_deg(K, box)
        print(f"           off-axis: {ecc_x_deg:+5.1f} deg horiz, {ecc_y_deg:+5.1f} deg vert")


def _stream_and_measure(depth_cam, model: YOLO, confidence: float) -> None:
    mouse = grasp._MouseTracker(grasp.CAM_WINDOW_NAME)
    hovered_box = None
    hover_start = None

    print(f"\n=== LIVE DIMENSION CHECK ===  Q = quit  |  hover a box for "
          f"{grasp.HOVER_HOLD_SECONDS:.0f}s to print its estimated size")

    while True:
        result = depth_cam.get_frame(view=grasp.CameraView.LEFT)
        if result is None:
            continue
        frame, _timestamp = result

        detections = grasp._detect_graspable_objects(model, frame, confidence)
        hovered = grasp._find_hovered_detection(detections, mouse.x, mouse.y)
        now = time.time()

        if hovered is None:
            hovered_box, hover_start = None, None
        else:
            box = hovered[2]
            if hovered_box is None or grasp._iou(box, hovered_box) < grasp.HOVER_IOU_MATCH:
                hover_start = now
            hovered_box = box
        hover_duration = (now - hover_start) if hover_start is not None else 0.0
        is_held = hovered is not None and hover_duration >= grasp.HOVER_HOLD_SECONDS

        hover_progress = min(hover_duration / grasp.HOVER_HOLD_SECONDS, 1.0) if hovered is not None else 0.0
        for class_name, conf, box in detections:
            is_hovered = hovered is not None and box == hovered[2]
            color = grasp.COLOR_GREEN if (is_hovered and is_held) else (
                grasp.COLOR_YELLOW if is_hovered else grasp.COLOR_BLUE)
            grasp._draw_box(frame, box, f"{class_name} {conf:.2f}", color, hover_progress if is_hovered else 0.0)
        cv2.imshow(grasp.CAM_WINDOW_NAME, frame)

        if is_held:
            class_name, _conf, box = hovered
            _measure_and_print(depth_cam, class_name, box)
            hovered_box, hover_start = None, None  # needs a fresh hold to fire again

        key = cv2.waitKey(1) & 0xFF
        if grasp._quit_requested(key, grasp.CAM_WINDOW_NAME):
            break

    cv2.destroyWindow(grasp.CAM_WINDOW_NAME)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Live camera/depth size-estimation check. The robot never moves."
    )
    parser.add_argument("robot_ip", nargs="?", default=DEFAULT_ROBOT_IP,
                        help=f"IP address of the Reachy robot (default: {DEFAULT_ROBOT_IP})")
    parser.add_argument("--yolo-model", default=grasp.YOLO_MODEL_PATH,
                        help=f"Path to YOLOv8 weights (default: {grasp.YOLO_MODEL_PATH})")
    parser.add_argument("--conf", type=float, default=grasp.YOLO_CONFIDENCE,
                        help=f"Minimum detection confidence (default: {grasp.YOLO_CONFIDENCE})")
    cli_args = parser.parse_args()

    reachy = ReachySDK(host=cli_args.robot_ip)
    if reachy.cameras is None or reachy.cameras.depth is None:
        print(f"[ERROR] No depth camera reported by the robot at '{cli_args.robot_ip}'")
        reachy.disconnect()
        sys.exit(1)
    depth_cam = reachy.cameras.depth

    print(f"Loading YOLO model '{cli_args.yolo_model}'...")
    model = YOLO(cli_args.yolo_model)

    try:
        _stream_and_measure(depth_cam, model, cli_args.conf)
    finally:
        cv2.destroyAllWindows()
        reachy.disconnect()


if __name__ == "__main__":
    main()
