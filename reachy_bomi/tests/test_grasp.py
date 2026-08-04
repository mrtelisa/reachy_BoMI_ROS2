#!/usr/bin/env python3
"""
Test script for the reachability-filtered object selection + grasp planning
pipeline -- with NO arm motion.

Connects to Reachy, captures a frame, and filters YOLO's detections down to
those reachy_grasp.is_position_reachable finds reachable by some arm,
printing each detection's keep/drop verdict and why -- that filter is what
this script exists to exercise (reachy_detection._capture_and_detect already
applies it silently; this inlines the same steps with logging). From there
it's the same hover-select/confirm flow as reachy_detection.py; once an
object is confirmed, its point cloud is built and reachy_grasp.plan_grasp
computes pre-grasp/grasp/lift poses, plotted together with the point cloud
via reachy_grasp.show_grasp_plan.

The robot itself never moves: reachy_grasp.execute_grasp's motion is already
commented out, and this script doesn't even call it.

Usage:
    python3 test_grasp.py [robot_ip]

    <robot_ip> is optional; if omitted, reachy_detection.DEFAULT_ROBOT_IP is used.

    Options:
        --yolo-model PATH   YOLOv8 weights (.pt). Default: yolov8n.pt
        --conf FLOAT        Minimum detection confidence. Default: 0.5

    Q, ESC, or closing a window with the X = quit / go back, same as
    reachy_detection.py.
"""

import argparse
import os
import sys
import elbow_135 as start_posotion

import cv2

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from reachy2_sdk import ReachySDK
from reachy2_sdk.media.camera import CameraView
from ultralytics import YOLO

import reachy_detection
import reachy_grasp


def _capture_and_filter(depth_cam, model: YOLO, confidence: float, reachy: ReachySDK):
    """Same job as reachy_detection._capture_and_detect, but prints each
    detection's reachability verdict instead of filtering silently -- the
    thing this script exists to let you check."""
    result = depth_cam.get_frame(view=CameraView.LEFT)
    if result is None:
        print("[ERROR] Could not capture a frame from the camera")
        return None
    base_frame, _timestamp = result

    depth_result = depth_cam.get_depth_frame(view=CameraView.DEPTH)
    depth_frame = depth_result[0] if depth_result is not None else None

    raw_detections = reachy_detection._detect_graspable_objects(model, base_frame, confidence)
    if depth_frame is None:
        print("[WARN] No depth frame -- can't verify reachability, 0 detections kept")
        return base_frame, [], {}

    kept = []
    for class_name, conf, box in raw_detections:
        x1, y1, x2, y2 = box
        u, v = (x1 + x2) // 2, (y1 + y2) // 2
        position = reachy_detection._estimate_object_position(depth_cam, depth_frame, u, v)
        if position is None:
            print(f"  [drop] {class_name}: no valid depth at box center")
            continue
        xyz = f"xyz=({position[0]:.2f},{position[1]:.2f},{position[2]:.2f})m"
        if reachy_grasp.is_position_reachable(reachy, position):
            print(f"  [keep] {class_name} {xyz} -- within reach")
            kept.append((class_name, conf, box))
        else:
            print(f"  [drop] {class_name} {xyz} -- out of reach")

    print(f"YOLO found {len(raw_detections)} candidate object(s), {len(kept)} within Reachy's workspace")

    labels = {
        box: reachy_detection._label_for_detection(class_name, conf, box, depth_cam, depth_frame)
        for class_name, conf, box in kept
    }
    return base_frame, kept, labels


def _test_grasp_planning(reachy: ReachySDK, model: YOLO, confidence: float) -> None:
    depth_cam = reachy.cameras.depth
    if depth_cam is None:
        print("[ERROR] No depth camera reported by the robot")
        return

    print("\n=== Capturing + filtering detections by workspace reachability ===")
    captured = _capture_and_filter(depth_cam, model, confidence, reachy)
    if captured is None:
        return

    while True:
        class_name, box, captured = reachy_detection._select_object_to_grasp(depth_cam, model, confidence, captured, reachy)
        if class_name is None:
            break

        decision = reachy_detection._confirm_grasp(class_name)
        if decision is None:
            break
        if decision:
            geometry = reachy_detection._build_object_point_cloud(depth_cam, class_name, box)
            if geometry is not None:
                print(f"[{class_name}] estimated width={geometry.width_m * 100:.1f}cm  "
                      f"height={geometry.height_m * 100:.1f}cm")
                plan = reachy_grasp.plan_grasp(reachy, geometry)
                if plan is None:
                    print(f"[{class_name}] no feasible grasp (too wide for the gripper, "
                          "or its pose couldn't be estimated)")
                else:
                    reachy_grasp.show_grasp_plan(geometry, plan)
                    print(f"[{class_name}] pre-grasp/grasp/lift plotted -- robot NOT moved (test script)")
            # Back to hover-select on the same captured frame, to try another object.
        # "No" -> same, back to hover-select

    cv2.destroyWindow(reachy_detection.CAM_WINDOW_NAME)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Test the reachability filter + grasp planning/plotting pipeline, without moving the robot."
    )
    parser.add_argument("robot_ip", nargs="?", default=reachy_detection.DEFAULT_ROBOT_IP,
                        help=f"IP address of the Reachy robot (default: {reachy_detection.DEFAULT_ROBOT_IP})")
    parser.add_argument("--yolo-model", default=reachy_detection.YOLO_MODEL_PATH,
                        help=f"Path to YOLOv8 weights (default: {reachy_detection.YOLO_MODEL_PATH})")
    parser.add_argument("--conf", type=float, default=reachy_detection.YOLO_CONFIDENCE,
                        help=f"Minimum reachy_detection confidence (default: {reachy_detection.YOLO_CONFIDENCE})")
    cli_args = parser.parse_args()

    reachy = ReachySDK(host=cli_args.robot_ip)

    if reachy.cameras is None or reachy.cameras.depth is None:
        print(f"[ERROR] No depth camera reported by the robot at '{cli_args.robot_ip}'")
        reachy.disconnect()
        sys.exit(1)
    if reachy.r_arm is None and reachy.l_arm is None:
        print(f"[ERROR] No arm reported by the robot at '{cli_args.robot_ip}'")
        reachy.disconnect()
        sys.exit(1)

    # Needed so arm.inverse_kinematics() has joint state to work from -- no
    # goto is ever issued by this script, so the arms never actually move.
    reachy.turn_on()

    print(f"Loading YOLO model '{cli_args.yolo_model}'...")
    model = YOLO(cli_args.yolo_model)

    try:
        _test_grasp_planning(reachy, model, cli_args.conf)
    finally:
        cv2.destroyAllWindows()
        reachy.disconnect()


if __name__ == "__main__":
    main()
