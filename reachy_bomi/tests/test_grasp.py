#!/usr/bin/env python3
"""
Test script for the object selection + grasp planning + execution pipeline.

Connects to Reachy, sends both arms straight to the elbow_135 starting
posture, then runs the same hover-select/confirm flow as
reachy_detection.py. Once an object is confirmed, its point cloud is built,
reachy_grasp.plan_grasp computes pre-grasp/grasp/lift poses (plotted via
show_grasp_plan), and reachy_grasp.execute_grasp drives the arm through
them for real.

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

import cv2

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from reachy2_sdk import ReachySDK
from ultralytics import YOLO

import graphs
import reachy_detection
import reachy_grasp

# Same convention as tests/elbow_135.py.
ELBOW_PITCH_DEG = -135.0
ELBOW_MOVE_DURATION_S = 3.0


def _test_grasp_planning(reachy: ReachySDK, model: YOLO, confidence: float) -> None:
    depth_cam = reachy.cameras.depth
    if depth_cam is None:
        print("[ERROR] No depth camera reported by the robot")
        return

    print("\n=== Capturing frame ===")
    captured = reachy_detection._capture_and_detect(depth_cam, model, confidence)
    if captured is None:
        return

    while True:
        class_name, box, captured = reachy_detection._select_object_to_grasp(depth_cam, model, confidence, captured)
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
                    # Same matrix as pregrasp_matrix below, printed so it can
                    # be pasted into tests/test_reachability.py to check a
                    # rejection in isolation from the rest of the pipeline.
                    rounded = [[round(float(v), 6) for v in row] for row in plan.pregrasp_matrix.tolist()]
                    print(f"[{class_name}] pregrasp target for {plan.arm_name}: np.array({rounded})")

                    graphs.show_grasp_plan(geometry, plan)
                    reachy_grasp.execute_grasp(reachy, plan)

            # Re-capture before offering selection again -- building the
            # point cloud takes real time, so the scene may have changed.
            print("\n=== Re-capturing ===")
            captured = reachy_detection._capture_and_detect(depth_cam, model, confidence)
            if captured is None:
                break
        # "No" -> same, back to hover-select

    cv2.destroyWindow(reachy_detection.CAM_WINDOW_NAME)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Test the object selection + grasp planning/execution pipeline."
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

    reachy.turn_on()

    # Straight to elbow_135's final joint configuration, no intermediate
    # "default" posture first (unlike tests/elbow_135.py itself).
    for arm in (reachy.r_arm, reachy.l_arm):
        if arm is None or not arm.is_on():
            continue
        joints = arm.get_default_posture_joints(common_posture="elbow_90")
        joints[3] = ELBOW_PITCH_DEG
        print(f"Moving {arm._part_id.name} to elbow_135 starting posture...")
        arm.goto(joints, duration=ELBOW_MOVE_DURATION_S, wait=True)

    print(f"Loading YOLO model '{cli_args.yolo_model}'...")
    model = YOLO(cli_args.yolo_model)

    try:
        _test_grasp_planning(reachy, model, cli_args.conf)
    finally:
        cv2.destroyAllWindows()
        reachy.disconnect()


if __name__ == "__main__":
    main()
