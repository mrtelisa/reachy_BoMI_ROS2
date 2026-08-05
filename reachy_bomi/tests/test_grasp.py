#!/usr/bin/env python3
"""
Test script for the object selection + grasp planning pipeline -- with NO
arm motion.

Connects to Reachy, captures a frame, and runs the same hover-select/confirm
flow as reachy_detection.py. Once an object is confirmed, its point cloud is
built and reachy_grasp.plan_grasp computes pre-grasp/grasp/lift poses,
plotted together with the point cloud via reachy_grasp.show_grasp_plan.

The robot itself never moves: this script doesn't call reachy_grasp.execute_grasp.

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
                    # Printed regardless of whether it's actually reachable,
                    # so it can be pasted into a standalone
                    # arm.goto()/inverse_kinematics() test -- separate from
                    # this whole pipeline -- to check whether a rejection is
                    # a real limit of Reachy's kinematics or a bug upstream.
                    pregrasp_rounded = [[round(float(v), 6) for v in row] for row in plan.pregrasp_matrix.tolist()]
                    print(f"[{class_name}] pregrasp target for {plan.arm_name}:")
                    print(f"    pregrasp_matrix = np.array({pregrasp_rounded})")

                    reachy_grasp.show_grasp_plan(geometry, plan)
                    # reachy_grasp.execute_grasp(reachy, plan)

                    # Open -> pregrasp -> grasp -> lift, gripper left open
                    # throughout (never closed) -- separate from
                    # execute_grasp's own open->close->lift sequence, which
                    # stays commented out above.
                    arm = getattr(reachy, plan.arm_name, None)
                    if arm is None or arm.gripper is None:
                        print(f"[{class_name}] {plan.arm_name} or its gripper not available -- not moving")
                    else:
                        for name, matrix in (("pregrasp", plan.pregrasp_matrix),
                                              ("grasp", plan.grasp_matrix),
                                              ("lift", plan.lift_matrix)):
                            try:
                                arm.inverse_kinematics(matrix)
                            except ValueError:
                                print(f"[{class_name}] {name} pose unreachable for {plan.arm_name} -- not moving")
                                break
                        else:
                            print(f"[{plan.arm_name}] opening gripper...")
                            arm.gripper.open()
                            print(f"[{plan.arm_name}] moving to pregrasp...")
                            arm.goto(plan.pregrasp_matrix, duration=reachy_grasp.ARM_GOTO_DURATION_S, wait=True)
                            print(f"[{plan.arm_name}] moving to grasp...")
                            arm.goto(plan.grasp_matrix, duration=reachy_grasp.ARM_GOTO_DURATION_S,
                                      interpolation_space="cartesian_space", wait=True)
                            arm.gripper.close()
                            print(f"[{plan.arm_name}] moving to lift...")
                            arm.goto(plan.lift_matrix, duration=reachy_grasp.ARM_GOTO_DURATION_S, wait=True)
                            print(f"[{plan.arm_name}] done -- gripper left open")

            # Re-capture before offering selection again -- building the
            # point cloud takes real time, so the scene may well have
            # changed since the frame this round started from.
            print("\n=== Re-capturing ===")
            captured = reachy_detection._capture_and_detect(depth_cam, model, confidence)
            if captured is None:
                break
        # "No" -> same, back to hover-select

    cv2.destroyWindow(reachy_detection.CAM_WINDOW_NAME)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Test the object selection + grasp planning/plotting pipeline, without moving the robot."
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
    # "default" posture first (tests/elbow_135.py goes through "default" on
    # the way there -- this skips that step).
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
