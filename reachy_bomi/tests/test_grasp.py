#!/usr/bin/env python3
"""
Dry-run of reachy_control.py's full pipeline (real robot, real camera, real
hand tracking) through object confirmation -- the grasp is only PLANNED and
VISUALIZED, never executed.

SAFETY: once an object is confirmed with "Yes", the robot does not move
again for any reason. execute_grasp() is never called anywhere in this
file. The robot sits in whatever pose the pre-grasping step left it in
until you quit (Q/ESC).

Reuses reachy_control.py's calibration -> cursor preview -> Control ->
pre-grasping pose -> object selection -> confirm flow unchanged (the base
still drives and the arms still move into the pre-grasping pose during
that part, exactly like the real script); only what happens after "Yes"
differs: instead of executing the grasp, it shows the point cloud plus
the planned pre-grasp/grasp gripper positions in a matplotlib window.

Dependencies:
    pip install reachy2-sdk mediapipe opencv-python scikit-learn numpy scipy ultralytics matplotlib pynput
    reachy2_symbolic_ik is NOT on PyPI -- install it separately:
        pip install git+https://github.com/pollen-robotics/reachy2_symbolic_ik.git

Usage:
    python3 test_grasp.py [robot_ip]
    (same options as reachy_control.py)
"""

import argparse
import os
import sys

import cv2
import matplotlib.pyplot as plt
import numpy as np
from mediapipe.tasks.python.core import base_options
from mediapipe.tasks.python.vision import hand_landmarker
from mediapipe.tasks.python.vision.core import vision_task_running_mode
from reachy2_sdk import ReachySDK
from ultralytics import YOLO

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

import bomi_detection as detection
import bomi_teleop as teleop
import reachy_control as ctrl
import reachy_grasp


def _show_point_cloud_with_grasp(point_cloud, p_pre, p_grasp, class_name) -> None:
    """Same 3D scatter as detection._show_point_cloud, with the planned
    pre-grasp/grasp gripper positions marked on top."""
    if point_cloud.shape[0] == 0:
        print("[WARN] Point cloud is empty, nothing to show")
        return

    fig = plt.figure(f"Point cloud + grasp - {class_name}")
    ax = fig.add_subplot(projection="3d")
    xs, ys, zs = point_cloud[:, 0], point_cloud[:, 1], point_cloud[:, 2]
    ax.scatter(xs, ys, zs, c=zs, cmap="viridis", s=4, label="object")
    ax.scatter(*p_grasp, c="red", s=120, marker="X", label="grasp point")
    ax.scatter(*p_pre, c="orange", s=80, marker="^", label="pre-grasp point")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_zlabel("z (m)")
    ax.set_title(f"{class_name}: {len(point_cloud)} points")
    ax.legend()

    all_points = np.vstack([point_cloud, p_grasp, p_pre])
    ranges = all_points.max(axis=0) - all_points.min(axis=0)
    half_range = max(ranges.max() / 2.0, 1e-3)
    mid = all_points.mean(axis=0)
    ax.set_xlim(mid[0] - half_range, mid[0] + half_range)
    ax.set_ylim(mid[1] - half_range, mid[1] + half_range)
    ax.set_zlim(mid[2] - half_range, mid[2] + half_range)

    print("Close the point cloud window to continue (robot stays put)...")
    plt.show()


def _plan_and_visualize_grasp(reachy, point_cloud, width_m, height_m, class_name) -> None:
    """Same bridging as detection._plan_and_execute_grasp, but stops at
    plan_grasp() -- execute_grasp() is deliberately never called here."""
    try:
        p_front, shape = reachy_grasp.point_cloud_to_grasp_input(point_cloud, class_name)
        arm = reachy_grasp.pick_arm(reachy, p_front)
        plan = reachy_grasp.plan_grasp(arm, p_front, width_m, height_m, shape)
    except reachy_grasp.ObjectTooWideError as e:
        print(f"[User] {e}")
    except reachy_grasp.GraspNotReachableError as e:
        print(f"[Error] {e}")
    else:
        p_grasp = plan["pose_grasp"][:3, 3]
        p_pre = plan["pose_pre"][:3, 3]
        print(f"Planned grasp point (robot frame, m): {p_grasp}")
        _show_point_cloud_with_grasp(point_cloud, p_pre, p_grasp, class_name)


def _run_grasp_dry_run(cap, landmarker, bomi_map, cursor_filter, depth_cam, model, confidence, reachy, crs_x, crs_y):
    """Same capture -> hover-select -> confirm loop as
    reachy_control._run_grasp_mode, but after "Yes" the grasp is only
    planned and visualized. No execute_grasp() call anywhere below: the
    robot cannot move again after this point except by you quitting."""
    captured = detection._capture_and_detect(depth_cam, model, confidence)
    if captured is None:
        return crs_x, crs_y

    while True:
        class_name, box, captured, crs_x, crs_y = ctrl._select_object_to_grasp_bomi(
            cap, landmarker, bomi_map, cursor_filter, crs_x, crs_y,
            depth_cam, model, confidence, captured,
        )
        if class_name is None:
            break

        decision, crs_x, crs_y = ctrl._confirm_grasp_bomi(
            cap, landmarker, bomi_map, cursor_filter, crs_x, crs_y, class_name,
        )
        if decision is None:
            break
        if decision:
            result = detection._build_object_point_cloud(depth_cam, class_name, box)
            if result is not None:
                point_cloud, width_m, height_m = result
                _plan_and_visualize_grasp(reachy, point_cloud, width_m, height_m, class_name)
                detection._stream_torso_camera(depth_cam)
            break
        # No -> back to the same captured frame/detections, all blue again

    cv2.destroyWindow(detection.CAM_WINDOW_NAME)
    return crs_x, crs_y


# reachy_control._teleop_with_grasp_switch calls _run_grasp_mode as a bare
# module-global name, resolved at call time -- patching it here redirects
# that one call into the dry-run above without duplicating the whole
# Control / pre-grasping-pose loop (which stays exactly the real one).
ctrl._run_grasp_mode = _run_grasp_dry_run


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Dry-run of reachy_control.py: plans and visualizes the "
                     "grasp instead of executing it."
    )
    parser.add_argument("robot_ip", nargs="?", default=ctrl.DEFAULT_ROBOT_IP,
                        help=f"IP address of the Reachy robot (default: {ctrl.DEFAULT_ROBOT_IP})")
    parser.add_argument("--cam", type=int, default=0, help="Webcam index (default: 0)")
    parser.add_argument("--model", default=teleop.DEFAULT_MODEL_PATH,
                        help="Path to the MediaPipe hand_landmarker.task model "
                             f"(default: {teleop.DEFAULT_MODEL_PATH}).")
    parser.add_argument("--yolo-model", default=detection.YOLO_MODEL_PATH,
                        help=f"Path to YOLOv8 weights (default: {detection.YOLO_MODEL_PATH})")
    parser.add_argument("--conf", type=float, default=detection.YOLO_CONFIDENCE,
                        help=f"Minimum detection confidence (default: {detection.YOLO_CONFIDENCE})")
    cli_args = parser.parse_args()

    if not os.path.exists(cli_args.model):
        print(f"[ERROR] MediaPipe model not found: '{cli_args.model}'")
        print("        Download hand_landmarker.task and pass its path with --model.")
        sys.exit(1)

    reachy = ReachySDK(host=cli_args.robot_ip)
    if reachy.mobile_base is None:
        print(f"[ERROR] No mobile base reported by the robot at '{cli_args.robot_ip}'")
        reachy.disconnect()
        sys.exit(1)
    if reachy.cameras is None or reachy.cameras.depth is None:
        print(f"[ERROR] No depth camera reported by the robot at '{cli_args.robot_ip}'")
        reachy.disconnect()
        sys.exit(1)
    if reachy.r_arm is None and reachy.l_arm is None:
        print(f"[ERROR] No arm reported by the robot at '{cli_args.robot_ip}'")
        reachy.disconnect()
        sys.exit(1)

    mobile_base = reachy.mobile_base
    depth_cam = reachy.cameras.depth

    reachy.turn_on()
    reachy.goto_posture("default", duration=3.0, wait=True)
    mobile_base.lidar.safety_enabled = True
    mobile_base.lidar.safety_slowdown_distance = teleop.LIDAR_SLOWDOWN_DISTANCE
    mobile_base.lidar.safety_critical_distance = teleop.LIDAR_CRITICAL_DISTANCE
    mobile_base.turn_on()

    def _emergency_shutdown() -> None:
        print("\n[QUIT] ESC/Q pressed — stopping the robot and exiting.")
        try:
            teleop.safe_robot_shutdown(reachy, mobile_base)
            reachy.disconnect()
            cv2.destroyAllWindows()
        finally:
            os._exit(0)

    teleop.start_global_quit_watcher(_emergency_shutdown)
    stop_terminal_watcher = teleop.start_terminal_quit_watcher(_emergency_shutdown)

    print(f"Loading YOLO model '{cli_args.yolo_model}'...")
    model = YOLO(cli_args.yolo_model)

    cap = None
    landmarker = None
    try:
        cap = cv2.VideoCapture(cli_args.cam, cv2.CAP_V4L2)
        if not cap.isOpened():
            print(f"[ERROR] Cannot open camera {cli_args.cam}")
            sys.exit(1)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        landmarker_options = hand_landmarker.HandLandmarkerOptions(
            base_options=base_options.BaseOptions(model_asset_path=cli_args.model),
            running_mode=vision_task_running_mode.VisionTaskRunningMode.VIDEO,
            num_hands=1,
            min_hand_detection_confidence=0.7,
            min_hand_presence_confidence=0.5,
            min_tracking_confidence=0.5,
        )
        landmarker = hand_landmarker.HandLandmarker.create_from_options(landmarker_options)

        bomi_map = teleop.BoMIMap()
        samples = teleop._calibration_phase(cap, landmarker)
        bomi_map.fit(samples)
        print("PCA map fitted")

        cursor_filter = teleop.CursorFilter()
        crs_x, crs_y = teleop._cursor_preview_phase(
            cap, landmarker, bomi_map, cursor_filter=cursor_filter, show_cam=False,
        )
        ctrl._teleop_with_grasp_switch(cap, landmarker, bomi_map, mobile_base, depth_cam, model, cli_args.conf, reachy,
                                        cursor_filter=cursor_filter, crs_x=crs_x, crs_y=crs_y)
    finally:
        if stop_terminal_watcher is not None:
            stop_terminal_watcher()
        teleop.safe_robot_shutdown(reachy, mobile_base)
        if cap is not None:
            cap.release()
        cv2.destroyAllWindows()
        if landmarker is not None:
            landmarker.close()
        reachy.disconnect()


if __name__ == "__main__":
    main()
