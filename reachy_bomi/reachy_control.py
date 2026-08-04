#!/usr/bin/env python3
"""
BoMI teleop + grasp selection for Reachy2: merges bomi_teleop.py and
reachy_detection.py into a single BoMI cursor. Driving the mobile base and hovering
YOLO boxes/buttons in the grasp UI both use the same hand-tracked PCA cursor —
there's no mouse involved anywhere.

Runs entirely on a single PC with a webcam and network access to the robot;
mediapipe/opencv/ultralytics computation and the reachy2_sdk client live in
the same process.

Dependencies:
    pip install reachy2-sdk mediapipe opencv-python scikit-learn numpy scipy ultralytics matplotlib pynput

Usage:
    python3 reachy_control.py [robot_ip]

    <robot_ip> is optional; if omitted, DEFAULT_ROBOT_IP (in bomi_teleop.py) is used.

    Options:
        --cam INDEX          Webcam index. Default: 0
        --model PATH         Path to the MediaPipe hand_landmarker.task model.
        --yolo-model PATH    YOLOv8 weights (.pt). Default: yolov8n.pt
        --conf FLOAT         Minimum YOLO detection confidence. Default: 0.5

Phases 1-2 (Calibration, Cursor preview): identical to bomi_teleop.py.

Phase 3 - Control:
    Hand movement -> PCA cursor -> 9-region velocity -> mobile base, exactly
    as in bomi_teleop.py. Hold the cursor centered (region 5) for
    SELECTION_HOLD_SECONDS straight to move the arms into a pre-grasping pose.

Phase 3.5 - Pre-grasping pose (opened from Control):
    Both arms move to a pre-grasping posture (elbows bent to about
    PRE_GRASP_ELBOW_PITCH_DEG degrees) the base is held at zero
    speed. Once the arms are in place, the cursor/9-region map reappears
    (same as Control) but sends no speed commands; ENTER resumes Control
    with linear/angular velocities halved. Hold the cursor centered (region
    5) for another SELECTION_HOLD_SECONDS straight to stop the base and open
    object selection.

Phase 4 - Object selection / grasp (opened from Control):
    Same capture -> hover-to-select -> Yes/No confirm flow as reachy_detection.py,
    and every hover point is the BoMI cursor (mapped into that window's pixel
    space). Answering "No" or quitting (Q/ESC/X) returns to Control.
"""

import argparse
import math
import os
import sys
import time

import cv2
import mediapipe as mp
import numpy as np
from mediapipe.tasks.python.core import base_options
from mediapipe.tasks.python.vision import hand_landmarker
from mediapipe.tasks.python.vision.core import vision_task_running_mode
from reachy2_sdk import ReachySDK
from ultralytics import YOLO

import reachy_detection as grasp
import bomi_teleop as teleop

# Placeholder — replace with the robot's actual IP.
DEFAULT_ROBOT_IP = "192.168.0.104"

# How long the cursor must stay in region 5 (dead-zone center) before Control
# moves to the next step: first into the pre-grasping pose, then (once there)
# into the grasp/selection UI.
SELECTION_HOLD_SECONDS = 5.0

# Elbow pitch (degrees) for the pre-grasping posture: same convention as the
# SDK's built-in "elbow_90" posture (get_default_posture_joints), bent further
# up since 90 isn't high enough to clear the object before grasping.
PRE_GRASP_ELBOW_PITCH_DEG = -135.0
PRE_GRASP_MOVE_DURATION = 5.0  # seconds, arm goto duration into the pre-grasping pose

# Linear/angular velocity multiplier for Control once the arms are in the
# pre-grasping pose, so driving up to the object is finer-grained.
HALVED_SPEED_FACTOR = 0.5

WAIT_WINDOW_NAME = "BoMI - Waiting"
WAIT_CANVAS_WIDTH = 640
WAIT_CANVAS_HEIGHT = 220

COLOR_CURSOR = (255, 0, 255)  # magenta marker for the BoMI cursor, distinct from grasp's box colors


def _update_bomi_cursor(cap, landmarker, bomi_map, cursor_filter, crs_x, crs_y):
    """One iteration of hand tracking: reads a webcam frame, runs the hand
    landmarker, and returns (frame_with_landmarks, crs_x, crs_y, hand_detected).
    Cursor position is carried over unchanged when no hand is detected;
    hand_detected tells callers that drive the robot to stop instead of
    coasting on a stale position."""
    ret, frame = cap.read()
    if not ret:
        return None, crs_x, crs_y, False

    frame = cv2.flip(frame, 1)
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_frame)
    results = landmarker.detect_for_video(mp_image, int(time.time() * 1000))

    if not results.hand_landmarks:
        return frame, crs_x, crs_y, False

    hl = results.hand_landmarks[0]
    teleop._draw_hand_landmarks(frame, hl)
    mirror_x = results.handedness[0][0].category_name == "Right"
    crs_x, crs_y = bomi_map.transform(teleop._extract_hand_features(hl, mirror_x))
    crs_x, crs_y = cursor_filter.update(crs_x, crs_y)
    return frame, crs_x, crs_y, True


def _map_bomi_to_frame(crs_x: float, crs_y: float, width: int, height: int) -> tuple:
    """Rescale a cursor position from the BASE_WIDTH x BASE_HEIGHT BoMI screen
    space into an arbitrary window's pixel space (a captured camera frame or
    the fixed-size confirm canvas)."""
    return (
        int(crs_x / teleop.BASE_WIDTH * width),
        int(crs_y / teleop.BASE_HEIGHT * height),
    )


def _draw_bomi_cursor(frame, x: int, y: int) -> None:
    cv2.drawMarker(frame, (x, y), COLOR_CURSOR, markerType=cv2.MARKER_CROSS, markerSize=20, thickness=2)


# --- BoMI-cursor equivalents of reachy_detection's mouse-driven UI ---

def _select_object_to_grasp_bomi(
    cap, landmarker, bomi_map, cursor_filter, crs_x, crs_y,
    depth_cam, model, confidence, captured,
):
    """Same hover-to-select/hover-Refresh loop as reachy_detection._select_object_to_grasp,
    but the hover point is the BoMI cursor mapped into the captured frame."""
    base_frame, detections, labels = captured
    frame_h, frame_w = base_frame.shape[:2]

    hovered_box = None
    hover_start = None
    button_hover_start = None

    print(f"\n=== CAPTURED FRAME (RGB + YOLO) ===  Q = quit  |  "
          f"hold Refresh for {grasp.REFRESH_HOVER_SECONDS:.0f}s to recapture")

    while True:
        _, crs_x, crs_y, _ = _update_bomi_cursor(cap, landmarker, bomi_map, cursor_filter, crs_x, crs_y)

        gx, gy = _map_bomi_to_frame(crs_x, crs_y, frame_w, frame_h)
        now = time.time()

        on_button = grasp._box_contains(grasp.REFRESH_BUTTON_BOX, gx, gy)
        if not on_button:
            button_hover_start = None
        elif button_hover_start is None:
            button_hover_start = now
        elif now - button_hover_start >= grasp.REFRESH_HOVER_SECONDS:
            refreshed = grasp._capture_and_detect(depth_cam, model, confidence)
            if refreshed is not None:
                base_frame, detections, labels = refreshed
                frame_h, frame_w = base_frame.shape[:2]
            hovered_box, hover_start = None, None
            button_hover_start = None

        frame = base_frame.copy()
        hovered = grasp._find_hovered_detection(detections, gx, gy)

        if hovered is None:
            hovered_box, hover_start = None, None
        else:
            box = hovered[2]
            if hovered_box is None or grasp._iou(box, hovered_box) < grasp.HOVER_IOU_MATCH:
                hover_start = now
            hovered_box = box
        hover_duration = (now - hover_start) if hover_start is not None else 0.0
        is_held = hovered is not None and hover_duration >= grasp.HOVER_HOLD_SECONDS

        if is_held:
            box = hovered[2]
            grasp._draw_box(frame, box, labels[box], grasp.COLOR_GREEN)
            _draw_bomi_cursor(frame, gx, gy)
            cv2.imshow(grasp.CAM_WINDOW_NAME, frame)
            cv2.waitKey(1)
            return hovered[0], box, (base_frame, detections, labels), crs_x, crs_y

        hover_progress = min(hover_duration / grasp.HOVER_HOLD_SECONDS, 1.0) if hovered is not None else 0.0
        for class_name, conf, box in detections:
            is_hovered = hovered is not None and box == hovered[2]
            color = grasp.COLOR_YELLOW if is_hovered else grasp.COLOR_BLUE
            grasp._draw_box(frame, box, labels[box], color, hover_progress if is_hovered else 0.0)

        button_progress = min((now - button_hover_start) / grasp.REFRESH_HOVER_SECONDS, 1.0) if button_hover_start else 0.0
        grasp._draw_refresh_button(frame, button_progress)
        _draw_bomi_cursor(frame, gx, gy)
        cv2.imshow(grasp.CAM_WINDOW_NAME, frame)

        key = cv2.waitKey(1) & 0xFF
        if grasp._quit_requested(key, grasp.CAM_WINDOW_NAME):
            return None, None, (base_frame, detections, labels), crs_x, crs_y


def _confirm_grasp_bomi(cap, landmarker, bomi_map, cursor_filter, crs_x, crs_y, class_name):
    """Same Yes/No dwell dialog as reachy_detection._confirm_grasp, hovered with the
    BoMI cursor mapped into the confirm canvas instead of the mouse."""
    yes_hover_start = None
    no_hover_start = None

    while True:
        _, crs_x, crs_y, _ = _update_bomi_cursor(cap, landmarker, bomi_map, cursor_filter, crs_x, crs_y)

        gx, gy = _map_bomi_to_frame(crs_x, crs_y, grasp.CONFIRM_CANVAS_WIDTH, grasp.CONFIRM_CANVAS_HEIGHT)
        now = time.time()
        on_yes = grasp._box_contains(grasp.YES_BUTTON_BOX, gx, gy)
        on_no = grasp._box_contains(grasp.NO_BUTTON_BOX, gx, gy)

        yes_hover_start = (yes_hover_start or now) if on_yes else None
        no_hover_start = (no_hover_start or now) if on_no else None
        yes_progress = min((now - yes_hover_start) / grasp.CONFIRM_HOVER_SECONDS, 1.0) if yes_hover_start else 0.0
        no_progress = min((now - no_hover_start) / grasp.CONFIRM_HOVER_SECONDS, 1.0) if no_hover_start else 0.0

        canvas = grasp._draw_confirm_canvas(class_name, yes_progress, no_progress)
        _draw_bomi_cursor(canvas, gx, gy)
        cv2.imshow(grasp.CONFIRM_WINDOW_NAME, canvas)

        key = cv2.waitKey(1) & 0xFF
        result = None
        quit_now = grasp._quit_requested(key, grasp.CONFIRM_WINDOW_NAME)
        if yes_progress >= 1.0:
            result = True
        elif no_progress >= 1.0:
            result = False

        if quit_now or result is not None:
            cv2.destroyWindow(grasp.CONFIRM_WINDOW_NAME)
            return (None if quit_now else result), crs_x, crs_y


def _run_grasp_mode(cap, landmarker, bomi_map, cursor_filter, depth_cam, model, confidence, reachy, crs_x, crs_y):
    """BoMI-driven equivalent of reachy_detection._show_torso_camera: capture ->
    hover-select -> confirm, looping back on "No" until an object is
    confirmed (then builds its point cloud and streams the live feed --
    grasp planning/execution is not implemented yet) or the user quits."""
    captured = grasp._capture_and_detect(depth_cam, model, confidence)
    if captured is None:
        return crs_x, crs_y

    while True:
        class_name, box, captured, crs_x, crs_y = _select_object_to_grasp_bomi(
            cap, landmarker, bomi_map, cursor_filter, crs_x, crs_y,
            depth_cam, model, confidence, captured,
        )
        if class_name is None:
            break

        decision, crs_x, crs_y = _confirm_grasp_bomi(
            cap, landmarker, bomi_map, cursor_filter, crs_x, crs_y, class_name,
        )
        if decision is None:
            break
        if decision:
            result = grasp._build_object_point_cloud(depth_cam, class_name, box)
            if result is not None:
                _, width_m, height_m = result
                print(f"[{class_name}] estimated width={width_m * 100:.1f}cm  "
                      f"height={height_m * 100:.1f}cm -- grasp planning not implemented yet")
                grasp._stream_torso_camera(depth_cam)
            break
        # No -> back to the same captured frame/detections, all blue again

    cv2.destroyWindow(grasp.CAM_WINDOW_NAME)
    return crs_x, crs_y


# --- Pre-grasping pose, held between the two Control dwell phases ---

def _goto_pre_grasp_pose(reachy, duration: float = PRE_GRASP_MOVE_DURATION) -> list:
    """Send both arms towards the pre-grasping posture (elbows bent to about
    PRE_GRASP_ELBOW_PITCH_DEG degrees), non-blocking. Returns the GoToIds to
    poll for completion, skipping any arm that isn't reported or powered on."""
    goto_ids = []
    for arm in (reachy.r_arm, reachy.l_arm):
        if arm is None or not arm.is_on():
            continue
        joints = arm.get_default_posture_joints(common_posture="elbow_90")
        joints[3] = PRE_GRASP_ELBOW_PITCH_DEG
        goto_ids.append(arm.goto(joints, duration=duration, wait=False))
    return goto_ids


def _draw_wait_canvas(progress: float) -> np.ndarray:
    """Dedicated canvas shown in its own window while the arms
    move into the pre-grasping pose -- a small text overlay on the
    already-open, busy camera feed is too easy to miss, so this pops up as
    a separate window instead, like reachy_detection's confirm dialog."""
    canvas = np.full((WAIT_CANVAS_HEIGHT, WAIT_CANVAS_WIDTH, 3), 30, dtype=np.uint8)
    cv2.putText(canvas, "Waiting that the robot",
                (30, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
    cv2.putText(canvas, "has finished its movement",
                (30, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)

    bar_x1, bar_y1, bar_x2, bar_y2 = 30, 160, WAIT_CANVAS_WIDTH - 30, 190
    cv2.rectangle(canvas, (bar_x1, bar_y1), (bar_x2, bar_y2), (90, 90, 90), 1)
    filled_x = bar_x1 + int((bar_x2 - bar_x1) * progress)
    cv2.rectangle(canvas, (bar_x1, bar_y1), (filled_x, bar_y2), (0, 255, 255), -1)
    return canvas


def _wait_for_pre_grasp_pose(cap, landmarker, bomi_map, cursor_filter, crs_x, crs_y, reachy, mobile_base, goto_ids):
    """Blocks until every goto in goto_ids has finished, showing a dedicated 
    waiting window with a progress bar. The base is repeatedly held at zero 
    speed throughout. Returns the possibly updated cursor position and whether 
    the user quit."""
    dt = 1.0 / teleop.PUBLISH_HZ
    last_publish = time.time()
    start = time.time()

    while True:
        _, crs_x, crs_y, _ = _update_bomi_cursor(cap, landmarker, bomi_map, cursor_filter, crs_x, crs_y)

        progress = min((time.time() - start) / PRE_GRASP_MOVE_DURATION, 1.0)
        cv2.imshow(WAIT_WINDOW_NAME, _draw_wait_canvas(progress))

        now = time.time()
        if now - last_publish >= dt:
            mobile_base.set_goal_speed(vx=0, vy=0, vtheta=0)
            mobile_base.send_speed_command()
            last_publish = now

        key = cv2.waitKey(1) & 0xFF
        if teleop._quit_requested(key, WAIT_WINDOW_NAME):
            cv2.destroyWindow(WAIT_WINDOW_NAME)
            return crs_x, crs_y, True

        if all(reachy.is_goto_finished(goto_id) for goto_id in goto_ids):
            cv2.destroyWindow(WAIT_WINDOW_NAME)
            return crs_x, crs_y, False


# --- Control, with a dwell-in-center switch into grasp mode ---

def _teleop_with_grasp_switch(cap, landmarker, bomi_map, mobile_base, depth_cam, model, confidence, reachy,
                               cursor_filter=None, crs_x=None, crs_y=None) -> None:
    # Pass in the cursor_filter/crs_x/crs_y from the preceding cursor preview
    # to continue them (no filter reset), avoiding a velocity blip on entry.
    dt = 1.0 / teleop.PUBLISH_HZ
    last_publish = time.time()
    cursor_filter = cursor_filter or teleop.CursorFilter()
    map_window = teleop.MAP_WINDOW_NAME

    if crs_x is None or crs_y is None:
        crs_x, crs_y = teleop.BASE_WIDTH / 2.0, teleop.BASE_HEIGHT / 2.0
    region = teleop.check_region_cursor(crs_x, crs_y)
    message = "lin_vel:0.000 ang_vel:0.000"
    center_hold_start = None
    speed_scale = 1.0
    pre_grasp_reached = False

    print("\n=== CONTROL ===  Q = quit  |  hold the cursor centered (region 5) "
          f"for {SELECTION_HOLD_SECONDS:.0f}s to move to the pre-grasping pose")

    while True:
        hand_frame, crs_x, crs_y, hand_detected = _update_bomi_cursor(
            cap, landmarker, bomi_map, cursor_filter, crs_x, crs_y,
        )
        if hand_frame is None:
            continue

        if hand_detected:
            region = teleop.check_region_cursor(crs_x, crs_y)
            lin_vel, ang_vel = teleop.compute_dynamic_vel_from_cursor(crs_x, crs_y)
            lin_vel, ang_vel = teleop.apply_region_velocity_mask(region, lin_vel, ang_vel)
            lin_vel *= speed_scale
            ang_vel *= speed_scale
        else:
            lin_vel, ang_vel = 0.0, 0.0

        now = time.time()
        # Only accrue dwell time while actively tracked and centered, so a
        # dropped hand while the stale cursor happens to sit in region 5
        # can't silently trigger the switch into the next step.
        center_hold_start = (center_hold_start or now) if (hand_detected and region == 5) else None
        center_progress = (
            min((now - center_hold_start) / SELECTION_HOLD_SECONDS, 1.0) if center_hold_start else 0.0
        )

        cv2.imshow(map_window, teleop._draw_cursor_map(crs_x, crs_y, region, message))

        if center_progress >= 1.0 and not pre_grasp_reached:
            # First dwell: stop and hold here while the arms move, then resume
            # Control at half speed — the base itself never switches modes.
            mobile_base.set_goal_speed(vx=0, vy=0, vtheta=0)
            mobile_base.send_speed_command()
            print("\nMoving arms to pre-grasping pose "
                  f"(elbow pitch {PRE_GRASP_ELBOW_PITCH_DEG:.0f} deg)...")
            goto_ids = _goto_pre_grasp_pose(reachy)
            crs_x, crs_y, quit_now = _wait_for_pre_grasp_pose(
                cap, landmarker, bomi_map, cursor_filter, crs_x, crs_y, reachy, mobile_base, goto_ids,
            )
            if quit_now:
                break
            cursor_filter.reset(crs_x, crs_y)

            crs_x, crs_y = teleop._cursor_preview_phase(
                cap, landmarker, bomi_map, cursor_filter=cursor_filter, crs_x=crs_x, crs_y=crs_y, show_cam=False,
            )

            pre_grasp_reached = True
            speed_scale = HALVED_SPEED_FACTOR
            center_hold_start = None
            print("\nPre-grasping pose reached. Control resumed at half speed — "
                  f"hold the cursor centered (region 5) for {SELECTION_HOLD_SECONDS:.0f}s "
                  "to open object selection")
            continue

        if center_progress >= 1.0 and pre_grasp_reached:
            # One-way switch: once object selection opens, the base is powered
            # off for good, not just zeroed — Control never runs again after
            # this, so there's no loop iteration left that could re-drive it.
            mobile_base.set_goal_speed(vx=0, vy=0, vtheta=0)
            mobile_base.send_speed_command()
            mobile_base.turn_off()
            print("\nMobile base powered off. Switching to object selection for good.")
            cv2.destroyWindow(map_window)
            _run_grasp_mode(
                cap, landmarker, bomi_map, cursor_filter, depth_cam, model, confidence, reachy, crs_x, crs_y,
            )
            break

        if now - last_publish >= dt:
            message = f"lin_vel:{lin_vel:.3f} ang_vel:{ang_vel:.3f}"
            # vtheta is in degrees/s for reachy2_sdk, ang_vel is computed in rad/s
            mobile_base.set_goal_speed(vx=lin_vel, vy=0, vtheta=math.degrees(ang_vel))
            mobile_base.send_speed_command()
            last_publish = now

        key = cv2.waitKey(1) & 0xFF
        if teleop._quit_requested(key, map_window):
            break

    mobile_base.set_goal_speed(vx=0, vy=0, vtheta=0)
    mobile_base.send_speed_command()
    mobile_base.turn_off()
    cv2.destroyWindow(map_window)


# --- Entry point ---

def main() -> None:
    parser = argparse.ArgumentParser(
        description="BoMI teleop for Reachy2 that switches into BoMI-driven "
                     "object selection/grasp when the cursor is held centered."
    )
    parser.add_argument("robot_ip", nargs="?", default=DEFAULT_ROBOT_IP,
                        help=f"IP address of the Reachy robot (default: {DEFAULT_ROBOT_IP})")
    parser.add_argument("--cam", type=int, default=0, help="Webcam index (default: 0)")
    parser.add_argument("--model", default=teleop.DEFAULT_MODEL_PATH,
                        help="Path to the MediaPipe hand_landmarker.task model "
                             f"(default: {teleop.DEFAULT_MODEL_PATH}).")
    parser.add_argument("--yolo-model", default=grasp.YOLO_MODEL_PATH,
                        help=f"Path to YOLOv8 weights (default: {grasp.YOLO_MODEL_PATH})")
    parser.add_argument("--conf", type=float, default=grasp.YOLO_CONFIDENCE,
                        help=f"Minimum detection confidence (default: {grasp.YOLO_CONFIDENCE})")
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
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # always the freshest frame, not a growing backlog

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
        _teleop_with_grasp_switch(cap, landmarker, bomi_map, mobile_base, depth_cam, model, cli_args.conf, reachy,
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
