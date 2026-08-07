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

This script never streams a camera in-process: a periodic network round-trip
(Camera.get_frame) sitting inside the same loop as time-critical cursor/
velocity work caused a recurring stutter even rate-limited. Instead, from
right after calibration through Phase 3.5 it spawns tests/camera_viewer.py
as its own OS process (see _start_camera_viewer) showing the head camera,
stopped the moment Phase 4 opens; Phase 4's look-before-you-grasp checkpoint
spawns the same script for the same head camera again, blocking until
closed (see _show_camera_viewer_blocking) -- the hover-select UI itself
still uses the torso depth camera directly (needed there for YOLO/point
cloud), this is just the passive pre-execute preview. Either window can be
closed/quit on its own without affecting this process.

Phases 1-2 (Calibration, Cursor preview): identical to bomi_teleop.py.

Phase 3 - Control:
    Hand movement -> PCA cursor -> 9-region velocity -> mobile base, exactly
    as in bomi_teleop.py. Hold the cursor centered (region 5) for
    SELECTION_HOLD_SECONDS straight to move the arms into a pre-grasping pose.

Phase 3.5 - Pre-grasping pose (opened from Control):
    Both arms move to a pre-grasping posture (elbows bent to about
    PRE_GRASP_ELBOW_PITCH_DEG degrees) the base is held at zero
    speed. Once the arms are in place, the cursor/9-region map reappears
    (same as Control) but sends no speed commands; holding the cursor
    centered (region 5) for SELECTION_HOLD_SECONDS straight resumes Control
    with linear/angular velocities halved. Hold it centered for another
    SELECTION_HOLD_SECONDS straight from there to stop the base and open
    object selection.

Phase 4 - Object selection / grasp (opened from Control):
    Same capture -> hover-to-select -> Yes/No confirm flow as reachy_detection.py,
    and every hover point is the BoMI cursor (mapped into that window's pixel
    space). Answering "No" re-offers hover-select on a fresh capture;
    quitting (Q/ESC/X) or a successful grasp both end the run for good --
    Control never resumes (see _teleop_with_grasp_switch's "one-way switch"
    comment). On a successful lift, _place_back_and_wind_down puts the
    object right back down where it was picked up, retracts both arms to
    the pre-grasping posture, rotates the base 180 deg (table no longer in
    front of it), returns to the default posture, then main()'s finally
    block powers everything off.
"""

import argparse
import math
import os
import subprocess
import sys
import time
from typing import Optional

import cv2
import mediapipe as mp
import numpy as np
from mediapipe.tasks.python.core import base_options
from mediapipe.tasks.python.vision import hand_landmarker
from mediapipe.tasks.python.vision.core import vision_task_running_mode
from reachy2_sdk import ReachySDK
from ultralytics import YOLO

import graphs
import reachy_detection
import reachy_grasp
import safety
import bomi_teleop as teleop

# Placeholder — replace with the robot's actual IP.
DEFAULT_ROBOT_IP = "192.168.0.121"

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
HALVED_SPEED_FACTOR = 0.75 # max_lin = 0.375 [m/s], max_ang = 0.825 [rad/s] 

# tests/camera_viewer.py (head camera, LEFT eye), spawned as its own
# process by _start_camera_viewer (Control/pre-grasping pose) and
# _show_camera_viewer_blocking (grasp checkpoint) -- see either for why
# this isn't just an in-process stream.show_frame call.
CAMERA_VIEWER_SCRIPT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "tests", "camera_viewer.py")

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


def _bring_window_to_front(window_name: str) -> None:
    """Pins a cv2 window on top of every other window, staying pinned
    (not just a one-time raise) -- so the 9-region map window stays
    visible in front of the camera-viewer subprocess windows (see
    _start_camera_viewer) once they open, not just at the moment right
    after calibration when it would otherwise stay wherever it first
    opened. Qt backend only; a harmless no-op elsewhere."""
    cv2.namedWindow(window_name)
    cv2.setWindowProperty(window_name, cv2.WND_PROP_TOPMOST, 1)


# True for the duration of _run_grasp_mode (object selection/grasping) --
# tracked here, not threaded through every shutdown path, so
# _on_emergency_quit/main()'s finally block always know whether the robot
# might be sitting close to a table with its arms about to fold in (see
# safety.safe_robot_shutdown's rotate_base_before_shutdown), however deep
# in the call stack the quit was triggered from.
_in_grasp_phase = False

# The head-camera-viewer subprocess (tests/camera_viewer.py), if one is
# currently running -- tracked here rather than threaded through every
# function that might need to stop it, so _on_emergency_quit/the finally
# block in main() can always kill it, however deep in the call stack it
# was started from.
_camera_viewer_proc: Optional[subprocess.Popen] = None


def _start_camera_viewer(robot_ip: str) -> None:
    """Spawns tests/camera_viewer.py as its own OS process and returns
    immediately -- keeps its network round-trips (Camera.get_frame) out
    of this process's own loops entirely, instead of the in-process
    stream.show_frame this replaced, which had to be rate-limited to
    avoid stalling _teleop_with_grasp_switch's cursor/velocity timing."""
    global _camera_viewer_proc
    _camera_viewer_proc = subprocess.Popen([sys.executable, CAMERA_VIEWER_SCRIPT, robot_ip])


def _stop_camera_viewer() -> None:
    global _camera_viewer_proc
    if _camera_viewer_proc is not None and _camera_viewer_proc.poll() is None:
        _camera_viewer_proc.terminate()
    _camera_viewer_proc = None


def _show_camera_viewer_blocking(robot_ip: str) -> None:
    """Like _start_camera_viewer, but blocks until the viewer window is
    closed/quit -- a deliberate pause, e.g. reachy_grasp's
    look-before-you-grasp checkpoint before executing a grasp (was an
    in-process stream.stream_blocking call; now an external process for
    the same reason as the navigation-phase one, and so ESC/Q still stops
    the robot via safety.py's global watcher even while this blocks)."""
    _start_camera_viewer(robot_ip)
    _camera_viewer_proc.wait()
    _stop_camera_viewer()


# --- BoMI-cursor equivalents of reachy_detection's mouse-driven UI ---

def _select_object_to_grasp_bomi(
    cap, landmarker, bomi_map, cursor_filter, crs_x, crs_y,
    depth_cam, model, confidence, captured, reachy,
):
    """Same hover-to-select/hover-Refresh loop as tests/test_grasp.py's
    _select_object_to_grasp, but the hover point is the BoMI cursor mapped
    into the captured frame."""
    base_frame, detections, labels = captured
    frame_h, frame_w = base_frame.shape[:2]

    hovered_box = None
    hover_start = None
    button_hover_start = None

    print(f"\n=== CAPTURED FRAME (RGB + YOLO) ===  Q = quit  |  "
          f"hold Refresh for {reachy_detection.REFRESH_HOVER_SECONDS:.0f}s to recapture")

    while True:
        _, crs_x, crs_y, _ = _update_bomi_cursor(cap, landmarker, bomi_map, cursor_filter, crs_x, crs_y)

        gx, gy = _map_bomi_to_frame(crs_x, crs_y, frame_w, frame_h)
        now = time.time()

        on_button = reachy_detection._box_contains(reachy_detection.REFRESH_BUTTON_BOX, gx, gy)
        if not on_button:
            button_hover_start = None
        elif button_hover_start is None:
            button_hover_start = now
        elif now - button_hover_start >= reachy_detection.REFRESH_HOVER_SECONDS:
            refreshed = reachy_detection._capture_and_detect(depth_cam, model, confidence, reachy=reachy)
            if refreshed is not None:
                base_frame, detections, labels = refreshed
                frame_h, frame_w = base_frame.shape[:2]
            hovered_box, hover_start = None, None
            button_hover_start = None

        frame = base_frame.copy()
        hovered = reachy_detection._find_hovered_detection(detections, gx, gy)

        if hovered is None:
            hovered_box, hover_start = None, None
        else:
            box = hovered[2]
            if hovered_box is None or reachy_detection._iou(box, hovered_box) < reachy_detection.HOVER_IOU_MATCH:
                hover_start = now
            hovered_box = box
        hover_duration = (now - hover_start) if hover_start is not None else 0.0
        is_held = hovered is not None and hover_duration >= reachy_detection.HOVER_HOLD_SECONDS

        if is_held:
            box = hovered[2]
            reachy_detection._draw_box(frame, box, labels[box], reachy_detection.COLOR_GREEN)
            _draw_bomi_cursor(frame, gx, gy)
            cv2.imshow(reachy_detection.CAM_WINDOW_NAME, frame)
            cv2.waitKey(1)
            return hovered[0], box, (base_frame, detections, labels), crs_x, crs_y

        hover_progress = min(hover_duration / reachy_detection.HOVER_HOLD_SECONDS, 1.0) if hovered is not None else 0.0
        for class_name, conf, box in detections:
            is_hovered = hovered is not None and box == hovered[2]
            color = reachy_detection.COLOR_YELLOW if is_hovered else reachy_detection.COLOR_BLUE
            reachy_detection._draw_box(frame, box, labels[box], color, hover_progress if is_hovered else 0.0)

        button_progress = min((now - button_hover_start) / reachy_detection.REFRESH_HOVER_SECONDS, 1.0) if button_hover_start else 0.0
        reachy_detection._draw_refresh_button(frame, button_progress)
        _draw_bomi_cursor(frame, gx, gy)
        cv2.imshow(reachy_detection.CAM_WINDOW_NAME, frame)

        key = cv2.waitKey(1) & 0xFF
        if safety.quit_requested(key, reachy_detection.CAM_WINDOW_NAME):
            return None, None, (base_frame, detections, labels), crs_x, crs_y


def _confirm_grasp_bomi(cap, landmarker, bomi_map, cursor_filter, crs_x, crs_y, class_name):
    """Same Yes/No dwell dialog as tests/test_grasp.py's _confirm_grasp,
    hovered with the BoMI cursor mapped into the confirm canvas instead of
    the mouse."""
    yes_hover_start = None
    no_hover_start = None

    while True:
        _, crs_x, crs_y, _ = _update_bomi_cursor(cap, landmarker, bomi_map, cursor_filter, crs_x, crs_y)

        gx, gy = _map_bomi_to_frame(crs_x, crs_y, reachy_detection.CONFIRM_CANVAS_WIDTH, reachy_detection.CONFIRM_CANVAS_HEIGHT)
        now = time.time()
        on_yes = reachy_detection._box_contains(reachy_detection.YES_BUTTON_BOX, gx, gy)
        on_no = reachy_detection._box_contains(reachy_detection.NO_BUTTON_BOX, gx, gy)

        yes_hover_start = (yes_hover_start or now) if on_yes else None
        no_hover_start = (no_hover_start or now) if on_no else None
        yes_progress = min((now - yes_hover_start) / reachy_detection.CONFIRM_HOVER_SECONDS, 1.0) if yes_hover_start else 0.0
        no_progress = min((now - no_hover_start) / reachy_detection.CONFIRM_HOVER_SECONDS, 1.0) if no_hover_start else 0.0

        canvas = reachy_detection._draw_confirm_canvas(class_name, yes_progress, no_progress)
        _draw_bomi_cursor(canvas, gx, gy)
        cv2.imshow(reachy_detection.CONFIRM_WINDOW_NAME, canvas)

        key = cv2.waitKey(1) & 0xFF
        result = None
        quit_now = safety.quit_requested(key, reachy_detection.CONFIRM_WINDOW_NAME)
        if yes_progress >= 1.0:
            result = True
        elif no_progress >= 1.0:
            result = False

        if quit_now or result is not None:
            cv2.destroyWindow(reachy_detection.CONFIRM_WINDOW_NAME)
            return (None if quit_now else result), crs_x, crs_y


def _run_grasp_mode(cap, landmarker, bomi_map, cursor_filter, depth_cam, model, confidence, reachy, crs_x, crs_y,
                     robot_ip, mobile_base):
    """BoMI-driven equivalent of tests/test_grasp.py's _test_grasp_planning:
    capture -> hover-select -> confirm, looping back on "No" until an object is
    confirmed (then builds its point cloud, streams the live feed as a
    checkpoint, then plans and executes the grasp) or the user quits."""
    global _in_grasp_phase
    _in_grasp_phase = True
    try:
        captured = reachy_detection._capture_and_detect(depth_cam, model, confidence, reachy=reachy)
        if captured is None:
            return crs_x, crs_y

        while True:
            class_name, box, captured, crs_x, crs_y = _select_object_to_grasp_bomi(
                cap, landmarker, bomi_map, cursor_filter, crs_x, crs_y,
                depth_cam, model, confidence, captured, reachy,
            )
            if class_name is None:
                break

            decision, crs_x, crs_y = _confirm_grasp_bomi(
                cap, landmarker, bomi_map, cursor_filter, crs_x, crs_y, class_name,
            )
            if decision is None:
                break
            if decision:
                geometry = reachy_detection._build_object_point_cloud(depth_cam, class_name, box)
                if geometry is not None:
                    print(f"[{class_name}] estimated width={geometry.width_m * 100:.1f}cm  "
                          f"height={geometry.height_m * 100:.1f}cm")
                    # Live feed as a visual checkpoint before the arm moves: the
                    # frame shown during depth fusion (see _build_object_point_cloud)
                    # otherwise stays frozen through planning/execution below. Head
                    # camera (LEFT eye), same one used during navigation -- not the
                    # torso depth camera the hover-select UI itself uses.
                    _show_camera_viewer_blocking(robot_ip)
                    plan = reachy_grasp.plan_grasp(reachy, geometry)
                    if plan is None:
                        print(f"[{class_name}] no feasible grasp (too wide for the gripper, "
                              "or its pose couldn't be estimated)")
                    else:
                        graphs.show_grasp_plan(geometry, plan)
                        if reachy_grasp.execute_grasp(reachy, plan):
                            _place_back_and_wind_down(reachy, mobile_base, plan)
                break
            # No -> back to the same captured frame/detections, all blue again

        cv2.destroyWindow(reachy_detection.CAM_WINDOW_NAME)
        return crs_x, crs_y
    finally:
        _in_grasp_phase = False


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


def _place_back_and_wind_down(reachy, mobile_base, plan: reachy_grasp.GraspPlan) -> None:
    """Runs once, right after a successful grasp+lift: puts the object back
    down exactly where it was picked up from and retreats to pregrasp
    along a straight line (reachy_grasp.place_back, head watching the
    end-effector throughout), then -- before retracting -- sends the head
    back to its own default posture (reachy.head.goto_posture), so it's no
    longer tracking the end-effector once the arms start moving to the
    pre-grasping elbow_135 posture (now a safe joint-space move since the
    gripper already cleared the object). Then rotates the base
    safety.SHUTDOWN_ROTATION_DEG so whatever was in front of it (the
    table) ends up behind it instead, then returns to the default posture
    -- leaving the robot in a safe, predictable state for main()'s finally
    block (see _in_grasp_phase) to power off from."""
    reachy_grasp.place_back(reachy, plan)

    if reachy.head is not None:
        reachy.head.goto_posture(duration=1.0, wait=False)

    print(f"\nRetracting both arms to the pre-grasping posture "
          f"(elbow pitch {PRE_GRASP_ELBOW_PITCH_DEG:.0f} deg)...")
    goto_ids = _goto_pre_grasp_pose(reachy)
    while not all(reachy.is_goto_finished(goto_id) for goto_id in goto_ids):
        time.sleep(0.1)

    print(f"\nRotating the base {safety.SHUTDOWN_ROTATION_DEG:.0f} deg...")
    try:
        mobile_base.turn_on()
        mobile_base.translate_by(x=-0.1, y=0.0, wait=True)
        mobile_base.rotate_by(safety.SHUTDOWN_ROTATION_DEG, wait=True)
    except Exception as exc:
        print(f"[WARN] Could not rotate the base ({exc}).")

    print("\nReturning to default posture...")
    reachy.goto_posture("default", duration=3.0, wait=True)


def _draw_wait_canvas(progress: float) -> np.ndarray:
    """Dedicated canvas shown in its own window while the arms
    move into the pre-grasping pose -- a small text overlay on the
    already-open, busy camera feed is too easy to miss, so this pops up as
    a separate window instead, like reachy_detection's confirm dialog."""
    canvas = np.full((WAIT_CANVAS_HEIGHT, WAIT_CANVAS_WIDTH, 3), 30, dtype=np.uint8)
    cv2.putText(canvas, "Waiting for the robot",
                (30, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
    cv2.putText(canvas, "to finished its movement",
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
        if safety.quit_requested(key, WAIT_WINDOW_NAME):
            cv2.destroyWindow(WAIT_WINDOW_NAME)
            return crs_x, crs_y, True

        if all(reachy.is_goto_finished(goto_id) for goto_id in goto_ids):
            cv2.destroyWindow(WAIT_WINDOW_NAME)
            return crs_x, crs_y, False


# --- Control, with a dwell-in-center switch into grasp mode ---

def _teleop_with_grasp_switch(cap, landmarker, bomi_map, mobile_base, depth_cam, robot_ip, model, confidence, reachy,
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
            # speed_scale narrows the range (smaller max) without touching
            # the min -- see _goto_pre_grasp_pose's HALVED_SPEED_FACTOR use
            # below; multiplying the computed velocity instead would also
            # shrink the min, defeating the point of having one.
            lin_vel, ang_vel = teleop.compute_dynamic_vel_from_cursor(
                crs_x, crs_y,
                max_linear=teleop.MAX_LINEAR * speed_scale,
                max_angular=teleop.MAX_ANGULAR * speed_scale,
            )
            lin_vel, ang_vel = teleop.apply_region_velocity_mask(region, lin_vel, ang_vel)
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
                hold_seconds=SELECTION_HOLD_SECONDS,
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
            _stop_camera_viewer()
            _run_grasp_mode(
                cap, landmarker, bomi_map, cursor_filter, depth_cam, model, confidence, reachy, crs_x, crs_y,
                robot_ip, mobile_base,
            )
            break

        if now - last_publish >= dt:
            message = f"lin_vel:{lin_vel:.3f} ang_vel:{ang_vel:.3f}"
            # vtheta is in degrees/s for reachy2_sdk, ang_vel is computed in rad/s
            mobile_base.set_goal_speed(vx=lin_vel, vy=0, vtheta=math.degrees(ang_vel))
            mobile_base.send_speed_command()
            last_publish = now

        key = cv2.waitKey(1) & 0xFF
        if safety.quit_requested(key, map_window):
            break

    mobile_base.set_goal_speed(vx=0, vy=0, vtheta=0)
    mobile_base.send_speed_command()
    mobile_base.turn_off()
    cv2.destroyWindow(map_window)
    _stop_camera_viewer()  # no-op if already stopped (Phase 4 switch stops it itself)


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
    parser.add_argument("--yolo-model", default=reachy_detection.YOLO_MODEL_PATH,
                        help=f"Path to YOLOv8 weights (default: {reachy_detection.YOLO_MODEL_PATH})")
    parser.add_argument("--conf", type=float, default=reachy_detection.YOLO_CONFIDENCE,
                        help=f"Minimum detection confidence (default: {reachy_detection.YOLO_CONFIDENCE})")
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
    if reachy.cameras.teleop is None:
        print(f"[ERROR] No head/teleop camera reported by the robot at '{cli_args.robot_ip}'")
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
    reachy.r_arm.gripper.open()
    reachy.l_arm.gripper.open()
    mobile_base.lidar.safety_enabled = True
    mobile_base.lidar.safety_slowdown_distance = teleop.LIDAR_SLOWDOWN_DISTANCE
    mobile_base.lidar.safety_critical_distance = teleop.LIDAR_CRITICAL_DISTANCE
    mobile_base.turn_on()

    def _on_emergency_quit() -> None:
        _stop_camera_viewer()
        safety.emergency_shutdown(reachy, mobile_base, rotate_base_before_shutdown=_in_grasp_phase)

    safety.start_global_quit_watcher(_on_emergency_quit)
    stop_terminal_watcher = safety.start_terminal_quit_watcher(_on_emergency_quit)

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
        _bring_window_to_front(teleop.MAP_WINDOW_NAME)
        _start_camera_viewer(cli_args.robot_ip)

        cursor_filter = teleop.CursorFilter()
        crs_x, crs_y = teleop._cursor_preview_phase(
            cap, landmarker, bomi_map, cursor_filter=cursor_filter, show_cam=False,
            hold_seconds=SELECTION_HOLD_SECONDS,
        )
        _teleop_with_grasp_switch(cap, landmarker, bomi_map, mobile_base, depth_cam, cli_args.robot_ip, model,
                                   cli_args.conf, reachy, cursor_filter=cursor_filter, crs_x=crs_x, crs_y=crs_y)
    finally:
        _stop_camera_viewer()
        if stop_terminal_watcher is not None:
            stop_terminal_watcher()
        safety.safe_robot_shutdown(reachy, mobile_base, rotate_base_before_shutdown=_in_grasp_phase)
        if cap is not None:
            cap.release()
        cv2.destroyAllWindows()
        if landmarker is not None:
            landmarker.close()
        reachy.disconnect()


if __name__ == "__main__":
    main()
