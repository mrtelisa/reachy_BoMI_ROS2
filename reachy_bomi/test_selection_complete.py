#!/usr/bin/env python3
"""
Test/dry-run version of bomi_selection_complete.py's full flow: same
calibration -> cursor preview -> Control -> pre-grasping pose -> object
selection sequence, but entirely self-contained like test_teleop.py --
no reachy2_sdk / ultralytics import, no robot, no depth camera, no YOLO
model. Use this to sanity-check the pre-grasping-pose addition (dwell,
waiting screen, halved-speed resume) before running the real script
against the robot.

Dependencies:
    pip install mediapipe opencv-python scikit-learn numpy scipy

Usage:
    python3 test_selection_complete.py
    Options:
        --model PATH    Path to the MediaPipe hand_landmarker.task model.
        --cam INDEX     Webcam index. Default: 0

Phase 1 - Calibration (always runs first):
    Move your hand through all positions you intend to use.
    SPACE = record sample   |   ENTER = finish (min 30 samples required)

Phase 2 - Cursor preview:
    Same cursor/region view as Control, nothing computed/logged yet.
    ENTER = proceed to Control   |   Q, ESC, or closing a window = quit

Phase 3 - Control:
    Hand movement -> PCA cursor -> 9-region velocity -> logged, not sent
    (there's no robot to send it to). Hold the cursor centered (region 5)
    for SELECTION_HOLD_SECONDS straight to simulate moving to the
    pre-grasping pose.

Phase 3.5 - Pre-grasping pose (simulated):
    A "Waiting that the robot has finished its movement" screen is shown
    for PRE_GRASP_MOVE_DURATION seconds (standing in for the real arm
    goto). The cursor/9-region map then reappears, sending nothing, until
    ENTER resumes Control with velocities halved.

Phase 4 - Object selection (stub):
    Since there's no real depth camera to capture a frame from, nothing is
    shown here -- exactly what the real script does when
    depth_cam.get_frame() fails.
"""

import argparse
import math
import os
import sys
import time

import cv2
import mediapipe as mp
import numpy as np
import scipy.signal as sgn
from mediapipe.tasks.python.core import base_options
from mediapipe.tasks.python.vision import hand_landmarker
from mediapipe.tasks.python.vision.core import vision_task_running_mode
from sklearn.decomposition import PCA

HAND_CONNECTIONS = hand_landmarker.HandLandmarksConnections.HAND_CONNECTIONS

# --- Virtual screen dimensions (same as bomi_teleop.py) ---
BASE_WIDTH = 2550
BASE_HEIGHT = 1500

MAX_LINEAR = 0.6      # m/s
MAX_ANGULAR = 0.8     # rad/s
DEAD_ZONE_PX = 200    # pixel radius around screen center before motion starts

PUBLISH_HZ = 20  # (test) rate for the periodic "would send" print, not an actual command rate

CURSOR_FILTER_HZ = 30.0        # assumed webcam/control loop sample rate
CURSOR_FILTER_CUTOFF_HZ = 4.0  # cutoff frequency

DEFAULT_MODEL_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "hand_landmarker.task"
)

CAM_WINDOW_NAME = "BoMI - Camera (TEST)"
MAP_WINDOW_NAME = "BoMI - Cursor Map (TEST)"
WAIT_WINDOW_NAME = "BoMI - Waiting (TEST)"
WAIT_CANVAS_WIDTH = 640
WAIT_CANVAS_HEIGHT = 220

# How long the cursor must stay in region 5 before Control moves to the next
# step -- matches SELECTION_HOLD_SECONDS in bomi_selection_complete.py.
SELECTION_HOLD_SECONDS = 5.0

# There's no real arm to move here, so the pre-grasping "goto" is simulated
# as a plain timeout of this many seconds.
PRE_GRASP_MOVE_DURATION = 5.0
PRE_GRASP_ELBOW_PITCH_DEG = -135.0

HALVED_SPEED_FACTOR = 0.5

COLOR_CURSOR = (255, 0, 255)


# --- Velocity helpers (copied from bomi_teleop.py) ---------

def check_region_cursor(crs_x: float, crs_y: float) -> int:
    """
    Returns region 1-9 based on 3x3 grid over BASE_WIDTH x BASE_HEIGHT.
    Layout:
        1 | 2 | 3   (top row)
        4 | 5 | 6   (middle row)
        7 | 8 | 9   (bottom row)
    """
    if crs_x < 847:
        col = 0
    elif crs_x <= 1697:
        col = 1
    else:
        col = 2

    if crs_y < 497:
        row = 0
    elif crs_y <= 997:
        row = 1
    else:
        row = 2

    return row * 3 + col + 1


def compute_dynamic_vel_from_cursor(
    crs_x: float,
    crs_y: float,
    max_linear: float = MAX_LINEAR,
    max_angular: float = MAX_ANGULAR,
    dead_zone_px: float = DEAD_ZONE_PX,
    ang_right_is_negative: bool = True,
) -> tuple:
    """
    Continuous linear/angular velocity from cursor position.
    Cursor at screen center -> zero velocity (dead zone).
    Up from center -> positive linear; right from center -> negative angular.
    """
    cx = BASE_WIDTH / 2.0
    cy = BASE_HEIGHT / 2.0
    dx = crs_x - cx
    dy = crs_y - cy

    if np.hypot(dx, dy) < dead_zone_px:
        return 0.0, 0.0

    x_norm = float(np.clip(dx / cx, -1.0, 1.0))
    y_norm = float(np.clip(-dy / cy, -1.0, 1.0))  # up = positive

    if abs(x_norm) < dead_zone_px / cx:
        x_norm = 0.0
    if abs(y_norm) < dead_zone_px / cy:
        y_norm = 0.0

    lin_vel = max_linear * y_norm
    ang_sign = -1.0 if ang_right_is_negative else 1.0
    ang_vel = ang_sign * max_angular * x_norm
    return lin_vel, ang_vel


def apply_region_velocity_mask(region: int, lin_vel: float, ang_vel: float) -> tuple:
    """
    Enforce active DOFs per region:
      center (5)        -> stop
      middle col (2, 8) -> linear only
      middle row (4, 6) -> angular only
      corners (1,3,7,9) -> both
    """
    if region == 5:
        return 0.0, 0.0
    if region in (2, 8):
        ang_vel = 0.0
    if region in (4, 6):
        lin_vel = 0.0
    return lin_vel, ang_vel


# --- PCA forward map (copied from bomi_teleop.py) ---

def _extract_hand_features(hand_landmarks, mirror_x: bool = False) -> np.ndarray:
    """Flatten all 21 hand landmarks (x, y) into a 42-element vector,
    mirroring x (1 - x) when mirror_x so the right hand maps to the same
    feature space as the left hand."""
    coords = [[1.0 - lm.x if mirror_x else lm.x, lm.y] for lm in hand_landmarks]
    return np.array(coords).flatten()


def _draw_hand_landmarks(frame, landmarks) -> None:
    """Draw MediaPipe Tasks hand landmarks/connections on a BGR OpenCV frame."""
    height, width = frame.shape[:2]
    points = []

    for landmark in landmarks:
        x = min(max(int(landmark.x * width), 0), width - 1)
        y = min(max(int(landmark.y * height), 0), height - 1)
        points.append((x, y))

    for connection in HAND_CONNECTIONS:
        cv2.line(frame, points[connection.start], points[connection.end], (0, 200, 255), 2)

    for point in points:
        cv2.circle(frame, point, 4, (0, 255, 0), -1)


def _quit_requested(key: int, window_name: str) -> bool:
    """True if Q/ESC was pressed, or the window was closed with the X button."""
    if key in (ord('q'), ord('Q'), 27):
        return True
    try:
        return cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) < 1
    except cv2.error:
        return False


def _draw_cursor_map(crs_x: float, crs_y: float, region: int, message: str,
                      map_width: int = 850, map_height: int = 500):
    """Rectangle representing the BASE_WIDTH x BASE_HEIGHT virtual screen, with
    the 9-region grid lines, a dot at the current cursor position, and the
    lin_vel/ang_vel message that would be sent to the mobile base."""
    canvas = np.full((map_height, map_width, 3), 30, dtype=np.uint8)
    sx = map_width / BASE_WIDTH
    sy = map_height / BASE_HEIGHT

    x1, x2 = int(847 * sx), int(1697 * sx)
    y1, y2 = int(497 * sy), int(997 * sy)
    for x in (x1, x2):
        cv2.line(canvas, (x, 0), (x, map_height), (90, 90, 90), 1)
    for y in (y1, y2):
        cv2.line(canvas, (0, y), (map_width, y), (90, 90, 90), 1)

    cx, cy = int(crs_x * sx), int(crs_y * sy)
    cv2.circle(canvas, (cx, cy), 10, (0, 0, 255), -1)

    cv2.putText(canvas, f"region={region}  cursor=({crs_x:.0f},{crs_y:.0f})",
                (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    cv2.putText(canvas, f"-> (test) would send: {message}",
                (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    return canvas


class CursorFilter:
    """3rd-order Butterworth low-pass filter for the (crs_x, crs_y) cursor
    position. Coefficients are derived from the actual sample rate
    (CURSOR_FILTER_HZ) instead of being hardcoded for a fixed frequency loop."""

    ORDER = 3

    def __init__(self, sample_hz: float = CURSOR_FILTER_HZ, cutoff_hz: float = CURSOR_FILTER_CUTOFF_HZ) -> None:
        nyquist = sample_hz / 2.0
        self._b, self._a = sgn.butter(self.ORDER, cutoff_hz / nyquist, btype="low")
        self._in_history = np.zeros((self.ORDER, 2))
        self._out_history = np.zeros((self.ORDER, 2))

    def update(self, crs_x: float, crs_y: float) -> tuple:
        new_input = np.array([crs_x, crs_y])
        new_output = self._b[0] * new_input
        for i in range(self.ORDER):
            new_output += self._b[i + 1] * self._in_history[i]
        for i in range(self.ORDER):
            new_output -= self._a[i + 1] * self._out_history[i]

        self._in_history = np.roll(self._in_history, 1, axis=0)
        self._in_history[0] = new_input
        self._out_history = np.roll(self._out_history, 1, axis=0)
        self._out_history[0] = new_output

        return float(new_output[0]), float(new_output[1])


class BoMIMap:
    """PCA forward map: raw hand landmarks -> 2D cursor in screen space.

    PCA(2 components) fitted directly on the raw landmark samples. Scale/offset
    map the calibration scores' peak-to-peak range onto the screen size,
    centered on the mean."""

    def __init__(self) -> None:
        self._mean = None
        self._components = None
        self._scale = np.ones(2)
        self._offset = np.zeros(2)
        self.fitted = False

    def fit(self, samples: list) -> None:
        X = np.array(samples)

        pca = PCA(n_components=2)
        scores = pca.fit_transform(X)

        extent = np.ptp(scores, axis=0)
        extent = np.where(extent > 1e-6, extent, 1.0)

        screen = np.array([BASE_WIDTH, BASE_HEIGHT], dtype=float)

        self._mean = pca.mean_
        self._components = pca.components_
        self._scale = screen / extent
        self._offset = screen / 2.0 - (scores * self._scale).mean(axis=0)
        self.fitted = True

    def transform(self, features: np.ndarray) -> tuple:
        cu = np.dot(features - self._mean, self._components.T)
        cu = cu * self._scale + self._offset
        crs_x = float(np.clip(cu[0], 0, BASE_WIDTH))
        crs_y = float(np.clip(cu[1], 0, BASE_HEIGHT))
        return crs_x, crs_y


def _update_cursor(cap, landmarker, bomi_map, cursor_filter, crs_x, crs_y):
    """One iteration of hand tracking: reads a webcam frame, runs the hand
    landmarker, and returns (frame_with_landmarks, crs_x, crs_y, hand_detected).
    Cursor position is carried over unchanged when no hand is detected."""
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
    _draw_hand_landmarks(frame, hl)
    mirror_x = results.handedness[0][0].category_name == "Right"
    crs_x, crs_y = bomi_map.transform(_extract_hand_features(hl, mirror_x))
    crs_x, crs_y = cursor_filter.update(crs_x, crs_y)
    return frame, crs_x, crs_y, True


# --- Phases ---

def _calibration_phase(cap, landmarker) -> list:
    MIN_SAMPLES = 30
    samples = []

    print("\n=== CALIBRATION ===")
    print("Move your hand through all positions you intend to use.")
    print("SPACE = record sample   |   ENTER = finish (need >= 30)   |   Q = quit")

    while True:
        ret, frame = cap.read()
        if not ret:
            continue
        frame = cv2.flip(frame, 1)
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_frame)
        results = landmarker.detect_for_video(mp_image, int(time.time() * 1000))

        if results.hand_landmarks:
            _draw_hand_landmarks(frame, results.hand_landmarks[0])

        label = f"Samples: {len(samples)}/{MIN_SAMPLES}  SPACE=add  ENTER=done  Q=quit"
        cv2.putText(frame, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
        window_name = "BoMI - Calibration"
        cv2.imshow(window_name, frame)
        key = cv2.waitKey(1) & 0xFF

        if key == ord(' ') and results.hand_landmarks:
            mirror_x = results.handedness[0][0].category_name == "Right"
            samples.append(_extract_hand_features(results.hand_landmarks[0], mirror_x))
            print(f"  Sample {len(samples)} recorded")

        elif key == 13:  # ENTER
            if len(samples) >= MIN_SAMPLES:
                print(f"  Calibration done ({len(samples)} samples)")
                break
            else:
                print(f"  Need at least {MIN_SAMPLES} samples (have {len(samples)})")

        elif _quit_requested(key, window_name):
            print("Aborted.")
            sys.exit(0)

    cv2.destroyWindow(window_name)
    return samples


def _cursor_preview_phase(cap, landmarker, bomi_map: BoMIMap, cursor_filter: CursorFilter = None,
                           crs_x: float = None, crs_y: float = None) -> tuple:
    """Same cursor/region view as Control, but nothing is computed/logged
    yet. Lets the user get a feel for the cursor before Control begins.

    Pass an existing cursor_filter/crs_x/crs_y to continue them (no filter
    reset) instead of starting fresh; returns the final (crs_x, crs_y)."""
    cursor_filter = cursor_filter or CursorFilter()
    if crs_x is None or crs_y is None:
        crs_x, crs_y = BASE_WIDTH / 2.0, BASE_HEIGHT / 2.0
    region = check_region_cursor(crs_x, crs_y)

    print("\n=== CURSOR PREVIEW (nothing computed/logged yet) ===")
    print("Get a feel for the cursor. ENTER = start Control   |   Q = quit")

    while True:
        frame, crs_x, crs_y, hand_detected = _update_cursor(cap, landmarker, bomi_map, cursor_filter, crs_x, crs_y)
        if frame is None:
            continue
        if hand_detected:
            region = check_region_cursor(crs_x, crs_y)

        cv2.putText(frame, f"region={region}  cursor=({crs_x:.0f},{crs_y:.0f})",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(frame, "PREVIEW - nothing logged. ENTER=start control  Q=quit",
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        cv2.imshow(CAM_WINDOW_NAME, frame)
        cv2.imshow(MAP_WINDOW_NAME, _draw_cursor_map(crs_x, crs_y, region, "(preview - not computed)"))

        key = cv2.waitKey(1) & 0xFF
        if key == 13:  # ENTER
            break
        if _quit_requested(key, CAM_WINDOW_NAME) or _quit_requested(key, MAP_WINDOW_NAME):
            print("Aborted.")
            sys.exit(0)

    # Windows are intentionally left open so they carry straight into Control.
    return crs_x, crs_y


def _draw_wait_canvas(progress: float) -> np.ndarray:
    """Dedicated, unmissable canvas shown in its own window while the
    (simulated) arm movement is in progress -- overlaying small text on the
    already-open camera feed was too easy to miss, so this pops up as a
    separate window instead, like bomi_grasp's confirm dialog."""
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


def _wait_for_pre_grasp_pose(cap, landmarker, bomi_map, cursor_filter, crs_x, crs_y,
                              duration: float = PRE_GRASP_MOVE_DURATION):
    """Stands in for _goto_pre_grasp_pose(reachy) + the real
    _wait_for_pre_grasp_pose: there's no real arm to move, so this is just a
    timeout, but shows the same dedicated waiting window (with a progress
    bar) for `duration` seconds. Returns the possibly updated cursor
    position and whether the user quit."""
    print(f"[TEST] Simulating both arms going to the pre-grasping pose "
          f"(elbow pitch {PRE_GRASP_ELBOW_PITCH_DEG:.0f} deg, duration={duration:.1f}s)...")
    start = time.time()
    finish_at = start + duration

    while True:
        frame, crs_x, crs_y, _ = _update_cursor(cap, landmarker, bomi_map, cursor_filter, crs_x, crs_y)
        if frame is not None:
            cv2.imshow(CAM_WINDOW_NAME, frame)

        progress = min((time.time() - start) / duration, 1.0)
        cv2.imshow(WAIT_WINDOW_NAME, _draw_wait_canvas(progress))

        key = cv2.waitKey(1) & 0xFF
        if _quit_requested(key, CAM_WINDOW_NAME) or _quit_requested(key, WAIT_WINDOW_NAME):
            cv2.destroyWindow(WAIT_WINDOW_NAME)
            return crs_x, crs_y, True

        if time.time() >= finish_at:
            cv2.destroyWindow(WAIT_WINDOW_NAME)
            return crs_x, crs_y, False


def _object_selection_stub() -> None:
    """Stands in for _run_grasp_mode(...): there's no real depth camera to
    capture from in test mode, so this fails exactly like the real
    grasp._capture_and_detect does when depth_cam.get_frame() returns None
    -- nothing is shown, matching what happens without a robot camera."""
    print("[ERROR] Could not capture a frame from the camera "
          "(test mode: no depth camera available)")


def _control_phase(cap, landmarker, bomi_map: BoMIMap, cursor_filter: CursorFilter = None,
                    crs_x: float = None, crs_y: float = None) -> None:
    # Pass in the cursor_filter/crs_x/crs_y from the preceding cursor preview
    # to continue them (no filter reset), avoiding a velocity blip on entry.
    dt = 1.0 / PUBLISH_HZ
    last_publish = time.time()
    cursor_filter = cursor_filter or CursorFilter()

    if crs_x is None or crs_y is None:
        crs_x, crs_y = BASE_WIDTH / 2.0, BASE_HEIGHT / 2.0
    region = check_region_cursor(crs_x, crs_y)
    message = "lin_vel:0.000 ang_vel:0.000"
    center_hold_start = None
    speed_scale = 1.0
    pre_grasp_reached = False

    print("\n=== CONTROL (TEST - nothing is sent to a robot) ===  Q = quit  |  "
          f"hold the cursor centered (region 5) for {SELECTION_HOLD_SECONDS:.0f}s "
          "to move to the pre-grasping pose")

    while True:
        frame, crs_x, crs_y, hand_detected = _update_cursor(cap, landmarker, bomi_map, cursor_filter, crs_x, crs_y)
        if frame is None:
            continue

        if hand_detected:
            region = check_region_cursor(crs_x, crs_y)
            lin_vel, ang_vel = compute_dynamic_vel_from_cursor(crs_x, crs_y)
            lin_vel, ang_vel = apply_region_velocity_mask(region, lin_vel, ang_vel)
            lin_vel *= speed_scale
            ang_vel *= speed_scale
        else:
            lin_vel, ang_vel = 0.0, 0.0

        now = time.time()
        center_hold_start = (center_hold_start or now) if (hand_detected and region == 5) else None
        center_progress = (
            min((now - center_hold_start) / SELECTION_HOLD_SECONDS, 1.0) if center_hold_start else 0.0
        )

        cv2.putText(frame, f"region={region}  cursor=({crs_x:.0f},{crs_y:.0f})",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(frame, f"-> (test) would send: {message}",
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        if center_progress > 0:
            hold_label = "hold to select object" if pre_grasp_reached else "hold for pre-grasping pose"
            cv2.putText(frame, f"{hold_label}: {center_progress * 100:.0f}%",
                        (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, COLOR_CURSOR, 2)

        cv2.imshow(CAM_WINDOW_NAME, frame)
        cv2.imshow(MAP_WINDOW_NAME, _draw_cursor_map(crs_x, crs_y, region, message))

        if center_progress >= 1.0 and not pre_grasp_reached:
            print("[TEST] lin_vel=+0.000 m/s   ang_vel=+0.000 rad/s   stop (dwell reached)")
            crs_x, crs_y, quit_now = _wait_for_pre_grasp_pose(
                cap, landmarker, bomi_map, cursor_filter, crs_x, crs_y,
            )
            if quit_now:
                break

            crs_x, crs_y = _cursor_preview_phase(
                cap, landmarker, bomi_map, cursor_filter=cursor_filter, crs_x=crs_x, crs_y=crs_y,
            )

            pre_grasp_reached = True
            speed_scale = HALVED_SPEED_FACTOR
            center_hold_start = None
            print("\n[TEST] Control resumed at half speed — hold the cursor centered "
                  f"(region 5) for {SELECTION_HOLD_SECONDS:.0f}s to switch to object selection")
            continue

        if center_progress >= 1.0 and pre_grasp_reached:
            print("[TEST] lin_vel=+0.000 m/s   ang_vel=+0.000 rad/s   stop (final dwell reached)")
            print("\n[TEST] Switching to object selection...")
            _object_selection_stub()
            break

        if now - last_publish >= dt:
            message = f"lin_vel:{lin_vel:.3f} ang_vel:{ang_vel:.3f}"
            print(f"[TEST] lin_vel={lin_vel:+.3f} m/s   ang_vel={ang_vel:+.3f} rad/s "
                  f"({math.degrees(ang_vel):+.3f} deg/s)   region={region}   "
                  f"speed_scale={speed_scale:.1f}")
            last_publish = now

        key = cv2.waitKey(1) & 0xFF
        if _quit_requested(key, CAM_WINDOW_NAME) or _quit_requested(key, MAP_WINDOW_NAME):
            break

    print("[TEST] lin_vel=+0.000 m/s   ang_vel=+0.000 rad/s   stop")
    cv2.destroyWindow(CAM_WINDOW_NAME)
    cv2.destroyWindow(MAP_WINDOW_NAME)


# --- Entry point ---

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Dry-run of bomi_selection_complete.py's full flow (calibration, "
                     "cursor preview, Control, pre-grasping pose, object-selection stub), "
                     "logging instead of sending anything to a robot."
    )
    parser.add_argument("--cam", type=int, default=0, help="Webcam index (default: 0)")
    parser.add_argument("--model", default=DEFAULT_MODEL_PATH,
                        help="Path to the MediaPipe hand_landmarker.task model "
                             f"(default: {DEFAULT_MODEL_PATH}).")
    cli_args = parser.parse_args()

    if not os.path.exists(cli_args.model):
        print(f"[ERROR] MediaPipe model not found: '{cli_args.model}'")
        print("        Download hand_landmarker.task and pass its path with --model.")
        sys.exit(1)

    cap = None
    landmarker = None
    try:
        cap = cv2.VideoCapture(cli_args.cam)
        if not cap.isOpened():
            print(f"[ERROR] Cannot open camera {cli_args.cam}")
            sys.exit(1)

        landmarker_options = hand_landmarker.HandLandmarkerOptions(
            base_options=base_options.BaseOptions(model_asset_path=cli_args.model),
            running_mode=vision_task_running_mode.VisionTaskRunningMode.VIDEO,
            num_hands=1,
            min_hand_detection_confidence=0.7,
            min_hand_presence_confidence=0.5,
            min_tracking_confidence=0.5,
        )
        landmarker = hand_landmarker.HandLandmarker.create_from_options(landmarker_options)

        bomi_map = BoMIMap()
        samples = _calibration_phase(cap, landmarker)
        bomi_map.fit(samples)
        print("PCA map fitted")

        cursor_filter = CursorFilter()
        crs_x, crs_y = _cursor_preview_phase(cap, landmarker, bomi_map, cursor_filter=cursor_filter)
        _control_phase(cap, landmarker, bomi_map, cursor_filter=cursor_filter, crs_x=crs_x, crs_y=crs_y)
    finally:
        if cap is not None:
            cap.release()
        cv2.destroyAllWindows()
        if landmarker is not None:
            landmarker.close()


if __name__ == "__main__":
    main()
