#!/usr/bin/env python3
"""
BoMI teleop building blocks for Reachy2: hand tracking (MediaPipe) to a
calibrated PCA cursor to 9-region base velocity, over reachy2_sdk
(gRPC/IP) -- no ROS 2 required on this machine, so its ROS 2 distro (if
any) doesn't need to match the robot's.

Library module, no CLI of its own -- reachy_control.py is the real entry
point and imports these pieces (calibration/cursor-preview phases, the
BoMI map, cursor filter, velocity helpers, drawing helpers). For a
teleop-only dry run with no robot, see tests/test_teleop.py -- it can't
just import this module instead, since `import safety` below pulls in
reachy2_sdk (needed for its ReachySDK type hints), so tests/test_teleop.py
duplicates the relevant pieces by hand to stay installable without it.

Phase 1 - Calibration (_calibration_phase): move your hand through all
    positions you intend to use. SPACE = record sample, ENTER = finish
    (min 30 samples required).

Phase 2 - Cursor preview (_cursor_preview_phase): same cursor/region view
    as Control, but nothing is sent to the robot -- lets the caller get a
    feel for the cursor before it starts driving anything.
"""

import os
import sys
import time

import cv2
import mediapipe as mp
import numpy as np
import scipy.signal as sgn
from mediapipe.tasks.python.vision import hand_landmarker
from sklearn.decomposition import PCA

import safety

HAND_CONNECTIONS = hand_landmarker.HandLandmarksConnections.HAND_CONNECTIONS

# --- Lidar safety ---
LIDAR_SLOWDOWN_DISTANCE = 0.7   # m
LIDAR_CRITICAL_DISTANCE = 0.55  # m

# --- Virtual screen dimensions ---
BASE_WIDTH = 2550
BASE_HEIGHT = 1500

# Below MIN_LINEAR/MIN_ANGULAR the mobile base's wheels don't overcome
# their own resistance and simply don't move -- so once a component leaves
# its dead zone it ramps from min_* (at the dead-zone edge) straight to
# max_* (at the screen edge), not from 0.
MIN_LINEAR = 0.15      # m/s
MAX_LINEAR = 0.5      # m/s
MIN_ANGULAR = 0.6     # rad/s
MAX_ANGULAR = 1.1     # rad/s
DEAD_ZONE_PX = 200    # pixel radius around screen center before motion starts

PUBLISH_HZ = 20  # speed-command rate (Hz) — comfortably under the mobile base's 0.2s command duration

# Cursor low-pass filter: 3rd-order
# Butterworth, coefficients derived from the actual control loop rate below
# rather than hardcoded for 50Hz.
CURSOR_FILTER_HZ = 30.0        # assumed webcam/control loop sample rate
CURSOR_FILTER_CUTOFF_HZ = 4.0  # cutoff frequency

# MediaPipe Tasks hand-landmarker model (.task), lives at the package root by default.
DEFAULT_MODEL_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "hand_landmarker.task"
)

# Placeholder — replace with the robot's actual IP.
DEFAULT_ROBOT_IP = "192.168.0.120"

# Shared window names for the cursor preview and control phases, so the same
# OS windows stay open across the transition instead of closing and reopening.
CAM_WINDOW_NAME = "BoMI - Camera"
MAP_WINDOW_NAME = "BoMI - Cursor Map"


# --- Velocity helpers (adapted from reaching_functions.py) ---------

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


def _ramped_axis_velocity(delta: float, half_extent: float, dead_zone_px: float, min_v: float, max_v: float) -> float:
    """Signed velocity for one axis: 0 inside dead_zone_px of center, then
    ramps linearly from min_v (right at the dead-zone edge) to max_v (at
    the screen edge) -- not from 0, since below min_v the wheels don't
    move at all (see MIN_LINEAR/MIN_ANGULAR)."""
    norm = delta / half_extent
    dead_zone_norm = dead_zone_px / half_extent
    magnitude = abs(norm)
    if magnitude <= dead_zone_norm:
        return 0.0
    t = min((magnitude - dead_zone_norm) / (1.0 - dead_zone_norm), 1.0)
    sign = 1.0 if norm > 0 else -1.0
    return sign * (min_v + t * (max_v - min_v))


def compute_dynamic_vel_from_cursor(
    crs_x: float,
    crs_y: float,
    min_linear: float = MIN_LINEAR,
    max_linear: float = MAX_LINEAR,
    min_angular: float = MIN_ANGULAR,
    max_angular: float = MAX_ANGULAR,
    dead_zone_px: float = DEAD_ZONE_PX,
    ang_right_is_negative: bool = True,
) -> tuple:
    """
    Linear/angular velocity from cursor position, each axis independent
    (so a corner region gets both ramps at once): 0 inside dead_zone_px of
    screen center on that axis, then min_*..max_* linearly from there to
    the screen edge -- see _ramped_axis_velocity.
    Up from center -> positive linear; right from center -> negative angular.
    """
    cx = BASE_WIDTH / 2.0
    cy = BASE_HEIGHT / 2.0
    dx = crs_x - cx
    dy = crs_y - cy

    lin_vel = _ramped_axis_velocity(-dy, cy, dead_zone_px, min_linear, max_linear)  # up = positive
    ang_sign = -1.0 if ang_right_is_negative else 1.0
    ang_vel = ang_sign * _ramped_axis_velocity(dx, cx, dead_zone_px, min_angular, max_angular)
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


# --- PCA forward map ---
def _extract_hand_features(hand_landmarks, mirror_x: bool = False) -> np.ndarray:
    """
    Flatten all 21 hand landmarks (x, y) into a 42-element vector.
    If mirror_x, x is mirrored (1 - x) so the right hand maps to the same
    feature space as the left hand.

    hand_landmarks is the list of NormalizedLandmark returned by MediaPipe
    Tasks (e.g. results.hand_landmarks[0]).
    """
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


def _draw_cursor_map(crs_x: float, crs_y: float, region: int, message: str,
                      map_width: int = 850, map_height: int = 500):
    """Rectangle representing the BASE_WIDTH x BASE_HEIGHT virtual screen, with
    the 9-region grid lines, a dot at the current cursor position, and the
    lin_vel/ang_vel message currently being sent to the mobile base."""
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
    cv2.putText(canvas, f"-> mobile base: {message}",
                (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    return canvas


class CursorFilter:
    """
    3rd-order Butterworth low-pass filter for the (crs_x, crs_y) cursor position.
    Coefficients are derived from the actual sample rate (CURSOR_FILTER_HZ) instead
    of being hardcoded for a fixed frequency loop.
    """

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

    def reset(self, crs_x: float, crs_y: float) -> None:
        """Reinit history to steady-state at (crs_x, crs_y), so tracking
        resumed after a gap doesn't overshoot on stale samples."""
        steady = np.array([crs_x, crs_y])
        self._in_history[:] = steady
        self._out_history[:] = steady


class BoMIMap:
    """
    PCA forward map: raw hand landmarks -> 2D cursor in screen space.

    PCA(2 components) fitted directly on the raw landmark samples. Scale/offset
    map the calibration scores' peak-to-peak range onto the screen size, centered
    on the mean.
    """

    def __init__(self) -> None:
        self._mean = None         # PCA mean (42,)
        self._components = None   # PCA components (2, 42)
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

        # Store the plain arrays needed for inference (decoupled from the sklearn object)
        self._mean = pca.mean_
        self._components = pca.components_
        self._scale = screen / extent
        self._offset = screen / 2.0 - (scores * self._scale).mean(axis=0)
        self.fitted = True

    def transform(self, features: np.ndarray) -> tuple:
        """
        Linear BoMI map: raw hand landmarks -> 2D cursor in screen space.
        Returns (crs_x, crs_y) in pixels, clipped to the screen size
        """
        cu = np.dot(features - self._mean, self._components.T) # Linear projection of the features onto the PCA components
        cu = cu * self._scale + self._offset # Scaling operation to map the PCA scores to the screen size
        crs_x = float(np.clip(cu[0], 0, BASE_WIDTH))
        crs_y = float(np.clip(cu[1], 0, BASE_HEIGHT))
        return crs_x, crs_y


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

        elif safety.quit_requested(key, window_name):
            print("Aborted.")
            sys.exit(0)

    cv2.destroyWindow(window_name)
    return samples

def _cursor_preview_phase(cap, landmarker, bomi_map: BoMIMap, cursor_filter: CursorFilter = None,
                           crs_x: float = None, crs_y: float = None, show_cam: bool = True,
                           on_frame=None, hold_seconds: float = 5.0) -> tuple:
    """
    Shows the same cursor/region view as the control phase, but never talks to
    the robot. Lets the user get a feel for the cursor and see where it starts
    out before enabling motion.

    Advances to Control once the cursor dwells in region 5 (center) for
    hold_seconds straight, same dwell mechanism (accrue while centered,
    reset otherwise) as reachy_control.py's own SELECTION_HOLD_SECONDS
    switches and the button hovers in reachy_detection.py's confirm/refresh
    UI -- not a keypress.

    Pass an existing cursor_filter/crs_x/crs_y to continue them (no filter
    reset) instead of starting fresh; returns the final (crs_x, crs_y).
    show_cam=False skips the raw camera/landmarks window entirely (still
    tracks the hand, just doesn't display it), for callers that only want
    the cursor map on screen. on_frame, if given, is called with no
    arguments once per loop iteration -- e.g. reachy_control.py uses it to
    keep the robot's torso camera window live during this phase, without
    this module needing to know anything about depth cameras.
    """
    cursor_filter = cursor_filter or CursorFilter()
    cam_window = CAM_WINDOW_NAME
    map_window = MAP_WINDOW_NAME

    if crs_x is None or crs_y is None:
        crs_x, crs_y = BASE_WIDTH / 2.0, BASE_HEIGHT / 2.0
    region = check_region_cursor(crs_x, crs_y)
    center_hold_start = None

    print("\n=== CURSOR PREVIEW (robot not moving) ===")
    print(f"Get a feel for the cursor. Hold it centered (region 5) for {hold_seconds:.0f}s "
          "to start Control   |   Q = quit")

    while True:
        ret, frame = cap.read()
        if not ret:
            continue
        frame = cv2.flip(frame, 1)
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_frame)
        results = landmarker.detect_for_video(mp_image, int(time.time() * 1000))

        hand_detected = bool(results.hand_landmarks)
        if hand_detected:
            hl = results.hand_landmarks[0]
            _draw_hand_landmarks(frame, hl)
            mirror_x = results.handedness[0][0].category_name == "Right"
            crs_x, crs_y = bomi_map.transform(_extract_hand_features(hl, mirror_x))
            crs_x, crs_y = cursor_filter.update(crs_x, crs_y)
            region = check_region_cursor(crs_x, crs_y)

        now = time.time()
        # Only accrue while actively tracked and centered, so a dropped
        # hand while the stale cursor happens to sit in region 5 can't
        # silently trigger the switch.
        center_hold_start = (center_hold_start or now) if (hand_detected and region == 5) else None
        center_progress = min((now - center_hold_start) / hold_seconds, 1.0) if center_hold_start else 0.0

        cv2.putText(
            frame, f"region={region}  cursor=({crs_x:.0f},{crs_y:.0f})",
            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2,
        )
        cv2.putText(
            frame, f"PREVIEW - robot not moving. Hold centered: {center_progress * 100:.0f}%  Q=quit",
            (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2,
        )

        if show_cam:
            cv2.imshow(cam_window, frame)
        cv2.imshow(map_window, _draw_cursor_map(crs_x, crs_y, region, "(preview - not sent)"))
        if on_frame is not None:
            on_frame()

        key = cv2.waitKey(1) & 0xFF
        if center_progress >= 1.0:
            break
        if (show_cam and safety.quit_requested(key, cam_window)) or safety.quit_requested(key, map_window):
            print("Aborted.")
            sys.exit(0)

    # Windows are intentionally left open (no destroyWindow) so the same cam/map
    # windows carry straight into the control phase instead of flickering shut.
    return crs_x, crs_y


