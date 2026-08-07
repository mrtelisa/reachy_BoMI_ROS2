#!/usr/bin/env python3
"""
BoMI grasp building blocks for Reachy2: YOLOv8 detection on the torso
camera, depth point-cloud pipeline (frame fusion, distortion correction,
table-plane/flying-pixel removal, shape-specific dimension/pose fit), over
reachy2_sdk (gRPC/IP).

Library module, no CLI of its own -- reachy_control.py is the real entry
point and imports these pieces (_capture_and_detect, _build_object_point_cloud,
drawing/hover-detection helpers). For the mouse-driven select/confirm flow
(no BoMI cursor), see tests/test_grasp.py; for a live size-estimation
diagnostic, see tests/test_object_dimensions.py.

Bounding boxes are blue by default. Hovering one for HOVER_HOLD_SECONDS
straight turns it green (see _find_hovered_detection/COLOR_*).
"""

import time
from typing import List, Optional, Tuple

import cv2
import numpy as np
import open3d as o3d
from reachy2_sdk import ReachySDK
from reachy2_sdk.media.camera import CameraView, DepthCamera
from reachy2_sdk.utils.utils import invert_affine_transformation_matrix
from ultralytics import YOLO

import graphs
import reachy_grasp
import safety

# Placeholder — replace with the robot's actual IP.
DEFAULT_ROBOT_IP = "192.168.0.121"

CAM_WINDOW_NAME = "BoMI - Depth Camera (RGB)"

YOLO_MODEL_PATH = "yolov8n.pt"
YOLO_CONFIDENCE = 0.5

# Curated subset of COCO classes small/light enough for Reachy's gripper.
GRASPABLE_CLASSES = {
    "bottle", "cup", "wine glass", "banana", "apple", "orange",
    "spoon", "fork", "knife", "teddy bear", "toothbrush", "vase",
}

# Rough shape prior from the YOLO class, used by _object_dimensions to pick
# the circle (cylinder) or sphere fit that corrects for a single view only
# ever seeing part of a round object. Only "cylinder"/"sphere" are
# supported -- GRASPABLE_CLASSES is limited to round objects for exactly
# this reason.
SHAPE_BY_CLASS = {
    "bottle": "cylinder", "cup": "cylinder", "wine glass": "cylinder", 
    "banana": "cylinder", "apple": "sphere", "orange": "sphere",
    "spoon": "cylinder", "fork": "cylinder", "knife": "cylinder", 
    "teddy bear": "cylinder", "toothbrush": "cylinder", "vase": "cylinder",
}

DEFAULT_SHAPE = "cylinder"

def shape_from_class(class_name: str) -> str:
    """Rough shape prior from the YOLO class name; defaults to cylinder for
    anything not in the table."""
    return SHAPE_BY_CLASS.get(class_name, DEFAULT_SHAPE)


Detection = Tuple[str, float, Tuple[int, int, int, int]]  # class_name, confidence, (x1, y1, x2, y2)
Box = Tuple[int, int, int, int]

# BGR colors
COLOR_BLUE = (255, 0, 0)
COLOR_YELLOW = (0, 255, 255)
COLOR_GREEN = (0, 255, 0)

HOVER_HOLD_SECONDS = 5.0
HOVER_IOU_MATCH = 0.3  # min overlap between frames to count as "still hovering the same object"

# Depth frames fused (per-pixel median) before building the point cloud, so
# flickering specular-reflection dropouts in any single frame get filled in
# by frames where that pixel did register. Camera and object are assumed
# static during this.
DEPTH_ACCUMULATION_FRAMES = 10

REFRESH_BUTTON_BOX: Box = (10, 10, 260, 90)
COLOR_BUTTON = (0, 0, 255)
COLOR_BUTTON_TEXT = (255, 255, 255)
REFRESH_HOVER_SECONDS = 5.0

CONFIRM_WINDOW_NAME = "BoMI - Confirm Grasp"
CONFIRM_CANVAS_WIDTH = 520
CONFIRM_CANVAS_HEIGHT = 260
CONFIRM_HOVER_SECONDS = 5.0
YES_BUTTON_BOX: Box = (60, 150, 240, 220)
NO_BUTTON_BOX: Box = (280, 150, 460, 220)
COLOR_YES = (0, 200, 0)
COLOR_NO = (0, 0, 255)

class _MouseTracker:
    """Tracks the latest mouse position over the given OpenCV window."""

    def __init__(self, window_name: str) -> None:
        self.x = -1
        self.y = -1
        cv2.namedWindow(window_name)
        cv2.setMouseCallback(window_name, self._on_mouse)

    def _on_mouse(self, event: int, x: int, y: int, flags: int, param: object) -> None:
        self.x, self.y = x, y


def _box_contains(box: Box, x: int, y: int) -> bool:
    x1, y1, x2, y2 = box
    return x1 <= x <= x2 and y1 <= y <= y2


def _iou(box_a: Box, box_b: Box) -> float:
    ax1, ay1, ax2, ay2 = box_a
    bx1, by1, bx2, by2 = box_b
    inter_w = max(0, min(ax2, bx2) - max(ax1, bx1))
    inter_h = max(0, min(ay2, by2) - max(ay1, by1))
    inter_area = inter_w * inter_h
    if inter_area == 0:
        return 0.0
    area_a = (ax2 - ax1) * (ay2 - ay1)
    area_b = (bx2 - bx1) * (by2 - by1)
    return inter_area / float(area_a + area_b - inter_area)


def _find_hovered_detection(detections: List[Detection], x: int, y: int) -> Optional[Detection]:
    """Smallest-area detection whose box contains (x, y), or None if none does."""
    hovered = None
    hovered_area = None
    for detection in detections:
        _, _, box = detection
        if not _box_contains(box, x, y):
            continue
        x1, y1, x2, y2 = box
        area = (x2 - x1) * (y2 - y1)
        if hovered_area is None or area < hovered_area:
            hovered, hovered_area = detection, area
    return hovered


def _detect_graspable_objects(model: YOLO, frame: np.ndarray, confidence: float) -> List[Detection]:
    """Run YOLO on frame, keeping only detections in GRASPABLE_CLASSES above confidence."""
    results = model(frame, verbose=False)[0]
    detections = []
    for box in results.boxes:
        class_name = model.names[int(box.cls[0])]
        conf = float(box.conf[0])
        if class_name not in GRASPABLE_CLASSES or conf < confidence:
            continue
        x1, y1, x2, y2 = (int(v) for v in box.xyxy[0].tolist())
        detections.append((class_name, conf, (x1, y1, x2, y2)))
    return detections


# Radius (px) of the square patch _estimate_object_position/_estimate_object_width
# search around their target pixel for a valid depth reading, median'd for
# robustness. Not 0 (exact pixel): dark/glossy round objects can have a
# depth hole exactly at a box's center from a specular/low-albedo dropout
# (the same failure mode as the missing-center-of-a-dark-cup case) -- a
# single unlucky pixel there would otherwise fail both the reachability
# and gripper-size pre-filters (_capture_and_detect) regardless of
# whether the object actually is reachable/gripper-sized.
CHEAP_DEPTH_SEARCH_RADIUS_PX = 15


def _median_valid_depth_mm(depth_frame: np.ndarray, u: int, v: int, radius: int) -> Optional[float]:
    """Median raw depth reading (mm) over a (2*radius+1) square patch
    centered at (u, v), ignoring invalid (<=0) readings -- None only if
    every pixel in the patch is invalid, not just (u, v) itself."""
    height, width = depth_frame.shape
    y_lo, y_hi = max(v - radius, 0), min(v + radius + 1, height)
    x_lo, x_hi = max(u - radius, 0), min(u + radius + 1, width)
    patch = depth_frame[y_lo:y_hi, x_lo:x_hi]
    valid_mm = patch[patch > 0]
    if valid_mm.size == 0:
        return None
    return float(np.median(valid_mm))


def _estimate_object_position(
    depth_cam: DepthCamera, depth_frame: np.ndarray, u: int, v: int,
    search_radius: int = CHEAP_DEPTH_SEARCH_RADIUS_PX,
) -> Optional[np.ndarray]:
    """XYZ position (meters, Reachy coordinate system) near pixel (u, v) --
    the depth used is the median over a search_radius patch there (see
    CHEAP_DEPTH_SEARCH_RADIUS_PX), not just (u, v) itself. None only if
    no pixel in that whole patch has valid depth."""
    height, width = depth_frame.shape
    if not (0 <= v < height and 0 <= u < width):
        return None

    depth_mm = _median_valid_depth_mm(depth_frame, u, v, search_radius)
    if depth_mm is None:
        return None

    return depth_cam.pixel_to_world(u, v, z_c=depth_mm / 1000.0, view=CameraView.LEFT)


_cached_fx_px: Optional[float] = None


def _get_focal_length_px(depth_cam: DepthCamera) -> float:
    """LEFT view's horizontal focal length (pixels), cached after the first
    call -- intrinsics are fixed for a given camera, so there's no reason
    to re-fetch them (a network round-trip, like other DepthCamera calls)
    on every detection."""
    global _cached_fx_px
    if _cached_fx_px is None:
        _, _, _, _, K, _, _ = depth_cam.get_parameters(view=CameraView.LEFT)
        _cached_fx_px = float(K[0, 0])
    return _cached_fx_px


def _estimate_object_width(
    depth_cam: DepthCamera, depth_frame: np.ndarray, box: Box,
    search_radius: int = CHEAP_DEPTH_SEARCH_RADIUS_PX,
) -> Optional[float]:
    """Cheap real-world width estimate (meters) via the pinhole projection
    formula (width_m = box_width_px * depth_m / fx), using the median
    depth around the box's center (see _median_valid_depth_mm/
    CHEAP_DEPTH_SEARCH_RADIUS_PX) -- not its left/right edges (an earlier
    version did, and read badly: the object's silhouette boundary is
    exactly where a depth sensor is least reliable -- flying pixels,
    grazing angle, foreground/background transition). Still rougher than
    the point-cloud-based width_m (_object_dimensions: PCA + circle/sphere
    fit correcting for viewing angle and partial-view bias) and inherits
    however loose YOLO's box is around the object, but only needs to catch
    "obviously too wide for the gripper" before the user selects it, not
    the true width."""
    x1, y1, x2, y2 = box
    cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
    depth_mm = _median_valid_depth_mm(depth_frame, cx, cy, search_radius)
    if depth_mm is None:
        return None
    depth_m = depth_mm / 1000.0

    fx = _get_focal_length_px(depth_cam)
    box_width_px = x2 - x1
    return box_width_px * depth_m / fx


def _label_for_detection(class_name: str, conf: float, position: Optional[np.ndarray]) -> str:
    label = f"{class_name} {conf:.2f}"
    if position is not None:
        label += f"  xyz=({position[0]:.2f},{position[1]:.2f},{position[2]:.2f})m"
    return label


def _draw_box(
    frame: np.ndarray, box: Box, label: str, color: Tuple[int, int, int], progress: float = 0.0,
) -> None:
    x1, y1, x2, y2 = box
    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
    cv2.putText(frame, label, (x1, max(y1 - 10, 0)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
    if progress > 0:
        bar_width = int((x2 - x1) * progress)
        cv2.rectangle(frame, (x1, y2 - 6), (x1 + bar_width, y2), color, -1)


def _draw_refresh_button(frame: np.ndarray, progress: float = 0.0) -> None:
    x1, y1, x2, y2 = REFRESH_BUTTON_BOX
    cv2.rectangle(frame, (x1, y1), (x2, y2), COLOR_BUTTON, -1)
    cv2.rectangle(frame, (x1, y1), (x2, y2), COLOR_BUTTON_TEXT, 1)

    text = "Refresh"
    font, scale, thickness = cv2.FONT_HERSHEY_SIMPLEX, 1.1, 2
    (text_w, text_h), _ = cv2.getTextSize(text, font, scale, thickness)
    text_x = x1 + ((x2 - x1) - text_w) // 2
    text_y = y1 + ((y2 - y1) + text_h) // 2
    cv2.putText(frame, text, (text_x, text_y), font, scale, COLOR_BUTTON_TEXT, thickness)

    if progress > 0:
        bar_width = int((x2 - x1) * progress)
        cv2.rectangle(frame, (x1, y2 - 8), (x1 + bar_width, y2), COLOR_BUTTON_TEXT, -1)


def _draw_confirm_canvas(class_name: str, yes_progress: float, no_progress: float) -> np.ndarray:
    canvas = np.full((CONFIRM_CANVAS_HEIGHT, CONFIRM_CANVAS_WIDTH, 3), 30, dtype=np.uint8)

    cv2.putText(canvas, f"You have select the {class_name} to be grasped.",
                (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.55, COLOR_BUTTON_TEXT, 1)
    cv2.putText(canvas, "Do you want to confirm?",
                (20, 95), cv2.FONT_HERSHEY_SIMPLEX, 0.6, COLOR_BUTTON_TEXT, 2)

    for box, label, color, progress in (
        (YES_BUTTON_BOX, "Yes", COLOR_YES, yes_progress),
        (NO_BUTTON_BOX, "No", COLOR_NO, no_progress),
    ):
        x1, y1, x2, y2 = box
        cv2.rectangle(canvas, (x1, y1), (x2, y2), color, -1)
        cv2.rectangle(canvas, (x1, y1), (x2, y2), COLOR_BUTTON_TEXT, 1)
        cv2.putText(canvas, label, (x1 + 60, y2 - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.9, COLOR_BUTTON_TEXT, 2)
        if progress > 0:
            bar_width = int((x2 - x1) * progress)
            cv2.rectangle(canvas, (x1, y2 - 8), (x1 + bar_width, y2), COLOR_BUTTON_TEXT, -1)

    return canvas


Capture = Tuple[np.ndarray, List[Detection], dict]


def _capture_and_detect(
    depth_cam: DepthCamera, model: YOLO, confidence: float, reachy: Optional[ReachySDK] = None,
) -> Optional[Capture]:
    """Grab one RGB + depth frame and run YOLO once, returning the frame,
    detections, and their labels.

    If reachy is given, detections are pre-filtered down to the ones
    reachy_grasp.is_roughly_reachable accepts from their single-pixel depth
    position -- objects the robot has no plausible line to never get a box
    drawn or become selectable in the first place, instead of the user
    finding out only after confirming and building the full point cloud.
    The gripper-size check (_estimate_object_width <=
    reachy_grasp.GRIPPER_MAX_OPENING_M) is still computed but currently
    commented out of the filter below -- re-enable it there if too-wide
    objects start showing up as selectable again. reachy=None (e.g.
    tests/test_object_dimensions.py, which wants every detection
    regardless) skips this entirely."""
    result = depth_cam.get_frame(view=CameraView.LEFT)
    if result is None:
        print("[ERROR] Could not capture a frame from the camera")
        return None
    base_frame, _timestamp = result

    depth_result = depth_cam.get_depth_frame(view=CameraView.DEPTH)
    depth_frame = depth_result[0] if depth_result is not None else None

    detections = _detect_graspable_objects(model, base_frame, confidence)

    positions = {}
    widths = {}
    if depth_frame is not None:
        for _class_name, _conf, box in detections:
            x1, y1, x2, y2 = box
            u, v = (x1 + x2) // 2, (y1 + y2) // 2
            positions[box] = _estimate_object_position(depth_cam, depth_frame, u, v)
            widths[box] = _estimate_object_width(depth_cam, depth_frame, box)

    if reachy is not None:
        before = len(detections)
        detections = [
            detection for detection in detections
            if positions.get(detection[2]) is not None
            and reachy_grasp.is_roughly_reachable(reachy, positions[detection[2]])
            # Gripper-size check
            and widths.get(detection[2]) is not None
            and widths[detection[2]] <= reachy_grasp.GRIPPER_MAX_OPENING_M
        ]
        print(f"Reachability pre-filter: {before} detected -> {len(detections)} plausibly reachable")

    labels = {
        box: _label_for_detection(class_name, conf, positions.get(box))
        for class_name, conf, box in detections
    }
    return base_frame, detections, labels


# --- Point cloud pipeline (first step towards an actual grasp pose) ---
#
# Crop margin around the YOLO box: generous on purpose, not just to avoid
# clipping the object, but so _remove_table_plane's RANSAC fit has enough
# real table in the crop to reliably win against the object's own curved
# surface (see its docstring -- Table normal print is the check if that
# ever goes wrong).
BBOX_PADDING_PX = 60


def _pad_box(box: Box, padding: int, frame_shape: Tuple[int, int]) -> Box:
    """Grow box by padding pixels on every side, clamped to the frame."""
    height, width = frame_shape
    x1, y1, x2, y2 = box
    return (
        max(0, x1 - padding), max(0, y1 - padding),
        min(width, x2 + padding), min(height, y2 + padding),
    )


def _crop_depth_to_box(depth_frame: np.ndarray, box: Box) -> np.ndarray:
    x1, y1, x2, y2 = box
    return depth_frame[y1:y2, x1:x2]


def _fuse_depth_frames(depth_stack: List[np.ndarray]) -> np.ndarray:
    """Per-pixel median over valid (>0) readings across depth_stack, so a
    pixel dropped by specular reflection in some frames is filled in by the
    frames where it did register. Pixels invalid in every frame stay 0."""
    stacked = np.stack(depth_stack).astype(np.float32)
    stacked[stacked <= 0] = np.nan
    with np.errstate(all="ignore"):
        fused = np.nanmedian(stacked, axis=0)
    return np.nan_to_num(fused, nan=0.0).astype(depth_stack[0].dtype)


def _depth_crop_to_point_cloud(
    depth_cam: DepthCamera, depth_crop: np.ndarray, box: Box, correct_distortion: bool = True,
) -> np.ndarray:
    """3D points (Reachy coords, meters) for every valid pixel in depth_crop.
    Replicates pixel_to_world()'s math locally over all pixels at once
    instead of calling it per pixel (each call is 2 gRPC round trips).

    With correct_distortion (the default), lens distortion is corrected via
    cv2.undistortPointsIter -- pixel_to_world's own K_inv @ [u,v,1] doesn't,
    which matters increasingly towards the edges of this wide-FOV lens.
    correct_distortion=False replicates that same uncorrected math instead,
    only for callers plotting a "before correction" comparison (see
    commented-out plots in _build_object_point_cloud) -- never for an
    actual size/position estimate."""
    x1, y1, _, _ = box
    rows, cols = np.nonzero(depth_crop > 0)
    if rows.size == 0:
        return np.empty((0, 3))

    params = depth_cam.get_parameters(view=CameraView.LEFT)
    extrinsics = depth_cam.get_extrinsics(view=CameraView.LEFT)
    if params is None or extrinsics is None:
        return np.empty((0, 3))
    _, _, _, D, K, _, _ = params

    u = (x1 + cols).astype(np.float64)
    v = (y1 + rows).astype(np.float64)
    z_c = depth_crop[rows, cols].astype(np.float64) / 1000.0

    if correct_distortion:
        # undistortPointsIter over undistortPoints: the latter always runs
        # a fixed 5 iterations with no convergence check, leaving a
        # residual error at the edges of this wide-FOV lens; the Iter
        # variant takes a real stopping criterion (at the cost of passing
        # identity R/P explicitly, which undistortPoints defaults
        # internally when omitted).
        uv_points = np.stack([u, v], axis=1).reshape(-1, 1, 2)
        undistort_criteria = (cv2.TERM_CRITERIA_MAX_ITER + cv2.TERM_CRITERIA_EPS, 100, 1e-6)
        undistorted = cv2.undistortPointsIter(
            uv_points, K, D, np.eye(3), np.eye(3), undistort_criteria
        ).reshape(-1, 2)
    else:
        fx, cx = K[0, 0], K[0, 2]
        fy, cy = K[1, 1], K[1, 2]
        undistorted = np.stack([(u - cx) / fx, (v - cy) / fy], axis=1)
    camera_coords = np.hstack([undistorted, np.ones((undistorted.shape[0], 1))])

    camera_coords_homogeneous = np.stack(
        [camera_coords[:, 0] * z_c, camera_coords[:, 1] * z_c, z_c, np.ones_like(z_c)], axis=1,
    )
    world_coords = camera_coords_homogeneous @ invert_affine_transformation_matrix(extrinsics).T
    return world_coords[:, :3]


FLYING_PIXEL_NEIGHBORS = 16
FLYING_PIXEL_RADIUS_M = 0.02


def _remove_flying_pixels(
    point_cloud: np.ndarray, nb_points: int = FLYING_PIXEL_NEIGHBORS, radius: float = FLYING_PIXEL_RADIUS_M,
) -> np.ndarray:
    """Drops points with fewer than nb_points neighbors within radius --
    catches the diagonal 'bridge' of interpolated depth a sensor leaves
    between an object's edge and whatever is behind/around it, a real
    depth-sensor artifact rather than camera noise."""
    if point_cloud.shape[0] == 0:
        return point_cloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(point_cloud)
    cleaned, _ = pcd.remove_radius_outlier(nb_points=nb_points, radius=radius)
    return np.asarray(cleaned.points)


TABLE_PLANE_DISTANCE_M = 0.003


def _remove_table_plane(
    point_cloud: np.ndarray, distance_threshold: float = TABLE_PLANE_DISTANCE_M,
) -> Tuple[np.ndarray, Optional[np.ndarray]]:
    """RANSAC-fits the single dominant plane in point_cloud -- the table --
    and returns everything else, plus that plane's unit normal (oriented up
    and away from the table). Works regardless of how much of the cloud is
    table vs. object or the table's angle relative to the camera -- it just
    finds the largest flat surface. reachy_grasp uses the normal as its "up"
    direction. Returns (point_cloud, None) if too few points to fit."""
    if point_cloud.shape[0] < 4:
        return point_cloud, None
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(point_cloud)
    plane_model, inliers = pcd.segment_plane(distance_threshold=distance_threshold, ransac_n=3, num_iterations=2000)
    without_plane = pcd.select_by_index(inliers, invert=True)

    normal = np.array(plane_model[:3], dtype=np.float64)
    normal /= np.linalg.norm(normal)
    if normal[2] < 0:
        normal = -normal
    return np.asarray(without_plane.points), normal


CLUSTER_EPS_M = 0.012
CLUSTER_MIN_POINTS = 40


def _largest_cluster(point_cloud: np.ndarray, eps: float = CLUSTER_EPS_M, min_points: int = CLUSTER_MIN_POINTS) -> np.ndarray:
    """DBSCAN-splits point_cloud and keeps only its largest cluster: once the
    table plane is gone, this is what separates the actual object from any
    other non-planar leftovers in the box (e.g. a second object at a similar
    depth) -- a spatial check _remove_table_plane can't do on its own, since
    it only removes the one dominant plane, not arbitrary clutter."""
    if point_cloud.shape[0] < min_points:
        return point_cloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(point_cloud)
    labels = np.array(pcd.cluster_dbscan(eps=eps, min_points=min_points))
    if labels.max() < 0:
        return point_cloud
    largest_label = np.bincount(labels[labels >= 0]).argmax()
    return point_cloud[labels == largest_label]


def _fit_cylinder_circle(
    point_cloud: np.ndarray, centroid: np.ndarray, long_axis: np.ndarray, cross_axes: np.ndarray,
    slice_frac: float = 0.1,
) -> Optional[Tuple[np.ndarray, float]]:
    """True circle (center, radius) of a cylindrical object's cross-section,
    least-squares (Kasa) fit to a thin band of points around its mid-height
    -- not read off raw PCA extent or the point mean, both of which are
    biased since a single view only ever sees a partial arc of the
    circumference, never the full circle. Returns (center, radius) in the
    same 3D frame as point_cloud, or None if too few points in the band."""
    centered = point_cloud - centroid
    heights = centered @ long_axis
    lo, hi = np.percentile(heights, [5, 95])
    mid = (lo + hi) / 2
    band = slice_frac * (hi - lo)
    ring = centered[np.abs(heights - mid) < band / 2] @ cross_axes
    if ring.shape[0] < 20:
        return None

    x, y = ring[:, 0], ring[:, 1]
    design = np.column_stack([x, y, np.ones_like(x)])
    rhs = x ** 2 + y ** 2
    a, b, c = np.linalg.lstsq(design, rhs, rcond=None)[0]
    cx, cy = a / 2, b / 2
    radius = np.sqrt(c + cx ** 2 + cy ** 2)
    center = centroid + cx * cross_axes[:, 0] + cy * cross_axes[:, 1]
    return center, float(radius)


def _fit_sphere_center(point_cloud: np.ndarray) -> Optional[Tuple[np.ndarray, float]]:
    """True (center, radius) of a round object, same Kasa-style least-squares
    fit as _fit_cylinder_circle generalized to a sphere -- simpler, since a
    sphere is isotropic and needs no long axis/band, the whole point cloud
    is used directly. Returns None if too few points."""
    if point_cloud.shape[0] < 20:
        return None
    guess = point_cloud.mean(axis=0)
    x, y, z = (point_cloud - guess).T

    design = np.column_stack([2 * x, 2 * y, 2 * z, np.ones_like(x)])
    rhs = x ** 2 + y ** 2 + z ** 2
    cx, cy, cz, k = np.linalg.lstsq(design, rhs, rcond=None)[0]
    radius = np.sqrt(k + cx ** 2 + cy ** 2 + cz ** 2)
    center = guess + np.array([cx, cy, cz])
    return center, float(radius)


def _object_dimensions(
    point_cloud: np.ndarray, shape: str,
) -> Tuple[float, float, Optional[np.ndarray], Optional[np.ndarray]]:
    """Object (width_m, height_m, centroid, axes) from PCA on the isolated
    point cloud. height_m/width_m are the largest/2nd-largest extent
    (3rd dropped as unreliable "thickness" -- a single view never sees
    behind the object), except for "cylinder"/"sphere" -- the only shapes
    SHAPE_BY_CLASS produces -- where a circle/sphere fit
    (_fit_cylinder_circle / _fit_sphere_center) replaces width_m/height_m
    *and* centroid, correcting the same partial-view bias extent and
    circle/sphere fits both handle for a round object; GRASPABLE_CLASSES
    is round-objects-only for exactly this reason (a box's hidden depth
    can't be recovered the same way from one view).

    Extents use the 1st-99th percentile spread, not raw max-min, so a
    stray leftover point can't inflate the result. Returns (0.0, 0.0,
    None, None) if too few points remain to fit axes."""
    if point_cloud.shape[0] < 3:
        return 0.0, 0.0, None, None
    centroid = point_cloud.mean(axis=0)
    _, eigvecs = np.linalg.eigh(np.cov((point_cloud - centroid).T))
    proj = (point_cloud - centroid) @ eigvecs
    raw_extents = np.percentile(proj, 99, axis=0) - np.percentile(proj, 1, axis=0)
    order = np.argsort(raw_extents)[::-1]
    axes = eigvecs[:, order]
    extents = raw_extents[order]
    height_m, width_m = float(extents[0]), float(extents[1])

    if shape == "cylinder":
        circle = _fit_cylinder_circle(point_cloud, centroid, axes[:, 0], axes[:, 1:3])
        if circle is not None:
            centroid, radius = circle
            width_m = 2 * radius
    elif shape == "sphere":
        sphere = _fit_sphere_center(point_cloud)
        if sphere is not None:
            centroid, radius = sphere
            width_m = height_m = 2 * radius

    return width_m, height_m, centroid, axes


def _build_object_point_cloud(
    depth_cam: DepthCamera, class_name: str, box: Box, num_frames: int = DEPTH_ACCUMULATION_FRAMES,
) -> Optional[reachy_grasp.ObjectGeometry]:
    """First pipeline step once an object is confirmed: crop to its (padded)
    bbox and fuse num_frames depth readings (median per pixel), backproject
    every valid pixel to 3D, then isolate the object itself (RANSAC table-
    plane removal + DBSCAN largest-cluster) before measuring it (PCA
    principal axes, see _object_dimensions).
    Camera and object are assumed static during this.
    Returns an ObjectGeometry (see reachy_grasp) for reachy_grasp.plan_grasp
    to consume, or None if the user quit."""
    print(f"\n=== Building point cloud for '{class_name}' "
          f"({num_frames} depth frames) ===  Q = quit")

    padded_box = None
    depth_stack: List[np.ndarray] = []
    while len(depth_stack) < num_frames:
        rgb_result = depth_cam.get_frame(view=CameraView.LEFT)
        if rgb_result is not None:
            frame, _timestamp = rgb_result
            if padded_box is None:
                padded_box = _pad_box(box, BBOX_PADDING_PX, frame.shape[:2])
            _draw_box(frame, padded_box, class_name, COLOR_GREEN)
            cv2.putText(frame, f"Fusing depth frame {len(depth_stack) + 1}/{num_frames}...",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, COLOR_GREEN, 2)
            cv2.imshow(CAM_WINDOW_NAME, frame)

        depth_result = depth_cam.get_depth_frame(view=CameraView.DEPTH)
        if depth_result is not None and padded_box is not None:
            depth_frame, _timestamp = depth_result
            depth_stack.append(_crop_depth_to_box(depth_frame, padded_box))

        key = cv2.waitKey(1) & 0xFF
        if safety.quit_requested(key, CAM_WINDOW_NAME):
            return None

    fused_crop = _fuse_depth_frames(depth_stack)
    valid_before = 100.0 * np.mean(depth_stack[-1] > 0)
    valid_after = 100.0 * np.mean(fused_crop > 0)
    print(f"Valid depth pixels: {valid_before:.0f}% (single frame) -> "
          f"{valid_after:.0f}% (after {num_frames}-frame fusion)")

    # Diagnostic plots for the 4 pipeline stages are commented out below --
    # not needed for normal runs (the printed counts + table normal verdict
    # cover it), but handy for e.g. thesis figures. Uncomment to show.
    # raw_cloud = _depth_crop_to_point_cloud(depth_cam, fused_crop, padded_box, correct_distortion=False)
    # graphs.show_point_cloud(raw_cloud, f"{class_name} - 1 acquired")

    point_cloud = _depth_crop_to_point_cloud(depth_cam, fused_crop, padded_box)
    # graphs.show_point_cloud(point_cloud, f"{class_name} - 2 after removing distortion")

    points_before_isolation = len(point_cloud)
    point_cloud = _remove_flying_pixels(point_cloud)
    point_cloud, table_normal = _remove_table_plane(point_cloud)
    if table_normal is not None:
        tilt_deg = np.degrees(np.arccos(np.clip(table_normal[2], -1.0, 1.0)))
        verdict = "OK" if tilt_deg < 15 else "SUSPICIOUS -- expected close to vertical (see BBOX_PADDING_PX comment)"
        print(f"Table normal: ({table_normal[0]:.3f}, {table_normal[1]:.3f}, {table_normal[2]:.3f})  "
              f"{tilt_deg:.0f} deg from vertical  [{verdict}]")
    else:
        print("Table normal: not fitted (too few points) -- reachy_grasp will fall back to a vertical assumption")
    # graphs.show_point_cloud(point_cloud, f"{class_name} - 3 after background isolation")

    point_cloud = _largest_cluster(point_cloud)
    print(f"Point cloud: {points_before_isolation} points (distortion-corrected) -> "
          f"{len(point_cloud)} (table + flying pixels removed, largest cluster kept)")
    # graphs.show_point_cloud(point_cloud, f"{class_name} - 4 final")

    shape = shape_from_class(class_name)
    width_m, height_m, centroid, axes = _object_dimensions(point_cloud, shape)
    box_w_px, box_h_px = box[2] - box[0], box[3] - box[1]
    print(f"Box: {box_w_px}x{box_h_px}px (axis-aligned)  shape={shape}  -> "
          f"width={width_m * 100:.1f}cm  height={height_m * 100:.1f}cm "
          f"(3D PCA{' + circle fit' if shape == 'cylinder' else ''})")

    return reachy_grasp.ObjectGeometry(
        class_name=class_name, shape=shape, width_m=width_m, height_m=height_m,
        centroid=centroid, axes=axes, table_normal=table_normal, point_cloud=point_cloud,
    )


