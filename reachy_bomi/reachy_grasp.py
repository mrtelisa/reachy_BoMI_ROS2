"""
Reachy 2 - Grasp planning for CYLINDER, SPHERE, CUBE or BOX shaped objects.

Input: you have already selected a bbox containing the object (from the depth camera).
From bbox + depth you derive:
  - p_front : 3D point on the visible front surface, in the ROBOT frame
  - width_m : estimated horizontal width (meters) -> diameter (cyl/sphere) or side (cube/box)
  - height_m: estimated vertical height (meters)
  - shape   : "cylinder" | "sphere" | "cube" | "box"

Output: two 4x4 end-effector poses (pre-grasp and grasp), already validated:
  1) width <= gripper max opening (100 mm), otherwise ObjectTooWideError
  2) poses reachable by the IK, otherwise GraspNotReachableError

Strategy: horizontal SIDE grasp at mid-body. The gripper approaches the object
horizontally and closes its fingers around the body.

Note on BOX/rectangle: from a single depth view we only see the front face, so
the object's depth along the viewing direction is unknown. The width check uses
the OBSERVED width (the face perpendicular to the view). If a box is presented
with its long side facing the robot and that side exceeds the gripper opening,
it is rejected even though it might fit if approached from the short side --
a "gentle" failure. Handling that would require estimating the object's full
oriented bounding box (e.g. PCA on the face point cloud) and choosing the
approach side; not done here.

Dependencies:
    pip install reachy2-sdk scipy numpy
    reachy2_symbolic_ik is NOT on PyPI -- install it separately:
        pip install git+https://github.com/pollen-robotics/reachy2_symbolic_ik.git
"""

import numpy as np
from reachy2_sdk import ReachySDK
from scipy.spatial.transform import Rotation as R
from reachy2_symbolic_ik.utils import make_homogenous_matrix_from_rotation_matrix


# --- Hardware constants -----------------------------------------------------
GRIPPER_MAX_OPENING_M = 0.100          # gripper max opening = 100 mm

# Reference orientation: a good horizontal front grasp for an object standing
# straight in front of the robot (+X). Calibrate this ONCE for your setup by
# reading forward_kinematics() in a known pose that grasps an object well.
R0 = R.from_euler("xyz", [0.0, -np.pi / 2, 0.0]).as_matrix()


# --- Dedicated exceptions ---------------------------------------------------
class ObjectTooWideError(Exception):
    """The estimated object is wider than the gripper's max opening."""


class GraspNotReachableError(Exception):
    """No IK solution for the grasp or pre-grasp pose."""


# --- Grasp geometry ---------------------------------------------------------
def _approach(p_center_xy):
    """Horizontal robot->object unit vector and its yaw (rotation about Z)."""
    yaw = float(np.arctan2(p_center_xy[1], p_center_xy[0]))
    a = np.array([np.cos(yaw), np.sin(yaw), 0.0])
    return a, yaw


def _grasp_width(width_m, shape):
    """Width the gripper must span when closing, per shape.
    - cylinder/sphere: the diameter
    - cube/box: the observed side (approach perpendicular to the visible face)
    In all cases this is the observed horizontal width from the bbox.
    """
    return float(width_m)


def _front_to_center(p_front, width_m, shape, a, depth_cap=0.04):
    """Move the point from the visible FRONT face (what the depth sees) to the
    object CENTER, pushing along the viewing direction `a`.
    - sphere/cylinder: back by one radius (depth == width, geometrically observed)
    - cube/box: the depth dimension along the view is NOT observed from a single
      view. Stay conservative: cap the offset at `depth_cap` so we never drive the
      tip into the object. Undershooting only grabs the front part, which is fine;
      overshooting would push the tip into (or past) the object.
    """
    if shape in ("cylinder", "sphere"):
        offset = width_m / 2.0
    elif shape in ("cube", "box"):
        offset = min(width_m / 2.0, depth_cap)
    else:
        raise ValueError(f"Unsupported shape: {shape!r}")
    return np.asarray(p_front, dtype=float) + offset * a


def compute_grasp_poses(p_front, width_m, height_m, shape,
                        grasp_offset=0.02, pre_dist=0.12, depth_cap=0.04):
    """Build (pose_pre, pose_grasp) as 4x4 EE poses.
    Note: this does NOT run the reachability check yet (see plan_grasp)."""
    p_front = np.asarray(p_front, dtype=float)

    a, yaw = _approach(p_front[:2])
    p_center = _front_to_center(p_front, width_m, shape, a, depth_cap)

    # Grasp height: mid-body. If p_front is the mask centroid, its z is already
    # ~mid-height, so I leave it unchanged here.
    # (if you prefer, compute z = z_bottom + height_m/2 from the top/bottom pixels)

    # Orientation: rotate R0 about Z so the gripper "faces" the object while
    # staying horizontal. A sphere has no preferred axis: this still works.
    R_grasp = R.from_euler("z", yaw).as_matrix() @ R0

    # Tip placed just BEFORE the center (object ends up between the fingers,
    # not against the palm).
    p_grasp = p_center - grasp_offset * a
    # Pre-grasp backed off along the same approach line.
    p_pre = p_grasp - pre_dist * a

    pose_grasp = make_homogenous_matrix_from_rotation_matrix(p_grasp, R_grasp)
    pose_pre = make_homogenous_matrix_from_rotation_matrix(p_pre, R_grasp)
    return pose_pre, pose_grasp


def plan_grasp(arm, p_front, width_m, height_m, shape,
               grasp_offset=0.02, pre_dist=0.12, margin=0.010, depth_cap=0.04):
    """Plan and VALIDATE the grasp. Returns a dict with the poses.
    Raises:
      - ObjectTooWideError    if width_m exceeds the gripper opening
      - GraspNotReachableError if the IK finds no solution for pre/grasp
    `arm` = reachy.r_arm or reachy.l_arm.
    """
    # 1) Size check: does the object fit in the gripper?
    grasp_w = _grasp_width(width_m, shape)
    if grasp_w > GRIPPER_MAX_OPENING_M:
        raise ObjectTooWideError(
            f"Object is ~{grasp_w * 1000:.0f} mm wide, beyond the gripper's max "
            f"opening ({GRIPPER_MAX_OPENING_M * 1000:.0f} mm): cannot be grasped."
        )

    # 2) Compute poses
    pose_pre, pose_grasp = compute_grasp_poses(
        p_front, width_m, height_m, shape, grasp_offset, pre_dist, depth_cap
    )

    # 3) Reachability check via IK (raises ValueError if unreachable)
    try:
        arm.inverse_kinematics(pose_pre)
        arm.inverse_kinematics(pose_grasp)
    except ValueError as e:
        raise GraspNotReachableError(
            "Grasp pose not reachable by this arm: "
            "move the mobile base closer or switch arms."
        ) from e

    # Suggested gripper opening before approaching (with a margin).
    open_target_m = min(GRIPPER_MAX_OPENING_M, grasp_w + margin)
    return {
        "pose_pre": pose_pre,
        "pose_grasp": pose_grasp,
        "grasp_width_m": grasp_w,
        "open_target_m": open_target_m,
    }


def execute_grasp(arm, plan, lift_height=0.10):
    """Run the sequence: open -> pre-grasp -> straight-line descent -> close -> lift.
    I pass the 4x4 POSES to goto (it runs the IK internally); the endpoints are
    already validated."""
    arm.gripper.open()

    arm.goto(plan["pose_pre"], duration=4.0,
             interpolation_mode="minimum_jerk", wait=True)

    # Straight cartesian approach/descent toward the grasp.
    arm.goto(plan["pose_grasp"], duration=2.0,
             interpolation_mode="linear", wait=True)

    # close() uses the force sensor to detect when it has grabbed something.
    arm.gripper.close()

    # Lift the object (avoid holding the grip too long: overheat risk).
    lifted = plan["pose_pre"].copy()
    lifted[2, 3] += lift_height
    arm.goto(lifted, duration=2.0, interpolation_mode="minimum_jerk", wait=True)


def pick_arm(reachy, p_front):
    """Pick the arm based on the side (Y > 0 = left in Reachy's frame)."""
    return reachy.l_arm if p_front[1] > 0 else reachy.r_arm


# --- Bridge: raw point cloud -> (p_front, width_m, height_m, shape) --------
#
# A single depth view only ever sees the object's front shell, so shape
# can't be inferred from the cloud itself: it's a prior from the YOLO class.
# width/height instead come straight from the cloud's bounding extent.
# Flat/rectangular items map to "box"; blocky/roughly-cubic ones to "cube"
# (geometrically identical here, kept distinct only for readability).
SHAPE_BY_CLASS = {
    "bottle": "cylinder", "cup": "cylinder", "wine glass": "cylinder", "vase": "cylinder",
    "carrot": "cylinder", "hair drier": "cylinder", "toothbrush": "cylinder",
    "spoon": "cylinder", "fork": "cylinder", "knife": "cylinder", "banana": "cylinder",
    "apple": "sphere", "orange": "sphere", "donut": "sphere",
    "book": "box", "cell phone": "box", "remote": "box", "laptop": "box",
    "keyboard": "box", "tv": "box", "sandwich": "box",
    "mouse": "cube", "bowl": "cube", "cake": "cube", "teddy bear": "cube",
    "broccoli": "cube", "scissors": "cube",
}
DEFAULT_SHAPE = "cylinder"


def shape_from_class(class_name):
    """Rough shape prior from the YOLO class name; defaults to cylinder for
    anything not in the table."""
    return SHAPE_BY_CLASS.get(class_name, DEFAULT_SHAPE)


def point_cloud_to_grasp_input(point_cloud, class_name):
    """Reduce a raw point cloud (Nx3, robot frame, meters) to the
    (p_front, width_m, height_m, shape) plan_grasp needs. p_front is the
    cloud's centroid; width/height are its Y/Z bounding extent."""
    if point_cloud.shape[0] == 0:
        raise ValueError("Empty point cloud: nothing to grasp")

    p_front = point_cloud.mean(axis=0)
    mins = point_cloud.min(axis=0)
    maxs = point_cloud.max(axis=0)
    width_m = float(maxs[1] - mins[1])
    height_m = float(maxs[2] - mins[2])
    return p_front, width_m, height_m, shape_from_class(class_name)


# --- Example of hooking into your pipeline ---------------------------------
if __name__ == "__main__":
    # These values come FROM YOUR already-implemented part
    # (selected bbox + depth + intrinsics K + extrinsics -> robot frame):
    #   p_front  : np.array([x, y, z]) in the robot frame
    #   width_m  : estimated width (diameter or side)
    #   height_m : estimated height
    #   shape    : "cylinder" | "sphere" | "cube" | "box"

    reachy = ReachySDK(host="10.0.0.201")
    reachy.turn_on()
    reachy.goto_posture(wait=True)

    p_front = np.array([0.45, -0.10, -0.05])
    width_m, height_m, shape = 0.07, 0.15, "cylinder"

    arm = pick_arm(reachy, p_front)
    try:
        plan = plan_grasp(arm, p_front, width_m, height_m, shape)
    except ObjectTooWideError as e:
        print(f"[User] {e}")             # -> show this message to the user
    except GraspNotReachableError as e:
        print(f"[Error] {e}")
        raise                            # as requested: propagate the error
    else:
        execute_grasp(arm, plan)