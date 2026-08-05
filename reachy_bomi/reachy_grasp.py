#!/usr/bin/env python3
"""
Grasp planning + execution for Reachy2: from reachy_detection.py's point
cloud (position, axes, width/height, table normal) for a confirmed object,
computes pre-grasp/grasp/lift end-effector poses and drives the arm through
them over reachy2_sdk.

Pose matrix convention (world: X forward, Y left, Z up; verified against
https://docs.pollen-robotics.com/developing-with-reachy-2/basics/4-use-arm-kinematics/):
end-effector local -Z = approach direction, local Y = gripper open/close
axis. See _orientation_from_approach.
"""

from typing import List, NamedTuple, Optional

import numpy as np
import numpy.typing as npt
from reachy2_sdk import ReachySDK
from reachy2_sdk.parts.arm import Arm

# Reachy2 parallel gripper's max fingertip opening, in meters.
GRIPPER_MAX_OPENING_M = 0.1

# How far back from the grasp point, opposite the approach direction, the
# pre-grasp waypoint sits.
PREGRASP_STANDOFF_M = 0.10

# How far straight up (along the table normal) the arm lifts once the
# gripper has closed on the object.
GRASP_LIFT_M = 0.15

ARM_GOTO_DURATION_S = 6.0

# How far outside the object's near surface (beyond its radius) the
# commanded EE position sits. Reachy2's EE frame is at the gripper's
# center, not the fingertips, so aiming at the object's center would put
# the frame origin -- and the fingers, further along approach still --
# inside the object's own volume.
GRASP_APPROACH_MARGIN_M = 0.0

# Fallback "up" direction (Reachy world frame) when the table plane fit
# fails (see reachy_detection._remove_table_plane).
DEFAULT_TABLE_NORMAL: npt.NDArray[np.float64] = np.array([0.0, 0.0, 1.0])

# How many horizontal approach directions _approach_candidates spreads
# across the full circle around the object when searching for one IK accepts.
APPROACH_CANDIDATE_COUNT = 16


class ObjectGeometry(NamedTuple):
    """Everything reachy_detection.py's point cloud pipeline knows about a
    confirmed object, in Reachy's world frame (meters). centroid/axes/
    table_normal are None if too few points survived isolation to fit
    them -- plan_grasp then treats the object as ungraspable."""

    class_name: str
    shape: str
    width_m: float
    height_m: float
    centroid: Optional[npt.NDArray[np.float64]]
    axes: Optional[npt.NDArray[np.float64]]  # 3x3, columns = principal axes, sorted long -> short extent
    table_normal: Optional[npt.NDArray[np.float64]]
    point_cloud: npt.NDArray[np.float64]  # (N, 3), isolated object points, for show_grasp_plan


class GraspPlan(NamedTuple):
    arm_name: str  # "r_arm" or "l_arm"
    pregrasp_matrix: npt.NDArray[np.float64]  # 4x4, Reachy world frame
    grasp_matrix: npt.NDArray[np.float64]  # 4x4, Reachy world frame
    lift_matrix: npt.NDArray[np.float64]  # 4x4, Reachy world frame


def _orientation_from_approach(
    approach: npt.NDArray[np.float64], closing_axis: npt.NDArray[np.float64],
) -> npt.NDArray[np.float64]:
    """3x3 rotation for an end-effector reaching along world-frame `approach`
    with its gripper opening/closing along world-frame `closing_axis`
    (local -Z = approach, local Y = closing_axis, local X completes a
    right-handed frame). closing_axis is re-projected perpendicular to
    approach first, so it only needs to be roughly right."""
    local_z = -approach / np.linalg.norm(approach)

    local_y = closing_axis - local_z * np.dot(closing_axis, local_z)
    local_y_norm = np.linalg.norm(local_y)
    if local_y_norm < 1e-6:
        # closing_axis was parallel to approach -- fall back to whichever
        # world axis is least aligned with it.
        fallback = np.eye(3)[np.argmin(np.abs(approach))]
        local_y = fallback - local_z * np.dot(fallback, local_z)
        local_y_norm = np.linalg.norm(local_y)
    local_y /= local_y_norm

    local_x = np.cross(local_y, local_z)
    return np.column_stack([local_x, local_y, local_z])


def _pose_matrix(rotation: npt.NDArray[np.float64], position: npt.NDArray[np.float64]) -> npt.NDArray[np.float64]:
    matrix = np.eye(4)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = position
    return matrix


def _side_grasp_closing_axis(
    approach: npt.NDArray[np.float64], up: npt.NDArray[np.float64],
) -> npt.NDArray[np.float64]:
    """The one closing axis, for a horizontal `approach`, that keeps the
    gripper's local X axis pointing straight up (parallel to the table,
    aligned with the object's own axis) -- derived directly as
    cross(-approach, up), not searched, since local_X = up and local_Z =
    -approach fully determine local_Y = local_Z x local_X."""
    closing_axis = np.cross(-approach, up)
    return closing_axis / np.linalg.norm(closing_axis)


def _approach_candidates(
    mid_position: npt.NDArray[np.float64], table_normal: npt.NDArray[np.float64], count: int = APPROACH_CANDIDATE_COUNT,
) -> List[npt.NDArray[np.float64]]:
    """`count` horizontal approach directions spanning the full circle
    around the object (not just 180 deg -- opposite sides reach through
    completely different parts of the arm's workspace), starting from the
    direct line from Reachy's origin to the object. A cylinder/sphere looks
    the same from every horizontal direction, so any of these is an equally
    valid grasp; plan_grasp searches them for one IK actually accepts."""
    radial_xy = mid_position[:2]
    radial_norm = np.linalg.norm(radial_xy)
    if radial_norm < 1e-6:
        default = np.array([1.0, 0.0, 0.0])  # object directly below the origin -- arbitrary but valid start
    else:
        default = np.array([radial_xy[0] / radial_norm, radial_xy[1] / radial_norm, 0.0])

    up = table_normal / np.linalg.norm(table_normal)
    perp = np.cross(up, default)
    perp_norm = np.linalg.norm(perp)
    if perp_norm < 1e-6:
        # table_normal isn't close to vertical (a bad table-plane fit) --
        # fall back to an arbitrary horizontal-ish axis.
        perp = np.cross(up, np.array([1.0, 0.0, 0.0]))
        if np.linalg.norm(perp) < 1e-6:
            perp = np.cross(up, np.array([0.0, 1.0, 0.0]))
    perp /= np.linalg.norm(perp)

    return [default * np.cos(theta) + perp * np.sin(theta) for theta in np.linspace(0, 2 * np.pi, count, endpoint=False)]


def _grasp_height_position(geometry: ObjectGeometry) -> npt.NDArray[np.float64]:
    """geometry.centroid recentered along the object's long axis to the
    middle of its observed height range, correcting for the density bias
    of a partial view (a cylinder's cap has fewer points than its body, so
    the raw mean skews low). Skip for "sphere": its centroid is already
    the true fitted 3D center (reachy_detection._object_dimensions)."""
    long_axis = geometry.axes[:, 0]
    offsets = (geometry.point_cloud - geometry.centroid) @ long_axis
    mid_offset = (float(offsets.max()) + float(offsets.min())) / 2.0
    return geometry.centroid + mid_offset * long_axis


def plan_grasp(reachy: ReachySDK, geometry: ObjectGeometry) -> Optional[GraspPlan]:
    """Pre-grasp + grasp + lift end-effector poses for `geometry`, or None
    if its pose couldn't be estimated or it's too wide for the gripper.

    Approaches horizontally at the object's mid-height, gripper kept
    "parallel to the table" (see _side_grasp_closing_axis) -- both
    "cylinder" and "sphere" are radially symmetric around a vertical axis,
    so any height/horizontal direction is an equally valid grasp. For each
    arm (near-side by y sign first, then the other), searches
    _approach_candidates for a direction IK accepts for both pregrasp and
    grasp, instead of committing to the single most direct line. Falls
    back to the near-side arm at the default direction if nothing is
    found, so a plan is always returned to inspect (e.g. via
    show_grasp_plan) even if execute_grasp will then refuse to move.
    """
    if geometry.centroid is None or geometry.axes is None:
        return None
    if not (0 < geometry.width_m <= GRIPPER_MAX_OPENING_M):
        return None

    table_normal = geometry.table_normal if geometry.table_normal is not None else DEFAULT_TABLE_NORMAL
    table_normal = table_normal / np.linalg.norm(table_normal)

    mid_position = geometry.centroid if geometry.shape == "sphere" else _grasp_height_position(geometry)
    radius = geometry.width_m / 2.0

    def _pregrasp_grasp_rotation_for(approach: npt.NDArray[np.float64]):
        grasp_position = mid_position - (radius + GRASP_APPROACH_MARGIN_M) * approach
        pregrasp_position = grasp_position - approach * PREGRASP_STANDOFF_M
        rotation = _orientation_from_approach(approach, _side_grasp_closing_axis(approach, table_normal))
        return pregrasp_position, grasp_position, rotation

    near_side = "r_arm" if mid_position[1] < 0 else "l_arm"
    far_side = "l_arm" if near_side == "r_arm" else "r_arm"

    # Fallback if nothing below is confirmed reachable: the single most
    # direct line, near-side arm -- unvalidated, but still a plan to inspect.
    arm_name = near_side
    approach = _approach_candidates(mid_position, table_normal)[0]
    pregrasp_position, grasp_position, rotation = _pregrasp_grasp_rotation_for(approach)

    found = False
    for candidate_arm_name in (near_side, far_side):
        arm = getattr(reachy, candidate_arm_name, None)
        if arm is None:
            continue
        for candidate_approach in _approach_candidates(mid_position, table_normal):
            candidate_pregrasp_position, candidate_grasp_position, candidate_rotation = \
                _pregrasp_grasp_rotation_for(candidate_approach)
            try:
                arm.inverse_kinematics(_pose_matrix(candidate_rotation, candidate_pregrasp_position))
                arm.inverse_kinematics(_pose_matrix(candidate_rotation, candidate_grasp_position))
            except ValueError:
                continue
            arm_name = candidate_arm_name
            approach = candidate_approach
            rotation = candidate_rotation
            pregrasp_position, grasp_position = candidate_pregrasp_position, candidate_grasp_position
            found = True
            break
        else:
            continue
        break

    lift_position = grasp_position + table_normal * GRASP_LIFT_M

    if found:
        print(f"[plan_grasp] found a reachable approach line for {arm_name} "
              f"({APPROACH_CANDIDATE_COUNT} directions x 2 arms tried)")
    else:
        print(f"[plan_grasp] WARNING: no approach line reachable on either arm -- "
              f"falling back to {arm_name} at the default line, UNVALIDATED.")

    return GraspPlan(
        arm_name=arm_name,
        pregrasp_matrix=_pose_matrix(rotation, pregrasp_position),
        grasp_matrix=_pose_matrix(rotation, grasp_position),
        lift_matrix=_pose_matrix(rotation, lift_position),
    )


def execute_grasp(reachy: ReachySDK, plan: GraspPlan, duration: float = ARM_GOTO_DURATION_S) -> bool:
    """Drives plan.arm_name through open -> pregrasp -> grasp -> close ->
    lift. The pregrasp -> grasp leg uses interpolation_space="cartesian_space"
    for a straight path, since the two sit on the same horizontal line
    (see plan_grasp). Returns False without moving if the arm/gripper isn't
    available or any pose is unreachable from the arm's current joints."""
    arm: Optional[Arm] = getattr(reachy, plan.arm_name)
    if arm is None or arm.gripper is None:
        print(f"[ERROR] {plan.arm_name} or its gripper is not available -- grasp not executed")
        return False

    for name, matrix in (("pregrasp", plan.pregrasp_matrix), ("grasp", plan.grasp_matrix), ("lift", plan.lift_matrix)):
        try:
            arm.inverse_kinematics(matrix)
        except ValueError:
            print(f"[ERROR] {name} pose unreachable for {plan.arm_name} -- grasp not executed")
            return False

    try:
        print(f"[{plan.arm_name}] opening gripper...")
        arm.gripper.open()
        print(f"[{plan.arm_name}] moving to pregrasp...")
        arm.goto(plan.pregrasp_matrix, duration=duration, wait=True)
        print(f"[{plan.arm_name}] moving to grasp...")
        arm.goto(plan.grasp_matrix, duration=duration, interpolation_space="cartesian_space", wait=True)
        print(f"[{plan.arm_name}] closing gripper...")
        arm.gripper.close()
        print(f"[{plan.arm_name}] moving to lift...")
        arm.goto(plan.lift_matrix, duration=duration, wait=True)
    except RuntimeError as exc:
        print(f"[ERROR] {plan.arm_name} grasp aborted: {exc}")
        return False

    print(f"[{plan.arm_name}] grasp sequence done")
    return True
