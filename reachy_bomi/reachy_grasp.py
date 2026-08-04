#!/usr/bin/env python3
"""
Grasp planning + execution for Reachy2.

Takes the object geometry produced by reachy_detection.py's point cloud
pipeline (position, principal axes, width, height, table normal) for a
confirmed object, computes a pre-grasp and grasp end-effector pose, and
drives the arm through them over reachy2_sdk (gRPC/IP).

Reachy2 end-effector pose matrix convention (see
https://docs.pollen-robotics.com/developing-with-reachy-2/basics/4-use-arm-kinematics/
and reachy2-sdk's src/examples/draw_square.py): the world frame is X
forward, Y left (-Y is right), Z up. In the end-effector's own local frame,
the identity rotation has the hand facing straight down, and rotating -90
about world Y turns it to face forward -- from those two examples, the
end-effector's local -Z axis is its approach/pointing direction, and its
local Y axis is the gripper's opening/closing direction. _orientation_from_approach
reproduces both example matrices exactly from that rule.
"""

from typing import NamedTuple, Optional

import matplotlib.pyplot as plt
import numpy as np
import numpy.typing as npt
from reachy2_sdk import ReachySDK
from reachy2_sdk.parts.arm import Arm

# Reachy2 parallel gripper's max fingertip opening, in meters. 
GRIPPER_MAX_OPENING_M = 0.1

# How far back from the grasp point, opposite the approach direction, the
# pre-grasp waypoint sits (i.e. straight above the object, for the top-down
# approach plan_grasp always uses).
PREGRASP_STANDOFF_M = 0.10

# How far straight up (along the table normal) the arm lifts once the
# gripper has closed on the object.
GRASP_LIFT_M = 0.15

# How far *above* the object's true top (see _top_down_grasp_position) the
# commanded grasp pose sits. Reachy2's end-effector frame origin is at the
# center of the gripper, not at the fingertips -- the fingers extend
# further along the approach direction from there. So a straight-down
# approach must keep the commanded EE position at or above the object's
# top: pushing it down to (or below) the top, as if the frame origin were
# the fingertips, drags the actual fingers into the object regardless.
GRASP_ABOVE_TOP_M = 0.05

ARM_GOTO_DURATION_S = 6.0

# Fallback "up" direction (Reachy world frame) when the table plane fit
# failed (see reachy_detection._remove_table_plane) and no measured normal
# is available.
DEFAULT_TABLE_NORMAL: npt.NDArray[np.float64] = np.array([0.0, 0.0, 1.0])


class ObjectGeometry(NamedTuple):
    """Everything reachy_detection.py's point cloud pipeline knows about a
    confirmed object, in Reachy's world coordinate frame (meters).

    centroid/axes/table_normal are None when too few points survived
    isolation to fit them (see reachy_detection._object_dimensions /
    _remove_table_plane) -- plan_grasp treats that object as ungraspable.
    """

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
    with its gripper opening/closing along world-frame `closing_axis`.

    Builds the frame from the module docstring's rule (approach on local
    -Z, opening/closing on local Y, local X completing a right-handed
    frame). closing_axis only needs to be roughly right -- it's re-projected
    to be exactly perpendicular to `approach` before use.
    """
    local_z = -approach / np.linalg.norm(approach)

    local_y = closing_axis - local_z * np.dot(closing_axis, local_z)
    local_y_norm = np.linalg.norm(local_y)
    if local_y_norm < 1e-6:
        # closing_axis was (numerically) parallel to the approach axis --
        # fall back to whichever world axis is least aligned with it.
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


def _closing_axis_for(geometry: ObjectGeometry) -> npt.NDArray[np.float64]:
    """World-frame direction the gripper fingers should open/close along.

    Reuses the point cloud's 2nd-largest principal axis (axes[:, 1] -- the
    same axis reachy_detection._object_dimensions measures as
    width_m/diameter_m), which generalizes across shapes without a
    per-class special case: for an upright cylinder it's the horizontal
    diameter direction; for a box lying flat it's the shorter in-plane
    side; for an elongated object lying on its side (e.g. a banana) it's
    the across-the-body direction -- always the narrower, more reliably
    graspable cross-section. Doesn't need to be exact -- passed through
    _orientation_from_approach, which projects it perpendicular to the
    approach axis before use.
    """
    assert geometry.axes is not None
    return geometry.axes[:, 1]


def is_position_reachable(reachy: ReachySDK, position: npt.NDArray[np.float64]) -> bool:
    """Coarse a-priori reachability check for a 3D position (Reachy world
    frame) -- meant to run on every YOLO detection at capture time, before
    the expensive per-object point cloud / pose pipeline, so a box is only
    ever drawn for something at least one arm can plausibly reach.

    Tries a generic top-down grasp orientation at `position` (straight
    down, arbitrary closing axis -- IK reachability barely depends on
    gripper roll) against whichever arms the robot reports, via the same
    arm.inverse_kinematics() used by execute_grasp. This only needs a
    single depth sample (reachy_detection._estimate_object_position), not a
    fused point cloud, so it's cheap enough to run on every detection.

    Seeds the IK solve with the arm's *actual current* joints (leaving q0
    unset -- that's inverse_kinematics()'s own default): the arm is assumed
    to already be wherever it'll actually move from, so reachability should
    reflect that real starting position, not a hypothetical one. If the arm
    isn't in a good starting posture yet, position it first (see
    tests/elbow_135.py) -- this deliberately doesn't second-guess that by
    substituting a fixed pose of its own.
    """
    approach = -DEFAULT_TABLE_NORMAL
    rotation = _orientation_from_approach(approach, np.array([1.0, 0.0, 0.0]))
    matrix = _pose_matrix(rotation, position)

    for arm in (reachy.r_arm, reachy.l_arm):
        if arm is None:
            continue
        try:
            arm.inverse_kinematics(matrix)
            return True
        except ValueError:
            continue
    return False


def _top_down_grasp_position(geometry: ObjectGeometry) -> npt.NDArray[np.float64]:
    """geometry.centroid, moved along the object's long axis to just above
    its observed top (GRASP_ABOVE_TOP_M above the highest point along that
    axis) -- not to its mid-height, and not left at the raw density-weighted
    mean either.

    plan_grasp always approaches straight down, and Reachy2's end-effector
    frame origin sits at the center of the gripper, not at the fingertips
    -- the fingers extend further along the approach direction from there.
    So the commanded position has to stay at or above the object's top: if
    it were pushed down to (or below) the top instead, the fingers --
    further along approach, i.e. further down still -- would end up
    *inside* the object regardless of how precisely the top itself is
    measured.

    Deliberately uses the raw min/max here, not the 1st-99th percentile
    trim height_m/_object_dimensions use for extent robustness: that trim
    protects against a single stray point (a table sliver, a flying-pixel
    bridge) inflating a *measurement*, but reachy_detection._remove_flying_pixels
    already removed that kind of noise upstream, before this function ever
    sees the point cloud. Trimming again here would only push the "top"
    reference below what was actually observed, which matters more for a
    cylinder's already-sparse cap (a cylinder viewed from one side, with
    the camera tilted down, shows more of the body than the neck/cap, and
    the cap's shiny material makes its already-thin sliver of points
    sparser still).

    Only call this for shapes not already corrected in full 3D by
    reachy_detection._object_dimensions (i.e. anything but "sphere"): a
    sphere's centroid there is already the true fitted center, not a
    density-weighted mean, and this function's own assumption -- that
    point_cloud sits mostly to one side of centroid, the way a partial
    view's raw mean does -- no longer holds once centroid is already
    correct.
    """
    long_axis = geometry.axes[:, 0]
    offsets = (geometry.point_cloud - geometry.centroid) @ long_axis
    hi = float(offsets.max())
    lo = float(offsets.min())

    # long_axis's sign from PCA is arbitrary, so table_normal (falling back
    # to DEFAULT_TABLE_NORMAL) is what actually says which end is "up" --
    # i.e. which bound (hi or lo) is the object's physical top, and which
    # direction is "further above" it.
    table_normal = geometry.table_normal if geometry.table_normal is not None else DEFAULT_TABLE_NORMAL
    up_sign = 1.0 if np.dot(long_axis, table_normal) >= 0 else -1.0
    above_top_offset = (hi + GRASP_ABOVE_TOP_M) if up_sign > 0 else (lo - GRASP_ABOVE_TOP_M)

    return geometry.centroid + above_top_offset * long_axis


def plan_grasp(reachy: ReachySDK, geometry: ObjectGeometry) -> Optional[GraspPlan]:
    """Pre-grasp + grasp + lift end-effector poses for `geometry`, or None
    if the object's pose couldn't be estimated or it's too wide for the
    gripper (see GRIPPER_MAX_OPENING_M).

    Strategy: always approach straight down from above the table. This is
    robust across every shape in reachy_detection.SHAPE_BY_CLASS -- including
    objects lying on their side -- because the gripper always closes across
    whatever the narrowest horizontal cross-section is (see
    _closing_axis_for), rather than needing a per-shape side/top decision.

    Arm choice tries both r_arm/l_arm (IK against each's actual current
    joints, same as is_position_reachable) and picks whichever can actually
    reach both pregrasp and grasp, preferring the position's near-side arm
    (by y sign) when both can. Falls back to that near-side arm if neither
    can -- a plan is always still returned so it can be inspected (e.g. via
    show_grasp_plan); execute_grasp's own reachability check is what
    ultimately gates actually moving. Without this, a naive sign-only
    choice can pick an arm that is_position_reachable's own (both-arms)
    check never actually confirmed for this specific top-down orientation
    -- reachable in principle, unreachable for the arm this function
    happened to lock into.
    """
    if geometry.centroid is None or geometry.axes is None:
        return None
    if not (0 < geometry.width_m <= GRIPPER_MAX_OPENING_M):
        return None

    table_normal = geometry.table_normal if geometry.table_normal is not None else DEFAULT_TABLE_NORMAL
    table_normal = table_normal / np.linalg.norm(table_normal)
    approach = -table_normal  # straight down, into the table

    closing_axis = _closing_axis_for(geometry)
    rotation = _orientation_from_approach(approach, closing_axis)

    # A straight-down approach must keep the commanded (gripper-center) EE
    # position at or above an object's top -- see _top_down_grasp_position
    # for why. "sphere" has no long_axis worth using for that (its PCA axes
    # are arbitrary noise, a sphere has no true one), but its center is
    # already exact (see reachy_detection._object_dimensions) and its
    # radius is width_m/2, so the above-top point is just centroid raised
    # by (radius + margin) along table_normal, the one direction that's
    # actually meaningful here.
    if geometry.shape == "sphere":
        radius = geometry.width_m / 2.0
        grasp_position = geometry.centroid + table_normal * (radius + GRASP_ABOVE_TOP_M)
    else:
        grasp_position = _top_down_grasp_position(geometry)
    pregrasp_position = grasp_position - approach * PREGRASP_STANDOFF_M
    lift_position = grasp_position + table_normal * GRASP_LIFT_M

    pregrasp_matrix = _pose_matrix(rotation, pregrasp_position)
    grasp_matrix = _pose_matrix(rotation, grasp_position)
    lift_matrix = _pose_matrix(rotation, lift_position)

    near_side = "r_arm" if grasp_position[1] < 0 else "l_arm"
    far_side = "l_arm" if near_side == "r_arm" else "r_arm"
    arm_name = near_side
    for candidate in (near_side, far_side):
        arm = getattr(reachy, candidate, None)
        if arm is None:
            continue
        try:
            arm.inverse_kinematics(pregrasp_matrix)
            arm.inverse_kinematics(grasp_matrix)
        except ValueError:
            continue
        arm_name = candidate
        break

    return GraspPlan(
        arm_name=arm_name,
        pregrasp_matrix=pregrasp_matrix,
        grasp_matrix=grasp_matrix,
        lift_matrix=lift_matrix,
    )


def execute_grasp(reachy: ReachySDK, plan: GraspPlan, duration: float = ARM_GOTO_DURATION_S) -> bool:
    """Drives plan.arm_name through pregrasp -> grasp -> close -> lift.

    Returns False without moving the arm if it (or its gripper) isn't
    available, or if either pose is outside its reachable workspace.

    Reachability (via arm.inverse_kinematics(), same as
    is_position_reachable) is checked against the arm's actual current
    joints -- the arm is assumed to already be wherever it'll actually move
    from (e.g. positioned by tests/elbow_135.py beforehand), not force-moved
    to some pose of this function's own choosing first.

    MOTION DISABLED: the actual goto/gripper calls are commented out below
    while the planned poses are being checked visually (see show_grasp_plan)
    -- for now this only validates reachability and leaves the robot still.
    Uncomment the block to actually execute the grasp.
    """
    arm: Optional[Arm] = getattr(reachy, plan.arm_name)
    if arm is None or arm.gripper is None:
        print(f"[ERROR] {plan.arm_name} or its gripper is not available -- grasp not executed")
        return False

    for name, matrix in (("pregrasp", plan.pregrasp_matrix), ("grasp", plan.grasp_matrix)):
        try:
            arm.inverse_kinematics(matrix)
        except ValueError:
            print(f"[ERROR] {name} pose unreachable for {plan.arm_name} -- grasp not executed")
            return False

    print(f"[{plan.arm_name}] grasp plan reachable -- motion disabled, robot stays still")
    # try:
    #     arm.gripper.open()
    #     arm.goto(plan.pregrasp_matrix, duration=duration, wait=True)
    #     arm.goto(plan.grasp_matrix, duration=duration, wait=True)
    #     arm.gripper.close()
    #     arm.goto(plan.lift_matrix, duration=duration, wait=True)
    # except RuntimeError as exc:
    #     print(f"[ERROR] {plan.arm_name} grasp aborted: {exc}")
    #     return False
    return True


def show_grasp_plan(geometry: ObjectGeometry, plan: GraspPlan) -> None:
    """Non-blocking 3D scatter of the object's point cloud (same style as
    reachy_detection._show_point_cloud) with the planned pre-grasp/grasp/
    lift end-effector positions marked on top, so the plan can be checked
    visually before execute_grasp is trusted to actually move the arm."""
    point_cloud = geometry.point_cloud
    if point_cloud.shape[0] == 0:
        print("[WARN] Point cloud is empty, nothing to show")
        return

    fig = plt.figure(f"Grasp plan - {geometry.class_name} ({plan.arm_name})")
    ax = fig.add_subplot(projection="3d")

    xs, ys, zs = point_cloud[:, 0], point_cloud[:, 1], point_cloud[:, 2]
    ax.scatter(xs, ys, zs, c=zs, cmap="viridis", s=4, label="object point cloud")

    pregrasp_pos = plan.pregrasp_matrix[:3, 3]
    grasp_pos = plan.grasp_matrix[:3, 3]
    lift_pos = plan.lift_matrix[:3, 3]

    ax.plot(*zip(pregrasp_pos, grasp_pos), c="blue", linestyle="--", linewidth=1.5, label="approach path")
    ax.scatter(*pregrasp_pos, c="orange", marker="^", s=80, label="pre-grasp EE")
    ax.scatter(*grasp_pos, c="red", marker="X", s=100, label="grasp EE")
    ax.scatter(*lift_pos, c="green", marker="^", s=80, label="lift EE")

    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_zlabel("z (m)")
    ax.set_title(f"{geometry.class_name}: planned grasp ({plan.arm_name})")
    ax.legend(loc="upper left", fontsize=8)

    # Equal aspect ratio over both the point cloud and the plan's waypoints,
    # so the pre-grasp/lift points (offset from the object) aren't clipped
    # and the object's proportions aren't visually distorted.
    all_points = np.vstack([point_cloud, [pregrasp_pos, grasp_pos, lift_pos]])
    ranges = all_points.max(axis=0) - all_points.min(axis=0)
    half_range = max(ranges.max() / 2.0, 1e-3)
    mid = all_points.mean(axis=0)
    ax.set_xlim(mid[0] - half_range, mid[0] + half_range)
    ax.set_ylim(mid[1] - half_range, mid[1] + half_range)
    ax.set_zlim(mid[2] - half_range, mid[2] + half_range)

    plt.show(block=False)
    plt.pause(0.001)
