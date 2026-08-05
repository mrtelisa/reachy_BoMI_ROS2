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

import time
from typing import List, NamedTuple, Optional

import matplotlib.pyplot as plt
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

# How far *before* the object's near surface, back along the approach
# direction, the commanded (gripper-center) EE position sits. Reachy2's
# end-effector frame origin is at the center of the gripper, not at the
# fingertips -- the fingers extend further along the approach direction
# from there. So the commanded position has to stay outside the object,
# not at its center: aiming the frame origin at the center would already
# put it (and push the fingers even further) inside the object's own
# volume. Combined with the object's own radius (width_m/2) in plan_grasp,
# this keeps the EE origin just outside the near surface regardless of how
# big around the object is.
GRASP_APPROACH_MARGIN_M = 0

# Pause after gripper.open()/close() -- those calls return immediately
# without waiting for the physical motion, so back-to-back with no pause
# the close would fire before the open is even visible.
GRIPPER_ACTUATION_PAUSE_S = 2.0

# Fallback "up" direction (Reachy world frame) when the table plane fit
# failed (see reachy_detection._remove_table_plane) and no measured normal
# is available.
DEFAULT_TABLE_NORMAL: npt.NDArray[np.float64] = np.array([0.0, 0.0, 1.0])

# How many horizontal approach directions _approach_candidates spreads
# across the full [0, 360 deg) circle around the object when searching for
# one inverse kinematics accepts.
APPROACH_CANDIDATE_COUNT = 16


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


def _side_grasp_closing_axis(
    approach: npt.NDArray[np.float64], up: npt.NDArray[np.float64],
) -> npt.NDArray[np.float64]:
    """The one closing axis, for a horizontal `approach`, that keeps the
    gripper "parallel to the table" -- its local X axis pointing straight
    up, aligned with the object's own (vertical) axis, rather than rolled
    to some arbitrary angle.

    Derived directly, not searched: for an orthonormal right-handed frame,
    local X = local_Y x local_Z (see _orientation_from_approach), which
    rearranges to local_Y = local_Z x local_X. With local_Z = -approach
    and local_X = up, that's just cross(-approach, up) -- automatically
    perpendicular to approach already (a cross product is perpendicular to
    both its inputs), and always well-defined since approach is always
    horizontal (by construction) and so never parallel to the vertical
    `up`.
    """
    closing_axis = np.cross(-approach, up)
    return closing_axis / np.linalg.norm(closing_axis)


def _approach_candidates(
    mid_position: npt.NDArray[np.float64], table_normal: npt.NDArray[np.float64], count: int = APPROACH_CANDIDATE_COUNT,
) -> List[npt.NDArray[np.float64]]:
    """`count` horizontal approach directions -- the "fascio di rette"
    (pencil of lines through the object's centroid, perpendicular to its
    vertical axis) -- evenly spread across the full [0, 360 deg) circle
    around `mid_position`, starting from the direction pointing from
    Reachy's origin straight towards the object (the most direct
    approach). Unlike a roll (which repeats every 180 deg for a parallel
    gripper), approaching from opposite sides of the object means reaching
    through completely different parts of the arm's workspace, so the
    full circle matters here, not just half of it.

    A cylinder/sphere's circular cross-section looks the same from every
    horizontal direction -- gripping from any of them is an equally valid
    grasp -- but the arm's reach can still favor one direction over
    another even though the grasp itself doesn't care which "line" is
    used. plan_grasp searches this list (arm outermost, then approach)
    instead of committing to the single most direct line.
    """
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
        # table_normal itself isn't close to vertical (a bad table-plane
        # fit) -- fall back to an arbitrary horizontal-ish axis rather than
        # divide by ~0.
        perp = np.cross(up, np.array([1.0, 0.0, 0.0]))
        if np.linalg.norm(perp) < 1e-6:
            perp = np.cross(up, np.array([0.0, 1.0, 0.0]))
    perp /= np.linalg.norm(perp)

    return [default * np.cos(theta) + perp * np.sin(theta) for theta in np.linspace(0, 2 * np.pi, count, endpoint=False)]


def _grasp_height_position(geometry: ObjectGeometry) -> npt.NDArray[np.float64]:
    """geometry.centroid, recentered along the object's long axis to the
    middle of its observed height range -- not left at the raw
    density-weighted mean.

    A cylinder viewed from one side, with the camera tilted down, shows
    more of the body than the neck/cap -- and the cap's shiny material
    makes its already-thin sliver of points sparser still. That skew
    doesn't bias height_m (an extent, not sensitive to density), but it
    does bias the plain mean towards the wider/denser base. Left
    uncorrected, the grasp point ends up too low even when the measured
    height spans the full object.

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
    mid_offset = (float(offsets.max()) + float(offsets.min())) / 2.0
    return geometry.centroid + mid_offset * long_axis


def plan_grasp(reachy: ReachySDK, geometry: ObjectGeometry) -> Optional[GraspPlan]:
    """Pre-grasp + grasp + lift end-effector poses for `geometry`, or None
    if the object's pose couldn't be estimated or it's too wide for the
    gripper (see GRIPPER_MAX_OPENING_M).

    Strategy: approach horizontally, closing around the object at its
    mid-height -- not a straight-down approach onto its top. Both
    "cylinder" and "sphere" (the only shapes reachy_detection.SHAPE_BY_CLASS
    produces) are radially symmetric around a vertical axis, so gripping
    at any height, from any horizontal direction, is an equally valid
    grasp -- there's no single "correct" line to approach along. The
    gripper stays "parallel to the table" throughout -- its local X axis
    pointing straight up, aligned with the object's own axis -- rather
    than at some arbitrary roll (see _side_grasp_closing_axis); that's a
    fixed requirement, not searched.

    Position is fixed by the object's geometry, but which of the
    equally-valid horizontal lines to use isn't: for each arm (preferring
    the position's near-side one by y sign, falling back to the other),
    this searches _approach_candidates (the full circle of horizontal
    directions through the object, the "fascio di rette" perpendicular to
    its axis) for one inverse kinematics actually accepts for both
    pregrasp and grasp -- instead of committing to the single most direct
    line and giving up if IK happens to reject it. Falls back to the
    near-side arm at the default direction if nothing is found, so a plan
    is always still returned to inspect (e.g. via show_grasp_plan);
    execute_grasp's own reachability check is what ultimately gates
    actually moving.
    """
    if geometry.centroid is None or geometry.axes is None:
        return None
    if not (0 < geometry.width_m <= GRIPPER_MAX_OPENING_M):
        return None

    table_normal = geometry.table_normal if geometry.table_normal is not None else DEFAULT_TABLE_NORMAL
    table_normal = table_normal / np.linalg.norm(table_normal)

    # Mid-height point: "sphere" is already the true fitted 3D center (see
    # reachy_detection._object_dimensions), "cylinder" still needs
    # _grasp_height_position's density-bias correction.
    mid_position = geometry.centroid if geometry.shape == "sphere" else _grasp_height_position(geometry)
    radius = geometry.width_m / 2.0

    def _pregrasp_grasp_rotation_for(approach: npt.NDArray[np.float64]):
        # Pull the commanded EE position back from the object's center to
        # just outside its near surface along this approach direction --
        # see GRASP_APPROACH_MARGIN_M for why.
        grasp_position = mid_position - (radius + GRASP_APPROACH_MARGIN_M) * approach
        pregrasp_position = grasp_position - approach * PREGRASP_STANDOFF_M
        rotation = _orientation_from_approach(approach, _side_grasp_closing_axis(approach, table_normal))
        return pregrasp_position, grasp_position, rotation

    near_side = "r_arm" if mid_position[1] < 0 else "l_arm"
    far_side = "l_arm" if near_side == "r_arm" else "r_arm"

    # Fallback if nothing below is confirmed reachable: the single most
    # direct line (straight from Reachy's origin to the object),
    # near-side arm -- unvalidated, but still a plan to inspect.
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
        print(f"[plan_grasp] WARNING: no approach line reachable on either arm "
              f"({APPROACH_CANDIDATE_COUNT} directions each) -- falling back to {arm_name} "
              "at the default line, UNVALIDATED. This plan's poses are not confirmed reachable.")

    return GraspPlan(
        arm_name=arm_name,
        pregrasp_matrix=_pose_matrix(rotation, pregrasp_position),
        grasp_matrix=_pose_matrix(rotation, grasp_position),
        lift_matrix=_pose_matrix(rotation, lift_position),
    )


def execute_grasp(reachy: ReachySDK, plan: GraspPlan, duration: float = ARM_GOTO_DURATION_S) -> bool:
    """Drives plan.arm_name through pregrasp -> open -> close -> lift.

    plan.grasp_matrix -- the actual approach onto the object -- is
    intentionally skipped for now: goes straight from pre-grasp to lift
    after opening/closing the gripper at pre-grasp, so the arm's reach and
    the gripper's open/close can be checked without yet trusting the final
    approach.

    Returns False without moving the arm if it (or its gripper) isn't
    available, or if pregrasp/lift (the poses actually used here) are
    outside its reachable workspace.

    Reachability (via arm.inverse_kinematics()) is checked against the
    arm's actual current joints -- the arm is assumed to already be
    wherever it'll actually move from (e.g. positioned by
    tests/elbow_135.py beforehand), not force-moved to some pose of this
    function's own choosing first.
    """
    arm: Optional[Arm] = getattr(reachy, plan.arm_name)
    if arm is None or arm.gripper is None:
        print(f"[ERROR] {plan.arm_name} or its gripper is not available -- grasp not executed")
        return False

    for name, matrix in (("pregrasp", plan.pregrasp_matrix), ("lift", plan.lift_matrix)):
        try:
            arm.inverse_kinematics(matrix)
        except ValueError:
            print(f"[ERROR] {name} pose unreachable for {plan.arm_name} -- grasp not executed")
            return False

    try:
        print(f"[{plan.arm_name}] moving to pregrasp...")
        arm.goto(plan.pregrasp_matrix, duration=duration, wait=True)
        print(f"[{plan.arm_name}] opening gripper...")
        arm.gripper.open()
        time.sleep(GRIPPER_ACTUATION_PAUSE_S)
        print(f"[{plan.arm_name}] closing gripper...")
        arm.gripper.close()
        time.sleep(GRIPPER_ACTUATION_PAUSE_S)
        # plan.grasp_matrix skipped for now -- straight from pregrasp to lift.
        # To re-enable, goto() defaults to interpolation_space="joint_space",
        # which interpolates joint angles, not the Cartesian path -- even
        # between two poses at the same height, the end effector isn't
        # guaranteed to travel in a straight line. For a genuinely straight
        # pregrasp -> grasp approach (pregrasp_matrix and grasp_matrix are
        # both on the same horizontal line, see plan_grasp), pass
        # interpolation_space="cartesian_space":
        #     arm.goto(plan.grasp_matrix, duration=duration,
        #              interpolation_space="cartesian_space", wait=True)
        print(f"[{plan.arm_name}] moving to lift...")
        arm.goto(plan.lift_matrix, duration=duration, wait=True)
    except RuntimeError as exc:
        print(f"[ERROR] {plan.arm_name} grasp aborted: {exc}")
        return False

    print(f"[{plan.arm_name}] pregrasp -> open -> close -> lift done (approach to object still skipped)")
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
    # A single short pause isn't always enough time for the window manager
    # to map and raise a brand-new window before the caller races on into
    # the OpenCV camera loop right after this returns (cv2.imshow +
    # waitKey(1), which keeps its own window on top and starves matplotlib's
    # event loop). Several short pauses -- still non-blocking overall, ~1s
    # total -- give it more chances to actually get drawn and shown.
    # Deliberately plt.pause(), not driving the Tk window object directly:
    # that bypasses matplotlib's own event handling and leaves the window
    # unresponsive afterwards (close button included).
    for _ in range(10):
        plt.pause(0.1)
