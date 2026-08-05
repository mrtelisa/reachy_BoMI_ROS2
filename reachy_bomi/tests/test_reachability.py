#!/usr/bin/env python3
"""Connect to Reachy2, check whether a given 4x4 end-effector pose matrix is
reachable (inverse kinematics) for one arm, and if so, move there.

Paste a matrix printed by test_grasp.py (e.g. the "pregrasp_matrix =
np.array([[...]])" line, and the arm name printed right above it) into
TARGET_MATRIX / TARGET_ARM below, then run this script. It isolates a single
pose from the whole detection/grasp-planning pipeline, so a rejection can be
pinned on either a real limit of Reachy's kinematics or a bug upstream in
how the pose was computed.

Usage:
    python3 test_reachability.py [robot_ip]
"""

import argparse
import sys

import numpy as np
from reachy2_sdk import ReachySDK

DEFAULT_ROBOT_IP = "192.168.0.121"
GOTO_DURATION_S = 6.0

# Which arm to test -- "r_arm" or "l_arm". Match whatever test_grasp.py
# printed the matrix for; the same matrix can have a different IK result on
# the other arm.
TARGET_ARM = "l_arm"

# Paste a pose matrix here (same structure test_grasp.py prints): a 4x4
# array, rotation in the top-left 3x3 block, position in the last column.
TARGET_MATRIX = np.array([[1, 0, 0, 0.43219],
                          [0, 1, 0, 0.071092],
                          [0, 0, 1, -0.02968], 
                          [0.0, 0.0, 0.0, 1.0]])


def _check_and_goto(reachy: ReachySDK, arm_name: str, matrix: np.ndarray) -> None:
    arm = getattr(reachy, arm_name, None)
    if arm is None:
        print(f"[ERROR] {arm_name} not available on this robot")
        return

    position = matrix[:3, 3]
    print(f"[{arm_name}] target position (Reachy world frame): "
          f"x={position[0]:.3f} y={position[1]:.3f} z={position[2]:.3f}")

    try:
        joints = arm.inverse_kinematics(matrix)
    except ValueError:
        print(f"[{arm_name}] NOT reachable")
        return

    print(f"[{arm_name}] reachable -- IK solution (deg): {[round(j, 1) for j in joints]}")
    print(f"[{arm_name}] moving there now...")
    arm.goto(matrix, duration=GOTO_DURATION_S, wait=True)
    print(f"[{arm_name}] arrived.")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("robot_ip", nargs="?", default=DEFAULT_ROBOT_IP,
                        help=f"IP address of the Reachy robot (default: {DEFAULT_ROBOT_IP})")
    cli_args = parser.parse_args()

    reachy = ReachySDK(host=cli_args.robot_ip)
    if reachy.r_arm is None and reachy.l_arm is None:
        print(f"[ERROR] No arm reported by the robot at '{cli_args.robot_ip}'")
        reachy.disconnect()
        sys.exit(1)

    reachy.turn_on()

    try:
        _check_and_goto(reachy, TARGET_ARM, TARGET_MATRIX)
    finally:
        reachy.disconnect()


if __name__ == "__main__":
    main()
