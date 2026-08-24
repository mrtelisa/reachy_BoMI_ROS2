# reachy_bomi

A package that turns **hand movements into control of a Reachy 2 robot**: driving its mobile base, and selecting + grasping objects it sees through its torso depth camera.

A webcam tracks the operator's hand with [MediaPipe](https://developers.google.com/mediapipe), a calibrated PCA map converts the hand pose into a 2D cursor, and the cursor position drives either base velocity commands or object hover-selection. Grasp planning turns a YOLOv8 detection + depth camera point cloud into pre-grasp/grasp/lift end-effector poses, executed over [`reachy2_sdk`](https://github.com/pollen-robotics/reachy2-sdk) (gRPC over IP) — no ROS 2 networking is involved between the operator's PC and the robot.

This work started from a ROS 1 implementation written for the TIAGo robot and was ported to ROS 2 for Reachy 2, then moved off ROS 2 topics onto `reachy2_sdk` so the operator's PC doesn't need a ROS 2 distro matching the robot's.

---

## How it works

Everything runs on a **single PC** (any PC with a webcam and network access to the robot); the head-camera live feed runs in a small helper subprocess (`camera_viewer.py`) so its network round-trips never block the main cursor/velocity loop, but there's no ROS 2/robot-side process split:

```
webcam → MediaPipe → PCA cursor → 9-region velocity → reachy2_sdk (gRPC/IP) → mobile base
                                        ↓ (cursor dwell)
                          arms → pre-grasping pose → object hover-select
                                        ↓
      torso depth camera → YOLOv8 → point cloud → grasp planning → arm execution
```

`reachy_control.py` is the only script with a CLI/`main()` for real robot use — `bomi_teleop.py`, `reachy_detection.py`, `reachy_selection.py`, `reachy_pregrasp.py`, `reachy_grasp.py`, `camera_viewer.py`, `graphs.py`, `stream.py`, and `safety.py` are library modules it's built from. Every other way of running things (mouse-driven grasp, dry runs, single-piece diagnostics) lives under `tests/`, see Usage below.

- **`bomi_teleop.py`** — hand tracking → PCA cursor → 9-region velocity building blocks (calibration/cursor-preview phases, the BoMI map, cursor filter, velocity helpers).
- **`reachy_detection.py`** — torso camera → YOLOv8 detection → depth point cloud → grasp geometry building blocks: `capture_and_detect` (grab frame + detect, optionally pre-filtered down to presentable candidates via an injected predicate) and `build_object_point_cloud` (crop/fuse/isolate/measure once an object is confirmed).
- **`reachy_selection.py`** — hover-to-select/confirm UI on top of `reachy_detection.py`'s boxes: dwell-to-select, refresh button, Yes/No confirm, and `presentable_filter` (the reachability/gripper-size pre-filter fed into `capture_and_detect`). `select_object_to_grasp_bomi`/`confirm_grasp_bomi` drive it from the BoMI cursor (used by `reachy_control.py`); `MouseTracker` is a standalone mouse-driven alternative used by `tests/*.py`.
- **`reachy_pregrasp.py`** — moves both arms to the pre-grasping posture (non-blocking) and waits for it, showing a progress-bar window while holding the mobile base at zero speed and keeping the BoMI cursor alive.
- **`reachy_grasp.py`** — grasp planning/execution library: from an `ObjectGeometry` (position, axes, width/height, table normal), computes pre-grasp/grasp/lift end-effector poses and drives the arm through them. Also exposes `is_roughly_reachable`, the cheap single-point reachability check `reachy_selection.py` uses to pre-filter detections.
- **`camera_viewer.py`** — standalone script that shows the head/teleop camera's live feed; spawned by `reachy_control.py` as its own OS process during Control/pre-grasping pose, so the camera's network round-trips stay out of the cursor/velocity loop.
- **`graphs.py`** — matplotlib diagnostics (point cloud stages, planned grasp visualization).
- **`stream.py`** — camera-streaming primitives (non-blocking grab+show, blocking live feed) used by `camera_viewer.py` and, in-process, by `reachy_control.py`'s look-before-you-grasp checkpoint right before a grasp executes.
- **`safety.py`** — quit/shutdown safety net: a local `quit_requested` check (Q/ESC or window closed, while a cv2 window has focus) plus an OS-level global watcher (`pynput`, works regardless of focus, even mid-`arm.goto`) that triggers `emergency_shutdown`.

Dependencies between these run one way only, with no cycles: `reachy_grasp.py`/`bomi_teleop.py` have no dependency on the rest, `reachy_detection.py` depends only on `reachy_grasp.py` (for the shared `ObjectGeometry` type), `reachy_selection.py` depends on both, and `reachy_control.py` ties everything together.

---

## Requirements

- The real Reachy 2 robot reachable over the network, with its SDK server running (mobile base + arms + torso depth camera, depending on which script you run).
- A webcam (for `reachy_control.py`).
- Python deps, in the same environment used to run the scripts:

```bash
pip install reachy2-sdk mediapipe opencv-python scikit-learn numpy scipy ultralytics matplotlib open3d pynput
```

`reachy_control.py` uses the MediaPipe **Tasks API** (`HandLandmarker`), which needs a `hand_landmarker.task` model file — it's not bundled with the `mediapipe` pip package. Download it once and point `--model` at it (default: `hand_landmarker.task` at the package root):

```bash
curl -o hand_landmarker.task \
  https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/latest/hand_landmarker.task
```

`reachy_control.py` uses YOLOv8 (`ultralytics`) for object detection — `yolov8n.pt` (COCO-pretrained) is auto-downloaded on first run, or pass `--yolo-model` to use different weights.

No ROS 2 install is required to run these scripts (they only talk to the robot via `reachy2_sdk`). The package is still packaged as an `ament_python` ROS 2 package for convenience if you keep it in a ROS 2 workspace, but nothing in the runtime code imports `rclpy`.

---

## Installation

Clone the package into the `src/` folder of a (ROS 2, optional) workspace and build it:

```bash
cd ~/ros2_ws/src
git clone https://github.com/mrtelisa/reachy_bomi.git
cd ~/ros2_ws
colcon build --packages-select reachy_bomi
source install/setup.bash
```

Or just run the scripts directly with `python3` — no build step required, since none of them depend on ROS 2 at runtime.

---

## Usage

### Full BoMI flow: teleop + grasp — `reachy_control.py`

The real entry point — everything else in the package exists to support this.

```bash
python3 reachy_bomi/reachy_control.py [robot_ip] [--cam 0] [--model hand_landmarker.task] [--yolo-model yolov8n.pt] [--conf 0.5]
# or, from a built ROS 2 workspace:
ros2 run reachy_bomi reachy_control [robot_ip] [--cam 0] [--model hand_landmarker.task] [--yolo-model yolov8n.pt] [--conf 0.5]
```

`robot_ip` is optional if you've set `DEFAULT_ROBOT_IP` in `reachy_control.py` to your robot's IP; otherwise pass it explicitly.

**Phase 1 — Calibration** (always runs first): move your hand through all the positions you intend to use.
`SPACE` records a sample, `ENTER` finishes (minimum 30 samples), `Q`/`Esc`/closing the window quits. The PCA map is only kept in memory for that run, not saved or reloaded.

**Phase 2 — Cursor preview:** the same cursor-map window used in Control is shown, but nothing is sent to the robot yet — also where Reachy's head/teleop camera live feed starts streaming, as its own subprocess (`camera_viewer.py`). Press `ENTER` to proceed into Control, or `Q`/`Esc`/close a window to quit.

**Phase 3 — Control:** hand → PCA cursor → 9-region base velocity, mapped and sent to the robot. Hold the cursor centered (region 5) for `SELECTION_HOLD_SECONDS` straight to move the arms into a pre-grasping pose.

```
 1 | 2 | 3      center (5)          → stop
 4 | 5 | 6      middle column (2,8) → linear only
 7 | 8 | 9      middle row (4,6)    → angular only
                corners (1,3,7,9)   → linear + angular
```

**Phase 3.5 — Pre-grasping pose:** both arms bend to `reachy_pregrasp.PRE_GRASP_ELBOW_PITCH_DEG` (`reachy_pregrasp.goto_pre_grasp_pose`/`wait_for_pre_grasp_pose`), the base holds zero speed; the head-camera feed from Phase 2 just keeps streaming throughout. The cursor/9-region map reappears but sends no speed commands; `ENTER` resumes Control at halved velocity. Hold the cursor centered again for `SELECTION_HOLD_SECONDS` to stop the base and open object selection (the head-camera subprocess is stopped right there, since Phase 4's own checkpoint streams the same camera differently, in-process).

**Phase 4 — Object selection / grasp:** capture → hover-select → Yes/No confirm (`reachy_selection.select_object_to_grasp_bomi`/`confirm_grasp_bomi`) → build point cloud → live feed checkpoint → plan → execute, every hover point being the BoMI cursor (mapped into the window's pixel space) instead of a mouse. "No" or quitting (`Q`/`Esc`/X) returns to Control.

ESC/Q stop the robot from *any* window (including a `graphs.py` plot) or the terminal, at any point — see `safety.py`.

### Everything else — `tests/`

Standalone scripts: dry runs, mouse-driven equivalents, and single-piece diagnostics, none of them needing the full calibration → teleop → grasp flow above to exercise one part of the pipeline:

| Script | What it does |
|---|---|
| `test_teleop.py` | `bomi_teleop.py`'s hand-tracking → velocity pipeline, logged instead of sent (no robot needed) |
| `test_navigation.py` | `reachy_control.py`'s calibration → cursor → pre-grasping-pose flow, fully self-contained (no robot, no depth camera, no YOLO) |
| `test_object_dimensions.py` | Live YOLO + depth size estimation only (robot arms/base never power on) |
| `test_reachability.py` | Whether a single pasted 4×4 end-effector pose is IK-reachable for one arm, and moves there if so — isolates a rejection from the rest of the grasp-planning pipeline |
| `test_grasp.py` | Mouse-driven select → confirm → plan → execute grasp flow (the counterpart to `reachy_selection.py`'s BoMI-cursor-driven `select_object_to_grasp_bomi`/`confirm_grasp_bomi`), starting both arms directly in the `elbow_135` posture |

---

## Package layout

```
reachy_bomi/
├── reachy_bomi/                     # Python package
│   ├── __init__.py
│   ├── reachy_control.py            # THE entry point: ties bomi_teleop/reachy_detection/reachy_selection/reachy_pregrasp/reachy_grasp together under one BoMI cursor
│   ├── bomi_teleop.py               # library: webcam/MediaPipe → PCA cursor → 9-region velocity building blocks
│   ├── reachy_detection.py          # library: torso camera → YOLOv8 → point cloud → grasp geometry building blocks
│   ├── reachy_selection.py          # library: hover-to-select/confirm UI (dwell-select, refresh, Yes/No confirm, presentable_filter)
│   ├── reachy_pregrasp.py           # library: pre-grasping-pose goto + wait-with-progress-bar UI
│   ├── reachy_grasp.py              # library: grasp planning/execution (pose math, IK search, execute_grasp)
│   ├── camera_viewer.py             # standalone script: head-camera live feed, spawned by reachy_control.py as its own process
│   ├── graphs.py                    # library: matplotlib diagnostics (point cloud stages, grasp plan visualization)
│   ├── stream.py                    # library: camera-streaming primitives (torso + teleop cameras)
│   ├── safety.py                    # library: quit/shutdown safety net (local check + OS-level global watcher)
│   ├── yolov8n.pt                   # YOLOv8 weights (auto-downloaded by ultralytics on first run)
│   └── tests/                       # standalone scripts (mouse-driven grasp, dry runs, diagnostics), see Usage above
├── hand_landmarker.task              # MediaPipe model (download separately, see Requirements)
├── reachy_vel.py                     # standalone script: manual mobile-base velocity/lidar-safety-distance calibration, not part of the package
├── resource/
│   └── reachy_bomi                  # ament resource marker
├── package.xml
├── setup.py
├── setup.cfg
├── .gitignore
└── README.md
```

## Known limitations
- Requires a functioning webcam on the machine running `reachy_control.py` (or the teleop-only `tests/test_teleop.py`).
- Grasp planning only handles round objects (`cylinder`/`sphere` shapes in `reachy_detection.SHAPE_BY_CLASS`) — a box's hidden depth can't be recovered from a single camera view the same way.
- The operator's PC and the robot just need network (IP) reachability to each other — no ROS 2 distro matching is required, since communication goes through `reachy2_sdk`'s gRPC interface.
