# reachy_bomi

A package that turns **hand movements into control of a Reachy 2 robot**: driving its mobile base, and selecting + grasping objects it sees through its torso depth camera.

A webcam tracks the operator's hand with [MediaPipe](https://developers.google.com/mediapipe), a calibrated PCA map converts the hand pose into a 2D cursor, and the cursor position drives either base velocity commands or object hover-selection. Grasp planning turns a YOLOv8 detection + depth camera point cloud into pre-grasp/grasp/lift end-effector poses, executed over [`reachy2_sdk`](https://github.com/pollen-robotics/reachy2-sdk) (gRPC over IP) — no ROS 2 networking is involved between the operator's PC and the robot.

This work started from a ROS 1 implementation written for the TIAGo robot and was ported to ROS 2 for Reachy 2, then moved off ROS 2 topics onto `reachy2_sdk` so the operator's PC doesn't need a ROS 2 distro matching the robot's.

---

## How it works

Everything runs in a **single process on a single PC** (any PC with a webcam and network access to the robot):

```
webcam → MediaPipe → PCA cursor → 9-region velocity → reachy2_sdk (gRPC/IP) → mobile base
                                        ↓ (cursor dwell)
                          arms → pre-grasping pose → object hover-select
                                        ↓
      torso depth camera → YOLOv8 → point cloud → grasp planning → arm execution
```

- **`bomi_teleop.py`** — hand tracking → PCA cursor → 9-region velocity → mobile base. Standalone.
- **`reachy_detection.py`** — torso camera → YOLOv8 detection → mouse-hover object selection → depth point cloud → grasp planning/execution. Standalone (mouse-driven, no BoMI cursor).
- **`reachy_control.py`** — merges the two above into one BoMI-cursor-driven flow: drive the base, dwell to move the arms into a pre-grasping pose, then hover-select and grasp an object, all with the same hand-tracked cursor (no mouse anywhere).
- **`reachy_grasp.py`** — grasp planning/execution library used by both `reachy_detection.py` and `reachy_control.py`: from an `ObjectGeometry` (position, axes, width/height, table normal), computes pre-grasp/grasp/lift end-effector poses and drives the arm through them. Not run directly.
- **`graphs.py`** — matplotlib diagnostics (point cloud stages, planned grasp visualization) used by the above. Not run directly.

---

## Requirements

- The real Reachy 2 robot reachable over the network, with its SDK server running (mobile base + arms + torso depth camera, depending on which script you run).
- A webcam (for `bomi_teleop.py` / `reachy_control.py`).
- Python deps, in the same environment used to run the scripts:

```bash
pip install reachy2-sdk mediapipe opencv-python scikit-learn numpy scipy ultralytics matplotlib open3d pynput
```

`bomi_teleop.py` / `reachy_control.py` use the MediaPipe **Tasks API** (`HandLandmarker`), which needs a `hand_landmarker.task` model file — it's not bundled with the `mediapipe` pip package. Download it once and point `--model` at it (default: `hand_landmarker.task` at the package root):

```bash
curl -o hand_landmarker.task \
  https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/latest/hand_landmarker.task
```

`reachy_detection.py` / `reachy_control.py` use YOLOv8 (`ultralytics`) for object detection — `yolov8n.pt` (COCO-pretrained) is auto-downloaded on first run, or pass `--yolo-model` to use different weights.

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

### Base teleop only — `bomi_teleop.py`

```bash
python3 reachy_bomi/bomi_teleop.py [robot_ip] [--model hand_landmarker.task] [--cam 0]
# or, from a built ROS 2 workspace:
ros2 run reachy_bomi bomi_teleop [robot_ip] [--model hand_landmarker.task] [--cam 0]
```

`robot_ip` is optional if you've set `DEFAULT_ROBOT_IP` in `bomi_teleop.py` to your robot's IP; otherwise pass it explicitly.

Every run starts with calibration, then a cursor preview, then goes into control — the PCA map is only kept in memory for that run, not saved or reloaded.

**Phase 1 — Calibration** (always runs first): move your hand through all the positions you intend to use.
`SPACE` records a sample, `ENTER` finishes (minimum 30 samples), `Q`/`Esc`/closing the window quits.

**Phase 2 — Cursor preview:** the same webcam and cursor-map windows used in Control are shown, but nothing is sent to the robot yet. Press `ENTER` to proceed into Control, or `Q`/`Esc`/close a window to quit.

**Phase 3 — Control:** your hand drives the cursor; the cursor position is mapped to base velocities and sent to the robot. Press `Q`/`Esc`, or close either window, to stop the robot and quit.

The control area is a 3×3 grid with a dead zone in the centre:

```
 1 | 2 | 3      center (5)          → stop
 4 | 5 | 6      middle column (2,8) → linear only
 7 | 8 | 9      middle row (4,6)    → angular only
                corners (1,3,7,9)   → linear + angular
```

### Object detection + grasp only (mouse-driven) — `reachy_detection.py`

```bash
python3 reachy_bomi/reachy_detection.py [robot_ip] [--yolo-model yolov8n.pt] [--conf 0.5]
```

Turns the robot on, captures a frame from the torso camera, and runs YOLOv8 on it. Bounding boxes are blue by default; hover the mouse over one to turn it yellow, keep hovering `HOVER_HOLD_SECONDS` straight to confirm it (green) and get a Yes/No grasp prompt. On "Yes", builds the object's point cloud (10-frame depth fusion, table-plane + flying-pixel removal, largest-cluster isolation), plans a grasp (`reachy_grasp.plan_grasp`), and executes it (`reachy_grasp.execute_grasp`): open gripper → pre-grasp → grasp (straight line) → close gripper → lift.

`Q`, `Esc`, or closing the window = quit.

### Full BoMI flow: teleop + grasp — `reachy_control.py`

```bash
python3 reachy_bomi/reachy_control.py [robot_ip] [--cam 0] [--model hand_landmarker.task] [--yolo-model yolov8n.pt] [--conf 0.5]
```

Same Phases 1–2 (Calibration, Cursor preview) as `bomi_teleop.py`, then:

**Phase 3 — Control:** hand → PCA cursor → 9-region base velocity, as above. Hold the cursor centered (region 5) for `SELECTION_HOLD_SECONDS` straight to move the arms into a pre-grasping pose.

**Phase 3.5 — Pre-grasping pose:** both arms bend to `PRE_GRASP_ELBOW_PITCH_DEG`, the base holds zero speed. The cursor/9-region map reappears but sends no speed commands; `ENTER` resumes Control at halved velocity. Hold the cursor centered again for `SELECTION_HOLD_SECONDS` to stop the base and open object selection.

**Phase 4 — Object selection / grasp:** same capture → hover-select → Yes/No confirm flow as `reachy_detection.py`, except every hover point is the BoMI cursor (mapped into the window's pixel space), not the mouse. "No" or quitting (`Q`/`Esc`/X) returns to Control.

### Diagnostics — `tests/`

Standalone scripts, none of them import `bomi_teleop`/`reachy_detection`/`reachy_control` together — each isolates one piece of the pipeline to debug it independently:

| Script | What it checks |
|---|---|
| `test_teleop.py` | `bomi_teleop.py`'s hand-tracking → velocity pipeline, logged instead of sent (no robot needed) |
| `test_reachy_control.py` | `reachy_control.py`'s calibration → cursor → pre-grasping-pose flow, fully self-contained (no robot, no depth camera, no YOLO) |
| `test_object_dimensions.py` | Live YOLO + depth size estimation only (robot arms/base never power on) |
| `test_reachability.py` | Whether a single pasted 4×4 end-effector pose is IK-reachable for one arm, and moves there if so — isolates a rejection from the rest of the grasp-planning pipeline |
| `test_grasp.py` | The same select → plan → execute grasp flow as `reachy_detection.py`, starting both arms directly in the `elbow_135` posture |

---

## Package layout

```
reachy_bomi/
├── reachy_bomi/                     # Python package
│   ├── __init__.py
│   ├── bomi_teleop.py               # webcam/MediaPipe → PCA cursor → reachy2_sdk mobile base
│   ├── reachy_detection.py          # torso camera → YOLOv8 → mouse hover-select → point cloud → grasp
│   ├── reachy_grasp.py              # grasp planning/execution library (pose math, IK search, execute_grasp)
│   ├── reachy_control.py            # merges bomi_teleop + reachy_detection under one BoMI cursor
│   ├── graphs.py                    # matplotlib diagnostics (point cloud stages, grasp plan visualization)
│   └── tests/                       # standalone diagnostic/dry-run scripts, see Usage above
├── hand_landmarker.task              # MediaPipe model (download separately, see Requirements)
├── yolov8n.pt                        # YOLOv8 weights (auto-downloaded by ultralytics on first run)
├── resource/
│   └── reachy_bomi                  # ament resource marker
├── package.xml
├── setup.py
├── setup.cfg
├── .gitignore
└── README.md
```

## Known limitations
- Requires a functioning webcam on the machine running `bomi_teleop.py`/`reachy_control.py`.
- Grasp planning only handles round objects (`cylinder`/`sphere` shapes in `reachy_detection.SHAPE_BY_CLASS`) — a box's hidden depth can't be recovered from a single camera view the same way.
- The operator's PC and the robot just need network (IP) reachability to each other — no ROS 2 distro matching is required, since communication goes through `reachy2_sdk`'s gRPC interface.
