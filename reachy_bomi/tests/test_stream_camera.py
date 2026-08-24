
from reachy2_sdk import ReachySDK
from reachy2_sdk.media.camera import CameraView
from PIL import Image
import cv2

def main():
    reachy = ReachySDK(host="192.168.0.121")
    reachy.turn_on()
    reachy.goto_posture("default")

    while True:
        result = reachy.cameras.teleop.get_frame(view=view)
        if result is None:
            continue
            frame, _timestamp = result
            cv2.imshow(window_name, frame)


if __name__ == "__main__":
    main()

