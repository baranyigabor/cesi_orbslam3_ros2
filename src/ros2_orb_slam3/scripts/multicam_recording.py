import depthai as dai
import cv2
import os
from datetime import datetime
import numpy as np

def create_camera_directories(base_path="."):
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    session_dir = os.path.join(base_path, timestamp)
    os.makedirs(session_dir, exist_ok=True)

    cam_dirs = {
        "CAM_A": os.path.join(session_dir, "CAM_A"),  # RGB
        "CAM_B": os.path.join(session_dir, "CAM_B"),  # Mono Left
        "CAM_C": os.path.join(session_dir, "CAM_C"),  # Mono Right
    }

    for name, path in cam_dirs.items():
        os.makedirs(path, exist_ok=True)
        print(f"Created: {path}")

    return cam_dirs

def main():
    # Create folders
    output_dirs = create_camera_directories("/home/gables/Programs/nipg_gsfusion")

    # Set up the DepthAI pipeline
    pipeline = dai.Pipeline()

    # RGB camera
    cam_rgb = pipeline.create(dai.node.ColorCamera)
    cam_rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    cam_rgb.setFps(30)

    # Mono cameras
    mono_left = pipeline.create(dai.node.MonoCamera)
    mono_right = pipeline.create(dai.node.MonoCamera)
    mono_left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
    mono_right.setBoardSocket(dai.CameraBoardSocket.CAM_C)
    mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_800_P)
    mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_800_P)

    # Create output nodes
    xout_rgb = pipeline.create(dai.node.XLinkOut)
    xout_left = pipeline.create(dai.node.XLinkOut)
    xout_right = pipeline.create(dai.node.XLinkOut)
    xout_rgb.setStreamName("rgb")
    xout_left.setStreamName("left")
    xout_right.setStreamName("right")

    # Linking
    cam_rgb.video.link(xout_rgb.input)
    mono_left.out.link(xout_left.input)
    mono_right.out.link(xout_right.input)

    # Start device
    with dai.Device(pipeline) as device:
        q_rgb = device.getOutputQueue("rgb", maxSize=4, blocking=False)
        q_left = device.getOutputQueue("left", maxSize=4, blocking=False)
        q_right = device.getOutputQueue("right", maxSize=4, blocking=False)

        frame_id = 0

        print("[INFO] Starting capture. Press 'q' to quit.")
        while True:
            rgb_frame = q_rgb.get().getCvFrame()
            left_frame = q_left.get().getCvFrame()
            right_frame = q_right.get().getCvFrame()

            # Save frames as PNG
            cv2.imwrite(os.path.join(output_dirs["CAM_A"], f"frame_{frame_id:06d}.png"), rgb_frame)
            cv2.imwrite(os.path.join(output_dirs["CAM_B"], f"frame_{frame_id:06d}.png"), left_frame)
            cv2.imwrite(os.path.join(output_dirs["CAM_C"], f"frame_{frame_id:06d}.png"), right_frame)

            print(f"Saved frame {frame_id}")

            frame_id += 1

            # Show preview
            #cv2.imshow("RGB", rgb_frame)
            #cv2.imshow("Left Mono", left_frame)
            #cv2.imshow("Right Mono", right_frame)

            key = cv2.waitKey(1)
            if key == ord('q'):
                break

        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
