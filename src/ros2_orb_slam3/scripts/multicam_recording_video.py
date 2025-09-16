import depthai as dai
import cv2
import os
from datetime import datetime
import keyboard
import threading

def create_camera_directories(base_path="."):
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    session_dir = os.path.join(base_path, timestamp)
    os.makedirs(session_dir, exist_ok=True)
    
    return session_dir

def create_video_writer(filepath, frame_size, fps=30, is_color=True):
    # Use mp4 (codec: mp4v), or avi (codec: XVID)
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # change to 'XVID' for AVI
    return cv2.VideoWriter(filepath, fourcc, fps, frame_size, is_color)


stop_flag = False

def wait_for_q():
    global stop_flag
    input("Press ENTER to stop recording...\n")
    stop_flag = True

threading.Thread(target=wait_for_q, daemon=True).start()

def main():
    # Create output directories
    base_path = "/home/gables/Programs/nipg_gsfusion"
    session_path = create_camera_directories(base_path)
    print(f"[INFO] Saving session at: {session_path}")

    # Initialize DepthAI pipeline
    pipeline = dai.Pipeline()

    cam_rgb = pipeline.create(dai.node.ColorCamera)
    cam_rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    cam_rgb.setFps(30)
    cam_rgb.initialControl.setManualFocus(120) # Set focus to 120

    mono_left = pipeline.create(dai.node.MonoCamera)
    mono_right = pipeline.create(dai.node.MonoCamera)
    mono_left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
    mono_right.setBoardSocket(dai.CameraBoardSocket.CAM_C)
    mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_800_P)
    mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_800_P)
    mono_left.setFps(30)
    mono_right.setFps(30)

    xout_rgb = pipeline.create(dai.node.XLinkOut)
    xout_left = pipeline.create(dai.node.XLinkOut)
    xout_right = pipeline.create(dai.node.XLinkOut)
    xout_rgb.setStreamName("rgb")
    xout_left.setStreamName("left")
    xout_right.setStreamName("right")

    cam_rgb.video.link(xout_rgb.input)
    mono_left.out.link(xout_left.input)
    mono_right.out.link(xout_right.input)

    frame_id = 0

    with dai.Device(pipeline) as device:
        q_rgb = device.getOutputQueue("rgb", maxSize=4, blocking=False)
        q_left = device.getOutputQueue("left", maxSize=4, blocking=False)
        q_right = device.getOutputQueue("right", maxSize=4, blocking=False)

        # Read initial frames to get size info
        rgb_frame = q_rgb.get().getCvFrame()
        left_frame = q_left.get().getCvFrame()
        right_frame = q_right.get().getCvFrame()

        # Define writers
        rgb_writer = create_video_writer(os.path.join(session_path, "rgb.mp4"), (rgb_frame.shape[1], rgb_frame.shape[0]), is_color=True)
        left_writer = create_video_writer(os.path.join(session_path, "left.mp4"), (left_frame.shape[1], left_frame.shape[0]), is_color=False)
        right_writer = create_video_writer(os.path.join(session_path, "right.mp4"), (right_frame.shape[1], right_frame.shape[0]), is_color=False)

        print("[INFO] Recording... Press 'q' to stop.")

        while not stop_flag:
            rgb_frame = q_rgb.get().getCvFrame()
            left_frame = q_left.get().getCvFrame()
            right_frame = q_right.get().getCvFrame()

            # Write to video
            if frame_id > 30:
                rgb_writer.write(rgb_frame)
                left_writer.write(left_frame)
                right_writer.write(right_frame)

            # Optional live preview
            #cv2.imshow("RGB", rgb_frame)
            #cv2.imshow("Mono Left", left_frame)
            #cv2.imshow("Mono Right", right_frame)
            frame_id += 1

        # Cleanup
        rgb_writer.release()
        left_writer.release()
        right_writer.release()
        #cv2.destroyAllWindows()
        print("[INFO] Recording finished and saved.")

if __name__ == "__main__":
    main()
