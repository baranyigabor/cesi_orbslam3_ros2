#!/usr/bin/env python3

import depthai as dai
import cv2

# Initialize pipeline
pipeline = dai.Pipeline()

# Create a ColorCamera node
cam_rgb = pipeline.create(dai.node.ColorCamera)
cam_rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_4_K)

cam_rgb.setFps(30)

# Enable manual focus control
control_in = pipeline.create(dai.node.XLinkIn)
control_in.setStreamName("control")
control_in.out.link(cam_rgb.inputControl)

# Output stream for the camera
xout_rgb = pipeline.create(dai.node.XLinkOut)
xout_rgb.setStreamName("rgb")
cam_rgb.video.link(xout_rgb.input)

# Start the device
device = dai.Device(pipeline)
rgb_queue = device.getOutputQueue(name="rgb", maxSize=1, blocking=False)
control_queue = device.getInputQueue(name="control")

# Initialize focus value (Range: 0 to 255)
focus_value = 100  # Start with an arbitrary focus value

print("Press 'Q' to increase focus, 'E' to decrease focus, and 'ESC' to exit.")

while True:
    # Get frame from RGB queue
    in_rgb = rgb_queue.get()
    frame = in_rgb.getCvFrame()

    # Display the frame
    cv2.putText(frame, f"Focus: {focus_value}", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.imshow("Live Camera Feed", frame)

    # Read keyboard input
    key = cv2.waitKey(1) & 0xFF

    if key == ord('q') and focus_value < 255:
        focus_value += 5  # Increase focus
    elif key == ord('e') and focus_value > 0:
        focus_value -= 5  # Decrease focus
    elif key == 27:  # ESC key
        print("Exiting...")
        break

    # Send focus update to camera
    control_msg = dai.CameraControl()
    control_msg.setManualFocus(focus_value)
    control_queue.send(control_msg)

    print(f"Updated Focus Value: {focus_value}")

# Cleanup
cv2.destroyAllWindows()
