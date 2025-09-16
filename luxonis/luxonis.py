import depthai as dai
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt

# Create DepthAI pipeline
pipeline = dai.Pipeline()

# Stereo cameras for depth
left = pipeline.create(dai.node.MonoCamera)
right = pipeline.create(dai.node.MonoCamera)
stereo = pipeline.create(dai.node.StereoDepth)

left.setBoardSocket(dai.CameraBoardSocket.CAM_B)  # Left Camera
right.setBoardSocket(dai.CameraBoardSocket.CAM_C)  # Right Camera
left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)

stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
stereo.setLeftRightCheck(True)
stereo.setDepthAlign(dai.CameraBoardSocket.CAM_B)  # Align to Left Camera

left.out.link(stereo.left)
right.out.link(stereo.right)

# IMU for VIO tracking
imu = pipeline.create(dai.node.IMU)
imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, 500)  # 500Hz
imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, 500)  # 500Hz
imu.setBatchReportThreshold(1)
imu.setMaxBatchReports(10)

# Output nodes
depthOut = pipeline.create(dai.node.XLinkOut)
depthOut.setStreamName("depth")

imuOut = pipeline.create(dai.node.XLinkOut)
imuOut.setStreamName("imu")

stereo.depth.link(depthOut.input)
imu.out.link(imuOut.input)

# Position, velocity, and rotation tracking
position = np.array([0.0, 0.0, 0.0], dtype=np.float32)  # x, y, z
velocity = np.array([0.0, 0.0, 0.0], dtype=np.float32)
orientation = R.from_quat([0, 0, 0, 1])  # Quaternion rotation
trajectory = [position.copy()]

# ORB Feature Detector
orb = cv2.ORB_create(1000)

# Start pipeline
with dai.Device(pipeline) as device:
    depthQueue = device.getOutputQueue("depth", maxSize=4, blocking=False)
    imuQueue = device.getOutputQueue("imu", maxSize=50, blocking=False)

    last_timestamp = None
    last_frame = None
    last_keypoints = None
    last_descriptors = None

    while True:
        # Read depth frame
        inDepth = depthQueue.get()
        depthFrame = inDepth.getFrame()

        max_depth = depthFrame.max()
        if max_depth > 0:
            depthFrame = (depthFrame * (255.0 / max_depth)).astype(np.uint8)
            depthFrame = cv2.applyColorMap(depthFrame, cv2.COLORMAP_JET)
        else:
            depthFrame = np.zeros_like(depthFrame, dtype=np.uint8)

        # Read IMU data
        imuData = imuQueue.tryGet()
        if imuData:
            for packet in imuData.packets:
                accel = packet.acceleroMeter
                gyro = packet.gyroscope

                # Get time delta
                current_timestamp = packet.acceleroMeter.timestamp.get()
                if last_timestamp is not None:
                    dt = (current_timestamp - last_timestamp).total_seconds()

                    # Convert gyroscope data to rotation (rad/s)
                    gyro_data = np.array([gyro.x, gyro.y, gyro.z]) * dt
                    dR = R.from_rotvec(gyro_data)  # Convert to rotation matrix
                    orientation = dR * orientation  # Update orientation

                    # Convert accelerometer to world frame & remove gravity
                    accel_data = np.array([accel.x, accel.y, accel.z], dtype=np.float32)
                    accel_corrected = orientation.apply(accel_data)  # Rotate to world frame
                    accel_corrected[2] -= 9.81  # Remove gravity

                    # Integrate to get velocity & position
                    velocity += accel_corrected * dt
                    position += velocity * dt
                    trajectory.append(position.copy())

                last_timestamp = current_timestamp  # Update timestamp

        # ORB Feature Tracking
        grayFrame = cv2.cvtColor(depthFrame, cv2.COLOR_BGR2GRAY)
        keypoints, descriptors = orb.detectAndCompute(grayFrame, None)

        if last_frame is not None and last_keypoints is not None and last_descriptors is not None:
            # Match ORB features
            bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
            matches = bf.match(last_descriptors, descriptors)
            matches = sorted(matches, key=lambda x: x.distance)

            # Extract matching points
            points1 = np.float32([last_keypoints[m.queryIdx].pt for m in matches])
            points2 = np.float32([keypoints[m.trainIdx].pt for m in matches])

            if len(points1) > 10:
                # Estimate motion using Essential Matrix
                E, mask = cv2.findEssentialMat(points1, points2, focal=500, pp=(320, 240), method=cv2.RANSAC, prob=0.999, threshold=1.0)
                _, R_mat, t_vec, _ = cv2.recoverPose(E, points1, points2, focal=500, pp=(320, 240))

                # Convert to numpy
                t_vec = t_vec.flatten()
                R_mat = R.from_matrix(R_mat)

                # Fuse with IMU
                position += t_vec * 0.1  # Scale motion
                orientation = R_mat * orientation  # Update orientation
                trajectory.append(position.copy())

        # Store previous frame and keypoints
        last_frame = grayFrame
        last_keypoints = keypoints
        last_descriptors = descriptors

        # Show depth frame
        cv2.imshow("Depth Map", depthFrame)

        # Plot trajectory
        if len(trajectory) > 1:
            traj_array = np.array(trajectory)
            plt.clf()
            plt.plot(traj_array[:, 0], traj_array[:, 2], marker='o', linestyle="-", markersize=2)
            plt.xlabel("X (m)")
            plt.ylabel("Z (m)")
            plt.title("Camera 3D Trajectory")
            plt.pause(0.001)

        if cv2.waitKey(1) == ord('q'):
            break

cv2.destroyAllWindows()
plt.show()
