#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import depthai as dai
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class OAKDPublisher(Node):
    def __init__(self):
        super().__init__('oakd_publisher')

        # Declare ROS2 parameters for topics
        self.declare_parameter("rgb_topic", "/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/depth/image_raw")

        # Retrieve topic names from parameters
        self.rgb_topic = self.get_parameter("rgb_topic").get_parameter_value().string_value
        self.depth_topic = self.get_parameter("depth_topic").get_parameter_value().string_value

        # ROS2 Publishers
        self.rgb_pub = self.create_publisher(Image, self.rgb_topic, 10)
        self.depth_pub = self.create_publisher(Image, self.depth_topic, 10)

        # CvBridge for converting images
        self.bridge = CvBridge()

        # Initialize DepthAI Pipeline
        self.pipeline = dai.Pipeline()

        # Create color camera node
        cam_rgb = self.pipeline.create(dai.node.ColorCamera)
        cam_rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)  # Updated from RGB
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam_rgb.setFps(30)

        # Create mono cameras for depth
        mono_left = self.pipeline.create(dai.node.MonoCamera)
        mono_right = self.pipeline.create(dai.node.MonoCamera)
        mono_left.setBoardSocket(dai.CameraBoardSocket.CAM_B)  # Updated from LEFT
        mono_right.setBoardSocket(dai.CameraBoardSocket.CAM_C)  # Updated from RIGHT
        mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)

        # Create stereo depth node
        stereo = self.pipeline.create(dai.node.StereoDepth)
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)  # Updated from HIGH_DENSITY
        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)  # Updated from RGB
        stereo.setLeftRightCheck(True)
        stereo.setExtendedDisparity(False)
        stereo.setSubpixel(True)

        # Link mono cameras to depth node
        mono_left.out.link(stereo.left)
        mono_right.out.link(stereo.right)

        # Create output nodes
        xout_rgb = self.pipeline.create(dai.node.XLinkOut)
        xout_depth = self.pipeline.create(dai.node.XLinkOut)
        xout_rgb.setStreamName("rgb")
        xout_depth.setStreamName("depth")

        # Link nodes to outputs
        cam_rgb.video.link(xout_rgb.input)
        stereo.depth.link(xout_depth.input)

        # Start the device and process frames
        self.device = dai.Device(self.pipeline)
        self.rgb_queue = self.device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
        self.depth_queue = self.device.getOutputQueue(name="depth", maxSize=4, blocking=False)

        self.get_logger().info("OAK-D Publisher Node Started")
        self.timer = self.create_timer(1.0 / 30.0, self.publish_frames)  # Publish at 30 FPS

    def publish_frames(self):
        # Get RGB frame
        rgb_frame = self.rgb_queue.get().getCvFrame()
        depth_frame = self.depth_queue.get().getCvFrame()

        # Convert depth to millimeters
        depth_frame = depth_frame.astype(np.uint16)

        # Convert and publish RGB image
        rgb_msg = self.bridge.cv2_to_imgmsg(rgb_frame, encoding="bgr8")
        rgb_msg.header.stamp = self.get_clock().now().to_msg()
        self.rgb_pub.publish(rgb_msg)

        # Convert and publish Depth image
        depth_msg = self.bridge.cv2_to_imgmsg(depth_frame, encoding="mono16")
        depth_msg.header.stamp = rgb_msg.header.stamp  # Ensure timestamps match
        self.depth_pub.publish(depth_msg)

        self.get_logger().info("Published RGB & Depth Frames")

def main(args=None):
    rclpy.init(args=args)
    node = OAKDPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down OAK-D Publisher")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
