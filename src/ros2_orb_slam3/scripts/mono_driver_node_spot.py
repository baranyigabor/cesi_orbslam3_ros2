#!/usr/bin/env python3

import os
import glob
import time
import argparse
import natsort
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

from std_msgs.msg import String, Float64, Bool # ROS2 string message template

from geometry_msgs.msg import PoseStamped
from message_filters import Subscriber, ApproximateTimeSynchronizer
import json
import numpy as np

class FolderRGBDPublisher(Node):
    def __init__(self, folder_path, settings_name="Luxonis", image_seq="FolderSequence", node_name="folder_rgbd_publisher", fps = 1):
        super().__init__(node_name)

        self.folder_path = folder_path
        self.settings_name = settings_name
        self.image_seq = image_seq
        self.fps = fps
        self.bridge = CvBridge()
        self.frame_id = 0
        self.send_config = True

        # Parameters and Topics
        self.declare_parameter("rgb_topic", "/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/depth/image_raw")
        self.pub_bundle_adjustment_to_agent_name = "/mono_py_driver/bundle_adjustment_bool_msg"
        self.rgb_topic = self.get_parameter("rgb_topic").get_parameter_value().string_value
        self.depth_topic = self.get_parameter("depth_topic").get_parameter_value().string_value

        self.pose_sub = Subscriber(self, PoseStamped, '/orbslam3_ros2/odometry_data')
        self.rgb_sub = Subscriber(self, Image, '/camera/image_raw')
        self.depth_sub = Subscriber(self, Image, '/camera/image_depth')

        # ROS2 Publishers
        self.rgb_pub = self.create_publisher(Image, self.rgb_topic, 10)
        self.depth_pub = self.create_publisher(Image, self.depth_topic, 10)

        # ROS2 Handshake Publishers/Subscribers
        self.pub_exp_config_name = "/mono_py_driver/experiment_settings"
        self.publish_bundle_adjustment_msg = self.create_publisher(Bool, self.pub_bundle_adjustment_to_agent_name, 1)
        self.sub_exp_ack_name = "/mono_py_driver/exp_settings_ack"
        self.publish_exp_config_ = self.create_publisher(String, self.pub_exp_config_name, 1)
        self.subscribe_exp_ack_ = self.create_subscription(String, self.sub_exp_ack_name, self.ack_callback, 10)

        # Build and send config string
        self.exp_config_msg = self.settings_name

        # Load RGB and depth images
        self.rgb_paths = self.load_images("color_image_right")
        self.depth_paths = self.load_images("depth_image_right")

        assert len(self.rgb_paths) == len(self.depth_paths), \
            f"RGB and Depth image counts do not match! RGB: {len(self.rgb_paths)}, Depth: {len(self.depth_paths)}"

        self.get_logger().info(f"✅ Loaded {len(self.rgb_paths)} RGB and Depth image pairs")

        # Handshake timer
        self.handshake_timer = self.create_timer(0.5, self.handshake_with_cpp_node)

        # Image stream timer (starts only after handshake)
        self.image_timer = None

        self.round_counter = 0


        self.ts = ApproximateTimeSynchronizer(
            [self.pose_sub, self.rgb_sub, self.depth_sub],
            queue_size=10,
            slop=0.05,
            allow_headerless=False
        )
        self.ts.registerCallback(self.synced_callback)

        self.get_logger().info('🟢 Sync node initialized: listening for synchronized pose, RGB, and depth frames')

        self.filepath = "transforms_spot.json"
        if os.path.exists(self.filepath):
            with open(self.filepath, 'r') as f:
                self.transforms = json.load(f)
                self.transforms["frames"] = []  # Reset frames to avoid duplicates
        else: 
            self.transforms ={
            "w": 640,
            "h": 480,
            "fl_x": 332.80126953125,
            "fl_y": 315.255615234375,
            "cx": 331.806396484375,
            "cy": 239.77125549316406,
            "k1": 0.0,
            "k2": 0.0,
            "p1": 0.0,
            "p2": 0.0,
            "camera_model": "OPENCV",
            "frames": []}     
    def _save(self):
        with open(self.filepath, 'w') as f:
            json.dump(self.transforms, f, indent=4) 

    def quaternion_to_transformation_matrix(self, x, y, z, qx, qy, qz, qw):

        T_ros_to_nerf = np.array([
        [0, 0, 1, 0],  
        [1, 0, 0, 0],  
        [0, 1, 0, 0], 
        [0, 0, 0, 1]])

        # Apply transformation to each pose
        def convert_ros_pose_to_nerf(pose_ros):
            return T_ros_to_nerf @ pose_ros
        
        """Calculate transformation matrix from translation vector and quaternion."""
        # Translation vector
        translation = np.array([[x], [y], [z]])

        # Rotation matrix from quaternion
        rotation = np.array([
            [1 - 2 * (qy**2 + qz**2), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy)],
            [2 * (qx * qy + qw * qz), 1 - 2 * (qx**2 + qz**2), 2 * (qy * qz - qw * qx)],
            [2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 1 - 2 * (qx**2 + qy**2)]
        ])

        # Combine rotation and translation into a 4x4 transformation matrix
        transformation_matrix = np.eye(4)
        transformation_matrix[:3, :3] = rotation
        transformation_matrix[:3, 3] = translation.flatten()

        #return convert_ros_pose_to_nerf(transformation_matrix)
        return transformation_matrix

    def synced_callback(self, pose_msg, rgb_msg, depth_msg):
        #self.get_logger().info(f"📦 Synced message set received at time {pose_msg.header.stamp.sec}.{pose_msg.header.stamp.nanosec}")

        # From here you can:
        # - Convert images with CvBridge
        # - Extract transformation matrix from pose_msg
        # - Write to JSON, save images, etc.
        # Just a stub for now
        
        if self.round_counter == 5:
            sensor_name = "left"
            timestamp = rgb_msg.header.stamp.sec + rgb_msg.header.stamp.nanosec * 1e-9
            rgb_filename = f"left/color_image_{sensor_name}_{timestamp:.3f}.png"
            depth_filename = f"left/depth_image_{sensor_name}_{timestamp:.3f}.png"

            try:
                cv_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            except Exception as e:
                self.get_logger().error(f"❌ Failed to convert image: {e}")
                return

            cv2.imwrite(rgb_filename, cv_image)

            pos = pose_msg.pose.position
            quat = pose_msg.pose.orientation
            # Convert quaternion to transformation matrix

            transformation_matrix = self.quaternion_to_transformation_matrix(
                        pos.x, pos.y, pos.z,
                        quat.x, quat.y, quat.z, quat.w)

            self.transforms["frames"].append({
                            "file_path": f"{rgb_filename}",  # Use the correct image path from the dictionary
                            "transform_matrix": transformation_matrix.tolist(),
                            #"transform_matrix": [],
                            "colmap_im_id": len(self.transforms["frames"]) + 1
                        })
            self._save()
        else:
            pass

    def load_images(self, keyword):
        pattern = os.path.join(self.folder_path, f"*{keyword}*.png")
        files = glob.glob(pattern)
        sorted_files = natsort.natsorted(files)
        if not sorted_files:
            self.get_logger().error(f"❌ No '{keyword}' images found in {self.folder_path}")
            exit(1)
        return sorted_files

    def handshake_with_cpp_node(self):
        if self.send_config:
            msg = String()
            msg.data = self.exp_config_msg
            self.publish_exp_config_.publish(msg)
            self.get_logger().info(f"🔁 Sent config: {msg.data}")
        else:
            self.handshake_timer.cancel()
            self.get_logger().info("🤝 Handshake complete. Starting RGB-D stream...")
            self.image_timer = self.create_timer(1.0 / self.fps, self.publish_frame)

    def ack_callback(self, msg):
        if msg.data.strip() == "ACK":
            self.get_logger().info("✅ Received ACK from C++ node")
            self.send_config = False

    def publish_frame(self):
        if self.frame_id >= len(self.rgb_paths):
            self.get_logger().info("🔁 Reached end of image sequence. Restarting from first frame.")
            self.frame_id = 0  # Reset to loop infinitely
            ba_msg = Bool()
            ba_msg.data = True  # Set the trigger to true
            self.publish_bundle_adjustment_msg.publish(ba_msg)
            if self.round_counter >= 5:
                self.get_logger().info("🛑 Stopping after 5 rounds.")
                self.destroy_node()
                return
            
            self.round_counter += 1

        rgb_path = self.rgb_paths[self.frame_id]
        depth_path = self.depth_paths[self.frame_id]

        rgb_image = cv2.imread(rgb_path)
        depth_image = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)  # Keep 16-bit depth

        if rgb_image is None or depth_image is None:
            self.get_logger().warn(f"⚠️ Failed to read image pair: {rgb_path}, {depth_path}")
            self.frame_id += 1
            return

        timestamp = self.get_clock().now().to_msg()

        # Publish RGB
        rgb_msg = self.bridge.cv2_to_imgmsg(rgb_image, encoding="bgr8")
        rgb_msg.header.stamp = timestamp
        self.rgb_pub.publish(rgb_msg)

        # Publish Depth
        if depth_image.dtype != 'uint16':
            depth_image = depth_image.astype('uint16')  # Ensure mono16 type
        depth_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding="mono16")
        depth_msg.header.stamp = timestamp
        self.depth_pub.publish(depth_msg)

        self.get_logger().info(f"📸 Published Frame {self.frame_id + 1}/{len(self.rgb_paths)}: {os.path.basename(rgb_path)}")
        self.frame_id += 1


def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser(description='ROS2 RGB-D Folder Publisher with Handshake')
    parser.add_argument('--folder', required=True, help='Path to folder with RGB and Depth images')
    parser.add_argument('--settings_name', default="Luxonis", help='Config setting name (e.g., EuRoC)')
    parser.add_argument('--image_seq', default="FolderSequence", help='Image sequence name or tag')
    parser.add_argument('--fps', default="10", help='FPS for publishing images', type=int)
    parsed_args = parser.parse_args()

    node = FolderRGBDPublisher(
        folder_path=parsed_args.folder,
        settings_name=parsed_args.settings_name,
        image_seq=parsed_args.image_seq,
        fps=parsed_args.fps,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
