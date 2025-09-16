#!/usr/bin/env python3


"""
Python node for the MonocularMode cpp node.

Author: Azmyin Md. Kamal
Date: 01/01/2024

Requirements
* Dataset must be configured in EuRoC MAV format
* Paths to dataset must be set before bulding (or running) this node
* Make sure to set path to your workspace in common.hpp

Command line arguments
-- settings_name: EuRoC, TUM2, KITTI etc; the name of the .yaml file containing camera intrinsics and other configurations
-- image_seq: MH01, V102, etc; the name of the image sequence you want to run

"""

# Imports
#* Import Python modules
import sys # System specific modules
import os # Operating specific functions
import glob
import time # Python timing module
import copy # For deepcopying arrays
import shutil # High level folder operation tool
from pathlib import Path # To find the "home" directory location
import argparse # To accept user arguments from commandline
import natsort # To ensure all images are chosen loaded in the correct order
import yaml # To manipulate YAML files for reading configuration files
import copy # For making deepcopies of openCV matrices, python lists, numpy arrays etc.
import numpy as np # Python Linear Algebra module
import cv2 # OpenCV
import roslibpy

#* ROS2 imports
import ament_index_python.packages
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

#from pypylon import pylon
from PIL import Image

#DepthAI api
import depthai as dai
from primesense import openni2

# If you have more files in the submodules folder
# from .submodules.py_utils import fn1 # Import helper functions from files in your submodules folder

# Import a custom message interface
# from your_custom_msg_interface.msg import CustomMsg #* Note the camel caps convention

# Import ROS2 message templates
from sensor_msgs.msg import Image # http://wiki.ros.org/sensor_msgs
from std_msgs.msg import String, Float64, Bool # ROS2 string message template
from cv_bridge import CvBridge, CvBridgeError # Library to convert image messages to numpy array

class OAKDPublisher(Node):
    def __init__(self, node_name = "mono_py_node"):
        super().__init__(node_name)

        self.declare_parameter("settings_name","EuRoC")
        self.declare_parameter("image_seq","NULL")

        #* Parse values sent by command line
        self.settings_name = str(self.get_parameter('settings_name').value) 
        self.image_seq = str(self.get_parameter('image_seq').value)

        # DEBUG
        print(f"-------------- Received parameters --------------------------\n")
        print(f"self.settings_name: {self.settings_name}")
        print(f"self.image_seq: {self.image_seq}")
        print()

        self.manual_direction_changer = 1
        self.stream_on = False

        # Global variables
        self.node_name = node_name

        # Define a CvBridge object
        self.br = CvBridge()

        #* ROS2 publisher/subscriber variables [HARDCODED]
        self.pub_exp_config_name = "/mono_py_driver/experiment_settings" 
        self.sub_exp_ack_name = "/mono_py_driver/exp_settings_ack"
        self.pub_img_to_agent_name = "/camera/color/image_raw" # "/mono_py_driver/img_msg"
        self.pub_timestep_to_agent_name =  "/camera/depth/image_raw" # "/mono_py_driver/timestep_msg"
        self.pub_depth_img_to_agent_name = "/mono_py_driver/depth_img_msg"
        self.pub_bundle_adjustment_to_agent_name = "/mono_py_driver/bundle_adjustment_bool_msg"
        self.send_config = True # Set False once handshake is completed with the cpp node
        
        #* Setup ROS2 publishers and subscribers
        self.publish_exp_config_ = self.create_publisher(String, self.pub_exp_config_name, 1) # Publish configs to the ORB-SLAM3 C++ node

        #* Build the configuration string to be sent out
        #self.exp_config_msg = self.settings_name + "/" + self.image_seq # Example EuRoC/sample_euroc_MH05
        self.exp_config_msg = self.settings_name # Example EuRoC
        print(f"Configuration to be sent: {self.exp_config_msg}")


        #* Subscriber to get acknowledgement from CPP node that it received experimetn settings
        self.subscribe_exp_ack_ = self.create_subscription(String, 
                                                           self.sub_exp_ack_name, 
                                                           self.ack_callback ,10)


        # Publisher to send RGB image
        self.publish_img_msg_ = self.create_publisher(Image, self.pub_img_to_agent_name, 1)
        self.publish_depth_img_msg_ = self.create_publisher(Image, self.pub_depth_img_to_agent_name, 1)

        # Initialize work variables for main logic
        self.show_imgz = False # Default, False, set True to see the output directly from this node
        self.frame_id = 0 # Integer id of an image frame
        self.frame_count = 0 # Ensure we are consistent with the count number of the frame

        print()
        print(f"MonoDriver initialized, attempting handshake with CPP node")

        # Declare ROS2 parameters for topics
        self.declare_parameter("rgb_topic", "/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/depth/image_raw")

        # Retrieve topic names from parameters
        self.rgb_topic = self.get_parameter("rgb_topic").get_parameter_value().string_value
        self.depth_topic = self.get_parameter("depth_topic").get_parameter_value().string_value

        # ROS2 Publishers
        self.rgb_pub = self.create_publisher(Image, self.rgb_topic, 10)
        self.depth_pub = self.create_publisher(Image, self.depth_topic, 10)
        self.publish_bundle_adjustment_msg = self.create_publisher(Bool, self.pub_bundle_adjustment_to_agent_name, 1)

        # CvBridge for converting images
        self.bridge = CvBridge()

        ######################### ---------- OBRBEC Camera --------------#########################
        # Initialiser OpenNI
        #openni2.initialize()  # Charger les bibliothèques OpenNI2
        #dev = openni2.Device.open_any()

        ######################### ---------- OBRBEC Camera --------------#########################

        # Initialize DepthAI Pipeline
        '''self.pipeline = dai.Pipeline()    
        
        # Create output nodes
        xout_rgb = self.pipeline.create(dai.node.XLinkOut)
        xout_depth = self.pipeline.create(dai.node.XLinkOut)
        xout_rgb.setStreamName("rgb")
        xout_depth.setStreamName("depth")

        # Create color camera node
        cam_rgb = self.pipeline.create(dai.node.ColorCamera)
        cam_rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)  # Updated from RGB
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_720_P)
        cam_rgb.setFps(30)
        cam_rgb.initialControl.setManualFocus(120) # Set focus to 120
        cam_rgb.setIspScale(2, 3)  # Safe downscale
        cam_rgb.isp.link(xout_rgb.input)
        # Set manual exposure
        exposure_time_us = 10000  # Exposure time in microseconds (e.g., 10000us = 10ms)
        sensitivity_iso = 100     # Sensitivity (ISO) value

        #cam_rgb.initialControl.setManualExposure(exposure_time_us, sensitivity_iso)


        # ✅ Create XLinkIn node for dynamic focus control
        control_in = self.pipeline.create(dai.node.XLinkIn)
        control_in.setStreamName("control")
        control_in.out.link(cam_rgb.inputControl)
        # Create mono cameras for depth
        mono_left = self.pipeline.create(dai.node.MonoCamera)
        mono_right = self.pipeline.create(dai.node.MonoCamera)
        mono_left.setBoardSocket(dai.CameraBoardSocket.CAM_B)  # Updated from LEFT
        mono_right.setBoardSocket(dai.CameraBoardSocket.CAM_C)  # Updated from RIGHT
        mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)

        # Create stereo depth node
        stereo = self.pipeline.create(dai.node.StereoDepth)
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_ACCURACY)  # Updated from HIGH_DENSITY
        #stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A) 
        stereo.setLeftRightCheck(True)
        stereo.setExtendedDisparity(False)
        stereo.setSubpixel(False)
        #stereo.setMedianFilter(dai.MedianFilter.KERNEL_7x7)

        # Link mono cameras to depth node
        mono_left.out.link(stereo.left)
        mono_right.out.link(stereo.right)

        # Link nodes to outputs
        #cam_rgb.video.link(xout_rgb.input)
        stereo.depth.link(xout_depth.input)

        # Start the device and process frames
        self.device = dai.Device(self.pipeline)
        self.rgb_queue = self.device.getOutputQueue(name="rgb", maxSize=7, blocking=False)
        self.depth_queue = self.device.getOutputQueue(name="depth", maxSize=10, blocking=False)

        # ✅ Get control queue now that it exists
        control_queue = self.device.getInputQueue(name="control")

        # ✅ Send manual focus command
        control_msg = dai.CameraControl()
        control_msg.setManualFocus(120)  # Fix focus at 120
        control_queue.send(control_msg)

        
        calibData = self.device.readCalibration()
        #intrinsics = calibData.getCameraIntrinsics(dai.CameraBoardSocket.CAM_A, 1920, 1080)
        intrinsics = calibData.getCameraIntrinsics(dai.CameraBoardSocket.CAM_A, 640, 400)
        distortion = calibData.getDistortionCoefficients(dai.CameraBoardSocket.CAM_A)
        print('CAM_A intrinsics\n:', np.asarray(intrinsics))

        D_rgb = np.array(calibData.getDistortionCoefficients(dai.CameraBoardSocket.CAM_A))

        print("RGB Distortion Coefficients...")
        [print(name + ": " + value) for (name, value) in zip(["k1", "k2", "p1", "p2", "k3", "k4", "k5", "k6", "s1", "s2", "s3", "s4", "τx", "τy"], [str(data) for data in D_rgb])]

        #time.sleep(5)
        stereo_baseline = calibData.getBaselineDistance()
        print(f"✅ Luxonis Stereo Baseline (Stereo.b): {stereo_baseline} centimeters")

        self.get_logger().info("OAK-D Publisher Node Started")
        #self.timer = self.create_timer(1.0 / 30.0, self.publish_frames)  # Publish at 30 FPS
        '''
    def handshake_with_cpp_node(self):
        """
            Send and receive acknowledge of sent configuration settings
        """
        if (self.send_config == True):
            # print(f"Sent mesasge: {self.exp_config_msg}")
            msg = String()
            msg.data = self.exp_config_msg
            self.publish_exp_config_.publish(msg)
            time.sleep(0.01)

    def ack_callback(self, msg):
        """
            Callback function
        """
        print(f"Got ack: {msg.data}")
        
        if(msg.data == "ACK"):
            self.send_config = False
    '''
    def publish_frames(self):
        # Get RGB frame
        rgb_frame = self.rgb_queue.get().getCvFrame()
        depth_frame = self.depth_queue.get().getCvFrame()

        # Convert depth to millimeters
        depth_frame = depth_frame.astype(np.uint16)
        print(f"Depth Min: {np.min(depth_frame)} mm, Max: {np.max(depth_frame)} mm")

        # Convert and publish RGB image
        rgb_msg = self.bridge.cv2_to_imgmsg(rgb_frame, encoding="bgr8")
        rgb_msg.header.stamp = self.get_clock().now().to_msg()
        self.rgb_pub.publish(rgb_msg)

        # Convert and publish Depth image
        depth_msg = self.bridge.cv2_to_imgmsg(depth_frame, encoding="mono16")
        depth_msg.header.stamp = rgb_msg.header.stamp  # Ensure timestamps match
        self.depth_pub.publish(depth_msg)

        depth_vis = cv2.normalize(depth_frame, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        depth_vis = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)

        blended = cv2.addWeighted(rgb_frame, 0.6, depth_vis, 0.4, 0)

        cv2.imshow("RGB", rgb_frame)
        cv2.imshow("Aligned Depth", depth_vis)
        cv2.imshow("RGB + Depth Overlay", blended)
        cv2.waitKey(1)

        self.get_logger().info("Published RGB & Depth Frames")'''

    def publish_frames(self):
        while True:
            # Get the latest RGB frame
            temp_rgb = self.rgb_queue.get()
            rgb_timestamp = temp_rgb.getTimestamp().total_seconds()

            # Wait until a depth frame with a close timestamp arrives
            matched_depth_frame = None
            while matched_depth_frame is None:
                temp_depth = self.depth_queue.get()
                depth_timestamp = temp_depth.getTimestamp().total_seconds()

                # Check if depth and RGB timestamps are within a reasonable sync threshold (e.g., 0.05 sec)
                if abs(rgb_timestamp - depth_timestamp) < 0.04:
                    matched_depth_frame = temp_depth
                    self.frame_id+=1
                elif rgb_timestamp - depth_timestamp < -0.1:
                    print(f"⚠️ Depth frame is too far from RGB frame | RGB: {rgb_timestamp}, Depth: {depth_timestamp}")
                    return
                else:
                    print(f"⏳ Waiting for correct depth frame | RGB: {rgb_timestamp}, Depth: {depth_timestamp}")
            if self.frame_id % 100 == 0 and self.frame_id <= 1000 :
                ba_msg = Bool()
                ba_msg.data = True  # Set the trigger to true
                self.publish_bundle_adjustment_msg.publish(ba_msg)
            delay = abs(rgb_timestamp - depth_timestamp)
            print(f"✅ Synchronized RGB-Depth Pair | Delay: {delay:.3f} sec -- resolution: {matched_depth_frame.getWidth()}x{matched_depth_frame.getHeight()}")

            # Convert and publish RGB image
            rgb_msg = self.bridge.cv2_to_imgmsg(cv2.resize(temp_rgb.getCvFrame(), (640, 400)), encoding="bgr8")
            rgb_msg.header.stamp = self.get_clock().now().to_msg()
            self.rgb_pub.publish(rgb_msg)

            # Convert and publish Depth image
            depth_img_converted = matched_depth_frame.getCvFrame().astype(np.uint16)
            #depth_img_converted[depth_img_converted <= 0] = 1       # minimum depth: 1 mm
            #depth_img_converted[depth_img_converted > 5000] = 5000  # optional: cap max depth to 10 meters
            depth_msg = self.bridge.cv2_to_imgmsg(depth_img_converted, encoding="mono16")
            depth_msg.header.stamp = rgb_msg.header.stamp  # Ensure timestamps match
            self.depth_pub.publish(depth_msg)

            depth_vis = cv2.normalize(matched_depth_frame.getCvFrame().astype(np.uint16), None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
            depth_vis = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)

            #blended = cv2.addWeighted(temp_rgb.getCvFrame(), 0.6, depth_vis, 0.4, 0)

            '''cv2.imshow("RGB", temp_rgb.getCvFrame())
            cv2.imshow("Aligned Depth", depth_vis)
            cv2.imshow("RGB + Depth Overlay", blended)
            cv2.waitKey(1)'''


def main(args = None):
    rclpy.init(args=args) # Initialize node
    n = OAKDPublisher("Luxonis_node") #* Initialize the node
    rate = n.create_rate(20) # https://answers.ros.org/question/358343/rate-and-sleep-function-in-rclpy-library-for-ros2/
    
    #* Blocking loop to initialize handshake
    while(n.send_config == True):
        n.handshake_with_cpp_node()
        rclpy.spin_once(n)
        #self.rate.sleep(10) # Potential bug, breaks code

        if(n.send_config == False):
            break
        
    print(f"Handshake complete")      
    
    while True:
        try:
            rclpy.spin_once(n) # Blocking we need a non blocking take care of callbacks
            #n.run_py_node_zed2(None, None)
            #print("almafa")
            #exit()
            n.publish_frames()
                #n.run_py_node_basler(frame_undistorted)
            rate.sleep()

                # DEBUG, if you want to halt sending images after a certain Frame is reached
            '''if (n.frame_id>n.frame_stop and n.frame_stop != -1):
                print(f"BREAK!")
                break'''
        
        except KeyboardInterrupt:
            break
    # Cleanup
    cv2.destroyAllWindows() # Close all image windows
    n.destroy_node() # Release all resource related to this node
    rclpy.shutdown()

# Dunders, this .py is the main file
if __name__=="__main__":
    main()

# terminal 1
# ros2 run ros2_orb_slam3 mono_node_cpp --ros-args -p node_name_arg:=mono_slam_cpp

# terminal 2
# python3 src/ros2_orb_slam3/scripts/mono_driver_node_luxonis.py --ros-args -p settings_name:=Luxonis -p image_seq:=sample_euroc_MH05


