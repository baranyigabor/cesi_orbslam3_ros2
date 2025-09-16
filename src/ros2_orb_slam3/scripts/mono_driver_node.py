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
import pyzed.sl as sl
import roslibpy

#* ROS2 imports
import ament_index_python.packages
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from pypylon import pylon
from PIL import Image

# If you have more files in the submodules folder
# from .submodules.py_utils import fn1 # Import helper functions from files in your submodules folder

# Import a custom message interface
# from your_custom_msg_interface.msg import CustomMsg #* Note the camel caps convention

# Import ROS2 message templates
from sensor_msgs.msg import Image # http://wiki.ros.org/sensor_msgs
from std_msgs.msg import String, Float64, Bool # ROS2 string message template
from cv_bridge import CvBridge, CvBridgeError # Library to convert image messages to numpy array

#* Class definition
class MonoDriver(Node):
    def __init__(self, node_name = "mono_py_node"):
        super().__init__(node_name) # Initializes the rclpy.Node class. It expects the name of the node

        #self.rosbridge_client = roslibpy.Ros(host='localhost', port=9090)
        #self.rosbridge_client.run()

        # Initialize parameters to be passed from the command line (or launch file)
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

        # Global path definitions
        self.home_dir = "/ros2_test/src/ros2_orb_slam3" #! Change this to match path to your workspace
        self.parent_dir = "TEST_DATASET" #! Change or provide path to the parent directory where data for all image sequences are stored
        self.image_sequence_dir = self.home_dir + "/" + self.parent_dir + "/" + self.image_seq # Full path to the image sequence folder

        print(f"self.image_sequence_dir: {self.image_sequence_dir}\n")

        # Global variables
        self.node_name = "mono_py_driver"
        self.image_seq_dir = ""
        self.imgz_seqz = []
        self.time_seqz = [] # Maybe redundant

        # Define a CvBridge object
        self.br = CvBridge()

        # Read images from the chosen dataset, order them in ascending order and prepare timestep data as well
        self.imgz_seqz_dir, self.imgz_seqz, self.time_seqz = self.get_image_dataset_asl(self.image_sequence_dir, "mav0") 

        print(self.image_seq_dir)
        print(len(self.imgz_seqz))

        #* ROS2 publisher/subscriber variables [HARDCODED]
        self.pub_exp_config_name = "/mono_py_driver/experiment_settings" 
        self.sub_exp_ack_name = "/mono_py_driver/exp_settings_ack"
        self.pub_img_to_agent_name = "/camera/color/image_raw"
        self.pub_timestep_to_agent_name = "/mono_py_driver/timestep_msg"
        self.pub_depth_img_to_agent_name = "/camera/depth/image_raw"
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
        
        self.publish_timestep_msg_ = self.create_publisher(Float64, self.pub_timestep_to_agent_name, 1)
        self.publish_bundle_adjustment_msg = self.create_publisher(Bool, self.pub_bundle_adjustment_to_agent_name, 1)

        # Initialize work variables for main logic
        self.start_frame = 0 # Default 0
        self.end_frame = -1 # Default -1
        self.frame_stop = -1 # Set -1 to use the whole sequence, some positive integer to force sequence to stop, 350 test2, 736 test3
        self.show_imgz = False # Default, False, set True to see the output directly from this node
        self.frame_id = 0 # Integer id of an image frame
        self.frame_count = 0 # Ensure we are consistent with the count number of the frame
        self.inference_time = [] # List to compute average time

        print()
        print(f"MonoDriver initialized, attempting handshake with CPP node")
    # ****************************************************************************************

    # ****************************************************************************************
    def get_image_dataset_asl(self, exp_dir, agent_name = "mav0"):
        """
            Returns images and list of timesteps in ascending order from a ASL formatted dataset
        """
        
        # Define work variables
        imgz_file_list = []
        time_list = []

        #* Only works for EuRoC MAV format
        agent_cam0_fld = exp_dir + "/" + agent_name + "/" + "cam0"
        imgz_file_dir = agent_cam0_fld + "/" + "data" + "/"
        imgz_file_list = natsort.natsorted(os.listdir(imgz_file_dir),reverse=False)
        # print(len(img_file_list)) # Debug, checks the number of rgb images

        # Extract timesteps from image names
        for iox in imgz_file_list:
            time_step = iox.split(".")[0]
            time_list.append(time_step)
            #print(time_step)

        return imgz_file_dir, imgz_file_list, time_list
    # ****************************************************************************************

    # ****************************************************************************************
    def ack_callback(self, msg):
        """
            Callback function
        """
        print(f"Got ack: {msg.data}")
        
        if(msg.data == "ACK"):
            self.send_config = False
            # self.subscribe_exp_ack_.destory() # TODO doesn't work 
    # ****************************************************************************************
    
    # ****************************************************************************************
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
    # ****************************************************************************************
    
    # ****************************************************************************************
    def run_py_node(self, idx, imgz_name):
        """
            Master function that sends the RGB image message to the CPP node
        """

        # Initialize work variables
        img_msg = None # sensor_msgs image object

        # Path to this image
        img_look_up_path = self.imgz_seqz_dir  + imgz_name
        timestep = float(imgz_name.split(".")[0]) # Kept if you use a custom message interface to also pass timestep value
        self.frame_id = self.frame_id + 1  
        #print(img_look_up_path)
        # print(f"Frame ID: {frame_id}")

        # Based on the tutorials
        img_msg = self.br.cv2_to_imgmsg(cv2.imread(img_look_up_path), encoding="passthrough")
        timestep_msg = Float64()
        timestep_msg.data = timestep

        # Publish RGB image and timestep, must be in the order shown below. I know not very optimum, you can use a custom message interface to send both
        try:
            self.publish_timestep_msg_.publish(timestep_msg) 
            self.publish_img_msg_.publish(img_msg)
        except CvBridgeError as e:
            print(e)
    # ****************************************************************************************

    # ****************************************************************************************
    def run_py_node_webcam(self, idx, imgz_name, frame):
        """
            Master function that sends the RGB image message to the CPP node
        """

        # Initialize work variables
        img_msg = None # sensor_msgs image object

        # Path to this image
        img_look_up_path = self.imgz_seqz_dir  + imgz_name
        timestep = float(imgz_name.split(".")[0]) # Kept if you use a custom message interface to also pass timestep value
        self.frame_id = self.frame_id + 1  
        #print(img_look_up_path)
        # print(f"Frame ID: {frame_id}")

        # Based on the tutorials
        print(f'webcam shape: {frame.shape}')
        _img = cv2.imread(img_look_up_path)
        cv2.imwrite('/ros_noetic_base_2204/ros_noetic_base_2204/output.jpg', frame)
        print(f'original shape: {_img.shape}')
        img_msg = self.br.cv2_to_imgmsg(frame, encoding="passthrough")
        timestep_msg = Float64()
        timestep_msg.data = timestep

        # Publish RGB image and timestep, must be in the order shown below. I know not very optimum, you can use a custom message interface to send both
        try:
            self.publish_timestep_msg_.publish(timestep_msg) 
            self.publish_img_msg_.publish(img_msg)
        except CvBridgeError as e:
            print(e)
    # ****************************************************************************************

    # ****************************************************************************************
    def run_py_node_basler(self, frame):
        """
            Master function that sends the RGB image message to the CPP node
        """
        print(self.frame_id)
        # Initialize work variables
        img_msg = None # sensor_msgs image object

        timestep = 1234.121 # Kept if you use a custom message interface to also pass timestep value
        self.frame_id = self.frame_id + 1  
        #print(img_look_up_path)
        # print(f"Frame ID: {frame_id}")

        # Based on the tutorials
        # Based on the tutorials
        #cv2.imwrite('/ros2_test/output.jpg', frame)
        #img_msg = self.br.cv2_to_imgmsg(cv2.imread('/ros2_test/output.jpg'), encoding="passthrough")
        
        timestep_msg = Float64()
        timestep_msg.data = time.time()

        # Publish RGB image and timestep, must be in the order shown below. I know not very optimum, you can use a custom message interface to send both
        try:
            self.publish_timestep_msg_.publish(timestep_msg) 
            self.publish_img_msg_.publish(self.br.cv2_to_imgmsg(frame))
        except CvBridgeError as e:
            print(e)
    # ****************************************************************************************

    # ****************************************************************************************
    def run_py_node_zed2(self, frame, depth_img):
        """
            Master function that sends the RGB image message to the CPP node
        """
        print(f"\rFrame ID: {self.frame_id}", end="", flush=True)
        # Initialize work variables
        img_msg = None # sensor_msgs image object

        timestep = 1234.121 # Kept if you use a custom message interface to also pass timestep value
        self.frame_id = self.frame_id + (1*self.manual_direction_changer)  

        timestep_msg = Float64()
        timestep_msg.data = time.time()

        # Publish RGB image and timestep, must be in the order shown below. I know not very optimum, you can use a custom message interface to send both
        try:         
            self.publish_timestep_msg_.publish(timestep_msg) 

            temp_id = self.frame_id +1
            
            if self.stream_on:
                time.sleep(0.2)
                if temp_id > 4000:
                    exit(0)

            if temp_id > 4000:
                self.manual_direction_changer=self.manual_direction_changer*(-1)
            
            if self.frame_id == 0:
                print("we are at the beginning again! -- start bridge")
                ba_msg = Bool()
                ba_msg.data = True  # Set the trigger to true
                self.publish_bundle_adjustment_msg.publish(ba_msg)
                self.manual_direction_changer=self.manual_direction_changer*(-1)
                self.stream_on = True
                time.sleep(10)
            #filename = f"bosch_demo/frames/frame_{temp_id:03d}.png"
            filename = f"pres2/output_frame_{temp_id:04d}.png"
            cv_img = cv2.imread(filename)
            self.publish_img_msg_.publish(self.br.cv2_to_imgmsg(frame, encoding="bgr8"))
            #self.publish_img_msg_.publish(self.br.cv2_to_imgmsg(np.asarray(cv_img)))
            
            
            #self.publish_img_msg_.publish(self.br.cv2_to_imgmsg(frame))
            #Publish depth image! 
            if depth_img is not None:
                self.publish_depth_img_msg_.publish(self.br.cv2_to_imgmsg(depth_img))
            time.sleep(0.1)
        except CvBridgeError as e:
            print(e)
    # ****************************************************************************************
        

# main function
def main(args = None):
    rclpy.init(args=args) # Initialize node
    n = MonoDriver("mono_py_node") #* Initialize the node
    rate = n.create_rate(20) # https://answers.ros.org/question/358343/rate-and-sleep-function-in-rclpy-library-for-ros2/
    
    #* Blocking loop to initialize handshake
    while(n.send_config == True):
        n.handshake_with_cpp_node()
        rclpy.spin_once(n)
        #self.rate.sleep(10) # Potential bug, breaks code

        if(n.send_config == False):
            break
        
    print(f"Handshake complete")

    # Create a Camera object
    
    zed = sl.Camera()

    # Define configuration parameters
    init_params = sl.InitParameters()
    init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE  # Set the depth mode
    init_params.coordinate_units = sl.UNIT.MILLIMETER  # Use millimeter units

    init_params.camera_resolution = sl.RESOLUTION.HD1080
    init_params.camera_fps = 1

    # Open the camera
    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        print(err)
        exit(1)

    # Define a runtime parameters object
    runtime_parameters = sl.RuntimeParameters()

    # Create objects to store images and depth
    image = sl.Mat()
    depth = sl.Mat()
      
    #* Blocking loop to send RGB image and timestep message
    """ for idx, imgz_name in enumerate(n.imgz_seqz[n.start_frame:n.end_frame]):
        try:
            rclpy.spin_once(n) # Blocking we need a non blocking take care of callbacks
            n.run_py_node(idx, imgz_name)
            rate.sleep()

            # DEBUG, if you want to halt sending images after a certain Frame is reached
            if (n.frame_id>n.frame_stop and n.frame_stop != -1):
                print(f"BREAK!")
                break
        
        except KeyboardInterrupt:
            break """
    
    while True:
        try:
            if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
                zed.retrieve_image(image, sl.VIEW.LEFT)
                # Retrieve the depth map
                zed.retrieve_measure(depth, sl.MEASURE.DEPTH)

                # Convert the images to OpenCV format
                image_ocv = image.get_data()
                image_ocv = cv2.cvtColor(image_ocv, cv2.COLOR_BGR2RGB)
                depth_ocv = depth.get_data()

                # Handle invalid values in the depth map
                depth_ocv[np.isnan(depth_ocv)] = 0
                depth_ocv[np.isinf(depth_ocv)] = 0

                depth_for_ros = depth_ocv.astype(np.float32)

                # Normalize the depth for visualization
                depth_normalized = cv2.normalize(depth_ocv, None, 0, 255, cv2.NORM_MINMAX)
                depth_display = np.uint8(depth_normalized)

                rclpy.spin_once(n) # Blocking we need a non blocking take care of callbacks
                n.run_py_node_zed2(image_ocv[:,:,:3], depth_for_ros)
                #n.run_py_node_basler(frame_undistorted)
                rate.sleep()

                # DEBUG, if you want to halt sending images after a certain Frame is reached
                if (n.frame_id>n.frame_stop and n.frame_stop != -1):
                    print(f"BREAK!")
                    break
        
        except KeyboardInterrupt:
            break
    # Cleanup
    zed.close()
    cv2.destroyAllWindows() # Close all image windows
    n.destroy_node() # Release all resource related to this node
    rclpy.shutdown()

def main2(args = None):
    rclpy.init(args=args) # Initialize node
    n = MonoDriver("mono_py_node") #* Initialize the node
    rate = n.create_rate(20) # https://answers.ros.org/question/358343/rate-and-sleep-function-in-rclpy-library-for-ros2/
    
    #* Blocking loop to initialize handshake
    while(n.send_config == True):
        n.handshake_with_cpp_node()
        rclpy.spin_once(n)
        #self.rate.sleep(10) # Potential bug, breaks code

        if(n.send_config == False):
            break
        
    print(f"Handshake complete")

    # Create objects to store images and depth
    image = sl.Mat()
    depth = sl.Mat()
      
    
    while True:
        try:
            rclpy.spin_once(n) # Blocking we need a non blocking take care of callbacks
            n.run_py_node_zed2(None, None)
                #n.run_py_node_basler(frame_undistorted)
            rate.sleep()

                # DEBUG, if you want to halt sending images after a certain Frame is reached
            if (n.frame_id>n.frame_stop and n.frame_stop != -1):
                print(f"BREAK!")
                break
        
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
# ros2 run ros2_orb_slam3 mono_driver_node.py --ros-args -p settings_name:=EuRoC -p image_seq:=sample_euroc_MH05

