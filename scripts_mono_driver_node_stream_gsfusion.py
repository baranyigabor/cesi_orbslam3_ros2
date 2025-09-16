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
#import zmq 
import csv
from tqdm import tqdm
#* ROS2 imports
import ament_index_python.packages
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
# If you have more files in the submodules folder
# from .submodules.py_utils import fn1 # Import helper functions from files in your submodules folder

# Import a custom message interface
# from your_custom_msg_interface.msg import CustomMsg #* Note the camel caps convention

# Import ROS2 message templates
from sensor_msgs.msg import Image # http://wiki.ros.org/sensor_msgs
from std_msgs.msg import String, Float64 # ROS2 string message template
from geometry_msgs.msg import PoseStamped, TransformStamped
from cv_bridge import CvBridge, CvBridgeError # Library to convert image messages to numpy array
import requests
import msgpack
import msgpack_numpy as m
import http.client
import base64
class DataPublisher(Node):
    def __init__(self, dataset_path):
        super().__init__('image_publisher')



class MonoDriver(Node):
    def __init__(self, node_name = "mono_py_node",dataset_path = "./data/"):
        super().__init__(node_name)
        self.declare_parameter("settings_name","iPhone")
        self.declare_parameter("url","http://192.168.0.32:8080/shot.jpg")
        self.settings_name = str(self.get_parameter('settings_name').value) 
        self.url = str(self.get_parameter('url').value)
        print(f"-------------- Received parameters --------------------------\n")
        print(f"self.settings_name: {self.settings_name}")
        print(f"self.url: {self.url}")
        print(f"-------------------------------------------------------------\n")
        self.node_name = "mono_py_driver"
        self.br = CvBridge()
        self.pub_exp_config_name = "/mono_py_driver/experiment_settings" 
        self.sub_exp_ack_name = "/mono_py_driver/exp_settings_ack"
        self.pub_img_to_agent_name = "/camera/color/image_raw"
        self.pub_depth_to_agent_name = "/camera/depth/image_raw"
        self.pub_timestep_to_agent_name = "/mono_py_driver/timestep_msg"
        self.sub_pose_name = "/mono_py_driver/pose_msg"
        self.pub_termination_name = "/mono_py_driver/termination_msg"
        self.send_config = True
        self.publish_exp_config_ = self.create_publisher(String, self.pub_exp_config_name, 1)
        self.exp_config_msg = self.settings_name
        print(f"Configuration to be sent: {self.exp_config_msg}")
        self.subscribe_exp_ack_ = self.create_subscription(String,self.sub_exp_ack_name,self.ack_callback ,10)
        self.subscribe_pose_msg = self.create_subscription(String,self.sub_pose_name,self.pose_callback, 10)
        self.publish_img_msg_ = self.create_publisher(Image, self.pub_img_to_agent_name, 1)
        self.publish_depth_msg_ = self.create_publisher(Image, self.pub_depth_to_agent_name, 1)
        self.publish_timestep_msg_ = self.create_publisher(Float64, self.pub_timestep_to_agent_name, 1)
        self.publish_termination_msg_ = self.create_publisher(String, self.pub_termination_name, 1)
        self.subscribe_sync_signal_ = self.create_subscription(String,"/mono_py_driver/sync_msg",self.sync_callback ,10)
        self.start_frame = 0
        self.end_frame = -1
        self.frame_stop =1900 # Set -1 to use the whole sequence, some positive integer to force sequence to stop, 350 test2, 736 test3
        
        #base_path = "/media/buvar/dtdata/sam3d_prep/ESAM/data/"
        base_path = ""
        
        #self.imgdir_path = base_path+"BK_UPaFbq/color/"
        #self.depthdir_path = base_path+"BK_UPaFbq/depth/"
        # LB_DcGdi8
        #self.imgdir_path = base_path+"LB_DcGdi8/color/"
        #self.depthdir_path = base_path+"LB_DcGdi8/depth/"
        self.imgdir_path = base_path+""
        self.depthdir_path = base_path+""
        
        
        self.frame_id = 0 # Integer id of an image frame
        self.localization_buffer = 50
        self.allowed_to_send = True
        self.rgb_img = []
        self.depth_img = []
        self.pose = np.asarray([[1.0,0.0,0.0,0.0],
                               [0.0,1.0,0.0,0.0],
                               [0.0,0.0,1.0,0.0],
                               [0.0,0.0,0.0,1.0]])
        self.empty_pose = np.asarray([[1.0,0.0,0.0,0.0],
                               [0.0,1.0,0.0,0.0],
                               [0.0,0.0,1.0,0.0],
                               [0.0,0.0,0.0,1.0]])
        self.correction_mat = np.asarray([[1.0,0.0,0.0,0.0],
                                         [0.0,1.0,0.0,0.0],
                                         [0.0,0.0,1.0,0.0],
                                          [0.0,0.0,0.0,1.0]])
        self.corrected_pose = np.asarray([[1.0,0.0,0.0,0.0],
                               [0.0,1.0,0.0,0.0],
                               [0.0,0.0,1.0,0.0],
                               [0.0,0.0,0.0,1.0]])
        print(f"MonoDriver initialized, attempting handshake with CPP node")
        self.rgb_publisher_ = self.create_publisher(Image, 'camera/image_raw', 10)
        self.depth_publisher_ = self.create_publisher(Image, 'camera/image_depth', 10)
        self.odometry_publisher_ = self.create_publisher(PoseStamped, 'orbslam3_ros2/odometry_data', 10)
        #self.odometry_publisher_ = self.create_publisher(TransformStamped, 'orbslam3_ros2/odometry_data', 10)
        #self.transform_broadcaster_ = tf2_ros.TransformBroadcaster(self)  # Publishes 4x4 matrix as Transform
        self.timer = self.create_timer(0.1, self.publish_data)  # 10 Hz
        self.bridge = CvBridge()
        self.get_logger().info("Data publisher node started.")
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.yaw = 0.0  # Orientation around Z-axis

        self.dataset_path = dataset_path
        self.frame_index = 0  # Track current frame

        self.csv_files = []
        self.img_files = []
        # The maximum line length for HTTP headers
        # This is necessary to avoid the "line too long" error when receiving large headers, and in our case the depth data is 512964 bytes long
        http.client._MAXLINE = 600000
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
    def pose_callback(self, msg):
        """
            Callback function
        """
        raw_pose_msg = msg.data
        frame_id = msg.data.split("#")[1]
        raw_pose_data = msg.data.split("#")[2]
        print(f"Received pose data for frame [{frame_id}]:\n{raw_pose_data}")
        self.pose = np.asarray([float(x) for x in raw_pose_data.replace("\n"," ").split(' ')[:-1]]).reshape(4,4)
        self.corrected_pose = self.pose @ self.correction_mat
        print(f"Converted to {self.pose}")
        print(f"Corrected pose: {self.corrected_pose}")
        return 0
    # ****************************************************************************************
    
        # ****************************************************************************************
    def sync_callback(self, msg):
        """
            Callback function
        """
        if(msg.data == "SYNC"):
            self.allowed_to_send = True
        return 0
    # ****************************************************************************************
    
    # ****************************************************************************************
    def handshake_with_cpp_node(self):
        """
            Send and receive acknowledge of sent configuration settings
        """
        if (self.send_config == True):
            msg = String()
            msg.data = self.exp_config_msg
            self.publish_exp_config_.publish(msg)
            time.sleep(0.01)
    # ****************************************************************************************
    def list_csv_img_pairs(self,folder_path):
        self.csv_files = [os.path.join(folder_path,f) for f in os.listdir(folder_path) if f.endswith('.csv') and 'depthMat' in f]
        self.csv_files.sort()
        for file_id in range(len(self.csv_files)):
            csv_file = self.csv_files[file_id]
            chars_timestamp_extension = csv_file.split('depthMat')[1] 
            chars_timestamp = chars_timestamp_extension.split('.csv')[0]
            frame_filename = 'colorImg' + chars_timestamp+'.jpg'
            self.img_files.append(os.path.join(folder_path,frame_filename))
    def upscale_depth_mat(self,depthmat,new_width,new_height):
        depthmat *= 1000
        depth_image = (depthmat.copy()).astype(np.uint16)
        return cv2.resize(depth_image, (new_width,new_height), interpolation=cv2.INTER_LANCZOS4)
    def data_from_file_csv(self,idx):
        self.rgb_img = cv2.imread(self.img_files[idx], cv2.IMREAD_COLOR)
        with open(self.csv_files[idx], newline='') as csvfile:
            csv_reader = csv.reader(csvfile, delimiter=',')
            depthmat = np.array( [row for row in csv_reader], dtype=float)
        self.depth_img = self.upscale_depth_mat(depthmat,1920,1440)
    
    #
    def data_from_file(self,idx):
        #print("filename: ",self.imgdir_path+str(idx).zfill(6)+".png")
        #print(self.img_files[idx])
        img_tmp = cv2.imread(self.img_files[idx])
        if True:#img_tmp is not None:
            self.rgb_img = img_tmp
            #self.depth_img = cv2.imread(self.depthdir_path+str(idx).zfill(6)+".png",-1)


    # **
    # **************************************************************************************
    
    # ****************************************************************************************
    def data_from_stream(self, idx):
        try:
            # Send GET request to the URL
            response = requests.get(self.url, timeout=5)
            response.raise_for_status()  # Raise exception if status is not 200

            # Decode image content (RGB + depth as one image)
            arr = np.asarray(bytearray(response.content), dtype=np.uint8)
            image = cv2.imdecode(arr, -1)

            # Defensive: check if image decoded correctly
            if image is None or image.ndim < 3:
                self.get_logger().error("Failed to decode image from response.")
                raise ValueError("Invalid image data received")

            # Split RGB and depth (assuming RGBA-style layout)
            if image.shape[2] == 4:
                b, g, r, d = cv2.split(image)
            elif image.shape[2] == 3:
                b, g, r = cv2.split(image)
                d = np.zeros_like(b, dtype=np.uint8)
            else:
                self.get_logger().warn(f"Unexpected image shape: {image.shape}")
                b, g, r = image[:, :, 0], image[:, :, 1], image[:, :, 2]
                d = np.zeros_like(b, dtype=np.uint8)

            self.rgb_img = cv2.merge((b, g, r))

            # Try to get depth map from custom header
            depth_data_base64 = response.headers.get("X-Depth-Message-Pack")
            if depth_data_base64:
                try:
                    # Decode base64 and unpack msgpack using msgpack_numpy
                    depth_data_bytes = base64.b64decode(depth_data_base64)
                    depth_data = msgpack.unpackb(depth_data_bytes, object_hook=m.decode, raw=False)

                    # Convert to numpy and upscale
                    depth_arr = np.asarray(depth_data)
                    self.depth_img = self.upscale_depth_mat(depth_arr, 1920, 1440)
                except Exception as decode_err:
                    self.get_logger().error(f"Failed to decode depth data: {decode_err}")
                    self.depth_img = np.zeros((1440, 1920), dtype=np.uint16)
            else:
                self.get_logger().warn("Depth header missing, using zeros.")
                self.depth_img = np.zeros((1440, 1920), dtype=np.uint16)

        except requests.exceptions.RequestException as req_err:
            self.get_logger().error(f"HTTP request failed: {req_err}")
            self.rgb_img = np.zeros((1440, 1920, 3), dtype=np.uint8)
            self.depth_img = np.zeros((1440, 1920), dtype=np.uint16)
        except Exception as e:
            self.get_logger().error(f"Unexpected error during stream processing: {e}")
            self.rgb_img = np.zeros((1440, 1920, 3), dtype=np.uint8)
            self.depth_img = np.zeros((1440, 1920), dtype=np.uint16)

    # ****************************************************************************************
    def run_py_node(self, idx):
        """
            Master function that sends the RGB image message to the CPP node
        """
        
        img_msg = None # sensor_msgs image object
        timestep = float(idx)
        self.frame_id = self.frame_id + 1
        
        #self.data_from_file(idx)
        self.data_from_file_csv(idx)
        #self.data_from_stream(idx)
        
        img_msg = self.br.cv2_to_imgmsg(self.rgb_img, encoding="bgr8")
        depth_msg = self.br.cv2_to_imgmsg(self.depth_img, encoding="passthrough")
        timestep_msg = Float64()
        timestep_msg.data = timestep
        try:
            print("Publishing timestep: ",timestep)
            self.publish_timestep_msg_.publish(timestep_msg) 
            self.publish_img_msg_.publish(img_msg)
            self.publish_depth_msg_.publish(depth_msg)
        except CvBridgeError as e:
            print(e)
    # ****************************************************************************************
    def publish_data(self):
        pass
    def publish_to_gsfusion(self):
        # Create a blank black image (480x640)
        timestamp = self.get_clock().now().to_msg()
        if self.rgb_img is not None:
            ros_rgb = self.bridge.cv2_to_imgmsg(cv2.resize(self.rgb_img,(1920,1440)), encoding="bgr8")#Gergo: used to be bgr8
            ros_rgb.header.stamp = timestamp
            ros_rgb.header.frame_id = "camera_frame"
            self.rgb_publisher_.publish(ros_rgb)
        if self.depth_img is not None:
            ros_depth = self.bridge.cv2_to_imgmsg(cv2.resize(self.depth_img,(1920,1440)), encoding="16UC1")
            ros_depth.header.stamp = timestamp
            ros_depth.header.frame_id = "camera_frame"
            self.depth_publisher_.publish(ros_depth)

        pose_msg = PoseStamped()
        pose_msg.header.stamp = timestamp
        pose_msg.header.frame_id = "odom"
        odom_matrix = self.pose
        print(odom_matrix[0, 3])
        print(odom_matrix[1, 3])
        print(odom_matrix[2, 3])
        print("=============")
        # Extract translation
        pose_msg.pose.position.x = odom_matrix[0, 3]
        pose_msg.pose.position.y = odom_matrix[1, 3]
        pose_msg.pose.position.z = odom_matrix[2, 3]

        # Convert rotation matrix to quaternion
        quat = self.rotation_matrix_to_quaternion(odom_matrix[:3, :3])
        pose_msg.pose.orientation.x = quat[0]
        pose_msg.pose.orientation.y = quat[1]
        pose_msg.pose.orientation.z = quat[2]
        pose_msg.pose.orientation.w = quat[3]
        self.odometry_publisher_.publish(pose_msg)

        self.get_logger().info(f"{self.frame_index} Published rgb, depth and dummy pose.")

        self.frame_index += 1  # Move to next frame

    def load_odometry_data(self, odom_file):
        """Load odometry data from a file assuming each row is a 4x4 matrix"""
        if not os.path.exists(odom_file):
            self.get_logger().error(f"Odometry file not found: {odom_file}")
            return []
        
        data = []
        with open(odom_file, 'r') as f:
            lines = f.readlines()
            for line in lines:
                values = list(map(float, line.split()))
                if len(values) == 16:
                    matrix = np.array(values).reshape(4, 4)
                    data.append(matrix)
        return data

    def rotation_matrix_to_quaternion(self, R):
        """Convert a 3x3 rotation matrix to a quaternion"""
        q = np.zeros(4)
        trace = np.trace(R)
        if trace > 0:
            S = np.sqrt(trace + 1.0) * 2
            q[3] = 0.25 * S
            q[0] = (R[2, 1] - R[1, 2]) / S
            q[1] = (R[0, 2] - R[2, 0]) / S
            q[2] = (R[1, 0] - R[0, 1]) / S
        else:
            if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
                S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
                q[3] = (R[2, 1] - R[1, 2]) / S
                q[0] = 0.25 * S
                q[1] = (R[0, 1] + R[1, 0]) / S
                q[2] = (R[0, 2] + R[2, 0]) / S
            elif R[1, 1] > R[2, 2]:
                S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
                q[3] = (R[0, 2] - R[2, 0]) / S
                q[0] = (R[0, 1] + R[1, 0]) / S
                q[1] = 0.25 * S
                q[2] = (R[1, 2] + R[2, 1]) / S
            else:
                S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
                q[3] = (R[1, 0] - R[0, 1]) / S
                q[0] = (R[0, 2] + R[2, 0]) / S
                q[1] = (R[1, 2] + R[2, 1]) / S
                q[2] = 0.25 * S
        return q

    def euler_to_quaternion(self, roll, pitch, yaw):
        """Convert Euler angles to quaternion"""
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        return [qx, qy, qz, qw]
# ****************************************************************************************
def main(args = None):
    rclpy.init(args=args) # Initialize node
    
    #dataset_path = "/media/buvar/HBVCSSD/ES-SplatFusion/data/ikea_c/"#"/media/buvar/dtdata/sam3d_prep/ESAM/data/LB_DcGdi8/"  # Change this to your dataset directory

    node = MonoDriver("mono_py_node")
    
    #node.list_csv_img_pairs('/media/buvar/dtdata/3ddt_data/2025_03_14_lab241_DcGdi8_frames/')
    #node.list_csv_img_pairs('/home/anna/Desktop/ESFusion/LB_DcGdi8/')
    node.list_csv_img_pairs('/media/gables/Data/Data/Drone_Mamba/iphone/nas/2025_05_27_taskname_amx8L6_frames/')
    
    
    #print(node.csv_files)
    #print(node.img_files)
    # First handshake
    while(node.send_config == True):
        node.handshake_with_cpp_node()
        rclpy.spin_once(node)
        if(node.send_config == False):
            break
        
    print(f"Handshake complete")

    # Second handshake

    print("Handshake complete...")

    '''num_img_files = len(os.listdir(node.imgdir_path))
    num_depth_files = len(os.listdir(node.depthdir_path))
    assert num_img_files == num_depth_files, "The number of img and depth files are not equal!"
    node.frame_stop = min(node.frame_stop, num_img_files)-1'''
    idx = 0
    while True:
        try:
            rclpy.spin_once(node,timeout_sec=0.1)
            #print("alma")
            if True:
                time.sleep(0.1)
                node.run_py_node(idx)
                if np.array_equal(node.pose,node.empty_pose) is not True:
                    if idx >= node.localization_buffer:
                        end_flag = (idx>=node.frame_stop and node.frame_stop != -1)
                        #node.publish_to_gsfusion()
                if (idx>=node.frame_stop and node.frame_stop != -1):
                    print(f"BREAK!")
                    msg = String()
                    msg.data = "STOP"
                    node.publish_termination_msg_.publish(msg)
                    break
                idx += 1
        except KeyboardInterrupt:
            break
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
