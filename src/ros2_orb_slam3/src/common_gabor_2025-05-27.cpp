/*

A bare-bones example node demonstrating the use of the Monocular mode in ORB-SLAM3

Author: Azmyin Md. Kamal
Date: 01/01/24

REQUIREMENTS
* Make sure to set path to your workspace in common.hpp file

*/

//* Includes
#include "ros2_orb_slam3/common.hpp"
#include "Results.h"
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

// Define the Synchronization Policy
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> MySyncPolicy;

//* Constructor
MonocularMode::MonocularMode() :Node("mono_node_cpp")
{
    // Declare parameters to be passsed from command line
    // https://roboticsbackend.com/rclcpp-params-tutorial-get-set-ros2-params-with-cpp/
    
    //* Find path to home directory
    homeDir = getenv("HOME");
    // std::cout<<"Home: "<<homeDir<<std::endl;
    
    // std::cout<<"VLSAM NODE STARTED\n\n";
    RCLCPP_INFO(this->get_logger(), "\nORB-SLAM3-V1 NODE STARTED");

    this->declare_parameter("node_name_arg", "not_given"); // Name of this agent 
    this->declare_parameter("voc_file_arg", "file_not_set"); // Needs to be overriden with appropriate name  
    this->declare_parameter("settings_file_path_arg", "file_path_not_set"); // path to settings file  
    
    //* Watchdog, populate default values
    nodeName = "not_set";
    vocFilePath = "file_not_set";
    settingsFilePath = "file_not_set";

    //* Populate parameter values
    rclcpp::Parameter param1 = this->get_parameter("node_name_arg");
    nodeName = param1.as_string();
    
    rclcpp::Parameter param2 = this->get_parameter("voc_file_arg");
    vocFilePath = param2.as_string();

    rclcpp::Parameter param3 = this->get_parameter("settings_file_path_arg");
    settingsFilePath = param3.as_string();

    // rclcpp::Parameter param4 = this->get_parameter("settings_file_name_arg");
  
    //* HARDCODED, set paths
    if (vocFilePath == "file_not_set" || settingsFilePath == "file_not_set")
    {
        pass;
        vocFilePath = "/" + packagePath + "orb_slam3/Vocabulary/ORBvoc.txt.bin";
        settingsFilePath = "/" + packagePath + "/orb_slam3/config/Monocular/";
    }
    // std::cout<<"vocFilePath: "<<vocFilePath<<std::endl;
    // std::cout<<"settingsFilePath: "<<settingsFilePath<<std::endl;
    
    
    //* DEBUG print
    RCLCPP_INFO(this->get_logger(), "nodeName %s", nodeName.c_str());
    RCLCPP_INFO(this->get_logger(), "voc_file %s", vocFilePath.c_str());
    // RCLCPP_INFO(this->get_logger(), "settings_file_path %s", settingsFilePath.c_str());
    
    subexperimentconfigName = "/mono_py_driver/experiment_settings"; // topic that sends out some configuration parameters to the cpp ndoe
    pubconfigackName = "/mono_py_driver/exp_settings_ack"; // send an acknowledgement to the python node
    subImgMsgName = "/mono_py_driver/img_msg"; // topic to receive RGB image messages
    subBundleAdjustmentMsgName = "/mono_py_driver/bundle_adjustment_bool_msg"; // topic to receive RGB image messages
    subDepthImgMsgName = "/mono_py_driver/depth_img_msg"; // topic to receive Depth image messages
    subTimestepMsgName = "/mono_py_driver/timestep_msg"; // topic to receive RGB image messages
    pubodometryMsgName = "/orbslam3_ros2/odometry_data"; // ORB_SLAM3 based odometry data
    cameraInfoMsgName = "/camera/camera_info"; // Camera instrinsics
    cameraImageMsgName = "/camera/image_raw"; // Raw camera image --same image used for OrbSLAM3
    cameraDepthImageMsgName = "/camera/image_depth"; // Raw camera image --same image used for OrbSLAM3

    //* subscribe to python node to receive settings
    expConfig_subscription_ = this->create_subscription<std_msgs::msg::String>(subexperimentconfigName, 1, std::bind(&MonocularMode::experimentSetting_callback, this, _1));

    //* publisher to send out acknowledgement
    configAck_publisher_ = this->create_publisher<std_msgs::msg::String>(pubconfigackName, 10);

    odometry_data_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(pubodometryMsgName, 10);
    camera_info_publisher_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(cameraInfoMsgName, 10);
    camera_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(cameraImageMsgName, 10);
    camera_depth_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(cameraDepthImageMsgName, 10);

    //* subscrbite to the image messages coming from the Python driver node
    //subImgMsg_subscription_= this->create_subscription<sensor_msgs::msg::Image>(subImgMsgName, 1, std::bind(&MonocularMode::Img_callback, this, _1));
    //subDepthImgMsg_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(subDepthImgMsgName, 1, std::bind(&MonocularMode::Depth_Img_callback, this, _1));
    subBundleAdjustmentMsg_subscription_ = this->create_subscription<std_msgs::msg::Bool>(subBundleAdjustmentMsgName, 1, std::bind(&MonocularMode::TriggerRefinement, this, _1));

    
    //* subscribe to receive the timestep
    subTimestepMsg_subscription_= this->create_subscription<std_msgs::msg::Float64>(subTimestepMsgName, 1, std::bind(&MonocularMode::Timestep_callback, this, _1));


    this->declare_parameter<std::string>("rgb_topic", "/camera/color/image_raw");
    this->declare_parameter<std::string>("depth_topic", "/camera/depth/image_raw");

    std::string rgb_topic = this->get_parameter("rgb_topic").as_string();
    std::string depth_topic = this->get_parameter("depth_topic").as_string();

    // Setup ORB-SLAM3 system
    /*SLAM = std::make_unique<ORB_SLAM3::System>(
        homeDir + "/ros2_orb_slam3/Vocabulary/ORBvoc.txt",
        homeDir + "/ros2_orb_slam3/Config/RGB-D.yaml",
        ORB_SLAM3::System::RGBD, true
    );*/

    // Create subscribers using message_filters for automatic synchronization
    rgb_sub_.subscribe(this, rgb_topic, rmw_qos_profile_sensor_data);
    depth_sub_.subscribe(this, depth_topic, rmw_qos_profile_sensor_data);

    // Synchronize RGB and Depth messages
    sync_ = std::make_shared<message_filters::Synchronizer<MySyncPolicy>>(MySyncPolicy(10), rgb_sub_, depth_sub_);
    sync_->registerCallback(std::bind(&MonocularMode::RGBD_Img_callback, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Waiting to finish handshake ......");
}

//* Destructor
MonocularMode::~MonocularMode()
{   
    
    // Stop all threads
    // Call method to write the trajectory file
    // Release resources and cleanly shutdown
    pAgent->Shutdown();
    pass;

}

//* Callback which accepts experiment parameters from the Python node
void MonocularMode::experimentSetting_callback(const std_msgs::msg::String& msg){
    
    // std::cout<<"experimentSetting_callback"<<std::endl;
    bSettingsFromPython = true;
    experimentConfig = msg.data.c_str();
    // receivedConfig = experimentConfig; // Redundant
    
    RCLCPP_INFO(this->get_logger(), "Configuration YAML file name: %s", this->receivedConfig.c_str());

    //* Publish acknowledgement
    auto message = std_msgs::msg::String();
    message.data = "ACK";
    
    std::cout<<"Sent response: "<<message.data.c_str()<<std::endl;
    configAck_publisher_->publish(message);

    //* Wait to complete VSLAM initialization
    initializeVSLAM(experimentConfig);

}

//* Method to bind an initialized VSLAM framework to this node
void MonocularMode::initializeVSLAM(std::string& configString){
    
    // Watchdog, if the paths to vocabular and settings files are still not set
    if (vocFilePath == "file_not_set" || settingsFilePath == "file_not_set")
    {
        RCLCPP_ERROR(get_logger(), "Please provide valid voc_file and settings_file paths");       
        rclcpp::shutdown();
    } 
    
    //* Build .yaml`s file path
    
    settingsFilePath = settingsFilePath.append(configString);
    settingsFilePath = settingsFilePath.append(".yaml"); // Example ros2_ws/src/orb_slam3_ros2/orb_slam3/config/Monocular/TUM2.yaml

    RCLCPP_INFO(this->get_logger(), "Path to settings file: %s", settingsFilePath.c_str());
    
    // NOTE if you plan on passing other configuration parameters to ORB SLAM3 Systems class, do it here
    // NOTE you may also use a .yaml file here to set these values
    //sensorType = ORB_SLAM3::System::MONOCULAR; 
    sensorType = ORB_SLAM3::System::RGBD; 
    enablePangolinWindow = true; // Shows Pangolin window output
    enableOpenCVWindow = true; // Shows OpenCV window output
    
    pAgent = new ORB_SLAM3::System(vocFilePath, settingsFilePath, sensorType, enablePangolinWindow);

    std::cout << "MonocularMode node initialized" << std::endl; // TODO needs a better message
}

//* Callback that processes timestep sent over ROS
void MonocularMode::Timestep_callback(const std_msgs::msg::Float64& time_msg){
    // timeStep = 0; // Initialize
    timeStep = time_msg.data;
}

//* New RGB-D Image Callback
void MonocularMode::RGBD_Img_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr& rgb_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg)
{
    // Convert ROS images to OpenCV format
    cv::Mat rgb_image = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8)->image;
    cv::Mat depth_image = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1)->image;

    auto luxonis_rgb_image_ = std::make_shared<sensor_msgs::msg::Image>(*rgb_msg);
    auto luxonis_depth_image_ = std::make_shared<sensor_msgs::msg::Image>(*depth_msg);

    // Convert depth from millimeters to meters (ORB-SLAM3 expects meters)
    //depth_image.convertTo(depth_image, CV_32F, 1.0 / 1000.0);
    depth_image.convertTo(depth_image, CV_32F);

    // Get timestamp from the RGB message
    double timestamp = rgb_msg->header.stamp.sec + rgb_msg->header.stamp.nanosec * 1e-9;

    // Run ORB-SLAM3 in RGB-D mode
    //SLAM->TrackRGBD(rgb_image, depth_image, timestamp);
    //ORB_SLAM3::TrackingOutput tcp_out = pAgent->TrackRGBD(rgb_image, depth_image, timestamp); 
    //Sophus::SE3f Tcw = tcp_out.Tcw;

    if (depth_image.empty()) {
        std::cout << "⚠️ Warning: ORB-SLAM3 Not Using Depth!" << std::endl;
    }

    Sophus::SE3f camera_pose = pAgent->TrackRGBD(rgb_image, depth_image, timestamp);

    // Convert Sophus::SE3f to a usable transformation matrix
    Eigen::Matrix4f transformation_matrix = camera_pose.matrix();

    // Debug output (Optional: You can print the transformation matrix)
    RCLCPP_INFO(this->get_logger(), "Tracked Pose:\n[%f %f %f %f]\n[%f %f %f %f]\n[%f %f %f %f]\n[%f %f %f %f]",
                transformation_matrix(0, 0), transformation_matrix(0, 1), transformation_matrix(0, 2), transformation_matrix(0, 3),
                transformation_matrix(1, 0), transformation_matrix(1, 1), transformation_matrix(1, 2), transformation_matrix(1, 3),
                transformation_matrix(2, 0), transformation_matrix(2, 1), transformation_matrix(2, 2), transformation_matrix(2, 3),
                transformation_matrix(3, 0), transformation_matrix(3, 1), transformation_matrix(3, 2), transformation_matrix(3, 3));

    // Convert SE3f to Eigen 4x4
    Eigen::Matrix4f Tcw = camera_pose.matrix();

    // Invert to get T_world_camera
    Eigen::Matrix4f Twc = Tcw.inverse();

    // Apply coordinate system conversion: ORB → ROS (Z-up, X-forward)
    Eigen::Matrix4f T_fix;
    T_fix << 
        0,  0, 1, 0,
        -1,  0, 0, 0,
        0, -1, 0, 0,
        0,  0, 0, 1;

    Eigen::Matrix4f T_ros = Twc; //* T_fix is not needed. Author said: In GSFusion, the camera frame convention follows the x-axis to the right, the y-axis down, and the z-axis forward, which is consistent with OpenCV and Colmap.

    // Extract translation
    Eigen::Vector3f translation = T_ros.block<3, 1>(0, 3);

    // Extract rotation
    Eigen::Matrix3f R_ros = T_ros.block<3, 3>(0, 0);
    Eigen::Quaternionf quaternion(R_ros);

    /*Eigen::Vector3f translation = camera_pose.translation();

    // Extract the rotation component and convert to quaternion
    Eigen::Quaternionf quaternion(camera_pose.unit_quaternion());*/

    geometry_msgs::msg::PoseStamped odom_msg;
    auto current_timestamp = rclcpp::Node::now();
    odom_msg.header.stamp = current_timestamp;  // Set the current time
    odom_msg.header.frame_id = "world";  // Or whatever your fixed frame is

    // Set the child frame if necessary (e.g., "camera" or "base_link")
    //odom_msg.child_frame_id = "camera";

    // Populate the position data
    odom_msg.pose.position.x = translation.x();
    odom_msg.pose.position.y = translation.y();
    odom_msg.pose.position.z = translation.z();

    // Populate the orientation data
    odom_msg.pose.orientation.x = quaternion.x();
    odom_msg.pose.orientation.y = quaternion.y();
    odom_msg.pose.orientation.z = quaternion.z();
    odom_msg.pose.orientation.w = quaternion.w();

    odometry_data_publisher_->publish(odom_msg);

    luxonis_rgb_image_->header.stamp = current_timestamp;
    luxonis_depth_image_->header.stamp = current_timestamp;

    camera_image_publisher_->publish(*luxonis_rgb_image_); //parameter of ImgCallback function
    camera_depth_image_publisher_->publish(*luxonis_depth_image_); //parameter of ImgCallback function
}

void MonocularMode::Depth_Img_callback(const sensor_msgs::msg::Image& msg)
{   
    latest_depth_image_ = std::make_shared<sensor_msgs::msg::Image>(msg);

    cv_bridge::CvImagePtr cv_ptr; //* Does not create a copy, memory efficient

    //* Convert ROS image to openCV image
    try
    {
        //cv::Mat im =  cv_bridge::toCvShare(msg.img, msg)->image;
        cv_ptr = cv_bridge::toCvCopy(msg); // Local scope
        
        // DEBUGGING, Show image
        // Update GUI Window
        // cv::imshow("test_window", cv_ptr->image);
        // cv::waitKey(3);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(),"Error reading image");
        return;
    }

    latest_depth_image_->header.stamp = latest_image_->header.stamp;
    latest_depth_image_->header.frame_id = "camera_frame";

    camera_depth_image_publisher_->publish(*latest_depth_image_); //parameter of ImgCallback function
}

void MonocularMode::TriggerRefinement( const std_msgs::msg::Bool::SharedPtr msg) {
    if (pAgent) {
        // Retrieve components via public getters
        ORB_SLAM3::LoopClosing* pLoopCloser = pAgent->GetLoopCloser();
        ORB_SLAM3::Tracking* pTracker = pAgent->GetTracker();
        ORB_SLAM3::Atlas* pAtlas = pAgent->GetAtlas();

        if (pLoopCloser && pTracker && pAtlas) {
            // Retrieve the current map
            ORB_SLAM3::Map* pCurrentMap = pAtlas->GetCurrentMap();

            if (pCurrentMap) {
                // Get the last keyframe ID from the tracker
                ORB_SLAM3::KeyFrame* pLastKeyFrame = pTracker->GetLastKeyFrame();
                if (pLastKeyFrame) {
                    long unsigned int loopKeyFrameId = pLastKeyFrame->mnId;

                    RCLCPP_INFO(rclcpp::get_logger("refinement_node"), 
                                "Using KeyFrame ID: %lu for global bundle adjustment.", loopKeyFrameId);

                    // Run global bundle adjustment
                    pLoopCloser->RunGlobalBundleAdjustment(pCurrentMap, loopKeyFrameId);

                    RCLCPP_INFO(rclcpp::get_logger("refinement_node"), "Global bundle adjustment completed successfully.");
                } else {
                    RCLCPP_WARN(rclcpp::get_logger("refinement_node"), "No last keyframe found. Refinement skipped.");
                }
            } else {
                RCLCPP_WARN(rclcpp::get_logger("refinement_node"), "No active map found. Refinement skipped.");
            }
        } else {
            RCLCPP_WARN(rclcpp::get_logger("refinement_node"), "Tracker, LoopCloser, or Atlas not properly initialized. Refinement skipped.");
        }
    } else {
        RCLCPP_WARN(rclcpp::get_logger("refinement_node"), "SLAM system not properly initialized. Refinement skipped.");
    }
}

//* Callback to process image message and run SLAM node
void MonocularMode::Img_callback(const sensor_msgs::msg::Image& msg)
{
    latest_image_ = std::make_shared<sensor_msgs::msg::Image>(msg);
    // Initialize
    cv_bridge::CvImagePtr cv_ptr; //* Does not create a copy, memory efficient
    
    //* Convert ROS image to openCV image
    try
    {
        //cv::Mat im =  cv_bridge::toCvShare(msg.img, msg)->image;
        cv_ptr = cv_bridge::toCvCopy(msg); // Local scope
        
        // DEBUGGING, Show image
        // Update GUI Window
        // cv::imshow("test_window", cv_ptr->image);
        // cv::waitKey(3);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(),"Error reading image");
        return;
    }
    
    // std::cout<<std::fixed<<"Timestep: "<<timeStep<<std::endl; // Debug
    
    //* Perform all ORB-SLAM3 operations in Monocular mode
    //! Pose with respect to the camera coordinate frame not the world coordinate frame
    //Sophus::SE3f Tcw = pAgent->TrackMonocular(cv_ptr->image, timeStep); 
    ORB_SLAM3::TrackingOutput tcp_out = pAgent->TrackMonocular(cv_ptr->image, timeStep); 
    Sophus::SE3f Tcw = tcp_out.Tcw;
    //cout << "\rcurrent_frame_detection_state: " << tcp_out.isTrackingSuccessful << flush; 

    if(tcp_out.isTrackingSuccessful){

    
    //size_t numberOfMapPoints = tcp_out.mvpMapPoints.size();
    //cout << "tracking in progress... detected points: "  << numberOfMapPoints << endl; 

    //@TODO: MOVE TO A FUNCTION 
    // Extract the translation (position) component
    Eigen::Vector3f translation = Tcw.translation();

    // Extract the rotation component and convert to quaternion
    Eigen::Quaternionf quaternion(Tcw.unit_quaternion());

    geometry_msgs::msg::PoseStamped odom_msg;
    auto current_timestamp = rclcpp::Node::now();
    odom_msg.header.stamp = current_timestamp;  // Set the current time
    odom_msg.header.frame_id = "world";  // Or whatever your fixed frame is

    // Set the child frame if necessary (e.g., "camera" or "base_link")
    //odom_msg.child_frame_id = "camera";

    // Populate the position data
    odom_msg.pose.position.x = translation.x();
    odom_msg.pose.position.y = translation.y();
    odom_msg.pose.position.z = translation.z();

    // Populate the orientation data
    odom_msg.pose.orientation.x = quaternion.x();
    odom_msg.pose.orientation.y = quaternion.y();
    odom_msg.pose.orientation.z = quaternion.z();
    odom_msg.pose.orientation.w = quaternion.w();

    odometry_data_publisher_->publish(odom_msg);
    cout << "\rodometry data: ("    << translation.x() << ", " 
                                    << translation.y() << ", " 
                                    << translation.z() << ") quat: (" 
                                    <<  quaternion.x() << "," 
                                    <<  quaternion.y() << "," 
                                    <<  quaternion.z() << "," 
                                    <<  quaternion.w() <<  ")" << flush;
    //@TODO: MOVE TO A FUNCTION 
    //@TODO: read camera data from config
    sensor_msgs::msg::CameraInfo camera_info_;

    //auto img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();

    // Update the header information
    camera_info_.header.stamp = current_timestamp;
    camera_info_.header.frame_id = "camera_frame";
    camera_info_.width = 1920;  // Example width --
    camera_info_.height = 1080; // Example height

    // Set the camera intrinsic parameters (K matrix)
    // These values should be replaced with your actual camera parameters
    
    camera_info_.k = {1058.35,  0, 949.07,
                        0,  1057.4399, 531.6,
                        0,  0,  1};

    // Set the distortion model
    camera_info_.distortion_model = "plumb_bob";

    // Set the distortion parameters (D matrix)
    // These values should be replaced with your actual camera parameters
    // k1, k2, p1, p2, k3
    camera_info_.d = {-0.0428, 0.0117, -0.0002, -0.0005, -0.0053};

    camera_info_publisher_->publish(camera_info_);
    
    // IMAGE PUBLISH

    latest_image_->header.stamp = current_timestamp;
    latest_image_->header.frame_id = "camera_frame";

    camera_image_publisher_->publish(*latest_image_); //parameter of ImgCallback function
    
    //* An example of what can be done after the pose w.r.t camera coordinate frame is computed by ORB SLAM3
    //Sophus::SE3f Twc = Tcw.inverse(); //* Pose with respect to global image coordinate, reserved for future use
    }
}


