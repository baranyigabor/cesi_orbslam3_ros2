/*

A bare-bones example node demonstrating the use of the Monocular mode in ORB-SLAM3
With added test capabilities 
Author: Azmyin Md. Kamal
Date: 01/01/24

REQUIREMENTS
* Make sure to set path to your workspace in common.hpp file

*/

//* Includes
#include "ros2_orb_slam3/common.hpp"

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
    
    // Add test mode parameters
    this->declare_parameter("test_mode", false);
    this->declare_parameter("dataset_path", "");
    this->declare_parameter("results_path", "slam_comparison_results.txt");
    this->declare_parameter("start_frame", 0);
    this->declare_parameter("end_frame", 100);
    this->declare_parameter("num_runs", 1);
    this->declare_parameter("ground_truth_path", "");
    
    // Timing stats parameters
    this->declare_parameter("timing_stats_file", "timing_stats.csv");
    this->shouldTerminate = false;
    // Get test parameters
    rclcpp::Parameter testModeParam = this->get_parameter("test_mode");
    testMode = testModeParam.as_bool();
    
    if (testMode) {
        rclcpp::Parameter datasetPathParam = this->get_parameter("dataset_path");
        rclcpp::Parameter resultsPathParam = this->get_parameter("results_path");
        rclcpp::Parameter startFrameParam = this->get_parameter("start_frame");
        rclcpp::Parameter endFrameParam = this->get_parameter("end_frame");
        rclcpp::Parameter numRunsParam = this->get_parameter("num_runs");
        rclcpp::Parameter groundTruthPathParam = this->get_parameter("ground_truth_path");
        
        datasetPath = datasetPathParam.as_string();
        resultsPath = resultsPathParam.as_string();
        startFrame = startFrameParam.as_int();
        endFrame = endFrameParam.as_int();
        numRuns = numRunsParam.as_int();
        groundTruthPath = groundTruthPathParam.as_string();
        hasGroundTruth = !groundTruthPath.empty();
        
        RCLCPP_INFO(this->get_logger(), "Running in test mode with dataset: %s", datasetPath.c_str());
        RCLCPP_INFO(this->get_logger(), "Frames %d to %d, %d runs", startFrame, endFrame, numRuns);
    }
    
    // Initialize timing stats
    mStatsRGBD.frameCount = 0;
    mStatsRGBD.successfulFrames = 0;
    mStatsRGBD.totalTime = 0.0;
    
    mStatsMonocular.frameCount = 0;
    mStatsMonocular.successfulFrames = 0;
    mStatsMonocular.totalTime = 0.0;
    
    rclcpp::Parameter timingStatsParam = this->get_parameter("timing_stats_file");
    mStatsFileName = timingStatsParam.as_string();
    mSaveTiming = true;
    mStartTime = std::chrono::high_resolution_clock::now();
    
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
        settingsFilePath = "/" + packagePath + "orb_slam3/config/Monocular/";
        
        std::cout<<"settingsFilePath: "<<settingsFilePath<<std::endl;
    }

    // std::cout<<"vocFilePath: "<<vocFilePath<<std::endl;
    // std::cout<<"settingsFilePath: "<<settingsFilePath<<std::endl;
    
    
    //* DEBUG print
    RCLCPP_INFO(this->get_logger(), "nodeName %s", nodeName.c_str());
    RCLCPP_INFO(this->get_logger(), "voc_file %s", vocFilePath.c_str());
    // RCLCPP_INFO(this->get_logger(), "settings_file_path %s", settingsFilePath.c_str());
    
    if (testMode) {
        // If in test mode, run tests immediately
        runTests();
    } else {
        // Normal node operation - set up ROS topics
        subexperimentconfigName = "/mono_py_driver/experiment_settings"; // topic that sends out some configuration parameters to the cpp ndoe
        pubconfigackName = "/mono_py_driver/exp_settings_ack"; // send an acknowledgement to the python node
        subImgMsgName = "/mono_py_driver/img_msg"; // topic to receive RGB image messages
        subTimestepMsgName = "/mono_py_driver/timestep_msg"; // topic to receive RGB image messages
        poseMsgName = "/mono_py_driver/pose_msg";
        syncMsgName = "/mono_py_driver/sync_msg";
        depthMsgName = "/mono_py_driver/depth_msg";
        subTerminationMsgName = "/mono_py_driver/termination_msg";
        pose_publisher_ = this->create_publisher<std_msgs::msg::String>(poseMsgName,10);
        sync_publisher_ = this->create_publisher<std_msgs::msg::String>(syncMsgName,10);
        //* subscribe to python node to receive settings
        expConfig_subscription_ = this->create_subscription<std_msgs::msg::String>(subexperimentconfigName, 1, std::bind(&MonocularMode::experimentSetting_callback, this, _1));

        //* publisher to send out acknowledgement
        configAck_publisher_ = this->create_publisher<std_msgs::msg::String>(pubconfigackName, 10);
        subTerminationMsg_subscription_ = this->create_subscription<std_msgs::msg::String>(subTerminationMsgName, 1, std::bind(&MonocularMode::termination_callback, this, _1));
        //* subscrbite to the image messages coming from the Python driver node
        subImgMsg_subscription_= this->create_subscription<sensor_msgs::msg::Image>(subImgMsgName, 1, std::bind(&MonocularMode::Img_callback, this, _1));
        subDepthMsg_subscription_= this->create_subscription<sensor_msgs::msg::Image>(depthMsgName, 1, std::bind(&MonocularMode::Depth_callback, this, _1));

        //* subscribe to receive the timestep
        subTimestepMsg_subscription_= this->create_subscription<std_msgs::msg::Float64>(subTimestepMsgName, 1, std::bind(&MonocularMode::Timestep_callback, this, _1));
        
        RCLCPP_INFO(this->get_logger(), "Hello ......");
        
        RCLCPP_INFO(this->get_logger(), "Waiting to finish handshake ......");
    }
}

//* Destructor
MonocularMode::~MonocularMode()
{   
    // if (mSaveTiming && !testMode) {
    //     saveTimingStats();
    // }
    
    // Stop all threads
    if (pAgent) {
        pAgent->Shutdown();
    }
    pass;
}

void MonocularMode::saveTimingStats() {
    if (!mSaveTiming) return;
    
    // elapsed time
    auto endTime = std::chrono::high_resolution_clock::now();
    double elapsedTime = std::chrono::duration<double>(endTime - mStartTime).count();
    
    // average
    double avgFrameTimeRGBD = mStatsRGBD.frameCount > 0 ? 
        mStatsRGBD.totalTime / mStatsRGBD.frameCount : 0.0;
    
    // success rate
    double successRateRGBD = mStatsRGBD.frameCount > 0 ? 
        (double)mStatsRGBD.successfulFrames / mStatsRGBD.frameCount * 100.0 : 0.0;
    
    // detailed CSV file with all frame timing data
    std::ofstream csvFile(mStatsFileName);
    
    //  header
    csvFile << "frame,processing_time_ms,tracking_success" << std::endl;
    
    // Write RGBD frame data
    for (size_t i = 0; i < mStatsRGBD.frameTimes.size(); i++) {
        bool success = (i < mStatsRGBD.trackingSuccesses.size()) ? 
                      mStatsRGBD.trackingSuccesses[i] : false;
        csvFile << i << "," 
                << mStatsRGBD.frameTimes[i] * 1000.0 << "," 
                << (success ? "1" : "0") << std::endl;
    }
    
    csvFile.close();
    
    // summary file with aggregated statistics
    std::string summaryFileName = mStatsFileName + ".summary.txt";
    std::ofstream summaryFile(summaryFileName);
    
    summaryFile << "ORB-SLAM Performance - RGB-D" << std::endl;
    summaryFile << "===============================================" << std::endl << std::endl;
    
    summaryFile << "Total test time: " << elapsedTime << " seconds" << std::endl;
    summaryFile << "Frames processed: " << mStatsRGBD.frameCount << std::endl;
    summaryFile << std::endl;
    
    summaryFile << "  Average frame processing time: " << avgFrameTimeRGBD * 1000 << " ms" << std::endl;
    summaryFile << "  Tracking success rate: " << successRateRGBD << "%" << std::endl;
    
    summaryFile.close();
    
    RCLCPP_INFO(this->get_logger(), "Timing statistics saved to %s and %s", 
                mStatsFileName.c_str(), summaryFileName.c_str());
}

// Test methods fro comparing
void MonocularMode::runTests() {
    RCLCPP_INFO(this->get_logger(), "Starting comparison tests between RGB and RGB-D mode");
    
    // Reset metrics
    mStatsMonocular = TimingStats();
    mStatsRGBD = TimingStats();
    
    // Run tests for both modes
    runMonocularTest();
    runRGBDTest();
    
    // Save and report results
    saveTestResults();
    
    // Signal completion and exit
    RCLCPP_INFO(this->get_logger(), "Tests completed. Results saved to %s", resultsPath.c_str());
    rclcpp::shutdown();
}

void MonocularMode::runMonocularTest() {
    RCLCPP_INFO(this->get_logger(), "Running Monocular test");
    bool useViewer = false;
    std::vector<Sophus::SE3f> allPoses;

    for (int run = 0; run < numRuns; run++) {
        RCLCPP_INFO(this->get_logger(), "Monocular Run %d/%d", run + 1, numRuns);
        
        ORB_SLAM3::System SLAM(vocFilePath, settingsFilePath, ORB_SLAM3::System::MONOCULAR, useViewer);
        
        auto startInit = std::chrono::high_resolution_clock::now();
        
        // Process all frames
        int currentFrame = startFrame;
        int failures = 0;
        
        while (true) {
            std::string framePath;
            if (std::filesystem::exists(datasetPath + "/color/")) {
                framePath = datasetPath + "/color/" + 
                          std::string(6 - std::to_string(currentFrame).length(), '0') + 
                          std::to_string(currentFrame) + ".png";
            } else {
                RCLCPP_ERROR(this->get_logger(), "No RGB images found");
                break;
            }
            
            cv::Mat im = cv::imread(framePath, cv::IMREAD_COLOR);
            
            if (im.empty()) {
                RCLCPP_INFO(this->get_logger(), "End of sequence reached at frame %d", currentFrame);
                break;
            }
            
            // measure time
            auto startFrame = std::chrono::high_resolution_clock::now();
            
            ORB_SLAM3::TrackingOutput trackOut = SLAM.TrackMonocular(im, (double)currentFrame);
            
            auto endFrameTime  = std::chrono::high_resolution_clock::now();
            double frameTime = std::chrono::duration<double>(endFrameTime - startFrame).count();
            
            mStatsMonocular.frameTimes.push_back(frameTime);
            mStatsMonocular.totalTime += frameTime;
            
            // quality
            if (!trackOut.isTrackingSuccessful) {
                failures++;
            } else {
                mStatsMonocular.successfulFrames++;
                if (run == numRuns - 1) {
                    allPoses.push_back(trackOut.Tcw);
                }
            }
            
            mStatsMonocular.trackingSuccesses.push_back(trackOut.isTrackingSuccessful);
            mStatsMonocular.numTrackedPoints.push_back(trackOut.mvpMapPoints.size());
            
            currentFrame++;
            if (endFrame > 0 && currentFrame >= endFrame) {
                RCLCPP_INFO(this->get_logger(), "Reached end frame limit %d", endFrame);
                break;
            }
        }
        
        auto endInit = std::chrono::high_resolution_clock::now();
        mStatsMonocular.initializationTime += std::chrono::duration<double>(endInit - startInit).count();
        mStatsMonocular.trackingFailures += failures;
        mStatsMonocular.frameCount += (currentFrame - startFrame);
        
        // Clean shutdown
        SLAM.Shutdown();
    }
    
    // Average metrics over runs
    if (numRuns > 0) {
        mStatsMonocular.initializationTime /= numRuns;
    }
    if (!allPoses.empty()) {
        saveTrajectory("rgb", allPoses);
    }
    
    RCLCPP_INFO(this->get_logger(), "Monocular test completed: %d frames processed", mStatsMonocular.frameCount);
}

void MonocularMode::runRGBDTest() {
    RCLCPP_INFO(this->get_logger(), "Running RGB-D test");
    std::vector<Sophus::SE3f> allPoses;
    bool useViewer = false;
    for (int run = 0; run < numRuns; run++) {
        RCLCPP_INFO(this->get_logger(), "RGB-D Run %d/%d", run + 1, numRuns);
        
        // Create SLAM system
        ORB_SLAM3::System SLAM(vocFilePath, settingsFilePath, ORB_SLAM3::System::RGBD, useViewer);
        
        auto startInit = std::chrono::high_resolution_clock::now();
        
        // Process all frames
        int currentFrame = startFrame;
        int failures = 0;
        
        while (true) {
            std::string rgbPath, depthPath;
            
            if (std::filesystem::exists(datasetPath + "/color/") && 
                std::filesystem::exists(datasetPath + "/depth/")) {
                rgbPath = datasetPath + "/color/" + 
                        std::string(6 - std::to_string(currentFrame).length(), '0') + 
                        std::to_string(currentFrame) + ".png";
                depthPath = datasetPath + "/depth/" + 
                          std::string(6 - std::to_string(currentFrame).length(), '0') + 
                          std::to_string(currentFrame) + ".png";
            } else {
                RCLCPP_ERROR(this->get_logger(), "Could not find RGB-D images directories");
                break;
            }
            
            cv::Mat im = cv::imread(rgbPath, cv::IMREAD_COLOR);
            cv::Mat depthIm = cv::imread(depthPath, cv::IMREAD_UNCHANGED);
            
            if (im.empty() || depthIm.empty()) {
                RCLCPP_INFO(this->get_logger(), "End of sequence reached at frame %d", currentFrame);
                break;
            }
            
            // Process frame and measure time
            auto startFrame = std::chrono::high_resolution_clock::now();
            
            Sophus::SE3f pose = SLAM.TrackRGBD(im, depthIm, (double)currentFrame);
            bool isTrackingSuccessful = !pose.matrix().isZero(0);
            
            auto endFrameTime  = std::chrono::high_resolution_clock::now();
            double frameTime = std::chrono::duration<double>(endFrameTime  - startFrame).count();
            
            mStatsRGBD.frameTimes.push_back(frameTime);
            mStatsRGBD.totalTime += frameTime;
            
            // quality
            if (!isTrackingSuccessful) {
                failures++;
            } else {
                mStatsRGBD.successfulFrames++;
                if (run == numRuns - 1) {
                    allPoses.push_back(pose);
                }
            }
            
            mStatsRGBD.trackingSuccesses.push_back(isTrackingSuccessful);
            
            // number of tracked points 
            int numPoints = SLAM.GetTrackedMapPoints().size();
            mStatsRGBD.numTrackedPoints.push_back(numPoints);
            
            currentFrame++;
            if (endFrame > 0 && currentFrame >= endFrame) {
                RCLCPP_INFO(this->get_logger(), "Reached end frame limit %d", endFrame);
                break;
            }
        }
        
        auto endInit = std::chrono::high_resolution_clock::now();
        mStatsRGBD.initializationTime += std::chrono::duration<double>(endInit - startInit).count();
        mStatsRGBD.trackingFailures += failures;
        mStatsRGBD.frameCount += (currentFrame - startFrame);
        if (!allPoses.empty()) {
            saveTrajectory("rgbd", allPoses);
        }
        // Clean shutdown
        SLAM.Shutdown();
    }
    
    // Average metrics over runs
    if (numRuns > 0) {
        mStatsRGBD.initializationTime /= numRuns;
    }
    
    RCLCPP_INFO(this->get_logger(), "RGB-D test completed: %d frames processed", mStatsRGBD.frameCount);
}

// Calculate trajectory error if ground truth is available
double MonocularMode::calculateTrajectoryError(const std::vector<Sophus::SE3f>& trajectory, 
                              const std::string& gtFile) {
    // Implementation depends on ground truth format COLMAP?
    // This is a placeholder
    return 0.0;
}

void MonocularMode::saveTestResults() {
    std::ofstream outFile(resultsPath);
    
    outFile << "SLAM RGB vs RGB-D Performance Comparison" << std::endl;
    outFile << "=========================================" << std::endl << std::endl;
    
    outFile << "Dataset: " << datasetPath << std::endl;
    outFile << "Number of runs: " << numRuns << std::endl;
    outFile << "Frame range: " << startFrame << " to " << endFrame << std::endl << std::endl;
    
    outFile << "RGB Metrics:" << std::endl;
    outFile << "  Frames processed: " << mStatsMonocular.frameCount << std::endl;
    outFile << "  Average frame processing time: " << mStatsMonocular.avgFrameProcessingTime() * 1000.0 << " ms" << std::endl;
    outFile << "  Frames per second: " << 1.0 / mStatsMonocular.avgFrameProcessingTime() << std::endl;
    outFile << "  Initialization time: " << mStatsMonocular.initializationTime * 1000.0 << " ms" << std::endl;
    outFile << "  Average tracked points: " << mStatsMonocular.avgTrackedPoints() << std::endl;
    outFile << "  Tracking failures: " << mStatsMonocular.trackingFailures << std::endl;
    outFile << "  Tracking success rate: " << 
        (mStatsMonocular.frameCount > 0 ? 
         (double)mStatsMonocular.successfulFrames / mStatsMonocular.frameCount * 100.0 : 0.0) << "%" << std::endl;
    
    outFile << std::endl << "RGB-D Metrics:" << std::endl;
    outFile << "  Frames processed: " << mStatsRGBD.frameCount << std::endl;
    outFile << "  Average frame processing time: " << mStatsRGBD.avgFrameProcessingTime() * 1000.0 << " ms" << std::endl;
    outFile << "  Frames per second: " << 1.0 / mStatsRGBD.avgFrameProcessingTime() << std::endl;
    outFile << "  Initialization time: " << mStatsRGBD.initializationTime * 1000.0 << " ms" << std::endl;
    outFile << "  Average tracked points: " << mStatsRGBD.avgTrackedPoints() << std::endl;
    outFile << "  Tracking failures: " << mStatsRGBD.trackingFailures << std::endl;
    outFile << "  Tracking success rate: " << 
        (mStatsRGBD.frameCount > 0 ? 
         (double)mStatsRGBD.successfulFrames / mStatsRGBD.frameCount * 100.0 : 0.0) << "%" << std::endl;
    
    outFile << std::endl << "Performance Comparison:" << std::endl;
    if (mStatsMonocular.avgFrameProcessingTime() > 0 && mStatsRGBD.avgFrameProcessingTime() > 0) {
        double timeSpeedup = mStatsMonocular.avgFrameProcessingTime() / mStatsRGBD.avgFrameProcessingTime();
        outFile << "  RGB-D is " << (timeSpeedup > 1.0 ? "faster" : "slower") << " by a factor of " 
                << std::abs(timeSpeedup) << std::endl;
        
        if (mStatsMonocular.avgTrackedPoints() > 0) {
            double pointsImprovement = (mStatsRGBD.avgTrackedPoints() - mStatsMonocular.avgTrackedPoints()) / 
                                      mStatsMonocular.avgTrackedPoints() * 100.0;
            outFile << "  RGB-D tracks " << std::abs(pointsImprovement) << "% " 
                    << (pointsImprovement > 0 ? "more" : "fewer") << " points" << std::endl;
        }
        
        double successRateMonocular = mStatsMonocular.frameCount > 0 ? 
            (double)mStatsMonocular.successfulFrames / mStatsMonocular.frameCount * 100.0 : 0.0;
        double successRateRGBD = mStatsRGBD.frameCount > 0 ? 
            (double)mStatsRGBD.successfulFrames / mStatsRGBD.frameCount * 100.0 : 0.0;
            
        double successDiff = successRateRGBD - successRateMonocular;
        outFile << "  RGB-D tracking success rate is " 
                << std::abs(successDiff) << "% "
                << (successDiff > 0 ? "higher" : "lower") << std::endl;
    } else {
        outFile << "  Insufficient data for comparison" << std::endl;
    }
    
    outFile.close();
    
    // detailed frame-by-frame data
    std::string detailedDataFile = resultsPath + ".detailed.csv";
    std::ofstream csvFile(detailedDataFile);
    
    csvFile << "frame,mode,processing_time_ms,tracking_success,tracked_points" << std::endl;
    
    // Write Monocular 
    for (size_t i = 0; i < mStatsMonocular.frameTimes.size(); i++) {
        bool success = (i < mStatsMonocular.trackingSuccesses.size()) ? 
                    mStatsMonocular.trackingSuccesses[i] : false;
        int points = (i < mStatsMonocular.numTrackedPoints.size()) ?
                    mStatsMonocular.numTrackedPoints[i] : 0;
                    
        csvFile << startFrame + i << ",monocular," 
                << mStatsMonocular.frameTimes[i] * 1000.0 << "," 
                << (success ? "1" : "0") << ","
                << points << std::endl;
    }
    
    // Write RGBD 
    for (size_t i = 0; i < mStatsRGBD.frameTimes.size(); i++) {
        bool success = (i < mStatsRGBD.trackingSuccesses.size()) ? 
                    mStatsRGBD.trackingSuccesses[i] : false;
        int points = (i < mStatsRGBD.numTrackedPoints.size()) ?
                    mStatsRGBD.numTrackedPoints[i] : 0;
                    
        csvFile << startFrame + i << ",rgbd," 
                << mStatsRGBD.frameTimes[i] * 1000.0 << "," 
                << (success ? "1" : "0") << ","
                << points << std::endl;
    }
    
    csvFile.close();
    
    RCLCPP_INFO(this->get_logger(), "Detailed results saved to %s", detailedDataFile.c_str());
}
void MonocularMode::termination_callback(const std_msgs::msg::String& msg)
{
    std::cout<<"Received termination signal: "<<msg.data.c_str()<<std::endl;
    shouldTerminate = true;
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
    sensorType = ORB_SLAM3::System::RGBD; 
    //sensorType = ORB_SLAM3::System::MONOCULAR;
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
void MonocularMode::Depth_callback(const sensor_msgs::msg::Image& msg)
{
    try
    {
        depth_img_ptr = cv_bridge::toCvCopy(msg); // Local scope
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(),"Error reading image");
        return;
    }
    std::cout << "Depth callback!" << std::endl;
    newDepthImg = true;
    if (newDepthImg && newRGBImg)
    {
        processImages();
    }
}
//* Callback to process image message and run SLAM node
void MonocularMode::Img_callback(const sensor_msgs::msg::Image& msg)
{
    try
    {
        rgb_img_ptr = cv_bridge::toCvCopy(msg); // Local scope
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(),"Error reading image");
        return;
    }
    std::cout << "Image callback!" << std::endl;
    newRGBImg = true;
    if (newDepthImg && newRGBImg)
    {
        processImages();
    }

}
void MonocularMode::processImages()
{
    auto startTime = std::chrono::high_resolution_clock::now();

    std::cout << "processed image" << std::endl;
    newRGBImg = false;
    newDepthImg = false;
    std::cout<< time_step << "##" << std::endl;
    time_step += 1;

    // Process with RGBD mode
    Sophus::SE3f Tcw = pAgent->TrackRGBD(rgb_img_ptr->image, depth_img_ptr->image, (double)time_step); 

    // ORB_SLAM3::TrackingOutput tcp_out = pAgent->TrackMonocular(rgb_img_ptr->image, (double)time_step); 
    // Sophus::SE3f Tcw = tcp_out.Tcw;
    
    bool isTrackingSuccessful = !Tcw.matrix().isZero(0);
    
    // Store timing information
    auto endTime = std::chrono::high_resolution_clock::now();
    double frameTime = std::chrono::duration<double>(endTime - startTime).count();
    
    mStatsRGBD.frameTimes.push_back(frameTime);
    mStatsRGBD.totalTime += frameTime;
    mStatsRGBD.frameCount++;
    mStatsRGBD.trackingSuccesses.push_back(isTrackingSuccessful);
    if (isTrackingSuccessful) {
        mStatsRGBD.successfulFrames++;
    }

    savePose(Tcw, timeStep, time_step);
    auto message = std_msgs::msg::String();
    message.data = "SYNC";
    sync_publisher_->publish(message);
}

void MonocularMode::savePose(const Sophus::SE3f& Tcw, double timestamp, int frameNumber) {

    std::stringstream pose_data;
    Sophus::SE3f Twc = Tcw.inverse();
    Eigen::Vector3f translation = Twc.translation();
    Eigen::Quaternionf quaternion(Twc.unit_quaternion());
    Eigen::Matrix3f rotMat = Twc.rotationMatrix();
    // First row

    pose_data << "#"<< frameNumber <<"#";

    pose_data << std::fixed << std::setprecision(6)
                    << rotMat(0,0) << " " << rotMat(0,1) << " " << rotMat(0,2) << " " << translation.x() << std::endl;
    
    // Second row
    pose_data  << std::fixed << std::setprecision(6)
                    << rotMat(1,0) << " " << rotMat(1,1) << " " << rotMat(1,2) << " " << translation.y() << std::endl;
    
    // Third row
    pose_data  << std::fixed << std::setprecision(6)
                    << rotMat(2,0) << " " << rotMat(2,1) << " " << rotMat(2,2) << " " << translation.z() << std::endl;
    
    // Fourth row
    pose_data  << "0.000000 0.000000 0.000000 1.000000" << std::endl;
    std::cout << "Pose: " << std::endl;
    std::cout << pose_data.str() << std::endl;
    auto message = std_msgs::msg::String();
    message.data = pose_data.str();
    pose_publisher_->publish(message);
}

void MonocularMode::saveTrajectory(const std::string& mode, const std::vector<Sophus::SE3f>& poses) {
    std::string trajectoryFile = resultsPath + "." + mode + "_trajectory.txt";
    std::ofstream outFile(trajectoryFile);
    
    if (!outFile.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open trajectory file: %s", trajectoryFile.c_str());
        return;
    }
    
    // Header with information about the format
    outFile << "# ORB-SLAM3 " << mode << " Trajectory in COLMAP-compatible format" << std::endl;
    outFile << "# image_id qw qx qy qz tx ty tz" << std::endl;
    
    for (size_t i = 0; i < poses.size(); ++i) {
        Sophus::SE3f Twc = poses[i].inverse();
        Eigen::Vector3f translation = Twc.translation();
        Eigen::Quaternionf quaternion = Twc.unit_quaternion();
        
        // Write to file in COLMAP format (image_id qw qx qy qz tx ty tz)
        outFile << std::fixed << std::setprecision(6)
                << i << " " 
                << quaternion.w() << " "  // COLMAP uses qw first
                << quaternion.x() << " " 
                << quaternion.y() << " " 
                << quaternion.z() << " "
                << translation.x() << " " 
                << translation.y() << " " 
                << translation.z() << std::endl;
    }
    
    // save in matrix format which is sometimes used
    std::string matrixFile = resultsPath + "." + mode + "_matrices.txt";
    std::ofstream matOutFile(matrixFile);
    
    if (matOutFile.is_open()) {
        matOutFile << "# ORB-SLAM3 " << mode << " Trajectory (4x4 transformation matrices)" << std::endl;
        matOutFile << "# frame_id R00 R01 R02 t0 R10 R11 R12 t1 R20 R21 R22 t2 0 0 0 1" << std::endl;
        
        for (size_t i = 0; i < poses.size(); ++i) {
            Sophus::SE3f Twc = poses[i].inverse();
            Eigen::Matrix3f R = Twc.rotationMatrix();
            Eigen::Vector3f t = Twc.translation();
            
            // Write matrix in format that matches COLMAP's transforms.txt
            matOutFile << std::fixed << std::setprecision(9)
                    << i << " " 
                    << R(0,0) << " " << R(0,1) << " " << R(0,2) << " " << t(0) << " "
                    << R(1,0) << " " << R(1,1) << " " << R(1,2) << " " << t(1) << " "
                    << R(2,0) << " " << R(2,1) << " " << R(2,2) << " " << t(2) << " "
                    << "0 0 0 1" << std::endl;
        }
        
        matOutFile.close();
        RCLCPP_INFO(this->get_logger(), "Also saved matrix format to %s", matrixFile.c_str());
    }
    
    outFile.close();
    RCLCPP_INFO(this->get_logger(), "Saved %s trajectory to %s", mode.c_str(), trajectoryFile.c_str());
}