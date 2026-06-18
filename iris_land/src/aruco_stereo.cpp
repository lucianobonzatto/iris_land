#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <string>
#include <chrono>
#include <thread>
#include <map>
#include <cmath>
#include <iomanip>
#include <algorithm>
#include <numeric>
#include <sstream>
#include <limits>
#include <set>
#include <vector>

// Enhanced marker status with detailed failure points
enum class MarkerStatus
{
    OK = 0,
    NOT_MATCHED,
    FILTERED_OUT,
    JOINT_PNP_FAILED,
    JOINT_PNP_NO_CONVERGENCE,
    REPROJECTION_ERROR_LEFT_TOO_HIGH,
    REPROJECTION_ERROR_RIGHT_TOO_HIGH,
    REPROJECTION_ERROR_AVERAGE_TOO_HIGH,
    PIXEL_SPAN_TOO_SMALL,
    UNKNOWN
};

const char *statusToString(MarkerStatus status)
{
    switch (status)
    {
    case MarkerStatus::OK:
        return "OK";
    case MarkerStatus::NOT_MATCHED:
        return "Not matched between cameras";
    case MarkerStatus::FILTERED_OUT:
        return "Marker ID not in allowed list";
    case MarkerStatus::JOINT_PNP_FAILED:
        return "Joint stereo PnP failed to initialize";
    case MarkerStatus::JOINT_PNP_NO_CONVERGENCE:
        return "Joint stereo PnP did not converge";
    case MarkerStatus::REPROJECTION_ERROR_LEFT_TOO_HIGH:
        return "Left camera reprojection error too high";
    case MarkerStatus::REPROJECTION_ERROR_RIGHT_TOO_HIGH:
        return "Right camera reprojection error too high";
    case MarkerStatus::REPROJECTION_ERROR_AVERAGE_TOO_HIGH:
        return "Average reprojection error too high";
    case MarkerStatus::PIXEL_SPAN_TOO_SMALL:
        return "Marker too small in image (pixel span below threshold)";
    default:
        return "Unknown error";
    }
}

// Structure to hold stereo calibration parameters
struct StereoCalibration
{
    cv::Mat leftCameraMatrix;
    cv::Mat leftDistCoeffs;
    cv::Mat rightCameraMatrix;
    cv::Mat rightDistCoeffs;
    cv::Mat R, T, E, F;
    cv::Mat P1, P2;
    int imageWidth, imageHeight;
};

// Enhanced marker data with detailed metrics
struct MatchedMarker
{
    int id;
    std::vector<cv::Point2f> leftCorners;
    std::vector<cv::Point2f> rightCorners;
    cv::Mat rvec, tvec;

    // Raw single camera pose (for comparison with Python)
    cv::Mat rawLeftRvec, rawLeftTvec;
    cv::Mat rawRightRvec, rawRightTvec;

    // Detailed error metrics
    double reprojectionError;
    double leftReprojectionError;
    double rightReprojectionError;

    // Processing stage info
    bool jointPnPConverged = false;
    int jointPnPIterations = 0;
    double jointPnPInitialError = 0.0;
    double jointPnPFinalError = 0.0;

    bool geometryValidationPassed = false;
    double avgSideLength = 0.0;
    double avgDiagonalLength = 0.0;
    double diagonalRatio = 0.0;
    double pixelSpan = 0.0;
    double minPixelSpanRequired = 0.0;
    double maxReliableRange = 0.0;

    bool valid = false;
    MarkerStatus status = MarkerStatus::UNKNOWN;
    std::string detailedFailureReason;
};

struct Config
{
    // Marker parameters
    float markerSize = 0.08f;

    // -1 = all markers, otherwise filter to this ID
    int singleMarkerFilter = -1;

    // Error thresholds
    float reprojErrorThreshold = 4.0f; // best trade-off value that I've found for reliable detection without too much noise (Python reprojection error is usually around 3-5px for good detections, but can be higher for smaller markers or at longer ranges)
    float huberDelta = 1.5f;

    // Iteration control
    int jointPnPMaxIterations = 50;
    float convergenceThreshold = 1e-6f;

    // Detection parameters
    int cornerRefinementWinSize = 7;
    int cornerRefinementMaxIterations = 50;
    float cornerRefinementMinAccuracy = 0.005f;
    float minPixelSpan = 40.0f;  // Fallback minimum marker diagonal in pixels for reliable PnP

    // Debug levels - controlled by ROS params and optional command-line flags
    bool enableVisualization = true;
    bool enableDebug = false;       // Final results only
    bool enableDebugTrace = false;  // Everything detailed
    bool enablePerformance = false; // Nothing

    // Calibration file path
    std::string calibrationFile = "/home/jetson/catkin_ws/src/iris_land/iris_land/config/stereo_camera_params_1600_600.yml";
};

// MarkerTransform helper struct
struct MarkerTransform
{
    int id;
    cv::Vec3f position;
    cv::Mat rotation;
};

class StereoArucoDetectorNode
{
public:
    // Public config access
    Config config_;

    // Constructor
    StereoArucoDetectorNode(const Config& cfg = Config()) : nh_(),
                                pnh_("~"),
                                frameCounter_(0),
                                totalMarkers_(0),
                                validMarkers_(0),
                                config_(cfg)
    {
        ROS_INFO("Initializing Stereo ArUco Detector Node...");

        initializeParameters();
        initializeArucoSettings();
        initializeTopics();
        initializeStereoSettings();
        initializeTransformMatrices();

        ROS_INFO("Stereo ArUco Detector Node initialized successfully.");
    }

    // Public run method
    void run()
    {
        ROS_INFO("Stereo ArUco Detector Node is running...");
        ROS_INFO("Monitoring marker IDs: 363 (15cm), 682 (8cm), 417 (24.5cm)");
        ros::spin();
    }

private:
    // ROS members
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    // Subscribers and publishers
    message_filters::Subscriber<sensor_msgs::Image> left_image_sub_;
    message_filters::Subscriber<sensor_msgs::Image> right_image_sub_;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;
    typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
    std::shared_ptr<Synchronizer> sync_;

    ros::Publisher pose_pub_;
    ros::Publisher debug_image_pub_;
    ros::Publisher debug_left_pub_;
    ros::Publisher debug_right_pub_;
    ros::Publisher marker_quality_pub_;

    // Per-marker individual publishers
    std::map<int, ros::Publisher> per_marker_pose_pubs_;

    // Configuration and calibration
    StereoCalibration stereoCalib_;

    // ArUco detection
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Ptr<cv::aruco::DetectorParameters> parameters_;

    // Marker configuration - ID filtering and sizes
    std::set<int> allowedMarkerIds_;
    std::map<int, float> markerSizes_;
    std::map<int, float> markerMaxReliableRanges_;

    // Transform matrices for different marker IDs
    std::map<int, cv::Mat> TM_Landpad_To_Aruco_;

    // Undistortion maps (for efficiency)
    cv::Mat mapx1_, mapy1_, mapx2_, mapy2_;

    // Statistics
    int frameCounter_;
    int totalMarkers_;
    int validMarkers_;
    std::map<MarkerStatus, int> statusCounts_;

    void debugIndividualMarkerTransform(int markerId)
    {
        ROS_INFO("\n=== DEBUGGING MARKER %d TRANSFORM ===", markerId);

        // Test with a known simple pose
        cv::Mat testRvec = (cv::Mat_<double>(3, 1) << 0, 0, 0); // No rotation
        cv::Mat testTvec = (cv::Mat_<double>(3, 1) << 0, 0, 1); // 1 meter forward

        MatchedMarker testMarker;
        testMarker.id = markerId;
        testMarker.rvec = testRvec;
        testMarker.tvec = testTvec;

        cv::Vec3f pos, ori;
        if (applyMarkerTransform(testMarker, pos, ori))
        {
            ROS_INFO("Test pose [0,0,1] with no rotation -> Landpad: [%.3f, %.3f, %.3f]",
                     pos[0], pos[1], pos[2]);
        }

        // Check if transform matrix exists
        if (TM_Landpad_To_Aruco_.count(markerId))
        {
            ROS_INFO("Transform matrix for marker %d:", markerId);
            cv::Mat transform = TM_Landpad_To_Aruco_[markerId];
            for (int i = 0; i < 4; i++)
            {
                ROS_INFO("  [%8.3f, %8.3f, %8.3f, %8.3f]",
                         transform.at<double>(i, 0), transform.at<double>(i, 1),
                         transform.at<double>(i, 2), transform.at<double>(i, 3));
            }
        }

        ROS_INFO("=====================================\n");
    }

    void debugStereoCalibration()
    {
        ROS_INFO("\n=== STEREO CALIBRATION VALIDATION ===");

        // Check if baseline makes sense
        double baseline = cv::norm(stereoCalib_.T);
        ROS_INFO("Baseline: %.3f meters", baseline);
        if (baseline > 0.5)
        {
            ROS_WARN("Baseline seems too large (>50cm)");
        }
        if (baseline < 0.01)
        {
            ROS_WARN("Baseline seems too small (<1cm)");
        }

        // Check rotation matrix
        cv::Mat identity = cv::Mat::eye(3, 3, CV_64F);
        double rotAngle = cv::norm(stereoCalib_.R - identity);
        ROS_INFO("Rotation between cameras: %.3f", rotAngle);
        if (rotAngle > 0.1)
        {
            ROS_WARN("Large rotation between cameras detected");
        }

        // Check if T vector is reasonable
        ROS_INFO("Translation vector: [%.3f, %.3f, %.3f]",
                 stereoCalib_.T.at<double>(0),
                 stereoCalib_.T.at<double>(1),
                 stereoCalib_.T.at<double>(2));

        ROS_INFO("=====================================\n");
    }

    // Private member functions
    void readBoolParamAliases(const std::vector<std::string>& names, bool& value)
    {
        for (const auto& name : names)
        {
            bool paramValue;
            if (pnh_.getParam(name, paramValue))
            {
                value = paramValue;
                return;
            }
        }
    }

    void readIntParamAliases(const std::vector<std::string>& names, int& value)
    {
        for (const auto& name : names)
        {
            int paramValue;
            if (pnh_.getParam(name, paramValue))
            {
                value = paramValue;
                return;
            }
        }
    }

    void readDoubleParamAliases(const std::vector<std::string>& names, float& value)
    {
        for (const auto& name : names)
        {
            double paramValue;
            if (pnh_.getParam(name, paramValue))
            {
                value = static_cast<float>(paramValue);
                return;
            }
        }
    }

    void readStringParamAliases(const std::vector<std::string>& names, std::string& value)
    {
        for (const auto& name : names)
        {
            std::string paramValue;
            if (pnh_.getParam(name, paramValue) && !paramValue.empty())
            {
                value = paramValue;
                return;
            }
        }
    }

    void initializeParameters()
    {
        readStringParamAliases({"calibration_file", "calibrationFile"}, config_.calibrationFile);
        readBoolParamAliases({"enable_visualization", "enableVisualization"}, config_.enableVisualization);
        readBoolParamAliases({"enable_debug", "enableDebug"}, config_.enableDebug);
        readBoolParamAliases({"enable_debug_trace", "enableDebugTrace"}, config_.enableDebugTrace);
        readBoolParamAliases({"enable_performance", "enablePerformance"}, config_.enablePerformance);
        readIntParamAliases({"single_marker_filter", "singleMarkerFilter"}, config_.singleMarkerFilter);
        readDoubleParamAliases({"fallback_marker_size", "markerSize"}, config_.markerSize);
        readDoubleParamAliases({"reprojection_error_threshold", "reprojErrorThreshold"}, config_.reprojErrorThreshold);
        readDoubleParamAliases({"huber_delta", "huberDelta"}, config_.huberDelta);
        readIntParamAliases({"joint_pnp_max_iterations", "jointPnPMaxIterations"}, config_.jointPnPMaxIterations);
        readDoubleParamAliases({"convergence_threshold", "convergenceThreshold"}, config_.convergenceThreshold);
        readIntParamAliases({"corner_refinement_win_size", "cornerRefinementWinSize"}, config_.cornerRefinementWinSize);
        readIntParamAliases({"corner_refinement_max_iterations", "cornerRefinementMaxIterations"}, config_.cornerRefinementMaxIterations);
        readDoubleParamAliases({"corner_refinement_min_accuracy", "cornerRefinementMinAccuracy"}, config_.cornerRefinementMinAccuracy);
        readDoubleParamAliases({"min_pixel_span_fallback", "minPixelSpan"}, config_.minPixelSpan);

        if (config_.enablePerformance)
        {
            config_.enableDebug = false;
            config_.enableDebugTrace = false;
            config_.enableVisualization = false;
        }
        else if (config_.enableDebugTrace)
        {
            config_.enableDebug = true;
        }

        if (!config_.enablePerformance)
        {
            ROS_INFO("\n");
            ROS_INFO("=== STEREO ARUCO DETECTOR CONFIGURATION ===");
            if (config_.enablePerformance)
                ROS_INFO("Debug mode: PERFORMANCE (no output)");
            else if (config_.enableDebugTrace)
                ROS_INFO("Debug mode: TRACE (full detailed output)");
            else if (config_.enableDebug)
                ROS_INFO("Debug mode: STANDARD (final results only)");
            else
                ROS_INFO("Debug mode: MINIMAL");

            ROS_INFO("Visualization: %s", config_.enableVisualization ? "ENABLED" : "DISABLED");
            if (config_.singleMarkerFilter >= 0)
                ROS_INFO("Single marker filter: %d", config_.singleMarkerFilter);
            else
                ROS_INFO("Single marker filter: disabled");
            ROS_INFO("Marker sizes: 363(15cm), 682(8cm), 417(24.5cm)");
            ROS_INFO("Reprojection threshold: %.1f px", config_.reprojErrorThreshold);
            ROS_INFO("Huber delta: %.2f px", config_.huberDelta);
            ROS_INFO("Joint PnP max iterations: %d", config_.jointPnPMaxIterations);
            ROS_INFO("Corner refinement: win=%d, max_iter=%d, min_accuracy=%.4f",
                     config_.cornerRefinementWinSize,
                     config_.cornerRefinementMaxIterations,
                     config_.cornerRefinementMinAccuracy);
            ROS_INFO("Calibration file: %s", config_.calibrationFile.c_str());
            ROS_INFO("=============================================\n");
        }
    }

    void initializeTopics()
    {
        // Setup synchronized subscribers for stereo images
        left_image_sub_.subscribe(nh_, "/stereo/left/image_raw", 1);
        right_image_sub_.subscribe(nh_, "/stereo/right/image_raw", 1);

        sync_.reset(new Synchronizer(SyncPolicy(10), left_image_sub_, right_image_sub_));
        sync_->registerCallback(boost::bind(&StereoArucoDetectorNode::imageCallback, this, _1, _2));

        // Publishers
        pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/aruco/pose", 10);
        marker_quality_pub_ = nh_.advertise<std_msgs::String>("/aruco/debug/marker_quality", 50);

        // Per-marker publishers for individual analysis
        for (int id : allowedMarkerIds_)
        {
            per_marker_pose_pubs_[id] = nh_.advertise<geometry_msgs::PoseStamped>(
                "/aruco/pose/marker_" + std::to_string(id), 10);
            ROS_INFO("Publishing per-marker topic: /aruco/pose/marker_%d", id);
        }

        if (config_.enableVisualization)
        {
            debug_image_pub_ = nh_.advertise<sensor_msgs::Image>("/aruco/image", 10);
            debug_left_pub_ = nh_.advertise<sensor_msgs::Image>("/aruco/debug/left_undistorted", 10);
            debug_right_pub_ = nh_.advertise<sensor_msgs::Image>("/aruco/debug/right_undistorted", 10);
        }

        ROS_INFO("Topics initialized.");
    }

    void initializeArucoSettings()
    {
        // Define allowed marker IDs and their sizes (in meters)
        allowedMarkerIds_ = {363, 682, 417};
        markerSizes_[363] = 0.15f;  // 15 cm
        markerSizes_[682] = 0.08f;  // 8 cm (CORRECTED from 0.088f)
        markerSizes_[417] = 0.245f; // 24.5 cm

        // Desired maximum operating ranges, used to derive per-marker pixel-span
        // gates with span_px ~= marker_size * focal_length * sqrt(2) / range_m.
        markerMaxReliableRanges_[417] = 3.0f;
        markerMaxReliableRanges_[363] = 0.8f;
        markerMaxReliableRanges_[682] = 0.5f;

        double minPixelSpanParam = config_.minPixelSpan;
        double range417 = markerMaxReliableRanges_[417];
        double range363 = markerMaxReliableRanges_[363];
        double range682 = markerMaxReliableRanges_[682];

        pnh_.param("min_pixel_span_fallback", minPixelSpanParam, minPixelSpanParam);
        pnh_.param("marker_417_max_reliable_range", range417, range417);
        pnh_.param("marker_363_max_reliable_range", range363, range363);
        pnh_.param("marker_682_max_reliable_range", range682, range682);

        config_.minPixelSpan = static_cast<float>(minPixelSpanParam);
        markerMaxReliableRanges_[417] = static_cast<float>(range417);
        markerMaxReliableRanges_[363] = static_cast<float>(range363);
        markerMaxReliableRanges_[682] = static_cast<float>(range682);

        dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
        parameters_ = cv::aruco::DetectorParameters::create();

        // Setup detection parameters
        parameters_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
        parameters_->cornerRefinementWinSize = config_.cornerRefinementWinSize;
        parameters_->cornerRefinementMaxIterations = config_.cornerRefinementMaxIterations;
        parameters_->cornerRefinementMinAccuracy = config_.cornerRefinementMinAccuracy;
        parameters_->adaptiveThreshWinSizeMin = 3;
        parameters_->adaptiveThreshWinSizeMax = 23;
        parameters_->adaptiveThreshWinSizeStep = 10;

        ROS_INFO("ArUco settings initialized for marker IDs: 363 (15cm), 682 (8cm), 417 (24.5cm)");
        ROS_INFO("Pixel-span gate ranges: 417<=%.2fm, 363<=%.2fm, 682<=%.2fm (fallback %.1f px diagonal)",
                 markerMaxReliableRanges_[417],
                 markerMaxReliableRanges_[363],
                 markerMaxReliableRanges_[682],
                 config_.minPixelSpan);
    }

    void initializeStereoSettings()
    {
        if (!loadStereoCalibration(config_.calibrationFile, stereoCalib_))
        {
            ROS_ERROR("Failed to load stereo calibration from %s", config_.calibrationFile.c_str());
            ROS_ERROR("Using default parameters (this will likely produce incorrect results)");
            setDefaultCalibration();
        }

        // Debug camera calibration parameters
        if (config_.enableDebugTrace) // CHANGED: was enableDebugOutput
        {
            debugCameraCalibration();
            debugStereoCalibration();
        }

        calculateProjectionMatrices();

        // Pre-compute undistortion maps for efficiency
        cv::Size imageSize(stereoCalib_.imageWidth, stereoCalib_.imageHeight);
        cv::initUndistortRectifyMap(stereoCalib_.leftCameraMatrix, stereoCalib_.leftDistCoeffs,
                                    cv::Mat(), stereoCalib_.leftCameraMatrix, imageSize,
                                    CV_32FC1, mapx1_, mapy1_);
        cv::initUndistortRectifyMap(stereoCalib_.rightCameraMatrix, stereoCalib_.rightDistCoeffs,
                                    cv::Mat(), stereoCalib_.rightCameraMatrix, imageSize,
                                    CV_32FC1, mapx2_, mapy2_);

        ROS_INFO("Stereo calibration loaded successfully.");
    }
    void debugCameraCalibration()
    {
        ROS_INFO("\n");
        ROS_INFO("=== CAMERA CALIBRATION DEBUG ===");
        ROS_INFO("Image size: %d x %d", stereoCalib_.imageWidth, stereoCalib_.imageHeight);

        ROS_INFO("\nLeft camera matrix:");
        for (int i = 0; i < 3; i++)
        {
            ROS_INFO("  [%12.8f, %12.8f, %12.8f]",
                     stereoCalib_.leftCameraMatrix.at<double>(i, 0),
                     stereoCalib_.leftCameraMatrix.at<double>(i, 1),
                     stereoCalib_.leftCameraMatrix.at<double>(i, 2));
        }

        ROS_INFO("\nLeft distortion coefficients:");
        ROS_INFO("  [%9.6f, %9.6f, %9.6f, %9.6f, %9.6f]",
                 stereoCalib_.leftDistCoeffs.at<double>(0),
                 stereoCalib_.leftDistCoeffs.at<double>(1),
                 stereoCalib_.leftDistCoeffs.at<double>(2),
                 stereoCalib_.leftDistCoeffs.at<double>(3),
                 stereoCalib_.leftDistCoeffs.at<double>(4));

        ROS_INFO("\nStereo baseline (T vector):");
        ROS_INFO("  [%9.6f, %9.6f, %9.6f] meters",
                 stereoCalib_.T.at<double>(0),
                 stereoCalib_.T.at<double>(1),
                 stereoCalib_.T.at<double>(2));

        ROS_INFO("================================\n");
    }

    void initializeTransformMatrices()
    {
        cv::Mat rotation_363 = cv::Mat::eye(3, 3, CV_64F);
        cv::Mat rotation_682 = cv::Mat::eye(3, 3, CV_64F);
        cv::Mat rotation_417 = cv::Mat::eye(3, 3, CV_64F);

        std::vector<MarkerTransform> markers = {
            {363, cv::Vec3f(0.275f, 0.208f, 0.0f), rotation_363},
            {682, cv::Vec3f(0.043f, 0.038f, 0.0f), rotation_682},
            {417, cv::Vec3f(-0.255f, -0.160f, 0.0f), rotation_417}};

        if (!config_.enablePerformance) // CHANGED: Only show if not in performance mode
        {
            ROS_INFO("\n");
            ROS_INFO("=== TRANSFORM MATRICES INITIALIZATION ===");
        }

        for (const auto &marker : markers)
        {
            // Step 1: Create ARUCO->LANDPAD transform
            cv::Mat TM_Aruco_To_Landpad = cv::Mat::eye(4, 4, CV_64F);

            cv::Mat rotMat;
            if (marker.rotation.type() != CV_64F)
            {
                marker.rotation.convertTo(rotMat, CV_64F);
            }
            else
            {
                rotMat = marker.rotation;
            }

            rotMat.copyTo(TM_Aruco_To_Landpad(cv::Rect(0, 0, 3, 3)));
            TM_Aruco_To_Landpad.at<double>(0, 3) = static_cast<double>(marker.position[0]);
            TM_Aruco_To_Landpad.at<double>(1, 3) = static_cast<double>(marker.position[1]);
            TM_Aruco_To_Landpad.at<double>(2, 3) = static_cast<double>(marker.position[2]);

            // Step 2: Invert to get LANDPAD->ARUCO
            cv::Mat TM_Landpad_To_Aruco = TM_Aruco_To_Landpad.inv();

            // Store the inverted matrix
            if (TM_Landpad_To_Aruco.type() != CV_64F)
            {
                TM_Landpad_To_Aruco.convertTo(TM_Landpad_To_Aruco_[marker.id], CV_64F);
            }
            else
            {
                TM_Landpad_To_Aruco_[marker.id] = TM_Landpad_To_Aruco;
            }

            if (!config_.enablePerformance) // CHANGED: Only show if not in performance mode
            {
                ROS_INFO("\nMarker ID %d:", marker.id);
                ROS_INFO("  Position offset: [%.3f, %.3f, %.3f] meters",
                         marker.position[0], marker.position[1], marker.position[2]);
            }

            if (config_.enableDebugTrace) // CHANGED: Only show detailed matrix in trace mode
            {
                ROS_INFO("  Landpad->Aruco transform matrix:");
                for (int i = 0; i < 4; i++)
                {
                    ROS_INFO("    [%8.3f, %8.3f, %8.3f, %8.3f]",
                             TM_Landpad_To_Aruco.at<double>(i, 0), TM_Landpad_To_Aruco.at<double>(i, 1),
                             TM_Landpad_To_Aruco.at<double>(i, 2), TM_Landpad_To_Aruco.at<double>(i, 3));
                }
            }
        }

        if (!config_.enablePerformance)
        {
            ROS_INFO("==========================================\n");
        }
    }

    bool loadStereoCalibration(const std::string &filename, StereoCalibration &calib)
    {
        cv::FileStorage fs(filename, cv::FileStorage::READ);
        if (!fs.isOpened())
        {
            ROS_ERROR("Failed to open: %s", filename.c_str());
            return false;
        }

        fs["image_width"] >> calib.imageWidth;
        fs["image_height"] >> calib.imageHeight;
        fs["left_camera_matrix"] >> calib.leftCameraMatrix;
        fs["left_distortion_coefficients"] >> calib.leftDistCoeffs;
        fs["right_camera_matrix"] >> calib.rightCameraMatrix;
        fs["right_distortion_coefficients"] >> calib.rightDistCoeffs;
        fs["R"] >> calib.R;
        fs["T"] >> calib.T;
        if (fs["E"].isNone() == false)
            fs["E"] >> calib.E;
        if (fs["F"].isNone() == false)
            fs["F"] >> calib.F;

        fs.release();

        // Convert T from mm to meters if needed
        if (cv::norm(calib.T) > 1.0)
        {
            calib.T = calib.T / 1000.0;
            if (config_.enableDebugTrace) // CHANGED: was enableDebugOutput
                ROS_INFO("Converted T from mm to meters");
        }

        if (calib.leftCameraMatrix.empty() || calib.rightCameraMatrix.empty())
        {
            ROS_ERROR("Calibration matrices empty");
            return false;
        }

        return true;
    }

    void setDefaultCalibration()
    {
        // Set default calibration parameters
        stereoCalib_.imageWidth = 640;
        stereoCalib_.imageHeight = 480;

        stereoCalib_.leftCameraMatrix = (cv::Mat_<double>(3, 3) << 1000, 0, 320, 0, 1000, 240, 0, 0, 1);
        stereoCalib_.rightCameraMatrix = (cv::Mat_<double>(3, 3) << 1000, 0, 320, 0, 1000, 240, 0, 0, 1);
        stereoCalib_.leftDistCoeffs = cv::Mat::zeros(5, 1, CV_64F);
        stereoCalib_.rightDistCoeffs = cv::Mat::zeros(5, 1, CV_64F);
        stereoCalib_.R = cv::Mat::eye(3, 3, CV_64F);
        stereoCalib_.T = (cv::Mat_<double>(3, 1) << 0.06, 0, 0); // 6cm baseline
    }

    double huberLoss(double residual, double delta)
    {
        double abs_residual = std::abs(residual);
        if (abs_residual <= delta)
        {
            return 0.5 * residual * residual;
        }
        else
        {
            return delta * (abs_residual - 0.5 * delta);
        }
    }

    void calculateProjectionMatrices()
    {
        // Create projection matrices
        stereoCalib_.P1 = stereoCalib_.leftCameraMatrix * cv::Mat::eye(3, 4, CV_64F);

        cv::Mat RT = cv::Mat::zeros(3, 4, CV_64F);
        stereoCalib_.R.copyTo(RT(cv::Rect(0, 0, 3, 3)));
        stereoCalib_.T.copyTo(RT(cv::Rect(3, 0, 1, 3)));
        stereoCalib_.P2 = stereoCalib_.rightCameraMatrix * RT;

        if (config_.enableDebugTrace) // CHANGED: was enableDebugOutput
        {
            ROS_INFO("Projection matrices calculated.");
        }
    }

    void imageCallback(const sensor_msgs::ImageConstPtr &left_msg,
                       const sensor_msgs::ImageConstPtr &right_msg)
    {
        try
        {
            if (config_.enableDebugTrace)
            {
                ROS_INFO("\n");
                ROS_INFO("------------------------------------------------------------");
                ROS_INFO("                    FRAME %d PROCESSING", frameCounter_);
                ROS_INFO("------------------------------------------------------------");
            }

            // Convert ROS images to OpenCV
            cv_bridge::CvImagePtr left_cv = cv_bridge::toCvCopy(left_msg, sensor_msgs::image_encodings::BGR8);
            cv_bridge::CvImagePtr right_cv = cv_bridge::toCvCopy(right_msg, sensor_msgs::image_encodings::BGR8);

            // Process the stereo pair
            geometry_msgs::PoseStamped pose_msg;
            cv::Mat debug_image;
            ros::Time measurementStamp = left_msg->header.stamp;
            if (measurementStamp.isZero())
            {
                measurementStamp = right_msg->header.stamp;
            }
            if (measurementStamp.isZero())
            {
                measurementStamp = ros::Time::now();
            }

            if (detectMarkers(left_cv->image, right_cv->image, pose_msg, debug_image, measurementStamp))
            {
                // Publish best single marker on /aruco/pose (backward compat, not used by EKF)
                pose_msg.header.stamp = measurementStamp;
                pose_msg.header.frame_id = "stereo_camera_frame";
                pose_pub_.publish(pose_msg);
            }

            // Publish debug images if visualization is enabled
            if (config_.enableVisualization && !debug_image.empty())
            {
                sensor_msgs::ImagePtr debug_msg = cv_bridge::CvImage(left_msg->header, "bgr8", debug_image).toImageMsg();
                debug_image_pub_.publish(debug_msg);
            }

            frameCounter_++;

            if (config_.enableDebugTrace)
            {
                ROS_INFO("------------------------------------------------------------");
                ROS_INFO("                  FRAME %d COMPLETE", frameCounter_ - 1);
                ROS_INFO("------------------------------------------------------------\n");
            }

            // Print statistics every 100 frames
            if (frameCounter_ % 100 == 0 && config_.enableDebug)
            {
                printStatistics();
            }
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
        catch (cv::Exception &e)
        {
            ROS_ERROR("OpenCV exception: %s", e.what());
        }
    }

    bool detectMarkers(const cv::Mat &leftImage, const cv::Mat &rightImage,
                       geometry_msgs::PoseStamped &pose_msg, cv::Mat &debug_image,
                       const ros::Time &measurementStamp)
    {
        // Convert to grayscale if needed
        cv::Mat grayLeft, grayRight;
        if (leftImage.channels() == 3)
        {
            cv::cvtColor(leftImage, grayLeft, cv::COLOR_BGR2GRAY);
        }
        else
        {
            grayLeft = leftImage.clone();
        }

        if (rightImage.channels() == 3)
        {
            cv::cvtColor(rightImage, grayRight, cv::COLOR_BGR2GRAY);
        }
        else
        {
            grayRight = rightImage.clone();
        }

        // Undistort images using pre-computed maps
        cv::Mat undistortedLeft, undistortedRight;
        cv::remap(grayLeft, undistortedLeft, mapx1_, mapy1_, cv::INTER_LINEAR);
        cv::remap(grayRight, undistortedRight, mapx2_, mapy2_, cv::INTER_LINEAR);

        // Create debug image if visualization is enabled
        if (config_.enableVisualization)
        {
            if (leftImage.channels() == 3)
            {
                cv::remap(leftImage, debug_image, mapx1_, mapy1_, cv::INTER_LINEAR);
            }
            else
            {
                cv::cvtColor(undistortedLeft, debug_image, cv::COLOR_GRAY2BGR);
            }
        }

        // Detect ArUco markers
        std::vector<std::vector<cv::Point2f>> leftCorners, rightCorners;
        std::vector<int> leftIds, rightIds;

        detectArUcoMarkers(undistortedLeft, leftCorners, leftIds);
        detectArUcoMarkers(undistortedRight, rightCorners, rightIds);

        if (leftIds.empty() || rightIds.empty())
        {
            if (config_.enableDebugTrace)
                ROS_INFO("No markers detected in one or both cameras");
            return false;
        }

        if (config_.enableDebugTrace)
        {
            ROS_INFO("\n");
            ROS_INFO("----- MARKER DETECTION RESULTS ----");
            ROS_INFO("Left camera detected %zu markers: ", leftIds.size());
            for (size_t i = 0; i < leftIds.size(); i++)
                ROS_INFO("  Marker %d", leftIds[i]);
            ROS_INFO("Right camera detected %zu markers: ", rightIds.size());
            for (size_t i = 0; i < rightIds.size(); i++)
                ROS_INFO("  Marker %d", rightIds[i]);
        }

        // Match markers between views and filter by allowed IDs
        std::vector<MatchedMarker> matched = matchAndFilterMarkers(leftCorners, leftIds, rightCorners, rightIds);

        if (matched.empty())
        {
            if (config_.enableDebugTrace)
                ROS_INFO("No valid matched markers found");
            return false;
        }

        // Pixel span gate: reject markers that are too small for reliable PnP
        float focalLength = stereoCalib_.leftCameraMatrix.at<double>(0, 0);
        for (auto& marker : matched)
        {
            if (!marker.valid) continue;

            float apparentSize = getApparentSize(marker.leftCorners);
            float markerSize = markerSizes_.count(marker.id) ? markerSizes_[marker.id] : config_.markerSize;
            float minPixelSpan = config_.minPixelSpan;
            if (markerMaxReliableRanges_.count(marker.id) && markerMaxReliableRanges_[marker.id] > 0.0f)
            {
                float targetRange = markerMaxReliableRanges_[marker.id];
                float derivedThreshold = markerSize * focalLength * std::sqrt(2.0f) / targetRange;
                minPixelSpan = std::max(config_.minPixelSpan, derivedThreshold);
            }
            float maxRange = markerSize * focalLength * std::sqrt(2.0f) / minPixelSpan;
            marker.pixelSpan = apparentSize;
            marker.minPixelSpanRequired = minPixelSpan;
            marker.maxReliableRange = maxRange;

            if (apparentSize < minPixelSpan)
            {
                marker.valid = false;
                marker.status = MarkerStatus::PIXEL_SPAN_TOO_SMALL;
                if (config_.enableDebug)
                {
                    ROS_INFO("  Marker %d rejected: %.1f px < %.1f px minimum (max reliable range: %.2fm)",
                            marker.id, apparentSize, minPixelSpan, maxRange);
                }
            }
            else if (config_.enableDebugTrace)
            {
                ROS_INFO("  Marker %d pixel span: %.1f px (min: %.1f, max range: %.2fm)",
                        marker.id, apparentSize, minPixelSpan, maxRange);
            }
        }

        totalMarkers_ += matched.size();

        if (config_.enableDebugTrace)
        {
            ROS_INFO("\n");
            ROS_INFO("--- RAW POSE ESTIMATION ---");
        }

        // Calculate raw single camera poses first (for debugging)
        calculateRawSingleCameraPoses(matched, undistortedLeft, undistortedRight);

        if (config_.enableDebugTrace)
        {
            ROS_INFO("\n");
            ROS_INFO("--- STEREO POSE REFINEMENT ---");
        }

        // Estimate poses using enhanced stereo algorithm
        estimatePose(matched);

        if (config_.enableDebugTrace)
        {
            ROS_INFO("\n");
            ROS_INFO("--- LANDPAD POSE CALCULATION ---");
        }

        // Process results and publish per-marker poses
        std::vector<std::string> markerResults;
        bool anyValidPublished = false;
        double bestReprojError = std::numeric_limits<double>::max();

        for (const auto &marker : matched)
        {
            statusCounts_[marker.status]++;
            publishMarkerQuality(marker, measurementStamp);

            if (marker.valid)
            {
                validMarkers_++;

                // Apply transform to get landpad pose
                cv::Vec3f position, orientation;
                if (!applyMarkerTransform(marker, position, orientation))
                    continue;

                // === Build rotation matrix and full quaternion conversion ===
                cv::Mat indivRotMat;
                cv::Mat oriVec = (cv::Mat_<double>(3, 1) <<
                    orientation[0], orientation[1], orientation[2]);
                cv::Rodrigues(oriVec, indivRotMat);

                if (indivRotMat.type() != CV_64F)
                    indivRotMat.convertTo(indivRotMat, CV_64F);

                double trace = indivRotMat.at<double>(0, 0)
                            + indivRotMat.at<double>(1, 1)
                            + indivRotMat.at<double>(2, 2);
                double w, x, y, z;

                if (trace > 0)
                {
                    double s = sqrt(trace + 1.0) * 2;
                    w = 0.25 * s;
                    x = (indivRotMat.at<double>(2, 1) - indivRotMat.at<double>(1, 2)) / s;
                    y = (indivRotMat.at<double>(0, 2) - indivRotMat.at<double>(2, 0)) / s;
                    z = (indivRotMat.at<double>(1, 0) - indivRotMat.at<double>(0, 1)) / s;
                }
                else if ((indivRotMat.at<double>(0, 0) > indivRotMat.at<double>(1, 1)) &&
                        (indivRotMat.at<double>(0, 0) > indivRotMat.at<double>(2, 2)))
                {
                    double s = sqrt(1.0 + indivRotMat.at<double>(0, 0)
                                - indivRotMat.at<double>(1, 1)
                                - indivRotMat.at<double>(2, 2)) * 2;
                    w = (indivRotMat.at<double>(2, 1) - indivRotMat.at<double>(1, 2)) / s;
                    x = 0.25 * s;
                    y = (indivRotMat.at<double>(0, 1) + indivRotMat.at<double>(1, 0)) / s;
                    z = (indivRotMat.at<double>(0, 2) + indivRotMat.at<double>(2, 0)) / s;
                }
                else if (indivRotMat.at<double>(1, 1) > indivRotMat.at<double>(2, 2))
                {
                    double s = sqrt(1.0 + indivRotMat.at<double>(1, 1)
                                - indivRotMat.at<double>(0, 0)
                                - indivRotMat.at<double>(2, 2)) * 2;
                    w = (indivRotMat.at<double>(0, 2) - indivRotMat.at<double>(2, 0)) / s;
                    x = (indivRotMat.at<double>(0, 1) + indivRotMat.at<double>(1, 0)) / s;
                    y = 0.25 * s;
                    z = (indivRotMat.at<double>(1, 2) + indivRotMat.at<double>(2, 1)) / s;
                }
                else
                {
                    double s = sqrt(1.0 + indivRotMat.at<double>(2, 2)
                                - indivRotMat.at<double>(0, 0)
                                - indivRotMat.at<double>(1, 1)) * 2;
                    w = (indivRotMat.at<double>(1, 0) - indivRotMat.at<double>(0, 1)) / s;
                    x = (indivRotMat.at<double>(0, 2) + indivRotMat.at<double>(2, 0)) / s;
                    y = (indivRotMat.at<double>(1, 2) + indivRotMat.at<double>(2, 1)) / s;
                    z = 0.25 * s;
                }

                double norm = sqrt(x * x + y * y + z * z + w * w);

                // === Publish per-marker pose ===
                if (per_marker_pose_pubs_.count(marker.id))
                {
                    geometry_msgs::PoseStamped individual_msg;
                    individual_msg.header.stamp = measurementStamp;
                    individual_msg.header.frame_id = "stereo_camera_frame";
                    individual_msg.pose.position.x = position[0];
                    individual_msg.pose.position.y = position[1];
                    individual_msg.pose.position.z = position[2];
                    individual_msg.pose.orientation.x = (norm > 0) ? x / norm : 0;
                    individual_msg.pose.orientation.y = (norm > 0) ? y / norm : 0;
                    individual_msg.pose.orientation.z = (norm > 0) ? z / norm : 0;
                    individual_msg.pose.orientation.w = (norm > 0) ? w / norm : 1;

                    per_marker_pose_pubs_[marker.id].publish(individual_msg);
                    anyValidPublished = true;

                    // Track best marker for optional averaged topic
                    if (marker.reprojectionError < bestReprojError)
                    {
                        bestReprojError = marker.reprojectionError;
                        pose_msg.pose = individual_msg.pose;
                    }
                }

                // Store result for debug summary
                char result[256];
                sprintf(result, "Marker %d: [%7.3f, %7.3f, %7.3f] (reproj: %.2fpx, span: %.0fpx)",
                        marker.id, position[0], position[1], position[2],
                        marker.reprojectionError, getApparentSize(marker.leftCorners));
                markerResults.push_back(std::string(result));

                // Draw pose visualization if enabled
                if (config_.enableVisualization && !debug_image.empty())
                {
                    float markerSize = markerSizes_.count(marker.id) ? markerSizes_[marker.id] : config_.markerSize;
                    drawPose(debug_image, marker.rvec, marker.tvec, stereoCalib_.leftCameraMatrix,
                            cv::Mat(), markerSize / 2);

                    cv::putText(debug_image,
                                "ID:" + std::to_string(marker.id) +
                                    " E:" + std::to_string(marker.reprojectionError).substr(0, 4) + "px",
                                marker.leftCorners[0] - cv::Point2f(10, 15),
                                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 255), 1);
                }
            }
            else
            {
                if (config_.enableDebugTrace)
                {
                    ROS_INFO("  Marker %d: FAILED (%s)", marker.id, statusToString(marker.status));
                }
            }
        }

        if (!anyValidPublished)
        {
            if (config_.enableDebugTrace)
                ROS_INFO("No valid marker poses published");
            return false;
        }

        // Debug summary
        if (config_.enableDebug)
        {
            ROS_INFO("\n");
            ROS_INFO("--- PER-MARKER POSE ESTIMATES (no averaging) ---");
            for (const auto &result : markerResults)
            {
                ROS_INFO("  %s", result.c_str());
            }
            ROS_INFO("  Published %zu individual marker poses", markerResults.size());
            ROS_INFO("------------------------------------------------");
        }

        return true;
    }

    void calculateRawSingleCameraPoses(std::vector<MatchedMarker> &matched,
                                       const cv::Mat &leftImage, const cv::Mat &rightImage)
    {
        for (auto &marker : matched)
        {
            if (!marker.valid)
                continue;

            float markerSize = markerSizes_.count(marker.id) ? markerSizes_[marker.id] : config_.markerSize;
            std::vector<cv::Point3f> objectPoints = createMarkerModel(markerSize);

            // Calculate raw left camera pose on undistorted image (zero distortion model)
            bool leftSuccess = cv::solvePnP(objectPoints, marker.leftCorners,
                                            stereoCalib_.leftCameraMatrix, cv::Mat(),
                                            marker.rawLeftRvec, marker.rawLeftTvec, false, cv::SOLVEPNP_IPPE_SQUARE);

            // Calculate raw right camera pose on undistorted image (zero distortion model)
            bool rightSuccess = cv::solvePnP(objectPoints, marker.rightCorners,
                                             stereoCalib_.rightCameraMatrix, cv::Mat(),
                                             marker.rawRightRvec, marker.rawRightTvec, false, cv::SOLVEPNP_IPPE_SQUARE);

            if (config_.enableDebugTrace && leftSuccess)
            {
                ROS_INFO("  Marker %d RAW LEFT:  t=[%6.3f, %6.3f, %6.3f] r=[%5.2f, %5.2f, %5.2f] deg",
                         marker.id,
                         marker.rawLeftTvec.at<double>(0), marker.rawLeftTvec.at<double>(1), marker.rawLeftTvec.at<double>(2),
                         marker.rawLeftRvec.at<double>(0) * 180.0 / M_PI,
                         marker.rawLeftRvec.at<double>(1) * 180.0 / M_PI,
                         marker.rawLeftRvec.at<double>(2) * 180.0 / M_PI);
            }
            if (config_.enableDebugTrace && rightSuccess)
            {
                ROS_INFO("  Marker %d RAW RIGHT: t=[%6.3f, %6.3f, %6.3f] r=[%5.2f, %5.2f, %5.2f] deg",
                         marker.id,
                         marker.rawRightTvec.at<double>(0), marker.rawRightTvec.at<double>(1), marker.rawRightTvec.at<double>(2),
                         marker.rawRightRvec.at<double>(0) * 180.0 / M_PI,
                         marker.rawRightRvec.at<double>(1) * 180.0 / M_PI,
                         marker.rawRightRvec.at<double>(2) * 180.0 / M_PI);
            }
        }
    }

    void processSingleCameraMode(std::vector<MatchedMarker> &matched)
    {
        ROS_INFO("\n");
        ROS_INFO("=== SINGLE CAMERA MODE (LEFT CAMERA ONLY) ===");

        for (auto &marker : matched)
        {
            if (!marker.valid)
                continue;

            // Use the raw left camera pose as the final pose
            marker.rvec = marker.rawLeftRvec.clone();
            marker.tvec = marker.rawLeftTvec.clone();
            marker.status = MarkerStatus::OK;

            if (config_.enableDebugTrace)
            {
                ROS_INFO("\nMarker ID %d (Single Camera Mode):", marker.id);
                ROS_INFO("  Raw pose: t=[%.3f, %.3f, %.3f] r=[%.3f, %.3f, %.3f]",
                         marker.tvec.at<double>(0), marker.tvec.at<double>(1), marker.tvec.at<double>(2),
                         marker.rvec.at<double>(0), marker.rvec.at<double>(1), marker.rvec.at<double>(2));

                // Apply transform and show landpad pose
                cv::Vec3f landpadPos, landpadOri;
                if (applyMarkerTransform(marker, landpadPos, landpadOri))
                {
                    ROS_INFO("  Landpad pose: t=[%.3f, %.3f, %.3f] r=[%.3f, %.3f, %.3f]",
                             landpadPos[0], landpadPos[1], landpadPos[2],
                             landpadOri[0], landpadOri[1], landpadOri[2]);
                }
            }
        }

        ROS_INFO("==============================================\n");
    }

    void debugRawMarkerPose(const MatchedMarker &marker, const std::string &camera)
    {
        cv::Mat rvec, tvec;
        if (camera == "LEFT")
        {
            rvec = marker.rawLeftRvec;
            tvec = marker.rawLeftTvec;
        }
        else
        {
            rvec = marker.rawRightRvec;
            tvec = marker.rawRightTvec;
        }

        ROS_INFO("\n");
        ROS_INFO("=== RAW %s CAMERA POSE DEBUG ===", camera.c_str());
        ROS_INFO("Marker ID: %d (size: %.3fm)", marker.id,
                 markerSizes_.count(marker.id) ? markerSizes_[marker.id] : config_.markerSize);
        ROS_INFO("Raw marker pose in %s camera frame:", camera.c_str());
        ROS_INFO("  Translation: [%8.3f, %8.3f, %8.3f] meters",
                 tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
        ROS_INFO("  Rotation:    [%8.3f, %8.3f, %8.3f] radians",
                 rvec.at<double>(0), rvec.at<double>(1), rvec.at<double>(2));
        ROS_INFO("  Rotation:    [%8.1f, %8.1f, %8.1f] degrees",
                 rvec.at<double>(0) * 180.0 / M_PI,
                 rvec.at<double>(1) * 180.0 / M_PI,
                 rvec.at<double>(2) * 180.0 / M_PI);

        // Convert to rotation matrix and show orientation details
        cv::Mat rotMat;
        cv::Rodrigues(rvec, rotMat);

        ROS_INFO("  Rotation matrix:");
        for (int i = 0; i < 3; i++)
        {
            ROS_INFO("    [%7.3f, %7.3f, %7.3f]",
                     rotMat.at<double>(i, 0), rotMat.at<double>(i, 1), rotMat.at<double>(i, 2));
        }

        // Show marker axes directions in camera frame
        cv::Vec3d markerX(rotMat.at<double>(0, 0), rotMat.at<double>(1, 0), rotMat.at<double>(2, 0));
        cv::Vec3d markerY(rotMat.at<double>(0, 1), rotMat.at<double>(1, 1), rotMat.at<double>(2, 1));
        cv::Vec3d markerZ(rotMat.at<double>(0, 2), rotMat.at<double>(1, 2), rotMat.at<double>(2, 2));

        ROS_INFO("  Marker axes in camera frame:");
        ROS_INFO("    X-axis: [%6.3f, %6.3f, %6.3f] (should point right on marker)",
                 markerX[0], markerX[1], markerX[2]);
        ROS_INFO("    Y-axis: [%6.3f, %6.3f, %6.3f] (should point up on marker)",
                 markerY[0], markerY[1], markerY[2]);
        ROS_INFO("    Z-axis: [%6.3f, %6.3f, %6.3f] (should point out of marker)",
                 markerZ[0], markerZ[1], markerZ[2]);

        // Interpret orientation
        ROS_INFO("  Orientation analysis:");
        ROS_INFO("    Marker Y points %s (camera Y is down)", markerY[1] > 0 ? "DOWN" : "UP");
        ROS_INFO("    Marker is %s relative to camera",
                 std::abs(markerZ[2]) > 0.9 ? "FACING CAMERA" : "AT AN ANGLE");

        // Apply landpad transform if available
        if (TM_Landpad_To_Aruco_.count(marker.id))
        {
            // Create temporary marker with this pose
            MatchedMarker tempMarker = marker;
            tempMarker.rvec = rvec;
            tempMarker.tvec = tvec;

            cv::Vec3f landpadPos, landpadOri;
            if (applyMarkerTransform(tempMarker, landpadPos, landpadOri))
            {
                ROS_INFO("  Transformed landpad pose:");
                ROS_INFO("    Translation: [%8.3f, %8.3f, %8.3f] meters",
                         landpadPos[0], landpadPos[1], landpadPos[2]);
                ROS_INFO("    Rotation:    [%8.3f, %8.3f, %8.3f] radians",
                         landpadOri[0], landpadOri[1], landpadOri[2]);
                ROS_INFO("    Rotation:    [%8.1f, %8.1f, %8.1f] degrees",
                         landpadOri[0] * 180.0 / M_PI,
                         landpadOri[1] * 180.0 / M_PI,
                         landpadOri[2] * 180.0 / M_PI);
            }
        }

        ROS_INFO("================================\n");
    }

    void detectArUcoMarkers(const cv::Mat &image, std::vector<std::vector<cv::Point2f>> &corners, std::vector<int> &ids)
    {
        try
        {
            cv::aruco::detectMarkers(image, dictionary_, corners, ids, parameters_);
        }
        catch (const cv::Exception &e)
        {
            ROS_WARN("ArUco detection error: %s", e.what());
            corners.clear();
            ids.clear();
        }
    }

    std::vector<MatchedMarker> matchAndFilterMarkers(const std::vector<std::vector<cv::Point2f>> &leftCorners,
                                                     const std::vector<int> &leftIds,
                                                     const std::vector<std::vector<cv::Point2f>> &rightCorners,
                                                     const std::vector<int> &rightIds)
    {
        std::vector<MatchedMarker> matched;
        std::map<int, size_t> rightIdMap;

        // Create mapping for right camera detections
        for (size_t i = 0; i < rightIds.size(); i++)
        {
            rightIdMap[rightIds[i]] = i;
        }

        // Match and filter markers
        for (size_t i = 0; i < leftIds.size(); i++)
        {
            int markerId = leftIds[i];

            // Check if marker ID is in allowed list
            if (allowedMarkerIds_.find(markerId) == allowedMarkerIds_.end())
            {
                MatchedMarker m;
                m.id = markerId;
                m.valid = false;
                m.status = MarkerStatus::FILTERED_OUT;
                matched.push_back(m);
                continue;
            }

            // Single marker filter (for debugging individual markers)
            if (config_.singleMarkerFilter != -1 && markerId != config_.singleMarkerFilter)
            {
                MatchedMarker m;
                m.id = markerId;
                m.valid = false;
                m.status = MarkerStatus::FILTERED_OUT;
                matched.push_back(m);
                continue;
            }

            // Check if marker exists in right camera
            auto it = rightIdMap.find(markerId);
            if (it != rightIdMap.end()) // REMOVED: || config_.enableSingleCameraMode
            {
                MatchedMarker m;
                m.id = markerId;
                m.leftCorners = leftCorners[i];
                m.rightCorners = rightCorners[it->second]; // This will always be valid now
                m.valid = true;
                m.status = MarkerStatus::OK;
                matched.push_back(m);

                if (config_.enableDebugTrace)
                {
                    ROS_INFO("  Matched marker ID %d (size: %.3fm)", markerId,
                             markerSizes_.count(markerId) ? markerSizes_[markerId] : config_.markerSize);
                }
            }
            else
            {
                MatchedMarker m;
                m.id = markerId;
                m.valid = false;
                m.status = MarkerStatus::NOT_MATCHED;
                matched.push_back(m);

                if (config_.enableDebugTrace)
                {
                    ROS_INFO("  Marker ID %d not found in right camera", markerId);
                }
            }
        }

        return matched;
    }
    void estimatePose(std::vector<MatchedMarker> &matched)
    {
        for (auto &marker : matched)
        {
            if (!marker.valid)
                continue;

            try
            {
                // Get marker-specific size
                float markerSize = markerSizes_.count(marker.id) ? markerSizes_[marker.id] : config_.markerSize;
                std::vector<cv::Point3f> currentMarkerModel = createMarkerModel(markerSize);

                if (config_.enableDebugTrace)
                {
                    ROS_INFO("  Processing marker %d (%.1fcm):", marker.id, markerSize * 100);
                }

                // Sequential Joint Stereo PnP (left -> right -> left)
                cv::Mat rvec, tvec;
                bool success = sequentialJointStereoPnP(marker, currentMarkerModel, rvec, tvec);

                if (!success)
                {
                    marker.valid = false;
                    if (config_.enableDebugTrace)
                        ROS_INFO("     Stereo PnP failed");
                    continue;
                }

                marker.rvec = rvec;
                marker.tvec = tvec;

                if (config_.enableDebugTrace)
                {
                    ROS_INFO("    REFINED: t=[%6.3f, %6.3f, %6.3f] r=[%5.2f, %5.2f, %5.2f] deg",
                             marker.tvec.at<double>(0), marker.tvec.at<double>(1), marker.tvec.at<double>(2),
                             marker.rvec.at<double>(0) * 180.0 / M_PI,
                             marker.rvec.at<double>(1) * 180.0 / M_PI,
                             marker.rvec.at<double>(2) * 180.0 / M_PI);
                }

                // Final error validation
                calculateFinalErrors(marker, currentMarkerModel);

                if (config_.enableDebug)
                {
                    ROS_INFO("    Marker %d errors: L=%.2fpx R=%.2fpx Avg=%.2fpx (threshold=%.1fpx)",
                            marker.id, marker.leftReprojectionError, 
                            marker.rightReprojectionError, marker.reprojectionError,
                            config_.reprojErrorThreshold);
                }

                if (marker.reprojectionError > config_.reprojErrorThreshold)
                {
                    marker.valid = false;
                    marker.status = MarkerStatus::REPROJECTION_ERROR_AVERAGE_TOO_HIGH;
                    if (config_.enableDebugTrace)
                        ROS_INFO("     Average reprojection error too high: %.2fpx", marker.reprojectionError);
                    continue;
                }

                // Success!
                marker.status = MarkerStatus::OK;

                if (config_.enableDebugTrace)
                {
                    ROS_INFO("     Final errors: L=%.2fpx R=%.2fpx Avg=%.2fpx",
                             marker.leftReprojectionError, marker.rightReprojectionError, marker.reprojectionError);
                    ROS_INFO("     Stereo pose validation successful");
                }
            }
            catch (const cv::Exception &e)
            {
                ROS_WARN("Exception during pose estimation for marker %d: %s", marker.id, e.what());
                marker.valid = false;
                marker.status = MarkerStatus::UNKNOWN;
            }
        }
    }

    bool sequentialJointStereoPnP(MatchedMarker &marker, const std::vector<cv::Point3f> &objectPoints,
                                  cv::Mat &rvec, cv::Mat &tvec)
    {
        try
        {
            if (config_.enableDebugTrace)
            {
                ROS_INFO("  Running sequential joint stereo PnP...");
            }

            // Initial estimate using left camera on undistorted image (zero distortion model)
            bool success = cv::solvePnP(objectPoints, marker.leftCorners,
                                        stereoCalib_.leftCameraMatrix, cv::Mat(),
                                        rvec, tvec, false, cv::SOLVEPNP_IPPE_SQUARE);

            if (!success)
            {
                marker.status = MarkerStatus::JOINT_PNP_FAILED;
                marker.detailedFailureReason = "Initial PnP estimate failed";
                return false;
            }

            // Calculate initial combined error
            marker.jointPnPInitialError = calculateStereoReprojectionError(objectPoints, rvec, tvec, marker);

            if (config_.enableDebugTrace)
            {
                ROS_INFO("    Initial combined stereo error: %.2fpx", marker.jointPnPInitialError);
            }

            cv::Mat bestRvec = rvec.clone();
            cv::Mat bestTvec = tvec.clone();
            double bestError = marker.jointPnPInitialError;

            int iterations = 0;
            bool converged = false;

            // Sequential optimization loop: left -> right -> left
            for (int iter = 0; iter < config_.jointPnPMaxIterations / 3 && !converged; iter++)
            {
                cv::Mat iterRvec = bestRvec.clone();
                cv::Mat iterTvec = bestTvec.clone();
                double prevError = bestError;

                cv::TermCriteria criteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 5, config_.convergenceThreshold);

                // Stage 1: Refine using left camera
                cv::solvePnPRefineLM(objectPoints, marker.leftCorners,
                                     stereoCalib_.leftCameraMatrix, cv::Mat(),
                                     iterRvec, iterTvec, criteria);

                // Stage 2: Transform to right camera and refine
                cv::Mat R_marker;
                cv::Rodrigues(iterRvec, R_marker);
                cv::Mat R_right = stereoCalib_.R * R_marker;
                cv::Mat rvec_right;
                cv::Rodrigues(R_right, rvec_right);
                cv::Mat tvec_right = stereoCalib_.R * iterTvec + stereoCalib_.T;

                cv::solvePnPRefineLM(objectPoints, marker.rightCorners,
                                     stereoCalib_.rightCameraMatrix, cv::Mat(),
                                     rvec_right, tvec_right, criteria);

                // Transform back to left camera coordinate system
                cv::Mat R_right_refined;
                cv::Rodrigues(rvec_right, R_right_refined);
                cv::Mat R_left_refined = stereoCalib_.R.t() * R_right_refined;
                cv::Rodrigues(R_left_refined, iterRvec);
                iterTvec = stereoCalib_.R.t() * (tvec_right - stereoCalib_.T);

                // Stage 3: Final refinement using left camera
                cv::solvePnPRefineLM(objectPoints, marker.leftCorners,
                                     stereoCalib_.leftCameraMatrix, cv::Mat(),
                                     iterRvec, iterTvec, criteria);

                // Calculate combined error
                double currentError = calculateStereoReprojectionError(objectPoints, iterRvec, iterTvec, marker);

                iterations += 3; // Three stages per iteration

                // Check for improvement
                if (currentError < bestError)
                {
                    bestError = currentError;
                    bestRvec = iterRvec.clone();
                    bestTvec = iterTvec.clone();
                }

                // Check for convergence
                if (std::abs(currentError - prevError) < config_.convergenceThreshold)
                {
                    converged = true;
                }
            }

            // Apply best solution
            rvec = bestRvec;
            tvec = bestTvec;
            marker.jointPnPFinalError = bestError;
            marker.jointPnPIterations = iterations;
            marker.jointPnPConverged = converged;

            if (config_.enableDebugTrace)
            {
                ROS_INFO("    %s after %d iterations",
                         converged ? "Converged" : "Max iterations reached", iterations);
                ROS_INFO("    Error: %.2f -> %.2fpx",
                         marker.jointPnPInitialError, marker.jointPnPFinalError);
            }

            return true;
        }
        catch (const cv::Exception &e)
        {
            ROS_ERROR("Exception in sequentialJointStereoPnP: %s", e.what());
            marker.detailedFailureReason = "sequentialJointStereoPnP exception: " + std::string(e.what());
            marker.status = MarkerStatus::JOINT_PNP_FAILED;
            return false;
        }
    }

    double calculateStereoReprojectionError(const std::vector<cv::Point3f> &objectPoints,
                                            const cv::Mat &rvec, const cv::Mat &tvec,
                                            const MatchedMarker &marker)
    {
        // Left camera error with Huber loss
        std::vector<cv::Point2f> projectedLeft;
        cv::projectPoints(objectPoints, rvec, tvec,
                          stereoCalib_.leftCameraMatrix, cv::Mat(), projectedLeft);

        double leftError = 0.0;
        for (size_t i = 0; i < marker.leftCorners.size(); i++)
        {
            double dx = projectedLeft[i].x - marker.leftCorners[i].x;
            double dy = projectedLeft[i].y - marker.leftCorners[i].y;
            double pixelError = std::sqrt(dx * dx + dy * dy);
            leftError += huberLoss(pixelError, config_.huberDelta);
        }
        leftError = std::sqrt(leftError / marker.leftCorners.size());

        // Right camera error (transform pose to right camera) with Huber loss
        cv::Mat R_marker;
        cv::Rodrigues(rvec, R_marker);
        cv::Mat R_right = stereoCalib_.R * R_marker;
        cv::Mat rvec_right;
        cv::Rodrigues(R_right, rvec_right);
        cv::Mat tvec_right = stereoCalib_.R * tvec + stereoCalib_.T;

        std::vector<cv::Point2f> projectedRight;
        cv::projectPoints(objectPoints, rvec_right, tvec_right,
                          stereoCalib_.rightCameraMatrix, cv::Mat(), projectedRight);

        double rightError = 0.0;
        for (size_t i = 0; i < marker.rightCorners.size(); i++)
        {
            double dx = projectedRight[i].x - marker.rightCorners[i].x;
            double dy = projectedRight[i].y - marker.rightCorners[i].y;
            double pixelError = std::sqrt(dx * dx + dy * dy);
            rightError += huberLoss(pixelError, config_.huberDelta);
        }
        rightError = std::sqrt(rightError / marker.rightCorners.size());

        return (leftError + rightError) / 2.0;
    }

    void calculateFinalErrors(MatchedMarker &marker, const std::vector<cv::Point3f> &objectPoints)
    {
        // Calculate reprojection errors on undistorted images using zero distortion model with Huber loss
        std::vector<cv::Point2f> projectedLeft, projectedRight;

        cv::projectPoints(objectPoints, marker.rvec, marker.tvec,
                          stereoCalib_.leftCameraMatrix, cv::Mat(), projectedLeft);

        // Transform pose to right camera
        cv::Mat R_marker;
        cv::Rodrigues(marker.rvec, R_marker);
        cv::Mat R_right = stereoCalib_.R * R_marker;
        cv::Mat rvec_right;
        cv::Rodrigues(R_right, rvec_right);
        cv::Mat tvec_right = stereoCalib_.R * marker.tvec + stereoCalib_.T;

        cv::projectPoints(objectPoints, rvec_right, tvec_right,
                          stereoCalib_.rightCameraMatrix, cv::Mat(), projectedRight);

        // Calculate errors with Huber loss
        double leftError = 0.0, rightError = 0.0;
        for (size_t i = 0; i < marker.leftCorners.size(); i++)
        {
            // Left camera error with Huber loss
            double dx_left = projectedLeft[i].x - marker.leftCorners[i].x;
            double dy_left = projectedLeft[i].y - marker.leftCorners[i].y;
            double pixelError_left = std::sqrt(dx_left * dx_left + dy_left * dy_left);
            leftError += huberLoss(pixelError_left, config_.huberDelta);

            // Right camera error with Huber loss
            double dx_right = projectedRight[i].x - marker.rightCorners[i].x;
            double dy_right = projectedRight[i].y - marker.rightCorners[i].y;
            double pixelError_right = std::sqrt(dx_right * dx_right + dy_right * dy_right);
            rightError += huberLoss(pixelError_right, config_.huberDelta);
        }

        marker.leftReprojectionError = std::sqrt(leftError / marker.leftCorners.size());
        marker.rightReprojectionError = std::sqrt(rightError / marker.rightCorners.size());
        marker.reprojectionError = (marker.leftReprojectionError + marker.rightReprojectionError) / 2.0;
    }

    bool applyMarkerTransform(const MatchedMarker &marker, cv::Vec3f &position, cv::Vec3f &orientation)
    {
        if (TM_Landpad_To_Aruco_.count(marker.id))
        {
            if (config_.enableDebugTrace)
            {
                ROS_INFO("    Marker %d input pose: t=[%6.3f, %6.3f, %6.3f]",
                         marker.id, marker.tvec.at<double>(0), marker.tvec.at<double>(1), marker.tvec.at<double>(2));
            }

            // Create Aruco-to-Camera transform from PnP result
            cv::Mat rotMat;
            cv::Rodrigues(marker.rvec, rotMat);

            if (rotMat.type() != CV_64F)
            {
                rotMat.convertTo(rotMat, CV_64F);
            }

            cv::Mat TM_Aruco_To_Camera = cv::Mat::eye(4, 4, CV_64F);
            rotMat.copyTo(TM_Aruco_To_Camera(cv::Rect(0, 0, 3, 3)));
            TM_Aruco_To_Camera.at<double>(0, 3) = marker.tvec.at<double>(0);
            TM_Aruco_To_Camera.at<double>(1, 3) = marker.tvec.at<double>(1);
            TM_Aruco_To_Camera.at<double>(2, 3) = marker.tvec.at<double>(2);

            cv::Mat TM_Landpad_To_Camera = TM_Aruco_To_Camera * TM_Landpad_To_Aruco_[marker.id];

            // // Apply camera->landpad frame correction (camera backwards) (180deg around X)
            // cv::Mat R_correction = (cv::Mat_<double>(3,3) <<
            // 	1, 0, 0,
            // 	0, -1, 0,
            // 	0, 0, 1);
            // cv::Mat TM_correction = cv::Mat::eye(4,4,CV_64F);
            // R_correction.copyTo(TM_correction(cv::Rect(0,0,3,3)));
            // TM_Landpad_To_Camera = TM_correction * TM_Landpad_To_Camera;
            // // --------------------------------

            // Extract position
            position[0] = static_cast<float>(TM_Landpad_To_Camera.at<double>(0, 3));
            position[1] = static_cast<float>(TM_Landpad_To_Camera.at<double>(1, 3));
            position[2] = static_cast<float>(TM_Landpad_To_Camera.at<double>(2, 3));

            // Extract rotation
            cv::Mat rotPart = TM_Landpad_To_Camera(cv::Rect(0, 0, 3, 3)).clone();

            cv::Mat landpadRvec;
            cv::Rodrigues(rotPart, landpadRvec);

            orientation[0] = static_cast<float>(landpadRvec.at<double>(0));
            orientation[1] = static_cast<float>(landpadRvec.at<double>(1));
            orientation[2] = static_cast<float>(landpadRvec.at<double>(2));

            if (config_.enableDebugTrace)
            {
                ROS_INFO("    TRANSFORM -> Landpad: [%7.3f, %7.3f, %7.3f]",
                         position[0], position[1], position[2]);
            }

            return true;
        }
        else
        {
            // Use raw marker pose if no transform defined
            position[0] = static_cast<float>(marker.tvec.at<double>(0));
            position[1] = static_cast<float>(marker.tvec.at<double>(1));
            position[2] = static_cast<float>(marker.tvec.at<double>(2));
            orientation[0] = static_cast<float>(marker.rvec.at<double>(0));
            orientation[1] = static_cast<float>(marker.rvec.at<double>(1));
            orientation[2] = static_cast<float>(marker.rvec.at<double>(2));

            if (config_.enableDebugTrace)
            {
                ROS_INFO("    RAW POSE -> (no transform): [%7.3f, %7.3f, %7.3f]",
                         position[0], position[1], position[2]);
            }

            return true;
        }
    }

    float getApparentSize(const std::vector<cv::Point2f>& corners)
    {
        // Calculate the maximum diagonal distance between any two corners
        // This represents the apparent size of the marker in pixels
        float maxDist = 0;
        for (size_t i = 0; i < corners.size(); i++)
        {
            for (size_t j = i + 1; j < corners.size(); j++)
            {
                float dist = cv::norm(corners[i] - corners[j]);
                maxDist = std::max(maxDist, dist);
            }
        }
        return maxDist;
    }

    void publishMarkerQuality(const MatchedMarker &marker, const ros::Time &stamp)
    {
        if (!marker_quality_pub_)
        {
            return;
        }

        bool hasRange = false;
        double range = 0.0;
        if (!marker.tvec.empty())
        {
            range = marker.tvec.at<double>(2);
            hasRange = std::isfinite(range);
        }

        std::ostringstream ss;
        ss << std::fixed << std::setprecision(4);
        ss << "{"
           << "\"stamp\":" << stamp.toSec() << ","
           << "\"marker_id\":" << marker.id << ","
           << "\"accepted\":" << (marker.valid ? "true" : "false") << ","
           << "\"status\":\"" << statusToString(marker.status) << "\","
           << "\"pixel_span_diag\":" << marker.pixelSpan << ",";
        if (std::isfinite(marker.minPixelSpanRequired) && marker.minPixelSpanRequired > 0.0)
        {
            ss << "\"min_pixel_span_diag\":" << marker.minPixelSpanRequired << ",";
        }
        else
        {
            ss << "\"min_pixel_span_diag\":null,";
        }
        if (std::isfinite(marker.maxReliableRange) && marker.maxReliableRange > 0.0)
        {
            ss << "\"max_reliable_range_m\":" << marker.maxReliableRange << ",";
        }
        else
        {
            ss << "\"max_reliable_range_m\":null,";
        }
        if (hasRange)
        {
            ss << "\"range_m\":" << range << ",";
        }
        else
        {
            ss << "\"range_m\":null,";
        }
        ss
           << "\"reprojection_error_px\":" << marker.reprojectionError << ","
           << "\"left_reprojection_error_px\":" << marker.leftReprojectionError << ","
           << "\"right_reprojection_error_px\":" << marker.rightReprojectionError << ","
           << "\"joint_pnp_converged\":" << (marker.jointPnPConverged ? "true" : "false")
           << "}";

        std_msgs::String msg;
        msg.data = ss.str();
        marker_quality_pub_.publish(msg);
    }

    std::vector<cv::Point3f> createMarkerModel(float markerSize)
    {
        float halfSize = markerSize / 2.0f;
        return {
            cv::Point3f(-halfSize, halfSize, 0), // top-left
            cv::Point3f(halfSize, halfSize, 0),  // top-right
            cv::Point3f(halfSize, -halfSize, 0), // bottom-right
            cv::Point3f(-halfSize, -halfSize, 0) // bottom-left
        };
    }

    void drawPose(cv::Mat &image, const cv::Mat &rvec, const cv::Mat &tvec,
                  const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs, float length = 0.03f)
    {
        std::vector<cv::Point3f> axes = {
            cv::Point3f(0, 0, 0),      // Origin
            cv::Point3f(length, 0, 0), // X-axis (should point right)
            cv::Point3f(0, length, 0), // Y-axis (should point up)
            cv::Point3f(0, 0, length)  // Z-axis (should point out of marker)
        };

        std::vector<cv::Point2f> projectedAxes;
        cv::projectPoints(axes, rvec, tvec, cameraMatrix, distCoeffs, projectedAxes);

        // Check if points are within image bounds before drawing
        cv::Size imgSize = image.size();
        auto isValidPoint = [&imgSize](const cv::Point2f &pt)
        {
            return pt.x >= 0 && pt.x < imgSize.width && pt.y >= 0 && pt.y < imgSize.height;
        };

        if (isValidPoint(projectedAxes[0]))
        {
            // Draw axes with proper colors (BGR format):
            // X=Red (right), Y=Green (up), Z=Blue (out of marker)
            if (isValidPoint(projectedAxes[1]))
            {
                cv::line(image, projectedAxes[0], projectedAxes[1], cv::Scalar(0, 0, 255), 3); // X=Red
                cv::putText(image, "X", projectedAxes[1], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 2);
            }
            if (isValidPoint(projectedAxes[2]))
            {
                cv::line(image, projectedAxes[0], projectedAxes[2], cv::Scalar(0, 255, 0), 3); // Y=Green
                cv::putText(image, "Y", projectedAxes[2], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
            }
            if (isValidPoint(projectedAxes[3]))
            {
                cv::line(image, projectedAxes[0], projectedAxes[3], cv::Scalar(255, 0, 0), 3); // Z=Blue
                cv::putText(image, "Z", projectedAxes[3], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 2);
            }

            // Draw origin point
            cv::circle(image, projectedAxes[0], 4, cv::Scalar(255, 255, 255), -1);
        }
    }

    void debugTransformChain(const MatchedMarker &marker)
    {
        if (!config_.enableDebugTrace)
            return;

        ROS_INFO("\n");
        ROS_INFO("=== TRANSFORM CHAIN DEBUG FOR MARKER %d ===", marker.id);

        // Show final stereo pose
        ROS_INFO("Final stereo pose in camera frame:");
        ROS_INFO("  Translation: [%8.3f, %8.3f, %8.3f] meters",
                 marker.tvec.at<double>(0), marker.tvec.at<double>(1), marker.tvec.at<double>(2));
        ROS_INFO("  Rotation:    [%8.3f, %8.3f, %8.3f] radians",
                 marker.rvec.at<double>(0), marker.rvec.at<double>(1), marker.rvec.at<double>(2));

        // Convert rotation to matrix for analysis
        cv::Mat rotMat;
        cv::Rodrigues(marker.rvec, rotMat);

        cv::Vec3d markerY(rotMat.at<double>(0, 1), rotMat.at<double>(1, 1), rotMat.at<double>(2, 1));
        ROS_INFO("  Marker Y-axis in camera: [%6.3f, %6.3f, %6.3f] (%s)",
                 markerY[0], markerY[1], markerY[2], markerY[1] > 0 ? "points DOWN" : "points UP");

        if (TM_Landpad_To_Aruco_.count(marker.id))
        {
            cv::Vec3f landpadPos, landpadOri;
            if (applyMarkerTransform(marker, landpadPos, landpadOri))
            {
                ROS_INFO("\nTransformed landpad pose:");
                ROS_INFO("  Translation: [%8.3f, %8.3f, %8.3f] meters",
                         landpadPos[0], landpadPos[1], landpadPos[2]);
                ROS_INFO("  Rotation:    [%8.3f, %8.3f, %8.3f] radians",
                         landpadOri[0], landpadOri[1], landpadOri[2]);
                ROS_INFO("  Rotation:    [%8.1f, %8.1f, %8.1f] degrees",
                         landpadOri[0] * 180.0 / M_PI, landpadOri[1] * 180.0 / M_PI, landpadOri[2] * 180.0 / M_PI);
            }
        }
        ROS_INFO("============================================\n");
    }

    void printStatistics()
    {
        if (config_.enablePerformance)
            return; // ADDED: Don't print stats in performance mode

        ROS_INFO("\n");
        ROS_INFO("=== STEREO ARUCO STATISTICS ===");
        ROS_INFO("Frames processed: %d", frameCounter_);
        ROS_INFO("Total markers detected: %d", totalMarkers_);
        ROS_INFO("Valid markers: %d (%.1f%%)", validMarkers_,
                 totalMarkers_ > 0 ? 100.0 * validMarkers_ / totalMarkers_ : 0.0);

        if (!statusCounts_.empty())
        {
            ROS_INFO("\nDetailed status breakdown:");
            for (const auto &status_count : statusCounts_)
            {
                MarkerStatus status = status_count.first;
                int count = status_count.second;
                if (count > 0)
                {
                    ROS_INFO("  %-40s: %4d (%.1f%%)", statusToString(status), count,
                             totalMarkers_ > 0 ? 100.0 * count / totalMarkers_ : 0.0);
                }
            }
        }
        ROS_INFO("===============================\n");
    }

}; // Class closing brace

int main(int argc, char **argv)
{
    ros::init(argc, argv, "stereo_aruco_detector_node");

    // Enhanced command line argument parsing
    bool forceNoViz = false;
    bool enableDebug = false;
    bool enableDebugTrace = false;
    bool enablePerformance = false;
    int filterMarkerId = -1; // ← MUST be declared here, before the loop

    for (int i = 1; i < argc; i++)
    {
        std::string arg = argv[i];
        if (arg == "--performance")
        {
            enablePerformance = true;
            ROS_INFO("Command line: Performance mode ENABLED (no debug output)");
        }
        else if (arg == "--debug")
        {
            enableDebug = true;
            ROS_INFO("Command line: Debug mode ENABLED (final results only)");
        }
        else if (arg == "--debug-trace")
        {
            enableDebugTrace = true;
            ROS_INFO("Command line: Debug trace mode ENABLED (full detailed output)");
        }
        else if (arg == "--no-viz")
        {
            forceNoViz = true;
            ROS_INFO("Command line: Visualization DISABLED");
        }
        else if (arg == "--marker" && i + 1 < argc)
        {
            filterMarkerId = std::stoi(argv[i + 1]);
            i++; // skip next arg
            ROS_INFO("Command line: SINGLE MARKER MODE - only processing marker %d",
                     filterMarkerId);
        }
        else if (arg == "--help")
        {
            std::cout << "Usage: " << argv[0] << " [options]\n"
                      << "  --performance    Performance mode (no debug output)\n"
                      << "  --debug          Debug mode (final results only)\n"
                      << "  --debug-trace    Debug trace mode (full detailed output)\n"
                      << "  --no-viz         Disable visualization\n"
                      << "  --marker <id>    Process only one ArUco ID\n"
                      << "  --help           Show this help\n";
            return 0;
        }
    }

    try
    {
        Config cfg;
        if (filterMarkerId != -1)
        {
            cfg.singleMarkerFilter = filterMarkerId;
            ROS_INFO("Single marker filter active: ID %d", filterMarkerId);
        }

        // Apply command line overrides
        if (enablePerformance)
        {
            cfg.enablePerformance = true;
            cfg.enableDebug = false;
            cfg.enableDebugTrace = false;
            cfg.enableVisualization = false;
        }
        else if (enableDebugTrace)
        {
            cfg.enableDebugTrace = true;
            cfg.enableDebug = true; // Trace includes debug
        }
        else if (enableDebug)
        {
            cfg.enableDebug = true;
        }

        if (forceNoViz)
        {
            cfg.enableVisualization = false;
        }

        StereoArucoDetectorNode node(cfg);
        node.run();
    }
    catch (const std::exception &e)
    {
        ROS_FATAL("Exception in main: %s", e.what());
        return -1;
    }

    return 0;
}
