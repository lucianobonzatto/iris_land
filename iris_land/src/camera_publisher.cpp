#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <cctype>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <string>

int main(int argc, char** argv) {
    ros::init(argc, argv, "stereo_camera_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    int width = 1600;
    int height = 600;
    int fps = 120;
    bool enable_timing_profile = true;
    bool private_flight_optimized = false;
    std::string pose_estimation_mode = "stereo";
    pnh.param("enable_timing_profile", enable_timing_profile, true);
    pnh.param("flight_optimized", private_flight_optimized, false);
    pnh.param("pose_estimation_mode", pose_estimation_mode, pose_estimation_mode);

    bool runtime_flight_optimized = private_flight_optimized;
    bool runtime_timing_profile_enabled = enable_timing_profile;
    std::string last_reported_pose_estimation_mode;
    auto refresh_runtime_config = [&]() {
        bool shared_flight_optimized = false;
        bool shared_timing_profile_enabled = true;
        std::string shared_pose_mode;
        if (nh.getParam("/aruco_runtime/flight_optimized", shared_flight_optimized)) {
            runtime_flight_optimized =
                private_flight_optimized || shared_flight_optimized;
        } else {
            runtime_flight_optimized = private_flight_optimized;
        }
        if (nh.getParam(
                "/aruco_runtime/enable_timing_profile",
                shared_timing_profile_enabled)) {
            runtime_timing_profile_enabled =
                enable_timing_profile && shared_timing_profile_enabled;
        } else {
            runtime_timing_profile_enabled = enable_timing_profile;
        }
        if (nh.getParam("/aruco_runtime/pose_estimation_mode", shared_pose_mode)) {
            pose_estimation_mode = shared_pose_mode;
        }
        std::transform(
            pose_estimation_mode.begin(), pose_estimation_mode.end(),
            pose_estimation_mode.begin(),
            [](unsigned char value) {
                return static_cast<char>(std::tolower(value));
            }
        );
        if (pose_estimation_mode != "monocular") {
            pose_estimation_mode = "stereo";
        }
        if (pose_estimation_mode != last_reported_pose_estimation_mode) {
            ROS_INFO(
                "Camera publisher pose mode: %s (%s)",
                pose_estimation_mode.c_str(),
                pose_estimation_mode == "monocular"
                    ? "publishing left image only"
                    : "publishing left and right images"
            );
            last_reported_pose_estimation_mode = pose_estimation_mode;
        }
    };
    refresh_runtime_config();

    ros::Publisher pub_left = nh.advertise<sensor_msgs::Image>("/stereo/left/image_raw", 10);
    ros::Publisher pub_right = nh.advertise<sensor_msgs::Image>("/stereo/right/image_raw", 10);
    ros::Publisher timing_pub;
    if (enable_timing_profile) {
        timing_pub = nh.advertise<std_msgs::String>("/stereo/debug/timing", 100);
    }

    // Abrir câmera com backend V4L2
    cv::VideoCapture cap(0, cv::CAP_V4L2);
    if (!cap.isOpened()) {
        ROS_ERROR("Erro ao abrir /dev/video0");
        return -1;
    }

    // Definir propriedades
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
    cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);
    cap.set(cv::CAP_PROP_FPS, fps);

    // Verificar configurações reais
    ROS_INFO_STREAM("Resolução real: "
        << cap.get(cv::CAP_PROP_FRAME_WIDTH) << "x"
        << cap.get(cv::CAP_PROP_FRAME_HEIGHT));
    ROS_INFO_STREAM("FPS real: " << cap.get(cv::CAP_PROP_FPS));

    cv_bridge::CvImage cv_left, cv_right;
    cv_left.encoding = "bgr8";
    cv_right.encoding = "bgr8";

    ros::Rate loop_rate(fps);
    ROS_INFO("Publishing camera images as sensor_msgs::Image");
    uint32_t frame_sequence = 0;
    ros::WallTime last_runtime_config_refresh;

    while (ros::ok()) {
        const ros::WallTime wall_now = ros::WallTime::now();
        if (
            last_runtime_config_refresh.isZero() ||
            (wall_now - last_runtime_config_refresh).toSec() >= 1.0
        ) {
            refresh_runtime_config();
            last_runtime_config_refresh = wall_now;
        }
        const bool publish_right_image =
            pose_estimation_mode == "stereo";
        const bool timing_profile_active =
            runtime_timing_profile_enabled && !runtime_flight_optimized;

        auto t_start = std::chrono::steady_clock::now();

        cv::Mat frame;
        if (!cap.read(frame)) {
            ROS_WARN("Falha ao capturar frame");
            continue;
        }

        auto t_read = std::chrono::steady_clock::now();
        // This is the earliest reliable userspace timestamp available from the
        // current V4L2/OpenCV path: the complete stereo frame has just arrived.
        ros::Time stamp = ros::Time::now();

        // Dividir imagem estéreo
        int halfWidth = frame.cols / 2;
        cv::Mat left = frame(cv::Rect(0, 0, halfWidth, frame.rows));
        cv::Mat right = frame(cv::Rect(halfWidth, 0, halfWidth, frame.rows));

        auto t_split = std::chrono::steady_clock::now();

        cv_left.header.seq = frame_sequence;
        cv_left.header.stamp = stamp;
        cv_left.header.frame_id = "stereo_left_camera";
        cv_left.image = left;
        cv_right.header.seq = frame_sequence;
        cv_right.header.stamp = stamp;
        cv_right.header.frame_id = "stereo_right_camera";
        cv_right.image = right;

        sensor_msgs::ImagePtr left_msg = cv_left.toImageMsg();
        sensor_msgs::ImagePtr right_msg;
        if (publish_right_image) {
            right_msg = cv_right.toImageMsg();
        }
        auto t_message = std::chrono::steady_clock::now();

        pub_left.publish(left_msg);
        auto t_left_pub = std::chrono::steady_clock::now();
        ros::Time left_publish_complete_stamp = ros::Time::now();
        auto t_pub = t_left_pub;
        ros::Time right_publish_complete_stamp;
        ros::Time publish_complete_stamp = left_publish_complete_stamp;
        double right_publish_call_ms = 0.0;
        if (publish_right_image) {
            pub_right.publish(right_msg);
            t_pub = std::chrono::steady_clock::now();
            right_publish_complete_stamp = ros::Time::now();
            publish_complete_stamp = right_publish_complete_stamp;
            right_publish_call_ms =
                std::chrono::duration<double, std::milli>(
                    t_pub - t_left_pub
                ).count();
        }

        if (timing_profile_active) {
            const auto ms = [](const std::chrono::steady_clock::duration& duration) {
                return std::chrono::duration<double, std::milli>(duration).count();
            };
            std::ostringstream ss;
            ss << std::fixed << std::setprecision(9)
               << "{"
               << "\"stamp\":" << stamp.toSec() << ","
               << "\"sequence\":" << frame_sequence << ","
               << "\"pose_estimation_mode\":\"" << pose_estimation_mode << "\","
               << "\"left_publish_complete_stamp\":" << left_publish_complete_stamp.toSec() << ","
               << "\"right_publish_complete_stamp\":" << right_publish_complete_stamp.toSec() << ","
               << "\"publish_complete_stamp\":" << publish_complete_stamp.toSec() << ","
               << "\"capture_read_ms\":" << ms(t_read - t_start) << ","
               << "\"stereo_split_ms\":" << ms(t_split - t_read) << ","
               << "\"image_message_build_ms\":" << ms(t_message - t_split) << ","
               << "\"left_publish_call_ms\":" << ms(t_left_pub - t_message) << ","
               << "\"right_publish_call_ms\":" << right_publish_call_ms << ","
               << "\"camera_pipeline_ms\":" << ms(t_pub - t_read) << ","
               << "\"requested_fps\":" << fps << ","
               << "\"reported_fps\":" << cap.get(cv::CAP_PROP_FPS)
               << "}";
            std_msgs::String timing_msg;
            timing_msg.data = ss.str();
            timing_pub.publish(timing_msg);
        }
        ++frame_sequence;

        ros::spinOnce();
        loop_rate.sleep();

    }

    cap.release();
    return 0;
}
