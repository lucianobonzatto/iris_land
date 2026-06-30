#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <iomanip>
#include <sstream>

int main(int argc, char** argv) {
    ros::init(argc, argv, "stereo_camera_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    int width = 1600;
    int height = 600;
    int fps = 120;
    bool enable_timing_profile = true;
    pnh.param("enable_timing_profile", enable_timing_profile, true);

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
    ROS_INFO("Publicando imagens estéreo como sensor_msgs::Image");
    uint32_t frame_sequence = 0;

    while (ros::ok()) {
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
        sensor_msgs::ImagePtr right_msg = cv_right.toImageMsg();
        auto t_message = std::chrono::steady_clock::now();

        pub_left.publish(left_msg);
        auto t_left_pub = std::chrono::steady_clock::now();
        ros::Time left_publish_complete_stamp = ros::Time::now();
        pub_right.publish(right_msg);

        auto t_pub = std::chrono::steady_clock::now();
        ros::Time right_publish_complete_stamp = ros::Time::now();

        if (enable_timing_profile) {
            const auto ms = [](const std::chrono::steady_clock::duration& duration) {
                return std::chrono::duration<double, std::milli>(duration).count();
            };
            std::ostringstream ss;
            ss << std::fixed << std::setprecision(9)
               << "{"
               << "\"stamp\":" << stamp.toSec() << ","
               << "\"sequence\":" << frame_sequence << ","
               << "\"left_publish_complete_stamp\":" << left_publish_complete_stamp.toSec() << ","
               << "\"right_publish_complete_stamp\":" << right_publish_complete_stamp.toSec() << ","
               << "\"publish_complete_stamp\":" << right_publish_complete_stamp.toSec() << ","
               << "\"capture_read_ms\":" << ms(t_read - t_start) << ","
               << "\"stereo_split_ms\":" << ms(t_split - t_read) << ","
               << "\"image_message_build_ms\":" << ms(t_message - t_split) << ","
               << "\"left_publish_call_ms\":" << ms(t_left_pub - t_message) << ","
               << "\"right_publish_call_ms\":" << ms(t_pub - t_left_pub) << ","
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
