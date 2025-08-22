#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <chrono>

int main(int argc, char** argv) {
    ros::init(argc, argv, "stereo_camera_node");
    ros::NodeHandle nh;

    int width = 1280;
    int height = 480;
    int fps = 120;

    ros::Publisher pub_left = nh.advertise<sensor_msgs::Image>("/stereo/left/image_raw", 10);
    ros::Publisher pub_right = nh.advertise<sensor_msgs::Image>("/stereo/right/image_raw", 10);

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

    while (ros::ok()) {
        auto t_start = std::chrono::high_resolution_clock::now();

        cv::Mat frame;
        if (!cap.read(frame)) {
            ROS_WARN("Falha ao capturar frame");
            continue;
        }

        auto t_read = std::chrono::high_resolution_clock::now();

        // Dividir imagem estéreo
        int halfWidth = frame.cols / 2;
        cv::Mat left = frame(cv::Rect(0, 0, halfWidth, frame.rows));
        cv::Mat right = frame(cv::Rect(halfWidth, 0, halfWidth, frame.rows));

        auto t_split = std::chrono::high_resolution_clock::now();

        ros::Time stamp = ros::Time::now();
        cv_left.header.stamp = stamp;
        cv_left.image = left;
        cv_right.header.stamp = stamp;
        cv_right.image = right;

        auto t_cvbridge = std::chrono::high_resolution_clock::now();

        pub_left.publish(cv_left.toImageMsg());
        pub_right.publish(cv_right.toImageMsg());

        auto t_pub = std::chrono::high_resolution_clock::now();

        ros::spinOnce();
        loop_rate.sleep();

        auto t_end = std::chrono::high_resolution_clock::now();

        /*std::cout << "Tempo leitura: " << std::chrono::duration<double>(t_read - t_start).count() << "s\n";
        std::cout << "Tempo split: " << std::chrono::duration<double>(t_split - t_read).count() << "s\n";
        std::cout << "Tempo cv_bridge: " << std::chrono::duration<double>(t_cvbridge - t_split).count() << "s\n";
        std::cout << "Tempo publish: " << std::chrono::duration<double>(t_pub - t_cvbridge).count() << "s\n";
        std::cout << "Tempo total: " << std::chrono::duration<double>(t_end - t_start).count() << "s\n";*/
    }

    cap.release();
    return 0;
}
