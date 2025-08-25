#ifndef CONFIG
#define CONFIG

#include <thread>
#include <iostream>
#include <unistd.h>
#include <string>
#include <cmath>
#include <rclcpp/rclcpp.hpp>

#include <iris_land_msgs/msg/controllers_gain.hpp>

// Mensagens padrão ROS 2
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

// MAVROS
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/extended_state.hpp>
#include <mavros_msgs/msg/global_position_target.hpp>
#include <mavros_msgs/msg/rc_in.hpp>

#include <geographic_msgs/msg/geo_pose_stamped.hpp>

// TF2
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std;

#define CORRECT_LAND_STATE (2)
#define CORRECT_FLIGHT_MODE "OFFBOARD"

static std::string states_name[6] = {
    "STOPPED",
    "LAND",
    "LAND_CONTROL",
    "FOLLOW_CONTROL",
    "AWAITING_MODE"};

enum STATES
{
    STOPPED = 0,
    LAND,
    LAND_CONTROL,
    FOLLOW_CONTROL,
    AWAITING_MODE
};

enum RC_CHANNELS
{
    CHANNEL_1 = 0,
    CHANNEL_2,
    CHANNEL_3,
    CHANNEL_4,
    CHANNEL_5,
    CHANNEL_6,
    STATE_KEY,
    CHANNEL_8
};

enum KEY_POSITION
{
    OUT = 0,
    P1,
    P2,
    P3
};

enum STATE_KEY_LIMITS
{
    MAX       = 2500,
    MID_MAX   = 1750,
    MID_MIN   = 1238,
    MIN       = 0
};

#define IDENTIFY_STATE_KEY_POSITION(value)          \
    ((value >  MIN      && value <= MID_MIN) ? P1 : \
     (value >  MID_MIN  && value <= MID_MAX) ? P2 : \
     (value >  MID_MAX  && value <= MAX)     ? P3 : OUT)

double get_yaw(const geometry_msgs::msg::Quaternion& quaternion);

#endif