#ifndef CONFIG
#define CONFIG

#include <thread>
#include <iostream>
#include <unistd.h>
#include <string>
#include <ros/ros.h>
#include <tf/tf.h>
#include <cmath>

#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <nav_msgs/Odometry.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/RCIn.h>

#include <geographic_msgs/GeoPoseStamped.h>

#include <iris_land/controllers_gain.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

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

#endif