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
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/NavSatFix.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/GlobalPositionTarget.h>

#include <geographic_msgs/GeoPoseStamped.h>

#include <iris_land/controllers_gain.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// #include <tf2_ros/transform_broadcaster.h>
// #include <tf2_ros/transform_listener.h>

using namespace std;

#define TAG_ID 0

enum JOY_BUTTONS
{
    BUTTON_A,
    BUTTON_B,
    BUTTON_X,
    BUTTON_Y,
    BUTTON_LB,
    BUTTON_RB,
    BUTTON_LT,
    BUTTON_RT,
    BUTTON_BACK,
    BUTTON_START,
    BUTTON_MAIN,
    BUTTON_ANALOGIC_LEFT,
    BUTTON_ANALOGIC_RIGHT,
    BUTTON_ARROR_UP,
    BUTTON_ARROR_DOWN,
    BUTTON_ARROR_LEFT,
    BUTTON_ARROR_RIGHT,
};

enum JOY_AXES
{
    AXE_HORIZONTAL_ANALOGIC_LEFT,
    AXE_VERTICAL_ANALOGIC_LEFT,
    AXE_LT,
    AXE_HORIZONTAL_ANALOGIC_RIGHT,
    AXE_VERTICAL_ANALOGIC_RIGHT,
    AXE_RT,
};

enum STATES
{
    STOPPED = 0,
    TAKE_OFF,
    LAND,
    JOY_CONTROL,
    LAND_CONTROL,
    FOLLOW_CONTROL
};

static std::string states_name[6] = {
    "STOPPED",
    "TAKE_OFF",
    "LAND",
    "JOY_CONTROL",
    "LAND_CONTROL",
    "FOLLOW_CONTROL"
};

enum CONTROLERS
{
    _NENHUM = 0,
    _PD,
    _CASCADE,
    _PARALLEL,
    _PID
};

static std::string controlers_name[6] = {
    "NENHUM",
    "PD",
    "CASCADE",
    "PARALLEL",
    "PID"
};

#endif