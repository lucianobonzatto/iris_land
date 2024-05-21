#include "rosClient.h"
#include "manager.h"

ROSClient::ROSClient(ros::NodeHandle *handle)
{
  this->nh = handle;
}

void ROSClient::init(Manager *const manager)
{
  land_pub = nh->advertise<std_msgs::Empty>("/tello/land", 1);
  takeoff_pub = nh->advertise<std_msgs::Empty>("/tello/takeoff", 1);
  cmd_vel_pub = nh->advertise<geometry_msgs::Twist>("/tello/cmd_vel", 1);

  imu_sub = nh->subscribe<sensor_msgs::Imu>("/tello/imu", 1, &Manager::imuCallback, manager);
  joy_sub = nh->subscribe<sensor_msgs::Joy>("/joy_control", 1, &Manager::joyCallback, manager);
  odom_sub = nh->subscribe<nav_msgs::Odometry>("/tello/odom", 1, &Manager::odomCallback, manager);
  pose_sub = nh->subscribe<geometry_msgs::PoseStamped>("/aruco/pose", 10, &Manager::poseCallback, manager);
  parameters_sub = nh->subscribe<iris_land::controllers_gain>("/PID/parameters", 1, &Manager::parametersCallback, manager);
}

void ROSClient::setParam(const std::string &key, double d)
{
  nh->setParam(key, d);
}
