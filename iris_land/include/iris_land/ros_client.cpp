#include "ros_client.h"
#include "manager.h"
#include "drone_control.h"

ROSClient::ROSClient(ros::NodeHandle *handle)
{
    this->nh = handle;
}

void ROSClient::Init(Manager *const manager, DroneControl *const drone_control)
{
    joy_sub = nh->subscribe<sensor_msgs::Joy>("/joy_control", 1, &Manager::joyCallback, manager);
    pose_sub = nh->subscribe<geometry_msgs::PoseStamped>("/aruco/pose", 10, &Manager::arucoPoseCallback, manager);
    parameters_sub = nh->subscribe<iris_land::controllers_gain>("/PID/parameters", 1, &Manager::parametersCallback, manager);
}

void ROSClient::setParam(const std::string &key, double d)
{
    nh->setParam(key, d);
}
