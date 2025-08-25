#ifndef MANAGER_H
#define MANAGER_H

#include "general.h"
#include "state_machine.h"
#include "follow_controller.h"
#include "land_controller.h"
#include "ros_client.h"

class Manager
{
public:
    Manager(const rclcpp::Node::SharedPtr &node) : nh_(node) {}
    ~Manager();
    void Init(ROSClient *rosClient, DroneControl *droneControl);

    void print_parameters();
    void update();

    void arucoPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void rcCallback(const mavros_msgs::msg::RCIn::SharedPtr msg);
    void parametersCallback(const iris_land_msgs::msg::ControllersGain::SharedPtr msg);

private:
    rclcpp::Node::SharedPtr nh_;
    mavros_msgs::msg::RCIn rc_status;
    geometry_msgs::msg::PoseStamped aruco_pose;
    iris_land_msgs::msg::ControllersGain parameters;

    ROSClient *ROS_client;
    DroneControl *drone_control;

    State_Machine state_machine;
    Land_Controller land_controller;
    Follow_Controller follow_controller;

    void STOPPED_action(std::stringstream &ss);
    void LAND_CONTROL_action(std::stringstream &ss);
    void FOLLOW_CONTROL_action(std::stringstream &ss);
    void AWAITING_MODE_action(std::stringstream &ss);
    void send_velocity(double x_linear, double y_linear, double z_linear, double angular);
};

#endif // MANAGER_H
