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
    Manager();
    ~Manager();
    void Init(ROSClient *rosClient, DroneControl *droneControl);

    void print_parameters();
    void update();

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void joyCallback(const sensor_msgs::Joy::ConstPtr &msg);
    void parametersCallback(const iris_land::controllers_gain::ConstPtr &msg);
    void imuCallback(const sensor_msgs::Imu::ConstPtr &msg);

private:
    sensor_msgs::Joy joy;
    sensor_msgs::Imu imu;
    nav_msgs::Odometry odom;
    geometry_msgs::PoseStamped pose;
    iris_land::controllers_gain parameters;

    ROSClient *ROS_client;
    DroneControl *drone_control;
    
    State_Machine state_machine;
    Land_Controller land_controller;
    Follow_Controller follow_controller;

    void STOPPED_action();
    void TAKE_OFF_action();
    void LAND_action();
    void JOY_CONTROL_action();
    void LAND_CONTROL_action();
    void FOLLOW_CONTROL_action();
    void send_velocity(double x_linear, double y_linear, double z_linear, double angular);

    // joy parameters
    ros::Time joy_last_timestamp;
    double joy_linear_velocity;
    double joy_angular_velocity;
};

#endif // MANAGER_H
