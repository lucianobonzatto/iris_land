#ifndef ROS_CLIENT_H
#define ROS_CLIENT_H

#include "general.h"

class Manager;
class DroneControl;
class ROSClient
{
public:
    ROSClient(const rclcpp::Node::SharedPtr &node) : nh_(node) {}
    void init(Manager *const manager, DroneControl *const drone_control);
    void setParam(const std::string &key, double d);

    rclcpp::Subscription<mavros_msgs::msg::RCIn>::SharedPtr rc_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub;
    rclcpp::Subscription<iris_land_msgs::msg::ControllersGain>::SharedPtr parameters_sub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub;

    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Subscription<mavros_msgs::msg::ExtendedState>::SharedPtr extended_state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr global_pos_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_pos_sub_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::Publisher<geographic_msgs::msg::GeoPoseStamped>::SharedPtr global_setpoint_pos_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_pos_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_pub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_unstamped_pub;

    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr land_client_;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;

    rclcpp::Node::SharedPtr nh_;
};

#endif /* ROS_CLIENT_H */
