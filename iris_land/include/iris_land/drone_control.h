#ifndef DRONE_CONTROL_H
#define DRONE_CONTROL_H

#include "general.h"

class ROSClient;
class DroneControl
{
public:
    DroneControl(const rclcpp::Node::SharedPtr &node, ROSClient *ros_client);
    void Setup();

    static constexpr float TAKEOFF_ALTITUDE = 5.0;
    static constexpr float ROS_RATE = 20.0;
    static constexpr int MAX_ATTEMPTS = 100;
    static constexpr double LAT_DEG_TO_M = 111000.0;
    static constexpr double LON_DEG_TO_M = 75000.0;

    // The setpoint publishing rate MUST be faster than 2Hz
    rclcpp::Node::SharedPtr nh_;
    ROSClient *ros_client_;
    rclcpp::Rate *rate_;
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;
    tf2_ros::TransformBroadcaster br_;

    mavros_msgs::msg::State current_state_;
    geometry_msgs::msg::PoseStamped local_position_;
    sensor_msgs::msg::NavSatFix global_position_;
    geometry_msgs::msg::TransformStamped transformStamped_;
    uint8_t landed_state_;

    void state_cb(const mavros_msgs::msg::State::SharedPtr msg);
    void extended_state_cb(const mavros_msgs::msg::ExtendedState::SharedPtr msg);
    void local_position_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void global_position_cb(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

    void flyToGlobal(double latitude, double longitude, double altitude, double yaw);
    void flyToLocal(double x, double y, double z, double yaw);
    void hover(double seconds);
    void live_signal();

    void cmd_vel(double x, double y, double z, double ang);
    void cmd_vel_unstamped(double x, double y, double z, double ang);
    void cmd_vel_base_link(double x, double y, double z, double ang);

    void await_OFFBOARD_Mode();
    void set_OFFBOARD_Mode();

    void takeOff();
    void land();
    void disarm();
    void arm();

    string get_flight_mode();
    int get_landed_state();
    
private:
    geometry_msgs::msg::PoseStamped setpoint_pos_ENU_;
    geometry_msgs::msg::PoseStamped gps_init_pos_;

    rclcpp::Time last_request_;

    mavros_msgs::srv::CommandBool arm_cmd_;

    double currentYaw();
    double distance(const geometry_msgs::msg::PoseStamped &p1, const geometry_msgs::msg::PoseStamped &p2);
};

#endif /* DRONE_CONTROL_H */
