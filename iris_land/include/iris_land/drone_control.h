#ifndef DRONE_CONTROL_H
#define DRONE_CONTROL_H

#include "general.h"
#include "ros_client.h"

class DroneControl
{
public:
    DroneControl();
    ~DroneControl();
    void Init(ROSClient *drone_control);
    void Setup();

    static constexpr float TAKEOFF_ALTITUDE = 5.0;
    static constexpr float ROS_RATE = 20.0;
    static constexpr int MAX_ATTEMPTS = 100;
    static constexpr double LAT_DEG_TO_M = 111000.0;
    static constexpr double LON_DEG_TO_M = 75000.0;

    // The setpoint publishing rate MUST be faster than 2Hz
    ros::Rate *rate_;
    tf2_ros::Buffer tfBuffer_;

    mavros_msgs::State current_state_;
    geometry_msgs::PoseStamped local_position_;
    sensor_msgs::NavSatFix global_position_;
    geometry_msgs::TransformStamped transformStamped_;

    void state_cb(const mavros_msgs::State::ConstPtr &msg);
    void extended_state_cb(const mavros_msgs::ExtendedState::ConstPtr &msg);
    void local_position_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void global_position_cb(const sensor_msgs::NavSatFix::ConstPtr &msg);

    void flyToGlobal(double latitude, double longitude, double altitude, double yaw);
    void flyToLocal(double x, double y, double z, double yaw);
    void hover(double seconds);
    void live_signal();

    void cmd_vel(double x, double y, double z, double ang);
    void cmd_vel_unstamped(double x, double y, double z, double ang);
    void cmd_vel_base_link(double x, double y, double z, double ang);

    void set_offboardMode();
    void await_offboardMode();
    void takeOff();
    void land();
    void disarm();

    string get_flight_mode();
    int get_landed_state();

private:
    uint8_t landed_state_ = 0;
    geometry_msgs::PoseStamped setpoint_pos_ENU_;
    geometry_msgs::PoseStamped gps_init_pos_;
    ros::Time last_request_;
    mavros_msgs::CommandBool arm_cmd_;
    ROSClient *ros_client_;

    double currentYaw();
    double distance(const geometry_msgs::PoseStamped &p1, const geometry_msgs::PoseStamped &p2);
};

#endif // DRONE_CONTROL_H
