#ifndef ROS_CLIENT_H
#define ROS_CLIENT_H

#include "general.h"

class Manager;
class DroneControl;
class ROSClient
{
public:
    ROSClient(ros::NodeHandle *handle);
    void Init(Manager *const manager, DroneControl *const drone_control);
    void setParam(const std::string &key, double d);

    ros::Subscriber pose_sub;
    ros::Subscriber rc_sub;
    ros::Subscriber parameters_sub;
    ros::Publisher status_pub;

    ros::Subscriber state_sub_;
    ros::Subscriber extended_state_sub_;
    ros::Subscriber local_pos_sub_;
    ros::Subscriber global_pos_sub_;

    ros::Publisher global_setpoint_pos_pub_;
    ros::Publisher setpoint_pos_pub_;
    ros::Publisher velocity_pub;
    ros::Publisher velocity_unstamped_pub;

    ros::ServiceClient arming_client_;
    ros::ServiceClient land_client_;
    ros::ServiceClient takeoff_client_;
    ros::ServiceClient set_mode_client_;

private:
    ros::NodeHandle *nh;
};

#endif /* ROS_CLIENT_H */
