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
    ros::Subscriber joy_sub;
    ros::Subscriber parameters_sub;

private:
    ros::NodeHandle *nh;
};

#endif /* ROS_CLIENT_H */
