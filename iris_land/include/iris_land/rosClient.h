#ifndef ROS_CLIENT_H
#define ROS_CLIENT_H

#include "general.h"

class Manager; // Forward declaration because of circular reference

class ROSClient
{
public:
  ROSClient(ros::NodeHandle *handle);
  void init(Manager *const manager);
  void setParam(const std::string &key, double d);

  ros::Publisher takeoff_pub;
  ros::Publisher land_pub;
  ros::Publisher cmd_vel_pub;

  ros::Subscriber pose_sub;
  ros::Subscriber odom_sub;
  ros::Subscriber joy_sub;
  ros::Subscriber parameters_sub;
  ros::Subscriber imu_sub;
  ros::Subscriber status_sub;

private:
  ros::NodeHandle *nh;
};

#endif /* ROS_CLIENT_H */
