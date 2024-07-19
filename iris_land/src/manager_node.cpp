#include "general.h"
#include "manager.h"
#include "ros_client.h"
#include "drone_control.h"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "manager_control");
  ros::NodeHandle *nh = new ros::NodeHandle();
  ros::Rate loop_rate(20);

  Manager manager;
  DroneControl drone_control;
  ROSClient ros_client(nh);

  ros_client.Init(&manager, &drone_control);
  manager.Init(&ros_client);
  drone_control.Init(&ros_client);

  while(ros::ok()){
    ros::spinOnce();
    manager.print_parameters();
    manager.update();
    
    loop_rate.sleep();
  }
  delete nh;
}