#include "general.h"
#include "manager.h"
#include "ros_client.h"
#include "drone_control.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("manager_control");
    rclcpp::Rate loop_rate(20);

    Manager manager(node);
    ROSClient ros_client(node);
    DroneControl drone_control(node, &ros_client);

    manager.Init(&ros_client, &drone_control);

    ros_client.init(&manager, &drone_control);
    drone_control.Setup();

    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
        manager.print_parameters();
        manager.update();

        loop_rate.sleep();
    }

    return 0;
}