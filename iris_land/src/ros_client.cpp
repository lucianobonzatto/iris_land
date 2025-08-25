#include "ros_client.h"
#include "manager.h"
#include "drone_control.h"

void ROSClient::init(Manager *const manager, DroneControl *const drone_control)
{
    // Manager parameters
    rc_sub = nh_->create_subscription<mavros_msgs::msg::RCIn>("/mavros/rc/in",
                                                              rclcpp::QoS(1).best_effort(),
                                                              std::bind(&Manager::rcCallback, manager, std::placeholders::_1));
    pose_sub = nh_->create_subscription<geometry_msgs::msg::PoseStamped>("/aruco/pose",
                                                                         rclcpp::QoS(10).best_effort(),
                                                                         std::bind(&Manager::arucoPoseCallback, manager, std::placeholders::_1));
    parameters_sub = nh_->create_subscription<iris_land_msgs::msg::ControllersGain>("/PID/parameters",
                                                                                    rclcpp::QoS(1).best_effort(),
                                                                                    std::bind(&Manager::parametersCallback, manager, std::placeholders::_1));

    status_pub = nh_->create_publisher<std_msgs::msg::String>("/controller/status", 10);

    // DroneControl parameters
    state_sub_ = nh_->create_subscription<mavros_msgs::msg::State>("/mavros/state",
                                                                   rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local(),
                                                                   std::bind(&DroneControl::state_cb, drone_control, std::placeholders::_1));
    extended_state_sub_ = nh_->create_subscription<mavros_msgs::msg::ExtendedState>("/mavros/extended_state",
                                                                                    rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local(),
                                                                                    std::bind(&DroneControl::extended_state_cb, drone_control, std::placeholders::_1));
    local_pos_sub_ = nh_->create_subscription<geometry_msgs::msg::PoseStamped>("/mavros/local_position/pose",
                                                                               rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
                                                                               std::bind(&DroneControl::local_position_cb, drone_control, std::placeholders::_1));
    global_pos_sub_ = nh_->create_subscription<sensor_msgs::msg::NavSatFix>("/mavros/global_position/global",
                                                                            rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
                                                                            std::bind(&DroneControl::global_position_cb, drone_control, std::placeholders::_1));

    global_setpoint_pos_pub_ = nh_->create_publisher<geographic_msgs::msg::GeoPoseStamped>("/mavros/setpoint_position/global", 10);
    setpoint_pos_pub_ = nh_->create_publisher<geometry_msgs::msg::PoseStamped>("/mavros/setpoint_position/local", 10);
    velocity_pub = nh_->create_publisher<geometry_msgs::msg::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);
    velocity_unstamped_pub = nh_->create_publisher<geometry_msgs::msg::Twist>("/mavros/setpoint_velocity/cmd_vel_unstamped", 10);

    arming_client_ = nh_->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
    land_client_ = nh_->create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/land");
    takeoff_client_ = nh_->create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/takeoff");
    set_mode_client_ = nh_->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
}

void ROSClient::setParam(const std::string &key, double d)
{
    nh_->set_parameter(rclcpp::Parameter(key, d));
}
