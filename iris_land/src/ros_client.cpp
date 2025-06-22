#include "ros_client.h"
#include "manager.h"
#include "drone_control.h"

ROSClient::ROSClient(ros::NodeHandle *handle)
{
    this->nh = handle;
}

void ROSClient::Init(Manager *const manager, DroneControl *const drone_control)
{
// Manager parameters
    rc_sub = nh->subscribe<mavros_msgs::RCIn>("/mavros/rc/in", 1, &Manager::rcCallback, manager);
    pose_sub = nh->subscribe<geometry_msgs::PoseStamped>("/aruco/pose", 10, &Manager::arucoPoseCallback, manager);
    parameters_sub = nh->subscribe<iris_land::controllers_gain>("/PID/parameters", 1, &Manager::parametersCallback, manager);
    status_pub = nh->advertise<std_msgs::String>("/controller/status", 10);

// DroneControl parameters
    state_sub_ = nh->subscribe<mavros_msgs::State>("/mavros/state", 10, &DroneControl::state_cb, drone_control);
    extended_state_sub_ = nh->subscribe<mavros_msgs::ExtendedState>("/mavros/extended_state", 10, &DroneControl::extended_state_cb, drone_control);
    local_pos_sub_ = nh->subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, &DroneControl::local_position_cb, drone_control);
    global_pos_sub_ = nh->subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/global", 10, &DroneControl::global_position_cb, drone_control);

    global_setpoint_pos_pub_ = nh->advertise<geographic_msgs::GeoPoseStamped>("/mavros/setpoint_position/global", 10);
    setpoint_pos_pub_ = nh->advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
    velocity_pub = nh->advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);
    velocity_unstamped_pub = nh->advertise<geometry_msgs::Twist>("/mavros/setpoint_velocity/cmd_vel_unstamped", 10);

    arming_client_ = nh->serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    land_client_ = nh->serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
    takeoff_client_ = nh->serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
    set_mode_client_ = nh->serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
}

void ROSClient::setParam(const std::string &key, double d)
{
    nh->setParam(key, d);
}
