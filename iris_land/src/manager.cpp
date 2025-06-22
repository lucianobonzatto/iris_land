#include "manager.h"
#include "drone_control.h"

Manager::Manager()
{
}

Manager::~Manager()
{
}

void Manager::Init(ROSClient *rosClient, DroneControl *droneControl)
{
    parameters.linear_vel = 0.1;
    parameters.angular_vel = 0.1;

    ROS_client = rosClient;
    drone_control = droneControl;
}

void Manager::print_parameters()
{
    std::stringstream ss;

    ss << "================\n";

    ss << "Aruco Pose:\n";
    ss << "\tstamp: " << aruco_pose.header.stamp << "\n";
    ss << "\tx: " << aruco_pose.pose.position.x << "\n"
       << "\ty: " << aruco_pose.pose.position.y << "\n"
       << "\tz: " << aruco_pose.pose.position.z << "\n"
       << "\ttheta: " << get_yaw(aruco_pose.pose.orientation) << "\n";

    ss << "Parameters:\n";
    ss << "\tlinear_vel: " << parameters.linear_vel << "\n";
    ss << "\tangular_vel: " << parameters.angular_vel << "\n";

    ss << "State:\n";
    ss << "\tcontrol_state: " << states_name[state_machine.get_state()] << "\n";
    ss << "\tflight_mode:   " << drone_control->get_flight_mode() << "\n";
    ss << "\tlanded_state: " << drone_control->get_landed_state() << "\n";

    ss << "RC Status:\n";
    ss << "\tstamp: " << rc_status.header.stamp << "\n";
    ss << "\tconnected: " << (int) drone_control->current_state_.connected << "\n";

    follow_controller.append_parameters(ss);
    land_controller.append_parameters(ss);

    ROS_INFO_STREAM(ss.str());
}

void Manager::update()
{
    std::stringstream ss;
    ss << "======== EXECUTION ========\n";

    STATES state = state_machine.get_state();
    switch (state)
    {
    case STATES::STOPPED:
        STOPPED_action(ss);
        break;
    case STATES::LAND_CONTROL:
        LAND_CONTROL_action(ss);
        break;
    case STATES::FOLLOW_CONTROL:
        FOLLOW_CONTROL_action(ss);
        break;
    case STATES::AWAITING_MODE:
        AWAITING_MODE_action(ss);
        break;
    default:
        break;
    }

    follow_controller.update_parameters(parameters);
    land_controller.update_parameters(parameters);

    // string flight_mode = CORRECT_FLIGHT_MODE;
    string flight_mode = drone_control->get_flight_mode();

    // uint8_t landed_state = CORRECT_LAND_STATE;
    uint8_t landed_state = drone_control->get_landed_state();

    if (state_machine.update_state(rc_status, flight_mode, landed_state))
    {
        send_velocity(0, 0, 0, 0);
        land_controller.reset_altitude(2);
    }
    ROS_INFO_STREAM(ss.str());
}

void Manager::STOPPED_action(std::stringstream& ss)
{
    ss << "Velocity:\n";
    ss << "\tX:\t" << 0 << "\n";
    ss << "\tY:\t" << 0 << "\n";
    ss << "\tZ:\t" << 0 << "\n";
    ss << "\tYaw:\t" << 0 << "\n";
    send_velocity(0, 0, 0, 0);
}

void Manager::LAND_CONTROL_action(std::stringstream& ss)
{
    geometry_msgs::Twist velocity;

    velocity = land_controller.get_velocity(aruco_pose);
    send_velocity(velocity.linear.x,
                  velocity.linear.y,
                  velocity.linear.z,
                  velocity.angular.z);
    ss << "Velocity:\n";
    ss << "\tX:\t" << velocity.linear.x << "\n";
    ss << "\tY:\t" << velocity.linear.y << "\n";
    ss << "\tZ:\t" << velocity.linear.z << "\n";
    ss << "\tYaw:\t" << velocity.angular.z << "\n";

    bool completed = land_controller.completed_approach();
    ss << "\tCompleted approach: " << (completed ? "Yes" : "No") << "\n";

    if (completed)
    {
        ss << "\tTriggering LAND...\n";
        drone_control->land();
        state_machine.land();
    }
}

void Manager::FOLLOW_CONTROL_action(std::stringstream& ss)
{
    geometry_msgs::Twist velocity;

    velocity = follow_controller.get_velocity(aruco_pose);
    send_velocity(velocity.linear.x,
                  velocity.linear.y,
                  velocity.linear.z,
                  velocity.angular.z);
    ss << "Velocity:\n";
    ss << "\tX:\t" << velocity.linear.x << "\n";
    ss << "\tY:\t" << velocity.linear.y << "\n";
    ss << "\tZ:\t" << velocity.linear.z << "\n";
    ss << "\tYaw:\t" << velocity.angular.z << "\n";
}

void Manager::AWAITING_MODE_action(std::stringstream& ss)
{
    drone_control->live_signal();
    drone_control->await_offboardMode();
    // drone_control->takeOff();
}

void Manager::send_velocity(double x_linear, double y_linear, double z_linear, double angular)
{
    drone_control->cmd_vel(x_linear, y_linear, z_linear, angular);
    // ROS_INFO("SEND VELOCITY: x: %f y: %f z: %f yaw: %f", x_linear, y_linear, z_linear, angular);
}

void Manager::arucoPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    aruco_pose = *msg;
}

void Manager::rcCallback(const mavros_msgs::RCIn::ConstPtr &msg)
{
    rc_status = *msg;
}

void Manager::parametersCallback(const iris_land::controllers_gain::ConstPtr &msg)
{
    parameters = *msg;
}
