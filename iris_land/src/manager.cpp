#include "manager.h"
#include "drone_control.h"
#include <geometry_msgs/TwistStamped.h>
#include <cmath>

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

    ss << "Controller Pose:\n";
    ss << "\tstamp: " << aruco_pose.header.stamp << "\n";
    ss << "\tx: " << aruco_pose.pose.position.x
       << "\ty: " << aruco_pose.pose.position.y
       << "\tz: " << aruco_pose.pose.position.z
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

    std_msgs::String msg;
    msg.data = ss.str();
    ROS_client->status_pub.publish(msg);
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

    std_msgs::String msg;
    msg.data = ss.str();
    ROS_client->status_pub.publish(msg);
    ROS_INFO_STREAM(ss.str());
}

void Manager::STOPPED_action(std::stringstream& ss)
{
    ss << "Velocity:\n";
    ss << "\tX:\t" << 0;
    ss << "\tY:\t" << 0;
    ss << "\tZ:\t" << 0;
    ss << "\tYaw:\t" << 0;
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
    ss << "\tX:\t" << velocity.linear.x;
    ss << "\tY:\t" << velocity.linear.y;
    ss << "\tZ:\t" << velocity.linear.z;
    ss << "\tYaw:\t" << velocity.angular.z;

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
    ss << "\tX:\t" << velocity.linear.x;
    ss << "\tY:\t" << velocity.linear.y;
    ss << "\tZ:\t" << velocity.linear.z;
    ss << "\tYaw:\t" << velocity.angular.z;
}

void Manager::AWAITING_MODE_action(std::stringstream& ss)
{
    drone_control->live_signal();
    drone_control->await_offboardMode();
    // drone_control->takeOff();
}

void Manager::send_velocity(double x_linear, double y_linear, double z_linear, double angular)
{
    // Controller x/y is now a correction in the landing-pad/world frame because
    // it is computed directly from /ekf/pose, whose position is expressed in
    // the landpad frame. DroneControl::cmd_vel() expects a drone/body-frame
    // linear command and rotates it into MAVROS local/world before publishing.
    // Therefore we convert landpad/world x/y -> drone/body x/y here, using the
    // same MAVROS local pose yaw that DroneControl::cmd_vel() will use for the
    // forward body->world rotation.
    const double yaw_world_drone = get_yaw(drone_control->local_position_.pose.orientation);
    const double c = std::cos(yaw_world_drone);
    const double s = std::sin(yaw_world_drone);

    const double x_body =  c * x_linear + s * y_linear;
    const double y_body = -s * x_linear + c * y_linear;

    // Publish the exact body-frame command passed into DroneControl::cmd_vel().
    // Bag this together with /mavros/setpoint_velocity/cmd_vel to verify that
    // cmd_vel() rotates it into the intended MAVROS local/world command.
    geometry_msgs::TwistStamped raw_cmd;
    raw_cmd.header.stamp = ros::Time::now();
    raw_cmd.header.frame_id = "drone_body_cmd_input_to_cmd_vel";
    raw_cmd.twist.linear.x = x_body;
    raw_cmd.twist.linear.y = y_body;
    raw_cmd.twist.linear.z = z_linear;
    raw_cmd.twist.angular.z = angular;
    ROS_client->raw_velocity_pub.publish(raw_cmd);

    drone_control->cmd_vel(x_body, y_body, z_linear, angular);
    // ROS_INFO("SEND VELOCITY BODY: x: %f y: %f z: %f yaw: %f", x_body, y_body, z_linear, angular);
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
