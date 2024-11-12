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
    cout << "================" << endl;
    cout << "\tpose: " << aruco_pose.header.stamp << endl;
    cout << "\tx: " << aruco_pose.pose.position.x
         << "\ty: " << aruco_pose.pose.position.y
         << "\tz: " << aruco_pose.pose.position.z
         << "\ttheta: " << aruco_pose.pose.orientation.x << endl;

    cout << "\tlinear_vel: " << parameters.linear_vel
         << "\tangular_vel: " << parameters.angular_vel << endl;

    cout << "\tstate: " << states_name[state_machine.get_state()] << endl;
    cout << "\trc: " << rc_status.header.stamp << endl;
    if(!rc_status.header.stamp.isZero()){
        cout << "\trc position: " << IDENTIFY_STATE_KEY_POSITION(rc_status.channels[STATE_KEY]) << endl;
    }
    cout << "\tflight_mode: " << drone_control->get_flight_mode() << endl;
    cout << "\tlanded_state: " << drone_control->get_landed_state() << endl;

    follow_controller.print_parameters();
    land_controller.print_parameters();
}

void Manager::update()
{
    STATES state = state_machine.get_state();
    switch (state)
    {
    case STATES::STOPPED:
        STOPPED_action();
        break;
    // case STATES::LAND:
    //     LAND_action();
    //     break;
    // case STATES::LAND_CONTROL:
    //     LAND_CONTROL_action();
    //     break;
    // case STATES::FOLLOW_CONTROL:
    //     FOLLOW_CONTROL_action();
    //     break;
    case STATES::AWAITING_MODE:
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
        land_controller.reset_altitude(1);
    }
}

void Manager::STOPPED_action()
{
}

void Manager::LAND_action()
{
    // std_msgs::Empty emptyMsg;
    // ROS_client->land_pub.publish(emptyMsg);
    // TODO: chamar o land da droneControl
}

void Manager::LAND_CONTROL_action()
{
    cout << "**********************" << endl;
    geometry_msgs::Twist velocity;

    velocity = land_controller.get_velocity(aruco_pose);
    send_velocity(velocity.linear.x,
                  velocity.linear.y,
                  velocity.linear.z,
                  velocity.angular.z);

    cout << "completed_approach: " << land_controller.completed_approach() << endl;
    if (land_controller.completed_approach())
    {
        // std_msgs::Empty emptyMsg;
        // ROS_client->land_pub.publish(emptyMsg);
        // TODO: chamar o land da droneControl
        // state_machine.land();
    }
    cout << "**********************" << endl;
}

void Manager::FOLLOW_CONTROL_action()
{
    geometry_msgs::Twist velocity;

    velocity = follow_controller.get_velocity(aruco_pose);
    send_velocity(velocity.linear.x,
                  velocity.linear.y,
                  velocity.linear.z,
                  velocity.angular.z);
}

void Manager::send_velocity(double x_linear, double y_linear, double z_linear, double angular)
{
    // cout << "x_linear: " << x_linear
    //      << "\ty_linear: " << y_linear
    //      << "\tz_linear: " << z_linear
    //      << "\tangular: " << angular << endl;

    // geometry_msgs::Twist velocity;
    // velocity.linear.x = -y_linear;
    // velocity.linear.y = x_linear;
    // velocity.linear.z = z_linear;

    // velocity.angular.x = 0;
    // velocity.angular.y = 0;
    // velocity.angular.z = -angular;
    // ROS_client->cmd_vel_pub.publish(velocity);
    // TODO: chamar o cmdvel da droneControl
    // TODO: verificar se esta no modo

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
