#include "manager.h"

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
}

void Manager::print_parameters()
{
    cout << "================" << endl;
    cout << "\tpose: " << pose.header.stamp << endl;
    cout << "\tx: " << pose.pose.position.x
         << "\ty: " << pose.pose.position.y
         << "\tz: " << pose.pose.position.z
         << "\ttheta: " << pose.pose.orientation.x << endl;

    cout << "\tlinear_vel: " << parameters.linear_vel
         << "\tangular_vel: " << parameters.angular_vel << endl;

    cout << "\tjoy: " << joy.header.stamp << endl;
    cout << "\todom: " << odom.header.stamp << endl;
    cout << "\tstate: " << states_name[state_machine.get_state()] << endl;

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
    case STATES::TAKE_OFF:
        TAKE_OFF_action();
        break;
    case STATES::LAND:
        LAND_action();
        break;
    case STATES::JOY_CONTROL:
        JOY_CONTROL_action();
        break;
    case STATES::LAND_CONTROL:
        LAND_CONTROL_action();
        break;
    case STATES::FOLLOW_CONTROL:
        FOLLOW_CONTROL_action();
        break;
    default:
        break;
    }

    follow_controller.update_parameters(parameters);
    land_controller.update_parameters(parameters);
    joy_linear_velocity = parameters.linear_vel;
    joy_angular_velocity = parameters.angular_vel;
    if (state_machine.update_state(joy))
    {
        send_velocity(0, 0, 0, 0);
        land_controller.reset_altitude(1);
    }
}

void Manager::STOPPED_action()
{
}

void Manager::TAKE_OFF_action()
{
    std_msgs::Empty emptyMsg;
    ROS_client->takeoff_pub.publish(emptyMsg);
}

void Manager::LAND_action()
{
    std_msgs::Empty emptyMsg;
    ROS_client->land_pub.publish(emptyMsg);
}

void Manager::JOY_CONTROL_action()
{
    if (joy.header.stamp.isZero())
    {
        return;
    }
    // if (joy_last_timestamp == joy.header.stamp)
    // {
    //   send_velocity(0, 0, 0, 0);
    //   return;
    // }
    joy_last_timestamp = joy.header.stamp;

    if (joy.axes[JOY_AXES::VERTICAL_ARROW] != 0)
    {
        send_velocity(joy.axes[JOY_AXES::VERTICAL_ARROW] * joy_linear_velocity, 0, 0, 0);
        return;
    }

    if (joy.axes[JOY_AXES::HORIZONTAL_ARROW] != 0)
    {
        send_velocity(0, joy.axes[JOY_AXES::HORIZONTAL_ARROW] * joy_linear_velocity, 0, 0);
        return;
    }

    if (joy.buttons[JOY_BUTTONS::RB] != 0)
    {
        send_velocity(0, 0, 0, -joy_angular_velocity);
        return;
    }

    if (joy.buttons[JOY_BUTTONS::LB] != 0)
    {
        send_velocity(0, 0, 0, joy_angular_velocity);
        return;
    }

    if (joy.buttons[JOY_BUTTONS::ANALOGIC_LEFT] != 0)
    {
        send_velocity(0, 0, -joy_linear_velocity, 0);
        return;
    }

    if (joy.buttons[JOY_BUTTONS::ANALOGIC_RIGHT] != 0)
    {
        send_velocity(0, 0, joy_linear_velocity, 0);
        return;
    }

    send_velocity(joy.axes[JOY_AXES::VERTICAL_ANALOGIC_LEFT] * joy_linear_velocity,
                  joy.axes[JOY_AXES::HORIZONTAL_ANALOGIC_LEFT] * joy_linear_velocity,
                  joy.axes[JOY_AXES::VERTICAL_ANALOGIC_RIGHT] * joy_linear_velocity,
                  joy.axes[JOY_AXES::HORIZONTAL_ANALOGIC_RIGHT] * joy_angular_velocity);
}

void Manager::LAND_CONTROL_action()
{
    cout << "**********************" << endl;
    geometry_msgs::Twist velocity;
    Speed drone_vel;
    drone_vel.vx = odom.twist.twist.linear.x;
    drone_vel.vy = odom.twist.twist.linear.y;
    drone_vel.vz = odom.twist.twist.linear.z;
    drone_vel.vtheta = odom.twist.twist.angular.z;

    velocity = land_controller.get_velocity(pose, drone_vel);
    send_velocity(velocity.linear.x,
                  velocity.linear.y,
                  velocity.linear.z,
                  velocity.angular.z);

    cout << "completed_approach: " << land_controller.completed_approach() << endl;
    if (land_controller.completed_approach())
    {
        std_msgs::Empty emptyMsg;
        ROS_client->land_pub.publish(emptyMsg);
        state_machine.land();
    }
    cout << "**********************" << endl;
}

void Manager::FOLLOW_CONTROL_action()
{
    geometry_msgs::Twist velocity;
    Speed drone_vel;
    drone_vel.vx = odom.twist.twist.linear.x;
    drone_vel.vy = odom.twist.twist.linear.y;
    drone_vel.vz = odom.twist.twist.linear.z;
    drone_vel.vtheta = odom.twist.twist.angular.z;

    velocity = follow_controller.get_velocity(pose, drone_vel);
    send_velocity(velocity.linear.x,
                  velocity.linear.y,
                  velocity.linear.z,
                  velocity.angular.z);
}

void Manager::send_velocity(double x_linear, double y_linear, double z_linear, double angular)
{
    cout << "x_linear: " << x_linear
         << "\ty_linear: " << y_linear
         << "\tz_linear: " << z_linear
         << "\tangular: " << angular << endl;

    geometry_msgs::Twist velocity;
    velocity.linear.x = -y_linear;
    velocity.linear.y = x_linear;
    velocity.linear.z = z_linear;

    velocity.angular.x = 0;
    velocity.angular.y = 0;
    velocity.angular.z = -angular;
    ROS_client->cmd_vel_pub.publish(velocity);
}

void Manager::poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    pose = *msg;
}

void Manager::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    // geometry_msgs::TransformStamped transformStamped_;
    // transformStamped_.transform.translation.x = 0;
    // transformStamped_.transform.translation.y = 0;
    // transformStamped_.transform.translation.z = 0;
    // transformStamped_.transform.rotation = msg->pose.pose.orientation;

    // tf2::doTransform(msg->twist.twist.linear, odom.twist.twist.linear, transformStamped_);

    // double temp_double = odom.twist.twist.linear.x;
    // odom.twist.twist.linear.x = odom.twist.twist.linear.y;
    // odom.twist.twist.linear.y = -odom.twist.twist.linear.x;
    // odom.twist.twist.linear.z = odom.twist.twist.linear.z;

    // odom.twist.twist.angular = msg->twist.twist.angular;
    // odom.header.stamp = msg->header.stamp;

    odom = *msg;

    tf2::Quaternion quat;
    tf2::fromMsg(msg->pose.pose.orientation, quat);
    tf2::Matrix3x3 mat(quat);
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);

    odom.twist.twist.linear.x = msg->twist.twist.linear.y * cos(yaw) + msg->twist.twist.linear.x * sin(yaw);
    odom.twist.twist.linear.y = msg->twist.twist.linear.y * sin(yaw) + msg->twist.twist.linear.x * -cos(yaw);
    odom.twist.twist.linear.z = msg->twist.twist.linear.z;
    odom.twist.twist.angular.z = -msg->twist.twist.angular.z;
}

void Manager::joyCallback(const sensor_msgs::Joy::ConstPtr &msg)
{
    joy = *msg;
}

void Manager::parametersCallback(const iris_land::controllers_gain::ConstPtr &msg)
{
    parameters = *msg;
}

void Manager::imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    imu = *msg;
}
