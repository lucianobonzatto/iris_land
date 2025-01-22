#include "drone_control.h"

DroneControl::DroneControl()
{
    local_position_.header.stamp = ros::Time::now();
    local_position_.header.frame_id = "world";
    local_position_.pose.position.x = 0;
    local_position_.pose.position.y = 0;
    local_position_.pose.position.z = 0;
    local_position_.pose.orientation.x = 0;
    local_position_.pose.orientation.y = 0;
    local_position_.pose.orientation.z = 0;
    local_position_.pose.orientation.w = 1;
    setpoint_pos_ENU_ = gps_init_pos_ = local_position_;
}

DroneControl::~DroneControl()
{
}

void DroneControl::Init(ROSClient *drone_control)
{
    ros_client_ = drone_control;
    this->rate_ = new ros::Rate(ROS_RATE);
    static tf2_ros::TransformListener tfListener(tfBuffer_);
}

void DroneControl::Setup()
{
    while (ros::ok() && !current_state_.connected)
    {
        ros::spinOnce();
        rate_->sleep();
        ROS_INFO("connecting to FCU...");
    }
    for (int i = 0; ros::ok() && i < 4 * ROS_RATE; ++i)
    {
        ros::spinOnce();
        rate_->sleep();
    }
    if (ros::Time::now() - local_position_.header.stamp < ros::Duration(1.0))
    {
        ROS_INFO("Local_position available");
    }
    else
    {
        ROS_WARN("Local_position not available, initializing to 0");
        local_position_.header.stamp = ros::Time::now();
        local_position_.header.frame_id = "world";
        local_position_.pose.position.x = 0;
        local_position_.pose.position.y = 0;
        local_position_.pose.position.z = 0;
        local_position_.pose.orientation.x = 0;
        local_position_.pose.orientation.y = 0;
        local_position_.pose.orientation.z = 0;
        local_position_.pose.orientation.w = 1;
    }
}

void DroneControl::live_signal()
{
    if (ros::Time::now() - local_position_.header.stamp > ros::Duration(1.0))
    {
        ROS_WARN("Local_position not available, initializing to 0");
        local_position_.header.stamp = ros::Time::now();
        local_position_.header.frame_id = "world";
        local_position_.pose.position.x = 0;
        local_position_.pose.position.y = 0;
        local_position_.pose.position.z = 0;
        local_position_.pose.orientation.x = 0;
        local_position_.pose.orientation.y = 0;
        local_position_.pose.orientation.z = 0;
        local_position_.pose.orientation.w = 1;
    }
    setpoint_pos_ENU_ = gps_init_pos_ = local_position_;
    ros_client_->setpoint_pos_pub_.publish(setpoint_pos_ENU_);
}

void DroneControl::state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state_ = *msg;
}

void DroneControl::extended_state_cb(const mavros_msgs::ExtendedState::ConstPtr &msg)
{
    landed_state_ = msg->landed_state;
}

void DroneControl::local_position_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    local_position_ = *msg;
    static int cnt = 0;

    static tf2_ros::TransformBroadcaster br;
    static tf2_ros::StaticTransformBroadcaster sbr;

    // Transformation from world to drone
    transformStamped_.header.stamp = local_position_.header.stamp;
    transformStamped_.header.frame_id = "world";
    transformStamped_.child_frame_id = "drone";
    transformStamped_.transform.translation.x = local_position_.pose.position.x;
    transformStamped_.transform.translation.y = local_position_.pose.position.y;
    transformStamped_.transform.translation.z = local_position_.pose.position.z;
    transformStamped_.transform.rotation = local_position_.pose.orientation;
    br.sendTransform(transformStamped_);

    cnt++;
    if (cnt % 100 == 0)
    {
        ROS_INFO("Mavros local position: E: %f, N: %f, U: %f, yaw: %f", transformStamped_.transform.translation.x,
                 transformStamped_.transform.translation.y, transformStamped_.transform.translation.z, currentYaw());
    }
}

void DroneControl::global_position_cb(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
    global_position_ = *msg;
    static int cnt = 0;

    cnt++;
    if (cnt % 100 == 0)
    {
        // ROS_INFO("GPS: lat: %f, long: %f, alt: %f", msg->latitude, msg->longitude, msg->altitude);
    }
}

void DroneControl::flyToGlobal(double latitude, double longitude, double altitude, double yaw)
{
    mavros_msgs::GlobalPositionTarget target;
    target.coordinate_frame = mavros_msgs::GlobalPositionTarget::FRAME_GLOBAL_INT;
    target.type_mask = mavros_msgs::GlobalPositionTarget::IGNORE_VX |
                       mavros_msgs::GlobalPositionTarget::IGNORE_VY |
                       mavros_msgs::GlobalPositionTarget::IGNORE_VZ |
                       mavros_msgs::GlobalPositionTarget::IGNORE_AFX |
                       mavros_msgs::GlobalPositionTarget::IGNORE_AFY |
                       mavros_msgs::GlobalPositionTarget::IGNORE_AFZ |
                       mavros_msgs::GlobalPositionTarget::IGNORE_YAW_RATE;
    target.latitude = latitude;
    target.longitude = longitude;
    target.altitude = altitude;
    target.yaw = yaw;

    while (ros::ok() &&
           (fabs(latitude - global_position_.latitude) * LAT_DEG_TO_M > 1.0 || fabs(longitude - global_position_.longitude) * LON_DEG_TO_M > 1.0 || fabs(altitude - global_position_.altitude) > 1.0))
    {
        ROS_INFO("Dist: lat: %f, long: %f, alt: %f", fabs(latitude - global_position_.latitude) * LAT_DEG_TO_M, fabs(longitude - global_position_.longitude) * LON_DEG_TO_M, altitude - global_position_.altitude);
        ros_client_->global_setpoint_pos_pub_.publish(target);
        ros::spinOnce();
        rate_->sleep();
    }
}

void DroneControl::flyToLocal(double x, double y, double z, double yaw)
{
    if (!std::isfinite(yaw))
    {
        yaw = currentYaw();
        ROS_INFO("Flying to local coordinates E: %f, N: %f, U: %f, current yaw: %f", x, y, z, yaw);
    }
    else
        ROS_INFO("Flying to local coordinates E: %f, N: %f, U: %f, yaw: %f", x, y, z, yaw);

    setpoint_pos_ENU_.pose.position.x = x;
    setpoint_pos_ENU_.pose.position.y = y;
    setpoint_pos_ENU_.pose.position.z = z;
    setpoint_pos_ENU_.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

    while (ros::ok() && distance(setpoint_pos_ENU_, local_position_) > 0.5)
    {
        ros_client_->setpoint_pos_pub_.publish(setpoint_pos_ENU_);
        ros::spinOnce();
        rate_->sleep();
    }

    // Publish for another second
    for (int i = 0; ros::ok() && i < ROS_RATE / 20; ++i)
    {
        ros_client_->setpoint_pos_pub_.publish(setpoint_pos_ENU_);
        ros::spinOnce();
        rate_->sleep();
    }
}

void DroneControl::hover(double seconds)
{
    ROS_INFO("Hovering for %f seconds in position: E: %f, N: %f, U: %f", seconds,
             local_position_.pose.position.x,
             local_position_.pose.position.y,
             local_position_.pose.position.z);

    for (int i = 0; ros::ok() && i < 15 * ROS_RATE; ++i)
    {
        ros_client_->setpoint_pos_pub_.publish(local_position_);
        ros::spinOnce();
        rate_->sleep();
    }

    // ros::Time current_time, last_command_time = ros::Time::now();
    // while (ros::ok())
    // {
    //   ros_client_->setpoint_pos_pub_.publish(local_position_);
    //   ros::spinOnce();
    //   rate_->sleep();

    //   current_time = ros::Time::now();
    //   if ((current_time - last_command_time).toSec() >= seconds)
    //   {
    //     break;
    //   }
    // }
}

void DroneControl::cmd_vel(double x, double y, double z, double ang)
{
    geometry_msgs::TwistStamped vel_msg, world_msg;

    transformStamped_.header.stamp = local_position_.header.stamp;
    transformStamped_.header.frame_id = "world";
    transformStamped_.child_frame_id = "drone";
    transformStamped_.transform.translation.x = local_position_.pose.position.x;
    transformStamped_.transform.translation.y = local_position_.pose.position.y;
    transformStamped_.transform.translation.z = local_position_.pose.position.z;
    transformStamped_.transform.rotation = local_position_.pose.orientation;

    vel_msg.twist.linear.x = x;
    vel_msg.twist.linear.y = y;
    vel_msg.twist.linear.z = z;

    tf2::doTransform(vel_msg.twist.linear, world_msg.twist.linear, transformStamped_);
    world_msg.header.stamp = ros::Time::now();
    world_msg.twist.angular.x = 0;
    world_msg.twist.angular.y = 0;
    world_msg.twist.angular.z = ang;

    // ROS_INFO("SEND VELOCITY: x: %f y: %f z: %f yaw: %f", x, y, z, ang);
    ros_client_->velocity_pub.publish(world_msg);
    // ros::spinOnce();
    // rate_->sleep();
}

void DroneControl::cmd_vel_base_link(double x, double y, double z, double ang)
{
    geometry_msgs::TwistStamped vel_msg;
    vel_msg.header.stamp = ros::Time::now();
    vel_msg.header.frame_id = "base_link";

    vel_msg.twist.linear.x = x;
    vel_msg.twist.linear.y = y;
    vel_msg.twist.linear.z = z;
    vel_msg.twist.angular.x = 0;
    vel_msg.twist.angular.y = 0;
    vel_msg.twist.angular.z = ang;

    ROS_INFO("SEND VELOCITY: x: %f y: %f z: %f yaw: %f", x, y, z, ang);
    ros_client_->velocity_pub.publish(vel_msg);
    //   ros::spinOnce();
    //   rate_->sleep();
}

void DroneControl::cmd_vel_unstamped(double x, double y, double z, double ang)
{
    geometry_msgs::Twist vel_msg;

    vel_msg.linear.x = x;
    vel_msg.linear.y = y;
    vel_msg.linear.z = z;
    vel_msg.angular.x = 0;
    vel_msg.angular.y = 0;
    vel_msg.angular.z = ang;

    ROS_INFO("SEND VELOCITY: x: %f y: %f z: %f yaw: %f", x, y, z, ang);
    ros_client_->velocity_unstamped_pub.publish(vel_msg);
    // ros::spinOnce();
    // rate_->sleep();
}

void DroneControl::set_offboardMode()
{
    Setup();
    setpoint_pos_ENU_ = gps_init_pos_ = local_position_;

    // Send a few setpoints before starting
    for (int i = 20; ros::ok() && i > 0; --i)
    {
        ros_client_->setpoint_pos_pub_.publish(setpoint_pos_ENU_);
        ros::spinOnce();
        rate_->sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    arm_cmd_.request.value = true;
    last_request_ = ros::Time::now();
    // Change to offboard mode and arm
    ROS_INFO("awaiting to OFFBOARD mode");
    while (ros::ok() && !current_state_.armed)
    {
        if (current_state_.mode != "OFFBOARD" && (ros::Time::now() - last_request_ > ros::Duration(5.0)))
        {
            ROS_INFO("%s", current_state_.mode.c_str());
            if (ros_client_->set_mode_client_.call(offb_set_mode) && offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
            }
            last_request_ = ros::Time::now();
        }
        else
        {
            if (!current_state_.armed && (ros::Time::now() - last_request_ > ros::Duration(5.0)))
            {
                if (ros_client_->arming_client_.call(arm_cmd_) && arm_cmd_.response.success)
                {
                    ROS_INFO("Vehicle armed");
                }
                last_request_ = ros::Time::now();
            }
        }
        ros_client_->setpoint_pos_pub_.publish(setpoint_pos_ENU_);
        ros::spinOnce();
        rate_->sleep();
    }

    return;
}

void DroneControl::await_offboardMode()
{
    Setup();
    setpoint_pos_ENU_ = gps_init_pos_ = local_position_;

    // Send a few setpoints before starting
    for (int i = 20; ros::ok() && i > 0; --i)
    {
        ros_client_->setpoint_pos_pub_.publish(setpoint_pos_ENU_);
        ros::spinOnce();
        rate_->sleep();
    }

    ROS_INFO("awaiting to OFFBOARD mode");
    while (ros::ok())
    {
        if (current_state_.mode == "OFFBOARD")
        {
            ROS_INFO("OFFBOARD enabled");
            break;
        }

        ros_client_->setpoint_pos_pub_.publish(setpoint_pos_ENU_);
        ros::spinOnce();
        rate_->sleep();
    }
}

void DroneControl::takeOff()
{
    ROS_INFO("awaiting to arm");
    // arm
    mavros_msgs::CommandBool arm_request;
    arm_request.request.value = true;
    while (ros::ok() && !current_state_.armed)
    {
        ros::spinOnce();
        rate_->sleep();
        ros_client_->arming_client_.call(arm_request);
        ROS_INFO("awaiting to arm %d", current_state_.armed);
    }
    ROS_INFO("awaiting to arm %d", current_state_.armed);

    sleep(3);

    // Take off
    // mavros_msgs::CommandTOL takeoff_request;
    // takeoff_request.request.altitude = 3;
    setpoint_pos_ENU_ = gps_init_pos_;
    setpoint_pos_ENU_.pose.position.z += TAKEOFF_ALTITUDE;

    ROS_INFO("Trying to Takeoff");
    int i = 0;
    while (ros::ok() && i < 10 * ROS_RATE)
    {
        i++;
        ROS_INFO("Retrying to Takeoff");
        // ros_client_->takeoff_client_.call(takeoff_request);
        ros_client_->setpoint_pos_pub_.publish(setpoint_pos_ENU_);

        ros::spinOnce();
        rate_->sleep();
        // ros::Duration(.1).sleep();
    }
    // sleep(10);

    // ROS_INFO("landed_state_: %d", landed_state_);
    ROS_INFO("Takeoff finished!");
    return;
}

void DroneControl::land()
{
    int i;
    mavros_msgs::CommandTOL land_cmd;
    land_cmd.request.yaw = 0;
    land_cmd.request.latitude = NAN; // Land at current location
    land_cmd.request.longitude = NAN;
    land_cmd.request.altitude = 0;

    ROS_INFO("Trying to land");
    while (!(ros_client_->land_client_.call(land_cmd) && land_cmd.response.success))
    {
        ros_client_->setpoint_pos_pub_.publish(setpoint_pos_ENU_);
        ros::spinOnce();
        ROS_WARN("Retrying to land");
        rate_->sleep();
    }

    // Wait until proper landing (or a maximum of 15 seconds)
    for (i = 0; ros::ok() && landed_state_ != mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND && i < MAX_ATTEMPTS; ++i)
    {
        ros::spinOnce();
        rate_->sleep();
    }
    ROS_INFO("Landing success");

    return;
}

void DroneControl::disarm()
{
    // Disarm
    arm_cmd_.request.value = false;
    while (ros::ok() && current_state_.armed)
    {
        if (current_state_.armed && (ros::Time::now() - last_request_ > ros::Duration(5.0)))
        {
            if (ros_client_->arming_client_.call(arm_cmd_) && arm_cmd_.response.success)
            {
                ROS_INFO("Vehicle disarmed");
            }
            last_request_ = ros::Time::now();
        }
        ros::spinOnce();
        rate_->sleep();
    }
    return;
}

string DroneControl::get_flight_mode()
{
    return current_state_.mode;
}

int DroneControl::get_landed_state()
{
    return landed_state_;
}

double DroneControl::currentYaw()
{
    // Calculate yaw current orientation
    double roll, pitch, yaw;
    tf::Quaternion q;

    tf::quaternionMsgToTF(local_position_.pose.orientation, q);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

    return yaw;
}

double DroneControl::distance(const geometry_msgs::PoseStamped &p1, const geometry_msgs::PoseStamped &p2)
{
    tf::Point t1, t2;
    tf::pointMsgToTF(p1.pose.position, t1);
    tf::pointMsgToTF(p2.pose.position, t2);

    return t1.distance(t2);
}
