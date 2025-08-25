#include "drone_control.h"
#include "ros_client.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

DroneControl::DroneControl(const rclcpp::Node::SharedPtr &node, ROSClient *ros_client)
    : nh_(node),
      ros_client_(ros_client),
      rate_(new rclcpp::Rate(ROS_RATE)),
      tfBuffer_(ros_client->nh_->get_clock()),
      tfListener_(tfBuffer_),
      br_(ros_client->nh_)
{
    current_state_.connected = false;
}

void DroneControl::Setup()
{
    // Wait for FCU connection
    while (rclcpp::ok() && !current_state_.connected)
    {
        rclcpp::spin_some(ros_client_->nh_);
        rate_->sleep();
        RCLCPP_INFO(nh_->get_logger(), "connecting to FCU...");
    }

    for (int i = 0; rclcpp::ok() && i < 4 * ROS_RATE; ++i)
    {
        rclcpp::spin_some(ros_client_->nh_);
        rate_->sleep();
    }
    if (ros_client_->nh_->now() - local_position_.header.stamp < rclcpp::Duration::from_seconds(1.0))
    {
        RCLCPP_INFO(nh_->get_logger(), "Local_position available");
    }
    else
    {
        RCLCPP_INFO(nh_->get_logger(), "Local_position not available, initializing to 0");
        local_position_.header.stamp = ros_client_->nh_->now();
        local_position_.header.frame_id = "world";
        local_position_.pose.position.x = 0;
        local_position_.pose.position.y = 0;
        local_position_.pose.position.z = 0;
        local_position_.pose.orientation.x = 0;
        local_position_.pose.orientation.y = 0;
        local_position_.pose.orientation.z = 0;
        local_position_.pose.orientation.w = 1;

        gps_init_pos_.header.stamp = ros_client_->nh_->now();
        gps_init_pos_.header.frame_id = "world";
        gps_init_pos_.pose.position.x = 0;
        gps_init_pos_.pose.position.y = 0;
        gps_init_pos_.pose.position.z = 0;
        gps_init_pos_.pose.orientation.x = 0;
        gps_init_pos_.pose.orientation.y = 0;
        gps_init_pos_.pose.orientation.z = 0;
        gps_init_pos_.pose.orientation.w = 1;
    }
}

void DroneControl::live_signal()
{
    if (ros_client_->nh_->now() - local_position_.header.stamp < rclcpp::Duration::from_seconds(1.0))
    {
        RCLCPP_INFO(nh_->get_logger(), "Local_position not available, initializing to 0");
        local_position_.header.stamp = ros_client_->nh_->now();
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
    ros_client_->setpoint_pos_pub_->publish(setpoint_pos_ENU_);
}

void DroneControl::state_cb(const mavros_msgs::msg::State::SharedPtr msg)
{
    current_state_ = *msg;
}

void DroneControl::extended_state_cb(const mavros_msgs::msg::ExtendedState::SharedPtr msg)
{
    landed_state_ = msg->landed_state;
}

void DroneControl::local_position_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    local_position_ = *msg;

    // Transformation from world to drone
    transformStamped_.header.stamp = local_position_.header.stamp;
    transformStamped_.header.frame_id = "world";
    transformStamped_.child_frame_id = "drone";
    transformStamped_.transform.translation.x = local_position_.pose.position.x;
    transformStamped_.transform.translation.y = local_position_.pose.position.y;
    transformStamped_.transform.translation.z = local_position_.pose.position.z;
    transformStamped_.transform.rotation = local_position_.pose.orientation;
    // br_.sendTransform(transformStamped_);
}

void DroneControl::global_position_cb(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    global_position_ = *msg;
}

void DroneControl::flyToGlobal(double latitude, double longitude, double altitude, double yaw)
{
    geographic_msgs::msg::GeoPoseStamped msg;
    msg.header.stamp = ros_client_->nh_->now();
    msg.header.frame_id = "world";
    msg.pose.position.latitude = latitude;
    msg.pose.position.longitude = longitude;
    msg.pose.position.altitude = altitude;

    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    msg.pose.orientation.x = q.x();
    msg.pose.orientation.y = q.y();
    msg.pose.orientation.z = q.z();
    msg.pose.orientation.w = q.w();

    auto isTargetReached = [&](double lat, double lon, double alt)
    {
        double d_lat = fabs(lat - global_position_.latitude) * LAT_DEG_TO_M;
        double d_lon = fabs(lon - global_position_.longitude) * LON_DEG_TO_M;
        double d_alt = fabs(alt - global_position_.altitude);
        return (d_lat < 1.0 && d_lon < 1.0 && d_alt < 1.0);
    };

    RCLCPP_INFO(nh_->get_logger(),
                "Flying to global coordinates E: %f, N: %f, U: %f, yaw: %f",
                latitude, longitude, altitude, yaw);
    while (rclcpp::ok() && !isTargetReached(latitude, longitude, altitude))
    {
        RCLCPP_INFO(nh_->get_logger(),
                    "Dist: lat: %f, long: %f, alt: %f",
                    fabs(latitude - global_position_.latitude) * LAT_DEG_TO_M,
                    fabs(longitude - global_position_.longitude) * LON_DEG_TO_M,
                    altitude - global_position_.altitude);

        ros_client_->global_setpoint_pos_pub_->publish(msg);
        rclcpp::spin_some(ros_client_->nh_);
        rate_->sleep();
    }
}

void DroneControl::flyToLocal(double x, double y, double z, double yaw)
{
    setpoint_pos_ENU_.pose.position.x = x;
    setpoint_pos_ENU_.pose.position.y = y;
    setpoint_pos_ENU_.pose.position.z = z;

    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    setpoint_pos_ENU_.pose.orientation.x = q.x();
    setpoint_pos_ENU_.pose.orientation.y = q.y();
    setpoint_pos_ENU_.pose.orientation.z = q.z();
    setpoint_pos_ENU_.pose.orientation.w = q.w();

    RCLCPP_INFO(nh_->get_logger(),
                "Flying to local coordinates E: %f, N: %f, U: %f, yaw: %f",
                x, y, z, yaw);
    while (rclcpp::ok() && distance(setpoint_pos_ENU_, local_position_) > 0.5)
    {
        ros_client_->setpoint_pos_pub_->publish(setpoint_pos_ENU_);
        rclcpp::spin_some(ros_client_->nh_);
        rate_->sleep();
    }

    // Publish for another short period to stabilize
    for (int i = 0; rclcpp::ok() && i < ROS_RATE / 20; ++i)
    {
        ros_client_->setpoint_pos_pub_->publish(setpoint_pos_ENU_);
        rclcpp::spin_some(ros_client_->nh_);
        rate_->sleep();
    }
}

void DroneControl::hover(double seconds)
{
    RCLCPP_INFO(nh_->get_logger(),
                "Hovering for %f seconds in position: E: %f, N: %f, U: %f",
                seconds,
                local_position_.pose.position.x,
                local_position_.pose.position.y,
                local_position_.pose.position.z);

    auto start_time = ros_client_->nh_->now();

    while (rclcpp::ok() && (ros_client_->nh_->now() - start_time).seconds() < seconds)
    {
        local_position_.header.stamp = ros_client_->nh_->get_clock()->now();
        ros_client_->setpoint_pos_pub_->publish(local_position_);
        rclcpp::spin_some(ros_client_->nh_);
        rate_->sleep();
    }
}

void DroneControl::cmd_vel(double x, double y, double z, double ang)
{
    geometry_msgs::msg::TwistStamped vel_msg;
    geometry_msgs::msg::Vector3 linear_vel_in, linear_vel_out;
    linear_vel_in.x = x;
    linear_vel_in.y = y;
    linear_vel_in.z = z;

    // Criar transformStamped de drone → world
    transformStamped_.header.stamp = local_position_.header.stamp;
    transformStamped_.header.frame_id = "world";
    transformStamped_.child_frame_id = "drone";
    transformStamped_.transform.translation.x = local_position_.pose.position.x;
    transformStamped_.transform.translation.y = local_position_.pose.position.y;
    transformStamped_.transform.translation.z = local_position_.pose.position.z;
    transformStamped_.transform.rotation = local_position_.pose.orientation;

    // Preencher TwistStamped no frame world
    tf2::doTransform(linear_vel_in, linear_vel_out, transformStamped_);

    vel_msg.header.stamp = ros_client_->nh_->now();
    vel_msg.header.frame_id = "world";
    vel_msg.twist.linear = linear_vel_out;
    vel_msg.twist.angular.x = 0;
    vel_msg.twist.angular.y = 0;
    vel_msg.twist.angular.z = ang;

    RCLCPP_INFO(nh_->get_logger(),
                "SEND VELOCITY: x: %f y: %f z: %f yaw: %f",
                vel_msg.twist.linear.x,
                vel_msg.twist.linear.y,
                vel_msg.twist.linear.z,
                vel_msg.twist.angular.z);

    ros_client_->velocity_pub->publish(vel_msg);
}

void DroneControl::cmd_vel_base_link(double x, double y, double z, double ang)
{
    geometry_msgs::msg::TwistStamped vel_msg;
    vel_msg.header.stamp = ros_client_->nh_->now();
    vel_msg.header.frame_id = "base_link";

    vel_msg.twist.linear.x = x;
    vel_msg.twist.linear.y = y;
    vel_msg.twist.linear.z = z;
    vel_msg.twist.angular.x = 0;
    vel_msg.twist.angular.y = 0;
    vel_msg.twist.angular.z = ang;

    RCLCPP_INFO(nh_->get_logger(),
                "SEND VELOCITY: x: %f y: %f z: %f yaw: %f",
                vel_msg.twist.linear.x,
                vel_msg.twist.linear.y,
                vel_msg.twist.linear.z,
                vel_msg.twist.angular.z);
    ros_client_->velocity_pub->publish(vel_msg);
}

void DroneControl::cmd_vel_unstamped(double x, double y, double z, double ang)
{
    geometry_msgs::msg::Twist vel_msg;

    vel_msg.linear.x = x;
    vel_msg.linear.y = y;
    vel_msg.linear.z = z;
    vel_msg.angular.x = 0;
    vel_msg.angular.y = 0;
    vel_msg.angular.z = ang;

    RCLCPP_INFO(nh_->get_logger(),
                "SEND VELOCITY: x: %f y: %f z: %f yaw: %f",
                vel_msg.linear.x,
                vel_msg.linear.y,
                vel_msg.linear.z,
                vel_msg.angular.z);
    ros_client_->velocity_unstamped_pub->publish(vel_msg);
}

void DroneControl::await_OFFBOARD_Mode()
{
    if (ros_client_->nh_->now() - local_position_.header.stamp < rclcpp::Duration::from_seconds(1.0))
    {
        RCLCPP_INFO(nh_->get_logger(), "Local_position available");
    }
    else
    {
        RCLCPP_INFO(nh_->get_logger(), "Local_position not available, initializing to 0");
        local_position_.header.stamp = ros_client_->nh_->now();
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

    RCLCPP_INFO(nh_->get_logger(), "awaiting to OFFBOARD mode");
    while (rclcpp::ok())
    {
        if (current_state_.mode == "OFFBOARD")
        {
            RCLCPP_INFO(nh_->get_logger(), "OFFBOARD enabled");
            break;
        }
        ros_client_->setpoint_pos_pub_->publish(setpoint_pos_ENU_);
        rclcpp::spin_some(ros_client_->nh_);
        rate_->sleep();
    }
}

void DroneControl::set_OFFBOARD_Mode()
{
    if (ros_client_->nh_->now() - local_position_.header.stamp < rclcpp::Duration::from_seconds(1.0))
    {
        RCLCPP_INFO(nh_->get_logger(), "Local_position available");
    }
    else
    {
        RCLCPP_INFO(nh_->get_logger(), "Local_position not available, initializing to 0");
        local_position_.header.stamp = ros_client_->nh_->now();
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

    RCLCPP_INFO(nh_->get_logger(), "Trying to set OFFBOARD mode");
    if (!ros_client_->set_mode_client_->wait_for_service(std::chrono::seconds(5)))
    {
        RCLCPP_WARN(nh_->get_logger(), "SetMode service not available");
        return;
    }

    auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    request->custom_mode = "OFFBOARD";
    auto future = ros_client_->set_mode_client_->async_send_request(request);

    while (rclcpp::ok())
    {
        auto future = ros_client_->set_mode_client_->async_send_request(request);
        rclcpp::spin_some(ros_client_->nh_);
        rate_->sleep();

        if (current_state_.mode == "OFFBOARD")
        {
            RCLCPP_INFO(nh_->get_logger(), "OFFBOARD mode confirmed");
            break;
        }
        else
        {
            RCLCPP_INFO(nh_->get_logger(), "Waiting for OFFBOARD mode...");
        }
    }
}

void DroneControl::arm()
{
    auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    request->value = true;

    RCLCPP_INFO(nh_->get_logger(), "Trying to arm");

    // Espera o serviço ficar disponível
    if (!ros_client_->arming_client_->wait_for_service(std::chrono::seconds(5)))
    {
        RCLCPP_WARN(nh_->get_logger(), "Arming service not available");
        return;
    }

    auto future = ros_client_->arming_client_->async_send_request(request);
    while (rclcpp::ok())
    {
        rclcpp::spin_some(ros_client_->nh_);
        ros_client_->setpoint_pos_pub_->publish(setpoint_pos_ENU_);
        if (rclcpp::spin_until_future_complete(ros_client_->nh_, future) == rclcpp::FutureReturnCode::SUCCESS)
        {
            if (future.get()->success)
            {
                RCLCPP_INFO(nh_->get_logger(), "Drone armed successfully");
                break;
            }
            else
            {
                RCLCPP_WARN(nh_->get_logger(), "Arming failed, retrying...");
                future = ros_client_->arming_client_->async_send_request(request);
            }
        }
        rate_->sleep();
    }
}

void DroneControl::takeOff()
{
    arm();

    setpoint_pos_ENU_ = gps_init_pos_;
    setpoint_pos_ENU_.pose.position.z += TAKEOFF_ALTITUDE;

    RCLCPP_INFO(nh_->get_logger(), "Trying to Takeoff");

    int i = 0;
    while (rclcpp::ok() && i < MAX_ATTEMPTS)
    {
        setpoint_pos_ENU_.header.stamp = ros_client_->nh_->get_clock()->now();
        ros_client_->setpoint_pos_pub_->publish(setpoint_pos_ENU_);
        rclcpp::spin_some(ros_client_->nh_);
        rate_->sleep();
        i++;
    }

    RCLCPP_INFO(nh_->get_logger(), "Takeoff finished!");
}

void DroneControl::land()
{
    auto client = ros_client_->land_client_;
    auto request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
    request->yaw = 0;
    request->latitude = std::numeric_limits<double>::quiet_NaN(); // Land at current location
    request->longitude = std::numeric_limits<double>::quiet_NaN();
    request->altitude = 0;

    RCLCPP_INFO(nh_->get_logger(), "Trying to land");

    // Espera o serviço ficar disponível
    if (!client->wait_for_service(std::chrono::seconds(5)))
    {
        RCLCPP_WARN(nh_->get_logger(), "Land service not available");
        return;
    }

    auto future = client->async_send_request(request);
    while (rclcpp::ok())
    {
        rclcpp::spin_some(ros_client_->nh_);
        ros_client_->setpoint_pos_pub_->publish(setpoint_pos_ENU_);
        if (rclcpp::spin_until_future_complete(ros_client_->nh_, future) == rclcpp::FutureReturnCode::SUCCESS)
        {
            if (future.get()->success)
            {
                RCLCPP_INFO(nh_->get_logger(), "Land command accepted");
                break;
            }
            else
            {
                RCLCPP_WARN(nh_->get_logger(), "Land command failed, retrying");
                future = client->async_send_request(request);
            }
        }
        rate_->sleep();
    }

    auto start_time = ros_client_->nh_->now(); // tempo inicial
    rclcpp::Duration timeout = rclcpp::Duration::from_seconds(60.0);

    while (rclcpp::ok() &&
           landed_state_ != mavros_msgs::msg::ExtendedState::LANDED_STATE_ON_GROUND &&
           (ros_client_->nh_->now() - start_time) < timeout)
    {
        rclcpp::spin_some(ros_client_->nh_);
        rate_->sleep();
    }

    if ((ros_client_->nh_->now() - start_time) >= timeout)
        RCLCPP_WARN(nh_->get_logger(), "Landing failed, aborting");
    else
        RCLCPP_INFO(nh_->get_logger(), "Landing success");
}

void DroneControl::disarm()
{
    auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    request->value = false;

    while (rclcpp::ok() && current_state_.armed)
    {
        if (current_state_.armed && (ros_client_->nh_->now() - last_request_ > rclcpp::Duration::from_seconds(5.0)))
        {
            auto future = ros_client_->arming_client_->async_send_request(request);

            if (rclcpp::spin_until_future_complete(ros_client_->nh_, future) ==
                rclcpp::FutureReturnCode::SUCCESS)
            {
                if (future.get()->success)
                {
                    RCLCPP_INFO(nh_->get_logger(), "Vehicle disarmed");
                }
            }

            last_request_ = ros_client_->nh_->now();
        }

        rclcpp::spin_some(ros_client_->nh_);
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
    double roll, pitch, yaw;

    tf2::Quaternion q(
        local_position_.pose.orientation.x,
        local_position_.pose.orientation.y,
        local_position_.pose.orientation.z,
        local_position_.pose.orientation.w);

    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    return yaw;
}

double DroneControl::distance(const geometry_msgs::msg::PoseStamped &p1,
                              const geometry_msgs::msg::PoseStamped &p2)
{
    double dx = p1.pose.position.x - p2.pose.position.x;
    double dy = p1.pose.position.y - p2.pose.position.y;
    double dz = p1.pose.position.z - p2.pose.position.z;

    return std::sqrt(dx * dx + dy * dy + dz * dz);
}
