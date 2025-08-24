#include "general.h"

double get_yaw(const geometry_msgs::msg::Quaternion& quaternion) {
    tf2::Quaternion tf_quaternion(
        quaternion.x,
        quaternion.y,
        quaternion.z,
        quaternion.w);

    double roll, pitch, yaw;
    tf2::Matrix3x3(tf_quaternion).getRPY(roll, pitch, yaw);

    return yaw;
}