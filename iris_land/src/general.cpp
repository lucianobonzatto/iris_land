#include "general.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

double get_yaw(const geometry_msgs::Quaternion& quaternion) {
    tf2::Quaternion tf_quaternion(
        quaternion.x,
        quaternion.y,
        quaternion.z,
        quaternion.w);

    double roll, pitch, yaw;
    tf2::Matrix3x3(tf_quaternion).getRPY(roll, pitch, yaw);

    return yaw;
}
