#include "general.h"
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2/LinearMath/Matrix3x3.h>

double get_yaw(const geometry_msgs::msg::Quaternion &q)
{
    tf2::Quaternion tf_quat(q.x, q.y, q.z, q.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
    return yaw;
}