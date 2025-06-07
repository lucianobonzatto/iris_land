#ifndef LAND_CONTROLLER_H
#define LAND_CONTROLLER_H

#include "general.h"
#include "controllers.h"

class Land_Controller
{
public:
    Land_Controller();
    ~Land_Controller();

    void print_parameters();
    geometry_msgs::Twist get_velocity(geometry_msgs::PoseStamped poseStamped);
    void update_parameters(iris_land::controllers_gain newParameters);
    void reset_altitude(double altitude);
    bool completed_approach();

private:
    PID_velocity_ctl pidController;
    Pose setpoint;
    int controller_mode;
    double distance_threshold;
    double angular_threshold;

    Speed get_align_velocity(Pose poseMeasurement);
    double calc_vel(double valor_in);
    double calculate_distance(const Pose& point1, const Pose& point2);
    double update_altitude(double altitude);
};

#endif // LAND_CONTROLLER_H
