#ifndef FOLLOW_CONTROLLER_H
#define FOLLOW_CONTROLLER_H

#include "general.h"
#include "controllers.h"

class Follow_Controller
{
public:
    Follow_Controller();
    ~Follow_Controller();

    void print_parameters();
    geometry_msgs::Twist get_velocity(geometry_msgs::PoseStamped poseStamped);    
    void update_parameters(iris_land::controllers_gain newParameters);

private:
    PID_velocity_ctl pidController;
    Pose setpoint;

    double calc_vel(double valor);
};

#endif // FOLLOW_CONTROLLER_H
