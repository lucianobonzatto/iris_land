#ifndef FOLLOW_CONTROLLER_H
#define FOLLOW_CONTROLLER_H

#include "general.h"
#include "tello_controllers.h"

class Follow_Controller
{
public:
    Follow_Controller();
    ~Follow_Controller();

    void print_parameters();
    geometry_msgs::Twist get_velocity(geometry_msgs::PoseStamped poseStamped, Speed drone_vel);    
    void update_parameters(iris_land::controllers_gain newParameters);

private:
    TelloPDController pdController;
    TelloCascadePDPIController cascadeController;
    TelloParallelPDPIController parallelController;
    TelloPIDController pidController;
    Pose setpoint;
    int controller_mode;

    double calc_vel(double valor);
};

#endif // FOLLOW_CONTROLLER_H
