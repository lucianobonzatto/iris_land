#include "drone_control.h"

DroneControl::DroneControl()
{
}

DroneControl::~DroneControl()
{
}

void DroneControl::Init(ROSClient *drone_control)
{
    ros_client_ = drone_control;
}
