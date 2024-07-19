#ifndef DRONE_CONTROL_H
#define DRONE_CONTROL_H

#include "general.h"
#include "ros_client.h"

class DroneControl
{
public:
    DroneControl();
    ~DroneControl();

    void Init(ROSClient *drone_control);

private:
    ROSClient *ros_client_;
};

#endif // DRONE_CONTROL_H
