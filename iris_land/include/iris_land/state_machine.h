#ifndef STATES_MACHINE_H
#define STATES_MACHINE_H

#include "general.h"

class State_Machine {
public:
    State_Machine();
    ~State_Machine();

    STATES get_state();
    bool update_state(sensor_msgs::Joy newJoy);
    void land();

private:
    STATES state;

    bool STOPPED_update(sensor_msgs::Joy newJoy);
    bool LAND_update(sensor_msgs::Joy newJoy);
    bool LAND_CONTROL_update(sensor_msgs::Joy newJoy);
    bool FOLLOW_CONTROL_update(sensor_msgs::Joy newJoy);
    void swap_state(STATES new_state);
};

#endif //STATES_MACHINE_H

