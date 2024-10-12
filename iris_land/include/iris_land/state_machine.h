#ifndef STATES_MACHINE_H
#define STATES_MACHINE_H

#include "general.h"

class State_Machine
{
public:
    State_Machine();
    ~State_Machine();

    STATES get_state();
    bool update_state(mavros_msgs::RCIn rcStatus, string flight_mode, uint8_t landed_state);
    void land();

private:
    STATES state;

    bool STOPPED_update(mavros_msgs::RCIn rcStatus);
    bool LAND_update(mavros_msgs::RCIn rcStatus);
    bool LAND_CONTROL_update(mavros_msgs::RCIn rcStatus);
    bool FOLLOW_CONTROL_update(mavros_msgs::RCIn rcStatus);
    bool AWAITING_MODE_update(mavros_msgs::RCIn rcStatus);
    void swap_state(STATES new_state);
};

#endif // STATES_MACHINE_H
