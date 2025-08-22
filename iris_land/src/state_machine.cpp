#include "state_machine.h"

State_Machine::State_Machine()
{
    state = STATES::AWAITING_MODE;
}

State_Machine::~State_Machine()
{

}

STATES State_Machine::get_state()
{
    return state;
}

bool State_Machine::update_state(mavros_msgs::msg::RCIn rcStatus, string flight_mode, uint8_t landed_state)
{
    if((landed_state != CORRECT_LAND_STATE) || (flight_mode != CORRECT_FLIGHT_MODE))
    {
        swap_state(STATES::AWAITING_MODE);
    }

    if(rcStatus.header.stamp.sec == 0 && rcStatus.header.stamp.nanosec == 0){ return false; }
    switch (state)
    {
    case STATES::STOPPED:        return STOPPED_update(rcStatus);        break;
    case STATES::LAND:           return LAND_update(rcStatus);           break;
    case STATES::LAND_CONTROL:   return LAND_CONTROL_update(rcStatus);   break;
    case STATES::FOLLOW_CONTROL: return FOLLOW_CONTROL_update(rcStatus); break;
    case STATES::AWAITING_MODE:  return AWAITING_MODE_update(rcStatus, flight_mode, landed_state);  break;
    default: break;
    }
    return false;
}

void State_Machine::land()
{
    swap_state(STATES::LAND);
}

// private methods
bool State_Machine::STOPPED_update(mavros_msgs::msg::RCIn rcStatus)
{
    KEY_POSITION position = IDENTIFY_STATE_KEY_POSITION(rcStatus.channels[STATE_KEY]);
    if(position == OUT){swap_state(STATES::AWAITING_MODE); return true;}
    // else if(position == P1){swap_state(STATES::STOPPED); return true;}
    else if(position == P2){swap_state(STATES::LAND_CONTROL); return true;}
    else if(position == P3){swap_state(STATES::FOLLOW_CONTROL); return true;}
    
    return false;
}

bool State_Machine::LAND_update(mavros_msgs::msg::RCIn rcStatus)
{
    KEY_POSITION position = IDENTIFY_STATE_KEY_POSITION(rcStatus.channels[STATE_KEY]);
    if(position == OUT){swap_state(STATES::AWAITING_MODE); return true;}
    
    return false;
}

bool State_Machine::LAND_CONTROL_update(mavros_msgs::msg::RCIn rcStatus)
{
    KEY_POSITION position = IDENTIFY_STATE_KEY_POSITION(rcStatus.channels[STATE_KEY]);
    if(position == OUT){swap_state(STATES::AWAITING_MODE); return true;}
    else if(position == P1){swap_state(STATES::STOPPED); return true;}
    // else if(position == P2){swap_state(STATES::LAND_CONTROL); return true;}
    else if(position == P3){swap_state(STATES::FOLLOW_CONTROL); return true;}
    
    return false;
}

bool State_Machine::FOLLOW_CONTROL_update(mavros_msgs::msg::RCIn rcStatus)
{
    KEY_POSITION position = IDENTIFY_STATE_KEY_POSITION(rcStatus.channels[STATE_KEY]);
    if(position == OUT){swap_state(STATES::AWAITING_MODE); return true;}
    else if(position == P1){swap_state(STATES::STOPPED); return true;}
    else if(position == P2){swap_state(STATES::LAND_CONTROL); return true;}
    // else if(position == P3){swap_state(STATES::FOLLOW_CONTROL); return true;}
    
    return false;
}

bool State_Machine::AWAITING_MODE_update(mavros_msgs::msg::RCIn rcStatus, string flight_mode, uint8_t landed_state)
{
    if((landed_state == CORRECT_LAND_STATE) && (flight_mode == CORRECT_FLIGHT_MODE))
    {
        swap_state(STATES::STOPPED);
    }
    return false;
}

void State_Machine::swap_state(STATES new_state)
{
    if(new_state != state)
    {
        std::cout << "swap \t" << states_name[state] << "\t->\t" << states_name[new_state] << std::endl;
        state = new_state;
    }
}