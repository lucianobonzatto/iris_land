#include "state_machine.h"

State_Machine::State_Machine()
{
    state = STATES::STOPPED;
}

State_Machine::~State_Machine()
{

}

STATES State_Machine::get_state()
{
    return state;
}

bool State_Machine::update_state(mavros_msgs::RCIn rcStatus)
{
    if(rcStatus.header.stamp.isZero()){return false;}
    switch (state)
    {
    case STATES::STOPPED:        return STOPPED_update(rcStatus);        break;
    case STATES::LAND:           return LAND_update(rcStatus);           break;
    case STATES::LAND_CONTROL:   return LAND_CONTROL_update(rcStatus);   break;
    case STATES::FOLLOW_CONTROL: return FOLLOW_CONTROL_update(rcStatus); break;
    default: break;
    }
    return false;
}

void State_Machine::land()
{
    swap_state(STATES::LAND);
}

// private methods
bool State_Machine::STOPPED_update(mavros_msgs::RCIn rcStatus)
{
    KEY_POSITION position = IDENTIFY_STATE_KEY_POSITION(rcStatus.channels[STATE_KEY]);
    if(position == OUT){swap_state(STATES::STOPPED); return true;}
    else if(position == UC){swap_state(STATES::STOPPED); return true;}
    else if(position == UB){swap_state(STATES::STOPPED); return true;}
    else if(position == UA){swap_state(STATES::STOPPED); return true;}
    else if(position == DA){swap_state(STATES::STOPPED); return true;}
    else if(position == DB){swap_state(STATES::STOPPED); return true;}
    else if(position == DC){swap_state(STATES::STOPPED); return true;}
    
    return false;
}

bool State_Machine::LAND_update(mavros_msgs::RCIn rcStatus)
{
    KEY_POSITION position = IDENTIFY_STATE_KEY_POSITION(rcStatus.channels[STATE_KEY]);
    if(position == OUT){swap_state(STATES::STOPPED); return true;}
    else if(position == UC){swap_state(STATES::STOPPED); return true;}
    else if(position == UB){swap_state(STATES::STOPPED); return true;}
    else if(position == UA){swap_state(STATES::STOPPED); return true;}
    else if(position == DA){swap_state(STATES::STOPPED); return true;}
    else if(position == DB){swap_state(STATES::STOPPED); return true;}
    else if(position == DC){swap_state(STATES::STOPPED); return true;}
    
    return false;
}

bool State_Machine::LAND_CONTROL_update(mavros_msgs::RCIn rcStatus)
{
    KEY_POSITION position = IDENTIFY_STATE_KEY_POSITION(rcStatus.channels[STATE_KEY]);
    if(position == OUT){swap_state(STATES::STOPPED); return true;}
    else if(position == UC){swap_state(STATES::STOPPED); return true;}
    else if(position == UB){swap_state(STATES::STOPPED); return true;}
    else if(position == UA){swap_state(STATES::STOPPED); return true;}
    else if(position == DA){swap_state(STATES::STOPPED); return true;}
    else if(position == DB){swap_state(STATES::STOPPED); return true;}
    else if(position == DC){swap_state(STATES::STOPPED); return true;}
    
    return false;
}

bool State_Machine::FOLLOW_CONTROL_update(mavros_msgs::RCIn rcStatus)
{
    KEY_POSITION position = IDENTIFY_STATE_KEY_POSITION(rcStatus.channels[STATE_KEY]);
    if(position == OUT){swap_state(STATES::STOPPED); return true;}
    else if(position == UC){swap_state(STATES::STOPPED); return true;}
    else if(position == UB){swap_state(STATES::STOPPED); return true;}
    else if(position == UA){swap_state(STATES::STOPPED); return true;}
    else if(position == DA){swap_state(STATES::STOPPED); return true;}
    else if(position == DB){swap_state(STATES::STOPPED); return true;}
    else if(position == DC){swap_state(STATES::STOPPED); return true;}
    
    return false;
}

void State_Machine::swap_state(STATES new_state)
{
    std::cout << "swap \t" << states_name[state] << "\t->\t" << states_name[new_state] << std::endl;
    state = new_state;
}