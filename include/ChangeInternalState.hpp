#pragma once
#include "internal_states.hpp"
#include "FlightElement.hpp"
#include "IntegerMsg.hpp"
#include "MissionStateManager.hpp"

class ChangeInternalState : public FlightElement {

private:
    external_wall_fire_states m_new_state;
    
public:
    void perform();
    void receive_msg_data(DataMessage*);

    ChangeInternalState(external_wall_fire_states);
    ~ChangeInternalState();
};