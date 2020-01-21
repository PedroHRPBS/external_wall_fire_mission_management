#pragma once
#include "internal_states.hpp"
#include "FlightElement.hpp"
#include "IntegerMsg.hpp"

class ChangeInternalState : public FlightElement {

private:
    internal_state* m_current_state_ptr;
    internal_state m_new_state;
    
public:
    void perform();
    void receive_msg_data(DataMessage*);

    ChangeInternalState(internal_state*, internal_state);
    ~ChangeInternalState();
};