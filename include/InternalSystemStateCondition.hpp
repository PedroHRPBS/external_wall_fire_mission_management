#pragma once
#include "Condition.hpp"
#include "common_types.hpp"
#include "IntegerMsg.hpp"
#include "internal_states.hpp"

class InternalSystemStateCondition: public Condition {

private:
	bool _isConditionMet = false;
    internal_state m_check_state;
    internal_state* m_actual_state;

public:

    bool isConditionMet();

    void receive_msg_data(DataMessage* t_msg);

    InternalSystemStateCondition(internal_state*, internal_state);

};
