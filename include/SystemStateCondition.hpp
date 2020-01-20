#pragma once
#include "Condition.hpp"
#include "common_types.hpp"
#include "IntegerMsg.hpp"

class SystemStateCondition: public Condition {

private:
	bool _isConditionMet = false;
    
public:
    int state = 0;

    bool isConditionMet();

    void receive_msg_data(DataMessage* t_msg);

};
