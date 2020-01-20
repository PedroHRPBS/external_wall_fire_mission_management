#include "SystemStateCondition.hpp"

bool SystemStateCondition::isConditionMet(){
	return _isConditionMet;
}

void SystemStateCondition::receive_msg_data(DataMessage* t_msg){

	if(t_msg->getType() == msg_type::INTEGER){
        IntegerMsg* int_msg = (IntegerMsg*)t_msg;

        if(int_msg->data == state){
            _isConditionMet = true;
        }
    }
}
    
