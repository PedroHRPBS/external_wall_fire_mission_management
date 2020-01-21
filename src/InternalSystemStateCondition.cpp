#include "InternalSystemStateCondition.hpp"

InternalSystemStateCondition::InternalSystemStateCondition(internal_state* t_actual_state, internal_state t_check_state){
    m_check_state = t_check_state;
    m_actual_state = t_actual_state;
}

bool InternalSystemStateCondition::isConditionMet(){
	return (*m_actual_state == m_check_state);
}

void InternalSystemStateCondition::receive_msg_data(DataMessage* t_msg){

}
    
