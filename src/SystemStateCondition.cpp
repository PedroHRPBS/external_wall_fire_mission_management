#include "SystemStateCondition.hpp"

SystemStateCondition::SystemStateCondition(internal_state* t_actual_state, internal_state t_check_state){
    m_check_state = t_check_state;
    m_actual_state = t_actual_state;
}

bool SystemStateCondition::isConditionMet(){
	return (*m_actual_state == m_check_state);
}

void SystemStateCondition::receive_msg_data(DataMessage* t_msg){

}
    
