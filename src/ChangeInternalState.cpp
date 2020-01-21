#include "ChangeInternalState.hpp"

ChangeInternalState::ChangeInternalState(internal_state* t_current_state_ptr, internal_state t_new_state) {
    m_current_state_ptr = t_current_state_ptr;
    m_new_state = t_new_state;
}

ChangeInternalState::~ChangeInternalState() {

}

void ChangeInternalState::perform() {
    *m_current_state_ptr = m_new_state;
    std::cout << "Current state: " << (int)*m_current_state_ptr << std::endl;
}

void ChangeInternalState::receive_msg_data(DataMessage* t_msg){
    
    if(t_msg->getType() == msg_type::INTEGER){
        IntegerMsg* int_msg = (IntegerMsg*)t_msg;

        if(int_msg->data == (int)m_new_state){
            *m_current_state_ptr = static_cast<internal_state>(int_msg->data);
        }
    }
}