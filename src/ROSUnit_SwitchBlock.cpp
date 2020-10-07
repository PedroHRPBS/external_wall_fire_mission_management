#include "ROSUnit_SwitchBlock.hpp"

ROSUnit_SwitchBlock::ROSUnit_SwitchBlock(ros::NodeHandle& t_main_handler) : ROSUnit(t_main_handler) {
    _switch_client = t_main_handler.serviceClient<flight_controller::SwitchBlock>("switch_block");
}   

ROSUnit_SwitchBlock::~ROSUnit_SwitchBlock() {

}

void ROSUnit_SwitchBlock::receive_msg_data(DataMessage* t_msg){
    
    if(t_msg->getType() == msg_type::SWITCHBLOCK){

        SwitchBlockMsg_FS* _switch_msg = (SwitchBlockMsg_FS*)t_msg;
        
        flight_controller::SwitchBlock srv;
        srv.request.block_in = (int)(_switch_msg->getBlockToSwitchIn());
        srv.request.block_out = (int)(_switch_msg->getBlockToSwitchOut());

        bool success = _switch_client.call(srv);

        if (success)
        {
            ROS_INFO("SWITCHING BLOCKS. IN: %d, OUT: %d",srv.request.block_in, srv.request.block_out);
        }
        else 
        {
            ROS_ERROR("Failed to call service /switch_block");
        }
    }
    
}