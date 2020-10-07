#include "ROSUnit_ResetController.hpp"

ROSUnit_ResetController::ROSUnit_ResetController(ros::NodeHandle& t_main_handler) : ROSUnit(t_main_handler){

    _reset_controller_client = t_main_handler.serviceClient<flight_controller::Reset_Controller>("reset_controller");

}

ROSUnit_ResetController::~ROSUnit_ResetController() {

}

void ROSUnit_ResetController::receive_msg_data(DataMessage* t_msg){

    if(t_msg->getType() == msg_type::RestControllerMessage){

        ResetControllerMessage* _reset_msg = (ResetControllerMessage*)t_msg;
        
        flight_controller::Reset_Controller srv;

        MessageToBlock* _msg_block = (MessageToBlock*)_reset_msg;

        block_id id = _msg_block->target_block;

        srv.request.id = (int)id;

        bool success = _reset_controller_client.call(srv);

        if (success)
        {
            ROS_INFO("CONTROLLER RESET");
        }
        else 
        {
            ROS_ERROR("Failed to call service /update_controller");
        }

    }
}