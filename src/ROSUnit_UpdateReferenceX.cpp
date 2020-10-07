#include "ROSUnit_UpdateReferenceX_FS.hpp"

ROSUnit_UpdateReferenceX_FS::ROSUnit_UpdateReferenceX_FS(ros::NodeHandle& t_main_handler) : ROSUnit(t_main_handler){

    _setpoint_position_client = t_main_handler.serviceClient<flight_controller::Update_X_Reference>("update_reference/x");
}

ROSUnit_UpdateReferenceX_FS::~ROSUnit_UpdateReferenceX_FS() {

}

void ROSUnit_UpdateReferenceX_FS::receive_msg_data(DataMessage* t_msg){
    
    if(t_msg->getType() == msg_type::USERREFERENCE){

        UpdatePoseMessage_FS* ref_msg = (UpdatePoseMessage_FS*)t_msg;
        if(ref_msg->getRefType() == msg_type_reference::X){
            flight_controller::Update_X_Reference srv;
            srv.request.setpoint_x = ref_msg->getX();

            bool success = _setpoint_position_client.call(srv);

            if (success)
            {
                ROS_INFO("NEW X REFERENCE PUBLISHED: %f", srv.request.setpoint_x);
            }
            else 
            {
                ROS_ERROR("Failed to call service /update_reference/x");
            }
        }
    }
    

}
