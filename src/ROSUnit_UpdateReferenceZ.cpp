#include "ROSUnit_UpdateReferenceZ_FS.hpp"

ROSUnit_UpdateReferenceZ_FS::ROSUnit_UpdateReferenceZ_FS(ros::NodeHandle& t_main_handler) : ROSUnit(t_main_handler){

    _setpoint_position_client = t_main_handler.serviceClient<flight_controller::Update_Z_Reference>("update_reference/z");
}

ROSUnit_UpdateReferenceZ_FS::~ROSUnit_UpdateReferenceZ_FS() {

}

void ROSUnit_UpdateReferenceZ_FS::receive_msg_data(DataMessage* t_msg){
    
    if(t_msg->getType() == msg_type::USERREFERENCE){

        UpdatePoseMessage_FS* ref_msg = (UpdatePoseMessage_FS*)t_msg;
        if(ref_msg->getRefType() == msg_type_reference::Z){
            flight_controller::Update_Z_Reference srv;
            srv.request.setpoint_z = ref_msg->getZ();
        
            bool success = _setpoint_position_client.call(srv);

            if (success)
            {
                ROS_INFO("NEW Z REFERENCE PUBLISHED: %f", srv.request.setpoint_z);
            }
            else 
            {
                ROS_ERROR("Failed to call service /update_reference/z");
            }
        }
    }
    

}
