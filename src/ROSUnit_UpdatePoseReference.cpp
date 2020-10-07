#include "ROSUnit_UpdatePoseReference.hpp"

ROSUnit_UpdatePoseReference::ROSUnit_UpdatePoseReference(ros::NodeHandle& t_main_handler) : ROSUnit(t_main_handler){

    _setpoint_position_client = t_main_handler.serviceClient<flight_controller::Update_Pose_Reference>("update_pose_reference");
}

ROSUnit_UpdatePoseReference::~ROSUnit_UpdatePoseReference() {

}

void ROSUnit_UpdatePoseReference::receive_msg_data(DataMessage* t_msg){
    
    if(t_msg->getType() == msg_type::USERREFERENCE){

        UpdatePoseMessage_FS* ref_msg = (UpdatePoseMessage_FS*)t_msg;

        flight_controller::Update_Pose_Reference srv;
        srv.request.setpoint_pose.x = ref_msg->getX();
        srv.request.setpoint_pose.y = ref_msg->getY();
        srv.request.setpoint_pose.z = ref_msg->getZ();
        srv.request.setpoint_pose.yaw = ref_msg->getYaw();
    
        bool success = _setpoint_position_client.call(srv);

        if (success)
        {
            ROS_INFO("NEW POSE REFERENCE PUBLISHED x: %f, y: %f, z: %f, yaw: %f",srv.request.setpoint_pose.x,
                                                                                 srv.request.setpoint_pose.y,
                                                                                 srv.request.setpoint_pose.z,
                                                                                 srv.request.setpoint_pose.yaw);
        }
        else 
        {
            ROS_ERROR("Failed to call service /update_pose_reference");
        }

    }
    

}
