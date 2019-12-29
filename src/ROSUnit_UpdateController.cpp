#include "ROSUnit_UpdateController.hpp"

ROSUnit_UpdateController::ROSUnit_UpdateController(ros::NodeHandle& t_main_handler) : ROSUnit(t_main_handler){

    _update_controller_client = t_main_handler.serviceClient<positioning_system::Update_Controller>("update_controller");

}

ROSUnit_UpdateController::~ROSUnit_UpdateController() {

}

void ROSUnit_UpdateController::receive_msg_data(DataMessage* t_msg){

    if(t_msg->getType() == msg_type::pid_data_update){

        ControllerMessage* _update_msg = (ControllerMessage*)t_msg;
        
        positioning_system::Update_Controller srv;

        srv.request.controller_parameters.pid_kp = _update_msg->getPIDParam().kp;
        srv.request.controller_parameters.pid_ki = _update_msg->getPIDParam().ki;
        srv.request.controller_parameters.pid_kd = _update_msg->getPIDParam().kd;
        srv.request.controller_parameters.pid_kdd = _update_msg->getPIDParam().kdd;
        srv.request.controller_parameters.pid_anti_windup = _update_msg->getPIDParam().anti_windup;
        srv.request.controller_parameters.pid_en_pv_derivation = _update_msg->getPIDParam().en_pv_derivation;
        srv.request.controller_parameters.id = (int)(_update_msg->getPIDParam().id);

        bool success = _update_controller_client.call(srv);

        if (success)
        {
            ROS_INFO("CONTROLLER UPDATED");
        }
        else 
        {
            ROS_ERROR("Failed to call service /update_controller");
        }

    }
}