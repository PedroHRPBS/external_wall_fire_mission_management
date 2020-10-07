#pragma once
#include <flight_controller/Update_Controller_PID.h>
#include "ROSUnit.hpp"
#include "ControllerMessage.hpp"
#include <flight_controller/PID_param.h>

class ROSUnit_UpdateController : public ROSUnit {

private:
    ros::ServiceClient _update_controller_client;

public:
    void receive_msg_data(DataMessage* t_msg);
    ROSUnit_UpdateController(ros::NodeHandle&);
    ~ROSUnit_UpdateController();
};