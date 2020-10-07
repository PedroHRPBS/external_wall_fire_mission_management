#pragma once
#include "ROSUnit.hpp"
#include "flight_controller/Arm.h"
#include "ArmDataMessage.hpp"

class ROSUnit_Arm : public ROSUnit{

private:
    ros::ServiceClient _arm_client;

public:
    void receive_msg_data(DataMessage* t_msg);

    ROSUnit_Arm(ros::NodeHandle&);
    ~ROSUnit_Arm();
};