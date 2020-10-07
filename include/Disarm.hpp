#pragma once
#include "FlightElement.hpp"
#include "ArmDataMessage.hpp"
#include "flight_controller/Arm.h"

class Disarm : public FlightElement{

private:
    ros::ServiceClient _disarm_client;

public:

    void perform();
    void receive_msg_data(DataMessage* t_msg);
    
    //Disarm(ros::NodeHandle&);
    Disarm();
    ~Disarm();
};