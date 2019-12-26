#pragma once
#include "ROSUnit.hpp"
#include "SwitchBlockMsg.hpp"
#include <positioning_system/SwitchBlock.h>

class ROSUnit_SwitchBlock :  public ROSUnit{

    private:
        ros::ServiceClient _switch_client;
        
    public:
        void receive_msg_data(DataMessage* t_msg);  
        
        ROSUnit_SwitchBlock(ros::NodeHandle&);
        ~ROSUnit_SwitchBlock();
};