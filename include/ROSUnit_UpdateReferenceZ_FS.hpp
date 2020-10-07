#pragma once
#include "ROSUnit.hpp"
#include "flight_controller/Waypoint.h"
#include <flight_controller/Update_Z_Reference.h>
#include "UpdatePoseMessage_FS.hpp"
#include "MessageToBlock.hpp"

class ROSUnit_UpdateReferenceZ_FS : public ROSUnit{

private:
    ros::ServiceClient _setpoint_position_client;

public:
    void receive_msg_data(DataMessage* t_msg);
    

    ROSUnit_UpdateReferenceZ_FS(ros::NodeHandle&);
    ~ROSUnit_UpdateReferenceZ_FS();
};