#pragma once
#include "ROSUnit.hpp"
#include "flight_controller/Waypoint.h"
#include <flight_controller/Update_Pose_Reference.h>
#include "UpdatePoseMessage_FS.hpp"
#include "MessageToBlock.hpp"

class ROSUnit_UpdatePoseReference : public ROSUnit{

private:
    ros::ServiceClient _setpoint_position_client;

public:
    void receive_msg_data(DataMessage* t_msg);
    

    ROSUnit_UpdatePoseReference(ros::NodeHandle&);
    ~ROSUnit_UpdatePoseReference();
};