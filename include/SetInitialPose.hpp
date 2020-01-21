#pragma once
#include "FlightElement.hpp"
#include "MessageToBlock.hpp"
#include "UpdatePoseMessage_FS.hpp"
#include "MessageToBlock.hpp"
#include "PositionMsg.hpp"
#include "Vector3DMessage.hpp"

class SetInitialPose : public FlightElement{

private:
	float _current_x, _current_y, _current_z, _current_yaw = 0;

public:
	block_id target_block;
    UpdatePoseMessage_FS pose_reference;

    void perform();

    void receive_msg_data(DataMessage* t_msg);
    
    SetInitialPose();
    ~SetInitialPose();
};
