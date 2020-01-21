#pragma once
#include "DataMessage.hpp"
#include "common_types.hpp"

class MsgExtWallFireState : public DataMessage{
public:
    external_wall_fire_states msg_external_wall_fire_state;
    msg_type getType();
    const int getSize();
};