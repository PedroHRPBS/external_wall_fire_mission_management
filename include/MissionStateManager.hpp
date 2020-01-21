#pragma once
#include "MsgEmitter.hpp"
#include "MsgExtWallFireState.hpp"
#include "common_types.hpp"
                                
class MissionStateManager : public msg_emitter
{
    private:
        external_wall_fire_states current_external_wall_fire_state;
    public:
        void updateMissionState(external_wall_fire_states);
        external_wall_fire_states getMissionState();
};

extern MissionStateManager MainMissionStateManager;