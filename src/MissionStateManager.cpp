#include "MissionStateManager.hpp"
MissionStateManager MainMissionStateManager;

void MissionStateManager::updateMissionState(external_wall_fire_states t_current_state){
    current_external_wall_fire_state = t_current_state;
    MsgExtWallFireState state_msg;
    state_msg.msg_external_wall_fire_state=t_current_state;
    emit_message(&state_msg);
}

external_wall_fire_states MissionStateManager::getMissionState(){
    return current_external_wall_fire_state;
}
