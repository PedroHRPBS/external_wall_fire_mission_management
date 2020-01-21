#include "MsgExtWallFireState.hpp"

msg_type MsgExtWallFireState::getType(){
    return msg_type::EXT_WALL_FIRE_STATE;
}
const int MsgExtWallFireState::getSize(){
    const int size_msg=4; //TODO Refactor
    return size_msg;
}