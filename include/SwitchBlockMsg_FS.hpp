#pragma once
#include "DataMessage.hpp"

class SwitchBlockMsg_FS : public DataMessage {

private:
    msg_type _type;
    block_id _block_in, _block_out;

public:

    msg_type getType();
    const int getSize();
    void setSwitchBlockMsg_FS(block_id t_block_out, block_id t_block_in);
    block_id getBlockToSwitchIn();
    block_id getBlockToSwitchOut();
    SwitchBlockMsg_FS();
    ~SwitchBlockMsg_FS();
};