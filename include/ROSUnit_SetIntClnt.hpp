#pragma once

#include "ROSUnit.hpp"
#include "common_srv/set_int.h"
#include "IntegerMsg.hpp"

class ROSUnit_SetIntClnt : public ROSUnit{

    public:
        ROSUnit_SetIntClnt(std::string, ros::NodeHandle&);
        ~ROSUnit_SetIntClnt();
        void receive_msg_data(DataMessage* t_msg);

    private:

        ros::ServiceClient m_client;
};