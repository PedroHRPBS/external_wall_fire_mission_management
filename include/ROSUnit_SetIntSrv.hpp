#pragma once

#include "ROSUnit.hpp"
#include "common_srv/set_int.h"
#include "IntegerMsg.hpp"

class ROSUnit_SetIntSrv : public ROSUnit{

    public:
        ROSUnit_SetIntSrv(std::string, ros::NodeHandle&);
        ~ROSUnit_SetIntSrv();
        void receive_msg_data(DataMessage* t_msg) {};

    private:

        static ROSUnit_SetIntSrv* m_ptr;
        ros::ServiceServer m_server;
        static bool srv_callback(common_srv::set_int::Request&, common_srv::set_int::Response&);
};