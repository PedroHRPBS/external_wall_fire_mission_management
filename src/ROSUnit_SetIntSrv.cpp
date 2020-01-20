#include "ROSUnit_SetIntSrv.hpp"

ROSUnit_SetIntSrv* ROSUnit_SetIntSrv::m_ptr;

ROSUnit_SetIntSrv::ROSUnit_SetIntSrv(std::string t_name, ros::NodeHandle& t_main_handler) : ROSUnit(t_main_handler)
{
    m_server = t_main_handler.advertiseService(t_name, srv_callback);
    m_ptr = this;
}

ROSUnit_SetIntSrv::~ROSUnit_SetIntSrv()
{

}

bool ROSUnit_SetIntSrv::srv_callback(common_srv::set_int::Request& req, common_srv::set_int::Response& res)
{
    IntegerMsg t_msg;
    t_msg.data = req.data;
    m_ptr->emit_message((DataMessage*) &t_msg);
    return true;
}