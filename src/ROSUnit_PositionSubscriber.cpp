#include "ROSUnit_PositionSubscriber.hpp"
ROSUnit_PositionSubscriber* ROSUnit_PositionSubscriber::_instance_ptr = NULL;
PositionMsg ROSUnit_PositionSubscriber::position_msg;

ROSUnit_PositionSubscriber::ROSUnit_PositionSubscriber(ros::NodeHandle& t_main_handler) : ROSUnit(t_main_handler)  {

    _sub_position = t_main_handler.subscribe("body_position", 10, callbackPosition);
    _instance_ptr = this;

}

ROSUnit_PositionSubscriber::~ROSUnit_PositionSubscriber() {

}

void ROSUnit_PositionSubscriber::callbackPosition(const geometry_msgs::PointStamped& msg){

    position_msg.x = msg.point.x;
    position_msg.y = msg.point.y;
    position_msg.z = msg.point.z;

    _instance_ptr->emit_message((DataMessage*) &position_msg); 

}

void ROSUnit_PositionSubscriber::receive_msg_data(DataMessage* t_msg){

}