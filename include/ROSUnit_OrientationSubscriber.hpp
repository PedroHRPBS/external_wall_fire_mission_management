#pragma once
#include "ROSUnit.hpp"
#include <geometry_msgs/PointStamped.h>
#include "Vector3DMessage.hpp"

class ROSUnit_OrientationSubscriber : public ROSUnit {

private:  
    ros::Subscriber _sub_orientation;
    static ROSUnit_OrientationSubscriber* _instance_ptr;
    static Vector3DMessage orientation_msg; 
    static void callbackOrientation(const geometry_msgs::PointStamped& msg);
       
public:

    void receive_msg_data(DataMessage*);
    ROSUnit_OrientationSubscriber(ros::NodeHandle&);
    ~ROSUnit_OrientationSubscriber();
};