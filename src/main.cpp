 #include "ros/ros.h"
#include <iostream>
#include "ROSUnit.hpp"
#include "logger.hpp"
#include "std_logger.hpp"
#include "FlightElement.hpp"
#include "Wait.hpp"
#include "WaitForCondition.hpp"
#include "Arm.hpp"
#include "FlightPipeline.hpp"
#include "SimplePlaneCondition.hpp"
#include "Disarm.hpp"
#include "FlightScenario.hpp"
#include "UpdateController.hpp"
#include "SetInitialPose.hpp"
#include "ResetController.hpp"
#include "SwitchBlock.hpp"
#include "ROSUnit_Arm.hpp"
#include "ROSUnit_UpdateController.hpp"
#include "ROSUnit_UpdatePoseReference.hpp"
#include "ROSUnit_PositionSubscriber.hpp"
#include "ROSUnit_ResetController.hpp"
#include "ROSUnit_SwitchBlock.hpp"
#include "ROSUnit_OrientationSubscriber.hpp"
#include "ROSUnit_UpdateReferenceX_FS.hpp"
#include "ROSUnit_UpdateReferenceY_FS.hpp"
#include "ROSUnit_UpdateReferenceZ_FS.hpp"
#include "ROSUnit_UpdateReferenceYaw_FS.hpp"
#include "SetReference_X.hpp"
#include "SetReference_Y.hpp"
#include "SetReference_Z.hpp"
#include "SetReference_Yaw.hpp"
#include "ROSUnit_FlightCommand.hpp"
#include "FlightCommand.hpp"
#include "ROSUnit_SetIntClnt.hpp"
#include "ROSUnit_SetIntSrv.hpp"
#include "SetMissionState.hpp"
#include "SystemStateCondition.hpp"

int main(int argc, char** argv) {
    Logger::assignLogger(new StdLogger());

    //****************ROS Units********************
    ros::init(argc, argv, "ex_bldg_fire_mm_node");
    ros::NodeHandle nh;

    ROSUnit* ros_arm_srv = new ROSUnit_Arm(nh);
    ROSUnit* ros_updt_ctr = new ROSUnit_UpdateController(nh);
    ROSUnit* ros_pos_sub = new ROSUnit_PositionSubscriber(nh);
    ROSUnit* ros_ori_sub = new ROSUnit_OrientationSubscriber(nh);
    ROSUnit* ros_rst_ctr = new ROSUnit_ResetController(nh);
    ROSUnit* ros_updt_x_ref = new ROSUnit_UpdateReferenceX_FS(nh);
    ROSUnit* ros_updt_y_ref = new ROSUnit_UpdateReferenceY_FS(nh);
    ROSUnit* ros_updt_z_ref = new ROSUnit_UpdateReferenceZ_FS(nh);
    ROSUnit* ros_updt_yaw_ref = new ROSUnit_UpdateReferenceYaw_FS(nh);
    ROSUnit* ros_flight_command = new ROSUnit_FlightCommand(nh);
    ROSUnit* ros_set_int_srv = new ROSUnit_SetIntSrv("/ex_bldg_fire_mm/set_system_state", nh);
    ROSUnit* ros_set_int_clnt = new ROSUnit_SetIntClnt("/ex_bldg_fire_mm/set_system_state",nh);

    //*****************Flight Elements*************

    FlightElement* update_controller_pid_x = new UpdateController();
    FlightElement* update_controller_pid_y = new UpdateController();
    FlightElement* update_controller_pid_z = new UpdateController();
    FlightElement* update_controller_pid_roll = new UpdateController();
    FlightElement* update_controller_pid_pitch = new UpdateController();
    FlightElement* update_controller_pid_yaw = new UpdateController();
    FlightElement* update_controller_pid_yaw_rate = new UpdateController();
    
    FlightElement* set_initial_pose = new SetInitialPose();
    
    FlightElement* reset_z = new ResetController();
    
    FlightElement* arm_motors = new Arm();
    FlightElement* disarm_motors = new Disarm();

    FlightElement* ref_z_on_takeoff = new SetReference_Z();
    FlightElement* ref_z_on_land = new SetReference_Z();

    FlightElement* flight_command = new FlightCommand();

    FlightElement* set_not_ready = new SetMissionState();
    ((SetMissionState*)set_not_ready)->setMS(0);
    FlightElement* set_ready_to_start = new SetMissionState();
    ((SetMissionState*)set_ready_to_start)->setMS(1);

    Wait wait_1s;
    wait_1s.wait_time_ms=1000;

    SimplePlaneCondition z_cross_takeoff_waypoint;
    z_cross_takeoff_waypoint.selected_dim=Dimension3D::Z;
    z_cross_takeoff_waypoint.condition_value = 1.4;
    z_cross_takeoff_waypoint.condition_met_for_larger=true;
    ros_pos_sub->add_callback_msg_receiver((msg_receiver*) &z_cross_takeoff_waypoint);

    WaitForCondition z_cross_takeoff_waypoint_check;
    z_cross_takeoff_waypoint_check.Wait_condition=(Condition*)&z_cross_takeoff_waypoint;

    SimplePlaneCondition z_cross_land_waypoint;
    z_cross_land_waypoint.selected_dim=Dimension3D::Z;
    z_cross_land_waypoint.condition_value=0.2;
    z_cross_land_waypoint.condition_met_for_larger=false;
    ros_pos_sub->add_callback_msg_receiver((msg_receiver*) &z_cross_land_waypoint);

    WaitForCondition z_cross_land_waypoint_check;
    z_cross_land_waypoint_check.Wait_condition=(Condition*)&z_cross_land_waypoint;

    WaitForCondition not_ready_check, ready_to_start_check, scanning_outdoor_check;
    WaitForCondition approach_outdoor_check, extinguish_outdoor_check, return_to_base_check;
    WaitForCondition error_check, finished_check;

    SystemStateCondition not_ready_condition;
    not_ready_condition.state = 0;
    SystemStateCondition ready_to_start_condition;
    ready_to_start_condition.state = 1;
    SystemStateCondition scanning_outdoor_condition;
    SystemStateCondition approach_outdoor_condition;
    SystemStateCondition extinguish_outdoor_condition;
    SystemStateCondition return_to_base_condition;
    SystemStateCondition error_condition;
    SystemStateCondition finished_condition;


    //******************Connections******************

    update_controller_pid_x->add_callback_msg_receiver((msg_receiver*) ros_updt_ctr);
    update_controller_pid_y->add_callback_msg_receiver((msg_receiver*) ros_updt_ctr);
    update_controller_pid_z->add_callback_msg_receiver((msg_receiver*) ros_updt_ctr);
    update_controller_pid_roll->add_callback_msg_receiver((msg_receiver*) ros_updt_ctr);
    update_controller_pid_pitch->add_callback_msg_receiver((msg_receiver*) ros_updt_ctr);
    update_controller_pid_yaw->add_callback_msg_receiver((msg_receiver*) ros_updt_ctr);
    update_controller_pid_yaw_rate->add_callback_msg_receiver((msg_receiver*) ros_updt_ctr);

    ros_ori_sub->add_callback_msg_receiver((msg_receiver*) set_initial_pose);
    ros_pos_sub->add_callback_msg_receiver((msg_receiver*) set_initial_pose);

    set_initial_pose->add_callback_msg_receiver((msg_receiver*) ros_updt_x_ref);
    set_initial_pose->add_callback_msg_receiver((msg_receiver*) ros_updt_y_ref);
    set_initial_pose->add_callback_msg_receiver((msg_receiver*) ros_updt_z_ref);
    set_initial_pose->add_callback_msg_receiver((msg_receiver*) ros_updt_yaw_ref);

    reset_z->add_callback_msg_receiver((msg_receiver*) ros_rst_ctr);

    arm_motors->add_callback_msg_receiver((msg_receiver*) ros_arm_srv);
    disarm_motors->add_callback_msg_receiver((msg_receiver*) ros_arm_srv);

    ref_z_on_takeoff->add_callback_msg_receiver((msg_receiver*)ros_updt_z_ref);
    ref_z_on_land->add_callback_msg_receiver((msg_receiver*)ros_updt_z_ref);

    ros_flight_command->add_callback_msg_receiver((msg_receiver*) flight_command);

    ros_set_int_srv->add_callback_msg_receiver((msg_receiver*)&not_ready_condition);
    ros_set_int_srv->add_callback_msg_receiver((msg_receiver*)&ready_to_start_condition);
    ros_set_int_srv->add_callback_msg_receiver((msg_receiver*)&scanning_outdoor_condition);
    ros_set_int_srv->add_callback_msg_receiver((msg_receiver*)&approach_outdoor_condition);
    ros_set_int_srv->add_callback_msg_receiver((msg_receiver*)&extinguish_outdoor_condition);
    ros_set_int_srv->add_callback_msg_receiver((msg_receiver*)&return_to_base_condition);
    ros_set_int_srv->add_callback_msg_receiver((msg_receiver*)&error_condition);
    ros_set_int_srv->add_callback_msg_receiver((msg_receiver*)&finished_condition);

    set_not_ready->add_callback_msg_receiver((msg_receiver*)ros_set_int_clnt);
    set_ready_to_start->add_callback_msg_receiver((msg_receiver*)ros_set_int_clnt);

    //*************Setting Flight Elements*************

    ((UpdateController*)update_controller_pid_x)->pid_data.kp = 0.8 * 0.3;
    ((UpdateController*)update_controller_pid_x)->pid_data.ki = 0.0;
    ((UpdateController*)update_controller_pid_x)->pid_data.kd = 0.6 * 0.3;
    ((UpdateController*)update_controller_pid_x)->pid_data.kdd = 0.0;
    ((UpdateController*)update_controller_pid_x)->pid_data.anti_windup = 0;
    ((UpdateController*)update_controller_pid_x)->pid_data.en_pv_derivation = 1;
    ((UpdateController*)update_controller_pid_x)->pid_data.id = block_id::PID_X;

    ((UpdateController*)update_controller_pid_y)->pid_data.kp = 0.8 * 0.3;
    ((UpdateController*)update_controller_pid_y)->pid_data.ki = 0.0;
    ((UpdateController*)update_controller_pid_y)->pid_data.kd = 0.6 * 0.3;
    ((UpdateController*)update_controller_pid_y)->pid_data.kdd = 0.0;
    ((UpdateController*)update_controller_pid_y)->pid_data.anti_windup = 0;
    ((UpdateController*)update_controller_pid_y)->pid_data.en_pv_derivation = 1;
    ((UpdateController*)update_controller_pid_y)->pid_data.id = block_id::PID_Y;

    ((UpdateController*)update_controller_pid_z)->pid_data.kp = 0.7450; 
    ((UpdateController*)update_controller_pid_z)->pid_data.ki = 0.0980; 
    ((UpdateController*)update_controller_pid_z)->pid_data.kd = 0.3956; 
    ((UpdateController*)update_controller_pid_z)->pid_data.kdd = 0.0;
    ((UpdateController*)update_controller_pid_z)->pid_data.anti_windup = 0;
    ((UpdateController*)update_controller_pid_z)->pid_data.en_pv_derivation = 1;
    ((UpdateController*)update_controller_pid_z)->pid_data.id = block_id::PID_Z;

    ((UpdateController*)update_controller_pid_roll)->pid_data.kp = 0.1584; 
    ((UpdateController*)update_controller_pid_roll)->pid_data.ki = 0.0;
    ((UpdateController*)update_controller_pid_roll)->pid_data.kd = 0.0264; 
    ((UpdateController*)update_controller_pid_roll)->pid_data.kdd = 0.0;
    ((UpdateController*)update_controller_pid_roll)->pid_data.anti_windup = 0;
    ((UpdateController*)update_controller_pid_roll)->pid_data.en_pv_derivation = 1;
    ((UpdateController*)update_controller_pid_roll)->pid_data.id = block_id::PID_ROLL;

    ((UpdateController*)update_controller_pid_pitch)->pid_data.kp = 0.1584; 
    ((UpdateController*)update_controller_pid_pitch)->pid_data.ki = 0.0;
    ((UpdateController*)update_controller_pid_pitch)->pid_data.kd = 0.0264;
    ((UpdateController*)update_controller_pid_pitch)->pid_data.kdd = 0.0;
    ((UpdateController*)update_controller_pid_pitch)->pid_data.anti_windup = 0;
    ((UpdateController*)update_controller_pid_pitch)->pid_data.en_pv_derivation = 1;
    ((UpdateController*)update_controller_pid_pitch)->pid_data.id = block_id::PID_PITCH;

    ((UpdateController*)update_controller_pid_yaw)->pid_data.kp = 0.8;
    ((UpdateController*)update_controller_pid_yaw)->pid_data.ki = 0.0;
    ((UpdateController*)update_controller_pid_yaw)->pid_data.kd = 0.0;
    ((UpdateController*)update_controller_pid_yaw)->pid_data.kdd = 0.0;
    ((UpdateController*)update_controller_pid_yaw)->pid_data.anti_windup = 0;
    ((UpdateController*)update_controller_pid_yaw)->pid_data.en_pv_derivation = 1;
    ((UpdateController*)update_controller_pid_yaw)->pid_data.id = block_id::PID_YAW;

    ((UpdateController*)update_controller_pid_yaw_rate)->pid_data.kp = 0.08;
    ((UpdateController*)update_controller_pid_yaw_rate)->pid_data.ki = 0.0;
    ((UpdateController*)update_controller_pid_yaw_rate)->pid_data.kd = 0.0;
    ((UpdateController*)update_controller_pid_yaw_rate)->pid_data.kdd = 0.0;
    ((UpdateController*)update_controller_pid_yaw_rate)->pid_data.anti_windup = 0;
    ((UpdateController*)update_controller_pid_yaw_rate)->pid_data.en_pv_derivation = 1;
    ((UpdateController*)update_controller_pid_yaw_rate)->pid_data.id = block_id::PID_YAW_RATE;

    ((ResetController*)reset_z)->target_block = block_id::PID_Z;

    ((SetReference_Z*)ref_z_on_takeoff)->setpoint_z = 1.5;
    ((SetReference_Z*)ref_z_on_land)->setpoint_z = 0.2;

   
    not_ready_check.Wait_condition = (Condition*)&not_ready_condition;
    ready_to_start_check.Wait_condition = (Condition*)&ready_to_start_condition;
    scanning_outdoor_check.Wait_condition = (Condition*)&scanning_outdoor_condition;
    approach_outdoor_check.Wait_condition = (Condition*)&approach_outdoor_condition;
    extinguish_outdoor_check.Wait_condition = (Condition*)&extinguish_outdoor_condition;
    return_to_base_check.Wait_condition = (Condition*)&return_to_base_condition;
    error_check.Wait_condition = (Condition*)&error_condition;
    finished_check.Wait_condition = (Condition*)&finished_condition;

    //**********************************************

    FlightPipeline not_ready_pipeline, ready_to_start_pipeline, scanning_outdoor_pipeline;
    FlightPipeline approach_outdoor_pipeline, extinguish_outdoor_pipeline, return_to_base_pipeline;
    FlightPipeline error_pipeline, finished_pipeline;


    //The Wait is needed because otherwise the set_initial_pose will capture only zeros
    not_ready_pipeline.addElement((FlightElement*)&not_ready_check);
    not_ready_pipeline.addElement((FlightElement*)&wait_1s);
    not_ready_pipeline.addElement((FlightElement*)set_initial_pose);
    not_ready_pipeline.addElement((FlightElement*)update_controller_pid_x);
    not_ready_pipeline.addElement((FlightElement*)update_controller_pid_y);
    not_ready_pipeline.addElement((FlightElement*)update_controller_pid_z);
    not_ready_pipeline.addElement((FlightElement*)update_controller_pid_roll);
    not_ready_pipeline.addElement((FlightElement*)update_controller_pid_pitch);
    not_ready_pipeline.addElement((FlightElement*)update_controller_pid_yaw);
    not_ready_pipeline.addElement((FlightElement*)update_controller_pid_yaw_rate);
    not_ready_pipeline.addElement((FlightElement*)flight_command);
    not_ready_pipeline.addElement((FlightElement*)&set_ready_to_start);
    
    // ready_to_start_pipeline.addElement((FlightElement*)&ready_to_start_check);
    // ready_to_start_pipeline.addElement((FlightElement*)ref_z_on_takeoff);
    // ready_to_start_pipeline.addElement((FlightElement*)reset_z);
    // ready_to_start_pipeline.addElement((FlightElement*)arm_motors);
    // ready_to_start_pipeline.addElement((FlightElement*)&z_cross_takeoff_waypoint_check);
    // ready_to_start_pipeline.addElement((FlightElement*)set_mission_state.setMissionStateMsg(2));

    // default_pipeline.addElement((FlightElement*)flight_command);

    // default_pipeline.addElement((FlightElement*)ref_z_on_land);
    // default_pipeline.addElement((FlightElement*)&z_cross_land_waypoint_check);
    // default_pipeline.addElement((FlightElement*)&wait_1s);
    // default_pipeline.addElement((FlightElement*)disarm_motors);

    FlightPipeline safety_pipeline;

    Logger::getAssignedLogger()->log("FlightScenario main_scenario",LoggerLevel::Info);
    FlightScenario main_scenario;
    main_scenario.AddFlightPipeline(&not_ready_pipeline);
    main_scenario.AddFlightPipeline(&safety_pipeline);
    main_scenario.StartScenario();
    Logger::getAssignedLogger()->log("Main Done",LoggerLevel::Info);
    
    while(ros::ok){
        ros::spinOnce();
    }
}
