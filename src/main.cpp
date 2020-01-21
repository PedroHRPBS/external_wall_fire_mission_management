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
#include "SetMissionState.hpp"
#include "InternalSystemStateCondition.hpp"
#include "internal_states.hpp"
#include "ChangeInternalState.hpp"
#include "ExternalSystemStateCondition.hpp"
#include "SendMessage.hpp"
#include "EmptyMsg.hpp"
#include "ROSUnit_Factory.hpp"
#include "MissionStateManager.hpp"

int main(int argc, char** argv) {
    Logger::assignLogger(new StdLogger());

    internal_state current_state = internal_state::NOT_READY;
    internal_state* current_state_ptr = &current_state;

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

    ROSUnit_Factory ROSUnit_Factory_main{nh};
	ROSUnit* ros_set_system_state_srv = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Server_Publisher, ROSUnit_msg_type::ROSUnit_Int, "/ex_bldg_fire_mm/set_system_state");
	ROSUnit* ros_updt_fire_detection_state_srv = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Server_Publisher, ROSUnit_msg_type::ROSUnit_Int, "/ex_bldg_fire_mm/update_fire_detection_state");
    ROSUnit* ros_updt_uav_control_state_srv = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Server_Publisher, ROSUnit_msg_type::ROSUnit_Int, "/ex_bldg_fire_mm/update_uav_control_state");
    ROSUnit* ros_updt_water_ext_state_srv = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Server_Publisher, ROSUnit_msg_type::ROSUnit_Int, "/ex_bldg_fire_mm/update_water_ext_state");
    ROSUnit* ros_updt_outdoor_nav_state_srv = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Server_Publisher, ROSUnit_msg_type::ROSUnit_Int, "/ex_bldg_fire_mm/update_outdoor_nav_state");
    ROSUnit* ros_updt_water_level_srv = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Server_Publisher, ROSUnit_msg_type::ROSUnit_Int, "/ex_bldg_fire_mm/update_water_level");
    
    ROSUnit* ros_set_fire_detection_state_clnt = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Client_Subscriber, ROSUnit_msg_type::ROSUnit_Int, "/fire_detection/set_mission_state");
    ROSUnit* ros_trigger_uav_scan_path_clnt = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Client_Subscriber, ROSUnit_msg_type::ROSUnit_Empty, "/outdoor_nav/upload_uav_scan_path");
    ROSUnit* ros_trigger_uav_fire_path_clnt = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Client_Subscriber, ROSUnit_msg_type::ROSUnit_Int, "/outdoor_nav/upload_uav_fire_paths");
    ROSUnit* ros_trigger_uav_home_path_clnt = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Client_Subscriber, ROSUnit_msg_type::ROSUnit_Empty, "/outdoor_nav/upload_uav_home_path");
    ROSUnit* set_fire_extinguishing_state_clnt = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Client_Subscriber, ROSUnit_msg_type::ROSUnit_Int, "/water_ext/set_mission_state");
    
    //*****************Flight Elements*************
    //TODO send parameters of flightElements on the constructor.
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

    FlightElement* cs_to_not_ready = new ChangeInternalState(MainMissionStateManager, internal_state::NOT_READY);
    FlightElement* cs_to_ready_to_start = new ChangeInternalState(MainMissionStateManager, internal_state::READY_TO_START);
    FlightElement* cs_to_scanning_outdoor = new ChangeInternalState(MainMissionStateManager, internal_state::SCANNING_OUTDOOR);
    FlightElement* cs_to_approaching_outdoor = new ChangeInternalState(MainMissionStateManager, internal_state::APPROACHING_OUTDOOR);
    FlightElement* cs_to_extinguishing_outdoor = new ChangeInternalState(MainMissionStateManager, internal_state::EXTINGUISHING_OUTDOOR);
    FlightElement* cs_to_return_to_base = new ChangeInternalState(MainMissionStateManager, internal_state::RETURNING_TO_BASE);
    FlightElement* cs_to_finished = new ChangeInternalState(MainMissionStateManager, internal_state::FINISHED);
    FlightElement* cs_to_error = new ChangeInternalState(MainMissionStateManager, internal_state::ERROR);

    IntegerMsg ignoring_state;
    ignoring_state.data = 1;
    FlightElement* set_ignoring_state_outdoor_fire_detection = new SendMessage((DataMessage*)&ignoring_state);
    
    EmptyMsg uav_scan_path;
    FlightElement* trigger_upload_uav_scan_path = new SendMessage((DataMessage*)&uav_scan_path);
    
    IntegerMsg scanning_state;
    scanning_state.data = 2;
    FlightElement* set_scanning_state_outdoor_fire_detection = new SendMessage((DataMessage*)&scanning_state);
    
    IntegerMsg uav_fire_tag;
    uav_fire_tag.data = 0;
    FlightElement* trigger_upload_uav_fire_path = new SendMessage((DataMessage*)&uav_fire_tag);

    IntegerMsg armed_extinguishing_state;
    armed_extinguishing_state.data = 3;
    FlightElement* set_arming_ext_state_fire_extinguishing = new SendMessage((DataMessage*)&armed_extinguishing_state);

    EmptyMsg uav_home_path;
    FlightElement* trigger_upload_uav_home_path = new SendMessage((DataMessage*)&uav_home_path);

    Wait wait_1s;
    wait_1s.wait_time_ms=1000;

    SimplePlaneCondition z_cross_takeoff_waypoint;
    z_cross_takeoff_waypoint.selected_dim=Dimension3D::Z;
    z_cross_takeoff_waypoint.condition_value = 1.4;
    z_cross_takeoff_waypoint.condition_met_for_larger=true;
    ros_pos_sub->add_callback_msg_receiver((msg_receiver*) &z_cross_takeoff_waypoint);
    WaitForCondition* z_cross_takeoff_waypoint_check = new WaitForCondition((Condition*)&z_cross_takeoff_waypoint);

    SimplePlaneCondition z_cross_land_waypoint;
    z_cross_land_waypoint.selected_dim=Dimension3D::Z;
    z_cross_land_waypoint.condition_value=0.2;
    z_cross_land_waypoint.condition_met_for_larger=false;
    WaitForCondition* z_cross_land_waypoint_check = new WaitForCondition((Condition*)&z_cross_land_waypoint);


    InternalSystemStateCondition* not_ready_condition = new InternalSystemStateCondition(MainMissionStateManager, internal_state::NOT_READY);
    WaitForCondition* not_ready_check = new WaitForCondition((Condition*)not_ready_condition);

    InternalSystemStateCondition* ready_to_start_condition = new InternalSystemStateCondition(MainMissionStateManager, internal_state::READY_TO_START);
    WaitForCondition* ready_to_start_check = new WaitForCondition((Condition*)ready_to_start_condition);

    InternalSystemStateCondition* scanning_outdoor_condition = new InternalSystemStateCondition(MainMissionStateManager, internal_state::SCANNING_OUTDOOR);
    WaitForCondition* scanning_outdoor_check = new WaitForCondition((Condition*)scanning_outdoor_condition);

    InternalSystemStateCondition* approach_outdoor_condition = new InternalSystemStateCondition(MainMissionStateManager, internal_state::APPROACHING_OUTDOOR);
    WaitForCondition* approach_outdoor_check = new WaitForCondition((Condition*)approach_outdoor_condition);

    InternalSystemStateCondition* extinguish_outdoor_condition = new InternalSystemStateCondition(MainMissionStateManager, internal_state::EXTINGUISHING_OUTDOOR);
    WaitForCondition* extinguish_outdoor_check = new WaitForCondition((Condition*)extinguish_outdoor_condition);

    InternalSystemStateCondition* return_to_base_condition = new InternalSystemStateCondition(MainMissionStateManager, internal_state::RETURNING_TO_BASE);
    WaitForCondition* return_to_base_check = new WaitForCondition((Condition*)return_to_base_condition);

    InternalSystemStateCondition* error_condition = new InternalSystemStateCondition(MainMissionStateManager, internal_state::ERROR);
    WaitForCondition* error_check = new WaitForCondition((Condition*)error_condition);

    InternalSystemStateCondition* finished_condition = new InternalSystemStateCondition(MainMissionStateManager, internal_state::FINISHED);
    WaitForCondition* finished_check = new WaitForCondition((Condition*)finished_condition);

    //The int values passed on the following constructors, need to match the system states of the external systems.
    ExternalSystemStateCondition* outdoor_wall_fire_detection_idle = new ExternalSystemStateCondition(0); //OUTDOOR_WALL_FIRE_DETECTION SYSTEM STATE: IDLE
    WaitForCondition* outdoor_wall_fire_detection_idle_check = new WaitForCondition((Condition*)outdoor_wall_fire_detection_idle);
    
    ExternalSystemStateCondition* uav_control_landed = new ExternalSystemStateCondition(1); //UAV_CONTROL SYSTEM STATE: LANDED
    WaitForCondition* uav_control_landed_check = new WaitForCondition((Condition*)uav_control_landed);
    
    ExternalSystemStateCondition* uav_control_following_trajectory = new ExternalSystemStateCondition(6); //UAV_CONTROL SYSTEM STATE: FOLLOWING TRAJECTORY
    WaitForCondition* uav_control_following_trajectory_check = new WaitForCondition((Condition*)uav_control_following_trajectory);

    ExternalSystemStateCondition* uav_control_hovering = new ExternalSystemStateCondition(7); //UAV_CONTROL SYSTEM STATE: HOVERING
    WaitForCondition* uav_control_hovering_check = new WaitForCondition((Condition*)uav_control_hovering);

    ExternalSystemStateCondition* water_fire_extinguishing_idle = new ExternalSystemStateCondition(0); //WATER_FIRE_EXTINGUISING SYSTEM STATE: IDLE
    WaitForCondition* water_fire_extinguishing_idle_check = new WaitForCondition((Condition*)water_fire_extinguishing_idle);

    ExternalSystemStateCondition* water_fire_extinguishing_extinguished = new ExternalSystemStateCondition(4); //WATER_FIRE_EXTINGUISING SYSTEM STATE: ARMED EXTINGUISHED
    WaitForCondition* water_fire_extinguishing_extinguished_check = new WaitForCondition((Condition*)water_fire_extinguishing_extinguished);
    
    ExternalSystemStateCondition* outdoor_navigation_idle = new ExternalSystemStateCondition(0); //OUTDOOR_NAVIGATION SYSTEM STATE: IDLE
    WaitForCondition* outdoor_navigation_idle_check = new WaitForCondition((Condition*)outdoor_navigation_idle);
    
    ExternalSystemStateCondition* outdoor_navigation_all_wall_fire = new ExternalSystemStateCondition(1); //OUTDOOR_NAVIGATION SYSTEM STATE: ALL WALL FIRES DETECTED
    WaitForCondition* outdoor_navigation_all_wall_fire_check = new WaitForCondition((Condition*)outdoor_navigation_all_wall_fire);

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

    ros_set_system_state_srv->add_callback_msg_receiver((msg_receiver*)cs_to_not_ready);
    ros_set_system_state_srv->add_callback_msg_receiver((msg_receiver*)cs_to_ready_to_start);
    ros_set_system_state_srv->add_callback_msg_receiver((msg_receiver*)cs_to_scanning_outdoor);
    ros_set_system_state_srv->add_callback_msg_receiver((msg_receiver*)cs_to_approaching_outdoor);
    ros_set_system_state_srv->add_callback_msg_receiver((msg_receiver*)cs_to_extinguishing_outdoor);
    ros_set_system_state_srv->add_callback_msg_receiver((msg_receiver*)cs_to_return_to_base);
    ros_set_system_state_srv->add_callback_msg_receiver((msg_receiver*)cs_to_finished);
    ros_set_system_state_srv->add_callback_msg_receiver((msg_receiver*)cs_to_error);

    ros_updt_fire_detection_state_srv->add_callback_msg_receiver((msg_receiver*)outdoor_wall_fire_detection_idle);

    ros_updt_uav_control_state_srv->add_callback_msg_receiver((msg_receiver*)uav_control_landed);
    ros_updt_uav_control_state_srv->add_callback_msg_receiver((msg_receiver*)uav_control_following_trajectory);
    ros_updt_uav_control_state_srv->add_callback_msg_receiver((msg_receiver*)uav_control_hovering);

    ros_updt_water_ext_state_srv->add_callback_msg_receiver((msg_receiver*)water_fire_extinguishing_idle);
    ros_updt_water_ext_state_srv->add_callback_msg_receiver((msg_receiver*)water_fire_extinguishing_extinguished);

    ros_updt_outdoor_nav_state_srv->add_callback_msg_receiver((msg_receiver*)outdoor_navigation_idle);
    ros_updt_outdoor_nav_state_srv->add_callback_msg_receiver((msg_receiver*)outdoor_navigation_all_wall_fire);

    set_ignoring_state_outdoor_fire_detection->add_callback_msg_receiver((msg_receiver*)ros_set_fire_detection_state_clnt);
    trigger_upload_uav_scan_path->add_callback_msg_receiver((msg_receiver*)ros_trigger_uav_scan_path_clnt);
    set_scanning_state_outdoor_fire_detection->add_callback_msg_receiver((msg_receiver*)ros_set_fire_detection_state_clnt);
    trigger_upload_uav_fire_path->add_callback_msg_receiver((msg_receiver*)ros_trigger_uav_fire_path_clnt);
    set_arming_ext_state_fire_extinguishing->add_callback_msg_receiver((msg_receiver*)set_fire_extinguishing_state_clnt);
    trigger_upload_uav_home_path->add_callback_msg_receiver((msg_receiver*)ros_trigger_uav_home_path_clnt);


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

    //**********************************************
    FlightPipeline dumb_pipeline;
    FlightPipeline not_ready_pipeline, ready_to_start_pipeline, scanning_outdoor_pipeline;
    FlightPipeline approach_outdoor_pipeline, extinguish_outdoor_pipeline, return_to_base_pipeline;
    FlightPipeline error_pipeline, finished_pipeline;

    int duty = 2;

    //Check Current Mission State
    not_ready_pipeline.addElement((FlightElement*)not_ready_check);
    //Initialise UAV position and controllers
    not_ready_pipeline.addElement((FlightElement*)&wait_1s); //The Wait is needed because otherwise the set_initial_pose will capture only zeros
    not_ready_pipeline.addElement((FlightElement*)set_initial_pose);
    not_ready_pipeline.addElement((FlightElement*)update_controller_pid_x);
    not_ready_pipeline.addElement((FlightElement*)update_controller_pid_y);
    not_ready_pipeline.addElement((FlightElement*)update_controller_pid_z);
    not_ready_pipeline.addElement((FlightElement*)update_controller_pid_roll);
    not_ready_pipeline.addElement((FlightElement*)update_controller_pid_pitch);
    not_ready_pipeline.addElement((FlightElement*)update_controller_pid_yaw);
    not_ready_pipeline.addElement((FlightElement*)update_controller_pid_yaw_rate);
    //Check if all systems are ready
    not_ready_pipeline.addElement((FlightElement*)outdoor_wall_fire_detection_idle_check);
    not_ready_pipeline.addElement((FlightElement*)uav_control_landed_check);
    not_ready_pipeline.addElement((FlightElement*)water_fire_extinguishing_idle_check);
    not_ready_pipeline.addElement((FlightElement*)outdoor_navigation_idle_check);
    //Change internal state to READY_TO_START
    not_ready_pipeline.addElement((FlightElement*)cs_to_ready_to_start);
    
    //Check Current Mission State
    ready_to_start_pipeline.addElement((FlightElement*)ready_to_start_check);
    //Call set_mission_state and set to Ignore
    ready_to_start_pipeline.addElement((FlightElement*)set_ignoring_state_outdoor_fire_detection);
    //Trigger Upload_UAV_Scan_Path
    ready_to_start_pipeline.addElement((FlightElement*)trigger_upload_uav_scan_path);
    //Check if UAV is at "Following Trajectory"
    ready_to_start_pipeline.addElement((FlightElement*)uav_control_following_trajectory_check);
    //Change internal state to SCANNING_OUTDOOR
    ready_to_start_pipeline.addElement((FlightElement*)cs_to_scanning_outdoor);
    //Call set_mission_state (Outdoor Fire Detection) and set to Scanning
    ready_to_start_pipeline.addElement((FlightElement*)set_scanning_state_outdoor_fire_detection);
    
    //Check Current Mission State
    scanning_outdoor_pipeline.addElement((FlightElement*)scanning_outdoor_check);
    //Check Outdoor Navigation is at "All wall fire detected"
    scanning_outdoor_pipeline.addElement((FlightElement*)outdoor_navigation_all_wall_fire_check);
    //Trigger Upload_UAV_Fire_Paths
    scanning_outdoor_pipeline.addElement((FlightElement*)trigger_upload_uav_fire_path);
    //Check if UAV is at "Following Trajectory"
    scanning_outdoor_pipeline.addElement((FlightElement*)uav_control_following_trajectory_check);
    //Change internal state to APPROACHING_OUTDOOR
    scanning_outdoor_pipeline.addElement((FlightElement*)cs_to_approaching_outdoor);

    //Check Current Mission State
    approach_outdoor_pipeline.addElement((FlightElement*)approach_outdoor_check);
    //Check if UAV is at "Hovering"
    approach_outdoor_pipeline.addElement((FlightElement*)uav_control_hovering_check);
    //Call set_mission_state (Fire Extinguishing) and set to Armed w/ Extinguishing
    approach_outdoor_pipeline.addElement((FlightElement*)set_arming_ext_state_fire_extinguishing);
    //Change internal state to EXTINGUISHING_OUTDOOR
    approach_outdoor_pipeline.addElement((FlightElement*)cs_to_extinguishing_outdoor);

    //Check Current Mission State
    extinguish_outdoor_pipeline.addElement((FlightElement*)extinguish_outdoor_check);
    //Check if Fire Extinguished is at "Extinguished"
    extinguish_outdoor_pipeline.addElement((FlightElement*)water_fire_extinguishing_extinguished_check);
    //Trigger UAV to go home
    extinguish_outdoor_pipeline.addElement((FlightElement*)trigger_upload_uav_home_path);
    //Check if UAV is at "Following Trajectory"
    extinguish_outdoor_pipeline.addElement((FlightElement*)uav_control_following_trajectory_check);
    //Change internal state to RETURNING_TO_BASE
    extinguish_outdoor_pipeline.addElement((FlightElement*)cs_to_return_to_base);

    //Check Current Mission State
    return_to_base_pipeline.addElement((FlightElement*)return_to_base_check);
    //Check if UAV is at "Landed"
    return_to_base_pipeline.addElement((FlightElement*)uav_control_landed_check);
    //Change internal state to FINISHED
    return_to_base_pipeline.addElement((FlightElement*)cs_to_finished);

    FlightPipeline safety_pipeline;

    Logger::getAssignedLogger()->log("FlightScenario main_scenario",LoggerLevel::Info);
    FlightScenario main_scenario;
    main_scenario.AddFlightPipeline(&not_ready_pipeline);
    main_scenario.AddFlightPipeline(&ready_to_start_pipeline);
    main_scenario.AddFlightPipeline(&scanning_outdoor_pipeline);
    main_scenario.AddFlightPipeline(&approach_outdoor_pipeline);
    main_scenario.AddFlightPipeline(&extinguish_outdoor_pipeline);
    main_scenario.AddFlightPipeline(&return_to_base_pipeline);
    main_scenario.AddFlightPipeline(&safety_pipeline);
    main_scenario.StartScenario();
    Logger::getAssignedLogger()->log("Main Done",LoggerLevel::Info);
    
    while(ros::ok){
        ros::spinOnce();
    }
}

    // ready_to_start_pipeline.addElement((FlightElement*)ref_z_on_takeoff);
    // ready_to_start_pipeline.addElement((FlightElement*)reset_z);
    // ready_to_start_pipeline.addElement((FlightElement*)arm_motors);
    //ready_to_start_pipeline.addElement((FlightElement*)z_cross_takeoff_waypoint_check);

     // return_to_base_pipeline.addElement((FlightElement*)flight_command);
    // return_to_base_pipeline.addElement((FlightElement*)ref_z_on_land);
    // //return_to_base_pipeline.addElement((FlightElement*)z_cross_land_waypoint_check);
    // return_to_base_pipeline.addElement((FlightElement*)&wait_1s);
    // return_to_base_pipeline.addElement((FlightElement*)disarm_motors);
