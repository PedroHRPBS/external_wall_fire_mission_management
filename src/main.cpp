#include "ros/ros.h"
#include <iostream>
#include "ROSUnit.hpp"
#include "logger.hpp"
#include "std_logger.hpp"
#include "FlightElement.hpp"
#include "Wait.hpp"
#include "WaitForCondition.hpp"
#include "FlightPipeline.hpp"
#include "FlightScenario.hpp"
#include "SetMissionState.hpp"
#include "InternalSystemStateCondition.hpp"
#include "ChangeInternalState.hpp"
#include "ExternalSystemStateCondition.hpp"
#include "SendMessage.hpp"
#include "EmptyMsg.hpp"
#include "ROSUnit_Factory.hpp"
#include "MissionStateManager.hpp"

int main(int argc, char** argv) {
    Logger::assignLogger(new StdLogger());

    //****************ROS Units********************
    ros::init(argc, argv, "ex_bldg_fire_mm_node");
    ros::NodeHandle nh;

    ROSUnit_Factory ROSUnit_Factory_main{nh};
	ROSUnit* ros_set_system_state_srv = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Server_Publisher, ROSUnit_msg_type::ROSUnit_Int, "ex_bldg_fire_mm/set_system_state");
	ROSUnit* ros_updt_fire_detection_state_srv = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Server_Publisher, ROSUnit_msg_type::ROSUnit_Int, "ex_bldg_fire_mm/update_fire_detection_state");
    ROSUnit* ros_updt_uav_control_state_srv = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Server_Publisher, ROSUnit_msg_type::ROSUnit_Int, "ex_bldg_fire_mm/update_uav_control_state");
    ROSUnit* ros_updt_water_ext_state_srv = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Server_Publisher, ROSUnit_msg_type::ROSUnit_Int, "ex_bldg_fire_mm/update_water_ext_state");
    ROSUnit* ros_updt_outdoor_nav_state_srv = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Server_Publisher, ROSUnit_msg_type::ROSUnit_Int, "ex_bldg_fire_mm/update_outdoor_nav_state");
    ROSUnit* ros_updt_water_level_srv = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Server_Publisher, ROSUnit_msg_type::ROSUnit_Int, "ex_bldg_fire_mm/update_water_level");
    
    ROSUnit* ros_set_fire_detection_state_clnt = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Client_Subscriber, ROSUnit_msg_type::ROSUnit_Int, "fire_detection/set_mission_state");
    ROSUnit* ros_trigger_uav_scan_path_clnt = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Client_Subscriber, ROSUnit_msg_type::ROSUnit_Empty, "outdoor_navigation/upload_uav_scan_path");
    ROSUnit* ros_trigger_uav_fire_path_clnt = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Client_Subscriber, ROSUnit_msg_type::ROSUnit_Int, "outdoor_navigation/upload_uav_fire_paths");
    ROSUnit* ros_trigger_uav_home_path_clnt = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Client_Subscriber, ROSUnit_msg_type::ROSUnit_Empty, "outdoor_nav/upload_uav_landing_path");
    ROSUnit* set_fire_extinguishing_state_clnt = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Client_Subscriber, ROSUnit_msg_type::ROSUnit_Int, "water_ext/set_mission_state");
    ROSUnit* ros_set_mission_state_clnt = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Client_Subscriber, ROSUnit_msg_type::ROSUnit_Int, "uav_control/set_mission_state");

    //*****************Flight Elements*************

    FlightElement* cs_to_not_ready = new ChangeInternalState(external_wall_fire_states::NOT_READY);
    FlightElement* cs_to_ready_to_start = new ChangeInternalState(external_wall_fire_states::READY_TO_START);
    FlightElement* cs_to_scanning_outdoor = new ChangeInternalState(external_wall_fire_states::SCANNING_OUTDOOR);
    FlightElement* cs_to_approaching_outdoor = new ChangeInternalState(external_wall_fire_states::APPROACHING_OUTDOOR);
    FlightElement* cs_to_extinguishing_outdoor = new ChangeInternalState(external_wall_fire_states::EXTINGUISHING_OUTDOOR);
    FlightElement* cs_to_return_to_base = new ChangeInternalState(external_wall_fire_states::RETURNING_TO_BASE);
    FlightElement* cs_to_finished = new ChangeInternalState(external_wall_fire_states::FINISHED);
    FlightElement* cs_to_error = new ChangeInternalState(external_wall_fire_states::ERROR);

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

    IntegerMsg taking_off_state;
    taking_off_state.data = 4;
    FlightElement* set_taking_off_state_uav_control = new SendMessage((DataMessage*)&taking_off_state);

    IntegerMsg landing_state;
    landing_state.data = 5;
    FlightElement* set_landing_state_uav_control = new SendMessage((DataMessage*)&landing_state);


    InternalSystemStateCondition* not_ready_condition = new InternalSystemStateCondition(external_wall_fire_states::NOT_READY);
    WaitForCondition* not_ready_check = new WaitForCondition((Condition*)not_ready_condition);

    InternalSystemStateCondition* ready_to_start_condition = new InternalSystemStateCondition(external_wall_fire_states::READY_TO_START);
    WaitForCondition* ready_to_start_check = new WaitForCondition((Condition*)ready_to_start_condition);

    InternalSystemStateCondition* scanning_outdoor_condition = new InternalSystemStateCondition(external_wall_fire_states::SCANNING_OUTDOOR);
    WaitForCondition* scanning_outdoor_check = new WaitForCondition((Condition*)scanning_outdoor_condition);

    InternalSystemStateCondition* approach_outdoor_condition = new InternalSystemStateCondition(external_wall_fire_states::APPROACHING_OUTDOOR);
    WaitForCondition* approach_outdoor_check = new WaitForCondition((Condition*)approach_outdoor_condition);

    InternalSystemStateCondition* extinguish_outdoor_condition = new InternalSystemStateCondition(external_wall_fire_states::EXTINGUISHING_OUTDOOR);
    WaitForCondition* extinguish_outdoor_check = new WaitForCondition((Condition*)extinguish_outdoor_condition);

    InternalSystemStateCondition* return_to_base_condition = new InternalSystemStateCondition(external_wall_fire_states::RETURNING_TO_BASE);
    WaitForCondition* return_to_base_check = new WaitForCondition((Condition*)return_to_base_condition);

    InternalSystemStateCondition* error_condition = new InternalSystemStateCondition(external_wall_fire_states::ERROR);
    WaitForCondition* error_check = new WaitForCondition((Condition*)error_condition);

    InternalSystemStateCondition* finished_condition = new InternalSystemStateCondition(external_wall_fire_states::FINISHED);
    WaitForCondition* finished_check = new WaitForCondition((Condition*)finished_condition);

    //The int values passed on the following constructors need to match the system states of the external systems.
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

    Wait wait5s;
    wait5s.wait_time_ms = 5000;

    //******************Connections******************

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
    set_taking_off_state_uav_control->add_callback_msg_receiver((msg_receiver*)ros_set_mission_state_clnt);
    set_landing_state_uav_control->add_callback_msg_receiver((msg_receiver*)ros_set_mission_state_clnt);
    
    //**********************************************
    FlightPipeline not_ready_pipeline, ready_to_start_pipeline, scanning_outdoor_pipeline,
                   approach_outdoor_pipeline, extinguish_outdoor_pipeline, return_to_base_pipeline,
                   error_pipeline, finished_pipeline;

    //Check Current Mission State
    not_ready_pipeline.addElement((FlightElement*)not_ready_check);
    //Check if all systems are ready
    //not_ready_pipeline.addElement((FlightElement*)outdoor_wall_fire_detection_idle_check);
    not_ready_pipeline.addElement((FlightElement*)uav_control_landed_check);
    //not_ready_pipeline.addElement((FlightElement*)water_fire_extinguishing_idle_check);
    //not_ready_pipeline.addElement((FlightElement*)outdoor_navigation_idle_check); //No need to check this state
    //Change internal state to READY_TO_START
    not_ready_pipeline.addElement((FlightElement*)cs_to_ready_to_start);
    
    //Check Current Mission State
    ready_to_start_pipeline.addElement((FlightElement*)ready_to_start_check);
    //Call set_mission_state (Outdoor Fire Detection) and set to Ignore
    ready_to_start_pipeline.addElement((FlightElement*)set_ignoring_state_outdoor_fire_detection);
    //Call set_mission_state (UAV_Control) and set to Taking_Off
    ready_to_start_pipeline.addElement((FlightElement*)set_taking_off_state_uav_control); 
    //Trigger Upload_UAV_Scan_Path
    ready_to_start_pipeline.addElement((FlightElement*)trigger_upload_uav_scan_path);
    //Check if UAV is at "Following Trajectory"
    // ready_to_start_pipeline.addElement((FlightElement*)uav_control_following_trajectory_check);
    //Change internal state to SCANNING_OUTDOOR
    ready_to_start_pipeline.addElement((FlightElement*)cs_to_scanning_outdoor);
    //Call set_mission_state (Outdoor Fire Detection) and set to Scanning
    ready_to_start_pipeline.addElement((FlightElement*)set_scanning_state_outdoor_fire_detection);
    
    //Check Current Mission State
    scanning_outdoor_pipeline.addElement((FlightElement*)scanning_outdoor_check);
    //Check Outdoor Navigation is at "All wall fire detected"
    //scanning_outdoor_pipeline.addElement((FlightElement*)outdoor_navigation_all_wall_fire_check);
    //Trigger Upload_UAV_Fire_Paths with a fire tag
    // scanning_outdoor_pipeline.addElement((FlightElement*)trigger_upload_uav_fire_path);
    //Check if UAV is at "Following Trajectory"
    // scanning_outdoor_pipeline.addElement((FlightElement*)uav_control_following_trajectory_check);
    //Check if UAV is at "Hovering"
    scanning_outdoor_pipeline.addElement((FlightElement*)uav_control_hovering_check); 
    //Wait 5s
    scanning_outdoor_pipeline.addElement((FlightElement*)&wait5s); 
    //Trigger UAV to go home
    scanning_outdoor_pipeline.addElement((FlightElement*)trigger_upload_uav_home_path);
    //Change internal state to RETURNING_TO_BASE
    scanning_outdoor_pipeline.addElement((FlightElement*)cs_to_return_to_base);
    //Change internal state to APPROACHING_OUTDOOR
    //scanning_outdoor_pipeline.addElement((FlightElement*)cs_to_approaching_outdoor);

    // //Check Current Mission State
    // approach_outdoor_pipeline.addElement((FlightElement*)approach_outdoor_check);
    // //Check if UAV is at "Hovering"
    // approach_outdoor_pipeline.addElement((FlightElement*)uav_control_hovering_check); 
    // //Call set_mission_state (Fire Extinguishing) and set to Armed w/ Extinguishing
    // approach_outdoor_pipeline.addElement((FlightElement*)set_arming_ext_state_fire_extinguishing);
    // //Change internal state to EXTINGUISHING_OUTDOOR
    // approach_outdoor_pipeline.addElement((FlightElement*)cs_to_extinguishing_outdoor);

    // //Check Current Mission State
    // extinguish_outdoor_pipeline.addElement((FlightElement*)extinguish_outdoor_check);
    // //Check if Fire Extinguished is at "Extinguished"
    // extinguish_outdoor_pipeline.addElement((FlightElement*)water_fire_extinguishing_extinguished_check);
    // //Trigger UAV to go home
    // extinguish_outdoor_pipeline.addElement((FlightElement*)trigger_upload_uav_home_path);
    // //Check if UAV is at "Following Trajectory"
    // extinguish_outdoor_pipeline.addElement((FlightElement*)uav_control_following_trajectory_check);
    // //Change internal state to RETURNING_TO_BASE
    // extinguish_outdoor_pipeline.addElement((FlightElement*)cs_to_return_to_base);

    //Check Current Mission State
    return_to_base_pipeline.addElement((FlightElement*)return_to_base_check);
    //Check if UAV is at "Hovering"
    return_to_base_pipeline.addElement((FlightElement*)uav_control_hovering_check);
    //Call set_mission_state (UAV_Control) and set to Landing
    return_to_base_pipeline.addElement((FlightElement*)set_landing_state_uav_control);
    //Change internal state to FINISHED
    return_to_base_pipeline.addElement((FlightElement*)cs_to_finished);

    //TODO Error Pipeline

    Logger::getAssignedLogger()->log("FlightScenario main_scenario",LoggerLevel::Info);
    FlightScenario main_scenario;
    main_scenario.AddFlightPipeline(&not_ready_pipeline);
    main_scenario.AddFlightPipeline(&ready_to_start_pipeline);
    main_scenario.AddFlightPipeline(&scanning_outdoor_pipeline);
    main_scenario.AddFlightPipeline(&approach_outdoor_pipeline);
    main_scenario.AddFlightPipeline(&extinguish_outdoor_pipeline);
    main_scenario.AddFlightPipeline(&return_to_base_pipeline);
    main_scenario.StartScenario();
    Logger::getAssignedLogger()->log("Main Done",LoggerLevel::Info);
    
    while(ros::ok){
        ros::spinOnce();
    }
}
