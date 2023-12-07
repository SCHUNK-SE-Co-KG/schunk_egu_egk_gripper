//This Client shows you how to control your gripper in program. If you do something else, as long as the program works, 
//this program will exhibit undefined behavior.
/*
#include "rclcpp/rclcpp.hpp
#include "schunk_gripper/grip_eguAction.h"
#include "schunk_gripper/grip_with_pos_eguAction.h"
#include "schunk_gripper/mov_abs_posAction.h"
#include "schunk_gripper/mov_rel_posAction.h"
#include "schunk_gripper/state.h"
#include "schunk_gripper/acknowledge.h"
#include "schunk_gripper/stop.h"
#include "schunk_gripper/fast_stop.h"
#include "schunk_gripper/gripper_parameterConfig.h"
#include "schunk_gripper/prepare_for_shutdown.h"
#include "schunk_gripper/softreset.h"
#include "schunk_gripper/release_workpieceAction.h"
#include "schunk_gripper/release_for_man_mov.h"
#include "schunk_gripper/gripper_info.h"
#include "control_msgs/GripperCommandAction.h"
#include "actionlib/client/simple_action_client.h"
#include "dynamic_reconfigure/client.h"
#include "sensor_msgs/JointState.h"
#include "diagnostic_msgs/DiagnosticArray.h"

typedef actionlib::SimpleActionClient<schunk_gripper::mov_abs_posAction> movAbsClient;

schunk_gripper::state state_msg;
sensor_msgs::JointState joint_state_msg;
diagnostic_msgs::DiagnosticArray diagnostic_msg;
schunk_gripper::gripper_parameterConfig config;

void stateCallback(const schunk_gripper::state::ConstPtr& msg)
{
    state_msg = *msg;
}

void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    joint_state_msg = *msg;
}

void diagnosticsCallback(const diagnostic_msgs::DiagnosticArray::ConstPtr& msg)
{   
    diagnostic_msg = *msg;
}

void feedbackCB(const schunk_gripper::mov_abs_posFeedbackConstPtr& msg)
{
   // ROS_INFO("Gripper is at: %f mm",msg->current_position);
}

void config_CB(const schunk_gripper::gripper_parameterConfig& config2)
{
    config = config2;
}

void doneCb(const actionlib::SimpleClientGoalState &state,const schunk_gripper::grip_eguResultConstPtr& msg)
{
    ROS_INFO("%s", msg->gripped ? "gripped" : "Not gripped");
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "schunk_example");

    if(!ros::master::check()) 
    {
        ROS_INFO("No ROS-Master!");
        return -1;
    }

    ros::NodeHandle nd;
    ros::AsyncSpinner spinner(0);
    spinner.start();



    //ALL SERVICES
    ros::ServiceClient acknowledge_client = nd.serviceClient<schunk_gripper::acknowledge> ("acknowledge");
    ros::ServiceClient stop_client = nd.serviceClient<schunk_gripper::stop>("stop");
    ros::ServiceClient softreset_client = nd.serviceClient<schunk_gripper::softreset>("softreset");
    ros::ServiceClient release_for_manuale_mov_client = nd.serviceClient<schunk_gripper::release_for_man_mov>("release_for_manual_movement");
    ros::ServiceClient prepare_for_shutdown_client = nd.serviceClient<schunk_gripper::prepare_for_shutdown>("prepare_for_shutdown");
    ros::ServiceClient fast_stop_client = nd.serviceClient<schunk_gripper::fast_stop>("fast_stop");
    ros::ServiceClient info_client = nd.serviceClient<schunk_gripper::stop>("gripper_info");
    //ALL ACTIONS
    actionlib::SimpleActionClient<schunk_gripper::mov_abs_posAction>           move_abs_client("move_absolute", true);
    actionlib::SimpleActionClient<schunk_gripper::mov_rel_posAction>           move_rel_client("move_relative", true);
    actionlib::SimpleActionClient<schunk_gripper::grip_eguAction>              grip_client("grip_egu", true);
    actionlib::SimpleActionClient<schunk_gripper::grip_with_pos_eguAction>     grp_w_pos_client("grip_with_pos_egu", true);
    actionlib::SimpleActionClient<schunk_gripper::release_workpieceAction>     release_client("release_workpiece", true);
    
    actionlib::SimpleActionClient<control_msgs::GripperCommandAction>      control_client("gripper_control", true);

    //TOPICS
    ros::Subscriber state_sub = nd.subscribe("state", 1, stateCallback);
    ros::Subscriber joint_state_sub = nd.subscribe("joint_states", 1, jointStateCallback);
    ros::Subscriber diagnostics_sub = nd.subscribe("diagnostics", 1, diagnosticsCallback);
    //Dynamic Reconfigure
    dynamic_reconfigure::Client<schunk_gripper::gripper_parameterConfig> param_client("schunk_gripper_driver",config_CB);
    
    
    //GET CURRENT CONFIGURATION!!!!!
    if(!param_client.getCurrentConfiguration(config, ros::Duration(5)))
    {
        ROS_WARN("Parameter weren't loaded");
    };
    
    move_abs_client.waitForServer();
    
    schunk_gripper::acknowledge acknowledge_srv;
    ros::topic::waitForMessage<diagnostic_msgs::DiagnosticArray>("diagnostics");
    if(diagnostic_msg.status[0].level) 
    {
        if(acknowledge_client.call(acknowledge_srv))
        ROS_INFO("AN ERROR WAS ACKNOWELEDGED!");
    }
    
    bool handshake = state_msg.command_received_toggle;

    //Move to 0 mm with 100 mm/s
    schunk_gripper::mov_abs_posGoal goal_abs;
    goal_abs.abs_position = 100.0;
    goal_abs.velocity = 10.0;
    move_abs_client.sendGoal(goal_abs, movAbsClient::SimpleDoneCallback(), movAbsClient::SimpleActiveCallback(), feedbackCB);
    
    while(state_msg.command_received_toggle == handshake && ros::ok() && !state_msg.error)
        if(!state_msg.error) ros::Duration(1/60.0).sleep();
        else break;
    handshake = state_msg.command_received_toggle;
    ROS_WARN("%s: %f mm", state_msg.doing_command.c_str(), goal_abs.abs_position);
   
    move_abs_client.waitForResult(ros::Duration(10));

    if(move_abs_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("On the right position");

    ros::Duration(3).sleep();

    //Move relative
    move_rel_client.waitForServer();

    schunk_gripper::mov_rel_posGoal goal_rel;
    schunk_gripper::mov_rel_posResult res_rel;
    schunk_gripper::stop stop_srv;

    goal_rel.distance = -50.0;
    goal_rel.max_velocity = 7.0;
    move_rel_client.sendGoal(goal_rel);

    while(state_msg.command_received_toggle == handshake && ros::ok()) 
        if(!state_msg.error) ros::Duration(1/60.0).sleep();
        else break;
    handshake = state_msg.command_received_toggle;
    ROS_WARN("%s: %f mm", state_msg.doing_command.c_str(), goal_rel.distance);

    ros::Duration(5).sleep();
    
    
    //Stop the moving
    if(stop_client.call(stop_srv))
    while(state_msg.command_received_toggle == handshake && ros::ok())
        if(!state_msg.error) ros::Duration(1/60.0).sleep();
        else break;
    handshake = state_msg.command_received_toggle;
    ROS_WARN("%s", state_msg.doing_command.c_str());

    ROS_INFO("%s", stop_srv.response.stopped ? "Stopped!" : "Not stopped!");


    move_rel_client.waitForResult();
    res_rel = *move_rel_client.getResult();
    ROS_INFO_STREAM("Move relative was: " << move_rel_client.getState().toString().c_str()
                    << "\n End Position: " << res_rel.current_position);

    ros::Duration(3).sleep();

    //Move relative again
    move_rel_client.sendGoal(goal_rel);

    while(state_msg.command_received_toggle == handshake && ros::ok())
        if(!state_msg.error) ros::Duration(1/60.0).sleep();
        else break;
    handshake = state_msg.command_received_toggle;
    ROS_WARN("%s: %f mm", state_msg.doing_command.c_str(), goal_rel.distance);


    ros::Duration(5).sleep();
    //Cancel the goal
    move_rel_client.cancelGoal();          //This performs a fast stop

    while(state_msg.command_received_toggle == handshake && ros::ok()) 
        if(!state_msg.error) ros::Duration(1/60.0).sleep();
        else break;
    handshake = state_msg.command_received_toggle;
    ROS_WARN("%s", state_msg.doing_command.c_str());
    
    move_rel_client.waitForResult();
    res_rel = *move_rel_client.getResult();
    ROS_INFO("Move relative was: %s",move_rel_client.getState().toString().c_str());
    ROS_INFO("End Position: %f", res_rel.current_position);
    //Fast stop/cancel causes an Error. Look at the error in diagnostics
    ros::topic::waitForMessage<diagnostic_msgs::DiagnosticArray>("diagnostics");
    ROS_ERROR("%s",diagnostic_msg.status.at(0).message.c_str());
    
    
    ros::Duration(3).sleep();
    
    ROS_INFO("Call acknowledge-server.");
    //To end the error: acknowledge
    if(acknowledge_client.call(acknowledge_srv)) ROS_INFO("Acknowledge called");

    while(state_msg.command_received_toggle == handshake && ros::ok())
        if(!state_msg.error) ros::Duration(1/60.0).sleep();
        else break;
    handshake = state_msg.command_received_toggle;
    ROS_WARN("%s", state_msg.doing_command.c_str());                //Acknowledge is never as a Command displayed


    ROS_INFO("%s",acknowledge_srv.response.acknowledged ? "Acknowledged" : "Not acknowledged");

    ros::topic::waitForMessage<diagnostic_msgs::DiagnosticArray>("diagnostics");
    ROS_INFO("%s", diagnostic_msg.status.at(0).message.c_str());

    ros::Duration(3).sleep();

    //GET CURRENT CONFIGURATION!!!!! ELSE YOU COULD SAY THE GRIPPER TO MOVE, EVEN IF YOU DONT WANT IT!!!
    if(param_client.getCurrentConfiguration(config, ros::Duration(5)))
    {
    //Change what you wish
    config.wp_release_delta = 10;               //mm
    config.grp_prehold_time = 10000;            //ms
    //And set Configuration
    if(param_client.setConfiguration(config)) ROS_INFO("Parameter set: workpiece release delta, grip preold time");
    else ROS_WARN("Server did not accept Configuratuion!");
    }
    ros::Duration(3).sleep();

    schunk_gripper::grip_eguGoal goal_grip;
    goal_grip.effort  = 50;                     //Percent
    goal_grip.grp_dir = 0;                      //grip direction
    grip_client.sendGoal(goal_grip, doneCb);

    while(state_msg.command_received_toggle == handshake && ros::ok()) 
        if(!state_msg.error) ros::Duration(1/60.0).sleep();
        else break;
    handshake = !handshake;
    ROS_WARN("%s", state_msg.doing_command.c_str());
    
    while(state_msg.pre_grip_started == false && grip_client.getState() == actionlib::SimpleClientGoalState::ACTIVE);
    if(state_msg.pre_grip_started == true) ROS_INFO("Pre_grip started");

    grip_client.waitForResult();

    ros::topic::waitForMessage<schunk_gripper::state>("state");
    if(state_msg.workpiece_gripped == true)
    {
        schunk_gripper
    ::release_workpieceGoal goal_release;
        release_client.sendGoal(goal_release);
        
        while(state_msg.command_received_toggle == handshake && ros::ok())
            if(!state_msg.error) ros::Duration(1/60.0).sleep();
            else break;
        handshake = !handshake;
        ROS_WARN("%s", state_msg.doing_command.c_str());
        
        release_client.waitForResult(ros::Duration(10));

    }
    else ROS_INFO("No Workpiece gripped!");



    ros::Duration(3).sleep();


    ROS_INFO("Set configuration!");
    //Optionally you can control some basic Commands with dynamic reconfigure parameters
    //GET CURRENT CONFIGURATION!!!!! ELSE YOU COULD GET THE GRIPPER TO MOVE, EVEN IF YOU DONT WANT IT!!!
    if(param_client.getCurrentConfiguration(config, ros::Duration(5)))
    {
    config.move_gripper = 50;
    config.move_gripper_velocity = 50;
    if(!param_client.setConfiguration(config))
    {
        ROS_INFO("Server did not accept configuration.");
        return -1;
    }

    while(state_msg.command_received_toggle == handshake && ros::ok())  
        if(!state_msg.error) ros::Duration(1/60.0).sleep();
        else break;
    handshake = !handshake;
    ROS_WARN("%s: %f mm", state_msg.doing_command.c_str(), config.move_gripper);

    //NO FEEDBACK OR RESULT BUT YOU CAN LOOK IF ITS SUCCESSFULL WITH DIAGNOSTICS OR STATE;
    while((state_msg.error || state_msg.warning) == false && !state_msg.command_successfully_processed)
    ros::Duration(1/60.0).sleep();

    if(state_msg.position_reached == true)
    ROS_INFO("Position successfully reached");
    else
    ROS_ERROR("Is there an Error? %s", diagnostic_msg.status.at(0).message.c_str());
    }
    return 0;
}
*/