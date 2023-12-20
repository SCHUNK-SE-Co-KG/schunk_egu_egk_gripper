//This Client shows you how to control your gripper in program. If you do something else, as long as the program works, 
//this program will exhibit undefined behavior.
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include <rclcpp/parameter.hpp>
#include <rcl_interfaces/msg/parameter_event.hpp>
#include "schunk_gripper/schunk_gripper_lib.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <diagnostic_updater/diagnostic_updater.hpp>

#include "schunk_gripper/action/grip_with_vel.hpp"
#include "schunk_gripper/action/grip_with_pos_vel.hpp"
#include "schunk_gripper/action/mov_abs_pos.hpp"
#include "schunk_gripper/action/mov_rel_pos.hpp"
#include "schunk_gripper/msg/state.hpp"
#include "schunk_gripper/srv/acknowledge.hpp"
#include "schunk_gripper/srv/stop.hpp"
#include "schunk_gripper/srv/fast_stop.hpp"
#include "schunk_gripper/srv/prepare_for_shutdown.hpp"
#include "schunk_gripper/srv/softreset.hpp"
#include "schunk_gripper/action/release_workpiece.hpp"
#include "schunk_gripper/srv/release_for_man_mov.hpp"
#include "schunk_gripper/srv/gripper_info.hpp"
#include "control_msgs/action/gripper_command.hpp"
#include "schunk_gripper/action/grip.hpp"
#include "schunk_gripper/action/grip_with_pos.hpp"

schunk_gripper::msg::State state_msg;
sensor_msgs::msg::JointState joint_state_msg;
diagnostic_msgs::msg::DiagnosticArray diagnostic_msg;
bool handshake;

    using Acknowledge = schunk_gripper::srv::Acknowledge;
    using Stop = schunk_gripper::srv::Stop;
    using FastStop =  schunk_gripper::srv::FastStop;
    using ReleaseForManMov = schunk_gripper::srv::ReleaseForManMov;
    using Softreset = schunk_gripper::srv::Softreset;
    using PrepareForShutdown = schunk_gripper::srv::PrepareForShutdown;
    using GripperInfo= schunk_gripper::srv::GripperInfo;

    using MovAbsPos = schunk_gripper::action::MovAbsPos;
    using MovRelPos = schunk_gripper::action::MovRelPos;
    using GripWithVel = schunk_gripper::action::GripWithVel;
    using Grip = schunk_gripper::action::Grip;
    using GripWithPosVel = schunk_gripper::action::GripWithPosVel;
    using GripWithPos = schunk_gripper::action::GripWithPos;
    using ReleaseWorkpiece = schunk_gripper::action::ReleaseWorkpiece;
    using GripperCommand = control_msgs::action::GripperCommand;

//Callback Functions

void stateCallback(const schunk_gripper::msg::State::SharedPtr msg)
{
    state_msg = *msg;
}

void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    joint_state_msg = *msg;
}

void diagnosticsCallback(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg)
{   
    diagnostic_msg = *msg;
}

void feedbackCB(rclcpp_action::ClientGoalHandle<MovAbsPos>::SharedPtr, const std::shared_ptr<const MovAbsPos::Feedback> feedback)
{
    RCLCPP_INFO(rclcpp::get_logger("schunk_gripper_example"), "Gripper is at: %f mm",feedback->current_position);
}
/*
void doneEguCb(const actionlib::SimpleClientGoalState &state,const schunk_gripper::gripResultConstPtr& msg)
{
    RCLCPP_INFO(node->get_logger(), "%s", msg->gripped ? "gripped" : "Not gripped");
};

void doneEgkCb(const actionlib::SimpleClientGoalState &state,const schunk_gripper::grip_with_velResultConstPtr& msg)
{
    RCLCPP_INFO(node->get_logger(), "%s", msg->gripped ? "gripped" : "Not gripped");
};
*/
//Control Functions

void moveAbsoluteAndWaitForResult(rclcpp_action::Client<MovAbsPos>::SharedPtr move_abs_client)
{
    //Move to 0 mm with 100 mm/s
    MovAbsPos::Goal goal_abs;
    goal_abs.abs_position = 90.0;
    goal_abs.velocity = 10.0;

    auto send_goal_options = rclcpp_action::Client<MovAbsPos>::SendGoalOptions();
    send_goal_options.feedback_callback = std::bind(feedbackCB, std::placeholders::_1, std::placeholders::_2);
    auto result = move_abs_client->async_send_goal(goal_abs, send_goal_options);

    RCLCPP_WARN(rclcpp::get_logger("schunk_gripper_example"), "%s: %f mm", state_msg.doing_command.c_str(), goal_abs.abs_position);
    
    if(move_abs_client->async_get_result(result.get()).get().code == rclcpp_action::ResultCode::SUCCEEDED)
        RCLCPP_INFO(rclcpp::get_logger("schunk_gripper_example"), "On the right position");
        
}

void moveRelative(rclcpp_action::Client<MovRelPos>::SharedPtr move_rel_client)
{/*
    schunk_gripper::mov_rel_posGoal goal_rel;

    goal_rel.distance = -50.0;
    goal_rel.max_velocity = 7.0;
    move_rel_client.sendGoal(goal_rel);

    while(state_msg.command_received_toggle == handshake && rclcpp::ok()) 
        if(!state_msg.error) rclcpp::Duration(1/60.0).sleep();
        else break;
    handshake = state_msg.command_received_toggle;
    RCLCPP_WARN(rclcpp::get_logger("schunk_gripper_example"), "%s: %f mm", state_msg.doing_command.c_str(), goal_rel.distance);
*/
}

void stop(rclcpp::Client<Stop>::SharedPtr stop_client)
{
    /*
    Stop::Request::SharedPtr stop_req;
    auto resp = stop_client->async_send_request(stop_req);
    RCLCPP_WARN(rclcpp::get_logger("schunk_gripper_example"), "%s", state_msg.doing_command.c_str());
    resp.wait();
    //Stop the moving
    if(resp.valid())
    handshake = state_msg.command_received_toggle;

    RCLCPP_INFO(rclcpp::get_logger("schunk_gripper_example"), "%s", resp.get()->stopped ? "Stopped!" : "Not stopped!");
*/
}

void moveRelativeResult(rclcpp_action::Client<MovRelPos>::SharedPtr move_rel_client)
{   /*
    schunk_gripper::mov_rel_posResult res_rel;

    move_rel_client.waitForResult();
    res_rel = *move_rel_client.getResult();
    RCLCPP_INFO_node->get_logger(), STREAM("Move relative was: " << move_rel_client.getState().toString().c_str()
                    << "\n End Position: " << res_rel.current_position);
    */
}

void cancelMoveRelative(rclcpp_action::Client<MovRelPos>::SharedPtr move_rel_client)
{
    /*
    move_rel_client.cancelGoal();          //This performs a fast stop

    while(state_msg.command_received_toggle == handshake && rclcpp::ok()) 
        if(!state_msg.error) rclcpp::Duration(1/60.0).sleep();
        else break;
    handshake = state_msg.command_received_toggle;
    RCLCPP_WARN(rclcpp::get_logger("schunk_gripper_example"), "%s", state_msg.doing_command.c_str());
    */
}

void acknowledge(rclcpp::Client<Acknowledge>::SharedPtr acknowledge_client)
{
    /*
    schunk_gripper::acknowledge acknowledge_srv;
    
    RCLCPP_INFO(node->get_logger(), "Call acknowledge-server.");
    //To end the error: acknowledge
    if(acknowledge_client.call(acknowledge_srv)) RCLCPP_INFO(node->get_logger(), "Acknowledge called");

    while(state_msg.command_received_toggle == handshake && rclcpp::ok())
        if(!state_msg.error) rclcpp::Duration(1/60.0).sleep();
        else break;
    handshake = state_msg.command_received_toggle;
    RCLCPP_WARN(rclcpp::get_logger("schunk_gripper_example"), "%s", state_msg.doing_command.c_str());                //Acknowledge is never as a Command displayed


    RCLCPP_INFO(node->get_logger(), "%s",acknowledge_srv.response.acknowledged ? "Acknowledged" : "Not acknowledged");
    */
}
/*/
void changeConfiguration(dynamic_reconfigure::Client<schunk_gripper::gripper_parameterConfig> &param_client)
{
    //GET CURRENT CONFIGURATION!!!!! ELSE YOU COULD SAY THE GRIPPER TO MOVE, EVEN IF YOU DONT WANT IT!!!
    if(param_client.getCurrentConfiguration(config, rclcpp::Duration(5)))
    {
    //Change what you wish
    config.wp_release_delta = 10;               //mm
    config.grp_prehold_time = 10000;            //ms
    //And set Configuration
    if(param_client.setConfiguration(config)) RCLCPP_INFO(node->get_logger(), "Parameter set: workpiece release delta, grip pre-hold time");
    else RCLCPP_WARN(rclcpp::get_logger("schunk_gripper_example"), "Server did not accept Configuration!");
    }

}
*/
void gripEgu(rclcpp_action::Client<Grip>::SharedPtr grip_client)
{
    /*
    schunk_gripper::gripGoal goal_grip;
    goal_grip.effort  = 50;                     //Percent
    goal_grip.grp_dir = 0;                      //grip direction
    grip_client.sendGoal(goal_grip, doneEguCb);

    while(state_msg.command_received_toggle == handshake && rclcpp::ok()) 
        if(!state_msg.error) rclcpp::Duration(1/60.0).sleep();
        else break;
    handshake = !handshake;
    RCLCPP_WARN(rclcpp::get_logger("schunk_gripper_example"), "%s", state_msg.doing_command.c_str());
    
    while(state_msg.pre_grip_started == false && grip_client.getState() == actionlib::SimpleClientGoalState::ACTIVE);
    if(state_msg.pre_grip_started == true) RCLCPP_INFO(node->get_logger(), "Pre_grip started");

    grip_client.waitForResult();
    */
} 

void gripEgk(rclcpp_action::Client<GripWithVel>::SharedPtr grip_client)
{
    /*
    schunk_gripper::grip_with_velGoal goal_grip;
    goal_grip.effort  = 50;                     //Percent
    goal_grip.max_velocity = 0;
    goal_grip.grp_dir = 0;                      //grip direction
    grip_client.sendGoal(goal_grip, doneEgkCb);

    while(state_msg.command_received_toggle == handshake && rclcpp::ok()) 
        if(!state_msg.error) rclcpp::Duration(1/60.0).sleep();
        else break;
    handshake = !handshake;
    RCLCPP_WARN(rclcpp::get_logger("schunk_gripper_example"), "%s", state_msg.doing_command.c_str());
    
    while(state_msg.pre_grip_started == false && grip_client.getState() == actionlib::SimpleClientGoalState::ACTIVE);
    if(state_msg.pre_grip_started == true) RCLCPP_INFO(node->get_logger(), "Pre_grip started");

    grip_client.waitForResult();
    */
}

void releaseWorkpiece(rclcpp_action::Client<ReleaseWorkpiece>::SharedPtr release_client)
{/*
        schunk_gripper::release_workpieceGoal goal_release;
        release_client.sendGoal(goal_release);
        
        while(state_msg.command_received_toggle == handshake && rclcpp::ok())
            if(!state_msg.error) rclcpp::Duration(1/60.0).sleep();
            else break;
        handshake = !handshake;
        RCLCPP_WARN(rclcpp::get_logger("schunk_gripper_example"), "%s", state_msg.doing_command.c_str());
        
        release_client.waitForResult(rclcpp::Duration(10));
        */
}
/*
void moveAbsWithConfig(dynamic_reconfigure::Client<schunk_gripper::gripper_parameterConfig> &param_client)
{
    if(param_client.getCurrentConfiguration(config, rclcpp::Duration(5)))
    {
    config.move_gripper = 50;
    config.move_gripper_velocity = 50;
    if(!param_client.setConfiguration(config))
    {
        RCLCPP_INFO(node->get_logger(), "Server did not accept configuration.");
        return;
    }

    while(state_msg.command_received_toggle == handshake && rclcpp::ok())  
        if(!state_msg.error) rclcpp::Duration(1/60.0).sleep();
        else break;
    handshake = !handshake;
    RCLCPP_WARN(rclcpp::get_logger("schunk_gripper_example"), "%s: %f mm", state_msg.doing_command.c_str(), config.move_gripper);

    //NO FEEDBACK OR RESULT BUT YOU CAN LOOK IF ITS SUCCESSFUL WITH DIAGNOSTICS OR STATE;
    while((state_msg.error || state_msg.warning) == false && !state_msg.command_successfully_processed)
    rclcpp::Duration(1/60.0).sleep();

    if(state_msg.position_reached == true)
    RCLCPP_INFO(node->get_logger(), "Position successfully reached");
    else
    RCLCPP_ERROR(node->get_logger(),"Is there an Error? %s", diagnostic_msg.status.at(0).message.c_str());
    }
}
*/

void spinFunction(std::shared_ptr<rclcpp::Node> node)
{
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread spin_thread();
    executor.spin();
    rclcpp::shutdown();
}

int main(int argc, char** argv)
{
    std::string model;

    std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("schunk_gripper_example");
    //IS SCHUNK GRIPPER ACTIVE
/*
    if(rclcpp::param::has("model")) 
    {
        rclcpp::param::get("model", model);
        RCLCPP_INFO(node->get_logger(), "Your model is: %s", model.c_str());
    }
    else
    {
        RCLCPP_INFO(node->get_logger(), "No Model was detected");
        return -1;
    }
*/

    //ALL SERVICES
    auto acknowledge_client = node->create_client<Acknowledge> ("acknowledge");
    auto stop_client = node->create_client<Stop>("stop");
    auto softreset_client = node->create_client<Softreset>("softreset");
    auto release_for_manual_mov_client = node->create_client<ReleaseForManMov>("release_for_manual_movement");
    auto prepare_for_shutdown_client = node->create_client<PrepareForShutdown>("prepare_for_shutdown");
    auto fast_stop_client = node->create_client<FastStop>("fast_stop");
    auto info_client = node->create_client<GripperInfo>("gripper_info");
    //ALL ACTIONS
    auto move_abs_client = rclcpp_action::create_client<MovAbsPos>(node, "move_absolute");
    auto move_rel_client = rclcpp_action::create_client<MovRelPos>(node, "move_relative");
    auto grip_egu_client = rclcpp_action::create_client<Grip>(node, "grip_egu");
    auto grp_w_pos_egu_client = rclcpp_action::create_client<GripWithPos>(node, "grip_with_pos_egu");
    auto grip_egk_client = rclcpp_action::create_client<GripWithVel>(node, "grip_egk");
    auto grp_w_pos_egk_client = rclcpp_action::create_client<GripWithPosVel>(node, "grip_with_pos_egk");
    auto release_client = rclcpp_action::create_client<ReleaseWorkpiece>(node, "release_workpiece");
    
    auto control_client = rclcpp_action::create_client<GripperCommand>(node, "gripper_control");

    //TOPICS
    auto state_sub = node->create_subscription<schunk_gripper::msg::State>("state", 1, stateCallback);
    auto joint_state_sub = node->create_subscription<sensor_msgs::msg::JointState>("joint_states", 1, jointStateCallback);
    auto diagnostics_sub = node->create_subscription<diagnostic_msgs::msg::DiagnosticArray>("diagnostics", 1, diagnosticsCallback);

    std::thread spin_thread(&spinFunction, node);
    //Dynamic Reconfigure
 //   dynamic_reconfigure::Client<schunk_gripper::gripper_parameterConfig> param_client("schunk_gripper_driver",config_CB);
    
    
  /*  //GET CURRENT CONFIGURATION!!!!!
    if(!param_client.getCurrentConfiguration(config, rclcpp::Duration(5))) RCLCPP_WARN(rclcpp::get_logger("schunk_gripper_example"), "Parameter weren't loaded");
    move_abs_client.waitForServer();
  */  
    //acknowledge if the gripper has an error
    Acknowledge::Request::SharedPtr acknowledge_req;

 //   rclcpp::topic::waitForMessage<diagnostic_msgs::DiagnosticArray>("diagnostics");
    if(diagnostic_msg.status[0].level == diagnostic_msgs::msg::DiagnosticStatus::ERROR) 
    {
        auto response = acknowledge_client->async_send_request(acknowledge_req);
        if(response.get()->acknowledged)
        RCLCPP_INFO(node->get_logger(), "AN ERROR WAS ACKNOWLEDGED!");
    }
    
    handshake = state_msg.command_received_toggle;

    //Move to 0 mm with 100 mm/s and get result
    moveAbsoluteAndWaitForResult(move_abs_client);
    
    //(Not necessary, just that the gripper do not move immediately)
    

    //Move relative
    moveRelative(move_rel_client);
    
    //Let gripper move 5s
    std::this_thread::sleep_for(std::chrono::seconds(5));
    
    //Stop gripper with service
    stop(stop_client);
    
    //After the stop, the result of the action will be printed
    moveRelativeResult(move_rel_client);

    std::this_thread::sleep_for(std::chrono::seconds(3));

    //Move relative again
    moveRelative(move_rel_client);

    //Let it move 5s seconds
    std::this_thread::sleep_for(std::chrono::seconds(5));
    
    //Cancel the goal
    cancelMoveRelative(move_rel_client);
    
    //Print the result
    moveRelativeResult(move_rel_client);

    //Fast stop/cancel causes an Error. Show the error in diagnostics (or state as error_code)
    //Print diagnostics
  //  rclcpp::topic::waitForMessage<diagnostic_msgs::DiagnosticArray>("diagnostics");
    RCLCPP_ERROR(node->get_logger(),"%s",diagnostic_msg.status.at(0).message.c_str());
    
    std::this_thread::sleep_for(std::chrono::seconds(3));
    
    acknowledge(acknowledge_client);

    //print diagnostics
  //  rclcpp::topic::waitForMessage<diagnostic_msgs::DiagnosticArray>("diagnostics");
    RCLCPP_INFO(node->get_logger(), "%s", diagnostic_msg.status.at(0).message.c_str());

    std::this_thread::sleep_for(std::chrono::seconds(3));

    //Change grip prehold time and workpiece release delta
  //  changeConfiguration(param_client);
//
    std::this_thread::sleep_for(std::chrono::seconds(3));
    
    //Grip
    if(model.find("EGU") != std::string::npos) gripEgu(grip_egu_client);
    else if(model.find("EGK") != std::string::npos) gripEgk(grip_egk_client);
    else RCLCPP_INFO(node->get_logger(), "No model");

  //  rclcpp::topic::waitForMessage<schunk_gripper::state>("state");
    
    //Release Workpiece
    if(state_msg.workpiece_gripped == true) releaseWorkpiece(release_client);
    else RCLCPP_INFO(node->get_logger(), "No Workpiece gripped!");

    std::this_thread::sleep_for(std::chrono::seconds(3));

    RCLCPP_INFO(node->get_logger(), "Set configuration!");
    //Optionally you can control some basic Commands with dynamic reconfigure parameters
    //GET CURRENT CONFIGURATION!!!!! ELSE YOU COULD GET THE GRIPPER TO MOVE, EVEN IF YOU DONT WANT IT!!!
  //  moveAbsWithConfig(param_client);

    spin_thread.join();

    return 0;
}
