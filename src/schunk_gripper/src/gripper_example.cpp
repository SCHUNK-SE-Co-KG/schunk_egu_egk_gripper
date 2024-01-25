//This Client shows you how to control your gripper in program. If you do something else, as long as the program works, 
//this program will exhibit undefined behavior.
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include <rclcpp/parameter.hpp>
#include <rcl_interfaces/msg/parameter_event.hpp>
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
std::string name_space;

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

void doneEguCb(const rclcpp_action::ClientGoalHandle<Grip>::WrappedResult & result)
{   
    RCLCPP_INFO(rclcpp::get_logger("schunk_gripper_example"), "%s", result.result->gripped ? "gripped" : "Not gripped");
}

void doneEgkCb(const rclcpp_action::ClientGoalHandle<GripWithVel>::WrappedResult & result)
{
    RCLCPP_INFO(rclcpp::get_logger("schunk_gripper_example"), "%s", result.result->gripped ? "gripped" : "Not gripped");
}

void gripFeedback(rclcpp_action::ClientGoalHandle<Grip>::SharedPtr, const std::shared_ptr<const Grip::Feedback> feedback)
{    
    if(feedback->pre_grip == true) 
    RCLCPP_INFO_ONCE(rclcpp::get_logger("schunk_gripper_example"), "Pre_grip started"); 
}
void gripVelFeedback(rclcpp_action::ClientGoalHandle<GripWithVel>::SharedPtr, const std::shared_ptr<const GripWithVel::Feedback> feedback)
{    
    if(feedback->pre_grip == true) 
    RCLCPP_INFO_ONCE(rclcpp::get_logger("schunk_gripper_example"), "Pre_grip started");
}


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

void moveRelativeAndStop(rclcpp_action::Client<MovRelPos>::SharedPtr move_rel_client, rclcpp::Client<Stop>::SharedPtr stop_client)
{
    MovRelPos::Goal goal_rel;

    goal_rel.distance = -50.0;
    goal_rel.velocity = 7.0;
    auto goal_handle = move_rel_client->async_send_goal(goal_rel);
    goal_handle.get();
    std::this_thread::sleep_for(std::chrono::seconds(5));

    RCLCPP_WARN(rclcpp::get_logger("schunk_gripper_example"), "%s: %f mm", state_msg.doing_command.c_str(), goal_rel.distance);
    
    auto stop_req = std::make_shared<Stop::Request>();
    auto resp = stop_client->async_send_request(stop_req);
    bool stopped = resp.get()->stopped;
    
    RCLCPP_INFO(rclcpp::get_logger("schunk_gripper_example"), "%s", stopped ? "Stopped!" : "Not stopped!");
    auto res = move_rel_client->async_get_result(goal_handle.get());
    auto final_res = res.get();
    
    RCLCPP_INFO_STREAM(rclcpp::get_logger("schunk_gripper_example"), "Current Position: " << final_res.result->current_position);
    RCLCPP_INFO(rclcpp::get_logger("schunk_gripper_example"), "ResultCode: %i", static_cast<int>(final_res.code));
}

void MoveRelativeAndCancel(rclcpp_action::Client<MovRelPos>::SharedPtr move_rel_client)
{
    MovRelPos::Goal goal_rel;

    goal_rel.distance = -50.0;
    goal_rel.velocity = 7.0;
    auto goal_handle = move_rel_client->async_send_goal(goal_rel);
    
    std::this_thread::sleep_for(std::chrono::seconds(5));
    
    auto goal_handle_cancel = move_rel_client->async_cancel_goal(goal_handle.get());         //This performs a fast stop

    RCLCPP_WARN(rclcpp::get_logger("schunk_gripper_example"), "%s", state_msg.doing_command.c_str());
}

void acknowledge(rclcpp::Client<Acknowledge>::SharedPtr acknowledge_client)
{
    Acknowledge::Request::SharedPtr acknowledge_srv = std::make_shared<Acknowledge::Request>();
    
    RCLCPP_INFO(rclcpp::get_logger("schunk_gripper_example"), "Call acknowledge-server.");
    //To end the error: acknowledge
    bool acknowledged = acknowledge_client->async_send_request(acknowledge_srv).get()->acknowledged;
    RCLCPP_INFO(rclcpp::get_logger("schunk_gripper_example"), "%s", acknowledged ? "Acknowledged" : "Not acknowledged");

}

void changeConfiguration(std::shared_ptr<rclcpp::AsyncParametersClient> param_client)
{
    //GET CURRENT CONFIGURATION!!!!! ELSE YOU COULD SAY THE GRIPPER TO MOVE, EVEN IF YOU DONT WANT IT!!!
    std::vector<std::string> parameter_names;
    parameter_names.push_back("Gripper_Parameter.wp_release_delta");
    parameter_names.push_back("Gripper_Parameter.grp_prehold_time");
    auto parameter_fut = param_client->get_parameters(parameter_names);
    auto parameter = parameter_fut.get();
    RCLCPP_INFO(rclcpp::get_logger("schunk_gripper_example") ,"Parameter before:");

    RCLCPP_INFO(rclcpp::get_logger("schunk_gripper_example") ,"%s: %f", parameter.at(0).get_name().c_str(), parameter.at(0).as_double());
    RCLCPP_INFO(rclcpp::get_logger("schunk_gripper_example") ,"%s: %li", parameter.at(1).get_name().c_str(), parameter.at(1).as_int());

    parameter.at(0) = rclcpp::Parameter(parameter.at(0).get_name(), rclcpp::ParameterValue(double(10.0)));
    parameter.at(1) = rclcpp::Parameter(parameter.at(1).get_name(), int64_t(10000));
    //And set Configurations
    auto set_params = param_client->set_parameters(parameter);
    set_params.get();

    RCLCPP_INFO(rclcpp::get_logger("schunk_gripper_example") ,"Parameter after:");
    
    RCLCPP_INFO(rclcpp::get_logger("schunk_gripper_example") ,"%s: %f", parameter.at(0).get_name().c_str(), parameter.at(0).as_double());
    RCLCPP_INFO(rclcpp::get_logger("schunk_gripper_example") ,"%s: %li", parameter.at(1).get_name().c_str(), parameter.at(1).as_int());
}

void gripEgu(rclcpp_action::Client<Grip>::SharedPtr grip_client)
{
    Grip::Goal goal_grip;
    goal_grip.effort  = 50;                     //Percent
    goal_grip.grp_dir = 0;                      //grip direction
    rclcpp_action::Client<Grip>::SendGoalOptions goal_opt;
    goal_opt.result_callback = std::bind(&doneEguCb, std::placeholders::_1);
    goal_opt.feedback_callback = std::bind(&gripFeedback, std::placeholders::_1, std::placeholders::_2);
    
    auto goal_handle = grip_client->async_send_goal(goal_grip, goal_opt);
    if(goal_handle.get()->is_result_aware())
    {   
        RCLCPP_WARN(rclcpp::get_logger("schunk_gripper_example"), "%s", state_msg.doing_command.c_str());
        grip_client->async_get_result(goal_handle.get());
    }
} 

void gripEgk(rclcpp_action::Client<GripWithVel>::SharedPtr grip_client)
{
    GripWithVel::Goal goal_grip;
    goal_grip.effort  = 50.0;                       //Percent
    goal_grip.grp_dir = false;                      //grip direction
    goal_grip.max_velocity = 0.0;
    rclcpp_action::Client<GripWithVel>::SendGoalOptions goal_opt;
    goal_opt.result_callback = std::bind(&doneEgkCb, std::placeholders::_1);
    goal_opt.feedback_callback = std::bind(&gripVelFeedback, std::placeholders::_1, std::placeholders::_2);
    handshake = state_msg.command_received_toggle;
    auto goal_handle = grip_client->async_send_goal(goal_grip, goal_opt);
    
    if(goal_handle.get()->is_result_aware())
    {
        while(handshake == state_msg.command_received_toggle) std::chrono::milliseconds(15);


        RCLCPP_WARN(rclcpp::get_logger("schunk_gripper_example"), "%s", state_msg.doing_command.c_str());
        grip_client->async_get_result(goal_handle.get());
    }
}

void releaseWorkpiece(rclcpp_action::Client<ReleaseWorkpiece>::SharedPtr release_client)
{
        ReleaseWorkpiece::Goal release_goal;
        handshake = state_msg.command_received_toggle;
        auto goal_handle = release_client->async_send_goal(release_goal);

        while(handshake == state_msg.command_received_toggle) std::chrono::milliseconds(15);

        RCLCPP_WARN(rclcpp::get_logger("schunk_gripper_example"), "%s", state_msg.doing_command.c_str());
        auto result = (release_client->async_get_result(goal_handle.get())).get().result->released;
        RCLCPP_WARN(rclcpp::get_logger("schunk_gripper_example"), "%s" , result ? "released" : "not released");
        
}

void moveAbsWithConfig(std::shared_ptr<rclcpp::AsyncParametersClient> param_client)
{
    std::vector<std::string> parameter_names;
    parameter_names.push_back("Control_Parameter.move_gripper");
    parameter_names.push_back("Control_Parameter.move_gripper_velocity");
    auto parameter_fut = param_client->get_parameters(parameter_names);
    auto parameter = parameter_fut.get();
    RCLCPP_INFO(rclcpp::get_logger("schunk_gripper_example") ,"Parameter before:");

    RCLCPP_INFO(rclcpp::get_logger("schunk_gripper_example") ,"%s: %f", parameter.at(0).get_name().c_str(), parameter.at(0).as_double());
    RCLCPP_INFO(rclcpp::get_logger("schunk_gripper_example") ,"%s: %f", parameter.at(1).get_name().c_str(), parameter.at(1).as_double());

    parameter.at(0) = rclcpp::Parameter(parameter.at(0).get_name(), rclcpp::ParameterValue(double(50.0)));
    parameter.at(1) = rclcpp::Parameter(parameter.at(1).get_name(), rclcpp::ParameterValue(double(50.0)));
    //And set Configurations
    auto set_params = param_client->set_parameters(parameter);
    set_params.get();

    RCLCPP_INFO(rclcpp::get_logger("schunk_gripper_example") ,"Parameter after:");
    
    RCLCPP_INFO(rclcpp::get_logger("schunk_gripper_example") ,"%s: %f", parameter.at(0).get_name().c_str(), parameter.at(0).as_double());
    RCLCPP_INFO(rclcpp::get_logger("schunk_gripper_example") ,"%s: %f", parameter.at(1).get_name().c_str(), parameter.at(1).as_double());

    while(state_msg.command_successfully_processed == false  && state_msg.position_reached == false && rclcpp::ok())
    if(diagnostic_msg.status.at(0).message != "NON_ERROR") break; 
    else std::this_thread::sleep_for(std::chrono::milliseconds(15));

    if(state_msg.position_reached == true) RCLCPP_INFO(rclcpp::get_logger("schunk_gripper_example"), "Position reached");
}


void spinFunction(std::shared_ptr<rclcpp::Node> node)
{
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
}

int main(int argc, char** argv)
{
    
    rclcpp::init(argc, argv);
    std::string model;
    std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("schunk_gripper_example");
    
    std::thread spin_thread(&spinFunction, node);
    
    //IS SCHUNK GRIPPER ACTIVE
    name_space = "EGK_50_M_B/";

    auto param_client = std::make_shared<rclcpp::AsyncParametersClient>(node, name_space+"schunk_gripper_driver");
    param_client->wait_for_service();
    auto model_param = param_client->get_parameters({"model"}).get();

    //ALL SERVICES
    auto acknowledge_client = node->create_client<Acknowledge> (name_space+"acknowledge");
    auto stop_client = node->create_client<Stop>(name_space+"stop");
//    auto softreset_client = node->create_client<Softreset>("softreset");
//    auto release_for_manual_mov_client = node->create_client<ReleaseForManMov>("release_for_manual_movement");
//    auto prepare_for_shutdown_client = node->create_client<PrepareForShutdown>("prepare_for_shutdown");
//    auto fast_stop_client = node->create_client<FastStop>("fast_stop");
//    auto info_client = node->create_client<GripperInfo>("gripper_info");
    //ALL ACTIONS

    auto move_abs_client = rclcpp_action::create_client<MovAbsPos>(node, name_space+"move_absolute");
    auto move_rel_client = rclcpp_action::create_client<MovRelPos>(node, name_space+"move_relative");
    
    rclcpp_action::Client<Grip>::SharedPtr grip_egu_client;
    rclcpp_action::Client<GripWithPos>::SharedPtr grp_w_pos_egu_client;

    rclcpp_action::Client<GripWithVel>::SharedPtr grip_egk_client;
    rclcpp_action::Client<GripWithPosVel>::SharedPtr grp_w_pos_egk_client;

    if(model_param[0].as_string().find("EGU") != std::string::npos)
    {
        grip_egu_client = rclcpp_action::create_client<Grip>(node, name_space+"grip");
        grp_w_pos_egu_client = rclcpp_action::create_client<GripWithPos>(node, name_space+"grip_with_pos");
    }
    else if(model_param[0].as_string().find("EGK") != std::string::npos)
    {
        grip_egk_client = rclcpp_action::create_client<GripWithVel>(node, name_space+"grip");
        grp_w_pos_egk_client = rclcpp_action::create_client<GripWithPosVel>(node, name_space+"grip_with_pos");
    }
    auto release_client = rclcpp_action::create_client<ReleaseWorkpiece>(node, name_space+"release_workpiece");
    
    auto control_client = rclcpp_action::create_client<GripperCommand>(node, name_space+"gripper_control");

    //TOPICS
    auto state_sub = node->create_subscription<schunk_gripper::msg::State>(name_space+"state", 1, stateCallback);
    auto joint_state_sub = node->create_subscription<sensor_msgs::msg::JointState>(name_space+"joint_states", 1, jointStateCallback);
    auto diagnostics_sub = node->create_subscription<diagnostic_msgs::msg::DiagnosticArray>("diagnostics", 1, diagnosticsCallback);

    std::this_thread::sleep_for(std::chrono::seconds(2));
    //acknowledge if the gripper has an error
    Acknowledge::Request::SharedPtr acknowledge_req;
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
    moveRelativeAndStop(move_rel_client, stop_client);

    //Let it move 5s seconds
    std::this_thread::sleep_for(std::chrono::seconds(5));
    
    //Cancel the goal
    MoveRelativeAndCancel(move_rel_client);

    //Fast stop/cancel causes an Error. Show the error in diagnostics (or state as error_code)
    //Print diagnostics
  //  rclcpp::topic::waitForMessage<diagnostic_msgs::DiagnosticArray>("diagnostics");
  //  RCLCPP_ERROR(node->get_logger(),"%s",diagnostic_msg.status.at(0).message.c_str());
    
    std::this_thread::sleep_for(std::chrono::seconds(3));
    
    acknowledge(acknowledge_client);

    //print diagnostics
  //  rclcpp::topic::waitForMessage<diagnostic_msgs::DiagnosticArray>("diagnostics");
 //   RCLCPP_INFO(node->get_logger(), "%s", diagnostic_msg.status.at(0).message.c_str());

    std::this_thread::sleep_for(std::chrono::seconds(3));

    //Change grip prehold time and workpiece release delta
    changeConfiguration(param_client);
//
    std::this_thread::sleep_for(std::chrono::seconds(3));
    
    //Grip
    if(model_param[0].as_string().find("EGU") != std::string::npos) gripEgu(grip_egu_client);
    else if(model_param[0].as_string().find("EGK") != std::string::npos) gripEgk(grip_egk_client);
    else RCLCPP_INFO(node->get_logger(), "No model");

  //  rclcpp::topic::waitForMessage<schunk_gripper::state>("state");
    
    //Release Workpiece
    if(state_msg.workpiece_gripped == true) releaseWorkpiece(release_client);
    else RCLCPP_INFO(node->get_logger(), "No Workpiece gripped!");

    std::this_thread::sleep_for(std::chrono::seconds(3));

    RCLCPP_INFO(node->get_logger(), "Set configuration!");
    //Optionally you can control some basic Commands with dynamic reconfigure parameters (Not recommended)
    moveAbsWithConfig(param_client);

    spin_thread.join();

    return 0;
}
