//This Client shows you how to control your gripper in program. If you do something else, as long as the program works, 
//this program will exhibit undefined behavior.
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/wait_for_message.hpp"
#include <rclcpp/parameter.hpp>
#include <rcl_interfaces/msg/parameter_event.hpp>
#include "sensor_msgs/msg/joint_state.hpp"
#include <diagnostic_msgs/msg/diagnostic_array.hpp>

#include "schunk_egu_egk_gripper_interfaces/msg/state.hpp"
#include "schunk_egu_egk_gripper_interfaces/srv/acknowledge.hpp"
#include "schunk_egu_egk_gripper_interfaces/srv/stop.hpp"
#include "schunk_egu_egk_gripper_interfaces/srv/fast_stop.hpp"
#include "schunk_egu_egk_gripper_interfaces/srv/prepare_for_shutdown.hpp"
#include "schunk_egu_egk_gripper_interfaces/srv/softreset.hpp"
#include "schunk_egu_egk_gripper_interfaces/srv/brake_test.hpp"
#include "schunk_egu_egk_gripper_interfaces/srv/change_ip.hpp"
#include "schunk_egu_egk_gripper_interfaces/srv/parameter_get.hpp"
#include "schunk_egu_egk_gripper_interfaces/srv/parameter_set.hpp"
#include "schunk_egu_egk_gripper_interfaces/srv/release_for_manual_movement.hpp"
#include "schunk_egu_egk_gripper_interfaces/srv/gripper_info.hpp"

#include "schunk_egu_egk_gripper_interfaces/action/grip_with_velocity.hpp"
#include "schunk_egu_egk_gripper_interfaces/action/grip_with_position_and_velocity.hpp"
#include "schunk_egu_egk_gripper_interfaces/action/move_to_absolute_position.hpp"
#include "schunk_egu_egk_gripper_interfaces/action/move_to_relative_position.hpp"
#include "schunk_egu_egk_gripper_interfaces/action/release_workpiece.hpp"
#include "schunk_egu_egk_gripper_interfaces/action/grip.hpp"
#include "schunk_egu_egk_gripper_interfaces/action/grip_with_position.hpp"
#include "control_msgs/action/gripper_command.hpp"

schunk_egu_egk_gripper_interfaces::msg::State state_msg;
sensor_msgs::msg::JointState joint_state_msg;
diagnostic_msgs::msg::DiagnosticArray diagnostic_msg;
bool handshake;
std::string name_space;
    using State = schunk_egu_egk_gripper_interfaces::msg::State;

    using Acknowledge = schunk_egu_egk_gripper_interfaces::srv::Acknowledge;
    using BrakeTest = schunk_egu_egk_gripper_interfaces::srv::BrakeTest;
    using Stop = schunk_egu_egk_gripper_interfaces::srv::Stop;
    using FastStop =  schunk_egu_egk_gripper_interfaces::srv::FastStop;
    using ReleaseForManualMovement = schunk_egu_egk_gripper_interfaces::srv::ReleaseForManualMovement;
    using Softreset = schunk_egu_egk_gripper_interfaces::srv::Softreset;
    using PrepareForShutdown = schunk_egu_egk_gripper_interfaces::srv::PrepareForShutdown;
    using GripperInfo= schunk_egu_egk_gripper_interfaces::srv::GripperInfo;
    using ChangeIp = schunk_egu_egk_gripper_interfaces::srv::ChangeIp;
    using ParameterGet = schunk_egu_egk_gripper_interfaces::srv::ParameterGet;
    using ParameterSet = schunk_egu_egk_gripper_interfaces::srv::ParameterSet;

    using MoveToAbsolutePosition = schunk_egu_egk_gripper_interfaces::action::MoveToAbsolutePosition;
    using MoveToRelativePosition = schunk_egu_egk_gripper_interfaces::action::MoveToRelativePosition;
    using GripWithVelocity = schunk_egu_egk_gripper_interfaces::action::GripWithVelocity;
    using Grip = schunk_egu_egk_gripper_interfaces::action::Grip;
    using GripWithPositionAndVelocity = schunk_egu_egk_gripper_interfaces::action::GripWithPositionAndVelocity;
    using GripWithPosition = schunk_egu_egk_gripper_interfaces::action::GripWithPosition;
    using ReleaseWorkpiece = schunk_egu_egk_gripper_interfaces::action::ReleaseWorkpiece;
    using GripperCommand = control_msgs::action::GripperCommand;

//Callback Functions

/**
 * @brief Callback function for handling the state message.
 * 
 * This function is called when a new state message is received. It updates the state_msg variable with the contents of the message.
 * 
 * @param msg The shared pointer to the state message.
 */
void stateCallback(const State::SharedPtr msg)
{
    state_msg = *msg;
}

/**
 * @brief Callback function for the joint state message.
 * 
 * This function is called whenever a new joint state message is received.
 * It updates the joint_state_msg variable with the latest joint state data.
 * 
 * @param msg The joint state message.
 */
void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    joint_state_msg = *msg;
}

/**
 * @brief Callback function for handling diagnostic messages.
 * 
 * This function is called when a diagnostic message is received. It updates the diagnostic_msg variable with the received message.
 * 
 * @param msg The received diagnostic message.
 */
void diagnosticsCallback(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg)
{   
    diagnostic_msg = *msg;
}

/**
 * @brief Callback function for the feedback of the MoveToAbsolutePosition action.
 * 
 * This function is called when the action server sends feedback about the progress of the goal.
 * It logs the current position of the gripper in millimeters.
 * 
 * @param goal_handle The goal handle associated with the action.
 * @param feedback The feedback received from the action server.
 */
void feedbackCB(rclcpp_action::ClientGoalHandle<MoveToAbsolutePosition>::SharedPtr, const std::shared_ptr<const MoveToAbsolutePosition::Feedback> feedback)
{
    RCLCPP_INFO(rclcpp::get_logger("schunk_gripper_example"), "Gripper is at: %f mm",feedback->absolute_position);
}

/**
 * @brief Callback function called when the execution of the EGU action is done.
 * 
 * This function logs whether the workpiece has been gripped or not.
 * 
 * @param result The result of the EGU action.
 */
void doneEguCb(const rclcpp_action::ClientGoalHandle<Grip>::WrappedResult & result)
{   
    RCLCPP_INFO(rclcpp::get_logger("schunk_gripper_example"), "%s", result.result->workpiece_gripped ? "workpiece_gripped" : "Not workpiece_gripped");
}

/**
 * @brief Callback function called when the goal is done for the EGK action client.
 * 
 * This function prints whether the workpiece is gripped or not based on the result.
 * 
 * @param result The result of the EGK action client goal.
 */
void doneEgkCb(const rclcpp_action::ClientGoalHandle<GripWithVelocity>::WrappedResult & result)
{
    RCLCPP_INFO(rclcpp::get_logger("schunk_gripper_example"), "%s", result.result->workpiece_gripped ? "workpiece_gripped" : "Not workpiece_gripped");
}

/**
 * @brief Callback function for grip action feedback.
 * 
 * This function is called when feedback is received from the grip action server.
 * It logs a message if the workpiece pre-grip has started.
 * 
 * @param goal_handle The goal handle for the grip action.
 * @param feedback The feedback received from the grip action server.
 */
void gripFeedback(rclcpp_action::ClientGoalHandle<Grip>::SharedPtr, const std::shared_ptr<const Grip::Feedback> feedback)
{    
    if(feedback->workpiece_pre_grip_started == true) 
    RCLCPP_INFO_ONCE(rclcpp::get_logger("schunk_gripper_example"), "Pre_grip started"); 
}

/**
 * @brief Callback function for grip velocity feedback.
 * 
 * This function is called when receiving feedback from the grip velocity action.
 * It checks if the workpiece pre-grip has started and logs a message if it is true.
 * 
 * @param goal_handle The goal handle for the grip velocity action.
 * @param feedback The feedback received from the grip velocity action.
 */
void gripVelFeedback(rclcpp_action::ClientGoalHandle<GripWithVelocity>::SharedPtr, const std::shared_ptr<const GripWithVelocity::Feedback> feedback)
{    
    if(feedback->workpiece_pre_grip_started == true) 
    RCLCPP_INFO_ONCE(rclcpp::get_logger("schunk_gripper_example"), "Pre_grip started");
}


//Control Functions

/**
 * Moves the gripper to an absolute position and waits for the result.
 * 
 * @param move_abs_client The client for the MoveToAbsolutePosition action server.
 */
void moveAbsoluteAndWaitForResult(rclcpp_action::Client<MoveToAbsolutePosition>::SharedPtr move_abs_client)
{
    //Move to 0 mm with 100 mm/s
    MoveToAbsolutePosition::Goal goal_abs;
    goal_abs.absolute_position = 90.0;
    goal_abs.velocity_of_movement = 10.0;

    auto send_goal_options = rclcpp_action::Client<MoveToAbsolutePosition>::SendGoalOptions();
    send_goal_options.feedback_callback = std::bind(feedbackCB, std::placeholders::_1, std::placeholders::_2);

    auto result = move_abs_client->async_send_goal(goal_abs, send_goal_options);
        
    result.get();
    RCLCPP_WARN(rclcpp::get_logger("schunk_gripper_example"), "%s: %f mm", state_msg.doing_command.c_str(), goal_abs.absolute_position);

    if(move_abs_client->async_get_result(result.get()).get().code == rclcpp_action::ResultCode::SUCCEEDED)
    RCLCPP_INFO(rclcpp::get_logger("schunk_gripper_example"), "At the right position");     
}

/**
 * Moves the gripper to a relative position and stops it.
 *
 * @param move_rel_client The client for the MoveToRelativePosition action server.
 * @param stop_client The client for the Stop service.
 */
void moveRelativeAndStop(rclcpp_action::Client<MoveToRelativePosition>::SharedPtr move_rel_client, rclcpp::Client<Stop>::SharedPtr stop_client)
{
    MoveToRelativePosition::Goal goal_rel;

    goal_rel.signed_relative_position = -50.0;
    goal_rel.velocity_of_movement = 7.0;
    auto goal_handle = move_rel_client->async_send_goal(goal_rel);
    goal_handle.get();

    RCLCPP_WARN(rclcpp::get_logger("schunk_gripper_example"), "%s: %f mm", state_msg.doing_command.c_str(), goal_rel.signed_relative_position);

    std::this_thread::sleep_for(std::chrono::seconds(3));

    auto stop_req = std::make_shared<Stop::Request>();
    auto resp = stop_client->async_send_request(stop_req);
    bool stopped = resp.get()->stopped;
    
    RCLCPP_INFO(rclcpp::get_logger("schunk_gripper_example"), "%s", stopped ? "Stopped!" : "Not stopped!");
    auto res = move_rel_client->async_get_result(goal_handle.get());
    auto final_res = res.get();
    
    RCLCPP_INFO_STREAM(rclcpp::get_logger("schunk_gripper_example"), "Current Position: " << final_res.result->absolute_position);
    RCLCPP_INFO(rclcpp::get_logger("schunk_gripper_example"), "ResultCode: %i", static_cast<int>(final_res.code));
}

/**
 * @brief Moves the gripper to a relative position and cancels the movement.
 * 
 * This function sends a goal to move the gripper to a relative position and then cancels the movement
 * after a specified duration. It performs a fast stop to halt the movement.
 * 
 * @param move_rel_client A shared pointer to the action client for moving the gripper to a relative position.
 */
void MoveRelativeAndCancel(rclcpp_action::Client<MoveToRelativePosition>::SharedPtr move_rel_client)
{
    MoveToRelativePosition::Goal goal_rel;

    goal_rel.signed_relative_position = -50.0;
    goal_rel.velocity_of_movement = 7.0;
    auto goal_handle = move_rel_client->async_send_goal(goal_rel);
    std::this_thread::sleep_for(std::chrono::seconds(3));
    auto goal_handle_cancel = move_rel_client->async_cancel_goal(goal_handle.get());         //This performs a fast stop

    move_rel_client->async_get_result(goal_handle.get()).get();
    RCLCPP_WARN(rclcpp::get_logger("schunk_gripper_example"), "%s", state_msg.doing_command.c_str());
}

/**
 * @brief Sends an acknowledge request to the acknowledge server.
 * 
 * This function sends an acknowledge request to the acknowledge server using the provided client.
 * It logs the result of the request as "Acknowledged" or "Not acknowledged".
 * 
 * @param acknowledge_client The client used to send the acknowledge request.
 */
void acknowledge(rclcpp::Client<Acknowledge>::SharedPtr acknowledge_client)
{
    Acknowledge::Request::SharedPtr acknowledge_srv = std::make_shared<Acknowledge::Request>();

    RCLCPP_INFO(rclcpp::get_logger("schunk_gripper_example"), "Call acknowledge-server.");
    //To end the error: acknowledge
    bool acknowledged = acknowledge_client->async_send_request(acknowledge_srv).get()->acknowledged;
    RCLCPP_INFO(rclcpp::get_logger("schunk_gripper_example"), "%s", acknowledged ? "Acknowledged" : "Not acknowledged");

}

/**
 * @brief Changes the configuration of the gripper.
 * 
 * This function retrieves the current configuration parameters of the gripper,
 * modifies them, and sets the new configuration parameters.
 * 
 * @param param_client A shared pointer to the AsyncParametersClient used to retrieve and set parameters.
 */
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

/**
 * @brief Sends a grip command to the gripper.
 *
 * This function sends a grip command to the gripper using the provided grip client.
 * It sets the gripping force and grip direction in the goal message, and registers
 * callback functions for result and feedback handling.
 *
 * @param grip_client The grip client used to send the grip command.
 */
void gripEgu(rclcpp_action::Client<Grip>::SharedPtr grip_client)
{
    Grip::Goal goal_grip;
    goal_grip.gripping_force  = 50;                     //Percent
    goal_grip.grip_direction = 0;                      //grip direction
    rclcpp_action::Client<Grip>::SendGoalOptions goal_opt;
    goal_opt.result_callback = std::bind(&doneEguCb, std::placeholders::_1);
    goal_opt.feedback_callback = std::bind(&gripFeedback, std::placeholders::_1, std::placeholders::_2);
    
    auto goal_handle = grip_client->async_send_goal(goal_grip, goal_opt);
    if(goal_handle.get()->is_result_aware())
    {   
        RCLCPP_WARN(rclcpp::get_logger("schunk_gripper_example"), "%s", state_msg.doing_command.c_str());
        grip_client->async_get_result(goal_handle.get()).get();
    }
} 

/**
 * @brief Executes the grip action with the EGK gripper.
 *
 * This function sends a goal to the grip action server using the provided grip_client.
 * The goal specifies the gripping force, grip direction, and velocity of movement.
 * It also sets the result and feedback callbacks for the goal.
 * After sending the goal, it waits for the handshake to be received before proceeding.
 * Once the handshake is received, it logs a warning message and retrieves the result of the goal.
 *
 * @param grip_client The client for the grip action server.
 */
void gripEgk(rclcpp_action::Client<GripWithVelocity>::SharedPtr grip_client)
{
    GripWithVelocity::Goal goal_grip;
    goal_grip.gripping_force  = 50.0;                       //Percent
    goal_grip.grip_direction = false;                      //grip direction
    goal_grip.velocity_of_movement = 0.0;
    rclcpp_action::Client<GripWithVelocity>::SendGoalOptions goal_opt;
    goal_opt.result_callback = std::bind(&doneEgkCb, std::placeholders::_1);
    goal_opt.feedback_callback = std::bind(&gripVelFeedback, std::placeholders::_1, std::placeholders::_2);
    handshake = state_msg.command_received_toggle;
    auto goal_handle = grip_client->async_send_goal(goal_grip, goal_opt);
    
    if(goal_handle.get()->is_result_aware())
    {
        while(handshake == state_msg.command_received_toggle) std::this_thread::sleep_for(std::chrono::milliseconds(15));


        RCLCPP_WARN(rclcpp::get_logger("schunk_gripper_example"), "%s", state_msg.doing_command.c_str());
        grip_client->async_get_result(goal_handle.get()).get();
    }
}

/**
 * @brief Releases the workpiece using the release_client.
 * 
 * @param release_client The client used to send the release goal.
 */
void releaseWorkpiece(rclcpp_action::Client<ReleaseWorkpiece>::SharedPtr release_client)
{
        ReleaseWorkpiece::Goal release_goal;
        handshake = state_msg.command_received_toggle;
        auto goal_handle = release_client->async_send_goal(release_goal);

        while(handshake == state_msg.command_received_toggle) std::chrono::milliseconds(15);

        RCLCPP_WARN(rclcpp::get_logger("schunk_gripper_example"), "%s", state_msg.doing_command.c_str());
        auto result = (release_client->async_get_result(goal_handle.get())).get().result->released_workpiece;
        RCLCPP_WARN(rclcpp::get_logger("schunk_gripper_example"), "%s" , result ? "released_workpiece" : "not released_workpiece");
        
}

/**
 * @brief Moves the gripper to an absolute position with the specified configuration parameters.
 * 
 * This function retrieves the control parameters for moving the gripper from the parameter server
 * using the provided parameter client. It then sets the configuration parameters to the desired values
 * and updates them on the parameter server. Finally, it waits for the gripper to reach the desired position
 * and logs a message indicating if the position was reached.
 * 
 * @param param_client A shared pointer to the parameter client used to retrieve and update the parameters.
 */
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


/**
 * @brief Spins the given node using a multi-threaded executor.
 * 
 * This function adds the given node to a multi-threaded executor and spins it until shutdown is called.
 * 
 * @param node A shared pointer to the node to be spun.
 */
void spinFunction(std::shared_ptr<rclcpp::Node> node)
{
    rclcpp::executors::MultiThreadedExecutor executor;
    
    executor.add_node(node);
    
    executor.spin();
    
    rclcpp::shutdown();
}

/**
 * @brief The main function of the program.
 *
 * This function initializes the ROS 2 node, retrieves the model parameter of the gripper, creates clients for various gripper commands, and executes the gripper commands in the following order:
 * 1. Acknowledge any errors in the gripper.
 * 2. Move the gripper to the absolute position of 0 mm with a velocity of 100 mm/s.
 * 3. Move the gripper relative to its current position and stop.
 * 4. Cancel the ongoing relative movement of the gripper.
 * 5. Acknowledge any errors in the gripper again.
 * 6. Change the configuration parameters of the gripper.
 * 7. Grip the workpiece (if the gripper model is EGU or EGK).
 * 8. Release the workpiece (if it is currently gripped).
 * 8/9. Move the gripper with Parameters (Not recommended)
 *
 * @param argc The number of command-line arguments.
 * @param argv An array of command-line arguments.
 * @return int The exit status of the program.
 */
int main(int argc, char** argv)
{
    
    rclcpp::init(argc, argv);
    std::string model;
    std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("schunk_gripper_example");
    
    std::thread spin_thread(&spinFunction, node);
    
    //IS SCHUNK GRIPPER ACTIVE

    name_space = "/EGK_50_M_B/";

    std::vector<rclcpp::Parameter> model_param;

    auto param_client = std::make_shared<rclcpp::AsyncParametersClient>(node, name_space+"schunk_gripper_driver");
    if(param_client->wait_for_service(std::chrono::seconds(5)))
    {
        model_param = param_client->get_parameters({"model"}).get();
        RCLCPP_INFO(node->get_logger(), "Model: %s", model_param[0].as_string().c_str());
    }
    else
    {
        RCLCPP_ERROR_STREAM(node->get_logger(), "Node " << name_space << "schunk_gripper_driver not found");
    }
    //ALL SERVICES
    auto acknowledge_client = node->create_client<Acknowledge> (name_space+"acknowledge");
    auto stop_client = node->create_client<Stop>(name_space+"stop");
//    auto softreset_client = node->create_client<Softreset>("softreset");
//    auto release_for_manual_mov_client = node->create_client<ReleaseForManualMovement>("release_for_manual_movement");
//    auto prepare_for_shutdown_client = node->create_client<PrepareForShutdown>("prepare_for_shutdown");
//    auto fast_stop_client = node->create_client<FastStop>("fast_stop");
//    auto info_client = node->create_client<GripperInfo>("gripper_info");
    //ALL ACTIONS

    auto move_abs_client = rclcpp_action::create_client<MoveToAbsolutePosition>(node, name_space+"move_to_absolute_position");
    auto move_rel_client = rclcpp_action::create_client<MoveToRelativePosition>(node, name_space+"move_to_relative_position");
    
    rclcpp_action::Client<Grip>::SharedPtr grip_egu_client;
    rclcpp_action::Client<GripWithPosition>::SharedPtr grp_w_pos_egu_client;

    rclcpp_action::Client<GripWithVelocity>::SharedPtr grip_egk_client;
    rclcpp_action::Client<GripWithPositionAndVelocity>::SharedPtr grp_w_pos_egk_client;

    if(model_param[0].as_string().find("EGU") != std::string::npos)
    {
        grip_egu_client = rclcpp_action::create_client<Grip>(node, name_space+"grip");
        grp_w_pos_egu_client = rclcpp_action::create_client<GripWithPosition>(node, name_space+"grip_with_position");
    }
    else if(model_param[0].as_string().find("EGK") != std::string::npos)
    {
        grip_egk_client = rclcpp_action::create_client<GripWithVelocity>(node, name_space+"grip");
        grp_w_pos_egk_client = rclcpp_action::create_client<GripWithPositionAndVelocity>(node, name_space+"grip_with_position");
    }
    auto release_client = rclcpp_action::create_client<ReleaseWorkpiece>(node, name_space+"release_workpiece");
    
    auto control_client = rclcpp_action::create_client<GripperCommand>(node, name_space+"gripper_control");

    //TOPICS
    auto state_sub = node->create_subscription<State>(name_space+"state", 1, stateCallback);
    auto joint_state_sub = node->create_subscription<sensor_msgs::msg::JointState>(name_space+"joint_states", 1, jointStateCallback);
    auto diagnostics_sub = node->create_subscription<diagnostic_msgs::msg::DiagnosticArray>("diagnostics", 1, diagnosticsCallback);

    //acknowledge if the gripper has an error
    rclcpp::wait_for_message(diagnostic_msg, node, "diagnostics", std::chrono::seconds(3));
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
    

    //Move relative
    moveRelativeAndStop(move_rel_client, stop_client);
    
    //Cancel the goal
    MoveRelativeAndCancel(move_rel_client);
    
    acknowledge(acknowledge_client);

    //Change grip prehold time and workpiece release delta
    changeConfiguration(param_client);
    
    //Grip
    if(model_param[0].as_string().find("EGU") != std::string::npos) gripEgu(grip_egu_client);
    else if(model_param[0].as_string().find("EGK") != std::string::npos) gripEgk(grip_egk_client);
    else RCLCPP_INFO(node->get_logger(), "No model");

    
    //Release Workpiece
    if(state_msg.workpiece_gripped == true) releaseWorkpiece(release_client);
    else RCLCPP_INFO(node->get_logger(), "No Workpiece workpiece_gripped!");

    RCLCPP_INFO(node->get_logger(), "Set configuration!");
    //Optionally you can control some basic Commands with dynamic reconfigure parameters (Not recommended)
    moveAbsWithConfig(param_client);

    rclcpp::shutdown();
    spin_thread.join();

    return 0;
}
