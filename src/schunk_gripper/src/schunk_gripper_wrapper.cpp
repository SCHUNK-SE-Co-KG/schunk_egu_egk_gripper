#include "schunk_gripper/schunk_gripper_wrapper.hpp"


std::recursive_mutex lock_mutex;  //Locks if something is receiving or posting data

 std::map<std::string, const char*> parameter_map = {
        { "Gripper_Parameter.grp_pos_margin", GRP_POS_MARGIN_INST},
        { "Gripper_Parameter.grp_prepos_delta", GRP_PREPOS_DELTA_INST },
        { "Gripper_Parameter.zero_pos_ofs", ZERO_POS_OFS_INST },
        { "Gripper_Parameter.grp_prehold_time", GRP_PREHOLD_TIME_INST },
        { "Gripper_Parameter.wp_release_delta", WP_RELEASE_DELTA_INST },
        { "Gripper_Parameter.wp_lost_distance", WP_LOST_DISTANCE_INST },
    };

//Initialize the ROS Driver
SchunkGripperNode::SchunkGripperNode(const rclcpp::NodeOptions &options) : 
    rclcpp::Node("schunk_gripper_driver", options),
    Gripper(this->declare_parameter("IP", "0.0.0.0", parameter_descriptor("IP-Address of the gripper"))),
    limiting_rate(1000) //limiting_rate for loops
{ 
    //Callback groups
    messages_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    services_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
 //   actions_group  = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
 //   rest  = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    
    rclcpp::PublisherOptions option_messages;
    option_messages.callback_group = messages_group;
    rclcpp::SubscriptionOptions option;
    option.callback_group = rest;
    //ParameterDescription read_only
    rcl_interfaces::msg::ParameterDescriptor paramDesc;
    paramDesc.read_only = true;
    
    paramDesc.description = "Frequency of state";
    state_frq = this->declare_parameter("state_frq", 60.0, paramDesc);

    paramDesc.description = "Frequency of joint states";
    j_state_frq = this->declare_parameter("rate", 60.0, paramDesc);

    //Cycletime
    cycletime = std::make_shared<rclcpp::Duration>(std::chrono::milliseconds(static_cast<int>(1/state_frq)*1000));
    //Set flags
    param_exe = false;
    action_active = false;
    action_move = false;
    //Set Parametereventhandler
    parameter_event_handler = std::make_shared<rclcpp::ParameterEventHandler>(this);

    //Actually no Command
    actual_command = "NO COMMAND";
    try
    {
    //get an error and a warn string
    error_str = getErrorString(0);
    warn_str = getErrorString(0);
    }
   catch(const char* res)
   {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Failed Connection to gripper. "<< res << std::endl);
      connection_error = res;
   }

    callback_gripper_param =  std::bind(&SchunkGripperNode::callback_gripper_parameter,this,std::placeholders::_1);
    callback_move_param =  std::bind(&SchunkGripperNode::callback_move_parameter,this ,std::placeholders::_1);

    if(start_connection == true) 
    {
        advertiseConnectionRelevant();
        ip_changed_with_all_param = true;
        connection_error = "OK";
    }
    else 
    {
        ip_changed_with_all_param = false;
        connection_error = "";
    }

    declareParameter();
    advertiseTopics();
    advertiseServices();
    advertiseActions();
}
//State msg update
void SchunkGripperNode::updateStateMsg()
{
    msg.command_received_toggle = gripperBitInput(COMMAND_RECEIVED_TOGGLE); 

    msg.doing_command = actual_command;       //It is the only msg...,that is not updated through "runGets()"
                                              //It is asynchronous with the others
    msg.command_successfully_processed = gripperBitInput(SUCCESS);
    msg.position_reached = gripperBitInput(POSITION_REACHED);
    msg.workpiece_gripped = gripperBitInput(GRIPPED);
    msg.workpiece_lost = gripperBitInput(WORK_PIECE_LOST);
    msg.wrong_workpiece = gripperBitInput(WRONG_WORKPIECE_DETECTED);
    msg.no_workpiece_detected = gripperBitInput(NO_WORKPIECE_DETECTED);

    msg.actual_cur = actual_cur;
    msg.actual_vel = actual_vel;
    msg.actual_pos = actual_pos;

    splitted_Diagnosis = splitDiagnosis();

    msg.error = splitted_Diagnosis[2];
    msg.warning = splitted_Diagnosis[1];
    msg.not_feasible = splitted_Diagnosis[0];

    msg.control_authority = gripperBitInput(CONTROL_AUTHORITY);
    msg.ready_for_operation = gripperBitInput(READY_FOR_OPERATION);
    msg.software_limit_reached = gripperBitInput(SOFTWARE_LIMIT);
    msg.ready_for_shutdown = gripperBitInput(READY_FOR_SHUTDOWN);
    msg.pre_grip_started = gripperBitInput(PRE_HOLDING);
    msg.grip_force_and_maintenance = gripperBitInput(BRAKE_ACTIVE);
}
//parameter limits
void SchunkGripperNode::advertiseServices()
{
    //Create Services
    acknowledge_service = this->create_service<Acknowledge>("acknowledge", std::bind(&SchunkGripperNode::acknowledge_srv,this,std::placeholders::_1,std::placeholders::_2), rmw_qos_profile_services_default, services_group);
    stop_service = this->create_service<Stop>("stop", std::bind(&SchunkGripperNode::stop_srv,this,std::placeholders::_1,std::placeholders::_2), rmw_qos_profile_services_default, services_group);
    softreset_service = this->create_service<Softreset>("softreset", std::bind(&SchunkGripperNode::softreset_srv,this,std::placeholders::_1,std::placeholders::_2), rmw_qos_profile_services_default, services_group);
    releaseForManualMov_service = this->create_service<ReleaseForManMov>("release_for_manual_movement", std::bind(&SchunkGripperNode::releaseForManualMov_srv,this,std::placeholders::_1,std::placeholders::_2), rmw_qos_profile_services_default, services_group);
    prepare_for_shutdown_service = this->create_service<PrepareForShutdown>("prepare_for_shutdown", std::bind(&SchunkGripperNode::prepare_for_shutdown_srv,this,std::placeholders::_1,std::placeholders::_2), rmw_qos_profile_services_default, services_group);
    fast_stop_service = this->create_service<FastStop>("fast_stop", std::bind(&SchunkGripperNode::fast_stop_srv,this,std::placeholders::_1,std::placeholders::_2), rmw_qos_profile_services_default, services_group);
    parameter_get_service = this->create_service<ParameterGet>("parameter_get", std::bind(&SchunkGripperNode::parameter_get_srv,this,std::placeholders::_1,std::placeholders::_2), rmw_qos_profile_services_default, services_group);
    parameter_set_service= this->create_service<ParameterSet>("parameter_set", std::bind(&SchunkGripperNode::parameter_set_srv,this,std::placeholders::_1,std::placeholders::_2), rmw_qos_profile_services_default, services_group);
    info_service = this->create_service<GripperInfo>("gripper_info", std::bind(&SchunkGripperNode::info_srv,this,std::placeholders::_1,std::placeholders::_2), rmw_qos_profile_services_default, rest);
    change_ip_service = this->create_service<ChangeIp>("reconnect",  std::bind(&SchunkGripperNode::change_ip_srv,this,std::placeholders::_1,std::placeholders::_2), rmw_qos_profile_services_default, services_group);
  
}

void SchunkGripperNode::advertiseActions()
{
    //Create Actions  
    move_abs_server = rclcpp_action::create_server<MovAbsPos>(this, "move_absolute", std::bind(&SchunkGripperNode::handle_goal<MovAbsPos::Goal>, this, std::placeholders::_1, std::placeholders::_2),
                                                                                     std::bind(&SchunkGripperNode::handle_cancel<MovAbsPos>, this, std::placeholders::_1),
                                                                                     std::bind(&SchunkGripperNode::handle_accepted_abs, this, std::placeholders::_1),rcl_action_server_get_default_options(), services_group);
   
    move_rel_server = rclcpp_action::create_server<MovRelPos>(this, "move_relative", std::bind(&SchunkGripperNode::handle_goal<MovRelPos::Goal>, this, std::placeholders::_1, std::placeholders::_2),
                                                                                     std::bind(&SchunkGripperNode::handle_cancel<MovRelPos>, this, std::placeholders::_1),
                                                                                     std::bind(&SchunkGripperNode::handle_accepted_rel, this, std::placeholders::_1),rcl_action_server_get_default_options(), services_group);  
    
    release_wp_server = rclcpp_action::create_server<ReleaseWorkpiece>(this, "release_workpiece",   std::bind(&SchunkGripperNode::handle_goal<ReleaseWorkpiece::Goal>, this, std::placeholders::_1, std::placeholders::_2),
                                                                                                    std::bind(&SchunkGripperNode::handle_cancel<ReleaseWorkpiece>, this, std::placeholders::_1),
                                                                                                    std::bind(&SchunkGripperNode::handle_accepted_release, this, std::placeholders::_1),rcl_action_server_get_default_options(), services_group);
    
    control_server = rclcpp_action::create_server<GripperCommand>(this, "gripper_control",          std::bind(&SchunkGripperNode::handle_goal<GripperCommand::Goal>, this, std::placeholders::_1, std::placeholders::_2),
                                                                                                    std::bind(&SchunkGripperNode::handle_cancel<GripperCommand>, this, std::placeholders::_1),
                                                                                                    std::bind(&SchunkGripperNode::handle_accepted_control, this, std::placeholders::_1),rcl_action_server_get_default_options(), services_group);
    
}

void SchunkGripperNode::advertiseTopics()
{
    //Advertise state
    last_time = this->now(); //Initialize last_time 
    statePublisher = this->create_publisher<schunk_gripper::msg::State>("state", 10);
    publish_state_timer=this->create_wall_timer(std::chrono::duration<double>(1 / state_frq), std::bind(&SchunkGripperNode::publishState, this), messages_group);
    
    //Initialize diagnostics
    gripper_updater = std::make_shared<diagnostic_updater::Updater>(this);
    gripper_updater->setHardwareID("Module");
    gripper_updater->add(model, std::bind(&SchunkGripperNode::gripperDiagnostics,this,std::placeholders::_1));
    
    //Look if joint_state_frq is less than state_frq
    if(j_state_frq > state_frq)
    {
        j_state_frq = state_frq;
        RCLCPP_WARN(this->get_logger(),"joint_state topic will publish with %f!", j_state_frq);
    }
    //Advertise joint_states
    jointStatePublisher = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    publish_joint_timer = this->create_wall_timer(std::chrono::duration<double>(1 / j_state_frq), std::bind(&SchunkGripperNode::publishJointState, this));
}

void SchunkGripperNode::advertiseConnectionRelevant()
{   
    //Need Firmware
    if(sw_version >= 502) brake_test_service = this->create_service<BrakeTest>("brake_test", std::bind(&SchunkGripperNode::brake_test_srv, this,std::placeholders::_1,std::placeholders::_2), rmw_qos_profile_services_default, services_group);
    //Grip-action need the Gripper model
    if(model.find("EGK") != std::string::npos)
    {
        grip_server = rclcpp_action::create_server<GripWithVel>(this, "grip",   std::bind(&SchunkGripperNode::handle_goal<GripWithVel::Goal>, this, std::placeholders::_1, std::placeholders::_2),
                                                                                std::bind(&SchunkGripperNode::handle_cancel<GripWithVel>, this, std::placeholders::_1),
                                                                                std::bind(&SchunkGripperNode::handle_accepted_grip_egk, this, std::placeholders::_1),rcl_action_server_get_default_options(), services_group);
        
        grip_w_pos_server = rclcpp_action::create_server<GripWithPosVel>(this, "grip_with_pos",     std::bind(&SchunkGripperNode::handle_goal<GripWithPosVel::Goal>, this, std::placeholders::_1, std::placeholders::_2),
                                                                                                    std::bind(&SchunkGripperNode::handle_cancel<GripWithPosVel>, this, std::placeholders::_1),
                                                                                                    std::bind(&SchunkGripperNode::handle_accepted_gripPos_egk, this, std::placeholders::_1),rcl_action_server_get_default_options(), services_group);
    
    }
    else if(model.find("EGU") != std::string::npos)
    {
        grip_egu_server = rclcpp_action::create_server<Grip>(this, "grip",  std::bind(&SchunkGripperNode::handle_goal<Grip::Goal>, this, std::placeholders::_1, std::placeholders::_2),
                                                                            std::bind(&SchunkGripperNode::handle_cancel<Grip>, this, std::placeholders::_1),
                                                                            std::bind(&SchunkGripperNode::handle_accepted_grip_egu, this, std::placeholders::_1),rcl_action_server_get_default_options(), services_group);
    
        grip_w_pos_egu_server = rclcpp_action::create_server<GripWithPos>(this, "grip_with_pos_egu",  std::bind(&SchunkGripperNode::handle_goal<GripWithPos::Goal>, this, std::placeholders::_1, std::placeholders::_2),
                                                                                                      std::bind(&SchunkGripperNode::handle_cancel<GripWithPos>, this, std::placeholders::_1),
                                                                                                      std::bind(&SchunkGripperNode::handle_accepted_gripPos_egu, this, std::placeholders::_1),rcl_action_server_get_default_options(), services_group);
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(),"Gripper was not found");
    }

    //Set model (not read only)
    this->declare_parameter("model", model, parameter_descriptor("Gripper-model"), true);
    //declare Moving Parameter
    this->declare_parameter("Control_Parameter.move_gripper_velocity", static_cast<double>(min_vel), parameter_descriptor("Changing the velocity for move_Gripper in mm/s", FloatingPointRange(min_vel, max_vel)));
    this->declare_parameter("Control_Parameter.move_gripper", actualPosInterval(), parameter_descriptor("Moving the gripper with parameter in mm", FloatingPointRange(min_pos, max_pos)));
    abs_pos_param = actualPosInterval();
    
    //Callbacks for Parameters
    cb_handle[0] =  parameter_event_handler->add_parameter_callback("Gripper_Parameter.use_brk", callback_gripper_param);
    cb_handle[1] =  parameter_event_handler->add_parameter_callback("Gripper_Parameter.grp_pos_margin", callback_gripper_param);
    cb_handle[2] =  parameter_event_handler->add_parameter_callback("Gripper_Parameter.grp_prepos_delta", callback_gripper_param);
    cb_handle[3] =  parameter_event_handler->add_parameter_callback("Gripper_Parameter.zero_pos_ofs", callback_gripper_param);
    cb_handle[4] =  parameter_event_handler->add_parameter_callback("Gripper_Parameter.grp_prehold_time", callback_gripper_param);
    cb_handle[5] =  parameter_event_handler->add_parameter_callback("Gripper_Parameter.wp_release_delta", callback_gripper_param);
    cb_handle[6] =  parameter_event_handler->add_parameter_callback("Gripper_Parameter.wp_lost_distance", callback_gripper_param);

    cb_handle[7] =  parameter_event_handler->add_parameter_callback("Control_Parameter.grip", callback_move_param);
    cb_handle[8] =  parameter_event_handler->add_parameter_callback("Control_Parameter.release_workpiece", callback_move_param);
    cb_handle[9] =  parameter_event_handler->add_parameter_callback("Control_Parameter.move_gripper", callback_move_param);
}

void SchunkGripperNode::declareParameter()
{   
    // Gripper Parameter
    this->declare_parameter("Gripper_Parameter.use_brk", false, parameter_descriptor("Use brake"));
    this->declare_parameter("Gripper_Parameter.grp_pos_margin", 2.0, parameter_descriptor("Grip position margin in mm", FloatingPointRange(1.0, 10.0, 0.01)));
    this->declare_parameter("Gripper_Parameter.grp_prepos_delta", 5.0, parameter_descriptor("Grip position delta in mm", FloatingPointRange(1.0, 50.0, 0.01)));
    this->declare_parameter("Gripper_Parameter.zero_pos_ofs", 0.0, parameter_descriptor("Zero position offset in mm", FloatingPointRange(-10000.0, 10000.0, 0.01)));
    this->declare_parameter("Gripper_Parameter.grp_prehold_time", 0, parameter_descriptor("Grip prehold time in ms", IntegerRange(0, 60000, 1)));
    this->declare_parameter("Gripper_Parameter.wp_release_delta", 5.0, parameter_descriptor("Workpiece release delta in mm", FloatingPointRange(1.0, 50.0, 0.01)));
    this->declare_parameter("Gripper_Parameter.wp_lost_distance", 1.0, parameter_descriptor("Max. distance after workpiece lost in mm", FloatingPointRange(0.1, 50.0, 0.01)));
    // Control Parameter
    this->declare_parameter("Control_Parameter.grip_direction", false, parameter_descriptor("Grip direction for parameter gripping"));
    this->declare_parameter("Control_Parameter.grip", false, parameter_descriptor("Grip with parameter"));
    this->declare_parameter("Control_Parameter.grip_force", 50.0, parameter_descriptor("Grip force in %", FloatingPointRange(50.0, 100.0, 0.01)));
    this->declare_parameter("Control_Parameter.release_workpiece", false, parameter_descriptor("Release Workpiece"));
}
// Helper function to create a FloatingPointRange
rcl_interfaces::msg::FloatingPointRange SchunkGripperNode::FloatingPointRange(double from, double to, double step) 
{
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = from;
    range.to_value = to;
    if(step != 0.0)
    range.step = step;
    return range;
}
// Helper function to create an IntegerRange
rcl_interfaces::msg::IntegerRange SchunkGripperNode::IntegerRange(int64_t from, int64_t to, uint64_t step) 
{
    rcl_interfaces::msg::IntegerRange range;
    range.from_value = from;
    range.to_value = to;
    if(step != 0)
    range.step = step;
    return range;
}
// Overloaded helper function to set parameter descriptor with range
rcl_interfaces::msg::ParameterDescriptor SchunkGripperNode::parameter_descriptor(const std::string& description, const rcl_interfaces::msg::FloatingPointRange& range) 
{
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = description;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    descriptor.dynamic_typing = true;
    descriptor.floating_point_range.push_back(range);
    return descriptor;
}

rcl_interfaces::msg::ParameterDescriptor SchunkGripperNode::parameter_descriptor(const std::string& description, const rcl_interfaces::msg::IntegerRange& range) 
{
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    descriptor.description = description;
    descriptor.integer_range.push_back(range);
    return descriptor;
}

rcl_interfaces::msg::ParameterDescriptor SchunkGripperNode::parameter_descriptor(const std::string& description) 
{
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.dynamic_typing = true;
    descriptor.description = description;
    return descriptor;
}
//Get actual Position within the given min and max interval
double SchunkGripperNode::actualPosInterval()
{
    if(actual_pos < min_pos) return static_cast<double>(min_pos);
    else if(actual_pos > max_pos)return static_cast<double>(max_pos);
    else return actual_pos;
}
//Handle incoming goal
template<typename goaltype>
rclcpp_action::GoalResponse SchunkGripperNode::handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const goaltype> goal)
{
    (void)uuid;
    (void)goal;

    if(gripperBitInput(GRIPPER_ERROR)) RCLCPP_ERROR(this->get_logger(), "Action will not be performed as long an error is active");
    else if(param_exe == false && action_active == false) return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    else  
    {
        const std::lock_guard<std::recursive_mutex> lock(lock_mutex);
        try
        {
            //send fast stop
            handshake = gripperBitInput(COMMAND_RECEIVED_TOGGLE);
            runPost(FAST_STOP);
            //if command received, get values
            if(handshake != gripperBitInput(COMMAND_RECEIVED_TOGGLE)) 
            while (rclcpp::ok() && check())
            runGets(); 

            gripper_updater->force_update();
        }
        catch(const char* server_err)
        {
            connection_error = server_err;
            RCLCPP_ERROR(this->get_logger(), "Failed Connection! %s", connection_error.c_str());
        }
    }
    
    return rclcpp_action::GoalResponse::REJECT;
}
//Deal with gripper, if the Command where successful done. Else throw an Exception
template<typename restype, typename goaltype>
void SchunkGripperNode::setFinalState(std::shared_ptr<restype> res, const std::shared_ptr<rclcpp_action::ServerGoalHandle<goaltype>> goal_handle)
{   
    if(gripperBitOutput(STOP))
    {
        finishedCommand();
        goal_handle->abort(res);
        return;
    }
    //Gripper succeeded or Workpiece lost...
    else if(gripperBitInput(SUCCESS)
          ||gripperBitInput(NO_WORKPIECE_DETECTED)
          ||gripperBitInput(WORK_PIECE_LOST) 
          ||gripperBitInput(WRONG_WORKPIECE_DETECTED))
    {
        finishedCommand();
        goal_handle->succeed(res);
        return;
    }
    //Gripper was fast stopped
    else if(!gripperBitOutput(FAST_STOP))
    {  
        finishedCommand();
        RCLCPP_WARN(this->get_logger(),"Fast stopped. Reason could be another goal or fast stop service.");
        goal_handle->abort(res);
        return;
    }
    //Cancel or new goal arrived
    else if(goal_handle->is_canceling())  
    {
        RCLCPP_ERROR(this->get_logger(),"Cancel Request received. Fast stop.");
        
        std::lock_guard<std::recursive_mutex> lock(lock_mutex);
            //Fast stop
            handshake = gripperBitInput(COMMAND_RECEIVED_TOGGLE);
            runPost(FAST_STOP);
            if(handshake != gripperBitInput(COMMAND_RECEIVED_TOGGLE)) 
            while (rclcpp::ok() && check())
            runGets();
        
        gripper_updater->force_update();
        finishedCommand();
        
        goal_handle->canceled(res);
        return;
    }

    else 
    {
        throw static_cast<int32_t>(plc_sync_input[3]);
    }
}
//Deal with Exceptions on different levels of the driver
template<typename GoalType, typename ResType>
void SchunkGripperNode::exceptionHandling(const std::shared_ptr<rclcpp_action::ServerGoalHandle<GoalType>> goal_handle,const int32_t &i, const std::shared_ptr<ResType> res)
{

        if(plc_sync_input[3] != 0)  ;// gripper_updater.force_update();
        //If gripper did not receive command
        else if(i == -1)
        {                 
            RCLCPP_ERROR(this->get_logger(),"GRIPPER DID NOT RECEIVE THE COMMAND!");          //ToDo Feedback to robot
        }
        else RCLCPP_WARN(this->get_logger(),"Action aborted!");

        finishedCommand();
        goal_handle->abort(res);
        return;

}   
//Action publishing its feedback as long the Gripper is moving
template<typename feedbacktype, typename goaltype>
void SchunkGripperNode::runActionMove(std::shared_ptr<feedbacktype> feedback, std::shared_ptr<rclcpp_action::ServerGoalHandle<goaltype>> goal_handle)
{   
    rclcpp::Time start_time = this->now();

    while(endCondition() && rclcpp::ok() && (!goal_handle->is_canceling()) && connection_error == "OK")
    {   //Publish as fast as in state
        if(*cycletime <= this->now() - start_time)
        {
            feedback->current_position = actual_pos;
            feedback->current_velocity = actual_vel;
            feedback->motor_current = actual_cur;

            goal_handle->publish_feedback(feedback);
            start_time = this->now();
        }
        //Limits the While loop
        limiting_rate.sleep();  //By low frequencies of state, cycletime.sleep() would block the action to end. Relevant if the gripper was stopped
    }
    //Get end values
    std::unique_lock<std::recursive_mutex> lock(lock_mutex);
    runGets();
    lock.unlock();
    //Gripper error not provoked through fast stop
    if(gripperBitInput(GRIPPER_ERROR) && gripperBitOutput(FAST_STOP)) throw static_cast<int32_t>(plc_sync_input[3]);

    feedback->current_position = actual_pos;
    feedback->current_velocity = actual_vel;
    feedback->motor_current = actual_cur;

    goal_handle->publish_feedback(feedback);
}
//Action publishing feedback as long the gripper is moving
template<typename feedbacktype, typename goaltype>
void SchunkGripperNode::runActionGrip(std::shared_ptr<feedbacktype> feedback, std::shared_ptr<rclcpp_action::ServerGoalHandle<goaltype>> goal_handle)
{   
    rclcpp::Time start_time = this->now();

    while(endCondition() && rclcpp::ok() && (!goal_handle->is_canceling()) && connection_error == "OK")
    {   //Publish as fast as in state
        if(*cycletime <= this->now() - start_time)
        {
            feedback->current_position = actual_pos;
            feedback->current_velocity = actual_vel;
            feedback->motor_current = actual_cur;
            //if preempt started-> print once
            if(gripperBitInput(PRE_HOLDING) == true)
            {
                RCLCPP_INFO_ONCE(this->get_logger(),"PRE-HOLDING: %i ms", grp_prehold_time);
                feedback->pre_grip = gripperBitInput(PRE_HOLDING);
            }
                    
            goal_handle->publish_feedback(feedback);
            start_time = this->now();
        }
        //Limits the While loop
        limiting_rate.sleep();  //By low frequencies of state, cycletime.sleep() would block the action to end. Relevant if the gripper was stopped or has an error.
    }
    //Get actual values
    std::unique_lock<std::recursive_mutex> lock(lock_mutex);
    runGets();
    lock.unlock();
    //Error without fast stop
    if(gripperBitInput(GRIPPER_ERROR) && gripperBitOutput(FAST_STOP)) throw static_cast<int32_t>(plc_sync_input[3]);


    feedback->current_position = actual_pos;
    feedback->current_velocity = actual_vel;
    feedback->motor_current = actual_cur;
    feedback->pre_grip = gripperBitInput(PRE_HOLDING);

    goal_handle->publish_feedback(feedback);
}
//Move absolute action callback
void SchunkGripperNode::moveAbsExecute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<MovAbsPos>> goal_handle)
{
    action_active = true;
    action_move = true;
    actual_command = "MOVE TO ABSOLUTE POSITION";

    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<MovAbsPos::Feedback>();
    auto res = std::make_shared<MovAbsPos::Result>();
    
    uint32_t goal_position = mm2mu(goal->abs_position);
    uint32_t goal_velocity = mm2mu(goal->velocity);

try
{  
    std::unique_lock<std::recursive_mutex> lock(lock_mutex);
        //Run goal
        handshake = gripperBitInput(COMMAND_RECEIVED_TOGGLE);
        runPost(commands_str.at(actual_command), goal_position, goal_velocity);
        //Look if gripper received command     
        if(handshake == gripperBitInput(COMMAND_RECEIVED_TOGGLE)) throw int32_t(-1);

    lock.unlock();

    runActionMove(feedback, goal_handle);
    //set result
    res->current_position = actual_pos;
    res->reached_position = gripperBitInput(SUCCESS) && gripperBitInput(POSITION_REACHED);

    setFinalState(res, goal_handle);
}
catch(int32_t i)
{
    exceptionHandling(goal_handle, i, res);
}
catch(const char* response)
{
    connection_error = response;
    RCLCPP_ERROR(this->get_logger(), "Failed Connection! %s", connection_error.c_str());
    finishedCommand();
    //set result
    res->current_position = actual_pos;
    res->reached_position = gripperBitInput(SUCCESS) && gripperBitInput(POSITION_REACHED);
    goal_handle->abort(res);
}

}
//Move relative action callback
void SchunkGripperNode::moveRelExecute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<MovRelPos>> goal_handle)
{   
    action_active = true;
    action_move = true;
    actual_command = "MOVE TO RELATIVE POSITION";
    
    auto feedback = std::make_shared<MovRelPos::Feedback>();
    auto res = std::make_shared<MovRelPos::Result>();
    
    const auto goal = goal_handle->get_goal();

    uint32_t goal_position = mm2mu(goal->distance);
    uint32_t goal_velocity = mm2mu(goal->velocity);
try
{    

        std::unique_lock<std::recursive_mutex> lock(lock_mutex);
            //Send command to gripper
            handshake = gripperBitInput(COMMAND_RECEIVED_TOGGLE);
            runPost(commands_str.at(actual_command), goal_position, goal_velocity);
            //Look if gripper received command
            if(handshake == gripperBitInput(COMMAND_RECEIVED_TOGGLE)) throw int32_t(-1);     

        lock.unlock();

    runActionMove(feedback, goal_handle);    //running as long the gripper moves and no other goal or action is send
    //set result
    res->current_position = actual_pos;
    res->reached_position = gripperBitInput(SUCCESS) && gripperBitInput(POSITION_REACHED);

    setFinalState(res, goal_handle);

}
catch(int32_t i)
{
    exceptionHandling(goal_handle, i, res);
}
catch(const char* response)
{
    connection_error = response;
    RCLCPP_ERROR(this->get_logger(), "Failed Connection! %s", connection_error.c_str());
    finishedCommand();
        //set result
    res->current_position = actual_pos;
    res->reached_position = gripperBitInput(SUCCESS) && gripperBitInput(POSITION_REACHED);
    goal_handle->abort(res);
}
}

//Grip workpiece (EGK) action callback
void SchunkGripperNode::gripExecute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<GripWithVel>> goal_handle)
{
    
    action_active = true;
    actual_command = "GRIP WORKPIECE";

    auto feedback = std::make_shared<GripWithVel::Feedback>();
    auto res = std::make_shared<GripWithVel::Result>();

    feedback->pre_grip = false;

    auto goal = goal_handle->get_goal();
    //Float Goals to uint32_t
    uint32_t goal_effort = static_cast<uint32_t>(goal->effort);
    uint32_t goal_velocity = mm2mu(goal->max_velocity);
    uint32_t goal_command = commands_str.at(actual_command);
    //if last grip direction is not current grip direction
    if(gripperBitOutput(GRIP_DIRECTION) != goal->grp_dir)
    goal_command |= GRIP_DIRECTION;
    
try
{       
        std::unique_lock<std::recursive_mutex> lock(lock_mutex);
            //Send command to gripper
            handshake = gripperBitInput(COMMAND_RECEIVED_TOGGLE);
            runPost(goal_command, 0, goal_velocity, goal_effort);
            //Look if gripper received command
            if(handshake == gripperBitInput(COMMAND_RECEIVED_TOGGLE)) throw int32_t(-1);        

        lock.unlock();

    runActionGrip(feedback, goal_handle);
    //Set result
    res->no_workpiece_detected = gripperBitInput(NO_WORKPIECE_DETECTED);
    res->workpiece_lost = gripperBitInput(WORK_PIECE_LOST);
    res->gripped = gripperBitInput(GRIPPED);
    res->current_position = actual_pos;

    setFinalState(res, goal_handle);
}
catch(int32_t i)
{
    exceptionHandling(goal_handle, i, res);
}
catch(const char* response)
{
    connection_error = response;
    RCLCPP_ERROR(this->get_logger(), "Failed Connection! %s", connection_error.c_str());
    finishedCommand();
    //Set result
    res->no_workpiece_detected = gripperBitInput(NO_WORKPIECE_DETECTED);
    res->workpiece_lost = gripperBitInput(WORK_PIECE_LOST);
    res->gripped = gripperBitInput(GRIPPED);
    res->current_position = actual_pos;
    goal_handle->abort(res);
}
}

//Grip workpiece (EGU) action callback
void SchunkGripperNode::grip_eguExecute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<Grip>> goal_handle)
{
    action_active = true;
    actual_command = "GRIP WORKPIECE";

    auto feedback = std::make_shared<Grip::Feedback>();
    auto res = std::make_shared<Grip::Result>();

    feedback->pre_grip = false;

    auto goal = goal_handle->get_goal();
    
    uint32_t goal_effort = static_cast<uint32_t>(goal->effort);
    uint32_t goal_command = commands_str.at(actual_command);

    if(gripperBitOutput(GRIP_DIRECTION) != goal->grp_dir)
    goal_command |= GRIP_DIRECTION;
    
try
{       
    std::unique_lock<std::recursive_mutex> lock(lock_mutex);
        //Send command to gripper
        handshake = gripperBitInput(COMMAND_RECEIVED_TOGGLE);
        runPost(goal_command, 0, 0, goal_effort);
        //Look if gripper received command
        if(handshake == gripperBitInput(COMMAND_RECEIVED_TOGGLE)) throw uint32_t(-1);

        
    lock.unlock();

    runActionGrip(feedback, goal_handle);
    //Set result
    res->no_workpiece_detected = gripperBitInput(NO_WORKPIECE_DETECTED);
    res->workpiece_lost = gripperBitInput(WORK_PIECE_LOST);
    res->gripped = gripperBitInput(GRIPPED);
    res->current_position = actual_pos;

    setFinalState(res, goal_handle);
}
catch(int32_t i)
{
    exceptionHandling(goal_handle, i, res);
}
catch(const char* response)
{
    connection_error = response;
    RCLCPP_ERROR(this->get_logger(), "Failed Connection! %s", connection_error.c_str());
    finishedCommand();
    //Set result
    res->no_workpiece_detected = gripperBitInput(NO_WORKPIECE_DETECTED);
    res->workpiece_lost = gripperBitInput(WORK_PIECE_LOST);
    res->gripped = gripperBitInput(GRIPPED);
    res->current_position = actual_pos;
    goal_handle->abort(res);
}
}

//Grip workpiece with position (EGK) action callback
void SchunkGripperNode::gripWithPositionExecute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<GripWithPosVel>> goal_handle)
{   
    action_active = true;

    actual_command = "GRIP WORKPIECE WITH POSITION";
    msg.doing_command = actual_command;

    auto feedback = std::make_shared<GripWithPosVel::Feedback>();
    auto res = std::make_shared<GripWithPosVel::Result>();
    
    auto goal = goal_handle->get_goal();

    uint32_t goal_position = mm2mu(goal->abs_position);
    uint32_t goal_velocity = mm2mu(goal->velocity);
    uint32_t goal_effort = static_cast<uint32_t>(goal->effort);
    uint32_t goal_command = GRIP_WORKPIECE_WITH_POSITION;
    //Is last grip direction goal direction?
    if(gripperBitOutput(GRIP_DIRECTION) != goal->grp_dir)
    goal_command |= GRIP_DIRECTION;

try
{       
        std::unique_lock<std::recursive_mutex> lock(lock_mutex);
            //run Command
            handshake = gripperBitInput(COMMAND_RECEIVED_TOGGLE);
            runPost(goal_command, goal_position, goal_velocity, goal_effort);
            //Have Gripper received command
            if(handshake == gripperBitInput(COMMAND_RECEIVED_TOGGLE)) throw int32_t(-1);

            
        lock.unlock();

    runActionGrip(feedback, goal_handle);
    //Set result
    res->no_workpiece_detected = gripperBitInput(NO_WORKPIECE_DETECTED);
    res->workpiece_lost = gripperBitInput(WORK_PIECE_LOST);
    res->gripped = gripperBitInput(GRIPPED);
    res->current_position = actual_pos;
    res->wrong_workpiece_detected = gripperBitInput(WRONG_WORKPIECE_DETECTED);

    setFinalState(res, goal_handle);
}
catch(int32_t i)
{
    exceptionHandling(goal_handle, i, res);
}
catch(const char* response)
{
    connection_error = response;
    RCLCPP_ERROR(this->get_logger(), "Failed Connection! %s", connection_error.c_str());
    finishedCommand();
    //Set result
    res->no_workpiece_detected = gripperBitInput(NO_WORKPIECE_DETECTED);
    res->workpiece_lost = gripperBitInput(WORK_PIECE_LOST);
    res->gripped = gripperBitInput(GRIPPED);
    res->current_position = actual_pos;
    goal_handle->abort(res);
}
}

//Grip with position (EGU) action callback
void SchunkGripperNode::gripWithPosition_eguExecute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<GripWithPos>> goal_handle)
{   
    action_active = true;

    actual_command = "GRIP WORKPIECE WITH POSITION";
    msg.doing_command = actual_command;

    auto feedback = std::make_shared<GripWithPos::Feedback>();
    auto res = std::make_shared<GripWithPos::Result>();

    auto goal = goal_handle->get_goal();
    
    uint32_t goal_position = mm2mu(goal->abs_position);
    uint32_t goal_effort = static_cast<uint32_t>(goal->effort);
    uint32_t goal_command = GRIP_WORKPIECE_WITH_POSITION;
    //Is last grip direction current direction
    if(gripperBitOutput(GRIP_DIRECTION) != goal->grp_dir)
    goal_command |= GRIP_DIRECTION;

try
{       
        std::unique_lock<std::recursive_mutex> lock(lock_mutex);
            //Send command
            handshake = gripperBitInput(COMMAND_RECEIVED_TOGGLE);
            runPost(goal_command, goal_position, 0, goal_effort);
            //Look if gripper received command
            if(handshake == gripperBitInput(COMMAND_RECEIVED_TOGGLE)) throw int32_t(-1);

            
        lock.unlock();

    runActionGrip(feedback, goal_handle);
    //Set result
    res->no_workpiece_detected = gripperBitInput(NO_WORKPIECE_DETECTED);
    res->workpiece_lost = gripperBitInput(WORK_PIECE_LOST);
    res->wrong_workpiece_detected = gripperBitInput(WRONG_WORKPIECE_DETECTED);
    res->gripped = gripperBitInput(GRIPPED);
    res->current_position = actual_pos;

    setFinalState(res, goal_handle);
}
catch(int32_t i)
{
    exceptionHandling(goal_handle, i, res);
}
catch(const char* response)
{
    connection_error = response;
    RCLCPP_ERROR(this->get_logger(), "Failed Connection! %s", connection_error.c_str());
    finishedCommand();
    //Set result
    res->no_workpiece_detected = gripperBitInput(NO_WORKPIECE_DETECTED);
    res->workpiece_lost = gripperBitInput(WORK_PIECE_LOST);
    res->gripped = gripperBitInput(GRIPPED);
    res->current_position = actual_pos;
    goal_handle->abort(res);
}
}

//release workpiece action callback
void SchunkGripperNode::releaseExecute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ReleaseWorkpiece>> goal_handle)
{  
   action_active = true;
   actual_command = "RELEASE WORKPIECE";


   auto feedback = std::make_shared<ReleaseWorkpiece::Feedback>();
   auto res = std::make_shared<ReleaseWorkpiece::Result>();

try
{   
    std::unique_lock<std::recursive_mutex> lock(lock_mutex);
        //Send command
        handshake = gripperBitInput(COMMAND_RECEIVED_TOGGLE);
        runPost(commands_str.at(actual_command));
        //Have gripper received command
        if(handshake == gripperBitInput(COMMAND_RECEIVED_TOGGLE)) throw int32_t(-1);
    
    lock.unlock();
    
    runActionMove(feedback, goal_handle);

    res->current_position = actual_pos;
    res->released = gripperBitInput(SUCCESS);

    setFinalState(res, goal_handle);
}
catch(int32_t i)
{
    exceptionHandling(goal_handle, i, res);
}
catch(const char* response)
{
    connection_error = response;
    RCLCPP_ERROR(this->get_logger(), "Failed Connection! %s", connection_error.c_str());
    finishedCommand();
    res->current_position = actual_pos;
    res->released = gripperBitInput(SUCCESS);
    goal_handle->abort(res);
}
}
//Grip: control_msgs
void SchunkGripperNode::controlExecute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<GripperCommand>> goal_handle)
{   
    action_active = true;

    auto feedback = std::make_shared<GripperCommand::Feedback>();
    auto res = std::make_shared<GripperCommand::Result>();

    feedback->stalled = false;
    feedback->reached_goal = false;

    auto goal = goal_handle->get_goal();

    float effort_percent = (static_cast<float>(goal->command.max_effort) / max_grip_force) * 100.0;
    uint32_t goal_position = mm2mu(static_cast<float>(goal->command.position));
    uint32_t goal_effort = static_cast<uint32_t>(effort_percent);
    uint32_t goal_vel = 0;
    uint32_t goal_command;

    //position is zero and effort is not zero
    if(goal_position == 0 && goal_effort != 0) actual_command = "GRIP WORKPIECE";
    //effort is zero
    else if(goal_effort == 0) 
    {
        actual_command = "MOVE TO ABSOLUTE POSITION";
        action_move = true;
        goal_vel = mm2mu(max_vel/2);
    }
    //both are not zero
    else actual_command = "GRIP WORKPIECE WITH POSITION";
    //goal command
    goal_command = commands_str.at(actual_command);
    //last grip not direction zero and will the gripper grip -> Grip always in on direction
    if(gripperBitOutput(GRIP_DIRECTION) != 0 && goal_command != MOVE_TO_ABSOLUTE_POSITION)
    goal_command |= GRIP_DIRECTION;

try
{   
    std::unique_lock<std::recursive_mutex> lock(lock_mutex);
        //run command
        handshake = gripperBitInput(COMMAND_RECEIVED_TOGGLE);
        runPost(goal_command, goal_position, goal_vel, goal_effort);
        //Gripper received command
        if(handshake == gripperBitInput(COMMAND_RECEIVED_TOGGLE)) throw int32_t(-1);
        
    lock.unlock();

    rclcpp::Time start_time = this->now();

    while(endCondition() && rclcpp::ok() && (!goal_handle->is_canceling()))
    {
        if(*cycletime <= this->now() - start_time)
        {
            feedback->position = actual_pos;
            goal_handle->publish_feedback(feedback);
            start_time = this->now();
        }
        limiting_rate.sleep();
    }

    lock.lock();
    runGets();
    lock.unlock();

    feedback->position = actual_pos;
    //Set result
    res->position = actual_pos;
    if(actual_command == "MOVE TO ABSOLUTE POSITION")
    {
        res->reached_goal = gripperBitInput(SUCCESS) && gripperBitInput(POSITION_REACHED);
        res->stalled = gripperBitInput(SUCCESS);
    }
    else
    {
        res->reached_goal = gripperBitInput(SUCCESS) && gripperBitInput(GRIPPED);
        res->stalled = gripperBitInput(SUCCESS);
    }

    setFinalState(res, goal_handle);
}
catch(int32_t i)
{
    exceptionHandling(goal_handle, i, res);
}
catch(const char* response)
{
    connection_error = response;
    RCLCPP_ERROR(this->get_logger(), "Failed Connection! %s", connection_error.c_str());
    finishedCommand();
    //Set result
    res->position = actual_pos;
    if(actual_command == "MOVE TO ABSOLUTE POSITION")
    {
        res->reached_goal = gripperBitInput(SUCCESS) && gripperBitInput(POSITION_REACHED);
        res->stalled = gripperBitInput(SUCCESS);
    }
    else
    {
        res->reached_goal = gripperBitInput(SUCCESS) && gripperBitInput(GRIPPED);
        res->stalled = gripperBitInput(SUCCESS);
    }
    goal_handle->abort(res);
}

action_move = false;

}

//Updates final successful state and zero position offset
void SchunkGripperNode::finishedCommand()
{   
        const std::lock_guard<std::recursive_mutex> lock(lock_mutex);
        
        action_active = false;
        param_exe = false;
        action_move = false;      

        if(connection_error != "OK") 
        {   
            actual_command = "NO COMMAND";
            return;
        }
        //update Parameter
        std::vector<rclcpp::Parameter> params = {rclcpp::Parameter("Control_Parameter.release_workpiece", false),
                                                 rclcpp::Parameter("Control_Parameter.grip", false),
                                                 rclcpp::Parameter("Control_Parameter.move_gripper", actualPosInterval())};
                                                 
        abs_pos_param = actualPosInterval();

        this->set_parameters(params);
        //Was Command successfully processed?
        if(gripperBitInput(SUCCESS) && gripperBitInput(POSITION_REACHED)) 
        RCLCPP_INFO(this->get_logger(),"%s SUCCEEDED", actual_command.c_str());
        
        if(gripperBitInput(GRIPPED))  
        RCLCPP_WARN(this->get_logger(),"Gripped Workpiece!");

        actual_command = "NO COMMAND";
}
//publish with x hz gripper data
void SchunkGripperNode::publishState()
{
    rclcpp::Time now = this->now();
    *cycletime = now - last_time;
    last_time = now;

    std::unique_lock<std::recursive_mutex> lock(lock_mutex, std::defer_lock);

    if(lock.try_lock())
    {   try
        {
            runGets();
            
            if(!(connection_error == "OK")) 
            {
                reconnect();
            }

        }
        catch(const char *res)
        {  
            connection_error = res;
            rclcpp::Duration sleep_time(1,0);
            if(sleep_time > this->now() - last_time)
            {
                rclcpp::Duration sleep = sleep_time - (this->now() - last_time);
                std::this_thread::sleep_for(sleep.to_chrono<std::chrono::milliseconds>());
            }
        }
        lock.unlock();
    }
    
    if( ((param_exe == true && !endCondition()) && connection_error == "OK") && !gripperBitInput(PRE_HOLDING))                   //If param_exe was active and module is in an end state
    finishedCommand();                                                                      //If zero_offset was changed

    if(msg.workpiece_lost != gripperBitInput(WORK_PIECE_LOST) && !msg.workpiece_lost)
    RCLCPP_WARN(this->get_logger(),"Workpiece lost!");

    if(msg.no_workpiece_detected != gripperBitInput(NO_WORKPIECE_DETECTED) && !msg.no_workpiece_detected)
    RCLCPP_WARN(this->get_logger(),"No workpiece detected!");

    if(msg.wrong_workpiece != gripperBitInput(WRONG_WORKPIECE_DETECTED) && !msg.wrong_workpiece) 
    RCLCPP_WARN(this->get_logger(),"Wrong workpiece detected!");
    //Update msg
    updateStateMsg();
    //Publish state
    statePublisher->publish(msg);
}
//Reconnect
void SchunkGripperNode::reconnect()
{
 
                if(ip_changed_with_all_param == false)  //If Connection was found after the IP-changed
                {
                    startGripper();
                    getModel();
                    this->set_parameter(rclcpp::Parameter("model", model));

                    //get Versions
                    getWithInstance<uint16_t>(SW_VERSION_NUM_INST, &sw_version);
                    getWithInstance<char>(COMM_VERSION_TXT_INST, NULL, 12);
                    updateSaveData(1, 12, char_vector);

                    ip_changed_with_all_param = true;

                    if(start_connection == false)
                    {
                        getActualParameters();
                        advertiseConnectionRelevant();
                        start_connection = true;
                    }

                }

                if(start_connection == false) return;

                getActualParameters();                 //Get and set the possible new Parameter

                this->set_parameter(rclcpp::Parameter("Gripper_Parameter.use_brk", grp_pos_lock));
                this->set_parameter(rclcpp::Parameter("Gripper_Parameter.grp_pos_margin", grp_pos_margin));
                this->set_parameter(rclcpp::Parameter("Gripper_Parameter.grp_prepos_delta", grp_prepos_delta));
                this->set_parameter(rclcpp::Parameter("Gripper_Parameter.zero_pos_ofs", zero_pos_ofs));
                this->set_parameter(rclcpp::Parameter("Gripper_Parameter.grp_prehold_time", grp_prehold_time));
                this->set_parameter(rclcpp::Parameter("Gripper_Parameter.wp_release_delta", wp_release_delta));
                this->set_parameter(rclcpp::Parameter("Gripper_Parameter.wp_lost_distance", wp_lost_dst));
                this->set_parameter(rclcpp::Parameter("Control_Parameter.move_gripper", actualPosInterval()));
                abs_pos_param = actualPosInterval();    

                connection_error = "OK";  
}
//JointStatePublisher
void SchunkGripperNode::publishJointState()
{
    sensor_msgs::msg::JointState joint_msg;
    joint_msg.header.stamp = this->now();
    joint_msg.header.frame_id="base_link_left";
    joint_msg.name.push_back("egu_prismatic_joint_translational_left");
    joint_msg.position.push_back(actual_pos);
    joint_msg.velocity.push_back(actual_vel);
    jointStatePublisher->publish(joint_msg);
}
//Acknowledge service callback
void SchunkGripperNode::acknowledge_srv(const std::shared_ptr<Acknowledge::Request>, std::shared_ptr<Acknowledge::Response> res)
{
    const std::lock_guard<std::recursive_mutex> lock(lock_mutex);
    actual_command = "ACKNOWLEDGE";
    msg.doing_command = actual_command;
    try
    {

        //Acknowledge
        acknowledge();

        if(check()) //TODO handshake
        {
            res->acknowledged = true;
            RCLCPP_WARN(this->get_logger(),"Acknowledged");
        }
        else 
        {
            res->acknowledged = false;
            RCLCPP_WARN(this->get_logger(),"NOT ACKNOWLEDGED!");
        }
    }
    catch(const char* server_err)
    {
        connection_error = server_err ;
        RCLCPP_ERROR(this->get_logger(), "Failed Connection! %s", connection_error.c_str());
        res->acknowledged = false;
        RCLCPP_WARN(this->get_logger(), "NOT ACKNOWLEDGED!");
    }
    last_command = 0;
    gripper_updater->force_update();
    finishedCommand();
}
//Brake test
void SchunkGripperNode::brake_test_srv(const std::shared_ptr<BrakeTest::Request>, std::shared_ptr<BrakeTest::Response> res)
{
    std::lock_guard<std::recursive_mutex> lock(lock_mutex);
    try
    {
        //send stop
        handshake = gripperBitInput(COMMAND_RECEIVED_TOGGLE);
        runPost(BRAKE_TEST);
        //if command received, get values
        if(handshake != gripperBitInput(COMMAND_RECEIVED_TOGGLE)) 
        while (rclcpp::ok() && check() && !gripperBitInput(SUCCESS))   
        runGets();

        if((gripperBitInput(SUCCESS) == true) && (handshake != gripperBitInput(COMMAND_RECEIVED_TOGGLE)))
        {
            res->brake_test_successful = true;
            res->error_code = error_str;
            RCLCPP_INFO(this->get_logger(),"Brake test successful!");
        }
        else
        {
            runGets();
            gripper_updater->force_update();
            res->error_code = error_str;
            res->brake_test_successful = false;
            RCLCPP_INFO(this->get_logger(),"Brake test not successful!");
            
        }
    }
    catch(const char* server_response)
    {
        connection_error = server_response;
        res->error_code = server_response;
        res->brake_test_successful = false;
        RCLCPP_ERROR(this->get_logger(), "Failed Connection! %s", connection_error.c_str());
    } 

}
//Ip change service, if the ip should change during runtime of the program
void SchunkGripperNode::change_ip_srv(const std::shared_ptr<ChangeIp::Request> req, std::shared_ptr<ChangeIp::Response> res)
{   
    std::lock_guard<std::recursive_mutex> lock(lock_mutex);
    res->ip_changed = changeIp(req->new_ip);

    if(res->ip_changed == false)
    {
        connection_error = "No gripper found. Using old IP";
        gripper_updater->force_update();
        return;
    }

    if(res->ip_changed == true) 
    {
        this->set_parameter(rclcpp::Parameter("IP", req->new_ip));
    }

    if(start_connection == false && ip_changed_with_all_param == true)
    {   try
        {
            advertiseConnectionRelevant();
            start_connection = true;
        }
        catch(...)
        {
            RCLCPP_ERROR(this->get_logger(), "Not connected.");
        }
    }

    if(ip_changed_with_all_param == true)
    {
        this->set_parameter(rclcpp::Parameter("model", model));
       
        if(connection_error == "OK")
        {
            reconnect();
        }
    }

}
//Stop service callback
void SchunkGripperNode::stop_srv(const std::shared_ptr<Stop::Request>, std::shared_ptr<Stop::Response> res)
{
    const std::lock_guard<std::recursive_mutex> lock(lock_mutex);
    try
    {
        //send stop
        handshake = gripperBitInput(COMMAND_RECEIVED_TOGGLE);
        runPost(STOP);
        //if command received, get values
        if(handshake != gripperBitInput(COMMAND_RECEIVED_TOGGLE)) 
        while (rclcpp::ok() && check() && !gripperBitInput(SUCCESS))   
        runGets();            
    }
    catch(const char* res)
    {
        connection_error = res;
        RCLCPP_ERROR(this->get_logger(), "Failed Connection! %s", connection_error.c_str());
    }

    if((gripperBitInput(SUCCESS) == 1) && (handshake != gripperBitInput(COMMAND_RECEIVED_TOGGLE)))
    {
        res->stopped = 1;
        RCLCPP_WARN(this->get_logger(),"Stopped!");
    }
    else
    {
        res->stopped = 0;
        last_command = 0;
    }

}
//Fast stop service callback
void SchunkGripperNode::fast_stop_srv(const std::shared_ptr<FastStop::Request>, std::shared_ptr<FastStop::Response> res)
{
    const std::lock_guard<std::recursive_mutex> lock(lock_mutex);
    try
    {
        //send fast stop
        
        handshake = gripperBitInput(COMMAND_RECEIVED_TOGGLE);
        runPost(FAST_STOP);
        //if command received, get values
        if(handshake != gripperBitInput(COMMAND_RECEIVED_TOGGLE)) 
        while (rclcpp::ok() && check())
        runGets(); 

        if(gripperBitInput(GRIPPER_ERROR))
        {
            res->fast_stopped = 1;
            RCLCPP_WARN(this->get_logger(),"FAST STOPPED");
        }
        else
        {
            res->fast_stopped = 0;
            RCLCPP_WARN(this->get_logger(),"FAST STOP NOT SUCCEEDED");
        }
    }
    catch(const char* server_err)
    {
        connection_error = server_err;
        RCLCPP_ERROR(this->get_logger(), "Failed Connection! %s", connection_error.c_str());
        res->fast_stopped = 0;
        RCLCPP_WARN(this->get_logger(), "FAST STOP NOT SUCCEEDED");
    }
    gripper_updater->force_update();
}
//Softreset service callback 
void SchunkGripperNode::softreset_srv(const std::shared_ptr<Softreset::Request>, std::shared_ptr<Softreset::Response> res)
{  
    std::unique_lock<std::recursive_mutex> lock(lock_mutex);
    try
    {
        RCLCPP_INFO(this->get_logger(),"SOFTRESET");
        handshake = gripperBitInput(COMMAND_RECEIVED_TOGGLE);
        runPost(SOFT_RESET);
    }
    catch(const char* res){}

    bool connection_once_lost = false;
    
    if(handshake != gripperBitInput(COMMAND_RECEIVED_TOGGLE))
    {
        for(int i = 0; rclcpp::ok() && i <= 1000 && check(); i++)
        {
            try
            {
                runGets();               //if connection is back, don't catch
                if(connection_once_lost == true) 
                {
                    connection_error = "OK";
                    break;
                }
            }
            catch(const char* res)
            {
                connection_error = res;
                std::chrono::milliseconds sleep_time(10);
                std::this_thread::sleep_for(sleep_time);
                connection_once_lost = true;
            }
        }
    }
    else RCLCPP_WARN(this->get_logger(), "No Command received");

    if(connection_once_lost == true && rclcpp::ok() && connection_error == "OK")
    {
        RCLCPP_INFO(this->get_logger(), "Softreset succeeded");
        res->reset_success = true;
    }
    else res->reset_success = false;

    lock.unlock();
    gripper_updater->force_update();

}
//Prepare for shutdown service callback
void SchunkGripperNode::prepare_for_shutdown_srv(const std::shared_ptr<PrepareForShutdown::Request>, std::shared_ptr<PrepareForShutdown::Response> res)
{   
    const std::lock_guard<std::recursive_mutex> lock(lock_mutex);
    try
    {
        //send prepare for shutdown
        handshake = gripperBitInput(COMMAND_RECEIVED_TOGGLE);
        runPost(PREPARE_FOR_SHUTDOWN);
        //if command received, get values
        if(handshake != gripperBitInput(COMMAND_RECEIVED_TOGGLE))
        while (rclcpp::ok() && check() && !gripperBitInput(READY_FOR_SHUTDOWN))
        runGets();
    }
    catch(const char* res)
    {
        connection_error = res;
        RCLCPP_ERROR(this->get_logger(), "Failed Connection! %s", connection_error.c_str());
    }
    
    if((gripperBitInput(READY_FOR_SHUTDOWN) == true) && (handshake != gripperBitInput(COMMAND_RECEIVED_TOGGLE)) )
    {
        RCLCPP_WARN(this->get_logger(),"READY FOR SHUTDOWN");
        res->prepared_for_shutdown = true;
    }
    else
    {
        RCLCPP_INFO(this->get_logger(),"COMMAND FAILED");
        res->prepared_for_shutdown = false;
    }

    gripper_updater->force_update();
}
//Release for manual movement service callback
void SchunkGripperNode::releaseForManualMov_srv(const std::shared_ptr<ReleaseForManMov::Request>, std::shared_ptr<ReleaseForManMov::Response> res)
{   
    const std::lock_guard<std::recursive_mutex> lock(lock_mutex);
    try
    {
        //If no Errors
        if(splitted_Diagnosis[2] == 0) 
        {   
            //Send fast stop
            handshake = gripperBitInput(COMMAND_RECEIVED_TOGGLE);
            runPost(FAST_STOP); 
            //if command received, get values 
            if(handshake != gripperBitInput(COMMAND_RECEIVED_TOGGLE)) 
            while (rclcpp::ok() && check())
            runGets();

            gripper_updater->force_update();
            RCLCPP_WARN(this->get_logger(),"An error was provoked so that you can take the workpiece: %s\n", getErrorString(splitted_Diagnosis[2]).c_str());
        }
        //send Emergency release
        handshake = gripperBitInput(COMMAND_RECEIVED_TOGGLE);
        runPost(EMERGENCY_RELEASE);
        //if command received, get values
        if(handshake != gripperBitInput(COMMAND_RECEIVED_TOGGLE)) 
        while(rclcpp::ok() && !gripperBitInput(EMERGENCY_RELEASED))
        runGets();
    }
    
    catch(const char* res)
    {
        connection_error = res;
        RCLCPP_ERROR(this->get_logger(), "Failed Connection! %s", connection_error.c_str());
    }

    if(gripperBitInput(EMERGENCY_RELEASED) && (handshake != gripperBitInput(COMMAND_RECEIVED_TOGGLE)))
    {
        RCLCPP_WARN(this->get_logger(),"YOU CAN TAKE THE WORKPIECE!\n");
        RCLCPP_WARN(this->get_logger(),"If you want to end this mode, perform a fast stop and acknowledge!");
        res->released_for_manual_movement = true;
    }
    else
    {
        RCLCPP_WARN(this->get_logger(),"COMMAND FAILED");
        res->released_for_manual_movement = false;
    }
    gripper_updater->force_update();
}
//Get infos of the gripper service
void SchunkGripperNode::info_srv(const std::shared_ptr<GripperInfo::Request>, std::shared_ptr<GripperInfo::Response>)
{  
    const std::lock_guard<std::recursive_mutex> lock(lock_mutex);
    try
    {
        if(wrong_version) RCLCPP_WARN_STREAM(this->get_logger(),"Using not suitable software-version. Some informations may be misleading");

        std::vector<std::string> char_strings;
        uint32_t data;
        uint16_t data2;
        float data3;

        RCLCPP_INFO_STREAM(this->get_logger(),"\n\n\nGRIPPER TYPE: " << model.c_str()
                        << "\nIP: " << ip  << std::endl);
        
        getEnums(FIELDBUS_TYPE_INST,fieldbus_type);
        getParameter(MAC_ADDR_INST, 6, schunk_gripper::msg::DataTypesParameter::CHAR);
        std::array<int16_t,6> mac;
        
        for(size_t i = 0; i < 6; i++){mac[i] = static_cast<int16_t>(static_cast<unsigned char>(char_vector.at(i)));}
            
        RCLCPP_INFO_STREAM(this->get_logger(),
                               "\nFieldbustype:  " << json_data["string"] 
                               << "\nMac-address: " << std::hex << mac[0] << ":" << mac[1] << ":" << mac[2] << ":" << mac[3] << ":"
                               << mac[4] << ":" << mac[5] << std::endl);

                
        //Get the char-Parameter and save them as strings in char_strings
        getParameter(SERIAL_NO_TXT_INST, 16, schunk_gripper::msg::DataTypesParameter::CHAR);
        char_strings.push_back(char_vector.data());
        getParameter(ORDER_NO_TXT_INST, 16, schunk_gripper::msg::DataTypesParameter::CHAR);
        char_strings.push_back(char_vector.data());
        getParameter(SW_BUILD_DATE_INST, 12, schunk_gripper::msg::DataTypesParameter::CHAR);
        char_strings.push_back(char_vector.data());
        getParameter(SW_BUILD_TIME_INST, 9, schunk_gripper::msg::DataTypesParameter::CHAR);
        char_strings.push_back(char_vector.data());
        getParameter(SW_VERSION_TXT_INST, 22, schunk_gripper::msg::DataTypesParameter::CHAR);
        char_strings.push_back(char_vector.data());
        
        getWithInstance(SERIAL_NO_NUM_INST, &data);
        getWithInstance<uint16_t>(SW_VERSION_NUM_INST, &data2);

        RCLCPP_INFO_STREAM(this->get_logger(),"\nSerial number text:  " << char_strings[0]
                                            <<"\nOrder number text:  " <<  char_strings[1]
                                            <<"\nDevice serial number encoded: " << data
                                            <<"\nMain software build date:  " << char_strings[2]
                                            <<"\nMain software build time:  " << char_strings[3]
                                            <<"\nMain software version short:  " << data2
                                            <<"\nMain software version:  " << char_strings[4]
                                            <<"\nCommunication software version:  " << comm_version << std::endl);

        getWithInstance<uint32_t>(UPTIME_INST, &data);
        RCLCPP_INFO_STREAM(this->get_logger(),"\nSystem uptime: " << data << " s" << std::endl);
        
        getWithInstance<float>(DEAD_LOAD_KG_INST, &data3);
        RCLCPP_INFO_STREAM(this->get_logger(),"\nNet mass of the Gripper: " << data3 << " kg" << std::endl);
        
        getParameter(TOOL_CENT_POINT_INST, 6, schunk_gripper::msg::DataTypesParameter::FLOAT);
        RCLCPP_INFO_STREAM(this->get_logger(),"\nTool center point 6D-Frame: \n" 
        << float_vector[0] << " mm  " << float_vector[1] << " mm  " << float_vector[2] << " mm\n" 
        << float_vector[3] << "  " << float_vector[4] << "  " << float_vector[5] << std::endl);
        
        getParameter(CENT_OF_MASS_INST, 6, schunk_gripper::msg::DataTypesParameter::FLOAT);
        RCLCPP_INFO_STREAM(this->get_logger(), "\nCenter of Mass 6D-frame: \n" 
        << float_vector[0] << " mm  " << float_vector[1] << " mm  " << float_vector[2] << " mm\n"
        << float_vector[3] << " kg*m^2  " << float_vector[4] << " kg*m^2  "<< float_vector[5] << " kg*m^2"<< std::endl);

        RCLCPP_INFO_STREAM(this->get_logger(),"\nMin. absolute position: " << min_pos << " mm\n" 
        << "Max. absolute position: " << max_pos << " mm\n"
        << "Zero_pos_offset: " << zero_pos_ofs << " mm" << std::endl);

        RCLCPP_INFO_STREAM(this->get_logger(),"\nMin. velocity: " << min_vel << " mm/s\n"
        << "Max. velocity: " << max_vel << " mm/s" <<std::endl);

        RCLCPP_INFO_STREAM(this->get_logger(),"\nMax. grip velocity: "<< max_grp_vel << " mm/s\n" 
        << "Min. grip force: "   << min_grip_force << " N\n" 
        << "Max. grip force: "   << max_grip_force << " N" << std::endl);

        if(model.find("EGU") != std::string::npos)
        {
        RCLCPP_INFO_STREAM(this->get_logger(),"\nMax allowed grip force StrongGrip: " << max_allow_force  << " N  " << std::endl);
        }

        getWithInstance<float>(USED_CUR_LIMIT_INST, &data3);
        RCLCPP_INFO_STREAM(this->get_logger(),"\nUsed current limit: " << data3 << " A" << std::endl);
        getWithInstance<float>(MAX_PHYS_STROKE_INST, &data3);
        RCLCPP_INFO_STREAM(this->get_logger(),"Max. physical stroke: " << data3 << " mm" << std::endl);
        getWithOffset(MIN_ERR_MOT_VOLT_OFFSET, 6, 6);
        RCLCPP_INFO_STREAM(this->get_logger(),"\nMin. error motor voltage: " << float_vector[0] << " V\n"
                    <<    "Max. error motor voltage: " << float_vector[1] << " V\n"
                    <<    "Min. error logic voltage: " << float_vector[2] << " V\n"
                    <<    "Max. error logic voltage: " << float_vector[3] << " V\n"
                    <<    "Min. error logic temperature: " << float_vector[4] << " C\n"
                    <<    "Max. error logic temperature: " << float_vector[5] << " C" << std::endl);
        
        getWithInstance<float>(MEAS_LGC_TEMP_INST, &data3);
        RCLCPP_INFO_STREAM(this->get_logger(), "Measured logic temperature: " << data3 << " C" << std::endl);

        getWithOffset(MEAS_LGC_VOLT_OFFSET, 8, 8);
        RCLCPP_INFO_STREAM(this->get_logger(),
                          "\nMeasured logic voltage: " << float_vector[0] <<  " V\n"
                    <<    "Measured motor voltage: " << float_vector[1] <<     " V\n"
                    <<    "Min. warning motor voltage: " << float_vector[2] << " V\n"
                    <<    "Max. warning motor voltage: " << float_vector[3] << " V\n"
                    <<    "Min. warning logic voltage: " << float_vector[4] << " V\n"
                    <<    "Max. warning logic voltage: " << float_vector[5] << " V\n"
                    <<    "Min. warning logic temperature: " << float_vector[6] << " C\n"
                    <<    "Max. warning logic temperature: " << float_vector[7] << " C" << std::endl);
        
    }
    catch(const char* res)
    {
    connection_error = res;
    RCLCPP_ERROR(this->get_logger(), "Failed Connection! %s", connection_error.c_str());
    }
    catch(const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    }
}

void SchunkGripperNode::parameter_get_srv(const std::shared_ptr<ParameterGet::Request> req, std::shared_ptr<ParameterGet::Response> res)
{
    using Datatype = schunk_gripper::msg::DataTypesParameter;
    try
    {
    std::lock_guard<std::recursive_mutex> lock(lock_mutex);
    getParameter(req->instance, req->data_types_param.elements, req->data_types_param.datatype);
    }
    catch(const char *res)
    {   
        not_double_word = true;
        RCLCPP_ERROR_STREAM(this->get_logger(), res);
        return;
    }
    catch(std::exception &e)
    {   
        not_double_word = true;
        RCLCPP_ERROR_STREAM(this->get_logger(), e.what());
        return;
    }
       switch(req->data_types_param.datatype)
        {
        case Datatype::NONE: RCLCPP_WARN(this->get_logger(), "Please enter an Datatype");
                break;
        case Datatype::BOOL: res->bool_value = bool_vector;
                break;
        
        case Datatype::UINTEGER8: res->uint8_value = uint8_vector;
                break;
        
        case Datatype::UINTEGER16: res->uint16_value = uint16_vector;
                break;

        case Datatype::UINTEGER32: res->uint32_value = uint32_vector;
                break;
        
        case Datatype::INTEGER32: res->int32_value = int32_vector;
                break;

        case Datatype::FLOAT: res->float_value = float_vector;
                break;
        
        case Datatype::CHAR: res->char_value = char_vector.data();
                break;
        }
}

void SchunkGripperNode::parameter_set_srv(const std::shared_ptr<ParameterSet::Request> req, std::shared_ptr<ParameterSet::Response> res)
{
        using Datatype = schunk_gripper::msg::DataTypesParameter;
    try
    {
      char inst[7];
      if(req->instance.size() < 7) 
      {
       std::strcpy(inst, req->instance.c_str());
      }
      else throw "String to long";

        switch(req->data_types_param.datatype)
        {
        case Datatype::NONE: RCLCPP_WARN(this->get_logger(), "Please enter an Datatype");
                break;

        case Datatype::BOOL: 
                changeVectorParameter(inst, req->bool_value);
                getParameter(inst, req->bool_value.size(), req->data_types_param.datatype);
                res->actual_bool_value = bool_vector;
                break;
        
        case Datatype::UINTEGER8: 
                changeVectorParameter(inst, req->uint8_value);
                getParameter(inst, req->uint8_value.size(), req->data_types_param.datatype);
                res->actual_uint8_value = uint8_vector;
                break;
        
        case Datatype::UINTEGER16: 
                changeVectorParameter(inst, req->uint16_value);
                getParameter(inst, req->uint16_value.size(), req->data_types_param.datatype);
                res->actual_uint16_value = uint16_vector;
                break;

        case Datatype::UINTEGER32: 
                changeVectorParameter(inst, req->uint32_value);
                getParameter(inst, req->uint32_value.size(), req->data_types_param.datatype);
                res->actual_uint32_value = uint32_vector;
                break;
        
        case Datatype::INTEGER32: 
                changeVectorParameter(inst, req->int32_value);
                getParameter(inst, req->int32_value.size(), req->data_types_param.datatype);
                res->actual_int32_value = int32_vector;
                break;

        case Datatype::FLOAT: 
                changeVectorParameter(inst, req->float_value);
                getParameter(inst, req->float_value.size()-1, req->data_types_param.datatype);
                res->actual_float_value = float_vector;
                break;
        
      //  case Datatype::CHAR: changeParameter(inst, req->char_value, &res->actual_char_value);
      //          break;
        }
    }
    catch(const char *res)
    {   
        not_double_word = true;
        RCLCPP_ERROR_STREAM(this->get_logger(), res);
        return;
    }
    catch(std::exception &e)
    {   
        not_double_word = true;
        RCLCPP_ERROR_STREAM(this->get_logger(), e.what());
        return;
    }
    catch(nlohmann::json::exception &e)
    {   
        not_double_word = true;
        RCLCPP_ERROR_STREAM(this->get_logger(), e.what());
        return;
    }

}

void SchunkGripperNode::callback_gripper_parameter(const rclcpp::Parameter &p)
{
    try
    {
    if(p.get_name() == "diagnostic_updater.period") 
    {
        gripper_updater->setPeriod(p.as_double());
        return;
    }
    if(p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
    {
        const std::lock_guard<std::recursive_mutex> lock(lock_mutex);
        changeOneParameter(parameter_map.at(p.get_name()), static_cast<float>(p.as_double()));
        RCLCPP_INFO_STREAM(this->get_logger(), p.get_name() << " changed to: " << p.as_double());
        if(p.get_name() == "Gripper_Parameter.zero_pos_ofs")
        {   
            getWithInstance<float>(MAX_POS_INST, &max_pos);
            getWithInstance<float>(MIN_POS_INST,&min_pos);
            runGets();

            parameter_event_handler->remove_parameter_callback(cb_handle[9]);
            
            this->undeclare_parameter("Control_Parameter.move_gripper");
            this->declare_parameter("Control_Parameter.move_gripper", actualPosInterval(), parameter_descriptor("Moving the gripper with parameter in mm", FloatingPointRange(min_pos,max_pos)), true);
           
            abs_pos_param = actualPosInterval();
            cb_handle[9] =  parameter_event_handler->add_parameter_callback("Control_Parameter.move_gripper", callback_move_param);
        }
    }
    else if(p.get_name() == "Gripper_Parameter.grp_prehold_time")
    {
        const std::lock_guard<std::recursive_mutex> lock(lock_mutex);
        changeOneParameter(GRP_PREHOLD_TIME_INST, static_cast<uint16_t>(p.as_int()), &grp_prehold_time);
        RCLCPP_INFO_STREAM(this->get_logger(), p.get_name() << " changed to: " << grp_prehold_time);
    }
    else if(p.get_name() == "Gripper_Parameter.use_brk" && (gripperBitOutput(USE_GPE)!= p.as_bool()))
    {
        const std::lock_guard<std::recursive_mutex> lock(lock_mutex);
        handshake = gripperBitInput(COMMAND_RECEIVED_TOGGLE);
        runPost(USE_GPE);
        grp_pos_lock = gripperBitOutput(USE_GPE);
        RCLCPP_INFO(this->get_logger(),"The brake is %s.", grp_pos_lock ? "on" : "off");
    }
}
catch(const char* res)
{
    RCLCPP_ERROR(this->get_logger() , "Could't set the Parameter(s). \nConnection failed: %s", res);
    failed_param.push_back(p);
}
}

void SchunkGripperNode::callback_move_parameter(const rclcpp::Parameter &p)
{
try
{ 
    if(p.get_name() == "Control_Parameter.move_gripper" && param_exe == false && p.as_double() != abs_pos_param) 
    {
        const std::lock_guard<std::recursive_mutex> lock(lock_mutex);
            actual_command = "MOVE TO ABSOLUTE POSITION";
            handshake = gripperBitInput(COMMAND_RECEIVED_TOGGLE);
            runPost(MOVE_TO_ABSOLUTE_POSITION, static_cast<uint32_t>(p.as_double() * 1000) , static_cast<uint32_t>(this->get_parameter("Control_Parameter.move_gripper_velocity").as_double()*1000));
            param_exe = true;
    }
    else if(p.get_name() == "Control_Parameter.grip" && p.as_bool() == true && param_exe == false) 
    {
        const std::lock_guard<std::recursive_mutex> lock(lock_mutex);
        uint32_t command = GRIP_WORK_PIECE;

        if(gripperBitOutput(GRIP_DIRECTION) != this->get_parameter("Control_Parameter.grip_direction").as_bool())
            command |= GRIP_DIRECTION;
            
        actual_command = "GRIP WORKPIECE";
        msg.doing_command = actual_command;
        handshake = gripperBitInput(COMMAND_RECEIVED_TOGGLE);
        runPost(command, 0, 0, static_cast<uint32_t>(this->get_parameter("Control_Parameter.grip_force").as_double()));
        param_exe = true;

    }
    else if(p.get_name() == "Control_Parameter.release_workpiece" && p.as_bool() == true && param_exe == false) 
    {
        if (gripperBitInput(GRIPPED))
        {
            const std::lock_guard<std::recursive_mutex> lock(lock_mutex);
            actual_command = "RELEASE WORKPIECE";
            msg.doing_command = actual_command;
            handshake = gripperBitInput(COMMAND_RECEIVED_TOGGLE);
            runPost(commands_str.at(actual_command));
            param_exe = true;
        }
        else 
        {
            RCLCPP_INFO(this->get_logger(),"No Workpiece Gripped");
            this->set_parameter(rclcpp::Parameter("Control_Parameter.release_workpiece", false));
        }
    }
}
catch(const char* res)
{
    RCLCPP_ERROR(this->get_logger() , "Could't set the Parameter(s). \nConnection failed: %s", res);
}
}
//Diagnostics 
void SchunkGripperNode::gripperDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& status)
{   
    splitted_Diagnosis = splitDiagnosis();

    try
    {
        if(connection_error != "OK")    //If connection lost
        {
        status.summaryf(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Connection failed: %s", connection_error.c_str());
        return;
        }
        if(gripperBitInput(EMERGENCY_RELEASED)) //If release for manual Movement active
        status.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "If you want to end this mode, perform a fast stop and acknowledge!");
        
        else if(gripperBitInput(READY_FOR_SHUTDOWN)) //If ready for shutdown
        status.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Ready for shutdown!");
    
    
    //If an error or waning occurred
    else if(plc_sync_input[3] != 0)
    {  //if error
        if(splitted_Diagnosis[2])
        {    
            if(splitted_Diagnosis != old_diagnosis) 
            {
                const std::lock_guard<std::recursive_mutex> lock(lock_mutex);
                error_str = getErrorString(splitted_Diagnosis[2]);
                RCLCPP_ERROR(this->get_logger(),"%i\n%s",splitted_Diagnosis[2] , error_str.c_str());
            }
            status.summaryf(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Error: %i\n%s" ,splitted_Diagnosis[2] ,error_str.c_str());
        }
        //if not feasible
        if(splitted_Diagnosis[0])
        {   
            if(splitted_Diagnosis != old_diagnosis) 
            {
                const std::lock_guard<std::recursive_mutex> lock(lock_mutex);
                warn_str = getErrorString(splitted_Diagnosis[1]);
                RCLCPP_WARN(this->get_logger(),"%s",(warn_str+ ": " + std::to_string(splitted_Diagnosis[0])).c_str());
            }
            if(splitted_Diagnosis[2] == 0)
            status.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, warn_str +" "+ std::to_string(splitted_Diagnosis[0]));
        }
        //if warning
        else if(splitted_Diagnosis[1])
        {   
            if(splitted_Diagnosis != old_diagnosis)
            {
                const std::lock_guard<std::recursive_mutex> lock(lock_mutex);
                warn_str = getErrorString(splitted_Diagnosis[1]);
                RCLCPP_WARN(this->get_logger(),"%i\n%s", splitted_Diagnosis[1], warn_str.c_str());
            }
            if(splitted_Diagnosis[2] == 0)
            status.summaryf(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Warning: %i\n%s",splitted_Diagnosis[1], warn_str.c_str());
        }
    }

 else //No errors
    {  
        if(splitted_Diagnosis != old_diagnosis) 
        {
            const std::lock_guard<std::recursive_mutex> lock(lock_mutex);
            error_str = getErrorString(0);
            RCLCPP_INFO(this->get_logger(),"%s",error_str.c_str());
        }
        status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, error_str);
    }
}

catch(const char* res)  //Lost Connection catch
{
    connection_error = res;
    RCLCPP_ERROR(this->get_logger(), "Failed Connection! %s", connection_error.c_str());
}

    old_diagnosis = splitted_Diagnosis; 

    status.add("Ready for operation", gripperBitInput(READY_FOR_OPERATION));
    status.add("Command successfully processed", gripperBitInput(SUCCESS));
    status.add("position reached", gripperBitInput(POSITION_REACHED));
    status.add("Command received toggle", gripperBitInput(COMMAND_RECEIVED_TOGGLE));
    
    status.add("Error", gripperBitInput(GRIPPER_ERROR));
    status.add("Warning", gripperBitInput(WARNING));
    status.add("Not feasible", gripperBitInput(NOT_FEASIBLE));

    status.add("Pre-grip started",gripperBitInput(PRE_HOLDING));
    status.add("Workpiece gripped", gripperBitInput(GRIPPED));
    status.add("No workpiece detected", gripperBitInput(NO_WORKPIECE_DETECTED));
    status.add("Wrong workpiece gripped", gripperBitInput(WRONG_WORKPIECE_DETECTED));
    status.add("Workpiece lost", gripperBitInput(WORK_PIECE_LOST));

    status.add("Control authority", gripperBitInput(CONTROL_AUTHORITY));
    status.add("Software limit reached", gripperBitInput(SOFTWARE_LIMIT));
    status.add("Released for manual movement", gripperBitInput(EMERGENCY_RELEASED));
    status.add("Grip force and position maintenance", gripperBitInput(BRAKE_ACTIVE));
    status.add("Ready for shutdown", gripperBitInput(READY_FOR_SHUTDOWN));

}

SchunkGripperNode::~SchunkGripperNode()
{ 
  std::lock_guard<std::recursive_mutex> lock(lock_mutex);

  if(action_active == true || param_exe == true)        //If gripper is moving, fast stop
  {
    for(size_t i = 0; i <= 2; i++)                      //Try 3 times fast stop
    {
        try
        {
        //send fast stop
        handshake = gripperBitInput(COMMAND_RECEIVED_TOGGLE);
        runPost(FAST_STOP);
        if(gripperBitInput(COMMAND_RECEIVED_TOGGLE) != handshake) break;
        }
        catch(...)
        { 
            RCLCPP_ERROR(this->get_logger(),"Could't perform fast stop.");
        }
    }
  }

  RCLCPP_INFO(this->get_logger(),"Node shutdown!");

}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(SchunkGripperNode)
