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
    Gripper(options.parameter_overrides().at(0).as_string()), 
    nd(std::make_shared<rclcpp::Node>("schunk_gripper_driver", options)),
    cycletime(1/options.parameter_overrides().at(2).as_double()),
    limiting_rate(1000)
    //param_server(mutex)
    //ActionServer
{   

    state_frq = options.parameter_overrides().at(1).as_double();

    if(start_connection == false) return;

    using std::placeholders::_1;
    using std::placeholders::_2;

    messages_group = nd->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    services_group = nd->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    actions_group  = nd->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rest  = nd->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::PublisherOptions option_messages;
    option_messages.callback_group = messages_group;
    rclcpp::SubscriptionOptions option;
    option.callback_group = rest;

    parameters_client = std::make_shared<rclcpp::AsyncParametersClient>(nd);
    failed_param = std::make_shared<rcl_interfaces::msg::ParameterEvent>();

    while (!parameters_client->wait_for_service(std::chrono::seconds(1))) 
    {
        if (!rclcpp::ok()) 
        {
        RCLCPP_ERROR(nd->get_logger(), "Interrupted while waiting for the service. Exiting.");
        }
    }
    
    parameter_event = parameters_client->on_parameter_event(std::bind(&SchunkGripperNode::parameterEventCallback,this,_1), 10, option);

    declareParameter();

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
      RCLCPP_INFO_STREAM(nd->get_logger(), "Failed Connection to gripper. "<< res << std::endl);
      start_connection = false;
   }
   last_time = nd->now();
    //Advertise state
    statePublisher = nd->create_publisher<schunk_gripper::msg::State>("state", 10);
    publish_state_timer=nd->create_wall_timer(std::chrono::duration<float>(1/state_frq), std::bind(&SchunkGripperNode::publishState, this), messages_group);
    
    //Initialize diagnostics
    gripper_updater = std::make_shared<diagnostic_updater::Updater>(nd);
    gripper_updater->setHardwareID("Module");
    gripper_updater->add(model, std::bind(&SchunkGripperNode::gripperDiagnostics,this,_1));
    //Look if joint_state_frq is less than state_frq
    float j_state_frq = options.parameter_overrides().at(2).as_double();
    if(j_state_frq > state_frq)
    {
        j_state_frq = state_frq;
        RCLCPP_WARN(nd->get_logger(),"joint_state topic will publish with %f!", j_state_frq);
    }
    //Advertise joint_states
    jointStatePublisher = nd->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    publish_joint_timer = nd->create_wall_timer(std::chrono::duration<float>(1/j_state_frq), std::bind(&SchunkGripperNode::publishJointState, this));

    acknowledge_service = nd->create_service<Acknowledge>("acknowledge", std::bind(&SchunkGripperNode::acknowledge_srv,this,_1,_2), rmw_qos_profile_services_default, services_group);
    stop_service = nd->create_service<Stop>("stop", std::bind(&SchunkGripperNode::stop_srv,this,_1,_2), rmw_qos_profile_services_default, services_group);
    softreset_service = nd->create_service<Softreset>("softreset", std::bind(&SchunkGripperNode::softreset_srv,this,_1,_2), rmw_qos_profile_services_default, services_group);
    releaseForManualMov_service = nd->create_service<ReleaseForManMov>("release_for_manual_movement", std::bind(&SchunkGripperNode::releaseForManualMov_srv,this,_1,_2), rmw_qos_profile_services_default, services_group);
    prepare_for_shutdown_service = nd->create_service<PrepareForShutdown>("prepare_for_shutdown", std::bind(&SchunkGripperNode::prepare_for_shutdown_srv,this,_1,_2), rmw_qos_profile_services_default, services_group);
    fast_stop_service = nd->create_service<FastStop>("fast_stop", std::bind(&SchunkGripperNode::fast_stop_srv,this,_1,_2), rmw_qos_profile_services_default, services_group);
    info_service = nd->create_service<GripperInfo>("gripper_info", std::bind(&SchunkGripperNode::info_srv,this,_1,_2), rmw_qos_profile_services_default, rest);
    change_ip_service = nd->create_service<ChangeIp>("change_ip",  std::bind(&SchunkGripperNode::change_ip_srv,this,_1,_2), rmw_qos_profile_services_default, rest);
    //Start Grip actions depending opn the model    

    move_abs_server = rclcpp_action::create_server<MovAbsPos>(nd, "move_absolute", std::bind(&SchunkGripperNode::handle_goal<MovAbsPos::Goal>, this, _1, _2),
                                                                                   std::bind(&SchunkGripperNode::handle_cancel<MovAbsPos>, this, _1),
                                                                                   std::bind(&SchunkGripperNode::handle_accepted_abs, this, _1),rcl_action_server_get_default_options(), actions_group);
   
    move_rel_server = rclcpp_action::create_server<MovRelPos>(nd, "move_relative", std::bind(&SchunkGripperNode::handle_goal<MovRelPos::Goal>, this, _1, _2),
                                                                                   std::bind(&SchunkGripperNode::handle_cancel<MovRelPos>, this, _1),
                                                                                   std::bind(&SchunkGripperNode::handle_accepted_rel, this, _1),rcl_action_server_get_default_options(), actions_group);  
    
    release_wp_server = rclcpp_action::create_server<ReleaseWorkpiece>(nd, "release_workpiece",     std::bind(&SchunkGripperNode::handle_goal<ReleaseWorkpiece::Goal>, this, _1, _2),
                                                                                                    std::bind(&SchunkGripperNode::handle_cancel<ReleaseWorkpiece>, this, _1),
                                                                                                    std::bind(&SchunkGripperNode::handle_accepted_release, this, _1),rcl_action_server_get_default_options(), actions_group);
    
    control_server = rclcpp_action::create_server<GripperCommand>(nd, "gripper_control",            std::bind(&SchunkGripperNode::handle_goal<GripperCommand::Goal>, this, _1, _2),
                                                                                                    std::bind(&SchunkGripperNode::handle_cancel<GripperCommand>, this, _1),
                                                                                                    std::bind(&SchunkGripperNode::handle_accepted_control, this, _1),rcl_action_server_get_default_options(), actions_group);
    
    if(model.find("EGK") != std::string::npos)
    {
        grip_server = rclcpp_action::create_server<GripWithVel>(nd, "grip",     std::bind(&SchunkGripperNode::handle_goal<GripWithVel::Goal>, this, _1, _2),
                                                                                std::bind(&SchunkGripperNode::handle_cancel<GripWithVel>, this, _1),
                                                                                std::bind(&SchunkGripperNode::handle_accepted_grip_egk, this, _1),rcl_action_server_get_default_options(), actions_group);
        
        grip_w_pos_server = rclcpp_action::create_server<GripWithPosVel>(nd, "grip_with_pos",       std::bind(&SchunkGripperNode::handle_goal<GripWithPosVel::Goal>, this, _1, _2),
                                                                                                    std::bind(&SchunkGripperNode::handle_cancel<GripWithPosVel>, this, _1),
                                                                                                    std::bind(&SchunkGripperNode::handle_accepted_gripPos_egk, this, _1),rcl_action_server_get_default_options(), actions_group);
    
    }
    else if(model.find("EGU") != std::string::npos)
    {
        grip_egu_server = rclcpp_action::create_server<Grip>(nd, "grip",    std::bind(&SchunkGripperNode::handle_goal<Grip::Goal>, this, _1, _2),
                                                                            std::bind(&SchunkGripperNode::handle_cancel<Grip>, this, _1),
                                                                            std::bind(&SchunkGripperNode::handle_accepted_grip_egu, this, _1),rcl_action_server_get_default_options(), actions_group);
    
        grip_w_pos_egu_server = rclcpp_action::create_server<GripWithPos>(nd, "grip_with_pos_egu",    std::bind(&SchunkGripperNode::handle_goal<GripWithPos::Goal>, this, _1, _2),
                                                                                                      std::bind(&SchunkGripperNode::handle_cancel<GripWithPos>, this, _1),
                                                                                                      std::bind(&SchunkGripperNode::handle_accepted_gripPos_egu, this, _1),rcl_action_server_get_default_options(), actions_group);
    }
    else
    {
        RCLCPP_ERROR(nd->get_logger(),"GRIPPER NOT FOUND!");
        return;
    }

    handshake = gripperBitInput(COMMAND_RECEIVED_TOGGLE);
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
void SchunkGripperNode::declareParameter()
{   
    nd->declare_parameter("Control_Parameter.move_gripper", actualPosInterval(), parameter_descriptor("Moving the gripper with parameter in mm", FloatingPointRange(min_pos, max_pos)));
    abs_pos_param = actualPosInterval();
    // Gripper Parameter
    nd->declare_parameter("Gripper_Parameter.use_brk", false, parameter_descriptor("Use brake"));
    nd->declare_parameter("Gripper_Parameter.grp_pos_margin", 2.0, parameter_descriptor("Grip position margin in mm", FloatingPointRange(1.0, 10.0, 0.01)));
    nd->declare_parameter("Gripper_Parameter.grp_prepos_delta", 5.0, parameter_descriptor("Grip position delta in mm", FloatingPointRange(1.0, 50.0, 0.01)));
    nd->declare_parameter("Gripper_Parameter.zero_pos_ofs", 0.0, parameter_descriptor("Zero position offset in mm", FloatingPointRange(-10000.0, 10000.0, 0.01)));
    nd->declare_parameter("Gripper_Parameter.grp_prehold_time", 0, parameter_descriptor("Grip prehold time in ms", IntegerRange(0, 60000, 1)));
    nd->declare_parameter("Gripper_Parameter.wp_release_delta", 5.0, parameter_descriptor("Workpiece release delta in mm", FloatingPointRange(1.0, 50.0, 0.01)));
    nd->declare_parameter("Gripper_Parameter.wp_lost_distance", 1.0, parameter_descriptor("Max. distance after workpiece lost in mm", FloatingPointRange(0.1, 50.0, 0.01)));
    // Control Parameter
    nd->declare_parameter("Control_Parameter.move_gripper_velocity", static_cast<double>(min_vel), parameter_descriptor("Changing the velocity for move_Gripper in mm/s", FloatingPointRange(min_vel, max_vel)));
    nd->declare_parameter("Control_Parameter.grip_direction", false, parameter_descriptor("Grip direction for parameter gripping"));
    nd->declare_parameter("Control_Parameter.grip", false, parameter_descriptor("Grip with parameter"));
    nd->declare_parameter("Control_Parameter.grip_force", 50.0, parameter_descriptor("Grip force in %", FloatingPointRange(50.0, 100.0, 0.01)));
    nd->declare_parameter("Control_Parameter.release_workpiece", false, parameter_descriptor("Release Workpiece"));
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
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    descriptor.description = description;
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
    descriptor.description = description;
    return descriptor;
}

double SchunkGripperNode::actualPosInterval()
{
    if(actual_pos < min_pos) return static_cast<double>(min_pos);
    else if(actual_pos > max_pos)return static_cast<double>(max_pos);
    else return actual_pos;
}
template<typename goaltype>
rclcpp_action::GoalResponse SchunkGripperNode::handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const goaltype> goal)
{
    (void)uuid;
    (void)goal;

    if(gripperBitInput(GRIPPER_ERROR)) RCLCPP_ERROR(nd->get_logger(), "Action will not be performed as long an error is active");
    else if(param_exe == false && action_active == false) return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    else  
    {
        const std::lock_guard<std::recursive_mutex> lock(lock_mutex);
        try
        {
            //send fast stop
            runPost(FAST_STOP);
            //if command received, get values
            if(handshake != gripperBitInput(COMMAND_RECEIVED_TOGGLE)) 
            while (rclcpp::ok() && check())
            runGets(); 

            handshake = gripperBitInput(COMMAND_RECEIVED_TOGGLE);
            gripper_updater->force_update();
        }
        catch(const char* server_err)
        {
            connection_error = server_err;
            RCLCPP_ERROR(nd->get_logger(), "Failed Connection! %s", connection_error.c_str());
        }
    }
    
    return rclcpp_action::GoalResponse::REJECT;
}
//Deal with gripper, if the Command where successful done. Else throw an Exception
template<typename restype, typename goaltype>
void SchunkGripperNode::setFinalState(std::shared_ptr<restype> res, const std::shared_ptr<rclcpp_action::ServerGoalHandle<goaltype>> goal_handle)
{   
    //Get current handshake
    //handshake = gripperBitInput(COMMAND_RECEIVED_TOGGLE);
    //publish end feedback
    //Gripper was stopped
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
        RCLCPP_WARN(nd->get_logger(),"Fast stopped. Reason could be another goal or fast stop service.");
        goal_handle->abort(res);
        return;
    }
    //Cancel or new goal arrived
    else if(goal_handle->is_canceling())  
    {
        RCLCPP_ERROR(nd->get_logger(),"Cancel Request received. Fast stop.");
        
        std::lock_guard<std::recursive_mutex> lock(lock_mutex);
            //Fast stop
            runPost(FAST_STOP);
            if(handshake != gripperBitInput(COMMAND_RECEIVED_TOGGLE)) 
            while (rclcpp::ok() && check())
            runGets();
            
            handshake = gripperBitInput(COMMAND_RECEIVED_TOGGLE);

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
        handshake = gripperBitInput(COMMAND_RECEIVED_TOGGLE);

        if(plc_sync_input[3] != 0)  ;// gripper_updater.force_update();
        //If gripper did not receive command
        else if(i == -1)
        {                 
            RCLCPP_ERROR(nd->get_logger(),"GRIPPER DID NOT RECEIVE THE COMMAND!");          //ToDo Feedback to robot
        }
        else RCLCPP_WARN(nd->get_logger(),"Action aborted!");

        finishedCommand();
        goal_handle->abort(res);
        return;

}   
//Action publishing its feedback as long the Gripper is moving
template<typename feedbacktype, typename goaltype>
void SchunkGripperNode::runActionMove(std::shared_ptr<feedbacktype> feedback, std::shared_ptr<rclcpp_action::ServerGoalHandle<goaltype>> goal_handle)
{   
    rclcpp::Time start_time = nd->now();

    while(endCondition() && rclcpp::ok() && (!goal_handle->is_canceling()))
    {   //Publish as fast as in state
        if(cycletime <= nd->now() - start_time)
        {
            feedback->current_position = actual_pos;
            feedback->current_velocity = actual_vel;
            feedback->motor_current = actual_cur;

            goal_handle->publish_feedback(feedback);
            start_time = nd->now();
        }
        //Limits the While loop
        limiting_rate.sleep();
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
    rclcpp::Time start_time = nd->now();

    while(endCondition() && rclcpp::ok() && (!goal_handle->is_canceling()))
    {   //Publish as fast as in state
        if(cycletime <= nd->now() - start_time)
        {
            feedback->current_position = actual_pos;
            feedback->current_velocity = actual_vel;
            feedback->motor_current = actual_cur;
            //if preempt started-> print once
            if(gripperBitInput(PRE_HOLDING) == true)
            {
                RCLCPP_INFO_ONCE(nd->get_logger(),"PRE-HOLDING: %i ms", grp_prehold_time);
                feedback->pre_grip = gripperBitInput(PRE_HOLDING);
            }
                    
            goal_handle->publish_feedback(feedback);
            start_time = nd->now();
        }
        //Limits the While loop
        limiting_rate.sleep();
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
        runPost(commands_str.at(actual_command), goal_position, goal_velocity);
        //Look if gripper received command     
        if(handshake == gripperBitInput(COMMAND_RECEIVED_TOGGLE)) throw int32_t(-1);
        else handshake = gripperBitInput(COMMAND_RECEIVED_TOGGLE);
    
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
    RCLCPP_ERROR(nd->get_logger(), "Failed Connection! %s", connection_error.c_str());
    finishedCommand();
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
            runPost(commands_str.at(actual_command), goal_position, goal_velocity);
            //Look if gripper received command
            if(handshake == gripperBitInput(COMMAND_RECEIVED_TOGGLE)) throw int32_t(-1);
            else handshake = gripperBitInput(COMMAND_RECEIVED_TOGGLE);
        
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
    RCLCPP_ERROR(nd->get_logger(), "Failed Connection! %s", connection_error.c_str());
    finishedCommand();
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
            runPost(goal_command, 0, goal_velocity, goal_effort);
            //Look if gripper received command
            if(handshake == gripperBitInput(COMMAND_RECEIVED_TOGGLE)) throw int32_t(-1);
            handshake = gripperBitInput(COMMAND_RECEIVED_TOGGLE);

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
    RCLCPP_ERROR(nd->get_logger(), "Failed Connection! %s", connection_error.c_str());
    finishedCommand();
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
        runPost(goal_command, 0, 0, goal_effort);
        //Look if gripper received command
        if(handshake == gripperBitInput(COMMAND_RECEIVED_TOGGLE)) throw uint32_t(-1);
        handshake = gripperBitInput(COMMAND_RECEIVED_TOGGLE);
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
    RCLCPP_ERROR(nd->get_logger(), "Failed Connection! %s", connection_error.c_str());
    finishedCommand();
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
            runPost(goal_command, goal_position, goal_velocity, goal_effort);
            //Have Gripper received command
            if(handshake == gripperBitInput(COMMAND_RECEIVED_TOGGLE)) throw int32_t(-1);
            else handshake = gripperBitInput(COMMAND_RECEIVED_TOGGLE);
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
    RCLCPP_ERROR(nd->get_logger(), "Failed Connection! %s", connection_error.c_str());
    finishedCommand();
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
            runPost(goal_command, goal_position, 0, goal_effort);
            //Look if gripper received command
            if(handshake == gripperBitInput(COMMAND_RECEIVED_TOGGLE)) throw int32_t(-1);
            else handshake = gripperBitInput(COMMAND_RECEIVED_TOGGLE);
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
    RCLCPP_ERROR(nd->get_logger(), "Failed Connection! %s", connection_error.c_str());
    finishedCommand();
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
        runPost(commands_str.at(actual_command));
        //Have gripper received command
        if(handshake == gripperBitInput(COMMAND_RECEIVED_TOGGLE)) throw int32_t(-1);
        else handshake = gripperBitInput(COMMAND_RECEIVED_TOGGLE);
    
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
    RCLCPP_ERROR(nd->get_logger(), "Failed Connection! %s", connection_error.c_str());
    finishedCommand();
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
        runPost(goal_command, goal_position, goal_vel, goal_effort);
        //Gripper received command
        if(handshake == gripperBitInput(COMMAND_RECEIVED_TOGGLE)) throw int32_t(-1);
        else handshake = gripperBitInput(COMMAND_RECEIVED_TOGGLE);
    lock.unlock();

    rclcpp::Time start_time = nd->now();

    while(endCondition() && rclcpp::ok() && (!goal_handle->is_canceling()))
    {
        if(cycletime <= nd->now() - start_time)
        {
            feedback->position = actual_pos;
            goal_handle->publish_feedback(feedback);
            start_time = nd->now();
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
    RCLCPP_ERROR(nd->get_logger(), "Failed Connection! %s", connection_error.c_str());
    finishedCommand();
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

        //update Parameter
        std::vector<rclcpp::Parameter> params = {rclcpp::Parameter("Control_Parameter.release_workpiece", false),
                                                 rclcpp::Parameter("Control_Parameter.grip", false),
                                                 rclcpp::Parameter("Control_Parameter.move_gripper", actualPosInterval())};
        abs_pos_param = actualPosInterval();
        parameters_client->set_parameters(params);

        //Was Command successfully processed?
        if(gripperBitInput(SUCCESS) && gripperBitInput(POSITION_REACHED)) 
        RCLCPP_INFO(nd->get_logger(),"%s SUCCEEDED", actual_command.c_str());
        
        if(gripperBitInput(GRIPPED) && zero_changed == false)  
        RCLCPP_WARN(nd->get_logger(),"Gripped Workpiece!");
        //Flags
        actual_command = "NO COMMAND";
        msg.doing_command = actual_command;

}
//publish with x hz gripper data
void SchunkGripperNode::publishState()
{
    rclcpp::Time now = nd->now();
    cycletime = now - last_time;
    last_time = now;

    std::unique_lock<std::recursive_mutex> lock(lock_mutex, std::defer_lock);

    if(lock.try_lock())
    {   try
        {
            runGets();
            
            if(!(connection_error == "OK")) 
            {
                connection_error = "OK";

                if(!failed_param->changed_parameters.empty())
                {
                    for(auto element : failed_param->changed_parameters)
                    {
                        std::vector<rclcpp::Parameter> param = {rclcpp::Parameter(element.name,element.value)};
                        parameters_client->set_parameters(param);
                    }
                    failed_param->changed_parameters.clear();
                    failed_param->changed_parameters.shrink_to_fit();
                }
                
            }
        }
        catch(const char *res)
        {
            connection_error = res;
            rclcpp::Duration sleep_time(1,0);
            rclcpp::Duration sleep = sleep_time - (nd->now() - last_time);
            std::this_thread::sleep_for(sleep.to_chrono<std::chrono::milliseconds>());
        }
        lock.unlock();
    }
    
    if( ((param_exe == true && !endCondition()) || (zero_changed == true)) && !gripperBitInput(PRE_HOLDING))                   //If param_exe was active and module is in an end state
    finishedCommand();                                                                      //If zero_offset was changed

    if(msg.workpiece_lost != gripperBitInput(WORK_PIECE_LOST) && !msg.workpiece_lost)
    RCLCPP_WARN(nd->get_logger(),"Workpiece lost!");

    if(msg.no_workpiece_detected != gripperBitInput(NO_WORKPIECE_DETECTED) && !msg.no_workpiece_detected)
    RCLCPP_WARN(nd->get_logger(),"No workpiece detected!");

    if(msg.wrong_workpiece != gripperBitInput(WRONG_WORKPIECE_DETECTED) && !msg.wrong_workpiece) 
    RCLCPP_WARN(nd->get_logger(),"Wrong workpiece detected!");
    //Update msg
    updateStateMsg();
    //Publish state
    statePublisher->publish(msg);
}
//JointStatePublisher
void SchunkGripperNode::publishJointState()
{
    sensor_msgs::msg::JointState joint_msg;
    joint_msg.header.stamp = nd->now();
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
    //Acknowledge
    acknowledge();
    try
    {
        if(check()) 
        {
            res->acknowledged = true;
            RCLCPP_WARN(nd->get_logger(),"Acknowledged");
        }
        else 
        {
            res->acknowledged = false;
            RCLCPP_WARN(nd->get_logger(),"NOT ACKNOWLEDGED!");
        }
    }
    catch(const char* server_err)
    {
        connection_error = server_err ;
        RCLCPP_ERROR(nd->get_logger(), "Failed Connection! %s", connection_error.c_str());
        res->acknowledged = false;
        RCLCPP_WARN(nd->get_logger(), "NOT ACKNOWLEDGED!");
    }
    handshake = gripperBitInput(COMMAND_RECEIVED_TOGGLE);
    last_command = 0;
    gripper_updater->force_update();
    finishedCommand();
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
        nd->undeclare_parameter("IP");
        nd->undeclare_parameter("model");
        rcl_interfaces::msg::ParameterDescriptor param_des;
        param_des.description = "Ip address of the gripper";
        nd->declare_parameter("IP", req->new_ip, param_des);
        param_des.description = "Model of the gripper";
        nd->declare_parameter("model", model, param_des);
    }

    if(ip_changed_with_all_param == true)
    {
    nd->set_parameter(rclcpp::Parameter("Gripper_Parameter.use_brk", grp_pos_lock));
    nd->set_parameter(rclcpp::Parameter("Gripper_Parameter.grp_pos_margin", grp_pos_margin));
    nd->set_parameter(rclcpp::Parameter("Gripper_Parameter.grp_prepos_delta", grp_prepos_delta));
    nd->set_parameter(rclcpp::Parameter("Gripper_Parameter.zero_pos_ofs", zero_pos_ofs));
    nd->set_parameter(rclcpp::Parameter("Gripper_Parameter.grp_prehold_time", grp_prehold_time));
    nd->set_parameter(rclcpp::Parameter("Gripper_Parameter.wp_release_delta", wp_release_delta));
    nd->set_parameter(rclcpp::Parameter("Gripper_Parameter.wp_lost_distance", wp_lost_dst));
    nd->set_parameter(rclcpp::Parameter("Control_Parameter.move_gripper", actualPosInterval()));
    abs_pos_param = actualPosInterval();
    }
    

}
//Stop service callback
void SchunkGripperNode::stop_srv(const std::shared_ptr<Stop::Request>, std::shared_ptr<Stop::Response> res)
{
    const std::lock_guard<std::recursive_mutex> lock(lock_mutex);
    try
    {
        //send stop
        runPost(STOP);
        //if command received, get values
        if(handshake != gripperBitInput(COMMAND_RECEIVED_TOGGLE)) 
        while (rclcpp::ok() && check() && !gripperBitInput(SUCCESS))   
        runGets();            
    }
    catch(const char* res)
    {
        connection_error = res;
        RCLCPP_ERROR(nd->get_logger(), "Failed Connection! %s", connection_error.c_str());
    }

    if((gripperBitInput(SUCCESS) == 1) && (handshake != gripperBitInput(COMMAND_RECEIVED_TOGGLE)))
    {
        res->stopped = 1;
        RCLCPP_WARN(nd->get_logger(),"Stopped!");
    }
    else
    {
        res->stopped = 0;
        last_command = 0;
    }

    handshake = gripperBitInput(COMMAND_RECEIVED_TOGGLE);

}
//Fast stop service callback
void SchunkGripperNode::fast_stop_srv(const std::shared_ptr<FastStop::Request>, std::shared_ptr<FastStop::Response> res)
{
    const std::lock_guard<std::recursive_mutex> lock(lock_mutex);
    try
    {
        //send fast stop
        runPost(FAST_STOP);
        //if command received, get values
        if(handshake != gripperBitInput(COMMAND_RECEIVED_TOGGLE)) 
        while (rclcpp::ok() && check())
        runGets(); 

        if(gripperBitInput(GRIPPER_ERROR))
        {
            res->fast_stopped = 1;
            RCLCPP_WARN(nd->get_logger(),"FAST STOPPED");
        }
        else
        {
            res->fast_stopped = 0;
            RCLCPP_WARN(nd->get_logger(),"FAST STOP NOT SUCCEEDED");
        }
    }
    catch(const char* server_err)
    {
        connection_error = server_err;
        RCLCPP_ERROR(nd->get_logger(), "Failed Connection! %s", connection_error.c_str());
        res->fast_stopped = 0;
        RCLCPP_WARN(nd->get_logger(), "FAST STOP NOT SUCCEEDED");
    }
    handshake = gripperBitInput(COMMAND_RECEIVED_TOGGLE);
    gripper_updater->force_update();
}
//Softreset service callback 
void SchunkGripperNode::softreset_srv(const std::shared_ptr<Softreset::Request>, std::shared_ptr<Softreset::Response> res)
{  
    std::unique_lock<std::recursive_mutex> lock(lock_mutex);
    try
    {
        RCLCPP_INFO(nd->get_logger(),"SOFT RESET");
        runPost(SOFT_RESET);
    }
    catch(const char* res){}
    bool connection_once_lost = false;
    
    if(handshake != gripperBitInput(COMMAND_RECEIVED_TOGGLE))
    for(int i = 0; rclcpp::ok() && i <= 1000; i++)
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

    if(connection_once_lost == true && rclcpp::ok() && connection_error == "OK")
    {
        RCLCPP_INFO(nd->get_logger(), "Softreset succeeded");
        res->reset_success = true;
    }
    else res->reset_success = false;

    handshake = gripperBitInput(COMMAND_RECEIVED_TOGGLE);
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
        runPost(PREPARE_FOR_SHUTDOWN);
        //if command received, get values
        if(handshake != gripperBitInput(COMMAND_RECEIVED_TOGGLE))
        while (rclcpp::ok() && check() && !gripperBitInput(READY_FOR_SHUTDOWN))
        runGets();
    }
    catch(const char* res)
    {
        connection_error = res;
        RCLCPP_ERROR(nd->get_logger(), "Failed Connection! %s", connection_error.c_str());
    }
    
    if((gripperBitInput(READY_FOR_SHUTDOWN) == true) && (handshake != gripperBitInput(COMMAND_RECEIVED_TOGGLE)) )
    {
        RCLCPP_WARN(nd->get_logger(),"READY FOR SHUTDOWN");
        res->prepared_for_shutdown = true;
    }
    else
    {
        RCLCPP_INFO(nd->get_logger(),"COMMAND FAILED");
        res->prepared_for_shutdown = false;
    }
    
    handshake = gripperBitInput(COMMAND_RECEIVED_TOGGLE);
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
            runPost(FAST_STOP); 
            //if command received, get values 
            if(handshake != gripperBitInput(COMMAND_RECEIVED_TOGGLE)) 
            while (rclcpp::ok() && check())
            runGets();

            gripper_updater->force_update();
            RCLCPP_WARN(nd->get_logger(),"An error was provoked so that you can take the workpiece: %s\n", getErrorString(splitted_Diagnosis[2]).c_str());
            handshake = gripperBitInput(COMMAND_RECEIVED_TOGGLE);
        }
        //send Emergency release
        runPost(EMERGENCY_RELEASE);
        //if command received, get values
        if(handshake != gripperBitInput(COMMAND_RECEIVED_TOGGLE)) 
        while(rclcpp::ok() && !gripperBitInput(EMERGENCY_RELEASED))
        runGets();
    }
    catch(const char* res)
    {
        connection_error = res;
        RCLCPP_ERROR(nd->get_logger(), "Failed Connection! %s", connection_error.c_str());
    }

    if(gripperBitInput(EMERGENCY_RELEASED) && (handshake != gripperBitInput(COMMAND_RECEIVED_TOGGLE)))
    {
        RCLCPP_WARN(nd->get_logger(),"YOU CAN TAKE THE WORKPIECE!\n");
        RCLCPP_WARN(nd->get_logger(),"If you want to end this mode, perform a fast stop and acknowledge!");
        res->released_for_manual_movement = true;
    }
    else
    {
        RCLCPP_WARN(nd->get_logger(),"COMMAND FAILED");
        res->released_for_manual_movement = false;
    }
    
    handshake = gripperBitInput(COMMAND_RECEIVED_TOGGLE);
    gripper_updater->force_update();
}
//Get infos of the gripper service
void SchunkGripperNode::info_srv(const std::shared_ptr<GripperInfo::Request>, std::shared_ptr<GripperInfo::Response>)
{  
    const std::lock_guard<std::recursive_mutex> lock(lock_mutex);
    try
    {
        RCLCPP_INFO_STREAM(nd->get_logger(),"\n\n\nGRIPPER TYPE: " << model.c_str()
                        << "\nIP: " << ip  << std::endl);
        
        receiveWithOffset("40", 1, 1);
        RCLCPP_INFO_STREAM(nd->get_logger(),"\nNet mass of the Gripper: " << save_data[0] << " kg" << std::endl);
        
        receiveWithOffset("41", 1,6);
        RCLCPP_INFO_STREAM(nd->get_logger(),"\nTool center point 6D-Frame: \n" 
        << save_data[0] << " mm  " << save_data[1] << " mm  " << save_data[2] << " mm\n" 
        << save_data[3] << "  " << save_data[4] << "  " << save_data[5] << std::endl);
        
        receiveWithOffset("42", 1,6);
        RCLCPP_INFO_STREAM(nd->get_logger(), "\nCenter of Mass 6D-frame: \n" 
        << save_data[0] << " mm  " << save_data[1] << " mm  " << save_data[2] << " mm\n"
        << save_data[3] << " kg*m^2  " << save_data[4] << " kg*m^2  "<< save_data[5] << " kg*m^2"<< std::endl);

        RCLCPP_INFO_STREAM(nd->get_logger(),"\nMin. absolute position: " << min_pos << " mm\n" 
        << "Max. absolute position: " << max_pos << " mm\n"
        << "Zero_pos_offset: " << zero_pos_ofs << " mm" << std::endl);

        RCLCPP_INFO_STREAM(nd->get_logger(),"\nMin. velocity: " << min_vel << " mm/s\n"
        << "Max. velocity: " << max_vel << " mm/s" <<std::endl);
        receiveWithOffset("85", 1, 1);
        RCLCPP_INFO_STREAM(nd->get_logger(),"\nMax. grip velocity: "<< save_data[0] << " mm/s\n" 
        << "Min. grip force: "   << min_grip_force << " N\n" 
        << "Max. grip force: "   << max_grip_force << " N" << std::endl);

        if(model.find("_M_") != std::string::npos && model.find("EGU") != std::string::npos)
        {
        receiveWithOffset("93", 1, 1);
        RCLCPP_INFO_STREAM(nd->get_logger(),"\nMax allowed grip force StrongGrip: " << save_data[0]  << " N  " << std::endl);
        }

        receiveWithOffset("11", 1, 1);
        RCLCPP_INFO_STREAM(nd->get_logger(),"\nUsed current limit: " << save_data[0] << " A" << std::endl);
        receiveWithOffset("70", 1, 1);
        RCLCPP_INFO_STREAM(nd->get_logger(),"Max. physical stroke: " << save_data[0] << " mm" << std::endl);
        receiveWithOffset("96", 6, 6);
        RCLCPP_INFO_STREAM(nd->get_logger(),"\nMin. error motor voltage: " << save_data[0] << " V\n"
                    <<    "Max. error motor voltage: " << save_data[1] << " V\n"
                    <<    "Min. error logic voltage: " << save_data[2] << " V\n"
                    <<    "Max. error logic voltage: " << save_data[3] << " V\n"
                    <<    "Min. error logic temperature: " << save_data[4] << " C\n"
                    <<    "Max. error logic temperature: " << save_data[5] << " C" << std::endl);
        
        receiveWithOffset("112", 6, 6);
        RCLCPP_INFO_STREAM(nd->get_logger(),"\nMin. warning motor voltage: " << save_data[0] << " V\n"
                    <<    "Max. warning motor voltage: " << save_data[1] << " V\n"
                    <<    "Min. warning logic voltage: " << save_data[2] << " V\n"
                    <<    "Max. warning logic voltage: " << save_data[3] << " V\n"
                    <<    "Min. warning logic temperature: " << save_data[4] << " C\n"
                    <<    "Max. warning logic temperature: " << save_data[5] << " C" << std::endl);
        uint32_t uptime;
        getWithInstance("0x1400", &uptime);
        RCLCPP_INFO_STREAM(nd->get_logger(),"\nSystem uptime: " << uptime << " s" << std::endl);
    }
    catch(const char* res)
    {
    connection_error = res;
    RCLCPP_ERROR(nd->get_logger(), "Failed Connection! %s", connection_error.c_str());
    }
}
//Reconfigure Parameter callback
void SchunkGripperNode::parameterEventCallback(const rcl_interfaces::msg::ParameterEvent::SharedPtr param)
{  
    try
    {
    std::vector<rcl_interfaces::msg::Parameter> sorted_parameters = param->changed_parameters;
    if(!param->new_parameters.empty())
    { 
        sorted_parameters.clear();
        sorted_parameters = param->new_parameters;
    }
    std::sort(sorted_parameters.begin(), sorted_parameters.end(),
    [](const rcl_interfaces::msg::Parameter& a, const rcl_interfaces::msg::Parameter& b) {
    return a.name > b.name; });

    std::vector<rcl_interfaces::msg::Parameter>::iterator iter = sorted_parameters.begin();

    for(auto fparam : sorted_parameters)
    {    
        if(fparam.name == "diagnostic_updater.period") gripper_updater->setPeriod(fparam.value.double_value);
        if(fparam.name.substr(0,17) != "Gripper_Parameter") break;

        switch(fparam.value.type)
        {
        case rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE:
                {
                const std::lock_guard<std::recursive_mutex> lock(lock_mutex);
                changeParameter(parameter_map.at(fparam.name), static_cast<float>(fparam.value.double_value));
                RCLCPP_INFO_STREAM(nd->get_logger(), fparam.name << " changed to: " << fparam.value.double_value);
                if(fparam.name == "Gripper_Parameter.zero_pos_ofs")
                {   
                    getWithInstance<float>(MAX_POS_INST, &max_pos);
                    getWithInstance<float>(MIN_POS_INST,&min_pos);
                    runGets();
                    nd->undeclare_parameter("Control_Parameter.move_gripper");
                    nd->declare_parameter("Control_Parameter.move_gripper", actualPosInterval(), parameter_descriptor("Moving the gripper with parameter in mm", FloatingPointRange(min_pos,max_pos)), true);
                    abs_pos_param = actualPosInterval();
                }
                }
                break;

        case rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER:
                if(fparam.name == "Gripper_Parameter.grp_prehold_time")
                {
                    const std::lock_guard<std::recursive_mutex> lock(lock_mutex);
                    changeParameter(GRP_PREHOLD_TIME_INST, static_cast<uint16_t>(fparam.value.integer_value), &grp_prehold_time);
                    RCLCPP_INFO_STREAM(nd->get_logger(), fparam.name << " changed to: " << fparam.value.integer_value);
                }
                    break;
        case rcl_interfaces::msg::ParameterType::PARAMETER_BOOL:
                if(fparam.name == "Gripper_Parameter.use_brk" && (gripperBitOutput(USE_GPE)!= fparam.value.bool_value))
                {
                    const std::lock_guard<std::recursive_mutex> lock(lock_mutex);
                    runPost(USE_GPE);
                    grp_pos_lock = gripperBitOutput(USE_GPE);
                    RCLCPP_INFO(nd->get_logger(),"The brake is %s.", grp_pos_lock ? "on" : "off");
                }    
                    break;
        }
        iter++;
    }

   for(auto &controlparam = iter; controlparam != sorted_parameters.end(); controlparam++)
   {    
        if(controlparam->name.substr(0, 17) != "Control_Parameter") break;

        if(controlparam->name == "Control_Parameter.move_gripper" && param_exe == false && controlparam->value.double_value != abs_pos_param) 
        {
            const std::lock_guard<std::recursive_mutex> lock(lock_mutex);
            actual_command = "MOVE TO ABSOLUTE POSITION";
            runPost(MOVE_TO_ABSOLUTE_POSITION, static_cast<uint32_t>(nd->get_parameter("Control_Parameter.move_gripper").as_double()*1000) , static_cast<uint32_t>(nd->get_parameter("Control_Parameter.move_gripper_velocity").as_double()*1000));
            abs_pos_param = actualPosInterval();
            param_exe = true;
            handshake = gripperBitInput(COMMAND_RECEIVED_TOGGLE);
        }
        else if(controlparam->name == "Control_Parameter.grip" && controlparam->value.bool_value == true && param_exe == false) 
        {
            const std::lock_guard<std::recursive_mutex> lock(lock_mutex);
            uint32_t command = GRIP_WORK_PIECE;

            if(gripperBitOutput(GRIP_DIRECTION) != nd->get_parameter("Control_Parameter.grip_direction").as_bool())
                command |= GRIP_DIRECTION;
                
            actual_command = "GRIP WORKPIECE";
            msg.doing_command = actual_command;
            runPost(command, 0, 0, static_cast<uint32_t>(nd->get_parameter("Control_Parameter.grip_force").as_double()));
            param_exe = true;
            handshake = gripperBitInput(COMMAND_RECEIVED_TOGGLE);
        }
        else if(controlparam->name == "Control_Parameter.release_workpiece"  && controlparam->value.bool_value == true && param_exe == false) 
        {
            if (gripperBitInput(GRIPPED))
            {
                const std::lock_guard<std::recursive_mutex> lock(lock_mutex);
                actual_command = "RELEASE WORKPIECE";
                msg.doing_command = actual_command;
                runPost(commands_str.at(actual_command));
                param_exe = true;
                handshake = gripperBitInput(COMMAND_RECEIVED_TOGGLE);
            }
            else 
            {
                RCLCPP_INFO(nd->get_logger(),"No Workpiece Gripped");
                nd->set_parameter(rclcpp::Parameter("Control_Parameter.release_workpiece", false));
            }
        }
   }
   }
   catch(const char* res)
   {
        RCLCPP_ERROR(nd->get_logger() , "Could't set the Parameter(s). \nConnection failed: %s");
        for(auto element : param->changed_parameters)
        {
            if(element.name.substr(0,17) != "Control_Parameter") failed_param->changed_parameters.push_back(element);
        }
   }
}
//Diagnostics 
void SchunkGripperNode::gripperDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& status)
{   
    splitted_Diagnosis = splitDiagnosis();

    try
    {
    if(connection_error != "OK")
    {
    status.summaryf(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Connection failed: %s", connection_error.c_str());
    return;
    }
    if(gripperBitInput(EMERGENCY_RELEASED)) 
    status.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "If you want to end this mode, perform a fast stop and acknowledge!");
    
    else if(gripperBitInput(READY_FOR_SHUTDOWN)) 
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
                RCLCPP_ERROR(nd->get_logger(),"%i\n%s",splitted_Diagnosis[2] , error_str.c_str());
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
                RCLCPP_WARN(nd->get_logger(),"%s",(warn_str+ ": " + std::to_string(splitted_Diagnosis[0])).c_str());
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
                RCLCPP_WARN(nd->get_logger(),"%i\n%s", splitted_Diagnosis[1], warn_str.c_str());
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
            RCLCPP_INFO(nd->get_logger(),"%s",error_str.c_str());
        }
        status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, error_str);
    }
}
catch(const char* res)
{
    connection_error = res;
    RCLCPP_ERROR(nd->get_logger(), "Failed Connection! %s", connection_error.c_str());
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

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr SchunkGripperNode::get_node_base_interface() const
{
    return nd->get_node_base_interface();
}

SchunkGripperNode::~SchunkGripperNode()
{ }

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(SchunkGripperNode)
