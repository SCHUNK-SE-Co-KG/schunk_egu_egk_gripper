#include "schunk_gripper/schunk_gripper_wrapper.h"

 std::map<std::string, const char*> parameter_map = {
        { "Gripper_Parameter.grp_pos_margin", GRP_POS_MARGIN_INST},
        { "Gripper_Parameter.grp_prepos_delta", GRP_PREPOS_DELTA_INST },
        { "Gripper_Parameter.zero_pos_ofs", ZERO_POS_OFS_INST },
        { "Gripper_Parameter.grp_prehold_time", GRP_PREHOLD_TIME_INST },
        { "Gripper_Parameter.wp_release_delta", WP_RELEASE_DELTA_INST },
        { "Gripper_Parameter.wp_lost_distance", WP_LOST_DISTANCE_INST },
    };

//Initialize the ROS Driver
SchunkGripperNode::SchunkGripperNode(std::shared_ptr<rclcpp::Node> nd,std::string ip, float state_frq, float joint_state_frq) : 
    Gripper(ip),
    nd(nd),
    state_frq(state_frq),
    j_state_frq(joint_state_frq),
    cycletime(1/state_frq),
    limiting_rate(1000)
    //param_server(mutex)
    //ActionServer
    /*
    move_abs_server(*nd, "move_absolute", boost::bind(&SchunkGripperNode::moveAbsExecute, this, _1), false),
    move_rel_server(*nd, "move_relative", boost::bind(&SchunkGripperNode::moveRelExecute, this, _1), false),
    grip_server(*nd, "grip_egk", boost::bind(&SchunkGripperNode::gripExecute, this, _1), false),
    grip_egu_server(*nd, "grip_egu", boost::bind(&SchunkGripperNode::grip_eguExecute, this, _1), false),
    grip_w_pos_server(*nd, "grip_with_pos_egk", boost::bind(&SchunkGripperNode::gripWithPositionExecute, this, _1), false),
    grip_w_pos_egu_server(*nd, "grip_with_pos_egu", boost::bind(&SchunkGripperNode::gripWithPosition_eguExecute, this, _1),false),
    release_wp_server(*nd, "release_workpiece", boost::bind(&SchunkGripperNode::releaseExecute, this, _1),false),
    control_server(*nd, "gripper_control", boost::bind(&SchunkGripperNode::controlExecute, this, _1),false),
    */
{   

    using std::placeholders::_1;
    using std::placeholders::_2;

    messages_group = nd->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    services_group = nd->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rest = nd->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::PublisherOptions option_messages;
    option_messages.callback_group = messages_group;
    rclcpp::SubscriptionOptions option;
    option.callback_group = rest;
    
    declareParameter();
    
    parameters_client = std::make_shared<rclcpp::AsyncParametersClient>(nd);

    while (!parameters_client->wait_for_service(std::chrono::seconds(1))) 
    {
        if (!rclcpp::ok()) 
        {
        RCLCPP_ERROR(nd->get_logger(), "Interrupted while waiting for the service. Exiting.");
        }
    }
    
    parameter_event = parameters_client->on_parameter_event(std::bind(&SchunkGripperNode::parameterEventCallback,this,_1), 10, option);
    
    acknowledge_service = nd->create_service<schunk_gripper::srv::Acknowledge>("acknowledge", std::bind(&SchunkGripperNode::acknowledge_srv,this,_1,_2), rmw_qos_profile_services_default, services_group);
    stop_service = nd->create_service<schunk_gripper::srv::Stop>("stop", std::bind(&SchunkGripperNode::stop_srv,this,_1,_2), rmw_qos_profile_services_default, services_group);
    softreset_service = nd->create_service<schunk_gripper::srv::Softreset>("softreset", std::bind(&SchunkGripperNode::softreset_srv,this,_1,_2), rmw_qos_profile_services_default, services_group);
    releaseForManualMov_service = nd->create_service<schunk_gripper::srv::ReleaseForManMov>("release_for_manual_movement", std::bind(&SchunkGripperNode::releaseForManualMov_srv,this,_1,_2), rmw_qos_profile_services_default, services_group);
    prepare_for_shutdown_service = nd->create_service<schunk_gripper::srv::PrepareForShutdown>("prepare_for_shutdown", std::bind(&SchunkGripperNode::prepare_for_shutdown_srv,this,_1,_2), rmw_qos_profile_services_default, services_group);
    fast_stop_service = nd->create_service<schunk_gripper::srv::FastStop>("fast_stop", std::bind(&SchunkGripperNode::fast_stop_srv,this,_1,_2), rmw_qos_profile_services_default, services_group);
    info_service = nd->create_service<schunk_gripper::srv::GripperInfo>("gripper_info", std::bind(&SchunkGripperNode::info_srv,this,_1,_2), rmw_qos_profile_services_default, rest);

    //Actually no Command
    actual_command = "NO COMMAND";
    //get an error and a warn string
    error_str = getErrorString(0);
    warn_str = getErrorString(0);
    //Initlize diagnostics
  //  gripper_updater.setHardwareID("Modul");
  //  gripper_updater.add(model.c_str(), boost::bind(&SchunkGripperNode::gripperDiagnostics, this, _1));
    //Advertise state
    statePublisher = nd->create_publisher<schunk_gripper::msg::State>("state", 100);
    publish_state_timer=nd->create_wall_timer(std::chrono::duration<float>(1/state_frq), std::bind(&SchunkGripperNode::publishState, this), messages_group);
    //Look if joint_state_frq is less than state_frq
    float j_state_frq = joint_state_frq;
    if(joint_state_frq > state_frq)
    {
        j_state_frq = state_frq;
        RCLCPP_WARN(nd->get_logger(),"joint_state topic will publish with %f!", j_state_frq);
    }
    //Advertise joint_states
    //jointStatePublisher = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
 //   publish_joint_timer=this->create_wall_timer(std::chrono::duration<float>(1/j_state_frq), std::bind(&SchunkGripperNode::publishJointState, this));
    //Start Grip actions depending opn the model    
  
    if(model.find("EGK") != std::string::npos)
    {
        /*
        grip_server.start();
        grip_w_pos_server.start();
        grip_w_pos_egu_server.~SimpleActionServer();
        grip_egu_server.~SimpleActionServer();
        */
    }
    else if(model.find("EGU") != std::string::npos)
    {
        /*
        grip_egu_server.start();
        grip_w_pos_egu_server.start();
        grip_w_pos_server.~SimpleActionServer();
        grip_server.~SimpleActionServer();
        */
    }
    else
    {
        /*
        RCLCPP_ERROR("GRIPPER NOT FOUND!");
        ros::shutdown();
        return;
        */
    }
/*
    //Start Actions
    move_abs_server.start();
    move_rel_server.start();
    release_wp_server.start();
    control_server.start();
*/
    //Set dynamic reconfigure Callback
//    param_server.setCallback(boost::bind(&SchunkGripperNode::parameterConfigure, this, _1, _2));

    handshake = gripperBitInput(COMMAND_RECEIVED_TOGGLE);
}
//State msg update
void SchunkGripperNode::updateStateMsg()
{
   
    msg.command_received_toggle = gripperBitInput(COMMAND_RECEIVED_TOGGLE); 

    msg.doing_command = actual_command;       //It is the only msg...,that is not updatet through "runGets()"
                                              //It is asynchron with the others
    msg.command_successfully_processed = gripperBitInput(SUCCESS);
    msg.position_reached = gripperBitInput(POSITION_REACHED);
    msg.workpiece_gripped = gripperBitInput(GRIPPED);
    msg.workpiece_lost = gripperBitInput(WORK_PIECE_LOST);
    msg.wrong_workpiece = gripperBitInput(WRONG_WORKPIECE_DETECTET);
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
    msg.grip_force_and_maintance = gripperBitInput(BRAKE_ACTIVE);
}
//parameterlimits
void SchunkGripperNode::declareParameter()
{   
    // Gripper Parameter
    nd->declare_parameter("Gripper_Parameter.use_brk", false, parameter_descriptor("Use brake"));
    nd->declare_parameter("Gripper_Parameter.grp_pos_margin", 2.0, parameter_descriptor("Grip position margin in mm", FloatingPointRange(1.0, 10.0, 0.01)));
    nd->declare_parameter("Gripper_Parameter.grp_prepos_delta", 5.0, parameter_descriptor("Grip position delta in mm", FloatingPointRange(1.0, 50.0, 0.01)));
    nd->declare_parameter("Gripper_Parameter.zero_pos_ofs", 0.0, parameter_descriptor("Zero position offset in mm", FloatingPointRange(-10000.0, 10000.0, 0.01)));
    nd->declare_parameter("Gripper_Parameter.grp_prehold_time", 0, parameter_descriptor("Grip prehold time in ms", IntegerRange(0, 60000, 1)));
    nd->declare_parameter("Gripper_Parameter.wp_release_delta", 5.0, parameter_descriptor("Workpiece release delta in mm", FloatingPointRange(1.0, 50.0, 0.01)));
    nd->declare_parameter("Gripper_Parameter.wp_lost_distance", 1.0, parameter_descriptor("Max. distance after workpiece lost in mm", FloatingPointRange(0.1, 50.0, 0.01)));
    // Control Parameter
    nd->declare_parameter("Control_Parameter.move_gripper", actualPosInterval(), parameter_descriptor("Moving the gripper with parameter in mm", FloatingPointRange(min_pos, max_pos)));
    nd->declare_parameter("Control_Parameter.move_gripper_velocity", static_cast<double>(min_vel), parameter_descriptor("Changing the velocity for move_Gripper in mm/s", FloatingPointRange(min_vel, max_vel)));
    nd->declare_parameter("Control_Parameter.grip_direction", false, parameter_descriptor("Grip direction for parameter gripping"));
    nd->declare_parameter("Control_Parameter.grip", false, parameter_descriptor("Grip with parameter"));
    nd->declare_parameter("Control_Parameter.grip_force", 50.0, parameter_descriptor("Grip force in %", FloatingPointRange(50.0, 100.0, 0.01)));
    nd->declare_parameter("Control_Parameter.release_workpiece", false, parameter_descriptor("Release Workpiece"));
//    abs_pos_param = nd->get_parameter("Control_Parameter.move_gripper").as_double();
    
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
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    descriptor.description = description;
    return descriptor;
}

double SchunkGripperNode::actualPosInterval()
{
    if(actual_pos < min_pos) return static_cast<double>(min_pos);
    else if(actual_pos > max_pos)return static_cast<double>(max_pos);
    else return actual_pos;
}
/*
//Deal with gripper, if the Command where successfull done. Else throw an Exception
template<typename feedtype, typename restype, typename servertype>
void SchunkGripperNode::setFinalState(feedtype &feedback, restype &res, servertype &server)
{   
    //Get current handshake
   // handshake = gripperBitInput(COMMAND_RECEIVED_TOGGLE);
    //publish end feedback
    server.publishFeedback(feedback);
    //Gripper was stopped
    if(gripperBitOutput(STOP))
    {
        finishedCommand();
        server.setPreempted(res);
        return;
    }
    //Gripper succeded or Workpiece lost...
    else if(gripperBitInput(SUCCESS)
          ||gripperBitInput(NO_WORKPIECE_DETECTED)
          ||gripperBitInput(WORK_PIECE_LOST) 
          ||gripperBitInput(WRONG_WORKPIECE_DETECTET))
    {
        finishedCommand();
        server.setSucceeded(res);
        return;
    }
    //Gripper was fast stopped
    else if(!gripperBitOutput(FAST_STOP))
    {  
        finishedCommand();
        RCLCPP_WARN(nd->get_logger(),"Fast stopped. Reason could be another goal or fast stop service.");
        server.setPreempted(res);
        return;
    }
    //Cancel or new goal arived
    else if(server.isPreemptRequested())  
    {
        if(server.isNewGoalAvailable() && action_move == true) //If the gripper had a move command, it will move depending on new goal
        {
            RCLCPP_WARN(nd->get_logger(),"New Moving Goal. Server set preempted.");
            server.setPreempted(res);
            return;
        }
        else if(server.isNewGoalAvailable())ROS_ERROR("The action is canceled, because of a new goal! Fast stop.");
        else ROS_ERROR("Cancel Request received. Fast stop.");
        
        lock_mutex.lock();
            //Fast stop
            runPost(FAST_STOP);
            if(handshake != gripperBitInput(COMMAND_RECEIVED_TOGGLE)) 
            while (ros::ok() && check())
            runGets();
            
            handshake = gripperBitInput(COMMAND_RECEIVED_TOGGLE);
        
        lock_mutex.unlock();

        gripper_updater.force_update();
        finishedCommand();
        
        server.setPreempted(res);
        return;
    }

    else 
    {
        throw plc_sync_input[3];
    }
}
//Deal with Exceptions on different levels of the driver
template<typename actiontype>
void SchunkGripperNode::exceptionHandling(actiontype &server, uint32_t &i)
{
        handshake = gripperBitInput(COMMAND_RECEIVED_TOGGLE);

        if(plc_sync_input[3] != 0) gripper_updater.force_update();
        //If gripper did not receive command
        else if(i == -1)
        {                 
            ROS_ERROR("GRIPPER DID NOT RECEIVE THE COMMAND!");
            lock_mutex.unlock();
        }
        else RCLCPP_WARN(nd->get_logger(),"Action aborted!");

        finishedCommand();
        server.setAborted();
        return;

}   
//Looking if no other goal or Action is active. If all is fine it return 1
template<typename servertype>
bool SchunkGripperNode::canActionStart(servertype& server)
{   //Error
    if(splitted_Diagnosis[2] != 0) 
    {
        ROS_ERROR("IF THERE IS AN ERROR, NO ACTION IS PROCESSED!");
    }
    //action is active
    else if(action_active == true)
    {   
        RCLCPP_WARN(nd->get_logger(),"An other Action is active. Goal aborted. Gripper is: %s", actual_command.c_str());
        if(plc_sync_input[3] == 0)
        {
            lock_mutex.lock();
            runPost(FAST_STOP);
            handshake = gripperBitInput(COMMAND_RECEIVED_TOGGLE);
            lock_mutex.unlock();
        }
    }
    //An Command is active (Parameter)
    else if(actual_command != "NO COMMAND")
    {
        RCLCPP_WARN(nd->get_logger(),"Gripper is doing %s, goal aborted!",actual_command.c_str());
    }
    //Nothing is active
    else if(actual_command == "NO COMMAND") 
    {
        return 1;
    }
    else 
    {
        ROS_ERROR("Something went wrong.");
    }
    
    server.setAborted();
    return 0;
}
//Action publishing its feedback as long the Gripper is moving
template<typename feedbacktype, typename servertype>
void SchunkGripperNode::runActionMove(feedbacktype &feedback, servertype &server)
{   
    ros::Time start_time = ros::Time::now();

    while(endCondition() && ros::ok() && (!server.isPreemptRequested()))
    {   //Publish as fast as in state
        if(cycletime <= ros::Time::now() - start_time)
        {
            feedback.current_position = actual_pos;
            feedback.current_velocity = actual_vel;
            feedback.motor_current = actual_cur;

            server.publishFeedback(feedback);
            start_time = ros::Time::now();
        }
        //Limits the While loop
        limiting_rate.sleep();
    }
    //Get end values
    lock_mutex.lock();
    runGets();
    lock_mutex.unlock();
    //Gripper error not provoked through fast stop
    if(gripperBitInput(GRIPPER_ERROR) && gripperBitOutput(FAST_STOP)) throw plc_sync_input[3];

    feedback.current_position = actual_pos;
    feedback.current_velocity = actual_vel;
    feedback.motor_current = actual_cur;   
};
//Action publishing feedback as long the gripper is moving
template<typename feedbacktype, typename servertype>
void SchunkGripperNode::runActionGrip(feedbacktype &feedback, servertype &server)
{   
    ros::Time start_time = ros::Time::now();

    while(endCondition() && ros::ok() && (!server.isPreemptRequested()))
    {   //Publish as fast as in state
        if(cycletime <= ros::Time::now() - start_time)
        {
            feedback.current_position = actual_pos;
            feedback.current_velocity = actual_vel;
            feedback.motor_current = actual_cur;
            //if preempt started-> print once
            if(feedback.pre_grip == false && gripperBitInput(PRE_HOLDING) == true)
            {
                RCLCPP_INFO(nd->get_logger(),"PRE-HOLDING: %i ms", grp_prehold_time);
                feedback.pre_grip = gripperBitInput(PRE_HOLDING);
            }
                    
            server.publishFeedback(feedback);
            start_time = ros::Time::now();
        }
        //Limits the While loop
        limiting_rate.sleep();
    }
    //Get actual values
    lock_mutex.lock();
    runGets();
    lock_mutex.unlock();
    //Error without fast stop
    if(gripperBitInput(GRIPPER_ERROR) && gripperBitOutput(FAST_STOP)) throw plc_sync_input[3];


    feedback.current_position = actual_pos;
    feedback.current_velocity = actual_vel;
    feedback.motor_current = actual_cur;
    feedback.pre_grip = gripperBitInput(PRE_HOLDING);
};
//Move absolute action callback
void SchunkGripperNode::moveAbsExecute(const schunk_gripper::mov_abs_posGoalConstPtr &goal)
{
    //Last was an absolute move which was preempted
    if(action_move == true && actual_command == "MOVE TO ABSOLUTE POSITION");
    else if(!canActionStart(move_abs_server)) return;

    action_active = true;
    action_move = true;
    actual_command = "MOVE TO ABSOLUTE POSITION";

    schunk_gripper::mov_abs_posFeedback feedback;
    schunk_gripper::mov_abs_posResult res;
    
    uint32_t goal_position = mm2mu(goal->abs_position);
    uint32_t goal_velocity = mm2mu(goal->velocity);

try
{  
    lock_mutex.lock();
        //Run goal
        runPost(commands_str.at(actual_command), goal_position, goal_velocity);
        //Look if gripper received command     
        if(handshake == gripperBitInput(COMMAND_RECEIVED_TOGGLE)) throw uint32_t(-1);
        else handshake = gripperBitInput(COMMAND_RECEIVED_TOGGLE);
    
    lock_mutex.unlock();
    
    runActionMove(feedback, move_abs_server);
    //set result
    res.current_position = actual_pos;
    res.reached_position = gripperBitInput(SUCCESS) && gripperBitInput(POSITION_REACHED);

    setFinalState(feedback, res, move_abs_server);
}
catch(uint32_t i)
{
    exceptionHandling(move_abs_server, i);
}

};
//Move relative action callback
void SchunkGripperNode::moveRelExecute(const schunk_gripper::mov_rel_posGoalConstPtr &goal)
{   
    //Last was Move to relative position
    if(action_move == true && actual_command == "MOVE TO RELATIVE POSITION");
    else if(!canActionStart(move_rel_server)) return;
    
    action_active = true;
    action_move = true;
    actual_command = "MOVE TO RELATIVE POSITION";

    schunk_gripper::mov_rel_posFeedback feedback;
    schunk_gripper::mov_rel_posResult res;
    
    uint32_t goal_position = mm2mu(goal->distance);
    uint32_t goal_velocity = mm2mu(goal->max_velocity);
try
{    

        lock_mutex.lock();
            //Send command to gripper
            runPost(commands_str.at(actual_command), goal_position, goal_velocity);
            //Look if gripper received command
            if(handshake == gripperBitInput(COMMAND_RECEIVED_TOGGLE)) throw uint32_t(-1);
            else handshake = gripperBitInput(COMMAND_RECEIVED_TOGGLE);
        
        lock_mutex.unlock();

    runActionMove(feedback,move_rel_server);    //running as long the gripper moves and no other goal or action is send
    //set result
    res.current_position = actual_pos;
    res.reached_position = gripperBitInput(SUCCESS) && gripperBitInput(POSITION_REACHED);

    setFinalState(feedback, res, move_rel_server);

}
catch(uint32_t i)
{
    exceptionHandling(move_rel_server, i);
}
};
//Grip workpiece (EGK) action callback
void SchunkGripperNode::gripExecute(const schunk_gripper::gripGoalConstPtr &goal)
{
    if(!canActionStart(grip_server)) return;
    
    action_active = true;
    actual_command = "GRIP WORKPIECE";

    schunk_gripper::gripFeedback feedback;
    schunk_gripper::gripResult res;

    feedback.pre_grip = false;
    //Float Goals to uint32_t
    uint32_t goal_effort = static_cast<uint32_t>(goal->effort);
    uint32_t goal_velocity = mm2mu(goal->max_velocity);
    uint32_t goal_command = commands_str.at(actual_command);
    //if last grip diretion is not current grip direction
    if(gripperBitOutput(GRIP_DIRECTION) != goal->grp_dir)
    goal_command |= GRIP_DIRECTION;
    
try
{       
        lock_mutex.lock();
            //Send command to gripper
            runPost(goal_command, 0, goal_velocity, goal_effort);
            //Look if gripper received command
            if(handshake == gripperBitInput(COMMAND_RECEIVED_TOGGLE)) throw uint32_t(-1);
            handshake = gripperBitInput(COMMAND_RECEIVED_TOGGLE);

        lock_mutex.unlock();

    runActionGrip(feedback, grip_server);
    //Set result
    res.no_workpiece_detected = gripperBitInput(NO_WORKPIECE_DETECTED);
    res.workpiece_lost = gripperBitInput(WORK_PIECE_LOST);
    res.gripped = gripperBitInput(GRIPPED);
    res.current_position = actual_pos;

    setFinalState(feedback, res, grip_server);
}
catch(uint32_t i)
{
    exceptionHandling(grip_server, i);
}

};
//Grip workpiece (EGU) action callback
void SchunkGripperNode::grip_eguExecute(const schunk_gripper::grip_eguGoalConstPtr &goal)
{
    if(!canActionStart(grip_egu_server)) return;
    
    action_active = true;
    actual_command = "GRIP WORKPIECE";

    schunk_gripper::grip_eguFeedback feedback;
    schunk_gripper::grip_eguResult res;
    
    uint32_t goal_effort = static_cast<uint32_t>(goal->effort);
    uint32_t goal_command = commands_str.at(actual_command);

    if(gripperBitOutput(GRIP_DIRECTION) != goal->grp_dir)
    goal_command |= GRIP_DIRECTION;
    
try
{       
    lock_mutex.lock();
        //Send command to gripper
        runPost(goal_command, 0, 0, goal_effort);
        //Look if gripper received command
        if(handshake == gripperBitInput(COMMAND_RECEIVED_TOGGLE)) throw uint32_t(-1);
        handshake = gripperBitInput(COMMAND_RECEIVED_TOGGLE);
    lock_mutex.unlock();

    runActionGrip(feedback, grip_egu_server);
    //Set result
    res.no_workpiece_detected = gripperBitInput(NO_WORKPIECE_DETECTED);
    res.workpiece_lost = gripperBitInput(WORK_PIECE_LOST);
    res.gripped = gripperBitInput(GRIPPED);
    res.current_position = actual_pos;

    setFinalState(feedback, res, grip_egu_server);
}
catch(uint32_t i)
{
    exceptionHandling(grip_egu_server, i);
}

}
//Grip workpiece with position (EGK) action callback
void SchunkGripperNode::gripWithPositionExecute(const schunk_gripper::grip_with_posGoalConstPtr &goal)
{
    if(!canActionStart(grip_w_pos_server)) return;
    
    action_active = true;

    actual_command = "GRIP WORKPIECE WITH POSITION";
    msg.doing_command = actual_command;

    schunk_gripper::grip_with_posFeedback feedback;
    schunk_gripper::grip_with_posResult res;
    
    uint32_t goal_position = mm2mu(goal->abs_position);
    uint32_t goal_velocity = mm2mu(goal->velocity);
    uint32_t goal_effort = static_cast<uint32_t>(goal->effort);
    uint32_t goal_command = GRIP_WORKPIECE_WITH_POSITION;
    //Is last grip direction goal direction?
    if(gripperBitOutput(GRIP_DIRECTION) != goal->grp_dir)
    goal_command |= GRIP_DIRECTION;

try
{       
        lock_mutex.lock();
            //run Command
            runPost(goal_command, goal_position, goal_velocity, goal_effort);
            //Have Gripper received command
            if(handshake == gripperBitInput(COMMAND_RECEIVED_TOGGLE)) throw uint32_t(-1);
            else handshake = gripperBitInput(COMMAND_RECEIVED_TOGGLE);
        lock_mutex.unlock();

    runActionGrip(feedback, grip_w_pos_server);
    //Set result
    res.no_workpiece_detected = gripperBitInput(NO_WORKPIECE_DETECTED);
    res.workpiece_lost = gripperBitInput(WORK_PIECE_LOST);
    res.gripped = gripperBitInput(GRIPPED);
    res.current_position = actual_pos;
    res.wrong_workpiece_detected = gripperBitInput(WRONG_WORKPIECE_DETECTET);

    setFinalState(feedback, res, grip_w_pos_server);
}
catch(uint32_t i)
{
    exceptionHandling(grip_w_pos_server, i);
}

};
//Grip with position (EGU) action callback
void SchunkGripperNode::gripWithPosition_eguExecute(const schunk_gripper::grip_with_pos_eguGoalConstPtr &goal)
{
    if(!canActionStart(grip_w_pos_egu_server)) return;
    
    action_active = true;

    actual_command = "GRIP WORKPIECE WITH POSITION";
    msg.doing_command = actual_command;

    schunk_gripper::grip_with_pos_eguFeedback feedback;
    schunk_gripper::grip_with_pos_eguResult res;
    
    uint32_t goal_position = mm2mu(goal->abs_position);
    uint32_t goal_effort = static_cast<uint32_t>(goal->effort);
    uint32_t goal_command = GRIP_WORKPIECE_WITH_POSITION;
    //Is last grip direction current direction
    if(gripperBitOutput(GRIP_DIRECTION) != goal->grp_dir)
    goal_command |= GRIP_DIRECTION;

try
{       
        lock_mutex.lock();
            //Send command
            runPost(goal_command, goal_position, 0, goal_effort);
            //Look if gripper received command
            if(handshake == gripperBitInput(COMMAND_RECEIVED_TOGGLE)) throw uint32_t(-1);
            else handshake = gripperBitInput(COMMAND_RECEIVED_TOGGLE);
        lock_mutex.unlock();

    runActionGrip(feedback, grip_w_pos_egu_server);
    //Set result
    res.no_workpiece_detected = gripperBitInput(NO_WORKPIECE_DETECTED);
    res.workpiece_lost = gripperBitInput(WORK_PIECE_LOST);
    res.wrong_workpiece_detected = gripperBitInput(WRONG_WORKPIECE_DETECTET);
    res.gripped = gripperBitInput(GRIPPED);
    res.current_position = actual_pos;

    setFinalState(feedback, res, grip_w_pos_egu_server);
}
catch(uint32_t i)
{
    exceptionHandling(grip_w_pos_egu_server, i);
}
};
//release workpiece action callback
void SchunkGripperNode::releaseExecute(const schunk_gripper::release_workpieceGoalConstPtr &gaol)
{
   if(!canActionStart(release_wp_server)) return;
   
   action_active = true;
   actual_command = "RELEASE WORKPIECE";

   schunk_gripper::release_workpieceFeedback feedback;
   schunk_gripper::release_workpieceResult res;

try
{   
    lock_mutex.lock();
        //Send command
        runPost(commands_str.at(actual_command));
        //Have gripper received command
        if(handshake == gripperBitInput(COMMAND_RECEIVED_TOGGLE)) throw uint32_t(-1);
        else handshake = gripperBitInput(COMMAND_RECEIVED_TOGGLE);
    
    lock_mutex.unlock();
    
    runActionMove(feedback, release_wp_server);

    res.current_position = actual_pos;
    res.released = gripperBitInput(SUCCESS);

    setFinalState(feedback, res, release_wp_server);
}
catch(uint32_t i)
{
    exceptionHandling(release_wp_server, i);
}

};
//Grip: control_msgs
void SchunkGripperNode::controlExecute(const control_msgs::GripperCommandGoalConstPtr &goal)
{
    if(!canActionStart(control_server)) return;
    
    action_active = true;

    control_msgs::GripperCommandFeedback feedback;
    control_msgs::GripperCommandResult res;

    feedback.stalled = false;
    feedback.reached_goal = false;

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
{   lock_mutex.lock();
        //run command
        runPost(goal_command, goal_position, goal_vel, goal_effort);
        //Gripper received command
        if(handshake == gripperBitInput(COMMAND_RECEIVED_TOGGLE)) throw uint32_t(-1);
        else handshake = gripperBitInput(COMMAND_RECEIVED_TOGGLE);
    lock_mutex.unlock();

    while(endCondition() && ros::ok() && (!control_server.isPreemptRequested()))
    {
        feedback.position = actual_pos;
        control_server.publishFeedback(feedback);
        cycletime.sleep();
    }

    lock_mutex.lock();
    runGets();
    lock_mutex.unlock();

    feedback.position = actual_pos;
    //Set result
    res.position = actual_pos;
    if(actual_command == "MOVE TO ABSOLUTE POSITION")
    {
        res.reached_goal = gripperBitInput(SUCCESS) && gripperBitInput(POSITION_REACHED);
        res.stalled = gripperBitInput(SUCCESS);
    }
    else
    {
        res.reached_goal = gripperBitInput(SUCCESS) && gripperBitInput(GRIPPED);
        res.stalled = gripperBitInput(SUCCESS);
    }

    setFinalState(feedback, res, control_server);
}
catch(uint32_t i)
{
    exceptionHandling(control_server, i);
}

action_move = false;

};
*/
//Updates final successfull state and zero position offset
void SchunkGripperNode::finishedCommand()
{   
        const std::lock_guard<std::recursive_mutex> lock(lock_mutex);
        //update Parameter
        nd->set_parameter(rclcpp::Parameter("Control_Parameter.move_gripper", actualPosInterval()));
        abs_pos_param =  nd->get_parameter("Control_Parameter.move_gripper").as_double();
        nd->set_parameter(rclcpp::Parameter("Control_Parameter.grip", false));
        nd->set_parameter(rclcpp::Parameter("Control_Parameter.release_workpiece", false));

        //Was Command successfully processed?
        if(gripperBitInput(SUCCESS) && gripperBitInput(POSITION_REACHED)) 
        RCLCPP_INFO(nd->get_logger(),"%s SUCCEDED", actual_command.c_str());
        
        if(gripperBitInput(GRIPPED) && zero_changed == false)  
        RCLCPP_WARN(nd->get_logger(),"Gripped Workpiece!");
        //Flags
        actual_command = "NO COMMAND";
        msg.doing_command = actual_command;
        action_active = false;
        param_exe = false;
        action_move = false;

}
//publish with x hz gripper data
void SchunkGripperNode::publishState()
{
    if(lock_mutex.try_lock())
    {
        runGets();
        lock_mutex.unlock();
    }
    
    if( ((param_exe == true && !endCondition()) || (zero_changed == true)) && !gripperBitInput(PRE_HOLDING))                   //If param_exe was active and modul is in an endstate
    finishedCommand();                                                                      //If zero_offset was changed

    if(msg.workpiece_lost != gripperBitInput(WORK_PIECE_LOST) && !msg.workpiece_lost)
    RCLCPP_WARN(nd->get_logger(),"Workpiece lost!");

    if(msg.no_workpiece_detected != gripperBitInput(NO_WORKPIECE_DETECTED) && !msg.no_workpiece_detected)
    RCLCPP_WARN(nd->get_logger(),"No workpiece detected!");

    if(msg.wrong_workpiece != gripperBitInput(WRONG_WORKPIECE_DETECTET) && !msg.wrong_workpiece) 
    RCLCPP_WARN(nd->get_logger(),"Wrong workpiece detected!");
    //Update msg
    updateStateMsg();
    //Publish state
    statePublisher->publish(msg);

   // gripper_updater.update();

}
/*
//JointStatePublisher
void SchunkGripperNode::publishJointState(const ros::TimerEvent& event)
{
    gripper_updater.update();
    sensor_msgs::JointState joint_msg;
    joint_msg.header.stamp = ros::Time::now();
    joint_msg.header.frame_id="base_link_left";
    joint_msg.name.push_back("egu_prismatic_joint_translational_left");
    joint_msg.position.push_back(actual_pos);
    joint_msg.velocity.push_back(actual_vel);
    jointStatePublisher.publish(joint_msg);
};
*/
//Acknowledge service callback
void SchunkGripperNode::acknowledge_srv(const std::shared_ptr<schunk_gripper::srv::Acknowledge::Request>, std::shared_ptr<schunk_gripper::srv::Acknowledge::Response> res)
{
    const std::lock_guard<std::recursive_mutex> lock(lock_mutex);
    actual_command = "ACKNOWLEDGE";
    msg.doing_command = actual_command;
    //Acknowledge
    acknowledge();

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
    handshake = gripperBitInput(COMMAND_RECEIVED_TOGGLE);
    last_command = 0;
   // gripper_updater.force_update();
    finishedCommand();
}
//Stop service callback
void SchunkGripperNode::stop_srv(const std::shared_ptr<schunk_gripper::srv::Stop::Request>, std::shared_ptr<schunk_gripper::srv::Stop::Response> res)
{
    const std::lock_guard<std::recursive_mutex> lock(lock_mutex);
    //send stop
    runPost(STOP);
    //if command received, get values
    if(handshake != gripperBitInput(COMMAND_RECEIVED_TOGGLE)) 
    while (rclcpp::ok() && check() && !gripperBitInput(SUCCESS))   
    runGets();            

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
void SchunkGripperNode::fast_stop_srv(const std::shared_ptr<schunk_gripper::srv::FastStop::Request>, std::shared_ptr<schunk_gripper::srv::FastStop::Response> res)
{
    const std::lock_guard<std::recursive_mutex> lock(lock_mutex);
    //send fast stop
    runPost(FAST_STOP);
    //if command received, get values
    if(handshake != gripperBitInput(COMMAND_RECEIVED_TOGGLE)) 
    while (rclcpp::ok() && check())
    runGets(); 

    if(gripperBitInput(GRIPPER_ERROR))
    {
        res->stopped = 1;
        RCLCPP_WARN(nd->get_logger(),"FAST STOPPED");
    }
    else
    {
        res->stopped = 0;
        RCLCPP_WARN(nd->get_logger(),"FAST STOP NOT SUCCEDED");
    }

    handshake = gripperBitInput(COMMAND_RECEIVED_TOGGLE);
 //   gripper_updater.force_update();
}
//Softreset service callback 
void SchunkGripperNode::softreset_srv(const std::shared_ptr<schunk_gripper::srv::Softreset::Request>, std::shared_ptr<schunk_gripper::srv::Softreset::Response>)
{  
//    publish_state_timer->stop();
//    publish_joint_timer->stop();

    std::unique_lock<std::recursive_mutex> lock(lock_mutex);

    RCLCPP_INFO(nd->get_logger(),"SOFTRESET");
    updatePlcOutput(SOFTRESET);
    postCommand();
    RCLCPP_INFO(nd->get_logger(),"Server available in 7 seconds.");
    std::chrono::seconds sleep(7);
    std::this_thread::sleep_for(sleep);
    runGets();

    if(!check())                        RCLCPP_INFO(nd->get_logger(),"Softreset succeded");
    else                                RCLCPP_WARN(nd->get_logger(),"SOFTRESET NOT SUCCEDED");
    
    handshake = gripperBitInput(COMMAND_RECEIVED_TOGGLE);

//    gripper_updater.force_update();
    
    lock.unlock();

//    publish_joint_timer.start();
//    publish_state_timer.start();
}
//Prepare for shutdown service callback
void SchunkGripperNode::prepare_for_shutdown_srv(const std::shared_ptr<schunk_gripper::srv::PrepareForShutdown::Request>, std::shared_ptr<schunk_gripper::srv::PrepareForShutdown::Response> res)
{   
    const std::lock_guard<std::recursive_mutex> lock(lock_mutex);
    //send prepare for shutdown
    runPost(PREPARE_FOR_SHUTDOWN);
    //if command received, get values
    if(handshake != gripperBitInput(COMMAND_RECEIVED_TOGGLE))
    while (rclcpp::ok() && check() && !gripperBitInput(READY_FOR_SHUTDOWN))
    runGets();

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
//    gripper_updater.force_update();
}
//Release for manual movement service callback
void SchunkGripperNode::releaseForManualMov_srv(const std::shared_ptr<schunk_gripper::srv::ReleaseForManMov::Request>, std::shared_ptr<schunk_gripper::srv::ReleaseForManMov::Response> res)
{   
    const std::lock_guard<std::recursive_mutex> lock(lock_mutex);
    //If no Errors
    if(splitted_Diagnosis[2] == 0) 
    {   
        //Send fast stop
        runPost(FAST_STOP); 
        //if command received, get values 
        if(handshake != gripperBitInput(COMMAND_RECEIVED_TOGGLE)) 
        while (rclcpp::ok() && check())
        runGets();

     //   gripper_updater.force_update();
        RCLCPP_WARN(nd->get_logger(),"An error was provoked so that you can take the workpiece: %s\n", getErrorString(splitted_Diagnosis[2]).c_str());
        handshake = gripperBitInput(COMMAND_RECEIVED_TOGGLE);
    }
    //send Emergancy release
    runPost(EMERGENCY_RELEASE);
    //if command received, get values
    if(handshake != gripperBitInput(COMMAND_RECEIVED_TOGGLE)) 
    while(rclcpp::ok() && !gripperBitInput(EMERGENCY_RELEASED))
    runGets();

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
 //   gripper_updater.force_update();
}
//Get infos of the gripper service
void SchunkGripperNode::info_srv(const std::shared_ptr<schunk_gripper::srv::GripperInfo::Request>, std::shared_ptr<schunk_gripper::srv::GripperInfo::Response>)
{  
    const std::lock_guard<std::recursive_mutex> lock(lock_mutex);

    RCLCPP_INFO_STREAM(nd->get_logger(),"\n\n\nGRIPPERTYPE: " << model.c_str()
                    << "\nIP: " << ip  << std::endl);
    
    receiveWithOffset("40", 1, 1);
    RCLCPP_INFO_STREAM(nd->get_logger(),"\nNet mass of the Gripper: " << savedata[0] << " kg" << std::endl);
    
    receiveWithOffset("41", 1,6);
    RCLCPP_INFO_STREAM(nd->get_logger(),"\nTool center point 6D-Frame: \n" 
    << savedata[0] << " mm  " << savedata[1] << " mm  " << savedata[2] << " mm\n" 
    << savedata[3] << "  " << savedata[4] << "  " << savedata[5] << std::endl);
    
    receiveWithOffset("42", 1,6);
    RCLCPP_INFO_STREAM(nd->get_logger(), "\nCenter of Mass 6D-frame: \n" 
    << savedata[0] << " mm  " << savedata[1] << " mm  " << savedata[2] << " mm\n"
    << savedata[3] << " kg*m^2  " << savedata[4] << " kg*m^2  "<< savedata[5] << " kg*m^2"<< std::endl);

    RCLCPP_INFO_STREAM(nd->get_logger(),"\nMin. absolute position: " << min_pos << " mm\n" 
    << "Max. absolute position: " << max_pos << " mm\n"
    << "Zero_pos_offset: " << zero_pos_ofs << " mm" << std::endl);

    RCLCPP_INFO_STREAM(nd->get_logger(),"\nMin. velocity: " << min_vel << " mm/s\n"
    << "Max. velocity: " << max_vel << " mm/s" <<std::endl);
    receiveWithOffset("85", 1, 1);
    RCLCPP_INFO_STREAM(nd->get_logger(),"\nMax. grip velocity: "<< savedata[0] << " mm/s\n" 
    << "Min. grip force: "   << min_grip_force << " N\n" 
    << "Max. grip force: "   << max_grip_force << " N" << std::endl);

    if(model.find("_M_") != std::string::npos && model.find("EGU") != std::string::npos)
    {
    receiveWithOffset("93", 1, 1);
    RCLCPP_INFO_STREAM(nd->get_logger(),"\nMax allowed grip force StrongGrip: " << savedata[0]  << " N  " << std::endl);
    }

    receiveWithOffset("11", 1, 1);
    RCLCPP_INFO_STREAM(nd->get_logger(),"\nUsed current limit: " << savedata[0] << " A" << std::endl);
    receiveWithOffset("70", 1, 1);
    RCLCPP_INFO_STREAM(nd->get_logger(),"Max. physical stroke: " << savedata[0] << " mm" << std::endl);
    receiveWithOffset("96", 6, 6);
    RCLCPP_INFO_STREAM(nd->get_logger(),"\nMin. error motor voltage: " << savedata[0] << " V\n"
                <<    "Max. error motor voltage: " << savedata[1] << " V\n"
                <<    "Min. error logic voltage: " << savedata[2] << " V\n"
                <<    "Max. error logic voltage: " << savedata[3] << " V\n"
                <<    "Min. error logic temperature: " << savedata[4] << " C\n"
                <<    "Max. error logic temperature: " << savedata[5] << " C" << std::endl);
    
    receiveWithOffset("112", 6, 6);
    RCLCPP_INFO_STREAM(nd->get_logger(),"\nMin. warning motor voltage: " << savedata[0] << " V\n"
                <<    "Max. warning motor voltage: " << savedata[1] << " V\n"
                <<    "Min. warning logic voltage: " << savedata[2] << " V\n"
                <<    "Max. warning logic voltage: " << savedata[3] << " V\n"
                <<    "Min. warning logic temperature: " << savedata[4] << " C\n"
                <<    "Max. warning logic temperature: " << savedata[5] << " C" << std::endl);
    uint32_t uptime;
    getWithInstance("0x1400", &uptime);
    RCLCPP_INFO_STREAM(nd->get_logger(),"\nSystem uptime: " << uptime << " s" << std::endl);
}
//Reconfigure Parameter callback
void SchunkGripperNode::parameterEventCallback(const rcl_interfaces::msg::ParameterEvent::SharedPtr param)
{  
    std::vector<rcl_interfaces::msg::Parameter> sorted_parameters = param->changed_parameters;
    std::vector<rcl_interfaces::msg::Parameter> iterate_prarameters = param->changed_parameters;
    if(!param->new_parameters.empty())
    { 
        sorted_parameters = param->new_parameters;
        iterate_prarameters = param->new_parameters;
    }
    std::sort(sorted_parameters.begin(), sorted_parameters.end(),
    [](const rcl_interfaces::msg::Parameter& a, const rcl_interfaces::msg::Parameter& b) {
    return a.name < b.name;});

    size_t iter = 0;

   for(auto &controlparam : iterate_prarameters)
   {    
        if(controlparam.name.substr(0, 17) != "Control_Parameter") break;

        if(controlparam.name == "Control_Parameter.move_gripper" && param_exe == false && controlparam.value.double_value != abs_pos_param) 
        {
            const std::lock_guard<std::recursive_mutex> lock(lock_mutex);
            actual_command = "MOVE TO ABSOLUTE POSITON";
            runPost(MOVE_TO_ABSOLUTE_POSITION, static_cast<uint32_t>(nd->get_parameter("Control_Parameter.move_gripper").as_double()*1000) , static_cast<uint32_t>(nd->get_parameter("Control_Parameter.move_gripper_velocity").as_double()*1000));
            abs_pos_param = nd->get_parameter("Control_Parameter.move_gripper").as_double();
            param_exe = true;
            handshake = gripperBitInput(COMMAND_RECEIVED_TOGGLE);
        }
        else if(controlparam.name == "Control_Parameter.grip" && controlparam.value.bool_value == true && param_exe == false) 
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
        else if(controlparam.name == "Control_Parameter.release_workpiece"  && controlparam.value.bool_value == true && param_exe == false) 
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
            else RCLCPP_INFO(nd->get_logger(),"No Workpiece Gripped");
        }
        iter++;
   }

   for(auto fparam = iterate_prarameters.begin() + iter; fparam != iterate_prarameters.end(); ++fparam)
   {
        switch(fparam->value.type)
        {
        case rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE:
                lock_mutex.lock();
                changeParameter(parameter_map.at(fparam->name), static_cast<float>(fparam->value.double_value));
                RCLCPP_INFO_STREAM(nd->get_logger(), fparam->name << " changed to: " << fparam->value.double_value);
                if(fparam->name == "Gripper_Parameter.zero_pos_ofs")
                {
                    getWithInstance<float>(MAX_POS_INST, &max_pos);
                    getWithInstance<float>(MIN_POS_INST,&min_pos);
                    nd->undeclare_parameter("Control_Parameter.move_gripper");
                    nd->declare_parameter("Control_Parameter.move_gripper", actualPosInterval(), parameter_descriptor("Moving the gripper with parameter in mm", FloatingPointRange(min_pos,max_pos)));
                    abs_pos_param = nd->get_parameter("Control_Parameter.move_gripper").as_double();
                }
                lock_mutex.unlock();
                break;

        case rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER:
                lock_mutex.lock();
                changeParameter(GRP_PREHOLD_TIME_INST, static_cast<uint16_t>(fparam->value.integer_value), &grp_prehold_time);
                RCLCPP_INFO_STREAM(nd->get_logger(), fparam->name << " changed to: " << fparam->value.integer_value);
                lock_mutex.unlock();
                break;

        case rcl_interfaces::msg::ParameterType::PARAMETER_BOOL:
                lock_mutex.lock();
                runPost(USE_GPE);
                grp_pos_lock = gripperBitOutput(USE_GPE);
                RCLCPP_INFO(nd->get_logger(),"The brake is %s.", grp_pos_lock? "on" : "off");
                lock_mutex.unlock();
                break;
        }

    }
}
/*
//Diagnostics 
void SchunkGripperNode::gripperDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& status)
{   
    splitted_Diagnosis = splitDiagnosis();

    if(gripperBitInput(EMERGENCY_RELEASED)) 
    status.summary(diagnostic_msgs::DiagnosticStatus::WARN, "If you want to end this mode, perform a fast stop and acknowledge!");
    
    else if(gripperBitInput(READY_FOR_SHUTDOWN)) 
    status.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Ready for shutdown!");
    //If an error or waning occured
    else if(plc_sync_input[3] != 0)
    {  //if error
        if(splitted_Diagnosis[2])
        {    
            if(splitted_Diagnosis != old_diagnosis) 
            {
                const std::lock_guard<std::recursive_mutex> lock(lock_mutex);
                error_str = getErrorString(splitted_Diagnosis[2]);
                ROS_ERROR("%i\n%s",splitted_Diagnosis[2] , error_str.c_str());
            }
            status.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Error: %i\n%s" ,splitted_Diagnosis[2] ,error_str.c_str());
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
            status.summary(diagnostic_msgs::DiagnosticStatus::WARN, warn_str +" "+ std::to_string(splitted_Diagnosis[0]));
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
            status.summaryf(diagnostic_msgs::DiagnosticStatus::WARN, "Warning: %i\n%s",splitted_Diagnosis[1], warn_str.c_str());
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
        status.summary(diagnostic_msgs::DiagnosticStatus::OK, error_str);
    }

    old_diagnosis = splitted_Diagnosis;

    status.add("Ready for operation", gripperBitInput(READY_FOR_OPERATION));
    status.add("Command successfully proccesed", gripperBitInput(SUCCESS));
    status.add("position reached", gripperBitInput(POSITION_REACHED));
    status.add("Command received toggle", gripperBitInput(COMMAND_RECEIVED_TOGGLE));
    
    status.add("Error", gripperBitInput(GRIPPER_ERROR));
    status.add("Warning", gripperBitInput(WARNING));
    status.add("Not feasible", gripperBitInput(NOT_FEASIBLE));

    status.add("Pre-grip started",gripperBitInput(PRE_HOLDING));
    status.add("Workpiece gripped", gripperBitInput(GRIPPED));
    status.add("No workpiece detected", gripperBitInput(NO_WORKPIECE_DETECTED));
    status.add("Wrong workpiece gripped", gripperBitInput(WRONG_WORKPIECE_DETECTET));
    status.add("Workpiece lost", gripperBitInput(WORK_PIECE_LOST));

    status.add("Control authority", gripperBitInput(CONTROL_AUTHORITY));
    status.add("Software limit reached", gripperBitInput(SOFTWARE_LIMIT));
    status.add("Released for manaula movement", gripperBitInput(EMERGENCY_RELEASED));
    status.add("Grip force and position mainenance", gripperBitInput(BRAKE_ACTIVE));
    status.add("Ready for shutdown", gripperBitInput(READY_FOR_SHUTDOWN));

};

SchunkGripperNode::~SchunkGripperNode()
{ };
*/
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<rclcpp::Node>("schunk_gripper_driver");
    
    rcl_interfaces::msg::ParameterDescriptor param_des;
    
    param_des.read_only = true;
    param_des.description = "Ip adress of the gripper";
    node->declare_parameter("IP","0.0.0.0",param_des);
    param_des.description = "State publish rate";
    node->declare_parameter("state_frq", 60.0 ,param_des);
    param_des.description = "Jointstates publish rate";
    node->declare_parameter("rate", 10.0,param_des);
    
    std::string ip;
    float state_frq;
    float rate;

    node->get_parameter("IP" ,ip);
    node->get_parameter("state_frq",state_frq);
    node->get_parameter("rate",rate);

    auto schunkgrippernode = std::make_shared<SchunkGripperNode>(node,ip, state_frq, rate);
    
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"%s INITIALISED!", schunkgrippernode->model.c_str());

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
/*
    std::string ip;
    float state_frq;
    float rate;  

    if(ros::param::get("/schunk_gripper_driver/IP", ip)) RCLCPP_INFO(nd->get_logger(),"IP: %s",ip.c_str());
    else
    {
        ROS_ERROR("IP Parameter not retrieved!");
        ros::shutdown();
        return -1;
    }
    state_frq = nd.param("/schunk_gripper_driver/state_frq", 60.0);
    RCLCPP_INFO(nd->get_logger(),"state_frq: %f", state_frq);

    rate = nd.param("/rate", 10.0);
    RCLCPP_INFO(nd->get_logger(),"rate: %f",rate);
*/  
    //SchunkGripperNode *schunkgrippernode = new SchunkGripperNode(&nd, ip, state_frq, rate);
    
    
    rclcpp::shutdown();
    //delete schunkgrippernode;

    return 0;
}
