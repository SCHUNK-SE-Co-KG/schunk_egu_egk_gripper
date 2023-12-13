#ifndef SCHUNK_GRIPPER_WRAPPER_HPP
#define SCHUNK_GRIPPER_WRAPPER_HPP

#include <rclcpp/rclcpp.hpp>
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

//typedef dynamic_reconfigure::Server<schunk_gripper::gripper_parameterConfig>    ReconfigureServer;

std::recursive_mutex mutex;
std::recursive_mutex lock_mutex;  //Locks if something is receiving or posting data
extern  std::map<std::string, const char*> parameter_map;

class SchunkGripperNode : public Gripper
{
    private:

    void publishState();
    void parameterEventCallback(const rcl_interfaces::msg::ParameterEvent::SharedPtr);

    void publishJointState();

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

    void acknowledge_srv(const std::shared_ptr<Acknowledge::Request>, std::shared_ptr<Acknowledge::Response> );
    void stop_srv(const std::shared_ptr<Stop::Request>, std::shared_ptr<Stop::Response> );
    void fast_stop_srv(const std::shared_ptr<FastStop::Request>, std::shared_ptr<FastStop::Response>);
    void releaseForManualMov_srv(const std::shared_ptr<ReleaseForManMov::Request>, std::shared_ptr<ReleaseForManMov::Response>);
    void softreset_srv(const std::shared_ptr<Softreset::Request>, std::shared_ptr<Softreset::Response>);
    void prepare_for_shutdown_srv(const std::shared_ptr<PrepareForShutdown::Request>, std::shared_ptr<PrepareForShutdown::Response>);
    void info_srv(const std::shared_ptr<GripperInfo::Request>, std::shared_ptr<GripperInfo::Response>);

    double actualPosInterval();                                                         //Parameter accept just Position in Interval

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr     jointStatePublisher;
    rclcpp::Publisher<schunk_gripper::msg::State>::SharedPtr       statePublisher;
    rclcpp::TimerBase::SharedPtr                                   publish_state_timer;
    rclcpp::TimerBase::SharedPtr                                   publish_joint_timer;

    rclcpp::Time                        diagnostic_time;
    std::shared_ptr<rclcpp::Node>       nd;

    std::shared_ptr<diagnostic_updater::Updater> gripper_updater;

    schunk_gripper::msg::State msg;
    double abs_pos_param;

    float state_frq;
    float j_state_frq;

    rclcpp::Duration cycletime;
    rclcpp::Rate     limiting_rate;

    std::string actual_command;     

    std::array<uint8_t,3> old_diagnosis; 
    std::string error_str;
    std::string warn_str;

    bool param_exe;
    bool action_active;
    bool action_move;
    bool zero_changed;

    std::array<uint8_t, 3> splitted_Diagnosis;

    template<typename GoalType, typename ResType>
    void exceptionHandling(const std::shared_ptr<rclcpp_action::ServerGoalHandle<GoalType>>, const int32_t &, const std::shared_ptr<ResType>);
    template<typename restype, typename goaltype>
    void setFinalState(std::shared_ptr<restype>, const std::shared_ptr<rclcpp_action::ServerGoalHandle<goaltype>>);
    template<typename feedbacktype, typename goaltype>
    void runActionMove(std::shared_ptr<feedbacktype>, std::shared_ptr<rclcpp_action::ServerGoalHandle<goaltype>>);
    template<typename feedbacktype, typename goaltype>
    void runActionGrip(std::shared_ptr<feedbacktype>, std::shared_ptr<rclcpp_action::ServerGoalHandle<goaltype>>);
    void updateStateMsg();
    void declareParameter();
    void gripperDiagnostics(diagnostic_updater::DiagnosticStatusWrapper&);
    void finishedCommand();

    rcl_interfaces::msg::ParameterDescriptor parameter_descriptor(const std::string&, const rcl_interfaces::msg::FloatingPointRange&);
    rcl_interfaces::msg::ParameterDescriptor parameter_descriptor(const std::string&, const rcl_interfaces::msg::IntegerRange&);
    rcl_interfaces::msg::ParameterDescriptor parameter_descriptor(const std::string&);
    rcl_interfaces::msg::FloatingPointRange FloatingPointRange(double, double, double step = 0.0);
    rcl_interfaces::msg::IntegerRange IntegerRange(int64_t, int64_t, uint64_t step = 0);


    public:
    rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event;
    rclcpp::AsyncParametersClient::SharedPtr parameters_client;

    rclcpp::CallbackGroup::SharedPtr messages_group;
    rclcpp::CallbackGroup::SharedPtr services_group;
    rclcpp::CallbackGroup::SharedPtr actions_group;
    rclcpp::CallbackGroup::SharedPtr rest;

    SchunkGripperNode(std::shared_ptr<rclcpp::Node> nd,std::string ip, float state, float frq);
    ~SchunkGripperNode();

    rclcpp_action::Server<MovAbsPos>::SharedPtr              move_abs_server;
    rclcpp_action::Server<MovRelPos>::SharedPtr              move_rel_server;
    rclcpp_action::Server<GripWithPosVel>::SharedPtr         grip_w_pos_server;
    rclcpp_action::Server<GripWithPos>::SharedPtr            grip_w_pos_egu_server;
    rclcpp_action::Server<GripWithVel>::SharedPtr            grip_server;
    rclcpp_action::Server<Grip>::SharedPtr                   grip_egu_server;
    rclcpp_action::Server<ReleaseWorkpiece>::SharedPtr       release_wp_server;
    rclcpp_action::Server<GripperCommand>::SharedPtr         control_server;


    rclcpp::Service<Acknowledge>::SharedPtr            acknowledge_service;
    rclcpp::Service<Stop>::SharedPtr                   stop_service;
    rclcpp::Service<FastStop>::SharedPtr               fast_stop_service;
    rclcpp::Service<ReleaseForManMov>::SharedPtr       releaseForManualMov_service;
    rclcpp::Service<Softreset>::SharedPtr              softreset_service;
    rclcpp::Service<PrepareForShutdown>::SharedPtr     prepare_for_shutdown_service;
    rclcpp::Service<GripperInfo>::SharedPtr            info_service;

    template<typename goaltype>
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const goaltype> goal);

    template<typename GoalType>
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<GoalType>> cancel);

    void handle_accepted_abs(const std::shared_ptr<rclcpp_action::ServerGoalHandle<MovAbsPos>>);
    void handle_accepted_rel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<MovRelPos>>);
    void handle_accepted_grip_egk(const std::shared_ptr<rclcpp_action::ServerGoalHandle<GripWithVel>>);
    void handle_accepted_grip_egu(const std::shared_ptr<rclcpp_action::ServerGoalHandle<Grip>>);
    void handle_accepted_gripPos_egk(const std::shared_ptr<rclcpp_action::ServerGoalHandle<GripWithPosVel>>);
    void handle_accepted_gripPos_egu(const std::shared_ptr<rclcpp_action::ServerGoalHandle<GripWithPos>>);
    void handle_accepted_release(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ReleaseWorkpiece>>);
    void handle_accepted_control(const std::shared_ptr<rclcpp_action::ServerGoalHandle<GripperCommand>>);

    void moveAbsExecute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<MovAbsPos>>);
    void moveRelExecute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<MovRelPos>>);
    void gripExecute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<GripWithVel>>);
    void grip_eguExecute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<Grip>>);
    void gripWithPositionExecute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<GripWithPosVel>>);
    void gripWithPosition_eguExecute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<GripWithPos>>);
    void releaseExecute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ReleaseWorkpiece>>);
    void controlExecute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<GripperCommand>>);



};

template<typename goaltype>
inline rclcpp_action::GoalResponse SchunkGripperNode::handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const goaltype> goal)
{
    (void)uuid;
    (void)goal;

    if(param_exe == false && action_active == false)
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;

    else
    return rclcpp_action::GoalResponse::REJECT;

}

template<typename GoalType>
inline rclcpp_action::CancelResponse SchunkGripperNode::handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<GoalType>> goal_handle)
{
        (void)goal_handle;

        RCLCPP_INFO(nd->get_logger(), "Received request to cancel goal");
        return rclcpp_action::CancelResponse::ACCEPT;
}

inline void SchunkGripperNode::handle_accepted_rel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<MovRelPos>> goal_handle)
{
      using namespace std::placeholders;
      std::thread{&SchunkGripperNode::moveRelExecute, this, goal_handle}.detach();
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
}
inline void SchunkGripperNode::handle_accepted_abs(const std::shared_ptr<rclcpp_action::ServerGoalHandle<MovAbsPos>> goal_handle)
{
      using namespace std::placeholders;
      std::thread{&SchunkGripperNode::moveAbsExecute, this, goal_handle}.detach();
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
}
inline void SchunkGripperNode::handle_accepted_grip_egk(const std::shared_ptr<rclcpp_action::ServerGoalHandle<GripWithVel>> goal_handle)
{
      using namespace std::placeholders;
      std::thread{&SchunkGripperNode::gripExecute, this, goal_handle}.detach();
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
}
inline void SchunkGripperNode::handle_accepted_grip_egu(const std::shared_ptr<rclcpp_action::ServerGoalHandle<Grip>> goal_handle)
{
      using namespace std::placeholders;
      std::thread{&SchunkGripperNode::grip_eguExecute, this, goal_handle}.detach();
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
}
inline void SchunkGripperNode::handle_accepted_gripPos_egk(const std::shared_ptr<rclcpp_action::ServerGoalHandle<GripWithPosVel>> goal_handle)
{
      using namespace std::placeholders;
      std::thread{&SchunkGripperNode::gripWithPositionExecute, this, goal_handle}.detach();
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
}
inline void SchunkGripperNode::handle_accepted_gripPos_egu(const std::shared_ptr<rclcpp_action::ServerGoalHandle<GripWithPos>> goal_handle)
{
      using namespace std::placeholders;
      std::thread{&SchunkGripperNode::gripWithPosition_eguExecute, this, goal_handle}.detach();
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
}
inline void SchunkGripperNode::handle_accepted_release(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ReleaseWorkpiece>> goal_handle)
{
      using namespace std::placeholders;
      std::thread{&SchunkGripperNode::releaseExecute, this, goal_handle}.detach();
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
}
inline void SchunkGripperNode::handle_accepted_control(const std::shared_ptr<rclcpp_action::ServerGoalHandle<GripperCommand>> goal_handle)
{
      using namespace std::placeholders;
      std::thread{&SchunkGripperNode::controlExecute, this, goal_handle}.detach();
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
}

#endif