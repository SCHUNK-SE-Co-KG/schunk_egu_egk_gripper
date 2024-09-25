// Copyright 2024 SCHUNK SE & Co. KG
//
// This program is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by the Free
// Software Foundation, either version 3 of the License, or (at your option)
// any later version.
//
// This program is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
// FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
// more details.
//
// You should have received a copy of the GNU General Public License along with
// this program. If not, see <https://www.gnu.org/licenses/>.
// --------------------------------------------------------------------------------

/*
 * Author:        Viktoria Krimer (viktoria.krimer@de.schunk.com)
 * Maintainer:    Stefan Scherzinger (stefan.scherzinger@de.schunk.com)
 */

#ifndef SCHUNK_GRIPPER_WRAPPER_HPP
#define SCHUNK_GRIPPER_WRAPPER_HPP

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include <rclcpp/parameter.hpp>
#include <rcl_interfaces/msg/parameter_event.hpp>
#include "schunk_egu_egk_gripper_library/schunk_gripper_lib.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "control_msgs/action/gripper_command.hpp"
#include <diagnostic_updater/diagnostic_updater.hpp>
#include "std_srvs/srv/trigger.hpp"

#include "schunk_egu_egk_gripper_interfaces/action/grip_with_position.hpp"
#include "schunk_egu_egk_gripper_interfaces/action/grip_with_velocity.hpp"
#include "schunk_egu_egk_gripper_interfaces/action/grip_with_position_and_velocity.hpp"
#include "schunk_egu_egk_gripper_interfaces/action/move_to_absolute_position.hpp"
#include "schunk_egu_egk_gripper_interfaces/action/move_to_relative_position.hpp"
#include "schunk_egu_egk_gripper_interfaces/msg/state.hpp"
#include "schunk_egu_egk_gripper_interfaces/srv/acknowledge.hpp"
#include "schunk_egu_egk_gripper_interfaces/srv/brake_test.hpp"
#include "schunk_egu_egk_gripper_interfaces/srv/stop.hpp"
#include "schunk_egu_egk_gripper_interfaces/srv/fast_stop.hpp"
#include "schunk_egu_egk_gripper_interfaces/srv/prepare_for_shutdown.hpp"
#include "schunk_egu_egk_gripper_interfaces/srv/softreset.hpp"
#include "schunk_egu_egk_gripper_interfaces/action/release_workpiece.hpp"
#include "schunk_egu_egk_gripper_interfaces/srv/release_for_manual_movement.hpp"
#include "schunk_egu_egk_gripper_interfaces/srv/gripper_info.hpp"
#include "schunk_egu_egk_gripper_interfaces/action/grip.hpp"
#include "schunk_egu_egk_gripper_interfaces/srv/parameter_get.hpp"
#include "schunk_egu_egk_gripper_interfaces/srv/parameter_set.hpp"

extern  std::map<std::string, const char*> param_inst;
extern std::map<std::string, std::string> inst_param;

class SchunkGripperNode :  public rclcpp::Node, public Gripper
{
    private:

    std::mutex lock_service_post;
    using State = schunk_egu_egk_gripper_interfaces::msg::State;

    using Acknowledge = schunk_egu_egk_gripper_interfaces::srv::Acknowledge;
    using BrakeTest = schunk_egu_egk_gripper_interfaces::srv::BrakeTest;
    using Stop = schunk_egu_egk_gripper_interfaces::srv::Stop;
    using FastStop =  schunk_egu_egk_gripper_interfaces::srv::FastStop;
    using ReleaseForManualMovement = schunk_egu_egk_gripper_interfaces::srv::ReleaseForManualMovement;
    using Softreset = schunk_egu_egk_gripper_interfaces::srv::Softreset;
    using PrepareForShutdown = schunk_egu_egk_gripper_interfaces::srv::PrepareForShutdown;
    using GripperInfo= schunk_egu_egk_gripper_interfaces::srv::GripperInfo;
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

    //Flags
    bool doing_something;
    bool action_active;
    bool wrong_version;
    //Topic publishing
    void publishJointState();
    void publishState();

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr     jointStatePublisher;
    rclcpp::Publisher<schunk_egu_egk_gripper_interfaces::msg::State>::SharedPtr       statePublisher;
    rclcpp::TimerBase::SharedPtr                                   publish_state_timer;
    rclcpp::TimerBase::SharedPtr                                   publish_joint_timer;

    schunk_egu_egk_gripper_interfaces::msg::State msg;
    double state_frq;
    double j_state_frq;
    //Diagnostic updater
    std::shared_ptr<diagnostic_updater::Updater> gripper_updater;

    std::array<uint8_t,3> old_diagnosis;
    std::string error_str;
    std::string warn_str;

    std::array<uint8_t, 3> splitted_Diagnosis;
    std::string connection_error;
    std::vector<rclcpp::Parameter> failed_param;
    rclcpp::Time start_time;

    void gripperDiagnostics(diagnostic_updater::DiagnosticStatusWrapper&);

    double actualPosInterval();                                                         //Parameter accept just Position in Interval
    void callback_gripper_parameter(const rclcpp::Parameter &);
    void callback_move_parameter(const rclcpp::Parameter &);

    double abs_pos_param;
    std::string actual_command;
    std::shared_ptr<rclcpp::Duration> cycletime;
    rclcpp::Time     last_time;
    rclcpp::Rate     limiting_rate;
    //Basic Functions
    void publishStateMsg();
    void finishedCommand();
    void declareParameter();
    void advertiseServices();
    void advertiseActions();
    void advertiseTopics();
    void advertiseConnectionRelevant();
    void checkVersions();
    //Describing Parameters
    rcl_interfaces::msg::ParameterDescriptor parameter_descriptor(const std::string&, const rcl_interfaces::msg::FloatingPointRange&);
    rcl_interfaces::msg::ParameterDescriptor parameter_descriptor(const std::string&, const rcl_interfaces::msg::IntegerRange&);
    rcl_interfaces::msg::ParameterDescriptor parameter_descriptor(const std::string&);
    rcl_interfaces::msg::FloatingPointRange FloatingPointRange(double, double, double step = 0.0);
    rcl_interfaces::msg::IntegerRange IntegerRange(int64_t, int64_t, uint64_t step = 0);

    //Services
    void acknowledge_srv(const std::shared_ptr<Acknowledge::Request>, std::shared_ptr<Acknowledge::Response> );
    void brake_test_srv(const std::shared_ptr<BrakeTest::Request>, std::shared_ptr<BrakeTest::Response> );
    void stop_srv(const std::shared_ptr<Stop::Request>, std::shared_ptr<Stop::Response> );
    void fast_stop_srv(const std::shared_ptr<FastStop::Request>, std::shared_ptr<FastStop::Response>);
    void parameter_get_srv(const std::shared_ptr<ParameterGet::Request>, std::shared_ptr<ParameterGet::Response>);
    void parameter_set_srv(const std::shared_ptr<ParameterSet::Request>, std::shared_ptr<ParameterSet::Response>);
    void releaseForManualMov_srv(const std::shared_ptr<ReleaseForManualMovement::Request>, std::shared_ptr<ReleaseForManualMovement::Response>);
    void softreset_srv(const std::shared_ptr<Softreset::Request>, std::shared_ptr<Softreset::Response>);
    void prepare_for_shutdown_srv(const std::shared_ptr<PrepareForShutdown::Request>, std::shared_ptr<PrepareForShutdown::Response>);
    void info_srv(const std::shared_ptr<GripperInfo::Request>, std::shared_ptr<GripperInfo::Response> res);
    void grip_srv(const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> res);
    void release_srv(const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> res);

    //Action-basic-functions
    template<typename GoalType, typename ResType>
    void exceptionHandling(const std::shared_ptr<rclcpp_action::ServerGoalHandle<GoalType>>, const int32_t &, const std::shared_ptr<ResType>);
    template<typename restype, typename goaltype>
    void setFinalState(std::shared_ptr<restype>, const std::shared_ptr<rclcpp_action::ServerGoalHandle<goaltype>>);
    template<typename feedbacktype, typename goaltype>
    void runActionMove(std::shared_ptr<feedbacktype>, std::shared_ptr<rclcpp_action::ServerGoalHandle<goaltype>>);
    template<typename feedbacktype, typename goaltype>
    void runActionGrip(std::shared_ptr<feedbacktype>, std::shared_ptr<rclcpp_action::ServerGoalHandle<goaltype>>);

    void reconnect();
    //Actions
    void handle_accepted_abs(const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveToAbsolutePosition>>);
    void handle_accepted_rel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveToRelativePosition>>);
    void handle_accepted_grip_egk(const std::shared_ptr<rclcpp_action::ServerGoalHandle<GripWithVelocity>>);
    void handle_accepted_grip_egu(const std::shared_ptr<rclcpp_action::ServerGoalHandle<Grip>>);
    void handle_accepted_gripPos_egk(const std::shared_ptr<rclcpp_action::ServerGoalHandle<GripWithPositionAndVelocity>>);
    void handle_accepted_gripPos_egu(const std::shared_ptr<rclcpp_action::ServerGoalHandle<GripWithPosition>>);
    void handle_accepted_release(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ReleaseWorkpiece>>);
    void handle_accepted_control(const std::shared_ptr<rclcpp_action::ServerGoalHandle<GripperCommand>>);

    void moveAbsExecute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveToAbsolutePosition>>);
    void moveRelExecute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveToRelativePosition>>);
    void gripExecute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<GripWithVelocity>>);
    void grip_eguExecute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<Grip>>);
    void gripWithPositionExecute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<GripWithPositionAndVelocity>>);
    void gripWithPosition_eguExecute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<GripWithPosition>>);
    void releaseExecute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ReleaseWorkpiece>>);
    void controlExecute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<GripperCommand>>);

    template<typename goaltype>
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const goaltype> goal);

    template<typename GoalType>
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<GoalType>> cancel);

    //Parameter
    std::shared_ptr<rclcpp::ParameterEventHandler> parameter_event_handler;
    std::array<std::shared_ptr<rclcpp::ParameterCallbackHandle>, 12> cb_handle;

    std::function<void(const rclcpp::Parameter &)> callback_gripper_param;
    std::function<void(const rclcpp::Parameter &)> callback_move_param;
    //Callback groups
    rclcpp::CallbackGroup::SharedPtr messages_group;
    rclcpp::CallbackGroup::SharedPtr services_group;
    rclcpp::CallbackGroup::SharedPtr actions_group;

    public:

    SchunkGripperNode(const rclcpp::NodeOptions&);
    ~SchunkGripperNode();

    rclcpp_action::Server<MoveToAbsolutePosition>::SharedPtr              move_abs_server;
    rclcpp_action::Server<MoveToRelativePosition>::SharedPtr              move_rel_server;
    rclcpp_action::Server<GripWithPositionAndVelocity>::SharedPtr         grip_w_pos_server;
    rclcpp_action::Server<GripWithPosition>::SharedPtr            grip_w_pos_egu_server;
    rclcpp_action::Server<GripWithVelocity>::SharedPtr            grip_server;
    rclcpp_action::Server<Grip>::SharedPtr                   grip_egu_server;
    rclcpp_action::Server<ReleaseWorkpiece>::SharedPtr       release_wp_server;
    rclcpp_action::Server<GripperCommand>::SharedPtr         control_server;

    rclcpp::Service<ParameterGet>::SharedPtr           parameter_get_service;
    rclcpp::Service<ParameterSet>::SharedPtr           parameter_set_service;
    rclcpp::Service<Acknowledge>::SharedPtr            acknowledge_service;
    rclcpp::Service<BrakeTest>::SharedPtr              brake_test_service;
    rclcpp::Service<Stop>::SharedPtr                   stop_service;
    rclcpp::Service<FastStop>::SharedPtr               fast_stop_service;
    rclcpp::Service<ReleaseForManualMovement>::SharedPtr       releaseForManualMov_service;
    rclcpp::Service<Softreset>::SharedPtr              softreset_service;
    rclcpp::Service<PrepareForShutdown>::SharedPtr     prepare_for_shutdown_service;
    rclcpp::Service<GripperInfo>::SharedPtr            info_service;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr grip_service;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr release_service;
};

#endif
