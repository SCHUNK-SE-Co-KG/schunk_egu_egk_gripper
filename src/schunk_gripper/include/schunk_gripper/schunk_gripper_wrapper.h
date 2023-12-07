#ifndef SCHUNK_GRIPPER_WRAPPER_H
#define SCHUNK_GRIPPER_WRAPPER_H

#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include <rclcpp/parameter.hpp>
#include <rcl_interfaces/msg/parameter_event.hpp>
#include "schunk_gripper/schunk_gripper_lib.h"
//#include <actionlib/server/simple_action_server.h>
//#include <dynamic_reconfigure/server.h>
   //           #include "sensor_msgs/msg/JointState.hpp"
//#include "diagnostic_updater/diagnostic_updater.h

#include "schunk_gripper/action/grip_egk.hpp"
#include "schunk_gripper/action/grip_with_pos_egk.hpp"
#include "schunk_gripper/action/mov_abs_pos.hpp"
#include "schunk_gripper/action/mov_rel_pos.hpp"
#include "schunk_gripper/msg/state.hpp"
#include "schunk_gripper/srv/acknowledge.hpp"
#include "schunk_gripper/srv/stop.hpp"
#include "schunk_gripper/srv/fast_stop.hpp"
//#include "schunk_gripper//gripper_parameterConfig.hpp"
#include "schunk_gripper/srv/prepare_for_shutdown.hpp"
#include "schunk_gripper/srv/softreset.hpp"
#include "schunk_gripper/action/release_workpiece.hpp"
#include "schunk_gripper/srv/release_for_man_mov.hpp"
#include "schunk_gripper/srv/gripper_info.hpp"
//           #include "control_msgs/action/GripperCommand.hpp"
#include "schunk_gripper/action/grip_egu.hpp"
#include "schunk_gripper/action/grip_with_pos_egu.hpp"

//typedef dynamic_reconfigure::Server<schunk_gripper::gripper_parameterConfig>    ReconfigureServer;

std::recursive_mutex mutex;
std::recursive_mutex lock_mutex;  //Locks if something is receiving or posting data

class SchunkGripperNode : public Gripper
{
    private:
    void publishState();
    void parameterEventCallback(const rcl_interfaces::msg::ParameterEvent::SharedPtr);
  //  void publishJointState();

    void acknowledge_srv(const std::shared_ptr<schunk_gripper::srv::Acknowledge::Request>, std::shared_ptr<schunk_gripper::srv::Acknowledge::Response> );
    void stop_srv(const std::shared_ptr<schunk_gripper::srv::Stop::Request>, std::shared_ptr<schunk_gripper::srv::Stop::Response> );
    void fast_stop_srv(const std::shared_ptr<schunk_gripper::srv::FastStop::Request>, std::shared_ptr<schunk_gripper::srv::FastStop::Response>);
    void releaseForManualMov_srv(const std::shared_ptr<schunk_gripper::srv::ReleaseForManMov::Request>, std::shared_ptr<schunk_gripper::srv::ReleaseForManMov::Response>);
    void softreset_srv(const std::shared_ptr<schunk_gripper::srv::Softreset::Request>, std::shared_ptr<schunk_gripper::srv::Softreset::Response>);
    void prepare_for_shutdown_srv(const std::shared_ptr<schunk_gripper::srv::PrepareForShutdown::Request>, std::shared_ptr<schunk_gripper::srv::PrepareForShutdown::Response>);
    void info_srv(const std::shared_ptr<schunk_gripper::srv::GripperInfo::Request>, std::shared_ptr<schunk_gripper::srv::GripperInfo::Response>);

   // rclcpp_action::Server<control_msgs::action::GripperCommand>                      control_server;

   // rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr     jointStatePublisher;
    rclcpp::Publisher<schunk_gripper::msg::State>::SharedPtr       statePublisher;
    rclcpp::TimerBase::SharedPtr         publish_state_timer;
//    rclcpp::TimerBase::SharedPtr         publish_joint_timer;

    rclcpp::Time          diagnostic_time;
    std::shared_ptr<rclcpp::Node>       nd;

   // diagnostic_updater::Updater gripper_updater;

    schunk_gripper::msg::State msg;
    float abs_pos_param;

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

    rclcpp::Parameter params;

    std::array<uint8_t, 3> splitted_Diagnosis;

  //  template<typename actiontype>
  //  void exceptionHandling(actiontype &,  uint32_t &);
  //  template<typename feedtype, typename restype, typename servertype>
 //   void setFinalState(feedtype &, restype &, servertype &);
  //  template<typename servertype>
 //   bool canActionStart(servertype&);
 //   template<typename feedbacktype, typename servertype>
 //   void runActionMove(feedbacktype &, servertype &);
 //   template<typename feedbacktype, typename servertype>
 //   void runActionGrip(feedbacktype &, servertype &);
    void updateStateMsg();
    void declareParameter();
    //void gripperDiagnostics(diagnostic_updater::DiagnosticStatusWrapper&);
    void finishedCommand();

    rcl_interfaces::msg::ParameterDescriptor parameter_descriptor(const std::string&, const rcl_interfaces::msg::FloatingPointRange&);
    rcl_interfaces::msg::ParameterDescriptor parameter_descriptor(const std::string&, const rcl_interfaces::msg::IntegerRange&);
    rcl_interfaces::msg::ParameterDescriptor parameter_descriptor(const std::string&);
    rcl_interfaces::msg::FloatingPointRange FloatingPointRange(double, double, double step = 0.0);
    rcl_interfaces::msg::IntegerRange IntegerRange(int64_t, int64_t, uint64_t step = 0);


    public:
    rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event;

    SchunkGripperNode(std::shared_ptr<rclcpp::Node> nd,std::string ip, float state, float frq);
//    ~SchunkGripperNode();

    rclcpp_action::Server<schunk_gripper::action::MovAbsPos>::SharedPtr              move_abs_server;
    rclcpp_action::Server<schunk_gripper::action::MovRelPos>::SharedPtr              move_rel_server;
    rclcpp_action::Server<schunk_gripper::action::GripWithPosEgk>::SharedPtr         grip_w_pos_server;
    rclcpp_action::Server<schunk_gripper::action::GripWithPosEgu>::SharedPtr         grip_w_pos_egu_server;
    rclcpp_action::Server<schunk_gripper::action::GripEgk>::SharedPtr                grip_server;
    rclcpp_action::Server<schunk_gripper::action::GripEgu>::SharedPtr                grip_egu_server;
    rclcpp_action::Server<schunk_gripper::action::ReleaseWorkpiece>::SharedPtr       release_wp_server;


    rclcpp::Service<schunk_gripper::srv::Acknowledge>::SharedPtr            acknowledge_service;
    rclcpp::Service<schunk_gripper::srv::Stop>::SharedPtr                   stop_service;
    rclcpp::Service<schunk_gripper::srv::FastStop>::SharedPtr               fast_stop_service;
    rclcpp::Service<schunk_gripper::srv::ReleaseForManMov>::SharedPtr       releaseForManualMov_service;
    rclcpp::Service<schunk_gripper::srv::Softreset>::SharedPtr              softreset_service;
    rclcpp::Service<schunk_gripper::srv::PrepareForShutdown>::SharedPtr     prepare_for_shutdown_service;
    rclcpp::Service<schunk_gripper::srv::GripperInfo>::SharedPtr            info_service;
/*
    void moveAbsExecute(const schunk_gripper::mov_abs_posGoalConstPtr & );
    void moveRelExecute(const schunk_gripper::mov_rel_posGoalConstPtr & );
    void gripExecute(const schunk_gripper::gripGoalConstPtr & );
    void grip_eguExecute(const schunk_gripper::grip_eguGoalConstPtr & );
    void gripWithPositionExecute(const schunk_gripper::grip_with_posGoalConstPtr & );
    void gripWithPosition_eguExecute(const schunk_gripper::grip_with_pos_eguGoalConstPtr & );
    void releaseExecute(const schunk_gripper::release_workpieceGoalConstPtr &);

    void controlExecute(const control_msgs::GripperCommandGoalConstPtr &);
*/


};

#endif