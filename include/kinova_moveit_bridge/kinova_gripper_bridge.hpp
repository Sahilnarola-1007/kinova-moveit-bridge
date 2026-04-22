#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include<vector>
#include<kinova_wrapper/KinovaInterface.hpp>
#include<memory>

#include <control_msgs/action/gripper_command.hpp>

using GripperCommand = control_msgs::action::GripperCommand;
using GripperGoalHandle = rclcpp_action::ServerGoalHandle<GripperCommand>;
class KinovaGripperBridge:public rclcpp::Node{

    public:
        KinovaGripperBridge(const rclcpp::NodeOptions &options,
                            std::shared_ptr<kinova_wrapper::KinovaInterface> kinova);

    private:
        //3 Callbacks
        rclcpp_action::GoalResponse handle_goal(
            const rclcpp_action::GoalUUID& uuid,
            std::shared_ptr<const GripperCommand::Goal> goal);
        
        rclcpp_action::CancelResponse handle_cancel(
            std::shared_ptr<GripperGoalHandle> goal_handle);  //goal_nadle object

        void handle_accepted(const std::shared_ptr<GripperGoalHandle> goal_handle);    
        
        void execute(const std::shared_ptr<GripperGoalHandle> goal_handle);
        
        static constexpr double kKnuckleMaxRad = 0.8;  // robotiq_85 fully closed, per SRDF

        std::shared_ptr<kinova_wrapper::KinovaInterface> kinova_;

        rclcpp_action::Server<GripperCommand>::SharedPtr action_server_;
    };

