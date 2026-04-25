#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include<vector>
#include<kinova_wrapper/KinovaInterface.hpp>
#include<memory>
#include <sensor_msgs/msg/joint_state.hpp>
#include<std_srvs/srv/trigger.hpp>


using FollowJointTrajectory= control_msgs::action::FollowJointTrajectory;
using JointTrajectory=trajectory_msgs::msg::JointTrajectory;
using GoalHandle=rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

class KinovaMoveitBridge:public rclcpp::Node{
    public:
        explicit KinovaMoveitBridge(
            const rclcpp::NodeOptions& options,
        std::shared_ptr<kinova_wrapper::KinovaInterface> kinova);
  
    private:
        
        rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_;

        //3 Callbacks
        rclcpp_action::GoalResponse handle_goal(
            const rclcpp_action::GoalUUID& uuid,
            std::shared_ptr<const FollowJointTrajectory::Goal> goal);
        
        rclcpp_action::CancelResponse handle_cancel(
            std::shared_ptr<GoalHandle> goal_handle);  //goal_nadle object

        void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);    
        
        void execute(const std::shared_ptr<GoalHandle> goal_handle);
        
        std::vector<kinova_wrapper::TrajectoryPoint> convertTrajectory(
            const JointTrajectory& traj);

        std::shared_ptr<kinova_wrapper::KinovaInterface> kinova_;

        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_faults_service_;
    
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
        rclcpp::TimerBase::SharedPtr joint_state_timer_;

        std::jthread execute_thread_;
        
        void publishJointStates();
        void handleClearFaults(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                std::shared_ptr<std_srvs::srv::Trigger::Response> response);    
    };