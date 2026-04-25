#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include<vector>
#include<kinova_moveit_bridge/kinova_moveit_bridge_node.hpp>


#include<thread>
#include<iostream>


using namespace std::placeholders;

KinovaMoveitBridge::KinovaMoveitBridge(const rclcpp::NodeOptions &options,
                                    std::shared_ptr<kinova_wrapper::KinovaInterface> kinova)
                    :rclcpp::Node("kinova_moveit_bridge",options),
                    kinova_(std::move(kinova))
                    {
                        
                        this->declare_parameter("robot_ip","192.168.1.10");
                        const std::string ip=this->get_parameter("robot_ip").as_string();

                        // TODO: Publish /robot_status topic for external visibility of connection state.
                        // TODO: Move connect() to async init or lifecycle on_activate() so node startup
                        // is not blocked by slow network calls.
                        bool connection=kinova_->connect(ip);
                        if(!connection){
                            RCLCPP_ERROR(get_logger(),"failed to connect with IP: %s",ip.c_str());
                        }

                        else {
                            kinova_->clearEmergencyStop();  // clear any leftover faults from previous session
                        }

                        // Action server intitialization with 3 callbacks
                        action_server_=rclcpp_action::create_server<FollowJointTrajectory>(
                            this,
                            "/kinova_moveit_bridge/follow_joint_trajectory",
                            std::bind(&KinovaMoveitBridge::handle_goal,this,_1,_2),
                            std::bind(&KinovaMoveitBridge::handle_cancel,this,_1),
                            std::bind(&KinovaMoveitBridge::handle_accepted,this,_1)
                        );
                
                    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

                    joint_state_timer_ = this->create_wall_timer(
                    std::chrono::milliseconds(50),
                    std::bind(&KinovaMoveitBridge::publishJointStates, this));
        
                    clear_faults_service_ = this->create_service<std_srvs::srv::Trigger>(
                    "/kinova_moveit_bridge/clear_faults",
                    std::bind(&KinovaMoveitBridge::handleClearFaults, this,
                    std::placeholders::_1, std::placeholders::_2));
                
        
                    }

rclcpp_action::GoalResponse KinovaMoveitBridge::handle_goal(
            const rclcpp_action::GoalUUID& uuid,
            std::shared_ptr<const FollowJointTrajectory::Goal> goal){
             
                (void)uuid;

                if(!kinova_->isConnected() || kinova_->isEStopActive()){
                    return rclcpp_action::GoalResponse::REJECT;
                }
                
                if (goal->trajectory.joint_names.empty() || goal->trajectory.points.empty())
                 {
                    return rclcpp_action::GoalResponse::REJECT;
                 }

                 return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;


            }

rclcpp_action::CancelResponse KinovaMoveitBridge::handle_cancel(
            const std::shared_ptr<GoalHandle> goal_handle){
                
                kinova_->emergencyStop();  // Stops all the current work
                (void)goal_handle;
                return rclcpp_action::CancelResponse::ACCEPT;
            }

void KinovaMoveitBridge::handle_accepted(const std::shared_ptr<GoalHandle> goal_handle) {
    if (execute_thread_.joinable()) {
        execute_thread_.join();
    }
    execute_thread_ = std::jthread([this, goal_handle]() { execute(goal_handle); });
}


void KinovaMoveitBridge::execute(const std::shared_ptr<GoalHandle> goal_handle)
{
    auto goal = goal_handle->get_goal();
    auto result = std::make_shared<FollowJointTrajectory::Result>();

    if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);
        return;
    }

    std::vector<kinova_wrapper::TrajectoryPoint> traj = convertTrajectory(goal->trajectory);

    RCLCPP_INFO(get_logger(), " Received trajectory with %zu waypoints", traj.size());

    auto feedback = std::make_shared<FollowJointTrajectory::Feedback>();

    auto fb_callback = [&](const std::vector<double>& joints, double progress) {
    feedback->joint_names = goal->trajectory.joint_names;
    feedback->actual.positions = joints;

    size_t idx = static_cast<size_t>(progress * traj.size()) - 1;
    if (idx < traj.size()) {
        feedback->desired.positions = traj[idx].joint_angles;
        feedback->desired.time_from_start.sec = static_cast<int32_t>(traj[idx].time_from_start);
        feedback->desired.time_from_start.nanosec = static_cast<uint32_t>(
            (traj[idx].time_from_start - static_cast<int32_t>(traj[idx].time_from_start)) * 1e9);
    }

    goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(get_logger(), "Progress: %.0f%%", progress * 100.0);
    };

    bool ok = kinova_->executeTrajectory(traj, fb_callback);
    
    if(ok){
        goal_handle->succeed(result);
        RCLCPP_INFO(get_logger(), "Trajectory execution succeeded");
    }
    else{
        goal_handle->abort(result);
        RCLCPP_WARN(get_logger(), "Trajectory execution failed");
    }
    
    }

std::vector<kinova_wrapper::TrajectoryPoint> KinovaMoveitBridge::convertTrajectory(const JointTrajectory& traj)
            {
            
                std::vector<kinova_wrapper::TrajectoryPoint> result;
                for (size_t i = 0; i < traj.points.size(); i++) {
                    kinova_wrapper::TrajectoryPoint pt;
                    pt.joint_angles = traj.points[i].positions;
                    pt.time_from_start = traj.points[i].time_from_start.sec +
                                        traj.points[i].time_from_start.nanosec * 1e-9;
                    result.push_back(pt);
                }
                return result;

            }

void KinovaMoveitBridge::publishJointStates() {
    if (!kinova_->isConnected()) return;

    auto angles = kinova_->getJointAngles();
    if (angles.empty()) return;

    sensor_msgs::msg::JointState msg;
    msg.header.stamp = this->now();
    msg.name = {"joint_1", "joint_2", "joint_3", "joint_4",
            "joint_5", "joint_6", "joint_7",
            "robotiq_85_left_knuckle_joint"};  // add this

    // get gripper position from wrapper
    auto gripper_pos = kinova_->getGripperPosition() * 0.8;  // denormalize [0,1] → [0, 0.8 rad]

    msg.position = angles;
    msg.position.push_back(gripper_pos);  // append gripper
    
    joint_state_pub_->publish(msg);
    }

void KinovaMoveitBridge::handleClearFaults(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    (void)request;
    bool ok = kinova_->clearEmergencyStop();
    response->success = ok;
    response->message = ok ? "Faults cleared, ready to operate"
                           : "Failed to clear faults, e-stop still active";
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
}
        

                             




