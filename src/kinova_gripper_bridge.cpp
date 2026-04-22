#include <control_msgs/action/gripper_command.hpp>
#include<rclcpp_action/rclcpp_action.hpp>
#include<kinova_moveit_bridge/kinova_gripper_bridge.hpp>
#include<thread>

using GripperCommand = control_msgs::action::GripperCommand;
using GripperGoalHandle = rclcpp_action::ServerGoalHandle<GripperCommand>;
using namespace std::placeholders;

KinovaGripperBridge::KinovaGripperBridge(const rclcpp::NodeOptions &options,
                                    std::shared_ptr<kinova_wrapper::KinovaInterface> kinova)
                                    :rclcpp::Node("robotiq_gripper_controller",options),
                                    kinova_(std::move(kinova))
                                    {

                            action_server_=rclcpp_action::create_server<GripperCommand>(
                            this,
                            "/robotiq_gripper_controller/gripper_cmd",
                            std::bind(&KinovaGripperBridge::handle_goal,this,_1,_2),
                            std::bind(&KinovaGripperBridge::handle_cancel,this,_1),
                            std::bind(&KinovaGripperBridge::handle_accepted,this,_1)
                        );
                                    }

rclcpp_action::GoalResponse KinovaGripperBridge::handle_goal(
            const rclcpp_action::GoalUUID& uuid,
            std::shared_ptr<const GripperCommand::Goal> goal){
             
                (void)uuid;
                (void)goal;

                if(!kinova_->isConnected() || kinova_->isEStopActive()){
                    return rclcpp_action::GoalResponse::REJECT;
                }
                
                 return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            }

rclcpp_action::CancelResponse KinovaGripperBridge::handle_cancel(
            const std::shared_ptr<GripperGoalHandle> goal_handle){
                
                (void)goal_handle;
                return rclcpp_action::CancelResponse::ACCEPT;
            }

void KinovaGripperBridge::handle_accepted(const std::shared_ptr<GripperGoalHandle> goal_handle){
    std::thread{std::bind(&KinovaGripperBridge::execute,this,goal_handle)}.detach();  
} 

void KinovaGripperBridge::execute(const std::shared_ptr<GripperGoalHandle> goal_handle)
{
    auto goal = goal_handle->get_goal();

    auto Tg_grip_pos=goal->command.position;
    if(Tg_grip_pos<0.0){ Tg_grip_pos=0.0;}
    if(Tg_grip_pos>kKnuckleMaxRad){Tg_grip_pos=kKnuckleMaxRad;}
    
    auto result = std::make_shared<GripperCommand::Result>();

    if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);
        return;
    }

    double normalized = Tg_grip_pos / kKnuckleMaxRad;  // normalise into [0,1]
    bool done=kinova_->setGripperPosition(normalized);
    if(done){
        result->position=Tg_grip_pos; // Fill the result
        result->reached_goal=true;   
        goal_handle->succeed(result);
        RCLCPP_INFO(get_logger(), "Gripper Position has set");
    
    }

    else{
        result->position=kinova_->getGripperPosition(); // Fill the result
        result->reached_goal=false;   
        goal_handle->abort(result);
    }

}