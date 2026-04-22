#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <kinova_wrapper/KinovaInterface.hpp>
#include <kinova_moveit_bridge/kinova_moveit_bridge_node.hpp>
#include <kinova_moveit_bridge/kinova_gripper_bridge.hpp>

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    
    auto interface_=std::make_shared<kinova_wrapper::KinovaInterface>();
    auto arm_bridge = std::make_shared<KinovaMoveitBridge>(rclcpp::NodeOptions{},interface_);
    auto gripper_bridge=std::make_shared<KinovaGripperBridge>(rclcpp::NodeOptions{},interface_);
    
    executor.add_node(arm_bridge);
    executor.add_node(gripper_bridge);
    executor.spin();   
    rclcpp::shutdown();
    return 0;
}       
