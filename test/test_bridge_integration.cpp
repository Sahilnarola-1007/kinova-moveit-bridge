/**
 * @file test_bridge_integration.cpp
 * @brief Integration tests for the KinovaMoveitBridge and KinovaGripperBridge action servers.
 *
 * These tests verify the full MoveIt → Bridge → Wrapper pipeline by spinning up
 * both bridge nodes (trajectory + gripper) with a mock KinovaInterface, then sending
 * action goals through ROS 2 action clients — the same path MoveIt uses in production.
 *
 * Test coverage:
 *   - Valid trajectory goal accepted and executed successfully
 *   - Empty trajectory points rejected at goal acceptance
 *   - Missing joint names rejected at goal acceptance
 *   - E-stop active causes goal rejection
 *   - Gripper open (position=0.0) accepted and reaches target
 *   - Gripper close (position=0.8) accepted and reaches target
 *   - Cancel request propagates to emergencyStop() on the wrapper
 *
 * Build: ament_add_gtest (see CMakeLists.txt)
 * Run:   ./build/kinova_moveit_bridge/test_bridge_integration
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <kinova_moveit_bridge/kinova_moveit_bridge_node.hpp>
#include <kinova_moveit_bridge/kinova_gripper_bridge.hpp>
#include <kinova_wrapper/KinovaInterface.hpp>
#include <gtest/gtest.h>
#include <thread>

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GripperCommand = control_msgs::action::GripperCommand;

/**
 * @class BridgeIntegrationTest
 * @brief Test fixture that sets up a complete ROS 2 action server/client environment.
 *
 * Architecture per test:
 *   - KinovaInterface (mock SDK) shared between both bridge nodes
 *   - KinovaMoveitBridge node hosts FollowJointTrajectory action server
 *   - KinovaGripperBridge node hosts GripperCommand action server
 *   - A lightweight client node hosts both action clients
 *   - MultiThreadedExecutor spins all three nodes in a background thread
 *
 * SetUp() creates fresh nodes and clients before each test.
 * TearDown() cancels the executor and joins the spin thread.
 * rclcpp::init/shutdown run once per test suite (not per test).
 */
class BridgeIntergrationTest : public ::testing::Test {
protected:
    // Wrapper instance — shared between trajectory and gripper bridges (same as production)
    std::shared_ptr<kinova_wrapper::KinovaInterface> interface_;

    // Bridge nodes — action servers for trajectory execution and gripper control
    std::shared_ptr<KinovaMoveitBridge> bridge_node;
    std::shared_ptr<KinovaGripperBridge> gripper_node;

    // Lightweight node that hosts both action clients (cannot share node with servers)
    std::shared_ptr<rclcpp::Node> action_client_node;

    // Action clients — simulate MoveIt sending goals to the bridge
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_;
    rclcpp_action::Client<GripperCommand>::SharedPtr gripper_client_;

    // Executor and its background thread — processes callbacks for all three nodes
    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
    std::thread spin_thread_;

    /// @brief Initialize ROS 2 context once for the entire test suite.
    static void SetUpTestSuite() {
        rclcpp::init(0, nullptr);
    }

    /// @brief Shut down ROS 2 context after all tests complete.
    static void TearDownTestSuite() {
        rclcpp::shutdown();
    }

    /**
     * @brief Create nodes, action clients, and spin the executor before each test.
     *
     * Both bridge nodes share the same KinovaInterface instance (mock SDK),
     * matching the production architecture in main.cpp.
     */
    void SetUp() override {
        // Create shared wrapper — both bridges use the same connection (mock)
        interface_ = std::make_shared<kinova_wrapper::KinovaInterface>();

        // Create bridge nodes with shared wrapper
        bridge_node = std::make_shared<KinovaMoveitBridge>(rclcpp::NodeOptions{}, interface_);
        gripper_node = std::make_shared<KinovaGripperBridge>(rclcpp::NodeOptions{}, interface_);

        // Client node — hosts action clients, kept separate to avoid executor deadlock
        action_client_node = std::make_shared<rclcpp::Node>("test_client_node");

        // Create action clients on the client node — topic names must match server topics
        action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
            action_client_node,
            "/kinova_moveit_bridge/follow_joint_trajectory");

        gripper_client_ = rclcpp_action::create_client<GripperCommand>(
            action_client_node,
            "/robotiq_gripper_controller/gripper_cmd");

        // Spin all three nodes in a background thread so test code can send goals
        executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
        executor_->add_node(bridge_node);
        executor_->add_node(action_client_node);
        executor_->add_node(gripper_node);
        spin_thread_ = std::thread([this]() { executor_->spin(); });

        // Block until both action servers are discoverable
        ASSERT_TRUE(action_client_->wait_for_action_server(std::chrono::seconds(5)));
        ASSERT_TRUE(gripper_client_->wait_for_action_server(std::chrono::seconds(5)));
    }

    /**
     * @brief Stop the executor and join the spin thread after each test.
     */
    void TearDown() override {
        executor_->cancel();
        spin_thread_.join();
    }
};

// ======================== Trajectory Tests ========================

/// @test Send a valid 3-waypoint trajectory — expect acceptance and successful execution.
TEST_F(BridgeIntergrationTest, ValidGoalSucceeds) {
    FollowJointTrajectory::Goal goal;
    trajectory_msgs::msg::JointTrajectoryPoint point;

    goal.trajectory.joint_names = {"joint_1", "joint_2", "joint_3", "joint_4",
                                   "joint_5", "joint_6", "joint_7"};

    for (int i = 0; i < 3; i++) {
        point.positions = {0.0 + i, 0.1, 0.0, -0.5, 0.0, -0.3, 0.0};
        point.time_from_start.sec = 1 + i;
        point.time_from_start.nanosec = 0;
        goal.trajectory.points.push_back(point);
    }

    auto goal_handle_future = action_client_->async_send_goal(goal);
    auto goal_handle = goal_handle_future.get();
    ASSERT_NE(goal_handle, nullptr);

    auto result_future = action_client_->async_get_result(goal_handle);
    auto result = result_future.get();
    EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
}

/// @test Send a goal with joint names but zero trajectory points — expect rejection.
TEST_F(BridgeIntergrationTest, EmptyPointsRejected) {
    FollowJointTrajectory::Goal goal;
    goal.trajectory.joint_names = {"joint_1", "joint_2", "joint_3", "joint_4",
                                   "joint_5", "joint_6", "joint_7"};
    // No points added — bridge should reject in handle_goal()

    auto goal_handle_future = action_client_->async_send_goal(goal);
    auto goal_handle = goal_handle_future.get();
    ASSERT_EQ(goal_handle, nullptr);
}

/// @test Send a goal with trajectory points but no joint names — expect rejection.
TEST_F(BridgeIntergrationTest, EmptyJointNamesRejected) {
    FollowJointTrajectory::Goal goal;
    trajectory_msgs::msg::JointTrajectoryPoint point;

    for (int i = 0; i < 3; i++) {
        point.positions = {0.0 + i, 0.1, 0.0, -0.5, 0.0, -0.3, 0.0};
        point.time_from_start.sec = 1 + i;
        point.time_from_start.nanosec = 0;
        goal.trajectory.points.push_back(point);
    }
    // No joint names — bridge should reject in handle_goal()

    auto goal_handle_future = action_client_->async_send_goal(goal);
    auto goal_handle = goal_handle_future.get();
    ASSERT_EQ(goal_handle, nullptr);
}

/// @test Trigger e-stop before sending a valid goal — expect rejection.
TEST_F(BridgeIntergrationTest, EStopRejectsGoal) {
    FollowJointTrajectory::Goal goal;
    trajectory_msgs::msg::JointTrajectoryPoint point;

    goal.trajectory.joint_names = {"joint_1", "joint_2", "joint_3", "joint_4",
                                   "joint_5", "joint_6", "joint_7"};

    for (int i = 0; i < 3; i++) {
        point.positions = {0.0 + i, 0.1, 0.0, -0.5, 0.0, -0.3, 0.0};
        point.time_from_start.sec = 1 + i;
        point.time_from_start.nanosec = 0;
        goal.trajectory.points.push_back(point);
    }

    // Activate e-stop directly on the shared interface
    interface_->emergencyStop();

    auto goal_handle_future = action_client_->async_send_goal(goal);
    auto goal_handle = goal_handle_future.get();
    ASSERT_EQ(goal_handle, nullptr);
}

// ======================== Gripper Tests ========================

/// @test Command gripper to fully open position (0.0 rad) — expect success.
TEST_F(BridgeIntergrationTest, GripperOpenSucceeds) {
    GripperCommand::Goal goal;
    goal.command.position = 0.0;
    goal.command.max_effort = 0.0;

    auto goal_handle_future = gripper_client_->async_send_goal(goal);
    auto goal_handle = goal_handle_future.get();
    ASSERT_NE(goal_handle, nullptr);

    auto result_future = gripper_client_->async_get_result(goal_handle);
    auto result = result_future.get();
    EXPECT_TRUE(result.result->reached_goal);
}

/// @test Command gripper to closed position (0.8 rad) — expect success.
TEST_F(BridgeIntergrationTest, GripperCloseSucceeds) {
    GripperCommand::Goal goal;
    goal.command.position = 0.8;
    goal.command.max_effort = 0.0;

    auto goal_handle_future = gripper_client_->async_send_goal(goal);
    auto goal_handle = goal_handle_future.get();
    ASSERT_NE(goal_handle, nullptr);

    auto result_future = gripper_client_->async_get_result(goal_handle);
    auto result = result_future.get();
    EXPECT_TRUE(result.result->reached_goal);
}

// ======================== Cancel Propagation Test ========================

/// @test Verify that cancelling an in-flight trajectory triggers emergencyStop().
///       This is the chain: RViz cancel → action client cancel → handle_cancel() → emergencyStop().
TEST_F(BridgeIntergrationTest, CancelTriggersEStop) {
    FollowJointTrajectory::Goal goal;
    trajectory_msgs::msg::JointTrajectoryPoint point;

    goal.trajectory.joint_names = {"joint_1", "joint_2", "joint_3", "joint_4",
                                   "joint_5", "joint_6", "joint_7"};

    for (int i = 0; i < 3; i++) {
        point.positions = {0.0 + i, 0.1, 0.0, -0.5, 0.0, -0.3, 0.0};
        point.time_from_start.sec = 1 + i;
        point.time_from_start.nanosec = 0;
        goal.trajectory.points.push_back(point);
    }

    // Send goal and confirm acceptance
    auto goal_handle_future = action_client_->async_send_goal(goal);
    auto goal_handle = goal_handle_future.get();
    ASSERT_NE(goal_handle, nullptr);

    // Cancel immediately — simulates RViz cancel button
    action_client_->async_cancel_goal(goal_handle);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Verify emergencyStop() was called by handle_cancel()
    ASSERT_TRUE(interface_->isEStopActive());
}
